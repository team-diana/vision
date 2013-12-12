#!/usr/bin/env python
import rospy
import sys, getopt
import datetime
import time
import dynamic_reconfigure.client
from Queue import *
from std_msgs.msg import String
from sensor_msgs.msg import Joy, JointState
from pgr_camera.msg import published
from pgr_camera.srv import oneshot
from pgr_camera.srv import boolean 
from camera_around.srv import SequenceParameters
from optparse import OptionParser
from numpy import *

class Pos():
   def __init__(self, x, y):
      self.x = x
      self.y = y
   def __str__(self):
	return "(%.2f, %.2f)" % (self.x, self.y)

class CameraAround():
    def __init__(self, pantilt_topic, shotNum):
        self.pantiltPub = rospy.Publisher(pantilt_topic+'/cmd', JointState)
	self.positionQueue = Queue()
	self.pantilt_root = pantilt_topic
        rospy.init_node('camera_around_controller')
	self.shotNum = shotNum
	self.camera0root = "/camera0"
	self.camera1root = "/camera1"
	self.pantiltSpeed = 0.2
        self.startSequenceService = rospy.Service('/camera_around/start_sequence', SequenceParameters, self.handleStartSequence)
	#camere 	
	#self.setCameraFramerate(10)
	self.cameraEnable(False)
	rospy.Subscriber("/camera0/image_published", published, self.onCamera0Published, queue_size=shotNum*2)
	rospy.Subscriber("/camera1/image_published", published, self.onCamera1Published, queue_size=shotNum*2)
	self.shotNum = shotNum
	self.camera0CompletedShot = False
	self.camera1CompletedShot = False
	self.pantiltLimits = self.getPantiltLimits(pantilt_topic)

    def setCameraFramerate(self, framerate):
	camera0 = dynamic_reconfigure.client.Client(self.camera0root)
	camera1 = dynamic_reconfigure.client.Client(self.camera1root)
	param = { "frame_rate" : framerate }
	camera0.update_configuration(param)
	camera1.update_configuration(param)
	
 
    def cameraEnable(self, enable):
        camera0Enable = '/camera0/enable_stream'
        camera1Enable = '/camera1/enable_stream'
	rospy.wait_for_service(camera0Enable)
	rospy.wait_for_service(camera1Enable)
	try:
	    camera0OneShot = rospy.ServiceProxy(camera0Enable, boolean)
	    camera1OneShot = rospy.ServiceProxy(camera1Enable, boolean)
            camera0OneShot(enable)
            camera1OneShot(enable)
	    return
	except rospy.ServiceException, e:
	    print "Service call failed: %s"%e

    def cameraShot(self, shotNum):
        camera0EnableOneshotName = '/camera0/enable_oneshot'
        camera1EnableOneshotName = '/camera1/enable_oneshot'
	camera0CompletedShot = False
	camera1CompletedShot = False
	rospy.wait_for_service(camera0EnableOneshotName)
	rospy.wait_for_service(camera1EnableOneshotName)
	try:
	    camera0OneShot = rospy.ServiceProxy(camera0EnableOneshotName, oneshot)
	    camera1OneShot = rospy.ServiceProxy(camera1EnableOneshotName, oneshot)
            camera0OneShot(True, shotNum)
            camera1OneShot(True, shotNum)
	    return
	except rospy.ServiceException, e:
	    print "Service call failed: %s"%e

    def checkPantiltValues(self, x, y , vx, vy):
	self.pantiltLimits = self.getPantiltLimits(self.pantilt_root)
	isIn = lambda a, p : self.pantiltLimits['min'+p] <= a <= self.pantiltLimits['max'+p]
	if not isIn(x, 'x'): raise Exception("Invalid x")
	if not isIn(y, 'y'): raise Exception("Invalid y: " + str(y) + '\n' + str(self.pantiltLimits))
	if not isIn(vx, 'vx'): raise Exception("Invalid x speed")
	if not isIn(vy, 'vy'): raise Exception("Invalid y speed")
  	return True	

    def publishToPantilt(self, x, y, vx, vy):
	self.checkPantiltValues(x, y, vx, vy)

        state = JointState()
	state.name.append('pan')
	state.position.append(x)
	state.velocity.append(vx)
	state.name.append('tilt')
	state.position.append(y)
	state.velocity.append(vy)
        self.pantiltPub.publish(state)

    def checkRequest(self, req):
	try:
		maxDiff = 0.00001
		dx = abs(req.pan_max - req.pan_min)
		dy = abs(req.tilt_max - req.tilt_min)
		nx = dx / req.pan_step
		ny = dy / req.tilt_step
	        modx = dx % req.pan_step 
		mody = dy % req.tilt_step
		if (modx > maxDiff and (abs( dx - nx * req.pan_step) > maxDiff)):
		  print "Wrong pan step. dx % req.pan_ste = " + str(modx)
		  return False
		if (mody > maxDiff and (abs( dy - ny * req.tilt_step) > maxDiff)):
		  print "Wrong tilt step. dy % req.tilt_step = " + str(mody)
		  return False
		return True
	except Exception as e:
		print("checkRequest: " + e)
		return False

    def getPantiltLimits(self, pantiltTopic):
	limits = {}
	try:
		limits['minx'] = rospy.get_param(pantiltTopic+'/min_pan')
		limits['maxx'] = rospy.get_param(pantiltTopic+'/max_pan')
		limits['miny'] = rospy.get_param(pantiltTopic+'/min_tilt')
		limits['maxy'] = rospy.get_param(pantiltTopic+'/max_tilt')
		limits['minvx'] = rospy.get_param(pantiltTopic+'/min_pan_speed')
		limits['maxvx'] = rospy.get_param(pantiltTopic+'/max_pan_speed')
		limits['minvy'] = rospy.get_param(pantiltTopic+'/min_tilt_speed')
		limits['maxvy'] = rospy.get_param(pantiltTopic+'/max_tilt_speed')
	except:
		print "\nERROR: can't retrieve pantilt limits \n"
		print "Is the pantilt running?"
	return limits
	
    def handleStartSequence(self, req):
	if not self.checkRequest(req):
	   print "Wrong parameters"
	   return False
	dx = abs(req.pan_max - req.pan_min)
	dy = abs(req.tilt_max - req.tilt_min)
        nx = math.floor(dx / req.pan_step) + 1
        ny = math.floor(dy / req.tilt_step) + 1
	print "nx {0}  ny {1} ".format (nx , ny)
	xlist = linspace(req.pan_min, req.pan_max, nx)
	ylist = linspace(req.tilt_min, req.tilt_max, ny)
	posList = []
	for yi in ylist:
	  for xi in xlist:
             pos = Pos(xi, yi)
	     posList.append(pos) 
	  xlist = xlist[::-1]
	
	print "Positions:"
	for p in posList:
	  self.positionQueue.put(p)
	  print p
	
	print "\nWaiting for pantilt to reach initial position\n"
	self.publishToPantilt(posList[0].x, posList[0].y, self.pantiltSpeed, self.pantiltSpeed)
	time.sleep(8)
	print "Starting sequence\n"
	self.cameraEnable(True)
	self.cameraShot(self.shotNum)
	return True
	
        
    def nextCycle(self):	
	if self.positionQueue.empty():
	  print "\nSequence ended"
	  rospy.signal_shutdown("Sequence ended")
        nextPosition = self.positionQueue.get() 
	self.camera0CompletedShot = False
	self.camera1CompletedShot = False
	print "next position is " + str(nextPosition)
	self.publishToPantilt(nextPosition.x, nextPosition.y, self.pantiltSpeed, self.pantiltSpeed)
	time.sleep(8)
	self.cameraShot(self.shotNum)

    def onCamera0Published(self, publishedInfo):
	if(publishedInfo.oneshot_count >= self.shotNum):
	  self.camera0CompletedShot = True  
	  self.startNextIfPossible()

    def onCamera1Published(self, publishedInfo):
	if(publishedInfo.oneshot_count >= self.shotNum):
	  self.camera1CompletedShot = True  
          self.startNextIfPossible()	

    def startNextIfPossible(self):
	if self.camera0CompletedShot and self.camera1CompletedShot:
	   self.nextCycle()
	

if __name__ == '__main__':
    try:
        cameraAround = CameraAround('/ptu', shotNum=5)
	print "Waiting for service call..."
        rospy.spin()
        
    except e as rospy.ROSInterruptException:
	print e
