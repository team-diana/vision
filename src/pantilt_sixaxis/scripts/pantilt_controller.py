#!/usr/bin/env python
import rospy
import sys, getopt
import datetime
import time
from std_msgs.msg import String
from sensor_msgs.msg import Joy, JointState
from optparse import OptionParser

class _Getch:
    """Gets a single character from standard input.  Does not echo to the screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

sixaxisEnabled = False
keyboardEnabled = False




class SixaxisController():
    def __init__(self, pantilt_topic):
        self.pantiltPub = rospy.Publisher(pantilt_topic, JointState)
        rospy.init_node('pantilt_sixaxis_controller')
        rospy.Subscriber('joy', Joy, self.sixaxisCallback) 
        getch = _Getch()
        start = self.getNow()
        print "Press h or l to move left or right" 
        print "Press j or k to move down or up" 
        print "Press q to exit"
        while True:
            c = getch()
            if not (self.processChar(c)):
                break
        #rospy.spin()

    def processChar(self, c):
	v = 0.2
        if c == 'h':
           self.publishToPantilt(-v,0) 
        elif c == 'l':
           self.publishToPantilt(v,0) 
        elif c == 'j':
           self.publishToPantilt(0,-v) 
        elif c == 'k':
           self.publishToPantilt(0,v) 
        elif c == 'q':
            return False
	time.sleep(0.05)
	#reset
        self.stopPantilt()
        return True

    def sixaxisCallback(self, joy):
        print("sixaxis callback")
        xIndex=2
        yIndex=3
        x=joy.axes[xIndex]
        y=joy.axes[yIndex]
        # clamp values
        x /= 0.2
        y /= 0.2
        self.publishToPantilt(x, y)
        #rospy.loginfo("x: %f y: %f" % (x, y))

    def stopPantilt(self):
        state = JointState()
        state.name.append('pan')
        state.position.append(0.1)
        state.velocity.append(0)
        self.pantiltPub.publish(state)
        state.name.append('tilt')
        state.position.append(0.1)
        state.velocity.append(0)
	#print(state)
        self.pantiltPub.publish(state)
        
	

    def publishToPantilt(self, x, y):
        state = JointState()
	state.name.append('pan')
	state.position.append(self.dirX(x))
	state.velocity.append(x)
	state.name.append('tilt')
	state.position.append(self.dirY(y))
	state.velocity.append(y)
	#print(state)
        self.pantiltPub.publish(state)
        

    def dirX(self, f):
        if f == 0: 
            return 0
        else:
            return 1 if (f > 0) else -1
    def dirY(self, f):
        if f == 0: 
            return 0
        else:
            return 0.45 if (f > 0) else -0.45

    def getNow(self):
        return datetime.datetime.now()


if __name__ == '__main__':
    try:
        parser = OptionParser()
        parser.add_option("-p", "--pantilt_topic", dest="pantilt_topic",
                help="topic of the pantilt setter")
        parser.add_option("-k", "--keyboard-only", dest="keyboardOnly",
                help="enable only the keyboard", default=False)
        parser.add_option("-j", "--joystick-only", dest="joystickOnly",
                help="enable only the joystick", default=False)
        (options, args) = parser.parse_args()
        if options.pantilt_topic == '':
            print "invalid topic name. use -p"
            sys.exit(-1)
        print ("connecting to pantilt at " +  options.pantilt_topic)
        if(options.keyboardOnly):
            sixaxisEnabled=False
        elif(options.joystickOnly):
            keyboardEnabled=False
        sixaxisController = SixaxisController(options.pantilt_topic)
    except rospy.ROSInterruptException:
        pass
