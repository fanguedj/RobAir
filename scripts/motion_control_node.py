#!/usr/bin/env python
import roslib
roslib.load_manifest('robair_demo')
import rospy
from robair_demo.motorcmd import MotionControlNode

if __name__ == '__main__':
    # in the following line, put the right port as argument
    mc_node = MotionControlNode('/dev/ttyUSB0')
    rospy.loginfo("%s running..." % mc_node.node_name)
    mc_node.main_loop()
    mc_node.shutdown()
