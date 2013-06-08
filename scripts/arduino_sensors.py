#!/usr/bin/env python
import roslib
roslib.load_manifest('robair_demo')
import rospy

from robair_demo.arduino import ArduinoSensorsNode

if __name__ == '__main__':
    # in the following line, put the right port as argument
    robair_node = ArduinoSensorsNode('/dev/ttyACM0')
    rospy.loginfo("%s running..." % robair_node.node_name)
    robair_node.main_loop()
    robair_node.shutdown()
