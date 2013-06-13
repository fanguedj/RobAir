#!/usr/bin/env python
import roslib

roslib.load_manifest('robair_demo')

import rospy

from robair_demo.msg import Command
from robair_demo import keylogger


class KeyboardNode(object):
    '''Robair keyboard control node
    This node handles the control of the robot by the keyboard
    Depending on the key pressed, it publishes a particular
    command to the node that subscribed to the topic /cmd'''
    def __init__(self, topic='/cmd', node_name="keyboard"):
        self.node_name = node_name
        self.pub = rospy.Publisher(topic, Command)
        rospy.init_node('keyboard')

    def main_loop(self):
        done = lambda: rospy.is_shutdown()
        keylogger.log(done, self.key_pressed)

    def key_pressed(self, dtime, modifiers, key):
        directions = {  "top": 0,
                        "bottom": 1,
                        "left": 2,
                        "right": 3,
                        "s": 4}
        if key in directions.keys():
            print("%s" % key)
            self.pub.publish(Command(directions[key],0.0,0.0))


if __name__ == '__main__':
    keyboard_node = KeyboardNode()
    rospy.loginfo("%s running..." % keyboard_node.node_name)
    keyboard_node.main_loop()
    rospy.loginfo("%s stopped." % keyboard_node.node_name)
