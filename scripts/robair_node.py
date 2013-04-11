#!/usr/bin/env python
import roslib
import time
import serial

roslib.load_manifest('robair_demo')

import rospy

from robair_demo.msg import Command


class RobairNode(object):
    '''Robair robot node
    This node subscribes to the topic /cmd and listens
    the commands that are sent'''
    def __init__(self, serial_port, node_name="robair_motion_control"):
        self.node_name = node_name
        rospy.init_node(self.node_name)
        rospy.Subscriber('/cmd', Command, self.new_cmd_callback)
        self.current_cmd = Command(5)
        self.ser = serial.Serial(serial_port, 38400)

    def new_cmd_callback(self, cmd):
        self.current_cmd = cmd

    def set_mode(self, mode):
        '''set the mode of the MD49'''
        if mode == 0:
            self.ser.write("\x00")
            self.ser.write("\x34")
            self.ser.write("\x00")
        elif mode == 1:
            self.ser.write("\x00")
            self.ser.write("\x34")
            self.ser.write("\x01")
        elif mode == 2:
            self.ser.write("\x00")
            self.ser.write("\x34")
            self.ser.write("\x02")
        elif mode == 3:
            self.ser.write("\x00")
            self.ser.write("\x34")
            self.ser.write("\x03")

    def stop_wheels(self):
        '''stop both motors'''
        self.set_mode(2)
        self.ser.write("\x00")
        self.ser.write("\x31")
        self.ser.write("\x80")
        self.set_mode(0)
        self.ser.write("\x00")
        self.ser.write("\x31")
        self.ser.write("\x80")
        self.ser.write("\x00")
        self.ser.write("\x32")
        self.ser.write("\x80")
        
    def send_order(self, order):
        '''send orders through serial port'''
        if order == 0: # move forward
            print "forward"
            self.stop_wheels()
            self.set_mode(2)
            self.ser.write("\x00")
            self.ser.write("\x31")
            self.ser.write("\x64") # medium speed
            self.current_cmd.move = 5
        elif order == 1: # move backward
            print "backward"
            self.stop_wheels()
            self.set_mode(2)
            self.ser.write("\x00")
            self.ser.write("\x31")
            self.ser.write("\x9b")
            self.current_cmd.move = 5
        elif order == 2: # turn left
            print "left"
            self.stop_wheels()
            self.set_mode(2)
            self.ser.write("\x00")
            self.ser.write("\x31")
            self.ser.write("\x78")
            self.ser.write("\x00")
            self.ser.write("\x32")
            self.ser.write("\x87")
            self.current_cmd.move = 5
        elif order == 3: # turn right
            print "right"
            self.stop_wheels()
            self.set_mode(2)
            self.ser.write("\x00")
            self.ser.write("\x31")
            self.ser.write("\x78")
            self.ser.write("\x00")
            self.ser.write("\x32")
            self.ser.write("\x79")
            self.current_cmd.move = 5
        elif order == 4: # stop wheels
            print "stop"
            self.stop_wheels()
            self.current_cmd.move = 5

    def move(self):
        direction = self.current_cmd.move
        if direction < 5:
            self.send_order(direction)

    def main_loop(self):
        self.ser.write("\x00")
        self.ser.write("\x38") # disable timeout
        while not rospy.is_shutdown():
            self.move()
            time.sleep(1)

    def shutdown(self):
        self.ser.close()


if __name__ == '__main__':
    # in the following line, put the right port as argument
    robair_node = RobairNode('/dev/ttyUSB0')
    rospy.loginfo("%s running..." % robair_node.node_name)
    robair_node.main_loop()
    robair_node.shutdown()
