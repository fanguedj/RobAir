#!/usr/bin/env python
import roslib
roslib.load_manifest('robair_demo')
import rospy
import time
import serial
from robair_demo.msg import InfraredPotholes
from robair_demo.msg import UltrasoundObstacles

class ArduinoSensorsNode(object):
    '''
    This node publishes informations from sensors to two topics:
    Infrared holes detection information goes to /sensor/infrared_potholes
    Ultrasound obstacles detection goes to /sensor/ultrasound_obstacles
    '''
    INFRA_NB = 3
    ULTRA_FRONT_NB = 4
    ULTRA_REAR_NB = 0
    OBSTACLE_STOP_DIST = 20 # centimeters

    def __init__(self, serial_port, node_name="robair_arduino_sensors"):
        self.node_name = node_name
        rospy.init_node(self.node_name)
        self.infrared_pub = rospy.Publisher('/sensor/infrared_potholes',
                                            InfraredPotholes)
        self.ultrasound_pub = rospy.Publisher('/sensor/ultrasound_obstacles',
                                              UltrasoundObstacles)
        # Number of bauds must match the Arduino sketch value
        # in arduino_sketches/infrared_and_ultrasound.ino
        self.ser = serial.Serial(serial_port, 115200)

    def process_infrared_line(self, split_line):
        hole = False
        sensor_holes = []
        for sensor_value in split_line[1:1+self.INFRA_NB]:
            sensor_holes.append(sensor_value == "1")
            hole = hole or sensor_holes[-1]
        self.infrared_pub.publish(InfraredPotholes(hole, *sensor_holes))

    def process_ultrasound_line(self, split_line):
        front_obstacle = False
        obstacles = []
        for dist in split_line[1:1+self.ULTRA_FRONT_NB]:
            try:
                val = int(dist)
                obstacles.append(val)
                front_obstacle = front_obstacle or (
                    val <= self.OBSTACLE_STOP_DIST)
            except exceptions.ValueError:
                pass
        rear_obstacle = False
        for dist in split_line[1:1+self.ULTRA_REAR_NB]:
            try:
                val = int(dist)
                obstacles.append(val)
                rear_obstacle = rear_obstacle or (
                    val <= self.OBSTACLE_STOP_DIST)
            except exceptions.ValueError:
                pass
        self.ultrasound_pub.publish(
            UltrasoundObstacles(front_obstacle, rear_obstacle, *obstacles))

    def main_loop(self):
        while not rospy.is_shutdown():
            split_line = self.ser.readline().split();
            # TODO Add timeouts to readLine so that it does report inactivity if
            # it does not receive data
            if split_line and split_line[0] == "INFRA":
                self.process_infrared_line(split_line)
            elif split_line and split_line[0] == "ULTRA":
                self.process_ultrasound_line(split_line)

    def shutdown(self):
        self.ser.close()


