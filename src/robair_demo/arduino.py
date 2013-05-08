#!/usr/bin/env python
import roslib
roslib.load_manifest('robair_demo')
import rospy
import time
import serial
from robair_demo.msg import InfraredPotholes



class ArduinoSensorsNode(object):
    '''
    This node publishes informations from sensors to two topics:
    Infrared holes detection information goes to /sensor/infrared_potholes
    Ultrasound obstacles detection goes to /sensor/ultrasound_obstacles
    '''
    
    INFRA_NB = 3
    ULTRA_NB = 0

    def __init__(self, serial_port, node_name="robair_arduino_sensors"):
        self.node_name = node_name
        rospy.init_node(self.node_name)
        self.infrared_pub = rospy.Publisher('/sensor/infrared_potholes', std_msgs.msg.String)
        self.ser = serial.Serial(serial_port, 9600)

    def process_infrared_line(self, split_line):
        hole = False;
        for sensor_value in split_line[1:self.INFRA_NB]:
            sensor_hole = (sensor_value == "1")
            hole = hole or sensor_hole

        self.infra_pub

    def main_loop(self):
        while not rospy.is_shutdown():
            self.todo()
            self.infrared_pub.publish(InfraredPotholes(false, false, false))
            time.sleep(1)

    def shutdown(self):
        self.ser.close()


