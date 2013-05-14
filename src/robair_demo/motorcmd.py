#!/usr/bin/env python
import roslib
import time
import serial
roslib.load_manifest('robair_demo')
import rospy
from robair_demo.msg import Command
from robair_demo.msg import InfraredPotholes
from robair_demo.msg import UltrasoundObstacles

# TODO for this node: add odometry
#beyond this limit, the speed is maximum
REDUCE_SPEED_DISTANCE = 100;

# Available motor commands, from PDF document at
# http://www.robotshop.com/eu/content/PDF/md49-documentation.pdf
def enum(**enums):
    return type('Enum', (), enums)
Commands = enum(
    SYNC="\x00",
    GET_SPEED_1="\x21",
    GET_SPEED_2="\x22",
    GET_ENCODER_1="\x23",
    GET_ENCODER_2="\x24",
    GET_ENCODERS="\x25",
    GET_VOLTS="\x26",
    GET_CURRENT_1="\x27",
    GET_CURRENT_2="\x28",
    GET_VERSION="\x29",
    GET_ACCELERATION="\x2A",
    GET_MODE="\x2B",
    GET_VI="\x2C",
    GET_ERROR="\x2D",
    SET_SPEED_1_OR_BOTH="\x31",
    SET_SPEED_2_OR_TURN="\x32",
    SET_ACCELERATION="\x33",
    SET_MODE="\x34",
    RESET_ENCODERS="\x35",
    DISABLE_REGULATOR="\x36",
    ENABLE_REGULATOR="\x37",
    DISABLE_TIMEOUT="\x38",
    ENABLE_TIMEOUT="\x39");
Modes = enum(
    UNSIGNED_SEPARATE_SPEEDS="\x00",
    SIGNED_SEPARATE_SPEEDS="\x01",
    UNSIGNED_SPEED_TURN="\x02",
    SIGNED_SPEED_TURN="\x03")
# Speeds for use with UNSIGNED modes
Speeds = enum(
    FORWARD_MEDIUM="\x64",
    NONE="\x80",
    TURNING="\x78",
    BACKWARD_MEDIUM="\x9b");
Turns = enum(
    LEFT = "\x87",
    RIGHT = "\x79");
    

class MotionControlNode(object):
    '''
    Robair motion control node
    This node subscribes to the topic /cmd and forwards
    the commands that are sent
    '''
    def __init__(self, serial_port, node_name="robair_motion_control"):
        self.node_name = node_name
        rospy.init_node(self.node_name)
        rospy.Subscriber('/cmd', Command, self.new_cmd_callback)
        rospy.Subscriber('/sensor/infrared_potholes', InfraredPotholes, self.new_potholes_callback)
        rospy.Subscriber('/sensor/ultrasound_obstacles', UltrasoundObstacles, self.new_obstacles_callback)
        self.ser = serial.Serial(serial_port, 38400)
        self.current_cmd = Command(5); # Invalid command
        self.potholes = InfraredPotholes(hole=True);

    def new_cmd_callback(self, cmd):
        self.current_cmd = cmd
        self.move()

    def new_potholes_callback(self, potholes):
        self.potholes = potholes
        self.move()
        
    def new_obstacles_callback(self, obstacles):
        self.obstacles = obstacles
        print obstacles
        self.move()

    def send_bytes(self, *bytes_to_send):
        '''Sends the given bytes, preceded by a synchronisation.'''
        self.ser.write(Commands.SYNC);
        for b in bytes_to_send:
            self.ser.write(b)

    def set_mode(self, mode):
        '''set the mode of the MD49'''
        self.send_bytes(Commands.SET_MODE, mode)

    def stop_wheels(self):
        '''stop both motors'''
        self.set_mode(Modes.UNSIGNED_SPEED_TURN)
        self.send_bytes(Commands.SET_SPEED_1_OR_BOTH, Speeds.NONE)
        # TODO Test whether the last 3 lines are useful
        self.set_mode(Modes.UNSIGNED_SEPARATE_SPEEDS)
        self.send_bytes(Commands.SET_SPEED_1_OR_BOTH, Speeds.NONE)
        self.send_bytes(Commands.SET_SPEED_2_OR_TURN, Speeds.NONE)
        
    def send_order(self, order):
        '''send orders through serial port'''
        self.stop_wheels() # Cancel previous commands
        self.set_mode(Modes.UNSIGNED_SPEED_TURN)
        if order == 0: # move forward
            print "forward"
            self.send_bytes(Commands.SET_SPEED_1_OR_BOTH, self.computeSpeed())
        elif order == 1: # move backward
            print "backward"
            self.send_bytes(Commands.SET_SPEED_1_OR_BOTH, Speeds.BACKWARD_MEDIUM)
        elif order == 2: # turn left
            print "left"
            self.send_bytes(Commands.SET_SPEED_1_OR_BOTH, Speeds.TURNING)
            self.send_bytes(Commands.SET_SPEED_2_OR_TURN, Turns.LEFT)
        elif order == 3: # turn right
            print "right"
            self.send_bytes(Commands.SET_SPEED_1_OR_BOTH, Speeds.TURNING)
            self.send_bytes(Commands.SET_SPEED_2_OR_TURN, Turns.RIGHT)
        elif order == 4: # stop wheels
            print "stop"
            
    def send_order_backtrack(self, order):
        '''send backtrack orders(reverse order) through serial port'''
    	if order % 2: 
    		newOrder = order + 1
    	else:
    		newOrder = order - 1
    	self.send_order(newOrder)

    def move(self):
        direction = self.current_cmd.move
        if direction < 5:
            if not self.potholes.hole and not self.obstacles.front_obstacle and not self.obstacles.rear_obstacle:
                print "order normal"
                self.send_order(direction)
            else:
                print "reverse order"
                self.stop_wheels()
            #    self.send_order_backtrack(direction)
        else:
            print "order incorrect"
            self.stop_wheels()

    def computeSpeed(self):
        #mini research
        mini = self.obstacles.north_west
        if self.obstacles.north_east < mini :
            mini = self.obstacles.north_east
        if self.obstacles.north_left < mini :
            mini = self.obstacles.north_left
        if self.obstacles.north_right < mini :
            mini = self.obstacles.north_right

        print "mini=", mini
        #speed reduction field
        if mini < REDUCE_SPEED_DISTANCE:
            print "reduction vitesse !"
            #compute to have a value between 0..15
            distModif = ( mini * 28 ) / 100
            #reverse value
            distModif = 28 - distModif
            #add normal speed value
            distModif += 100
            #merge strings
          #  chaine = r"\x" + str(distModif)
           # print chaine
            return unichr(distModif)
        else:
            return Speeds.FORWARD_MEDIUM


    def main_loop(self):
        # We chose to leave the timeout enabled (i.e. the robot will stop if it
        # does not receive orders every two seconds), and to repeat the current
        # order every second. This way, we ensure that the robot works
        # continuously while the system is running, and that it stops whenerver
        # there is a problem.
        self.send_bytes(Commands.ENABLE_TIMEOUT)
        while not rospy.is_shutdown():
            self.move()
            # TODO use rospy.rate
            time.sleep(1)

    def shutdown(self):
        self.ser.close()
