#!/usr/bin/env python
import roslib; roslib.load_manifest('robair_demo')
import rospy
import serial
import array
from ctypes import *
#from std_msgs.msg import String
from robair_demo.msg import encoderData



def send_bytes(serie, *bytes_to_send):
    '''Sends the given bytes, preceded by a synchronisation.'''
    serie.write("\x00"); #SYNC
    for b in bytes_to_send:
        serie.write(b)

def talker():
    pub = rospy.Publisher('encoder', encoderData)
    rospy.init_node('encoder_publish')
    ser = serial.Serial('/dev/ttyUSB0', 38400)
    send_bytes(ser,"\x35") #RESET_ENCODERS
    while not rospy.is_shutdown():
        send_bytes(ser,"\x25") #GET_ENCODERS
        recu = ser.read(8)
        #print recu.encode("hex")
        recu = long(recu.encode("hex"),16)
        wheel1 = recu % 0x100000000#0xFFFFFFFF
        wheel2 = recu / 0x100000000 #0xFFFFFFFF
        wheel1 =  - c_int(wheel1).value #inversion empirique
        wheel2 = - c_int(wheel2).value
        print "roue gauche :" + str(wheel1)
        print "roue droite :" + str(wheel2)
        pub.publish(wheel2,wheel1)
        rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
