#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import struct
import serial
import time



class RFD900_Rover:
    def __init__(self):
        self.s=serial.Serial('/dev/ttyUSB0',57600)
        rospy.init_node('rfd_rover', anonymous=True)
        rospy.Subscriber("tf", TFMessage, self.tf_callback)

        self.tf_fmt='c10s10s7f'
        self.tf_rate=5
        self.tf_timer=time.time()

    def tf_callback(self,data):
        if time.time() < self.tf_timer:
            return
        MSG=data.transforms[0]
        H=MSG.header
        Ro=MSG.transform.rotation
        Tr=MSG.transform.translation
        #print(self.tf_fmt,H.frame_id,data.transforms[0].child_frame_id,
           # Tr.x,Tr.y,Tr.z,Ro.x,Ro.y,Ro.z,Ro.w)
        packet=struct.pack(self.tf_fmt,'t',H.frame_id,MSG.child_frame_id,Tr.x,Tr.y,Tr.z,Ro.x,Ro.y,Ro.z,Ro.w)
        #print(packet)
        #self.s.write(binascii.hexlify(packet)+'\n')
        self.s.write(bytes(packet)+'\n')
        self.tf_timer=time.time()+1/float(self.tf_rate)

    def spinner(self):
        rospy.spin()

if __name__ == '__main__':
    aa=RFD900_Rover()
    aa.spinner()