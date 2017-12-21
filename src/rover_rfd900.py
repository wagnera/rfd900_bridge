#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2
import struct
import serial
import binascii
import zlib
import time



class RFD900_Rover:
    def __init__(self):
        self.s=serial.Serial('/dev/ttyUSB0',57600)
        rospy.init_node('rfd_rover', anonymous=True)
        rospy.Subscriber("tf", TFMessage, self.tf_callback)

        rospy.init_node('rfd_rover', anonymous=True)
        rospy.Subscriber("/voxel_grid/output", PointCloud2, self.pc_callback)

        self.tf_fmt='c10s10s7f'
        self.pc_fmt='c10sII??II'
        self.eol = b'\r\n\r\n\r\n'
        self.tf_rate=5
        self.tf_timer=time.time()
        self.pc_rate=0.5
        self.pc_timer=time.time()

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
                
    def pc_callback(self,data):
        print("received pc msg")
        if time.time() < self.pc_timer:
            print("throttling pc")
            return
        H=data.header
        packet=struct.pack(self.pc_fmt,'p',H.frame_id,data.height,data.width,
            data.is_bigendian,data.is_dense,data.point_step,data.row_step)
        print(packet)
        pc_comp=zlib.compress(data.data,9)
        print(len(data.data),len(pc_comp))
        self.s.write(bytes(packet)+';'+bytes(pc_comp)+self.eol)
        print(bytes(packet)+';'+bytes(pc_comp)+self.eol)
        self.pc_timer=time.time()+(1/float(self.pc_rate))

    def spinner(self):
        rospy.spin()

if __name__ == '__main__':
    aa=RFD900_Rover()
    aa.spinner()