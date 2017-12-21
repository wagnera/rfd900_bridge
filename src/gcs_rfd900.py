#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import serial
import struct
import binascii
import time


class RFD900_GCS:
    def __init__(self):
        self.s=serial.Serial('/dev/ttyUSB1',57600)
        self.tf_pub = rospy.Publisher('tf_rfd', TFMessage, queue_size=10)
        self.pc_pub = rospy.Publisher('pc_rfd', PointCloud2, queue_size=10)
        rospy.init_node('rfd_GCS', anonymous=True)
        self.tf_fmt='c10s10s7f'
        self.pc_fmt='c10sII??II'
        self.eol = b'\r\n\r\n\r\n'

    def read_msg(self):
            
        try:
            print("reading")
            data=self._readline()
            print(len(data))
            self.msg_type=data[0]
            print("message type:",self.msg_type)
            
        except:
            print("failed to read")
            return
        print("got message")
        if self.msg_type == 't':
                self.data=data.strip()
                self.publish_tf()
        elif self.msg_type == 'p':
            print("enter publish pc")
            self.data=data.strip(self.eol)
            self.publish_PC()
        
    def publish_tf(self):
        #print(self.msg_type)
        read_msg=struct.unpack(self.tf_fmt,self.data)
        #print(read_msg)
        t = TransformStamped()
        t.header.frame_id = read_msg[1]
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = read_msg[2]
        t.transform.translation.x = read_msg[3]
        t.transform.translation.y = read_msg[4]
        t.transform.translation.z = read_msg[5]
        t.transform.rotation.x = read_msg[6]
        t.transform.rotation.y = read_msg[7]
        t.transform.rotation.z = read_msg[8]
        t.transform.rotation.w = read_msg[9]
        tfm = TFMessage([t])
        self.tf_pub.publish(tfm)
        rospy.loginfo("Published TF Message")

    def publish_PC(self):
        PC=PointCloud2()
        print("splitting")
        split_index=self.data.find(';')
        info=self.data[:split_index]
        print(info)
        pcdata=self.data[(split_index):]
        print(pcdata[0],len(pcdata))
        read_msg=struct.unpack(self.pc_fmt,info)
        print(read_msg)
        PC.header.frame_id=read_msg[1]
        PC.height=read_msg[2]
        PC.width=read_msg[3]
        PC.is_bigendian=read_msg[4]
        PC.is_dense=read_msg[5]
        PC.point_step=read_msg[6]
        PC.row_step=read_msg[7]
        Fields=[PointField(),PointField(),PointField(),PointField()]
        Fields[0].name='x'
        Fields[1].name='y'
        Fields[2].name='z'
        Fields[3].name='rgb'
        Fields[0].offset=0
        Fields[1].offset=4
        Fields[2].offset=8
        Fields[3].offset=16
        Fields[0].datatype=7
        Fields[1].datatype=7
        Fields[2].datatype=7
        Fields[3].datatype=7
        Fields[0].count=1
        Fields[1].count=1
        Fields[2].count=1
        Fields[3].count=1
        PC.fields=Fields
        PC.data=pcdata
        self.PointCloud=PC
        self.pc_pub.publish(PC)
        rospy.loginfo("Published PointCloud2 Message")

    def calc_checksum(self):
        pass

    def _readline(self):
        leneol = len(self.eol)
        line = bytearray()
        while True:
            c = self.s.read(1)
            if c:
                line += c
                if line[-leneol:] == self.eol:
                    break
            else:
                break
        return bytes(line)
                
aa=RFD900_GCS()
while not rospy.is_shutdown():
    #try:
        aa.read_msg()
    #except rospy.ROSInterruptException:
     #   pass
print(count)