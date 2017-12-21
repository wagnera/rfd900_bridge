#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import serial
import struct
import time


class RFD900_GCS:
    def __init__(self):
        self.s=serial.Serial('/dev/ttyUSB1',57600)
        self.tf_pub = rospy.Publisher('tf_rfd', TFMessage, queue_size=10)
        rospy.init_node('rfd_GCS', anonymous=True)
        self.tf_fmt='c10s10s7f'

    def read_msg(self):
            
        try:
            print("reading")
            data=self.s.readline()
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

    def calc_checksum(self):
        pass
                
aa=RFD900_GCS()
while not rospy.is_shutdown():
    #try:
        aa.read_msg()
    #except rospy.ROSInterruptException:
     #   pass
print(count)