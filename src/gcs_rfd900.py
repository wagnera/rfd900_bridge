#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import serial
import struct
import time
import thread


class RFD900_GCS:
    def __init__(self):
        port=rospy.get_param("rfd900_bridge_gcs/rfd900_port")
        self.s=serial.Serial(port,57600)
        rospy.init_node('rfd_GCS', anonymous=True)
        self.tf_pub = rospy.Publisher('tf_rfd', TFMessage, queue_size=10)
        rospy.Subscriber("cmd_vel", Twist, self.send_cmd_vel)
        self.tf_fmt='c10s10s7f'
        self.cv_fmt='c2f'
        self.cv_rate=5
        self.cv_timer=time.time()

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
        try:
            if self.msg_type == 't':
                    self.data=data.strip()
                    self.publish_tf()
        except:
            pass
            
    def publish_tf(self):
        #print(self.msg_type)
        read_msg=struct.unpack(self.tf_fmt,self.data)
        print(read_msg)
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

    def send_cmd_vel(self,data):
        if time.time() < self.cv_timer:
            return
        L=data.linear
        A=data.angular
        packet=struct.pack(self.cv_fmt,'c',L.x,A.z)
        self.s.write(bytes(packet)+'\n')
        self.cv_timer=time.time()+1/float(self.cv_rate)

    def read_msg_spinner(self):
        while 1:
            self.read_msg()

    def spinner(self):
        thread.start_new_thread(self.read_msg_spinner())
        rospy.spin()

    def calc_checksum(self):
        pass
                
aa=RFD900_GCS()
aa.spinner()