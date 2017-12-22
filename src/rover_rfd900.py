#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
import tf2_ros
import struct
import serial
import time
import thread



class RFD900_Rover:
    def __init__(self):
        port=rospy.get_param("rfd900_bridge_gcs/rfd900_port")
        self.s=serial.Serial(port,57600)
        rospy.init_node('rfd_rover', anonymous=True)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.cv_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.cv_fmt='c2f'
        self.tf_fmt='c10s10s7f'
        self.tf_rate=20
        self.tf_timer=time.time()
        self.serial_buffer=""
        self.data=""

    def read_msg(self):
        current_read=''
        current_read=self.s.read(self.s.inWaiting())
        self.serial_buffer=self.serial_buffer + current_read

        while self.serial_buffer.find('\n') > 0:
            self.data,self.serial_buffer=self.serial_buffer.split('\n',1)
            print("complet packet: ",self.data) 
            self.msg_type=self.data[0]
            #print("message type:",self.msg_type)

        try:
            if self.msg_type == 'c':
                    #self.data=data.strip()
                    self.publish_cmd_vel()
        except:
            pass

    def get_tf(self):
        try:
            MSG = self.tfBuffer.lookup_transform('map', 'odom', rospy.Time())
        except:
            print("failed to get tf")
            return
        H=MSG.header
        Ro=MSG.transform.rotation
        Tr=MSG.transform.translation
        packet=struct.pack(self.tf_fmt,'t',H.frame_id,MSG.child_frame_id,Tr.x,Tr.y,Tr.z,Ro.x,Ro.y,Ro.z,Ro.w)
        self.s.write(bytes(packet)+'\n')
        self.tf_timer=time.time()+1/float(self.tf_rate)

    def publish_cmd_vel(self):
        read_msg=struct.unpack(self.cv_fmt,self.data)
        T=Twist()
        T.linear.x=read_msg[1]
        T.angular.z=read_msg[2]
        self.cv_pub.publish(T)
        rospy.loginfo("Publish Twist Msg")

    def read_msg_spinner(self):
        while 1:

            if self.s.inWaiting() > 0:
                self.read_msg()
            if self.tf_timer < time.time():
                self.get_tf()
                #print(self.s.out_waiting)

    def spinner(self):
        thread.start_new_thread(self.read_msg_spinner())
        rospy.spin()

if __name__ == '__main__':
    aa=RFD900_Rover()
    aa.spinner()