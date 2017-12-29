#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import NavSatFix
import tf2_ros
import struct
import serial
import time
import thread
import zlib
import difflib

class RFD900_Rover:
    def __init__(self):
        port=rospy.get_param("rfd900_bridge_rover/rfd900_port")
        self.s=serial.Serial(port,57600)
        rospy.init_node('rfd_rover', anonymous=True)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.cv_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.local_CM_callback)
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.global_CM_callback)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.gps_callback)
        self.tf_rate=4
        self.tf_timer=time.time()
        self.gps_rate=1
        self.gps_timer=time.time()
        self.lcm_rate=0.2
        self.lcm_timer=time.time()
        self.gcm_rate=1.0/60.0
        self.gcm_timer=time.time()
        self.serial_buffer=""
        self.data=""
        self.write_buffer=list()
        self.read_buffer=list()

    def read_msg(self):
        current_read=''
        current_read=self.s.read(self.s.inWaiting())
        self.serial_buffer=self.serial_buffer + current_read

        while self.serial_buffer.find('\x04\x17\xfe') > 0:
            data,self.serial_buffer=self.serial_buffer.split('\x04\x17\xfe',1)
            self.read_buffer.append(data) 
      

    def get_tf(self):
        MSGS=list()
        try:
            MSGS.append(self.tfBuffer.lookup_transform('map', 'odom', rospy.Time()))
        except:
            pass
        try:
            MSGS.append(self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time()))
        except:
            pass
        if len(MSGS) == 0:
            return
        tf_fmt='c10s10s7f'
        for MSG in MSGS:
            H=MSG.header
            Ro=MSG.transform.rotation
            Tr=MSG.transform.translation
            packet=struct.pack(tf_fmt,'t',H.frame_id,MSG.child_frame_id,Tr.x,Tr.y,Tr.z,Ro.x,Ro.y,Ro.z,Ro.w)
            self.s.write(bytes(packet)+'\x04\x17\xfe')
        self.tf_timer=time.time()+1/float(self.tf_rate)

    def send_costmap(self,data,Type):
        cm_fmt='c10sfIIff'
        bytess = struct.pack("{}h".format(len(data.data)), *data.data)
        compressed_data=zlib.compress(bytess,9)
        packet=struct.pack(cm_fmt,Type,data.header.frame_id,data.info.resolution,data.info.width,data.info.height,data.info.origin.position.x,data.info.origin.position.y)
        self.s.write(bytes(packet)+bytes(compressed_data)+'\x04\x17\xfe')
        rospy.loginfo("Sent Cost Map of size %i",len(bytes(packet)+bytes(compressed_data)+'\x04\x17\xfe'))
        if Type == 'a':
            self.gcm_timer=time.time()+1/float(self.gcm_rate)
        if Type == 'b':
            self.lcm_timer=time.time()+1/float(self.lcm_rate)

    def send_gps(self):
        gps_fmt='c3f'
        data=self.gps_msg
        packet=struct.pack(gps_fmt,'f',data.latitude,data.longitude,data.altitude)
        self.s.write(bytes(packet)+'\x04\x17\xfe')
        rospy.loginfo("Sent GPS msg")
        self.gps_timer=time.time()+1/float(self.gps_rate)

    def local_CM_callback(self,data):
        self.local_CM=data
        #self.send_costmap(data,'b')

    def global_CM_callback(self,data):
        self.global_CM=data
        #self.send_costmap(data,'a')

    def gps_callback(self,data):
        self.gps_msg=data

    def publish_cmd_vel(self,data):
        cv_fmt='c2f'
        read_msg=struct.unpack(cv_fmt,data)
        T=Twist()
        T.linear.x=read_msg[1]
        T.angular.z=read_msg[2]
        self.cv_pub.publish(T)
        rospy.loginfo("Publish Twist Msg")

    def read_msg_spinner(self):
        while not rospy.is_shutdown():

            if self.s.inWaiting() > 0:
                self.read_msg()
            if self.tf_timer < time.time():
                self.get_tf()
            if self.gps_timer < time.time():
                try: self.send_gps()
                except AttributeError:
                    pass
            self.process_msgs()
            if self.gcm_timer < time.time():
                try: self.send_costmap(self.global_CM,'a')
                except AttributeError:
                    pass
            if self.lcm_timer < time.time():
                try: self.send_costmap(self.local_CM,'b')
                except AttributeError:
                    pass


    def process_msgs(self):
        if len(self.write_buffer) > 0:
           pass    

        if len(self.read_buffer) > 0:
            msg=self.read_buffer.pop(0)
            msg_type=msg[0]
            try:
                if msg_type == 'c':
                        self.publish_cmd_vel(msg)
            except:
                rospy.logwarn("Failed to Send Twist Msg")


    def spinner(self):
        thread.start_new_thread(self.read_msg_spinner())
        rospy.spin()

if __name__ == '__main__':
    aa=RFD900_Rover()
    aa.spinner()