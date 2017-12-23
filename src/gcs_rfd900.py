#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import serial
import struct
import time
import thread
import zlib


class RFD900_GCS:
    def __init__(self):
        port=rospy.get_param("rfd900_bridge_gcs/rfd900_port")
        self.s=serial.Serial(port,57600)
        rospy.init_node('rfd_GCS', anonymous=True)
        self.tf_pub = rospy.Publisher('tf_rfd', TFMessage, queue_size=10)
        self.Lcm_pub = rospy.Publisher('/rfd_bridge/local_costmap/costmap', OccupancyGrid, queue_size=10)
        self.Gcm_pub = rospy.Publisher('/rfd_bridge/global_costmap/costmap', OccupancyGrid, queue_size=10)
        rospy.Subscriber("cmd_vel", Twist, self.send_cmd_vel)
        self.serial_buffer=""
        self.data=""
        self.tf_fmt='c10s10s7f'
        self.cv_fmt='c2f'
        self.cv_rate=5
        self.cv_timer=time.time()

    def read_msg(self):
        current_read=''
        current_read=self.s.read(self.s.inWaiting())
        self.serial_buffer=self.serial_buffer + current_read

        while self.serial_buffer.find('\x04\x17\xfe') > 0:
            #print(self.serial_buffer.find('\x04\x17\xfe'),self.serial_buffer[self.serial_buffer.find('\x04\x17\xfe')-2:self.serial_buffer.find('\x04\x17\xfe')+2])
            self.data,self.serial_buffer=self.serial_buffer.split('\x04\x17\xfe',1)
            #print("new buffer: ",self.serial_buffer)
            #print("complet packet: ",self.data) 
            self.msg_type=self.data[0]
            #print("message type:",self.msg_type)

        try:
            if self.msg_type == 't':
                    #self.data=data.strip()
                    self.publish_tf()
        except AttributeError:
            pass

        try:
            if self.msg_type == 'b':
                    #self.data=data.strip()
                    self.publish_cm('b')
            if self.msg_type == 'a':
                    #self.data=data.strip()
                    self.publish_cm('a')
        except AttributeError:
            pass
        except:
            rospy.logwarn("CostMap Faile Somewhere")
            
            
    def publish_tf(self):
        #print(self.msg_type)
        read_msg=struct.unpack(self.tf_fmt,self.data)
        t = TransformStamped()
        t.header.frame_id = read_msg[1].strip(b'\x00')
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = read_msg[2].strip(b'\x00')
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

    def publish_cm(self,Type):
        CM=OccupancyGrid()
        cm_fmt='c10sfIIff'
        header=self.data[:struct.calcsize(cm_fmt)]
        read_header=struct.unpack(cm_fmt,header)
        CM.header.frame_id=read_header[1].strip('\x00')
        CM.info.resolution=read_header[2]
        CM.info.width=read_header[3]
        CM.info.height=read_header[4]
        CM.info.origin.position.x=read_header[5]
        CM.info.origin.position.y=read_header[6]
        uncompressed=zlib.decompress(self.data[struct.calcsize(cm_fmt):])
        data=struct.unpack(str(read_header[3]*read_header[4])+'h',uncompressed)
        CM.data=data
        if Type == 'b':
            self.Lcm_pub.publish(CM)
            rospy.loginfo("Published Local Cost Map")
        elif Type == 'a':
            rospy.loginfo
            self.Gcm_pub.publish(CM)
            rospy.loginfo("Published Global Cost Map")

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
                if self.s.inWaiting() > 0:
                    self.read_msg()

    def spinner(self):
        thread.start_new_thread(self.read_msg_spinner())
        rospy.spin()

    def calc_checksum(self):
        pass
                
aa=RFD900_GCS()
aa.spinner()