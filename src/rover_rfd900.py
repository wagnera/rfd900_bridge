#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
import struct
import serial
import time
import thread



class RFD900_Rover:
    def __init__(self):
        port=rospy.get_param("rfd900_bridge_gcs/rfd900_port")
        self.s=serial.Serial(port,57600)
        rospy.init_node('rfd_rover', anonymous=True)
        rospy.Subscriber("tf", TFMessage, self.tf_callback)
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
            #if self.write_timer < time.time():
            #    self.write('dasdasd')
            #    print(self.s.out_waiting)

    def spinner(self):
        thread.start_new_thread(self.read_msg_spinner())
        rospy.spin()

if __name__ == '__main__':
    aa=RFD900_Rover()
    aa.spinner()