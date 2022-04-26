#!/usr/bin/env python
import socket
import rospy
import time
from geometry_msgs.msg import  TransformStamped
from Socket_data import Data

Sock = Data()

HOST = '10.30.33.7'  # Symbolic name meaning all available interfaces
PORT = 25001          # Arbitrary non-privileged port
sling = 'bowg'
hand = 'headg'
head = 'head'

if __name__ == '__main__':

    rospy.init_node('socket_connection', anonymous=True) 
    s = socket.socket()      # socket object     
    print('socket created ')
    s.connect((HOST, PORT))
    print('socket connected')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown(): 
        rospy.Subscriber('/vicon/' + sling + '/' + sling, TransformStamped, Sock.sling_call_back)
        rospy.Subscriber('/vicon/' + hand + '/' + hand, TransformStamped, Sock.hand_call_back)
        rospy.Subscriber('/vicon/' + head + '/' + head, TransformStamped, Sock.head_call_back)
        data = ','.join(map(str, Sock.data))
        print(data)
        s.sendall(data.encode("UTF-8"))             #Converting string to Byte, and sending it to C#
        rate.sleep()








