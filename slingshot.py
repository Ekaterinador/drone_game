#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int8
import numpy as np 
from angrydrone import Angrydrone

Angry = Angrydrone('cf30')  
sling = 'left_hand'
hand = 'right_hand'

def start_pose_callback(msg): 
    global Angry
    pos = msg.transform.translation
    Angry.start_pos = [pos.x - 0.2, pos.y, pos.z + 0.1]
    print(Angry.start_pos)
    start_pos.unregister()

if __name__ == '__main__':

    rospy.init_node('slingshot', anonymous=True)
    Angry.rate = rospy.Rate(5)
    cf = Angry.cf_init()
    start_pos = rospy.Subscriber('/vicon/' + sling + '/' + sling, TransformStamped, start_pose_callback)
    cf.takeoff(targetHeight = 1.0, duration = 5.0)
    time.sleep(5.0)
    cf.goTo(goal = Angry.start_pos, yaw=0.0, duration = 5.0, relative = False)
    time.sleep(5.0)

    while not rospy.is_shutdown():

        rospy.Subscriber('/vicon/' + sling + '/' + sling, TransformStamped, Angry.sling_pose_callback)
        rospy.Subscriber('/vicon/' + hand + '/' + hand, TransformStamped, Angry.hand_pose_callback)
        rospy.Subscriber('/flag', Int8, Angry.flag_callback)

        rospy.spin()    



        
        

            

            




