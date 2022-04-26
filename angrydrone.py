#!/usr/bin/env python
import numpy as np
import rospy
import crazyflie
from crazyflie_driver.msg import Position




class Angrydrone(): 
    def __init__(self, name):
        self.name = name
        # self.sling_distance = None
        self.pub = rospy.Publisher('/' + self.name + '/' + 'cmd_position', Position, queue_size=1)
        self.traj_x = None 
        self.traj_y = None 
        self.traj_z = None
        self.sling_pos = np.array([0.0, 0.0, 0.0])
        self.hand_pos = np.array([0.0, 0.0, 0.0])
        self.flag = 0
        self.yaw = 0.0
        self.start_pos = None
        self.counter = 0
        self.rate = None


    def cf_init(self):
        cf = crazyflie.Crazyflie(self.name, '/vicon/' + self.name + '/' + self.name)
        cf.setParam("commander/enHighLevel", 1)
        cf.setParam("stabilizer/estimator", 2)  # Use EKF
        cf.setParam("stabilizer/controller", 2)  # Use mellinger controller
        cf.setParam("kalman/resetEstimation", 1)  # reset kalman
        return cf


    def calculate_trajectory(self): 
        print('sling pose ', self.sling_pos)
        print('hand pose ', self.hand_pos)

        p = self.sling_pos - self.hand_pos  # vector in the direction of the projectile motion 
        k = 1.2      # hook's constant 
        m = 27*1e-3  # CF mass 
        g = 9.8      # m/s^2 
        N = 1000      # number of time steps taken ( 10 seconds )
        d = np.linalg.norm(p)
        alpha = np.arctan2(np.linalg.norm(np.cross(p,np.array([1.0, 0.0, 0.0]))), np.dot(p,np.array([1.0, 0.0, 0.0])))
        gama = np.arctan2(np.linalg.norm(np.cross(p,np.array([0.0, 0.0, 1.0]))), np.dot(p,np.array([0.0, 0.0, 1.0])))
        print('alpha', alpha)
        print('gamma', gama)
        V = np.sqrt((2*k*d**2)/m)
        vz = V*np.cos(gama)
        vx = V*np.sin(gama)*np.cos(alpha)
        vy = V*np.sin(gama)*np.sin(alpha)
        X = np.zeros(N)
        Y = np.zeros(N)
        Z = np.zeros(N)
        T = np.linspace(0,10,N)
        for i in range(1, N): 
            X[i] = vx*T[i] 
            Y[i] = vy*T[i] 
            Z[i] = vz*T[i]- 0.5*g*T[i]**2 + self.sling_pos[2]
            if Z[i]<= 0.5:
                Z[i] = Z[i-1] 
                X[i] = X[i-1]
                Y[i] = Y[i-1]

        self.traj_x =  X + self.sling_pos[0]
        self.traj_y =  Y + self.sling_pos[1]
        self.traj_z =  Z + 0.2 

    def sling_pose_callback(self, msg): 
        pos = msg.transform.translation
        if self.flag == 0: 
            self.sling_pos = np.array([pos.x, pos.y, pos.z])
            goal = np.array([pos.x , pos.y, pos.z + 0.2])
            self.puplish_way_point(goal)
        
            
    

    def hand_pose_callback(self, msg): 
        pos = msg.transform.translation
        if self.flag == 0: 
            self.hand_pos = np.array([pos.x, pos.y, pos.z])

            # shift = 0.15
            # p = self.sling_pos - self.hand_pos
            # alpha = np.arctan2(np.linalg.norm(np.cross(p,np.array([1.0, 0.0, 0.0]))), np.dot(p,np.array([1.0, 0.0, 0.0])))
            # x_shift = shift * alpha
            # y_shift = shift * alpha

            # goal = np.array([pos.x, pos.y, pos.z + 0.2])
            # self.puplish_way_point(goal)

        elif self.flag == 1: 
            pass
        #     self.calculate_trajectory()
        #     if self.counter < self.traj_x.shape[0]: 
        #         self.publish_trajectory(self.counter)
        #         self.counter = self.counter + 1
        # self.rate.sleep()
                
    def publish_trajectory(self,i): 
        worldFrame = rospy.get_param("~worldFrame", "/world")
        msg = Position()
        msg.header.frame_id = worldFrame
        msg.x = self.traj_x[i]
        msg.y = self.traj_y[i]
        msg.z = self.traj_z[i] 
        msg.yaw = self.yaw
        msg.header.seq = 0
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)

    def drone_pos_callback(self, msg):  
        q1 = msg.transform.rotation.x
        q2 = msg.transform.rotation.y
        q3 = msg.transform.rotation.z
        q0 = msg.transform.rotation.w
        self.yaw = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))

    def flag_callback(self, msg): 
        self.flag = msg.data 
        if self.flag == 1: 
            self.calculate_trajectory()
            while self.counter < self.traj_x.shape[0]:
                self.publish_trajectory(self.counter)
                self.counter = self.counter + 1
                self.rate.sleep()

    
    def puplish_way_point(self, goal): 
        worldFrame = rospy.get_param("~worldFrame", "/world")
        msg = Position()
        msg.header.frame_id = worldFrame
        msg.yaw = self.yaw
        msg.header.seq = 0
        msg.x = goal[0]
        msg.y = goal[1]
        msg.z = goal[2]
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)
        #self.rate.sleep()
