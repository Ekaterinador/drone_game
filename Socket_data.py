#!/usr/bin/env python
import numpy as np
from numpy.lib.histograms import _histogram_bin_edges_dispatcher
from crazyflie_driver.msg import Position
from scipy.spatial.transform import Rotation as R

class Data(): 
    def __init__(self):
        self.hand = None
        self.sling = None
        self.head = None
        self.data =[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]]

    def hand_call_back(self,msg): 
        pos = msg.transform.translation
        self.hand = [pos.x, pos.y, pos.z]

    def sling_call_back(self,msg): 
        pos = msg.transform.translation
        self.sling = [pos.x, pos.y, pos.z]

    def head_call_back(self,msg): 
        pos = msg.transform.translation
        rot = msg.transform.rotation
        r = R.from_quat([rot.x, rot.y, rot.z, rot.w])
        angels = r.as_euler('zyx', degrees=True)
        quat = [rot.x, rot.y, rot.z, rot.w]
        self.head = [pos.x, pos.y, pos.z]
        self.data = [self.hand, self.sling, self.head, quat]


