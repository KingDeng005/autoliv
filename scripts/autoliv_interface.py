#!/usr/bin/env python
# created by Fuheng Deng on 10/26/2017

import rospy
import Autoliv.msg import TargetPolarLong
import visualization_msgs.msg import Marker
import visualization_msgs.msg import MarkerArray
import numpy as np
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Point32

class AutoliveInterface:
    def __init__(self):
        self.frame_id = 'laser'
 	self.scale_v = 0.25 * 4
	self.scale_h = 1
	self.sub_radar_track =  

