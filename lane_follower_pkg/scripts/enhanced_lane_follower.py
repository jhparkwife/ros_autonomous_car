#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import math
import time
import dynamic_reconfigure.server
from lane_follower_pkg.cfg import LaneFollowerConfig

class EnhancedLaneFollower:
    def __init__(self):
        # ... (기존 코드 유지)
        
        # Dynamic Reconfigure
        self.dyn_reconf_srv = dynamic_reconfigure.server.Server(
            LaneFollowerConfig, self.reconfigure_callback)
    
    def reconfigure_callback(self, config, level):
        self.obstacle_distance_threshold = config.obstacle_distance_threshold
        self.safety_angle_range = config.safety_angle_range
        self.drive_speed = config.drive_speed
        return config