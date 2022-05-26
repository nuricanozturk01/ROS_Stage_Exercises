#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from math import atan2,sqrt
from euclideanDistance.srv import EuclideanDistance, EuclideanDistanceResponse

def handle(req):
	result = sqrt(pow((req.goal_pose_x - req.odom_x),2) + pow((req.goal_pose_y - req.odom_y),2))
	print("Result is ",result)
	return EuclideanDistanceResponse(result)

def euclideanDistanceServer():
    rospy.init_node("euclideanDistanceServer")
    rospy.Service("euclideanDistanceServer",EuclideanDistance,handle)
    print("Start services")
    rospy.spin()

if __name__ == "__main__":
    euclideanDistanceServer() 


