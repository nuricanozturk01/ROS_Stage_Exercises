#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#18070006034 Nuri Can ÖZTÜRK
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/moveToGo.py

import rospy
from geometry_msgs.msg import Twist,Point
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import atan2, pow,sqrt
import sys
from tf.transformations import euler_from_quaternion
from euclideanDistance.srv import EuclideanDistance, EuclideanDistanceResponse



class Turtlebot:


    def __init__(self,nodeName,goalx,goaly):
        rospy.init_node("name",anonymous=True)
        #self.goalX = rospy.get_param("x1")
        #self.goalX = rospy.get_param("y1")
	    self.pub = rospy.Publisher(nodeName+'/cmd_vel',Twist,queue_size=10)
        #self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.sub = rospy.Subscriber(nodeName+"/odom",Odometry,callback=self.update_pose)
	    #self.sub = rospy.Subscriber("/odom",Odometry,callback=self.update_pose)
        self.yaw = 0
        self.odometry = Odometry()
        self.rate = rospy.Rate(10)
        self.goalX = goalx
        self.goalY = goaly

	
        
    def update_pose(self,msg):
        self.odometry = msg
        self.orientation_q = msg.pose.pose.orientation
        self.orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(self.orientation_list)
        print(self.positionx, self.positiony, self.yaw)
     

    def euclidean_distance(self,goal_pose):    
       return sqrt(pow((goal_pose.x - self.odometry.pose.pose.position.x),2) + pow((goal_pose.y - self.odometry.pose.pose.position.y),2))

	# service

	#rospy.wait_for_service("euclideanDistanceServer")
   	#try:
        #    calculateSteeringAngle= rospy.ServiceProxy("euclideanDistanceServer",EuclideanDistance)
        #    response = calculateSteeringAngle(goal_pose.y, self.odometry.pose.pose.position.y, goal_pose.x, self.odometry.pose.pose.position.x)
        #    rospy.loginfo(response.result)
	#    return response.result
	  

    	#except Exception:
       	#  print("Error: ")  
		  

    def linear_vel(self,goal_pose,constant=1.5):
        return constant * self.euclidean_distance(goal_pose)  

    def steering_angle(self,goal_pose):
        return atan2(goal_pose.y - self.odometry.pose.pose.position.y, goal_pose.x - self.odometry.pose.pose.position.x)

    def angular_vel(self,goal_pose,constant=1.5):
        return constant * (self.steering_angle(goal_pose) - self.yaw)

    def move2goal (self):
        goalpose = Pose()
    
        goalpose.x = self.goalX
        goalpose.y = self.goalY
        
        tolarence = 0.01
        vel_msg = Twist()
    
        while self.euclidean_distance(goalpose)>= tolarence:
            vel_msg.linear.x  = self.linear_vel(goalpose)
            vel_msg.linear.y  = 0
            vel_msg.linear.z  = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goalpose)
            self.pub.publish(vel_msg)
	    self.rate.sleep()
	    print(self.euclidean_distance(goalpose))
	    
		
 
        vel_msg.linear.x  = 0
        vel_msg.angular.z = 0
        self.pub.publish(vel_msg)
        



def way3(nodeName):
	
	if nodeName == "robot_0":
	    x = Turtlebot(nodeName,2,0)
	    x.move2goal()
	    rospy.spin()
	else:
	    x = Turtlebot(nodeName,6,0)
	    x.move2goal()
	    rospy.spin()

	# gazebo3
	"""
	x = Turtlebot(nodeName,4,0)
	x.move2goal()
	rospy.spin()
	"""
def way1(nodeName):
	x1 = rospy.get_param("x1")
	y1 = rospy.get_param("y1")
	x2 = rospy.get_param("x2")
	y2 = rospy.get_param("y2")
	if nodeName == "robot_0":
		x = Turtlebot(nodeName,x1,y1)
		x.move2goal()
		rospy.spin()
	else:
		x = Turtlebot(nodeName,x2,y2)
		x.move2goal()
		rospy.spin()
		
def way2(nodeName):
	x1 = rospy.get_param("x1")
	y1 = rospy.get_param("y1")
	x2 = rospy.get_param("x2")
	y2 = rospy.get_param("y2")
	if nodeName == "robot_0":
		x = Turtlebot(nodeName,x1,y1)
		x.move2goal()
		rospy.spin()
	else:
		x = Turtlebot(nodeName,x2,y2)
		x.move2goal()
		rospy.spin()

if __name__ == "__main__":
    nodeId = str(sys.argv[1])
    try:
	    nodeName = "robot_"+nodeId
	    way3(nodeName)
	    #way2(nodeName)
	    #way1(nodeName)	
		

               
    except IndexError as e:	
	print("error ", e)
	#main(nodeName)
    except Exception as e:
	print("error ", e)
	#main(nodeName)
    except rospy.ROSInterruptException:
        pass
	   	
