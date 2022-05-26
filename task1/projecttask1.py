#!/usr/bin/env python
#18070006034 Nuri Can OZTURK
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/projecttask1.py

import rospy
import sys,math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
nodeId = str(sys.argv[1])
nodeName = "robot_"+nodeId

vel_msg = Twist()
vel_msg.linear.x = 0.5
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
vel_msg.angular.z = 0

minfrontdist=0.9
stopdist = 0.7

stop = 0
obstacle = False
speed = 1
sspeed = 0.1
minleft=1000000.0
minright=1000000.0

maxleft = 5
maxright = 5


rospy.init_node(nodeName , anonymous = True)
pub = rospy.Publisher(nodeName + '/cmd_vel', Twist, queue_size=10)


def rotate(clockwise):
    angular_speed = 30*(math.pi/180.0) 
    relativeangle = 86*(math.pi/180.0)
    vel_msg.linear.x  = 0
    vel_msg.angular.z = angular_speed * clockwise
    rate = rospy.Rate(10)
    t0 = rospy.Time.now().to_sec()
    current_angle = 0 
    while current_angle < relativeangle: 
        pub.publish(vel_msg) 
        t1 = rospy.Time.now().to_sec() 
        current_angle = (t1 - t0) * angular_speed
        rate.sleep() 
    stopRobot() 



def moveRobot(lx,ly,dist=2):
    vel_msg.linear.x  = lx
    vel_msg.linear.y  = ly
    vel_msg.linear.z  = 0
    vel_msg.angular.z = 0
    rate = rospy.Rate(10)
    t0 = rospy.Time.now().to_sec() 
    currentDistance = 0 
    while currentDistance < dist:
        pub.publish(vel_msg) 
        t1 = rospy.Time.now().to_sec() 
        currentDistance = (t1 - t0) * vel_msg.linear.x 
        rate.sleep()
    stopRobot()    


def stopRobot():
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0.0
    pub.publish(vel_msg)

def obstacleFunc(minleft, minright):
    stopRobot()
    if nodeName == "robot_1":
        if minleft <= minright:
            rotate(-1)
        else:
            rotate(+1)
    else:
        if minleft <= minright:
            rotate(1)
        else:
            rotate(-1)

       

def callback(msg):
    global obstacle
    obstacle = False
    size = len(msg.ranges)
    global minleft
    global minright
    global maxleft
    global maxright
    global stop
    for i in range(0,size):
        if float(msg.ranges[i] < minfrontdist):
            obstacle = True
        if float(msg.ranges[i] < stopdist):
            stop = 1
        if obstacle:
            minleft = msg.ranges[90] # left
            minright = msg.ranges[269] # right  


def moveNotObstacle(flag):
    global obstacle
    while not obstacle:
        vel_msg.linear.x = speed
        vel_msg.angular.z = 0.0
        pub.publish(vel_msg)
    if flag:
        if obstacle:
            obstacleFunc(minleft,minright)    
        
def moveRobot1():
    moveNotObstacle(True)
    moveRobot(1,0)
    stopRobot()
    rotate(-1)
    moveNotObstacle(False)
    stopRobot()
    rotate(1)
    moveRobot(1,0)
    rotate(1)
    moveNotObstacle(True)
    moveRobot(1,0)
    rotate(-1)
    moveRobot(1,0,7)


def moveRobot0():
    moveNotObstacle(True)
    moveRobot(1,0)
    stopRobot()
    rotate(1)
    moveNotObstacle(False)
    stopRobot()
    rotate(-1)
    moveRobot(1,0)
    rotate(-1)
    moveNotObstacle(True)
    moveRobot(1,0)
    rotate(1)
    moveRobot(1,0,7)      



if __name__ == "__main__":
    sub = rospy.Subscriber(nodeName+'/base_scan', LaserScan , callback)
    if nodeName == "robot_1":
        moveRobot1()
    else: moveRobot0()
    rospy.spin()
