#!/usr/bin/env python3
#####
# import libraries
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
import math
import time
#####
# global variables of turtlebot pose
robot_x=0
robot_y=0
robot_theta=0

# subscriber pose callback function
def get_pose(pose):
    global robot_x, robot_y, robot_theta
    robot_x = pose.x
    robot_y = pose.y
    robot_theta = pose.theta
    # print("m called")

# move turtlebot from one coordinate to another
def move_turtle(initial_coor, final_coor):
    vel = Twist()
    # initial coordinate of turtle bot
    init = 5.544445
    # small constant to prevent slope becoming undefined
    epsilon = 1e-7

    # slope formula tan inverse manipulation
    slope = math.atan((final_coor[1]-initial_coor[1])/(final_coor[0]-initial_coor[0]+epsilon))
    if (final_coor[1]-initial_coor[1]) < 0 and (final_coor[0]-initial_coor[0])<0:
        slope = slope - math.pi
    elif (final_coor[1]-initial_coor[1]) < 0 and (final_coor[0]-initial_coor[0]) != 0:
        slope = slope + math.pi
    elif (final_coor[0]-initial_coor[0]) == 0 and (final_coor[1]-initial_coor[1])>0:
        slope = math.pi/2
    elif (final_coor[0]-initial_coor[0]) == 0 and (final_coor[1]-initial_coor[1])<0:
        slope = -1*math.pi/2


    print('slope{}'.format(slope))
    # calculate distance b/w initial and goal
    distance = math.sqrt((final_coor[1]-initial_coor[1])**2+(final_coor[0]-initial_coor[0]+epsilon)**2)
    error_theta = abs(robot_theta - slope)
    print(error_theta)
    rate = rospy.Rate(10)
    print("rotation")

    # rotate until in correct orientation
    while error_theta>0.02:
        vel.linear.x = 0
        vel.angular.z = 0.5*error_theta
        pub.publish(vel)
        # slope = math.atan((final_coor[1]+init-robot_y)/(final_coor[0]+init-robot_x+epsilon))
        error_theta = abs(robot_theta - slope)
        print("rotation_loop")
        print(error_theta)
        rate.sleep()

    # go straight until goal is reached
    while distance>0.15:
        vel.linear.x = 1*distance
        vel.angular.z = 0
        distance = math.sqrt((final_coor[1]+init-robot_y)**2+(final_coor[0]+init-robot_x)**2)
        print('distance')
        print(distance)
        pub.publish(vel)
        rate.sleep()
        
    # print("hello")
    # print(robot_x, robot_y, robot_theta)
    vel.linear.x = 0
    vel.angular.z = 0
    print("hello")
    # while True:
    pub.publish(vel)
    #     print(robot_x)
        
    pass


def travel_turtle(coords_list):
    # initial coordinate of turtle bot
    initial_coor = (0, 0)
    rospy.init_node("painter")
    rospy.Subscriber('/turtle1/pose', Pose, get_pose)
    global pub 
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    for coor in coords_list:
        # turtle moving function
        move_turtle(initial_coor, coor)
        print("reached")
        initial_coor = coor
        # rate.sleep()
    pass

if __name__ == "__main__":
    # list of coordinates to go
    coords = [(1, 2), (0, 0), (1, 0), (1, 1), (1, -2)]
    # coords = [(0, -1)]
    travel_turtle(coords)