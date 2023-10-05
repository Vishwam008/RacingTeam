#! /usr/bin/env python3
import rospy
from beginner_tutorials.msg import custom, velocity, yaw, position
import random

curr_vel = 0
curr_yaw = 0
timestamp = 0

def vel_callback(msg: velocity):
    rospy.loginfo("Received velocity")
    global curr_vel, time_vel
    curr_vel = msg.velocity
    time_vel = msg.timestamp


def yaw_callback(msg: yaw):
    rospy.loginfo("Received yaw")
    global curr_yaw, time_yaw
    curr_yaw = msg.yaw
    time_yaw = msg.timestamp    


def cone_callback(msg: custom):
    rospy.loginfo("Received cone array.")


if __name__ == "__main__":
    rospy.init_node('SLAM', anonymous=True)
    pub = rospy.Publisher('car_pos', position, queue_size=10) # publisher to publish position and time

    vel_sub = rospy.Subscriber('velocity', velocity, callback=vel_callback) # subscribers for 3 topics
    yaw_sub = rospy.Subscriber('yaw', yaw, callback=yaw_callback)
    cone_sub = rospy.Subscriber('cone_arr', custom, callback=cone_callback)

    rate = rospy.Rate(2) # 2hertz

    while not rospy.is_shutdown():
        msg = position() # making a message of type position.msg
        msg.x = random.randrange(1000) # random values for position
        msg.y = random.randrange(1000)
        msg.theta = random.randrange(314)/100
        msg.t = rospy.get_time()
        pub.publish(msg)
        rospy.loginfo(msg)
        rate.sleep()