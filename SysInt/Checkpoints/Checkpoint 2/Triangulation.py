#! /usr/bin/env python3
import rospy
from beginner_tutorials.msg import Cone
from beginner_tutorials.msg import custom
import random 

def r():
    return [random.randrange(0,100), random.randrange(0,100), random.randrange(0,100), rospy.get_time(), [0,0,255]] 
    # returns a random Cone datatype

if __name__ == "__main__":
    rospy.init_node("Triangulation", anonymous=True)
    pub = rospy.Publisher("Cone", custom, queue_size=10)
    # rate = rospy.Rate(10)
    msg = custom() # constructor to create an empty custom message
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg.cones.append(r()) # append a Cone datatype (Cone: x y z time color[])
        msg.cones.append(r())
        msg.cones.append(r())
        msg.cones.append(r())
        msg.cones.append(r())
        pub.publish(msg)
        rate.sleep()