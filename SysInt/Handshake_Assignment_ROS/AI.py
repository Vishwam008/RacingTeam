#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

rospy.init_node('AI', anonymous=True)
time = 0

def process(msg):
    if msg.data == "Alive":
        rospy.loginfo("VCU is working!")
        global time
        time = rospy.get_time()


pub = rospy.Publisher('AI2VCU', String, queue_size=10)
sub = rospy.Subscriber('VCU2AI', String, callback= process)


def AI():
    global time
    time = rospy.get_time()
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        pub.publish('Alive')
        rate.sleep()
        if rospy.get_time() - time > 5:
            rospy.logerr('VCU is not working')

if __name__ == '__main__':
    try:
        AI()
    except:
        pass