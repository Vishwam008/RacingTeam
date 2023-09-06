#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

rospy.init_node('VCU', anonymous=True)
time = 0

def process(msg):
    if msg.data == "Alive":
        rospy.loginfo("AI is working!")
        global time
        time = rospy.get_time()


pub = rospy.Publisher('VCU2AI', String, queue_size=10)
sub = rospy.Subscriber('AI2VCU', String, callback= process)


def VCU():
    global time
    time = rospy.get_time()
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        pub.publish('Alive')
        rate.sleep()
        if rospy.get_time() - time > 5:
            rospy.logerr('AI is not working')

if __name__ == '__main__':
    try:
        VCU()
    except:
        pass