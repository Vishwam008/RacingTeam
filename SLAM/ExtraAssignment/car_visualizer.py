#! /usr/bin/env python3

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import rospy

class car_visualizer:
    car_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    def callback(self, data: Odometry):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z

        car = Marker()
        car.header.frame_id = 'map'
        car.header.stamp = rospy.Time.now()
        car.ns = 'car'
        car.action = Marker.ADD
        car.pose.position.x = x
        car.pose.position.y = y
        car.pose.position.z = z
        car.pose.orientation.w = data.pose.pose.orientation.w
        car.pose.orientation.x = data.pose.pose.orientation.x
        car.pose.orientation.y = data.pose.pose.orientation.y
        car.pose.orientation.z = data.pose.pose.orientation.z
        car.id = 0
        car.type = Marker.CYLINDER

        car.color.r = 1.0
        car.color.a = 1.0
        car.scale.x = 0.5
        car.scale.y = 0.5
        car.scale.z = 1

        self.car_pub.publish(car)    
        # rospy.loginfo('Received: x[{}], y[{}], z[{}]'.format(str(x), str(y), str(z)))

    def listener(self):
        rospy.init_node('car', anonymous=True)
        rospy.Subscriber('/fsds/testing_only/odom', Odometry, callback=self.callback)
        rospy.spin()


s = car_visualizer()
if __name__ == '__main__':
    s.listener()