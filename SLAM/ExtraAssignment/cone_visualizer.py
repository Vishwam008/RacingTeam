#! /usr/bin/env python3

import rospy
from test_package.msg import Track
from test_package.msg import Cone
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class visualizer:
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    def callback(self, data: Track):
        cones = MarkerArray()
        lis = []
        i = 0
        for item in data.track:
            cone = Marker()
            cone.header.frame_id = 'map'
            cone.header.stamp = rospy.Time.now()
            cone.ns = 'cones'
            cone.action = Marker.ADD
            cone.pose.position.x = item.location.x
            cone.pose.position.y = item.location.y
            cone.pose.position.z = item.location.z
            cone.pose.orientation.w = 1.0
            cone.id = i
            cone.type = Marker.CYLINDER
            
            if item.color == Cone.ORANGE_BIG:
                cone.scale.x = 1
                cone.scale.y = 1
                cone.scale.z = 2
            else:
                cone.scale.x = 0.5
                cone.scale.y = 0.5
                cone.scale.z = 1
                if item.color == Cone.BLUE:
                    cone.color.b = 1.0
                elif item.color == Cone.YELLOW:
                    cone.color.r = 1.0
                    cone.color.g = 1.0
                else:
                    cone.color.r = 1.0
                    cone.color.g = 0.65
                cone.color.a = 1.0
            i+=1
            lis.append(cone)
        cones.markers = lis
        self.marker_pub.publish(cones)


    def listener(self):
        rospy.init_node("Visualizer", anonymous=True)
        rospy.Subscriber('/fsds/testing_only/track', Track, self.callback)
        rospy.spin()


s = visualizer()
if __name__ == '__main__':
    s.listener()