# rviz
<code>

    uint32_t shape = visualization_msgs::Marker::CUBE;
</code>

shape helps keep track of the shape (Cube for now)

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

<code>

    while (ros::ok())
    {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "my_frame";
        marker.header.stamp = ros::Time::now();
</code>
We add a frame with id="my_frame" and in reality this should be the frame relative to which we want the marker's pose.

<code>

    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type
</code>
ns(namespace) and id are used to uniquely identify the marker. A new marker with same details will replace the old one.

Type stores the type of shape(Here it is cube).

<code>

    marker.action = visualization_msgs::Marker::ADD;
</code>
marker.action specifies what to do with the marker.

<ol>
    <li>ADD: add the marker to display</li>
    <li>DELETE.ADD: create/modify</li>
    <li>DELETEALL: Delete all markers on display</li>
</ol>


<code>

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
</code>

Here we set the position and orientation(quaternion) of the marker. Then the scale in the 3 directions and the color. 

color.a is the aplha value that resembeles the opacity.

lifetime is the duration after which the marker will be autodeleted. ros::Duration means it will never delete.

<code>

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
</code>
We wait for a subscriber for the topic "visualization_markers"

## Interactive Markers

These markers can be used to interact with the user via rviz.

Interactive Marker Server is a node that communicates and processes signals from rviz.

Server -> Interactive Marker -> Interactive Marker Control -> Marker or Interaction

A regular marker also needs to be stored into a Interactive Marker control