# TF2
`rosrun tf2_ros static_transform_publisher 1 1 0 0 0 0 parent child` publishes a static transform. first 3 args are x,y,z and next 3 are roll, pitch and yaw. Parent is the parent frame and likewise.

`rosrun rqt_tf_tree rqt_tf_tree` to visualize the tree.

In rviz select the parent frame as the fixed frame to view all other frames (children).

`rosrun tf tf_echo parent grandchild` prints the tranformation matrix directly from parent to grand child.