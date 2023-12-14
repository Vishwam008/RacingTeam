# Ros 2
* `ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle` changes the name of the node to 'my_turtle'

* `ros2 run <node>`

* `ros2 node info` 		`ros2 node list` and likewise with topics, services etc.

* `ros2 topic list -t` Topics listed along with their message types.

* `rosmsg/rossrv show <message>` --> `ros2 interface show <message>/<service>/<action>`

* `ros2 topic pub <topic_name> <msg_type> '<args>'` Eg: `ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`
	Options: --rate, --once

* `ros2 topic hz`

* `colcon build` to Build the workspace

* `rosdep install -i --from-path src --rosdistro foxy -y` in the workspace directory to install all the dependencies of the package

* Always source the underlay in a different terminal than the one in which you built your worksapce

* Any changes in the underlay does not affect the overlay

* There cannot be nested packages and good practice is to make a separate folder in each package and each folder shall be in the `src` folder.

## Ros2 parameters
* `ros2 param list` `ros2 param get <node_name> <parameter_name>` `ros2 param set <node_name> <parameter_name> <value>`

* `ros2 param dump <node_name>` save all the values of the parameters of a node into a file

* `ros2 param load <node_name> <parameter_file>` load and set the parameters from a YAML file

* `ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>` startup a node with parameters from a file


## Action servers
* Used for longer tasks

* A goal is sent by the client which is acknowledged by the server. A steady feedback is given by the server till the task is finished. After the task is finished, the result is sent to the client.

* Consists of a goal service, result service and a feedback topic. After goal is acknowledged as response of the goal service, a result request is sent. Feedback is cont. published onto the feedback topic.

* It can be aborted before completion unlike services

* Have a type(action) similar to msg and srv.

* `ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"`

* `ros2 bag record <topic_name>` 

## Ros bags
* `ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose` -o is to choose file name. Subset is the file name

## Ros2 packages
* `ros2 pkg create --build-type ament_python <package_name>` to create a package

* Structure:

	* package.xml file containing meta information about the package

	* resource/\<package_name\> marker file for the package

	* setup.cfg is required when a package has executables, so ros2 run can find them

	* setup.py containing instructions for how to install the package

	* <package_name> - a directory with the same name as your package, used by ROS 2 tools to find your package, contains __init__.py





