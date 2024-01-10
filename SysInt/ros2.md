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

* `colcon build --packages-select my_package` to build a single package

* To use python scripts you cannot use cmake you have to build uding colcon and use setup.py etc.

* Fill the maintainer, description and license. These shall match exactly in the package.xml and setup.py

## Ros2 nodes
* ```
	<exec_depend>rclpy</exec_depend>
	<exec_depend>std_msgs</exec_depend>
  ``` Add these to package.xml

* ```entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},``` Add to setup.py

* Check dependencies and build

## Custom msg and srv
* Can only be created in a cmake package but can be used by any package

* implementation of custom msg in nodes is the same 

* Add this to `CMakeLists.txt`:
	```
	find_package(geometry_msgs REQUIRED)
	find_package(rosidl_default_generators REQUIRED)
	rosidl_generate_interfaces(${PROJECT_NAME}
	  "msg/Num.msg"
	  "msg/Sphere.msg"
	  "srv/AddThreeInts.srv"
	  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
	)
	```

* Add this to `package.xml`:
	```
	<depend>geometry_msgs</depend>
	<buildtool_depend>rosidl_default_generators</buildtool_depend>
	<exec_depend>rosidl_default_runtime</exec_depend>
	<member_of_group>rosidl_interface_packages</member_of_group>
	```

* Build the package

* Create a new package for interfaces with cmake.

* Add this to `package.xml` of the package in which you want to use the custom msg: `<exec_depend>tutorial_interfaces</exec_depend>`

