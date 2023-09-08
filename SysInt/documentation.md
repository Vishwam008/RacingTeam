# ROS

## Random Tips
<span style="color: red; font-size: 20">If you get 'yaml module not found' then change python to python3 (DO NOT PURGE ROS :p)</span>

To kill any astray roscores: 
`killall -9 roscore` and `killall -9 rosmaster`

## Rosnodes
A node is an executable that uses ROS to communicate with other nodes. 

roscore shall be run whenever using ros. It starts up master which handles all the publishwes, subscribers and messages.

rosnode list:
lists all active nodes

rosrun package_name node_name (to run a node of a package)<br><br>


## Rostopic
Any topic with a datatype to which subscribers and publishers can subscribe/publish

rostopic list: prints all topics currently subscribed to and published

rostopic type topic: returns the message type

rostopic echo: prints the messages of the topic

rostopic pub [topic] [message_type] --\[args\]: publish a message<br>
-1: publish one message<br>
-r [h]: publish multiple messages at h frequency

rostopic hz: to see rate at which data is published<br><br>


## Rosservice
rosservice list: lists services<br>
rosservice type [service] : shows the service type<br>
rosservice show [service_type]: shows the arguments in a service<br>
rosservice call [service] [args]: call a service <br><br>


## Rosparam
Rosparam is used to change the values, store, load and operate on the values in the parameter database of the server.<br>
rosparam dump [file_name]: save in a file<br>
rosparam list<br>
rosparam load [file_name] [name_space]:<br>
rosparam get [parameter]<br> 
rosparam set [parameter] [values]<br><br>


## rqt_logger_level and rqt_console
Both of the nodes are GUIs.

rqt_console prints the messages that are output by any node

rqt_logger_level is used to specify the level of message shown in console: Fatal Error Warn Info Debug

Selecting any level would show messages of that level and those before it<br><br>


## roslaunch
roslaunch is used to used to start multiple nodes at a time

A launch file:
```
    <launch>
        <group ns="turtlesim1">
            <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
        </group>

        <group ns="turtlesim2">
            <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
        </group>
        
        <node pkg="turtlesim" name="mimic" type="mimic">
            <remap from="input" to="turtlesim1/turtle1"/>
            <remap from="output" to="turtlesim2/turtle1"/>
        </node>
    </launch>
```


Here, we launch two nodes of the same type but diff. names: turtlesim1 and 2. 

The output of one node is givent to turtlesim2 as input. So both turtles will execute same movements.

`<include file="$(find pr2_alpha)/$(env ROBOT).machine" />`

Where ROBOT is a system tag (a variable which can set by EXPORT from command line). This command finds the ROBOT.machine file in the pr2_alpha package and includes it.

`<machine name="c1" address="pre1" ros-root="$(env ROS_ROOT)" ros-package-path="$(env ROS_PACKAGE_PATH)" default="true" />`
Maps logical name 'c1' to the machine name pre1 and can be used as a name in place of the machine.

`<node pkg="amcl" type="amcl" name="amcl" machine="c1">`

This causes the amcl node to run on the c1 machine.


```
    <node pkg="move_base" type="move_base" name="move_base" machine="c2">
        <remap from="odom" to="pr2_base_odometry/odom" />
        <param name="controller_frequency" value="10.0" />
        <param name="footprint_padding" value="0.015" />
        <param name="controller_patience" value="15.0" />
        <param name="clearing_radius" value="0.59" />
        <rosparam file="$(find 2dnav_pr2)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find 2dnav_pr2)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find 2dnav_pr2)/move_base/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find 2dnav_pr2)/move_base/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find 2dnav_pr2)/move_base/navfn_params.yaml" command="load" />
        <rosparam file="$(find 2dnav_pr2)/move_base/base_local_planner_params.yaml" command="load" />
    </node>
```
**remap:** move_base nodes accepts odom but data published on pr2_base_odometry/odom so remapped. [Used to change topics like this]

**param:** Used to set paramters and as used inside the node tag, are private parameters.

**rosparam:** Used to load data from yaml files.

`<param name="move_base/local_costmap/re*solution" value="0.5"/>`

This can be used to override a parameter of a file that was loaded before.

**roslaunch arguments:** Command line arguments to avoid parameter override. <a href='http://wiki.ros.org/roslaunch/XML'>Link</a>


.launch file cannot execute any process by itself but is passed to an executable.

***TIP:***Put all config files into one package and launch files into other to avoid cyclic dependencies.

<br><br>

## rosed
Allows us to edit a file from any location by using package and file name

rosed [package_name] [file_name]<br><br>

## ROS msg and srv
msg: a text file describing the fields of a message and allows to write the source code of a ros message

srv: describes a service and has two parts- request and response

fields can be integers, strings, or header(special datatype that contains timestamp and coordinates)

msg file:

```
    Header header
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/TwistWithCovariance twist
```

srv file:
```
    int64 A
    int64 B
    ---
    int64 Sum
```
fields after the --- are responses and before are the requests.<br><br>

## Creating a ros srv and msg
Create a msg direcory in your ros package

Add an .msg file containing the required datatypes

The package.xml shall have the following tags:
```
    <build_depend>message_generation</build_depend>
    <exec_depend>message_runtime</exec_depend>
```

The CMakeLists.txt shall have the following:

```
    find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    )
```
```
    catkin_package(
        CATKIN_DEPENDS message_runtime
    )
```
```
    add_message_files(
        FILES
        <file_name>.msg
    )
```
```
    generate_messages(
        DEPENDS
        std_msgs
    )
```

To check if the message is detected:

rosmsg show \<message_name\>

For rossrv, in CMakeLists.txt:
```
    add_service_files(
        FILES
        AddTwoInts.srv
    )
```

To see info about service:
```
    rossrv show <service name>```
```
Build the package again as we have added messages.<br><br>

## Writing a Subscriber and Publisher
Make a 'scripts' folder in the ros package.
In CMakeLists.txt:
```
    catkin_install_python(PROGRAMS scripts/talker.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
```
Build the project after adding the files.<br>

Publisher:
```
    import rospy    from std_msgs.msg import String

    def talker():
        pub = rospy.Publisher('chatter', String, queue_size=10) # publish to topic chatter of type String, maximum messages at a time is 10
        rospy.init_node('talker', anonymous=True) # initialize the talker node(this node) and make it unique adding numbers at the end of the name (anonymous)
        rate = rospy.Rate(10) # loops at 10hz
        while not rospy.is_shutdown(): # while the node is running
            hello_str = "hello world %s" % rospy.get_time() # a string message
            rospy.loginfo(hello_str) # adding info to the log console
            pub.publish(hello_str) # publishing the message
            rate.sleep() # sleep such that publishing rate is 10hz

    if __name__ == '__main__':
        try:
            talker()
        except rospy.ROSInterruptException: # thrown when ^C is pressed
            pass
```

Listener:
```

    #!/usr/bin/env python3
    import rospy
    from std_msgs.msg import String

    def callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    if __name__ == '__main__':
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("chatter", String, callback) # Subscribes to chatter topic of String type 
        rospy.spin() # spin simply keeps python from exiting until this node is stopped
```

These can be run using rosrun. The activities can be monitored by rqt_console.
<br><br>

## Creating a node
Create a python file in the scripts dir with the following basic structure:

```

    #!/usr/bin/env python3
    import rospy

    if __name__ == "__main__":
        rospy.init_node("test_node") # Initializing a node named test_node

        rospy.loginfo("Hello World")
        rospy.logwarn("Warning")
        rospy.logerr("Error")

        rospy.sleep(1.0) # Sleep for 1 sec

        rospy.loginfo("End")

```
<br>

## Using arrays  and custom data types in .msg files
We can make an array of variable or fixed size of any datatype using:
```

    int32[] x
    float32[3] y    
```

A custom datatype can also be used as follows:
```

    Cone c
```
where Cone is a datatype defined in Cone.msg:
```

    int32 x
    int32 y
    int32 z
    time t
```
<br>

## Rosbag
Rosbag is used to record data from a running ROS system into a .bag file.

This file can be used to store the messages published by any running nodes.

```

	mkdir ~/bagfiles
	cd ~/bagfiles
	rosbag record -a
```

This will create a .bag file that begins with a date and time.



```

	rosbag info <your bagfile>
```	
This will print text that contains info about the published topics and the number of messages recorded in each topic.

### Rosbag play
This feature allows to re-publish the messages that were recorded in rosbag to create an environment similar to that at the time of recording.

```

	rosbag play -s 3 -r 2 <your bagfile>
```
This will play the bag file at twice the rate of original message publish. It will begin from the point in the file that is at a 3 sec offset from the beginning.

Can select specific topics to publish using --topics topic1 topic2

Can use --immediate to start immediately



### Recording a subset of the data
```

	rosbag record -O[not 0] test /turtle1/cmd_vel /turtle1/pose
```
This will store the messages of only cmd_vel and pose in test.bag file.


The time accuracy and precision of rosbag is very limited.

### Reading messages from rosbag files
#### Using rostopic echo
Play the bag file and on a different tab subscribe to the required topic and save the messages into a file using the tee function:
```

    rostopic echo /obs1/gps/fix | tee topic1.yaml
```

Have to use a different tab and file for each topic

#### Using ros_readbagfile
```

    ros_readbagfile <mybagfile.bag> [topic1] [topic2] [topic3] [...] | tee topics.yaml
```

To see the progress use this on a different tab:
```

    watch -n 1 'du -sk topics.yaml | awk '\''{printf "%.3f MiB %s\n", $1/1024, $2}'\'''
```

Use ripgrep to search a yaml file.

## roswtf
Checks for errors in the file systems or runtime. Also issues warnings.

Basically used when facing a build or communication issue.

## Applying ros system to multiple machines
<ul>
    <li>You only need one master. Select one machine to run it on.</li>
    <li>All nodes must be configured to use the same master, via ROS_MASTER_URI.</li>
    <li>There must be complete, bi-directional connectivity between all pairs of machines, on all ports (see ROS/NetworkSetup).</li>
    <li>Each machine must advertise itself by a name that all other machines can resolve (see ROS/NetworkSetup). </li>
</ul>

```
    ssh hal
    roscore
```
Login to hal machine and start the master

```
    ssh hal
    export ROS_MASTER_URI=http://hal:11311
    rosrun rospy_tutorials listener.py
```
configure the master and start a listener on hal

```
    ssh marvin
    export ROS_MASTER_URI=http://hal:11311
    rosrun rospy_tutorials talker.py
```
Login to marvin, configure the master and run the talker.

## TF(Transform)
Used to transform the coordinates of the robot, basically work with changing frames of a bot.

`roslaunch turtle_tf turtle_tf_demo.launch` launches a single window with 2 turtles. One can be moved using arrow keys and the other follows you.

Here tf broadcaster broadcasts three frames: world, turtle1 and turtle2.

### rqt_tf_tree
To visualise this use:
'''
rosrun rqt_tf_tree rqt_tf_tree
'''

### tf_echo
tf_echo reports the transform between any two frames broadcast over ROS. 

```
rosrun tf tf_echo [reference_frame] [target_frame]
```

### tf frames
Frames are very convenient and aid to visualization. Easy to visualize the LIDAR data from the FOR of LIDAR.
