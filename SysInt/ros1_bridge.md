# Building the ros1 bridge
* Ros1 bridge is used to enable communication between ros1 nodes and to ros2 nodes.

* It works by mapping msg files from the ros1 packages to ros2 packages. (The files shall be exactly similar to each other for it to work)

* This is a very intricate process so please follow steps very carefully :P

* **Note:** Good reference- https://www.youtube.com/watch?v=vBlUFIOHEIo

## Checking workspaces
* We shall already have 2 workspaces-- one for ros1 and the other for ros2

* Create a new workspace with `mkdir -p ~/ros1_bridge_ws/src`

* In the src folder clone the ros1_bridge github repo: `git clone -b foxy https://github.com/ros2/ros1_bridge.git` (Change foxy according to ros distro)

* Build the ros1 and ros2 workspaces and check if their msg files are detected individually (`rosmsg list` and `ros2 interface list`)


## Changes to ROS2 workspace
* Note that this is for the **ros2** workspace.

* In the package with msg files to be mapped, add a file named  `bridge_mapping.yaml`

* This file will be used if the package names in ros1 and ros2 workspaces do not match. Likewise with msg names and their field names.

* Inside the yaml file write the mappings using this format:

```
    -
        ros1_package_name: 'ros1_pkg_name'
        ros1_message_name: 'ros1_msg_name'
        ros2_package_name: 'ros2_pkg_name'
        ros2_message_name: 'ros2_msg_name'
        fields_1_to_2:
            foo: 'foo'
            ros1_bar: 'ros2_bar'
    -
        <New mapping>

```
* Note that multiple mappings shall not be present in one point. Each point starts with `-`.

* It is a good practice to keep the names of the msg files and each of their field names exactly same in ros1 and ros2. In such a case only the package names need to be specified. Bridge will map only the msgs with matching names unless specified explicitly in YAML file.

```
    -
        ros1_package_name: 'mrpt_msgs'
        ros2_package_name: 'dv_msgs'
```

* Add the following to CMakeLists.txt of the package:

```
    install(
        FILES bridge_mapping.yaml
        DESTINATION share/${PROJECT_NAME})
```

* And this to package.xml inside the `<export>` block: `<ros1_bridge mapping_rules="bridge_mapping.yaml"/>`

* build the workspace


## Build
* Now open a new terminal

* Source the ros1 **distro**, then the ros2 distro, then the ros1 package and then the ros2 package. (Use build.sh for reference)

* Remove any 'build' 'install' 'log' folders if already present. (For safety)

* Build the bridge using `colcon build --symlink-install --cmake-force-configure`

* Source the workspace using `source install/setup.bash`

* `ros2 run ros1_bridge dynamic_bridge --print-pairs` To check all the mapped msg pairs.


## Running the bridge
* Source the workspace and run the bridge using: `ros2 run ros1_bridge dynamic_bridge`

* Run the ros1 and ros2 nodes separately and see them communicate!