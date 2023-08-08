## Explain SLAM in your own words.
SLAM stands for simultaneous localisation and mapping. This involves creating a map of the surroundings in which the robot has travelled and pinpointing the location of the robot in that map.

## Try to explain what SLAM achieves in our specific scenario (a race car and a world having cones only) ? State what are the inputs and outputs.
Using SLAM we can obtain the map of the track we are given which can then be used for fast subsequent laps and devising the racing line. The localisation provided is used to give appropriate command signals taking into account the nature of the path ahead(type of curve or straight).

The inputs at any given point of time is the instantaneous velocity, yaw rate, previous map, previous position, previous yaw and the timestamp.

The outputs given out is an updated map and the instantaneous position of the car.


## Which type of SLAM (full vs online) would we require for our problem statement? Why?
We would definitely need an online SLAM as we are not given a track beforehand and the car is supposed to identify the path ahead and act appropriately. Thus we would need to have the instantaneous map and position of the car to manoeuvre it.

## Differentiate between Filter based, Particle Filter based and Graph Based SLAM and which algorithm is preferable in which type of situations.
Filter based SLAM uses Kalman filters to optimize noisy sensor data and is one of the primitive types of SLAMs called EKF(Extended Kalman Filter). It is mainly used in linear environments and struggles with complex ones. It is also known for its high computational power.

Particle based SLAM is used in places where high accuracy is required and time is less of a constraint as it is computationally intensive. It identifies landmarks as particles in space which are optimized over time converging to their original position. It is recommeded to use this in complex environments.

Graph based SLAM visualizes the environment as a graph and optimizes the graph over time. It has good loop closure features and optimizes the track as the robot passes on it several times. It is recommended for applications wherein the track is a loop and it needs to optimized on every lap. It is not very computationally intensive and has real time applications. Thus it makes a very suitable method for our use.

## Explain the difference between mapping and localization in the context of SLAM.
Mapping refers to creating a map of the envrionment and identifying and locating obstacles and landmarks sensed by the sensors.

Localization refers to pinpointing the coordinates of the robot in the map we just created.

In SLAM, it is referred to as simultaneous as for mapping we need the coordinates of the car and for determining the coordinates, we need the map.

## What is loop closure, and why is it important in SLAM?
Loop closure refers to the mechanism which identifies that the robot has come around a full loop and has returned to the position where it started or was present before.

The problem that arises is that you sense a part of the environment that you have been to before but you are not at the end of a loop according to your mapping.

Thus transformations are needed on the map to convert it such that it represents a loop.

## Name and describe 3 popular open-source SLAM frameworks or libraries. Explain what algorithm they use and what all inputs they require and from what sensors.
<ul>
	<li><b>Google Cartographer: </b>It uses Graph based SLAM to identify nodes and optimizes the map over time. It uses 2d LIDAR and IMU data as input. It has 2 main parts: local and global SLAM. Local creates submaps of the environment and global integrates all of these submaps and creates the overall map in addition to optimmizing it over time. Google Cartographer also has a built in loop closure functionality.</li>
	<li><b>OpenSLAM GMapping: </b>It uses a particle based filter to determine obstacles. This makes it very preferable for mapping small and complex places such as the floor plan of a home. It is widely used in automated cleaner robots because of the high amount of accuracy of the maps delivered. It also takes 2d LIDAR and wheel odometry as input.</li>
	<li>
		<b>ORB-SLAM (Oriented FAST and Rotated BRIEF SLAM): </b>
		This type uses a mono ocular camera as input and optionally IMU data also. It uses a graph based SLAM and keypoint matching algorithm to map surroundings and identify robot trajectory. It has real time applications and very suitable for camera only applications. It has loop closing features available by using the geometric features of the trajectory and map.
	</li>
	<li><b></b></li>
</ul>
