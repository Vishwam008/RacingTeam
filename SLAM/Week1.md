# Introduction to SLAM
Two parts: Localisation and Mapping

In localisation we assume the location of a landmark as constant and these taken as reference to localise the car.

In mapping these landmarks are to be detected and their location from the car at each timestamp is to be noted to create a map.

So the problem statement is that we have a set of inputs commands and observations in terms of speeds and we have to find the set of coordinates and a map of the environment.

<b>Probabilistic approach: </b>We always have an uncertainty in the observations we make and the output of the input commands. This uncertainty can be taken into consideration using probabilistic approach.

The online SLAM shich we are using involves using the previous positions as input and recursively solving integrals to give a map of the surroundings.

The uncertainties are always correlated to each other in such a way that if we know the location of a certain landmark with high confidence then it also increases the confidence and accuracy of previous positions as well as landmarks.

Increasing the precision of our observations and deductions is very high as wrong data decisions leads to wrong decisions in the future and thus a wrong map.

### Loop closure:
Loop closure refers to the mechanism which identifies that the robot has come around a full loop and has returned to the position where it started or was present before.

The problem that arises is that you sense a part of the environment that you have been to before but you are not at the end of a loop according to your mapping.

Thus transformations are needed on the map to convert it such that it represents a loop.






# Rosbag
Rosbag is used to record data from a running ROS system into a .bag file.

This file can be used to store the messages published by any running nodes.

<code>

	mkdir ~/bagfiles
	cd ~/bagfiles
	rosbag record -a
</code>

This will create a .bag file that begins with a date and time.



<code>

	rosbag info <your bagfile>
</code>	
This will print text that contains info about the published topics and the number of messages recorded in each topic.

### Rosbag play
This feature allows to re-publish the messages that were recorded in rosbag to create an environment similar to that at the time of recording.

<code>

	rosbag play -s 3 -r 2 <your bagfile>
</code>
This will play the bag file at twice the rate of original message publish. It will begin from the point in the file that is at a 3 sec offset from the beginning.



### Recording a subset of the data
<code>

	rosbag record -O test /turtle1/cmd_vel /turtle1/pose
</code>
This will store the messages of only cmd_vel and pose in test.bag file.


The time accuracy and precision of rosbag is very limited.