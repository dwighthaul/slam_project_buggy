# slam_project_buggy

This project is to have a buggy running around and create a map autonomously using a RPLIDAR.

It's based int the ROS environement.



# How to create a map:

## Init
Plug your rplidar to your device

## 2 ways:
## -> Create a bag, then play the bag to create the map

1. 
Look at the custom_launch.launch
Comment/Remove the slam.launch include

Start:

	runcore
	runlaunch slam_project_buggy custom_launch.launch
So save the bag, in an other terminal
	rosbag record scan tf


2. 
Move your RPLidar around


3. 
Stop the record.
Stop the launch file.


4.
Uncomment the slam.launch
Comment the tdf node and the include of the rplidar launch file

5.
Launch the custom_launch file:

	runlaunch slam_project_buggy slam_project_buggy.launch
Or 

	runlaunch slam_project_buggy custom_launch.launch

To see the map in the rviz windows

Play the bag:

	rosbag play "name_of_your_bag.bag>
Wait for it to finish

If you run the rviz you should see the map creation

6.
Save the map:

	rosrun map_server map_saver "Mapname"


I use this method to create the bag in: data/final_data/2017-07-13-17-38-19.bag


## -> All in once:

1.
To create a map you need to launch: 

	roslaunch slam_project_buggy slam_project_buggy.launch
To see it in the rviz windows.
Or if you don't want to see the map in the rviz windows you can launch the 

	roslaunch slam_project_buggy custom_launch.launch
file.

2. 
Move your RPLidar around.

3.
Save the map:

	rosrun map_server map_saver "Mapname"


##links


How to save the map:
http://wiki.ros.org/map_server

Hector slam:
wiki.ros.org/hector_slam



# How to control the buggy:

