Rawseeds Dataset SLAM in ROS

The csv2bag.py is a conversion function that takes 4 dataset csv files and writes the data to a bag file.

In a terminal change the directory to the directory that the files were extracted to.
In the terminal type the command "python csv2bag.py".
Wait for the program to finish. You now have the bag file needed for playback.

In a new terminal run the command "roscore".
In the original terminal run the command "rosrun gmapping slam_gmapping scan:=SICK_FRONT" or "rosrun gmapping slam_gmapping scan:=SICK_REAR".
In a new terminal change directory to the same directory that the files were extracted to.
In that termianl run the command "rosbag play --clock rawseeds.bag".

To watch the process happen, in a seperate terminal run the command "rosrun rviz rviz".
Click the add button and add the TF in the by diplay type.
Click the add button and select by topic and add the map topic.
There now should be a map with transforms representing the robot moving.
