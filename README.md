# ENPM661_Project3_Phase2

Readme File

Github link:- https://github.com/Ishaan1810/ENPM661_Project3_Phase2

Team Members:-
Ishaan Parikh:- 119135891
Manav Nagda:- 119133545

Part 01:-
Libraries Used:-

matplotlib.pyplot
math
numpy
time
heapq
rospy
geometry_msgs.msgs


Part 02:-
The Package consists of 3 files:
1) src:- the souce code
	it is named publisher.py
2)launch file
	this is named turtlebot3_world.launch  
	a file named turtlebot3_world already existed in the turtlebot3_gazebo package so runnning this 	might be of an issue in other systems
3)world file
	This is named map.world and is unchanged and reamins as given in the project.
	Make sure this file is in the worlds folder in turtlebot3_gazebo.
To run the package two terminals will be required:

  In the first terminal run the following 
  
    cd ~/catkin_ws && catkin_make
    
    source ~/.bashrc
    
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
    
  In the second terminal after the turtlebot is spawned  run the following commands
    
    source ~/.bashrc
    
    rosrun turtlebot3_gazebo publisher.py
    
      

    
