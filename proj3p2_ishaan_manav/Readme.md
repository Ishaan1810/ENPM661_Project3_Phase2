# ENPM661_Project3_Phase2

Readme File

Github link:- https://github.com/Ishaan1810/ENPM661_Project3_Phase2

Drive link for videos: https://drive.google.com/drive/folders/1OV9w1nBQ6pC0hNXdmAt4pe3B1syIJ4Vh?usp=sharing

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

Instructions to run
1. enter clearance of bot in mm (please put more than 10 mm as less than that is too small to be reflected)
2. add Low rpm(between 5 to 15 rpm)
3. add high rpm(between 5 to 15 rpm)
4. enter start x,y, and theta in mm and degrees
5. enter goal x,y in mm

dont's
a. do not input float values 
b. do not input low and high rpm as equal, the code will work but the bot won't reach the goal and it will be displayed as such 
c. only put values in the format in which they are asked 

test case considered in 2d representation video
clearance = 50
low rpm = 5
high rpm = 9
start x,y,theta = 500 1000 30 
goal x,y = 2000 1200


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
    
The terminal will ask for user input and enter values as shown below  
Test Case 1 :-
Enter clearance of robot in mm: 50
Enter the Low RPM: 5
Enter the High RPM: 9
Enter starting x coordintes, y coordinates and orientation seperated by spaces in mm 500 200 0
Enter Goal node x coordinte, y coordinate seperated by spaces in mm 4500 200


Test Case 2:
Enter clearance of robot in mm: 50
Enter the Low RPM: 5
Enter the High RPM: 9
Enter starting x coordintes, y coordinates and orientation seperated by spaces in mm: 500 200 0
Enter Goal node x coordinte, y coordinate seperated by spaces in mm: 5000 1850

      
