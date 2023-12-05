Name: DAKSH RAMESH CHAWLA


Social Human-Robot Interaction Package
This package contains four nodes that generate human info, object info, perceived info and robot info.

Installation
To install this package, download or clone it into your catkin workspace and place it in the "src" folder then run catkin_make in the workspace directory:
"cd ~/catkin_ws/"
"catkin_make"
"source devel/setup.sh"

Requirements
This package makes use of the "pgmpy" python library which will have to be installed in order to be imported. To install the "pgmpy" python package you will have to navigate into the home directory of your ROS environment and follow the steps below:
"source /opt/ros/noetic/setup.bash"
"pip install pgmpy"
These commands will install the pgmpy package in your ROS environment for python scripts to make use of it.

Usage
To use this package, launch the human_robot_interaction.launch file using roslaunch:
"roslaunch cr_week6_test human_robot_interaction.launch"
This will start the four nodes

To check the data published by the 4 topics run the following commands in a separate ROS terminal:
"rostopic echo <topic_name>"
Note <topic_name> has to be replaced by either humanInfo,objectInfo,perceivedInfo or robotInfo
