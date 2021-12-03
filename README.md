# Intelligent-robotics-simulation
Basic simulation for the intellegient robotics assignment.
Simulation with turtlebot3 in cafe with 4 tables.

You will need the tutlebot3 repository downloaded.
Do this run this command(i think) in your workspace repository folder:

go to your catkin_ws/src

git clone this repository..having the my_simuluations package

then you will need turtle bot tpoo
in the same directory catkin_ws 


`git clone https://github.com/ROBOTIS-GIT/turtlebot3.git`

similarly clone the turtlebot3 msgs repository in the same directory.

then go back to catkin_ws and run catkin_make.


To launch the simulation run:

`roslaunch my_simluations my_world.launch`

To launch queue server python3 ./src/my_simluations/scripts/queue_server.py

launch the ui.py file from src directory similarly.
