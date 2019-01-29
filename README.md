# YUMI_IK_Solver

"The read-me will be written!"

## Installation
1. clone repository 
2. go inside YUMI_IK_SOLVER fold
```
git clone git@github.com:epfl-lasa/YUMI_IK_Solver.git
roscd
cd ../src/YUMI_IK_SOLVER
git submodule init
git submodule update
cd ../
rospack profile
catkin_make
```

## Calibrate Yumi
Start calibration script.

ERROR:
Move the robot manually initial position (note markers on each robot joint) as seen in picture.
Run the calibration script again to let the robot fine calibration and initalization. 

## Start Yumi
1. Turn motors on (Button III)
2. Mode <Auto> (Button III)
3. Move pointer to start of script (Button I)

How to run the robot:
```
roscore
roslaunch yumi_launch yumi_vel_control.launch
```

Start robot simulator:
```
roscd
cd ../src/YUMI_IK_SOLVER/miscellaneous/robot-toolkit
./bin/robot_simulator --config packages/yumi_ik_solver/Scenario
```

How to use!



#### robot_simulator interface
Possible robot_simulator commands:

##### home 
Home position for calibration and before shutdown.

##### job
Job position (robot with arms elveated). Go to before starting move.

##### init
Init inverse kinematic solver before going to move

##### move
The robot is ready to recieve position commands published to:
```
/yumi/ee_pos_l
/yumi/ee_pos_r
```

##### stop
Stops any robot movement. Always stop the robot before turning off.


