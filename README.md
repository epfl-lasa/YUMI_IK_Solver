This package provides a higher level as well as low level interface to YuMi ABB. You can send the joint space positions or you can directly send the end-effector position and orientation.

"The read-me will be written!"

Note: Make sure that you have set-up the ssh option for github on your PC!
# How to set-up the package

Note: You need to have set-up the Rapid EGM modules on Yumi. Follow this https://github.com/kth-ros-pkg/yumi/wiki. EGM modules are not available on-line and one needs to buy the license from ABB. 

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
Run the robot:
In Termianl 1:
```
roscore
```
In Termianl 2:
```
roslaunch yumi_launch yumi_vel_control.launch
```
In Termianl 3:
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

## Copyright
Please cite these paper if you are using this toolbox:
@article{mirrazavi2018unified,
  title={A unified framework for coordinated multi-arm motion planning},
  author={Mirrazavi Salehian, Seyed Sina and Figueroa, Nadia and Billard, Aude},
  journal={The International Journal of Robotics Research},
  pages={0278364918765952},
  publisher={SAGE Publications Sage UK: London, England}
}
More information regarding the IK solver is available in this paper. 
For more information contact Sina Mirrazavi or Lukas Huber.

