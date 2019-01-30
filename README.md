This package provides a higher level as well as low level interface to YuMi ABB. You can send the joint space positions or you can directly send the end-effector position and orientation.

"The read-me will be written!"
#  Dependencies
Eigen http://eigen.tuxfamily.org/index.php?title=Main_Page

Nlopt http://ab-initio.mit.edu/wiki/index.php/NLopt

CVXGEN http://cvxgen.com/docs/index.html **(In this package, Cvxgen for two 7 DOF robots has already been included. If you want to use this package for other robots, you just need to create new CVxgen files.)**

The rest of the required dependences are included in this package as git submodules.  

Note: Make sure that you have set-up the ssh option for github on your PC!

# Features:

- Joint space velocity level controller. 
- End-effector Pos/Orientation interface

# How to set-up the Robot

Note: You need to have set-up the Rapid EGM modules on Yumi. Follow this https://github.com/kth-ros-pkg/yumi/wiki. EGM modules are not available on-line and one needs to buy the license from ABB. If you are struggling with this, you can ask [Bernardo Fichera](http://lasa.epfl.ch/people/member.php?SCIPER=292897)

# How to set-up the packge

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

# How to run

Run the robot:

In Termianl 1:
```
roscore
```
In Termianl 2:
```
roslaunch yumi_launch yumi_vel_control.launch
```
In Termianl 3; start robot simulator:
```
roscd
cd ../src/YUMI_IK_SOLVER/miscellaneous/robot-toolkit
./bin/robot_simulator --config packages/yumi_ik_solver/Scenario
```

# How to use:

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
For more information contact [Sina Mirrazavi](http://lasa.epfl.ch/people/member.php?SCIPER=233855) or [Lukas Huber](http://lasa.epfl.ch/people/member.php?SCIPER=274454).

