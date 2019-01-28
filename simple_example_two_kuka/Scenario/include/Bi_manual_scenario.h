/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef example_H_
#define example_H_

#include "RobotLib/RobotInterface.h"
#include "qp_ik_solver.h"
#include "sKinematics.h"
#include "mathlib_eigen_conversions.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
/*#include "sg_filter.h"*/
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Transform.h"
#include "sensor_msgs/JointState.h"


const int KUKA_DOF=7;
int IK_CONSTRAINTS=9;
double IK_ORIENTATIONCONTROLSTART=0.01;
double Gain_velocity_limit=50; //4 on the real robot
/*SGF::real sample_time = 0.002;*/
enum ENUM_AXIS{AXIS_X=0, AXIS_Y, AXIS_Z};
enum ENUM_PLANNER{PLANNER_CARTESIAN=0, PLANNER_JOINT,PLANNER_NONE};
enum ENUM_COMMAND{COMMAND_INITIAL=0,COMMAND_JOB,COMMAND_Move,COMMAND_STOP,COMMAND_Home,COMMAND_NONE};


int order = 3;
int winlen = 5;

const int N_robots=2;
const double dt=0.002;


using namespace Eigen;
enum ENUM_Robot{Left, Right};

class Bi_manual_scenario : public RobotInterface
{
public:
	Bi_manual_scenario();
	virtual ~Bi_manual_scenario();

	virtual Status              RobotInit();
	virtual Status              RobotFree();

	virtual Status              RobotStart();
	virtual Status              RobotStop();

	virtual Status              RobotUpdate();
	virtual Status              RobotUpdateCore();

	virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
private :

	void 						chatterCallback_state_joint(const sensor_msgs::JointState & msg);
	void 						chatterCallback_Desired_end_right(const geometry_msgs::Pose & msg);
	void 						chatterCallback_Desired_end_left(const geometry_msgs::Pose & msg);

	void						Send_Velocity_To_Robot(int index,VectorXd Velocity);


	void 						Topic_initialization();
	void 						Parameter_initialization();
	void						initKinematics(int index);
	void 						initKinematics();
	void						prepare_solver_IK(int index);
	void 						sendCommand(int _command);
	void						reset_the_bool();
	bool						everythingisreceived();
	void						pubish_on_tf(VectorXd  X,Quaternionf  Q,std::string n);
	void						prepare_jacobian(int index);





	bool						flag_init[2];
	bool						flag_job;
	bool						Position_of_the_robot_recieved[N_robots];

	pthread_t					PredictionThread;

	VectorXd					cJob[N_robots];
	VectorXd					cHome[N_robots];
	VectorXd					cDesired[N_robots];

	ros::Subscriber				Desired_end_right_sub;
	ros::Subscriber				Desired_end_left_sub;
	ros::Subscriber				armJointTrajectorysub;
	ros::Publisher 				pub_command_robot_real[N_robots];
	ros::Publisher 				pub_end_of_robot[N_robots];
	ros::Publisher 				armJointTrajectoryPublisher[2][7];
	ros::Subscriber 			sub_position_robot[N_robots];

	ros::Publisher 				pub_command;

	geometry_msgs::Pose			msg_robot_end;


	tf::TransformBroadcaster		*tf_br;
	tf::Transform 					tf_transform;
	tf::Quaternion 					tf_q;

	VectorXd 					JointPos [N_robots];
	VectorXd 					JointVel [N_robots];

	VectorXd 					Desired_JointVel[N_robots];
	VectorXd 					JointDesVel[N_robots];


	Matrix4d					T0[N_robots];

	Vector3d					DirX_End[N_robots];
	Vector3d					DirY_End[N_robots];
	Vector3d					DirZ_End[N_robots];
	Vector3d					Pos_End[N_robots];

	Eigen::Quaternionf 			rotation_right_temp;
	Eigen::Matrix3f 			rot_mat_right_temp;
	Eigen::Quaternionf 			rotation_left_temp;
	Eigen::Matrix3f 			rot_mat_left_temp;
	Eigen::Matrix3f 			rot_mat_temp;


	Jacobian_S					Jacobian_R[N_robots];



	Vector3d					Desired_DirY[N_robots];
	Vector3d					Desired_DirZ[N_robots];
	Vector3d					Desired_Pos_End[N_robots];
	VectorXd 					Desired_Velocity[N_robots];



	sKinematics                 *mSKinematicChain[N_robots];
	qp_ik_solver				*IK_Solver;



	MatrixXd					Jacobian3[N_robots];
	MatrixXd					Jacobian9[N_robots];
	MatrixXd					lJacobianDirY[N_robots];
	MatrixXd					lJacobianDirZ[N_robots];


	ENUM_COMMAND 				mCommand;
	ENUM_PLANNER 				mPlanner;



/*	SGF::SavitzkyGolayFilter 	*filter_pos_robot[N_robots];
	SGF::Vec 					inp;
	SGF::Vec 					outp;
	int 						ret_code;*/
};

string addTwostring(string string1, string string2, int integer1)
{
	std::string str;
	str.append(string1);
	str.append(string2);
	ostringstream convert;
	convert << integer1;
	str.append(convert.str());
	return str;
}


#endif 
