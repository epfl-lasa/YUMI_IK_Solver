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

#include "Bi_manual_scenario.h"


/* The right robot is 1 and the left robot is 0*/
/*The right robot from the robot's point of view...Check the */
void Bi_manual_scenario::chatterCallback_Desired_end_right(const geometry_msgs::Pose & msg)
{
	Desired_Pos_End[1][0]=msg.position.x;
	Desired_Pos_End[1][1]=msg.position.y;
	Desired_Pos_End[1][2]=msg.position.z;
	rotation_right_temp.w() = msg.orientation.w;
	rotation_right_temp.x() = msg.orientation.x;
	rotation_right_temp.y() = msg.orientation.y;
	rotation_right_temp.z() = msg.orientation.z;
	rot_mat_right_temp = rotation_right_temp.toRotationMatrix();
	Desired_DirY[1](0)=rot_mat_right_temp(0,1); Desired_DirZ[1](0)=rot_mat_right_temp(0,2);
	Desired_DirY[1](1)=rot_mat_right_temp(1,1);	Desired_DirZ[1](1)=rot_mat_right_temp(1,2);
	Desired_DirY[1](2)=rot_mat_right_temp(2,1);	Desired_DirZ[1](2)=rot_mat_right_temp(2,2);
}

void Bi_manual_scenario::chatterCallback_Desired_end_left(const geometry_msgs::Pose & msg)
{
	Desired_Pos_End[0][0]=msg.position.x;
	Desired_Pos_End[0][1]=msg.position.y;
	Desired_Pos_End[0][2]=msg.position.z;
	rotation_left_temp.w() = msg.orientation.w;
	rotation_left_temp.x() = msg.orientation.x;
	rotation_left_temp.y() = msg.orientation.y;
	rotation_left_temp.z() = msg.orientation.z;
	rot_mat_left_temp = rotation_left_temp.toRotationMatrix();
	Desired_DirY[0](0)=rot_mat_left_temp(0,1);	Desired_DirZ[0](0)=rot_mat_left_temp(0,2);
	Desired_DirY[0](1)=rot_mat_left_temp(1,1);	Desired_DirZ[0](1)=rot_mat_left_temp(1,2);
	Desired_DirY[0](2)=rot_mat_left_temp(2,1);	Desired_DirZ[0](2)=rot_mat_left_temp(2,2);

}

void Bi_manual_scenario::chatterCallback_state_joint(const sensor_msgs::JointState & msg)
{
	if (msg.position.size()==14)
	{
		for (int i=0;i<2;i++)
		{
			JointPos[0](i)=msg.position[2*i];
			JointPos[1](i)=msg.position[2*i+1];
		}
		JointPos[0](2)=msg.position[12];
		JointPos[1](2)=msg.position[13];
		for (int i=3;i<7;i++)
		{
			JointPos[0](i)=msg.position[2*(i-1)];
			JointPos[1](i)=msg.position[2*(i-1)+1];
		}
	}
	else
	{
		cout<<"Something is wrong, the dimension of the robot joint state should be 14 and it is not!"<<endl;
	}
	Position_of_the_robot_recieved[0]=true;
	Position_of_the_robot_recieved[1]=true;
}


void Bi_manual_scenario::Send_Velocity_To_Robot(int index,VectorXd Velocity)
{
	std_msgs::Float64 desiredConfiguration[7];

	for (int i=0;i<2;i++)
	{
		desiredConfiguration[i].data =Velocity(i);
	}
	desiredConfiguration[6].data =Velocity(2);
	for (int i=3;i<7;i++)
	{
		desiredConfiguration[i-1].data =Velocity(i);
	}

	for (int i=0;i<7;i++)
	{
		armJointTrajectoryPublisher[index][i].publish(desiredConfiguration[i]);
	}

}


void Bi_manual_scenario::Topic_initialization()
{
	mRobot->SetControlMode(Robot::CTRLMODE_POSITION);
	ros::NodeHandle *n = mRobot->InitializeROS();


	Desired_end_right_sub = n->subscribe("/yumi/ee_pos_r", 3, & Bi_manual_scenario::chatterCallback_Desired_end_right,this);
	Desired_end_left_sub = n->subscribe("/yumi/ee_pos_l", 3, & Bi_manual_scenario::chatterCallback_Desired_end_left,this);



	armJointTrajectorysub = n->subscribe("/yumi/joint_states", 3, & Bi_manual_scenario::chatterCallback_state_joint,this);

	armJointTrajectoryPublisher[0][0] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_1_l/command", 1);
	armJointTrajectoryPublisher[0][1] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_2_l/command", 1);
	armJointTrajectoryPublisher[0][2] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_3_l/command", 1);
	armJointTrajectoryPublisher[0][3] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_4_l/command", 1);
	armJointTrajectoryPublisher[0][4] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_5_l/command", 1);
	armJointTrajectoryPublisher[0][5] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_6_l/command", 1);
	armJointTrajectoryPublisher[0][6] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_7_l/command", 1);

	armJointTrajectoryPublisher[1][0] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_1_r/command", 1);
	armJointTrajectoryPublisher[1][1] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_2_r/command", 1);
	armJointTrajectoryPublisher[1][2] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_3_r/command", 1);
	armJointTrajectoryPublisher[1][3] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_4_r/command", 1);
	armJointTrajectoryPublisher[1][4] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_5_r/command", 1);
	armJointTrajectoryPublisher[1][5] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_6_r/command", 1);
	armJointTrajectoryPublisher[1][6] = n->advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_7_r/command", 1);


	pub_end_of_robot[0]=n->advertise<geometry_msgs::Pose>("/robot/end/0", 3);
	pub_end_of_robot[1]=n->advertise<geometry_msgs::Pose>("/robot/end/1", 3);


	pub_command = n->advertise<std_msgs::Int64>("/command", 3);


	tf_br= new tf::TransformBroadcaster();

}
void Bi_manual_scenario::Parameter_initialization()
{

	for(int i=0;i<N_robots;i++)
	{
		JointVel[i].resize(KUKA_DOF);			JointVel[i].setZero();
		Desired_Velocity[i].resize(9);			Desired_Velocity[i].setZero();
		JointDesVel[i].resize(KUKA_DOF);		JointDesVel[i].setZero();
		//JointPos_mirror[i].resize(KUKA_DOF);	JointPos_mirror[i].setZero();
		cJob[i].resize(KUKA_DOF);
	}

	// Desired Target for the left
	Desired_DirY[0].setZero();
	Desired_DirZ[0].setZero();
	Desired_Pos_End[0].setZero();
	// Desired Target for the right
	Desired_DirY[1].setZero();
	Desired_DirZ[1].setZero();
	Desired_Pos_End[1].setZero();




	IK_Solver= new qp_ik_solver();

	// Initialize IK Solver without Self-Collision Avoidance
	IK_Solver->Initialize(N_robots,dt,Numerical,Velocity_level, true);


	cout<<"IK_Solver->Initialize is done"<<endl;

	cJob[1](0)=39.4*PI/180;	cJob[1](1)=-112*PI/180; cJob[0](2)=7.5*PI/180;	cJob[1](3)=48*PI/180;	cJob[1](4)=-43*PI/180;	cJob[1](5)=75*PI/180;	cJob[1](6)=-52*PI/180;
	cJob[0](0)=-40.0*PI/180;cJob[0](1)=-112*PI/180; cJob[1](2)=-8*PI/180;	cJob[0](3)=48*PI/180;	cJob[0](4)=34*PI/180;	cJob[0](5)=88*PI/180;	cJob[0](6)=73*PI/180;


	reset_the_bool();


}
void Bi_manual_scenario::initKinematics(int index)
{
	if (index==1) //right
	{
		mSKinematicChain[index] = new sKinematics(KUKA_DOF, dt);

		/*
		mSKinematicChain[index]->setDH(0,  0.0,  0.36, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0));
		mSKinematicChain[index]->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(98.0));
		mSKinematicChain[index]->setDH(2,  0.0,  0.42,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0));
		mSKinematicChain[index]->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(120.0));
		mSKinematicChain[index]->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0));
		mSKinematicChain[index]->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)); // reduced joint ang$
		//mSKinematicChain[index]->setDH(6,  0.0,  0.196, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		//	mSKinematicChain[index]->setDH(6,  0.0,  0.126+0.23, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
	//	mSKinematicChain[index]->setDH(6,  0.03,  0.126+0.14, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		mSKinematicChain[index]->setDH(6,  0.0,  0.1, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		 */

		mSKinematicChain[index]->setDH(0,  0.03,   0.11,   M_PI_2,  0.0,           1,  4*DEG2RAD(-168.5),  DEG2RAD(168.5), DEG2RAD(180.0)*0.90);
		mSKinematicChain[index]->setDH(1,  -0.03,  0.00,   -M_PI_2, 0.0,           1,  4*DEG2RAD(-143.5), DEG2RAD(43.5),  DEG2RAD(180.0)*0.90);
		mSKinematicChain[index]->setDH(2,  -0.04,  0.25,   M_PI_2,  0.0,           1,  DEG2RAD(-168.5),  DEG2RAD(168.5), DEG2RAD(180.0)*0.90);
		mSKinematicChain[index]->setDH(3,  0.04,   0.00,   -M_PI_2, DEG2RAD(90.0), 1,  DEG2RAD(-183.5),  DEG2RAD(180.),   DEG2RAD(180.0)*0.90);
		mSKinematicChain[index]->setDH(4,  -0.03,  0.26,   M_PI_2,  0.0,           1,  DEG2RAD(-290.),   DEG2RAD(290.),  DEG2RAD(400.0)*0.90);
		mSKinematicChain[index]->setDH(5,  0.03,   0.00,   -M_PI_2, 0.0,           1,  DEG2RAD(-88.),    DEG2RAD(138.),  DEG2RAD(400.0)*0.90); // reduced joint ang$
		mSKinematicChain[index]->setDH(6,  0.0,    0.23,   0.0, 	0.0,           1,  DEG2RAD(-229.),   DEG2RAD(229.),  DEG2RAD(400.0)*0.90); // reduced joint ang$
		//	mSKinematicChain[index]->setDH(6,  0.0,  0.1, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$

		T0[index].setZero();

		T0[index](0,0) = -0.5837 ;
		T0[index](0,1) =  0.1030 ;
		T0[index](0,2) =  0.8054 ;
		T0[index](0,3) = -0.0071 ;
		T0[index](1,0) = -0.6104 ;
		T0[index](1,1) = -0.7098 ;
		T0[index](1,2) = -0.3516 ;
		T0[index](1,3) = -0.0608 ;
		T0[index](2,0) =  0.5354 ;
		T0[index](2,1) = -0.6969 ;
		T0[index](2,2) =  0.4772 ;
		T0[index](2,3) =  0.3711 ;
		T0[index](3,3) =  1      ;
	}
	else if  (index==0) //left
	{
		mSKinematicChain[index] = new sKinematics(KUKA_DOF, dt);

		mSKinematicChain[index]->setDH(0,  0.03,   0.11,   M_PI_2,  0.0,           1,  4*DEG2RAD(-168.5),  DEG2RAD(168.5), DEG2RAD(180.0)*0.90);
		mSKinematicChain[index]->setDH(1,  -0.03,  0.00,   -M_PI_2, 0.0,           1,  4*DEG2RAD(-143.5), DEG2RAD(43.5),  DEG2RAD(180.0)*0.90);
		mSKinematicChain[index]->setDH(2,  -0.04,  0.25,   M_PI_2,  0.0,           1,  DEG2RAD(-168.5),  DEG2RAD(168.5), DEG2RAD(180.0)*0.90);
		mSKinematicChain[index]->setDH(3,  0.04,   0.00,   -M_PI_2, DEG2RAD(90.0), 1,  DEG2RAD(-183.5),  DEG2RAD(180.),   DEG2RAD(180.0)*0.90);
		mSKinematicChain[index]->setDH(4,  -0.03,  0.26,   M_PI_2,  0.0,           1,  DEG2RAD(-290.),   DEG2RAD(290.),  DEG2RAD(400.0)*0.90);
		mSKinematicChain[index]->setDH(5,  0.03,   0.00,   -M_PI_2, 0.0,           1,  DEG2RAD(-88.),    DEG2RAD(138.),  DEG2RAD(400.0)*0.90); // reduced joint ang$
		mSKinematicChain[index]->setDH(6,  0.0,    0.23,   0.0, 	0.0,           1,  DEG2RAD(-229.),   DEG2RAD(229.),  DEG2RAD(400.0)*0.90);

		T0[index].setZero();

		T0[index](0,0) = -0.5709 ;
		T0[index](0,1) = -0.1203 ;
		T0[index](0,2) =  0.8122 ;
		T0[index](0,3) =  0.0033 ;
		T0[index](1,0) =  0.6398 ;
		T0[index](1,1) = -0.6850 ;
		T0[index](1,2) =  0.3483 ;
		T0[index](1,3) =  0.0610 ;
		T0[index](2,0) =  0.5145 ;
		T0[index](2,1) =  0.7185 ;
		T0[index](2,2) =  0.4680 ;
		T0[index](2,3) =  0.3691 ;
		T0[index](3,3) =  1      ;
	}


	mSKinematicChain[index]->setT0(T0[index]);
	mSKinematicChain[index]->readyForKinematics();



	MatrixXd W(KUKA_DOF,KUKA_DOF); W.setZero();

	VectorXd U_p(KUKA_DOF);
	VectorXd L_p(KUKA_DOF);
	VectorXd U_Dp(KUKA_DOF);
	for(int i=0;i<KUKA_DOF;i++)
	{
		U_Dp(i)=Gain_velocity_limit*mSKinematicChain[index]->getMaxVel(i);
		U_p(i)=0.9*mSKinematicChain[index]->getMax(i);
		L_p(i)=0.9*mSKinematicChain[index]->getMin(i);
	}


	IK_Solver->Initialize_robot(index,KUKA_DOF,IK_CONSTRAINTS,W,U_p,L_p,U_Dp,-U_Dp);

	// variable for ik
	Jacobian3[index].resize(3,KUKA_DOF);
	lJacobianDirY[index].resize(3,KUKA_DOF);
	lJacobianDirZ[index].resize(3,KUKA_DOF);
	Jacobian9[index].resize(9,KUKA_DOF);

	/*	inp.resize(3);					outp.resize(3);*/
	RPos_End[index].setZero();
	/*
	filter_pos_robot[index]= new SGF::SavitzkyGolayFilter(3,order, winlen, sample_time);*/

	mSKinematicChain[index]->setJoints(JointPos[index].data());
	mSKinematicChain[index]->getEndPos(RPos_End[index]);
	msg_robot_end.position.x=RPos_End[index](0);
	msg_robot_end.position.y=RPos_End[index](1);
	msg_robot_end.position.z=RPos_End[index](2);
	pub_end_of_robot[index].publish(msg_robot_end);

	for (int i=0;i<7;i++)
	{
		Jacobian_R[index].Jacobian[i].resize(3,1+i);Jacobian_R[index].Jacobian[i].setZero();
		Jacobian_R[index].Jacobian_7[i].resize(3,7);Jacobian_R[index].Jacobian_7[i].setZero();
	}
}
void Bi_manual_scenario::prepare_sovlve_IK(int index)
{
	mSKinematicChain[index]->setJoints(JointPos[index].data());
	mSKinematicChain[index]->getEndPos(RPos_End[index]);
	mSKinematicChain[index]->getEndDirAxis(AXIS_X, lDirX[index]);
	mSKinematicChain[index]->getEndDirAxis(AXIS_Y, lDirY[index]);
	mSKinematicChain[index]->getEndDirAxis(AXIS_Z, lDirZ[index]);

	msg_robot_end.position.x=RPos_End[index](0);
	msg_robot_end.position.y=RPos_End[index](1);
	msg_robot_end.position.z=RPos_End[index](2);



	rot_mat_temp(0,0)=lDirX[index](0); 	rot_mat_temp(0,1)=lDirY[index](0); 	rot_mat_temp(0,2)=lDirZ[index](0);
	rot_mat_temp(1,0)=lDirX[index](1); 	rot_mat_temp(1,1)=lDirY[index](1); 	rot_mat_temp(1,2)=lDirZ[index](1);
	rot_mat_temp(2,0)=lDirX[index](2);	rot_mat_temp(2,1)=lDirY[index](2);	rot_mat_temp(2,2)=lDirZ[index](2);

	Eigen::Quaternionf rotation_temp(rot_mat_temp);

	msg_robot_end.orientation.w=rotation_temp.w();
	msg_robot_end.orientation.x=rotation_temp.x();
	msg_robot_end.orientation.y=rotation_temp.y();
	msg_robot_end.orientation.z=rotation_temp.z();

	pub_end_of_robot[index].publish(msg_robot_end);

	prepare_jacobian(index);

	mSKinematicChain[index]->getJacobianPos(Jacobian3[index]);
	mSKinematicChain[index]->getJacobianDirection(AXIS_Y, lJacobianDirY[index]);
	mSKinematicChain[index]->getJacobianDirection(AXIS_Z, lJacobianDirZ[index]);

	Jacobian9[index].block(0,0,3,KUKA_DOF)=Jacobian3[index];
	Jacobian9[index].block(3,0,3,KUKA_DOF)=lJacobianDirY[index];
	Jacobian9[index].block(6,0,3,KUKA_DOF)=lJacobianDirZ[index];


}
void Bi_manual_scenario::reset_the_bool()
{
	for (int i=0;i<N_robots;i++)
	{
		Position_of_the_robot_recieved[i]=false;
	}
}
bool Bi_manual_scenario::everythingisreceived()
{
	bool flag=true;

	/*	Must added once we ar going to move the robots*/

	for (int i=0;i<N_robots;i++)
	{

		if (Position_of_the_robot_recieved[i]==false)
		{
			cout<<"Position_of_the_robot_recieved[i] "<<i<<" "<<Position_of_the_robot_recieved[i]<<endl;
			flag=false;
		}
	}

	return flag;
}
void Bi_manual_scenario::pubish_on_tf(VectorXd  X,Quaterniond  Q,std::string n)
{
	tf_transform.setOrigin( tf::Vector3(X(0),X(1), X(2)) );
	tf_q.setX(Q.x());tf_q.setY(Q.y());tf_q.setZ(Q.z());tf_q.setW(Q.w());
	tf_transform.setRotation(tf_q);
	tf_br->sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "world_real", n));
}
void Bi_manual_scenario::prepare_jacobian(int index)
{
	for	(int i=0;i<7;i++)
	{

		mSKinematicChain[index]->getEndPos(i,Jacobian_R[index].Link_pos[i]);
		mSKinematicChain[index]->getJacobianPos_fast(i+1,Jacobian_R[index].Jacobian[i]);
		mSKinematicChain[index]->getJacobianPos(i+1,Jacobian_R[index].Jacobian_7[i]);
		/*		cout<<"Jacobian  of "<<index<<" robot. No "<< i<<"of jacobian"<<endl<<Jacobian_R[index].Jacobian[i]<<endl;
		cout<<"The full Jacobian  of "<<index<<" robot. No "<< i<<"of jacobian"<<endl<<Jacobian_R[index].Jacobian_7[i]<<endl;*/
	}
	Jacobian_R[index].Link_pos[0]=(Jacobian_R[index].Link_pos[0]+T0[index].block(0,3,3,1))/2;
	Jacobian_R[index].Link_pos[2]=(Jacobian_R[index].Link_pos[2]+Jacobian_R[index].Link_pos[1])/2;
	Jacobian_R[index].Link_pos[4]=(Jacobian_R[index].Link_pos[4]+Jacobian_R[index].Link_pos[3])/2;
	Jacobian_R[index].Link_pos[5]=(Jacobian_R[index].Link_pos[5]+Jacobian_R[index].Link_pos[6])/2;

	Jacobian_R[index].Jacobian[5]=Jacobian_R[index].Jacobian[6];
	Jacobian_R[index].Jacobian_7[5]=Jacobian_R[index].Jacobian_7[6];

}



Bi_manual_scenario::Bi_manual_scenario()
:RobotInterface(){
}
Bi_manual_scenario::~Bi_manual_scenario(){
}

RobotInterface::Status Bi_manual_scenario::RobotInit(){
	for(int i=0;i<N_robots;i++)
	{
		Desired_JointVel[i].resize(KUKA_DOF);	Desired_JointVel[i].setZero();
		JointPos[i].resize(KUKA_DOF);			JointPos[i].setZero();
	}

	Topic_initialization();
	Parameter_initialization();
	for (int i=0;i<N_robots;i++)
	{
		initKinematics(i);
	}

	IK_Solver->Finalize_Initialization();

	mPlanner=PLANNER_NONE;
	mCommand=COMMAND_NONE;

	flag_job=true;
	AddConsoleCommand("init");
	AddConsoleCommand("job");
	AddConsoleCommand("move");
	return STATUS_OK;
}
RobotInterface::Status Bi_manual_scenario::RobotFree(){

	for(int i=0;i<N_robots;i++)
	{
		Desired_JointVel[i].setZero();
		Send_Velocity_To_Robot(i,Desired_JointVel[i]);
	}
	return STATUS_OK;
}
RobotInterface::Status Bi_manual_scenario::RobotStart(){

	while (!everythingisreceived())
	{
		ros::spinOnce();
	}

	cout<<"JointPos_left"<<endl;cout<<JointPos[0]<<endl;
	cout<<"JointPos_right"<<endl;cout<<JointPos[1]<<endl;

	Topic_initialization();
	return STATUS_OK;
}    
RobotInterface::Status Bi_manual_scenario::RobotStop(){

	for(int i=0;i<N_robots;i++)
	{
		Desired_JointVel[i].setZero();
		Send_Velocity_To_Robot(i,Desired_JointVel[i]);
	}
	return STATUS_OK;
}
RobotInterface::Status Bi_manual_scenario::RobotUpdate(){
	ros::spinOnce();

	switch(mCommand){
	case COMMAND_INITIAL :
		if (!flag_init[0])
		{
			sendCommand(COMMAND_INITIAL);
			cout<<"Initialization"<<endl;
			Parameter_initialization();
			cout<<"Parameter_initialization is done"<<endl;
			for (int i=0;i<N_robots;i++)
			{
				initKinematics(i);
				Desired_JointVel[i].setZero();
				for(int i=0;i<N_robots;i++)
				{
					Send_Velocity_To_Robot(i,Desired_JointVel[i]);
				}
			}
			cout<<"initKinematics is done"<<endl;
			IK_Solver->Finalize_Initialization();
			cout<<"Finalize_Initialization is done"<<endl;
			ros::spinOnce();
			flag_init[0]=true;
			reset_the_bool();
		}
		if (everythingisreceived()&&!flag_init[1])
		{
			VectorXd handle;handle.resize(6); handle.setZero();
			for(int i=0;i<N_robots;i++)
			{
				prepare_sovlve_IK(i);
			}
			flag_init[1]=true;
			cout<<"Initialization finished"<<endl;
		}
		mPlanner=PLANNER_NONE;
		break;
	case COMMAND_JOB:
		sendCommand(COMMAND_JOB);
		for(int i=0;i<N_robots;i++)
		{
			prepare_jacobian(i);

			cout<<"RPos_End "<<i<<endl;cout<<RPos_End[i]<<endl;
		}
		mPlanner=PLANNER_JOINT;
		mCommand=COMMAND_NONE;
		break;
	case COMMAND_Move:
		sendCommand(COMMAND_Move);
		mPlanner=PLANNER_CARTESIAN;
		mCommand=COMMAND_NONE;
		break;
	}




	return STATUS_OK;
}
RobotInterface::Status Bi_manual_scenario::RobotUpdateCore(){

	switch(mPlanner){
	case PLANNER_CARTESIAN :

		for(int i=0;i<N_robots;i++)
		{
			prepare_sovlve_IK(i);
			// For closed loop
		}


		for(int i=0;i<N_robots;i++)
		{
			prepare_sovlve_IK(i);
			IK_Solver->set_jacobian_links(i,Jacobian_R[i]);
			IK_Solver->set_jacobian(i,Jacobian9[i]);

			Desired_Velocity[i].block(0,0,3,1)=(Desired_Pos_End[i].block(0,0,3,1)-RPos_End[i])/dt;
			Desired_Velocity[i].block(3,0,3,1)=(Desired_DirY[i]-lDirY[i])/(10*dt);
			Desired_Velocity[i].block(6,0,3,1)=(Desired_DirZ[i]-lDirZ[i])/(10*dt);
			IK_Solver->set_desired(i,Desired_Velocity[i]);
			IK_Solver->set_state(i,JointPos[i],JointVel[i]);
		}


		IK_Solver->Solve();


		for(int i=0;i<N_robots;i++)
		{
			IK_Solver->get_state(i,JointDesVel[i]);
			Desired_JointVel[i]=JointDesVel[i]*0.004;
			JointVel[i]=Desired_JointVel[i];
		}

		break;
	case PLANNER_JOINT:
		for(int i=0;i<N_robots;i++)
		{
			prepare_sovlve_IK(i);
			prepare_jacobian(i);
			Desired_JointVel[i]=0.2*(cJob[i]-JointPos[i]);
		}
		break;
	}


	for(int i=0;i<N_robots;i++)
	{
		Send_Velocity_To_Robot(i,Desired_JointVel[i]);
	}
	return STATUS_OK;
}
int Bi_manual_scenario::RespondToConsoleCommand(const string cmd, const vector<string> &args){
	if(cmd=="init"){
		if (!flag_job)
		{
			flag_init[0]=false;
			flag_init[1]=false;
			mPlanner=PLANNER_NONE;
			mCommand = COMMAND_INITIAL;
		}
	}
	else if(cmd=="job"){
		mCommand = COMMAND_JOB;
		mPlanner=PLANNER_NONE;
		flag_job=false;
	}
	else if(cmd=="move"){
		if (!flag_job)
		{
			cout<<"move!"<<endl;
			mCommand = COMMAND_Move;
			mPlanner=PLANNER_NONE;
			flag_job=true;
		}
	}
	return 0;
}



extern "C"{
// These two "C" functions manage the creation and destruction of the class
Bi_manual_scenario* create(){return new Bi_manual_scenario();}
void destroy(Bi_manual_scenario* module){delete module;}
}

