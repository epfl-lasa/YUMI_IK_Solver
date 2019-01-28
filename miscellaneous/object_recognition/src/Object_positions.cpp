#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int64.h"
#include "eigen3/Eigen/Dense"
#include "sg_filter.h"
#include <tf/transform_broadcaster.h>

ros::Publisher chatter_pub_object_7_target;
ros::Publisher chatter_pub_object_14_target;

using namespace std;
using namespace Eigen;


int order = 3;
int winlen = 20;
int order_vel = 3;
int winlen_vel = 100;
SGF::real sample_time = 0.002;
int dim=3;

Vector3d Position_7;
Vector3d Position_7_handle;
Vector3d Position_14_handle;
Vector3d Position_00;
Vector3d Position_01;
Vector3d Position_10;
Vector3d Position_11;
Vector3d Position_c;


Vector3d Position_7_target_1;
Vector3d Position_14_target_1;
Vector3d Position_7_target_2;
Vector3d Position_14_target_2;


Vector3d Position_00_old;
Vector3d Position_01_old;
Vector3d Position_10_old;
Vector3d Position_11_old;
Vector3d Position_c_old;
Vector3d Position_00_filter;
Vector3d Position_01_filter;
Vector3d Position_10_filter;
Vector3d Position_11_filter;
Vector3d Position_c_filter;
SGF::Vec inp_00(3);
SGF::Vec outp_00(3);
SGF::Vec inp_01(3);
SGF::Vec outp_01(3);
SGF::Vec inp_10(3);
SGF::Vec outp_10(3);
SGF::Vec inp_11(3);
SGF::Vec outp_11(3);
SGF::Vec inp_c(3);
SGF::Vec outp_c(3);

SGF::Vec inp_objec_7_1(3);
SGF::Vec outp_objec_7_1(3);
SGF::Vec inp_objec_7_2(3);
SGF::Vec outp_objec_7_2(3);

SGF::Vec inp_objec_14_1(3);
SGF::Vec outp_objec_14_1(3);
SGF::Vec inp_objec_14_2(3);
SGF::Vec outp_objec_14_2(3);


SGF::Vec inp(3);
SGF::Vec outp(3);

Vector3d Orientation_grasp_y_handle;
Vector3d Orientation_grasp_x[2];
Vector3d Orientation_grasp_y[2];
Vector3d Orientation_grasp_z[2];

Vector3d Orientation_grasp_x_o;
Vector3d Orientation_grasp_y_o;
Vector3d Orientation_grasp_z_o;

SGF::SavitzkyGolayFilter *filter_00;
SGF::SavitzkyGolayFilter *filter_01;
SGF::SavitzkyGolayFilter *filter_10;
SGF::SavitzkyGolayFilter *filter_11;
SGF::SavitzkyGolayFilter *filter_c;
SGF::SavitzkyGolayFilter *filter;
SGF::SavitzkyGolayFilter *filter_vel;

SGF::SavitzkyGolayFilter *filter_object_7_1;
SGF::SavitzkyGolayFilter *filter_object_14_1;
SGF::SavitzkyGolayFilter *filter_object_7_2;
SGF::SavitzkyGolayFilter *filter_object_14_2;


tf::TransformBroadcaster		*tf_br;
tf::Transform 					tf_transform;
tf::Quaternion 					tf_q;

geometry_msgs::Pose msg_p1;
geometry_msgs::Pose msg_p0;

int ret_code;


int Target_selector;
double time_squence[2];

Matrix3d mat_orien[2];


void position_00(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	Position_00(0)=msg->pose.position.y-Position_7(0);
	Position_00(1)=-msg->pose.position.x-Position_7(1);
	Position_00(2)=msg->pose.position.z-Position_7(2);
	if ((Position_00_old-Position_00).norm()==0)
	{
		cout<<"The position of the marker /KUKA_7_object_1/pose is lost"<<endl;
	}
//	cout<<"Position_00 "<<Position_00<<" Position_7 "<<Position_7<<endl;
	Position_00_old=Position_00;
	inp_00(0)=Position_00(0);
	inp_00(1)=Position_00(1);
	inp_00(2)=Position_00(2);
	ret_code = filter_00->AddData(inp_00);
	ret_code = filter_00->GetOutput(0, outp_00);
	Position_00_filter(0)=outp_00(0);
	Position_00_filter(1)=outp_00(1);
	Position_00_filter(2)=outp_00(2);

}

void position_01(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	Position_01(0)=msg->pose.position.y-Position_7(0);
	Position_01(1)=-msg->pose.position.x-Position_7(1);
	Position_01(2)=msg->pose.position.z-Position_7(2);
	if ((Position_01_old-Position_01).norm()==0)
	{
		cout<<"The position of the marker /KUKA_7_object_2/pose is lost"<<endl;
	}
	Position_01_old=Position_01;

	inp_01(0)=Position_01(0);
	inp_01(1)=Position_01(1);
	inp_01(2)=Position_01(2);
	ret_code = filter_01->AddData(inp_01);
	ret_code = filter_01->GetOutput(0, outp_01);
	Position_01_filter(0)=outp_01(0);
	Position_01_filter(1)=outp_01(1);
	Position_01_filter(2)=outp_01(2);
}

void position_10(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	Position_10(0)=msg->pose.position.y-Position_7(0);
	Position_10(1)=-msg->pose.position.x-Position_7(1);
	Position_10(2)=msg->pose.position.z-Position_7(2);
	if ((Position_10_old-Position_10).norm()==0)
	{
		cout<<"The position of the marker /KUKA_14_object_1/pose is lost"<<endl;
	}
	Position_10_old=Position_10;

	inp_10(0)=Position_10(0);
	inp_10(1)=Position_10(1);
	inp_10(2)=Position_10(2);
	ret_code = filter_10->AddData(inp_10);
	ret_code = filter_10->GetOutput(0, outp_10);
	Position_10_filter(0)=outp_10(0);
	Position_10_filter(1)=outp_10(1);
	Position_10_filter(2)=outp_10(2);
}

void position_11(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	Position_11(0)=msg->pose.position.y-Position_7(0);
	Position_11(1)=-msg->pose.position.x-Position_7(1);
	Position_11(2)=msg->pose.position.z-Position_7(2);
	if ((Position_11_old-Position_11).norm()==0)
	{
		cout<<"The position of the marker /KUKA_14_object_2/pose is lost"<<endl;
	}
	Position_11_old=Position_11;

	inp_11(0)=Position_11(0);
	inp_11(1)=Position_11(1);
	inp_11(2)=Position_11(2);
	ret_code = filter_11->AddData(inp_11);
	ret_code = filter_11->GetOutput(0, outp_11);
	Position_11_filter(0)=outp_11(0);
	Position_11_filter(1)=outp_11(1);
	Position_11_filter(2)=outp_11(2);
}

void position_c(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	Position_c(0)=msg->pose.position.y-Position_7(0);
	Position_c(1)=-msg->pose.position.x-Position_7(1);
	Position_c(2)=msg->pose.position.z-Position_7(2);

	if ((Position_c_old-Position_c).norm()==0)
	{
		cout<<"The position of the marker /center/pose is lost"<<endl;
	}
	Position_c_old=Position_c;

	inp_c(0)=Position_c(0);
	inp_c(1)=Position_c(1);
	inp_c(2)=Position_c(2);
	ret_code = filter_c->AddData(inp_c);
	ret_code = filter_c->GetOutput(0, outp_c);
	Position_c_filter(0)=outp_c(0);
	Position_c_filter(1)=outp_c(1);
	Position_c_filter(2)=outp_c(2);

}


void position_7(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	/*	X_new=Y_vision
	Z_new=Z_vision
	Y_new=-X_vision
	 */
	if ((Position_7_handle.norm()==0))
	{
		Position_7_handle(0)=msg->pose.position.y;
		Position_7_handle(1)=-msg->pose.position.x;
		Position_7_handle(2)=msg->pose.position.z;

	}

}


void position_14(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	/*	X_new=Y_vision
	Z_new=Z_vision
	Y_new=-X_vision
	 */
	if ((Position_14_handle.norm()==0))
	{
		Position_14_handle(0)=msg->pose.position.y;
		Position_14_handle(1)=-msg->pose.position.x;
		Position_14_handle(2)=msg->pose.position.z;

	}

}


void position_7_target_1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	/*	X_new=Y_vision
	Z_new=Z_vision
	Y_new=-X_vision
	 */

/*	inp_objec_7_1(0)=msg->pose.position.y-Position_7(0);
	inp_objec_7_1(1)=-msg->pose.position.x-Position_7(1);
	inp_objec_7_1(2)=msg->pose.position.z-Position_7(2);



	ret_code = filter_object_7_1->AddData(inp_objec_7_1);
	ret_code = filter_object_7_1->GetOutput(0, outp_objec_7_1);

	Position_7_target_1(0)=outp_objec_7_1(0);
	Position_7_target_1(1)=outp_objec_7_1(1);
	Position_7_target_1(2)=outp_objec_7_1(2);*/


//	cout<<"Position_7_target_1 "<<Position_7_target_1<<endl;
	//	chatter_pub_object_7_target.publish(msg_p0);
}

void position_14_target_1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	/*	X_new=Y_vision
	Z_new=Z_vision
	Y_new=-X_vision
	 */

/*
	inp_objec_14_1(0)=msg->pose.position.y-Position_7(0);
	inp_objec_14_1(1)=-msg->pose.position.x-Position_7(1);
	inp_objec_14_1(2)=msg->pose.position.z-Position_7(2);

	ret_code = filter_object_14_1->AddData(inp_objec_14_1);
	ret_code = filter_object_14_1->GetOutput(0, outp_objec_14_1);
*/

/*

	Position_14_target_1(0)=outp_objec_14_1(0);
	Position_14_target_1(1)=outp_objec_14_1(1);
	Position_14_target_1(2)=outp_objec_14_1(2);
*/



	//cout<<"Position_14_target_1 "<<Position_14_target_1<<endl;
	//	chatter_pub_object_14_target.publish(msg_p1);
}


void position_7_target_2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	/*	X_new=Y_vision
	Z_new=Z_vision
	Y_new=-X_vision
	 */

/*	inp_objec_7_2(0)=msg->pose.position.y-Position_7(0);
	inp_objec_7_2(1)=-msg->pose.position.x-Position_7(1);
	inp_objec_7_2(2)=msg->pose.position.z-Position_7(2);

	ret_code = filter_object_7_2->AddData(inp_objec_7_2);
	ret_code = filter_object_7_2->GetOutput(0, outp_objec_7_2);

	Position_7_target_2(0)=outp_objec_7_2(0);
	Position_7_target_2(1)=outp_objec_7_2(1);
	Position_7_target_2(2)=outp_objec_7_2(2);

	cout<<"Position_7_target_2 "<<Position_7_target_2<<endl;*/
	//	chatter_pub_object_7_target.publish(msg_p0);


}

void position_14_target_2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	/*	X_new=Y_vision
	Z_new=Z_vision
	Y_new=-X_vision
	 */

/*
	inp_objec_14_2(0)=msg->pose.position.y-Position_7(0);
	inp_objec_14_2(1)=-msg->pose.position.x-Position_7(1);
	inp_objec_14_2(2)=msg->pose.position.z-Position_7(2);

	ret_code = filter_object_14_2->AddData(inp_objec_14_2);
	ret_code = filter_object_14_2->GetOutput(0, outp_objec_14_2);


	Position_14_target_2(0)=outp_objec_14_2(0);
	Position_14_target_2(1)=outp_objec_14_2(1);
	Position_14_target_2(2)=outp_objec_14_2(2);
*/



//	cout<<"Position_14_target_2 "<<Position_14_target_2<<endl;
	//chatter_pub_object_14_target.publish(msg_p1);
}


void position_target_selector(const std_msgs::Int64::ConstPtr& msg)
{

	Target_selector=msg->data;

	if (Target_selector==71)
	{
		time_squence[1]=ros::Time::now().toSec();

	} else if (Target_selector==72)
	{
		time_squence[1]=ros::Time::now().toSec();

	}else if (Target_selector==141)
	{
		time_squence[0]=ros::Time::now().toSec();

	}else if (Target_selector==142)
	{
		time_squence[0]=ros::Time::now().toSec();
	}
}

void Target_publisher()
{
	mat_orien[0].setZero();
	Position_14_target_2(0)=-0.723128;
	Position_14_target_2(1)=-1.31853;
	Position_14_target_2(2)=-0.781618;
	Position_7_target_2(0)=-0.869967;
	Position_7_target_2(1)=0.0393236;
	Position_7_target_2(2)=-0.764838;

	Position_14_target_1(0)=-0.155164;
	Position_14_target_1(1)=-0.591975;
	Position_14_target_1(2)= 0.543482;
	Position_7_target_1(0)=0.180318;
	Position_7_target_1(1)=-0.526866;
	Position_7_target_1(2)= 0.607569;
	if (Target_selector==71)
	{
		if ((ros::Time::now().toSec()-time_squence[1])<5)
		{
			cout<<"Set_first_sequence motion for 7"<<endl;
			mat_orien[0].setZero();
			mat_orien[0](0,0)=1;mat_orien[0](2,1)=1;mat_orien[0](1,2)=-1;
			Quaterniond q_orien(mat_orien[0]);
			msg_p1.position.x=Position_14_target_1(0)-0.25;
			msg_p1.position.y=Position_14_target_1(1);
			msg_p1.position.z=Position_14_target_1(2)+0.35;
			msg_p1.orientation.x=q_orien.x();
			msg_p1.orientation.y=q_orien.y();
			msg_p1.orientation.z=q_orien.z();
			msg_p1.orientation.w=q_orien.w();
			chatter_pub_object_7_target.publish(msg_p1);

		}
		else
		{
			mat_orien[0].setZero();
			mat_orien[0](0,0)=1;mat_orien[0](2,1)=1;mat_orien[0](1,2)=-1;
			Quaterniond q_orien(mat_orien[0]);
			msg_p1.position.x=Position_7_target_1(0)-0.05;
			msg_p1.position.y=Position_7_target_1(1);
			msg_p1.position.z=Position_7_target_1(2)+0.05;
			msg_p1.orientation.x=q_orien.x();
			msg_p1.orientation.y=q_orien.y();
			msg_p1.orientation.z=q_orien.z();
			msg_p1.orientation.w=q_orien.w();
			chatter_pub_object_7_target.publish(msg_p1);
		}

	} else if (Target_selector==72)
	{

		mat_orien[0].setZero();
		mat_orien[0](2,0)=1;mat_orien[0](1,1)=1;mat_orien[0](0,2)=-1;
		Quaterniond q_orien(mat_orien[0]);
		msg_p1.position.x=Position_7_target_2(0);
		msg_p1.position.y=Position_7_target_2(1);
		msg_p1.position.z=Position_7_target_2(2)+1.20;
		msg_p1.orientation.x=q_orien.x();
		msg_p1.orientation.y=q_orien.y();
		msg_p1.orientation.z=q_orien.z();
		msg_p1.orientation.w=q_orien.w();
		chatter_pub_object_7_target.publish(msg_p1);


	}else if (Target_selector==141)
	{
		if ((ros::Time::now().toSec()-time_squence[0])<5)
		{
			cout<<"Set_first_sequence motion for 14"<<endl;
			mat_orien[1].setZero();
			mat_orien[1](0,0)=1;mat_orien[1](2,1)=-1;mat_orien[1](1,2)=1;
			Quaterniond q_orien(mat_orien[1]);
			msg_p1.position.x=Position_14_target_1(0);
			msg_p1.position.y=Position_14_target_1(1)-0.2;
			msg_p1.position.z=Position_14_target_1(2)+0.12;
			msg_p1.orientation.x=q_orien.x();
			msg_p1.orientation.y=q_orien.y();
			msg_p1.orientation.z=q_orien.z();
			msg_p1.orientation.w=q_orien.w();
			chatter_pub_object_14_target.publish(msg_p1);
		}
		else
		{
			mat_orien[1].setZero();
			mat_orien[1](0,0)=1;mat_orien[1](2,1)=-1;mat_orien[1](1,2)=1;
			Quaterniond q_orien(mat_orien[1]);
			msg_p1.position.x=Position_14_target_1(0);
			msg_p1.position.y=Position_14_target_1(1)-0.12;
			msg_p1.position.z=Position_14_target_1(2)+0.12;
			msg_p1.orientation.x=q_orien.x();
			msg_p1.orientation.y=q_orien.y();
			msg_p1.orientation.z=q_orien.z();
			msg_p1.orientation.w=q_orien.w();
			chatter_pub_object_14_target.publish(msg_p1);
		}
	}else if (Target_selector==142)
	{
		mat_orien[1].setZero();
		mat_orien[1](2,0)=1;mat_orien[1](1,1)=1;mat_orien[1](0,2)=-1;
		Quaterniond q_orien(mat_orien[1]);
		msg_p1.position.x=Position_14_target_2(0);
		msg_p1.position.y=Position_14_target_2(1);
		msg_p1.position.z=Position_14_target_2(2)+1.20;
		msg_p1.orientation.x=q_orien.x();
		msg_p1.orientation.y=q_orien.y();
		msg_p1.orientation.z=q_orien.z();
		msg_p1.orientation.w=q_orien.w();
		chatter_pub_object_14_target.publish(msg_p1);
	}
}


void pubish_on_tf(VectorXd  X)
{
	tf_transform.setOrigin( tf::Vector3(X(0),X(1), X(2)) );
	tf_q.setX(0);tf_q.setY(0);tf_q.setZ(0);tf_q.setW(1);
	tf_transform.setRotation(tf_q);
	tf_br->sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "world", "world_real"));
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "object_rec");
	tf_br= new tf::TransformBroadcaster();

	filter_00= new SGF::SavitzkyGolayFilter(dim,order, winlen, sample_time);
	filter_01= new SGF::SavitzkyGolayFilter(dim,order, winlen, sample_time);
	filter_10= new SGF::SavitzkyGolayFilter(dim,order, winlen, sample_time);
	filter_11= new SGF::SavitzkyGolayFilter(dim,order, winlen, sample_time);
	filter_c = new SGF::SavitzkyGolayFilter(dim,order, winlen, sample_time);
	filter	 = new SGF::SavitzkyGolayFilter(dim,order, winlen, sample_time);
	filter_vel= new SGF::SavitzkyGolayFilter(dim,order_vel, winlen_vel, sample_time);

	filter_object_7_1= new SGF::SavitzkyGolayFilter(dim,order_vel, winlen_vel, sample_time);
	filter_object_14_1= new SGF::SavitzkyGolayFilter(dim,order_vel, winlen_vel, sample_time);
	filter_object_7_2= new SGF::SavitzkyGolayFilter(dim,order_vel, winlen_vel, sample_time);
	filter_object_14_2= new SGF::SavitzkyGolayFilter(dim,order_vel, winlen_vel, sample_time);


	ros::NodeHandle n;
	ros::Subscriber sub[2][2]; //  The second one is important which [0][0] means the lower one and [0](1) indicates the higher one.
	ros::Subscriber sub_c_target; //

	ros::Subscriber sub_c; //

	ros::Subscriber sub_taget[2][2]; //  The second one is the target and the first one is for the robot

	ros::Publisher chatter_pub_taget[2];

	ros::Publisher chatter_pub_f;
	ros::Publisher chatter_pub_vel_f;
	ros::Publisher chatter_pub_acc_f;

	ros::Publisher chatter_pub;
	ros::Publisher chatter_pub_object_left;
	ros::Publisher chatter_pub_object_right;

	ros::Publisher chatter_pub_object_left_f;
	ros::Publisher chatter_pub_object_right_f;




	geometry_msgs::Pose Object_f;
	geometry_msgs::Pose Object_vel_f;
	geometry_msgs::Pose Object_acc_f;

	geometry_msgs::Pose Object;
	geometry_msgs::Pose Object_left;
	geometry_msgs::Pose Object_right;


	sub[0][0] = n.subscribe("/KUKA_7_object_1/pose", 3, position_00);
	sub[0][1] = n.subscribe("/KUKA_7_object_2/pose", 3, position_01);

	sub[1][0] = n.subscribe("/KUKA_14_object_1/pose", 3, position_10);
	sub[1][1] = n.subscribe("/KUKA_14_object_2/pose", 3, position_11);

	sub_taget[0][0] =n.subscribe("/KUKA7_target_1/pose", 3, position_7_target_1);
	sub_taget[0][1] =n.subscribe("/KUKA14_target_1/pose", 3, position_14_target_1);
	sub_taget[1][0] =n.subscribe("/KUKA7_target_2/pose", 3, position_7_target_2);
	sub_taget[1][1] =n.subscribe("/KUKA14_target_2/pose", 3, position_14_target_2);

	sub_c_target =n.subscribe("/KUKA/Target/selector", 3, position_target_selector);



	chatter_pub_object_7_target = n.advertise<geometry_msgs::Pose>("/object/KUKA7_target/position", 3);
	chatter_pub_object_14_target = n.advertise<geometry_msgs::Pose>("/object/KUKA14_target/position", 3);

	sub_c = n.subscribe("/center/pose", 3, position_c);

	ros::Subscriber sub_7 = n.subscribe("/KUKA_7/pose", 3, position_7);
	ros::Subscriber sub_14 = n.subscribe("/KUKA_14/pose", 3, position_14);

	chatter_pub = n.advertise<geometry_msgs::Pose>("/object/raw/position", 3);



	/*chatter_pub_object_left = n.advertise<geometry_msgs::Pose>("/object/raw/right/position", 3);
	chatter_pub_object_right = n.advertise<geometry_msgs::Pose>("/object/raw/left/position", 3);*/

	chatter_pub_object_left_f = n.advertise<geometry_msgs::Pose>("/object/filtered/right/position", 3);
	chatter_pub_object_right_f = n.advertise<geometry_msgs::Pose>("/object/filtered/left/position", 3);



	chatter_pub_f = n.advertise<geometry_msgs::Pose>("/object/filtered/position", 3);
	chatter_pub_vel_f = n.advertise<geometry_msgs::Pose>("/object/filtered/velocity", 3);
	chatter_pub_acc_f = n.advertise<geometry_msgs::Pose>("/object/filtered/acceleration", 3);


	Position_7.setZero();


	while ((Position_7_handle.norm()==0)||(Position_14_handle.norm()==0))
	{
		ros::spinOnce();
		Position_7=(Position_7_handle+Position_14_handle)/2;
	}
	cout<<"Position_7 "<<Position_7<<" Position_7_handle "<<Position_7_handle<<" Position_14_handle "<<Position_14_handle<<endl;
	ros::Rate r(240);

	Target_selector=0;
/*
	Quaterniond  Q;
	Q.x=0;
	Q.y=0;
	Q.z=0;
	Q.w=1;
*/

	while (ros::ok())
	{

		ros::spinOnce();
	//	Position_7=(Position_7_handle+Position_14_handle)/2;
		 pubish_on_tf(Position_7);

		Orientation_grasp_x[0]=(Position_01_filter-Position_00_filter).normalized();
		Orientation_grasp_x[1]=(Position_11_filter-Position_10_filter).normalized();


/*		cout<<"Position_c "<<Position_c<<endl;
		cout<<"Position_11 "<<Position_11<<endl;
		cout<<"Position_01 "<<Position_01<<endl;
		cout<<"(Position_01+Position_11)/2 "<<(Position_11+Position_01)/2<<endl;*/

		Orientation_grasp_y_handle=(Position_c_filter-(Position_01_filter+Position_11_filter)/2).normalized();

		Orientation_grasp_y[0]=Orientation_grasp_y_handle-(Orientation_grasp_y_handle.dot(Orientation_grasp_x[0]))*Orientation_grasp_x[0];
		Orientation_grasp_y[0].normalize();
		Orientation_grasp_y[1]=Orientation_grasp_y_handle-(Orientation_grasp_y_handle.dot(Orientation_grasp_x[1]))*Orientation_grasp_x[1];
		Orientation_grasp_y[1].normalize();

		Orientation_grasp_z[0]=Orientation_grasp_x[0].cross(Orientation_grasp_y[0]);
		Orientation_grasp_z[1]=Orientation_grasp_x[1].cross(Orientation_grasp_y[1]);



		Object.position.x=(Position_00(0)+Position_01(0)+Position_10(0)+Position_11(0))/4;
		Object.position.y=(Position_00(1)+Position_01(1)+Position_10(1)+Position_11(1))/4;
		Object.position.z=(Position_00(2)+Position_01(2)+Position_10(2)+Position_11(2))/4;

		inp(0) = Object.position.x;
		inp(1) = Object.position.y;
		inp(2) = Object.position.z;
		ret_code = filter->AddData(inp);
		ret_code = filter->GetOutput(0, outp);
		Object_f.position.x=outp(0);					Object_f.position.y=outp(1);					Object_f.position.z=outp(2);
		ret_code = filter->GetOutput(1, outp);
		ret_code = filter_vel->AddData(outp);
		ret_code = filter_vel->GetOutput(0, outp);
		Object_vel_f.position.x=outp(0);				Object_vel_f.position.y=outp(1);				Object_vel_f.position.z=outp(2);
		ret_code = filter_vel->GetOutput(1, outp);
		Object_acc_f.position.x=outp(0);				Object_acc_f.position.y=outp(1);				Object_acc_f.position.z=outp(2);


		Orientation_grasp_x_o=(Position_01-Position_11).normalized();
		Matrix3d mat;
		mat.block(0,0,3,1)=Orientation_grasp_x_o;
		Orientation_grasp_y_o=Orientation_grasp_y_handle-(Orientation_grasp_y_handle.dot(Orientation_grasp_x_o))*Orientation_grasp_x_o;
		mat.block(0,1,3,1)=Orientation_grasp_y_o;
		Orientation_grasp_z_o=Orientation_grasp_x_o.cross(Orientation_grasp_y_o);
		mat.block(0,2,3,1)=Orientation_grasp_z_o;
		Quaterniond q(mat);
		Object.orientation.x=q.x();
		Object.orientation.y=q.y();
		Object.orientation.z=q.z();
		Object.orientation.w=q.w();
		Object_f.orientation.x=q.x();		Object_f.orientation.y=q.y();		Object_f.orientation.z=q.z();
		Object_f.orientation.w=q.w();
		/*		Object.orientation.x=0;
		Object.orientation.y=0;
		Object.orientation.z=0;
		Object.orientation.w=1;*/

		Object_left.position.x=(Position_00_filter(0)+Position_01_filter(0))/2;
		Object_left.position.y=(Position_00_filter(1)+Position_01_filter(1))/2;
		Object_left.position.z=(Position_00_filter(2)+Position_01_filter(2))/2;
/*		mat.block(0,0,3,1)=Orientation_grasp_x[0];
		mat.block(0,1,3,1)=Orientation_grasp_y[0];
		mat.block(0,2,3,1)=Orientation_grasp_z[0];
		Quaterniond q1(mat);*/
		Object_left.orientation.x=q.x();
		Object_left.orientation.y=q.y();
		Object_left.orientation.z=q.z();
		Object_left.orientation.w=q.w();
		/*		Object_left.orientation.x=0;
		Object_left.orientation.y=0;
		Object_left.orientation.z=0;
		Object_left.orientation.w=1;*/

		Object_right.position.x=(Position_10_filter(0)+Position_11_filter(0))/2;
		Object_right.position.y=(Position_10_filter(1)+Position_11_filter(1))/2;
		Object_right.position.z=(Position_10_filter(2)+Position_11_filter(2))/2;
/*		mat.block(0,0,3,1)=Orientation_grasp_x[0];
		mat.block(0,1,3,1)=Orientation_grasp_y[0];
		mat.block(0,2,3,1)=Orientation_grasp_z[0];
		Quaterniond q2(mat);*/
		Object_right.orientation.x=q.x();
		Object_right.orientation.y=q.y();
		Object_right.orientation.z=q.z();
		Object_right.orientation.w=q.w();
		/*		Object_right.orientation.x=0;
		Object_right.orientation.y=0;
		Object_right.orientation.z=0;
		Object_right.orientation.w=1;*/


		Target_publisher();

		chatter_pub_f.publish(Object_f);
		chatter_pub_vel_f.publish(Object_vel_f);
		chatter_pub_acc_f.publish(Object_acc_f);
		chatter_pub.publish(Object);
		chatter_pub_object_left_f.publish(Object_left);
		chatter_pub_object_right_f.publish(Object_right);

		ros::spinOnce();
		r.sleep();
	}



	return 0;

}
