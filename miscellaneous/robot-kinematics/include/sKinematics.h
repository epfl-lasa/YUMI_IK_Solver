#ifndef __sKinematics_H__
#define __sKinematics_H__

#include <mathlib/MathLib.h>
#include "eigen3/Eigen/Dense"

#define SINGULAR_TOLERANCE 0.001
#define IK_TOLERANCE 0.000001



struct DH {
	int active;
	double alpha;
	double a;
	double d;
	double theta0;
	double min;
	double max;
	double theta;
	double weight; // 0.0 <= weight <= 1.0
	double maxVel;

	double mass;
	double CoM[3]; // expressed in the local frame

	double H[4][4];
	double H0i[4][4];
};

class sKinematics
{
private:
	int *active_index;
	int total_links;
	int dof;
	struct DH *sDH;	
	double T0[4][4];
	double TF[4][4];

	double H0F[4][4];

	// com
	double mTotalMass;
	double mCoM[3];

	// Inverse Kinematics for several imitation points
	int nPoints;	
	int *ik_points_index;
	double *ik_points_weight, *vweight;
	double **J;
	double **FJ, **FJi;

	// Inverse Kinematics for Position of end-effector and palm direction vector
	double **JX, **JP, **N;
	double **OJ, **OJi;
	double *dIn;
	double *q, *delta_q;
	double lamda;

	double deltaT;

	void calFwd(void);

public:
	// initialization
	sKinematics(int num, double dt);
	int setDH(int index, double a, double d, double alpha, double theta0, int active, double min, double max, double maxVel);
	void setT0(double T[][4]);
	void setT0(MathLib::Matrix4 T);
	void setT0(Eigen::Matrix4d T);
	void getT0(MathLib::Matrix4 &T);
	void getT0(Eigen::Matrix4d& T);
	void setTF(double T[][4]);
	void setTF(MathLib::Matrix4 T);
	void readyForKinematics(void);

	void checkVelocityLimit(double *q_old, double *q, double dt);

	// com
	void setCOM(int index, double CoM[], double mass);
	void getCOM(double CoM[]);
	void computeCOM(void);

	// Interface for joints
	int getDOF(void);
	void setJoints(double ang[]);
	void getJoints(double ang[]);
	double getMin(int active_link);
	double getMax(int active_link);
	double getMaxVel(int active_link);
	void getMaxVel(double maxVel[]);
	void setJointWeight(double weight[]);
	
	// Forward Kinematics
	void getEndPos(double pos[]);
	void getEndPos(int link_index, Eigen::Vector3d& pos);
	void getEndPos(int link_index, double pos[]);
	void getEndDirAxis(int axis, double dir[]);
	void getLinkTMatrix(int link_index, double T[][4]);

	void getEndPos(Eigen::Vector3d &pos);
	void getEndDirAxis(int axis, Eigen::Vector3d &dir);

	// Jacobians
	void getJacobianPos(double **J);
	void getJacobianPos(int link_index, double **J);
	void getJacobianDirection(int axis, double **J);
	void getJacobianDirection(int link_index, int axis, double **J);
	void getJacobianPalm(int axis, double **J);
	void getJacobian(double **J);

	// usgin MathLib
	void getLinkTMatrix(int link_index, MathLib::Matrix4 &T);
	void getEndTMatrix(MathLib::Matrix4 &T);
	void getJacobianPos(MathLib::Matrix &J);
	void getJacobianPos(Eigen::MatrixXd &J);
	void getJacobianPos(int link_index, MathLib::Matrix &J);
	void getJacobianPos(int link_index, Eigen::MatrixXd &J);
	void getJacobianPos_fast(int link_index, Eigen::MatrixXd &J);
	void getJacobianDirection(int axis, MathLib::Matrix &J);
	void getJacobianDirection(int link_index, int axis, MathLib::Matrix &J);
	void getJacobianDirection( int axis, Eigen::MatrixXd &J);
	void getJacobianFullDirection(int axis1, int axis2, MathLib::Matrix &J);
	void getJacobian(MathLib::Matrix &J);
	void getJacobian(Eigen::MatrixXd &J);


	// Inverse Kinematics for several imitation points
	void initIKpoints(int nPoints, int points_index[], double points_weight[]);
	void solveIKpoints(double delta_x[], double delta_q[]);
	double solveIKpointsItr(double x_dest[], double q_out[]);

	// Inverse Kinematics for Position of end-effector and palm direction vector (5 DOF)
	void initIKPalm(double lamda);
	void solveIKPalm(double delta_x[], int axis, double delta_dir[], double q_rest[], double delta_q[]);
	double solveIKPalmItr(double x_dest[], int axis, double dir_dest[], double q_rest[], double q_out[]);

	MathLib::Vector3 calAngularVelocity(MathLib::Matrix3 &A, MathLib::Matrix3 &An, double dt);
	double CalAngle(MathLib::Vector3 vBase, MathLib::Vector3 vMeasure);
};

#endif //__sKinematics_H__
