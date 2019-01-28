/*
 * iktestiCub.cpp
 *
 *  Created on: Dec 20, 2012
 *      Author: seungsu
 */

#include "MathLib.h"
#include "IKGroupSolver.h"
#include "sKinematics.h"

#define ROBOT_DOF 7
#define IK_CONSTRAINTS 9
#define D2R (M_PI/180.)
#define R2D (180./M_PI)

double myjob[] = {0.0, 0.0, 0.0, 0.0, 50.0,  -00.0,  50,   0.0,   0.0,  10.0};
const int JOINTDIRECT_CONTROL_RATE  = 500; // Hz
const double DT  = 1./double(JOINTDIRECT_CONTROL_RATE); // milisecond


MathLib::Vector mJoint;

sKinematics *miCubKinematics;

MathLib::IKGroupSolver mIKSolver;
MathLib::Matrix        mJacobianPos;
MathLib::Matrix        mJacobianDir;
MathLib::Matrix        mJacobian9;

MathLib::Vector        mJointPos;
MathLib::Vector        mJointWeights;
MathLib::Vector        mJointVelocityLimitsDT;

void initiCubKinematics(){
	miCubKinematics = new sKinematics(ROBOT_DOF+1, DT);
	//miCubKinematics = new sKinematics(11, DT);

	// you should call sKin->setHD function lDofRobot times
	// @index               : starting from 0 to (num-1)
	// @a, d, alpha, theta0 : D-H Parameters
	// @min, max            : joint limits
	// @maxVel              : maximum joint speed
/*
	miCubKinematics->setDH(0,      0.032,      0.0,  M_PI_2,             0.0, 1,  -22.0*R2D,  22.0*R2D, 45.0*R2D);
	miCubKinematics->setDH(1,        0.0,      0.0,  M_PI_2,         -M_PI_2, 1,  -39.0*R2D,  39.0*R2D, 45.0*R2D);
	miCubKinematics->setDH(2,        0.0,  -0.1433,     0.0,             0.0, 1,  -10.0*R2D,  10.0*R2D, 45.0*R2D);
	miCubKinematics->setDH(3, -0.0233647,      0.0,  M_PI_2,      -105.0*R2D, 0,    0.0*R2D,   0.0*R2D,  0.0*R2D);
	miCubKinematics->setDH(4,        0.0, -0.10774,  M_PI_2,         -M_PI_2, 1,  -95.5*R2D,   5.0*R2D, 90.0*R2D);
	miCubKinematics->setDH(5,        0.0,      0.0, -M_PI_2,         -M_PI_2, 1,    0.0*R2D, 160.8*R2D, 90.0*R2D);
	miCubKinematics->setDH(6,        0.0, -0.15228, -M_PI_2,      -105.0*R2D, 1,  -37.0*R2D,  90.0*R2D, 90.0*R2D);
	miCubKinematics->setDH(7,      0.015,      0.0,  M_PI_2,             0.0, 1,    5.5*R2D, 106.0*R2D, 90.0*R2D);
	miCubKinematics->setDH(8,        0.0,  -0.1373,  M_PI_2,         -M_PI_2, 1,  -90.0*R2D,  90.0*R2D, 90.0*R2D);
	miCubKinematics->setDH(9,        0.0,      0.0,  M_PI_2,          M_PI_2, 1,  -90.0*R2D,  00.0*R2D, 90.0*R2D);
	miCubKinematics->setDH(10,       0.0,      0.0, -M_PI_2,          M_PI_2, 1,  -20.0*R2D,  20.0*R2D, 90.0*R2D);
	miCubKinematics->setDH(11,    0.0625,    0.016,     0.0,            M_PI, 0,    0.0*R2D,   0.0*R2D,  0.0*R2D);
*/


	miCubKinematics->setDH(0,      0.032,      0.0,  M_PI_2,             0.0, 1,  -20.0*D2R,  20.0*D2R, 10.0*D2R*1.0);
	miCubKinematics->setDH(1,        0.0,  -0.0055,  M_PI_2,         -M_PI_2, 1,  -20.0*D2R,  20.0*D2R, 10.0*D2R*1.0);
	miCubKinematics->setDH(2, -0.0233647,  -0.1433,  M_PI_2,      -105.0*D2R, 1,  -20.0*D2R,  20.0*D2R, 10.0*D2R*1.0);
	miCubKinematics->setDH(3,        0.0, -0.10774,  M_PI_2,         -M_PI_2, 1,  -95.0*D2R,  10.0*D2R, 90.0*D2R*1.0);
	miCubKinematics->setDH(4,        0.0,      0.0, -M_PI_2,         -M_PI_2, 1,    0.0*D2R, 160.8*D2R, 90.0*D2R*1.0);
	miCubKinematics->setDH(5,     -0.015, -0.15228, -M_PI_2,      -105.0*D2R, 1,  -37.0*D2R, 100.0*D2R, 90.0*D2R*1.0);
	miCubKinematics->setDH(6,      0.015,      0.0,  M_PI_2,             0.0, 1,   15.5*D2R, 106.0*D2R, 90.0*D2R*1.0);
	miCubKinematics->setDH(7,        0.0,  -0.1373,  M_PI_2,         -M_PI_2, 1,  -90.0*D2R,  90.0*D2R, 90.0*D2R*1.0);
	miCubKinematics->setDH(8,        0.0,      0.0,  M_PI_2,          M_PI_2, 1,  -90.0*D2R,  10.0*D2R, 90.0*D2R*1.0);
	//miCubKinematics->setDH(9,     0.0625,    0.016,     0.0,            M_PI, 1,  -20.0*D2R,  40.0*D2R, 90.0*D2R*50.0);
	miCubKinematics->setDH(9,     0.09,    0.05,     0.0,            M_PI, 1,  -20.0*D2R,  40.0*D2R, 90.0*D2R*50.0);
	miCubKinematics->setDH(10,       0.0,      0.0,     0.0,             0.0, 0,  -20.0*D2R,  20.0*D2R, 90.0*D2R*50.0);

	// T0 is a transformation matrix from global basement to base coordinate of 0th links
	// T0 are allocated by Identity matrix default. (if you not call this function T0 = I )
	double T0[4][4];
	MathLib::Matrix3 R0;
	R0(0,1) = -1.0;
	R0(1,2) = -1.0;
	R0(2,0) = 1.0;
	//R0 = MathLib::Matrix3::SRotationZ(-M_PI_2) * R0;

	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++) T0[i][j] = R0(i,j);

	for(int j=0; j<3; j++) T0[3][j] = 0.0;
	T0[3][3] = 1.0;
	miCubKinematics->setT0(T0);

	// ready for kinematics
	miCubKinematics->readyForKinematics();

	// variable for ik
	mJacobianPos.Resize(3,ROBOT_DOF);
	mJacobianDir.Resize(3,ROBOT_DOF);
	mJacobian9.Resize(9,ROBOT_DOF);

	double maxVel[ROBOT_DOF];
	miCubKinematics->getMaxVel(maxVel);
	mJointVelocityLimitsDT.Resize(ROBOT_DOF);
	mJointVelocityLimitsDT.Set(maxVel, ROBOT_DOF);
	mJointVelocityLimitsDT *= DT;


	// Solver init
	mIKSolver.SetSizes(ROBOT_DOF);  // Dof counts
	mIKSolver.AddSolverItem(IK_CONSTRAINTS);                 // One solver with 6 constraints
	mIKSolver.SetVerbose(false);                // No comments
	mIKSolver.SetThresholds(0.0001,0.00001);    // Singularities thresholds
	mIKSolver.Enable(true,0);                   // Enable first solver

	MathLib::IndicesVector mJointMapping;
	mJointMapping.clear();
	for(int i=0; i<ROBOT_DOF; i++){
		mJointMapping.push_back(i);
	}
	mIKSolver.SetDofsIndices(mJointMapping,0); // Joint maps for first solver

}


int main(int argc, char **argv)
{
	MathLib::Vector lJob(ROBOT_DOF);
	MathLib::Vector lJoint(ROBOT_DOF);
	MathLib::Vector lJointVel(ROBOT_DOF);

	MathLib::Vector mJointVelLimitsDn(ROBOT_DOF);
	MathLib::Vector mJointVelLimitsUp(ROBOT_DOF);

	MathLib::Vector lNextPos(3), lNextDirY(3), lNextDirZ(3);
	MathLib::Vector lPos(3), lDirY(3), lDirZ(3);
	MathLib::Vector lNextTargetVel(9);
	MathLib::Vector lError(3);
	MathLib::Matrix lWeight(ROBOT_DOF, ROBOT_DOF);


	initiCubKinematics();

	// set current joint angle;
	lJoint.Set(myjob, ROBOT_DOF);
	lJob.Set(myjob, ROBOT_DOF);
	miCubKinematics->setJoints(myjob);

	miCubKinematics->getEndPos(lNextPos.Array());
	miCubKinematics->getEndDirAxis(1, lNextDirY.Array());
	miCubKinematics->getEndDirAxis(2, lNextDirZ.Array());

	// set target
	lNextPos(0) +=  0.01;
	lNextPos(1) +=  0.01;
	lNextPos(2) +=  0.01;

	lNextDirY(0) += -0.1;
	lNextDirY(1) += -0.07;
	lNextDirY(2) +=  0.0224649;

	lNextDirZ(0) += -0.121096;
	lNextDirZ(1) +=  0.0587782;
	lNextDirZ(2) += -0.0736494;

	lDirY.Normalize();
	lDirZ.Normalize();


		// forward kinematics
		miCubKinematics->getEndPos(lPos.Array());
		miCubKinematics->getEndDirAxis(1, lDirY.Array());
		miCubKinematics->getEndDirAxis(2, lDirZ.Array());

		// inverse kinematics
		miCubKinematics->getJacobianPos(mJacobianPos);
		mJacobian9.SetRow(mJacobianPos.GetRow(0), 0);
		mJacobian9.SetRow(mJacobianPos.GetRow(1), 1);
		mJacobian9.SetRow(mJacobianPos.GetRow(2), 2);

		miCubKinematics->getJacobianDirection(1, mJacobianDir);
		mJacobian9.SetRow(mJacobianDir.GetRow(0), 3);
		mJacobian9.SetRow(mJacobianDir.GetRow(1), 4);
		mJacobian9.SetRow(mJacobianDir.GetRow(2), 5);

		miCubKinematics->getJacobianDirection(2, mJacobianDir);
		mJacobian9.SetRow(mJacobianDir.GetRow(0), 6);
		mJacobian9.SetRow(mJacobianDir.GetRow(1), 7);
		mJacobian9.SetRow(mJacobianDir.GetRow(2), 8);

		mIKSolver.SetJacobian(mJacobian9, 0);

		for(int i=0;i<ROBOT_DOF;i++){
			mJointVelLimitsDn(i) = -miCubKinematics->getMaxVel(i);
			mJointVelLimitsUp(i) =  miCubKinematics->getMaxVel(i);

			if( lJoint(i) <= miCubKinematics->getMin(i)){
				mJointVelLimitsDn(i) = 0.0;
			}
			else if( lJoint(i) >= miCubKinematics->getMax(i)){
				mJointVelLimitsUp(i) = 0.0;
			}
			else if( lJoint(i) < (miCubKinematics->getMin(i)+DEG2RAD(3))){
				mJointVelLimitsDn(i) *= (lJoint(i) - miCubKinematics->getMin(i))/DEG2RAD(3);
			}else if( lJoint(i) > (miCubKinematics->getMax(i)-DEG2RAD(3))){
				mJointVelLimitsUp(i) *= (miCubKinematics->getMax(i)-lJoint(i))/DEG2RAD(3);
			}
		}

		mIKSolver.SetLimits(mJointVelLimitsDn,mJointVelLimitsUp);


		lWeight.Identity();
		lJointVel.Zero();
		for(int ikrpt =0; ikrpt<5; ikrpt++)
		{
			for(int i=0;i<3;i++){
				lNextTargetVel(i)   = (lNextPos(i)  - lPos(i) )/DT;
				lNextTargetVel(i+3) = (lNextDirY(i) - lDirY(i))/DT*0.0;
				lNextTargetVel(i+6) = (lNextDirZ(i) - lDirZ(i))/DT*0.0;
			}
			lNextTargetVel -= mJacobian9*lJointVel;

			mIKSolver.SetJacobian(mJacobian9*lWeight);
			mIKSolver.SetTarget(lNextTargetVel, 0);
			mIKSolver.Solve();
			lJointVel += mIKSolver.GetOutput();

			for(int i=0;i<ROBOT_DOF; i++){
				if( lJointVel(i) >=  miCubKinematics->getMaxVel(i) ){
					lJointVel(i) = miCubKinematics->getMaxVel(i);
					lWeight(i,i) = 0.0;
				}
				else if( lJointVel(i) <= -miCubKinematics->getMaxVel(i) ){
					lJointVel(i) = -miCubKinematics->getMaxVel(i);
					lWeight(i,i) = 0.0;
				}
				else{
					lWeight(i,i) = 1.0;
				}
			}
			//lWeight.Print();
			lNextTargetVel.Print();
			//(mJacobian9*lJointVel).Print();
		}


		/*
	// solve ik
	for( int ikitr=0; ikitr<10; ikitr++){
		// forward kinematics
		miCubKinematics->getEndPos(lPos.Array());
		miCubKinematics->getEndDirAxis(1, lDirY.Array());
		miCubKinematics->getEndDirAxis(2, lDirZ.Array());

		// inverse kinematics
		miCubKinematics->getJacobianPos(mJacobianPos);
		mJacobian9.SetRow(mJacobianPos.GetRow(0), 0);
		mJacobian9.SetRow(mJacobianPos.GetRow(1), 1);
		mJacobian9.SetRow(mJacobianPos.GetRow(2), 2);

		miCubKinematics->getJacobianDirection(1, mJacobianDir);
		mJacobian9.SetRow(mJacobianDir.GetRow(0), 3);
		mJacobian9.SetRow(mJacobianDir.GetRow(1), 4);
		mJacobian9.SetRow(mJacobianDir.GetRow(2), 5);

		miCubKinematics->getJacobianDirection(2, mJacobianDir);
		mJacobian9.SetRow(mJacobianDir.GetRow(0), 6);
		mJacobian9.SetRow(mJacobianDir.GetRow(1), 7);
		mJacobian9.SetRow(mJacobianDir.GetRow(2), 8);

		mIKSolver.SetJacobian(mJacobian9, 0);


		miCubKinematics->getJoints(lJoint.Array());
		// Set maximum joint velocity

		for(int i=0;i<ROBOT_DOF;i++){
			mJointVelLimitsDn(i) = -miCubKinematics->getMaxVel(i);
			mJointVelLimitsUp(i) =  miCubKinematics->getMaxVel(i);

			if( lJoint(i) <= miCubKinematics->getMin(i)){
				mJointVelLimitsDn(i) = 0.0;
			}
			else if( lJoint(i) >= miCubKinematics->getMax(i)){
				mJointVelLimitsUp(i) = 0.0;
			}
			else if( lJoint(i) < (miCubKinematics->getMin(i)+DEG2RAD(3))){
				mJointVelLimitsDn(i) *= (lJoint(i) - miCubKinematics->getMin(i))/DEG2RAD(3);
			}else if( lJoint(i) > (miCubKinematics->getMax(i)-DEG2RAD(3))){
				mJointVelLimitsUp(i) *= (miCubKinematics->getMax(i)-lJoint(i))/DEG2RAD(3);
			}
		}

		mIKSolver.SetLimits(mJointVelLimitsDn,mJointVelLimitsUp);


		// calculate target velocity
		for(int i=0;i<3;i++){
			lNextTargetVel(i)   = (lNextPos(i)  - lPos(i) )/DT;
			lNextTargetVel(i+3) = (lNextDirY(i) - lDirY(i))/DT*0.0;
			lNextTargetVel(i+6) = (lNextDirZ(i) - lDirZ(i))/DT*0.0;
		}

		//(lNextTargetVel*DT).Print();
		mIKSolver.SetTarget(lNextTargetVel, 0);
		mIKSolver.Solve();
		lJointVel = mIKSolver.GetOutput();

		lJoint += lJointVel*DT;
		miCubKinematics->setJoints(lJoint.Array());

		lError = (lNextPos - lPos);
		printf("%lf %lf %lf \n", lError(0), lError(1), lError(2) );
	}
*/

	delete miCubKinematics;
	return 0;
}
