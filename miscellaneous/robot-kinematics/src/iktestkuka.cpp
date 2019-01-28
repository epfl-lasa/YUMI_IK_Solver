/*
 * iktestiCub.cpp
 *
 *  Created on: Dec 20, 2012
 *      Author: seungsu
 */

#include "MathLib.h"
#include "IKGroupSolver.h"
#include "sKinematics.h"

#define KUKA_DOF 7
#define IK_CONSTRAINTS 9
#define D2R (M_PI/180.)
#define R2D (180./M_PI)

double myjob[] = {0., -PI/4.0, 0.0, -PI/2.0, 0.0, -PI/4.0, 0.0};
const int JOINTDIRECT_CONTROL_RATE  = 500; // Hz
const double _dt  = 1./double(JOINTDIRECT_CONTROL_RATE); // milisecond


MathLib::Vector mJoint;

sKinematics *mSKinematicChain;

MathLib::IKGroupSolver mIKSolver;
MathLib::Matrix        mJacobianPos;
MathLib::Matrix        mJacobianDirY, mJacobianDirZ;
MathLib::Matrix        mJacobian9;

MathLib::Vector        mJointPos;
MathLib::Vector        mJointWeights;
MathLib::Vector        mJointVelocityLimits_dt;

void initiCubKinematics(){
	mSKinematicChain = new sKinematics(KUKA_DOF, _dt);
	//mSKinematicChain->setDH(0,  0.0,  0.31, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(110.0)*0.77);
	mSKinematicChain->setDH(0,  0.0,  0.31, M_PI_2, 0.0, 1,  DEG2RAD( -85.), DEG2RAD( 85.), DEG2RAD(132.0)*0.95);
	mSKinematicChain->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -90.), DEG2RAD( 90.), DEG2RAD(132.0)*0.95);
	//mSKinematicChain->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(132.0)*0.8);
	//mSKinematicChain->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(128.0)*0.30);
	mSKinematicChain->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-100.), DEG2RAD(100.), DEG2RAD(128.0)*0.95);
	mSKinematicChain->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(128.0)*0.95);
	mSKinematicChain->setDH(4,  0.0,  0.39, M_PI_2, 0.0, 1,  DEG2RAD(-140.), DEG2RAD(140.), DEG2RAD(204.0)*0.95);
	//mSKinematicChain->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(132.0));
	mSKinematicChain->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -120.), DEG2RAD( 90.), DEG2RAD(180.0)*0.95); // reduced joint angle to save the fingers
	//mSKinematicChain->setDH(6,  0.0,  0.20,    0.0, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(132.0)*0.99);
	//mSKinematicChain->setDH(6,  0.0,  0.217,    0.0, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(184.0)*0.95); // for barrett
	//mSKinematicChain->setDH(6, -0.06,  0.180,    0.0, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(184.0)*0.95); // for sim lab
	mSKinematicChain->setDH(6, -0.068,  0.210,    0.0, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(184.0)*0.95); // for sim lab

	// T0 is a transformation matrix from global basement to base coordinate of 0th links
	// T0 are allocated by Identity matrix default. (if you not call this function T0 = I )
	double T0[4][4];
	MathLib::Matrix3 R0;
	R0.Identity();

	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++) T0[i][j] = R0(i,j);

	for(int j=0; j<3; j++) T0[3][j] = 0.0;
	T0[3][3] = 1.0;
	mSKinematicChain->setT0(T0);


	MathLib::Matrix4 mTF;
	mTF.Identity();
	mTF.SetOrientation( MathLib::Matrix3::SRotationY(M_PI/4.0) );

	double TF[4][4];
		for(int i=0; i<4; i++)
			for(int j=0; j<4; j++)
				TF[i][j] = mTF(i,j);

	mTF.Print();
	mSKinematicChain->setTF(TF);


	// ready for kinematics
	mSKinematicChain->readyForKinematics();

	// variable for ik
	mJacobianPos.Resize(3,KUKA_DOF);
	mJacobianDirY.Resize(3,KUKA_DOF);
	mJacobianDirZ.Resize(3,KUKA_DOF);
	mJacobian9.Resize(9,KUKA_DOF);

	double maxVel[KUKA_DOF];
	mSKinematicChain->getMaxVel(maxVel);
	mJointVelocityLimits_dt.Resize(KUKA_DOF);
	mJointVelocityLimits_dt.Set(maxVel, KUKA_DOF);
	mJointVelocityLimits_dt *= _dt;


	// Solver init
	mIKSolver.SetSizes(KUKA_DOF);  // Dof counts
	mIKSolver.AddSolverItem(IK_CONSTRAINTS);                 // One solver with 6 constraints
	mIKSolver.SetVerbose(false);                // No comments
	mIKSolver.SetThresholds(0.0001,0.00001);    // Singularities thresholds
	mIKSolver.Enable(true,0);                   // Enable first solver

	MathLib::IndicesVector mJointMapping;
	mJointMapping.clear();
	for(int i=0; i<KUKA_DOF; i++){
		mJointMapping.push_back(i);
	}
	mIKSolver.SetDofsIndices(mJointMapping,0); // Joint maps for first solver

}


int main(int argc, char **argv)
{
	double lDirWeight;

	MathLib::Vector lJob(KUKA_DOF);
	MathLib::Vector lJoints(KUKA_DOF);
	MathLib::Vector lJointVel(KUKA_DOF);
	MathLib::Vector mJointDesVel(KUKA_DOF);
	MathLib::Vector mJointDesPos(KUKA_DOF);

	MathLib::Vector mJointVelLimitsDn(KUKA_DOF);
	MathLib::Vector mJointVelLimitsUp(KUKA_DOF);

	MathLib::Vector lTargetPos(3), lTargetDirY(3), lTargetDirZ(3);
	MathLib::Vector lPos(3), lDirY(3), lDirZ(3);
	MathLib::Vector mTargetVelocity(9);
	MathLib::Vector lLocalCartVel(9);

	MathLib::Vector lError(3);
	MathLib::Matrix lWeight(KUKA_DOF, KUKA_DOF);

	double lVelMux = 50.0;

	initiCubKinematics();

	// set current joint angle;
	lJoints.Set(myjob, KUKA_DOF);
	lJob.Set(myjob, KUKA_DOF);
	mSKinematicChain->setJoints(myjob);

	FILE *fout;
	fout = fopen("/home/seungsu/devel/roscodes/SKinematics/data/out.txt", "w+");


	FILE *flog;
	char line[1024];
	flog = fopen("/home/seungsu/devel/roscodes/SKinematics/data/log.txt", "r+");

	for(int i=0; i<5000; i++)
	{
		fgets(line, 1000, flog );
	}

	double temp;
	sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
			&temp,  &temp,  &temp,
			&lTargetPos(0), &lTargetPos(1), &lTargetPos(2),
			&lTargetDirY(0), &lTargetDirY(1), &lTargetDirY(2),
			&lTargetDirZ(0), &lTargetDirZ(1), &lTargetDirZ(2) );
	lTargetPos.Print();
	lTargetDirY.Print();
	lTargetDirZ.Print();

	lDirY.Normalize();
	lDirZ.Normalize();

	MathLib::Vector perror(3);


	for(int frame=0; frame<30; frame++)
	{
		// forward kinematics
		mSKinematicChain->getJoints(lJoints.Array());
		mSKinematicChain->getEndPos(lPos.Array());
		mSKinematicChain->getEndDirAxis(1, lDirY.Array());
		mSKinematicChain->getEndDirAxis(2, lDirZ.Array());

		//mSKinematicChain->getEndPos(6, lPos.Array());

		//lTargetDirY.Print("Target Dir Y");
		//lDirY.Print("local dir");

		perror = (lTargetPos - lPos);
		printf("%02d %lf %lf %lf ", frame, perror(0), perror(1), perror(2) );
		perror = (lTargetDirY - lDirY);
		for(int i=0; i<3; i++) printf("%lf ", perror(i) );
		perror = (lTargetDirZ - lDirZ);
		for(int i=0; i<3; i++) printf("%lf ", perror(i) );
		printf("\n");


    	// Set Jacobian
		mSKinematicChain->getJacobianPos(mJacobianPos);
    	mSKinematicChain->getJacobianDirection(1, mJacobianDirY);
    	mSKinematicChain->getJacobianDirection(2, mJacobianDirZ);

    	for(int i=0; i<3; i++){
    		mJacobian9.SetRow(mJacobianPos.GetRow(i)   , i  );
    		mJacobian9.SetRow(mJacobianDirY.GetRow(i), i+3);
    		mJacobian9.SetRow(mJacobianDirZ.GetRow(i), i+6);
    	}

    	// Set maximum joint velocity
		for(int i=0;i<KUKA_DOF;i++){
			mJointVelLimitsDn(i) = -mSKinematicChain->getMaxVel(i)*lVelMux;
			mJointVelLimitsUp(i) =  mSKinematicChain->getMaxVel(i)*lVelMux;
			/*
			if( lJoints(i) <= mSKinematicChain->getMin(i)){
				mJointVelLimitsDn(i) = 0.0;
			}
			else if( lJoints(i) >= mSKinematicChain->getMax(i)){
				mJointVelLimitsUp(i) = 0.0;
			}
			else if( lJoints(i) < (mSKinematicChain->getMin(i)+DEG2RAD(3))){
				mJointVelLimitsDn(i) *= (lJoints(i) - mSKinematicChain->getMin(i))/DEG2RAD(3);
			}else if( lJoints(i) > (mSKinematicChain->getMax(i)-DEG2RAD(3))){
				mJointVelLimitsUp(i) *= (mSKinematicChain->getMax(i)-lJoints(i))/DEG2RAD(3);
			}
			*/
		}

    	mIKSolver.SetJacobian(mJacobian9, 0);
		mIKSolver.SetLimits(mJointVelLimitsDn,mJointVelLimitsUp);

		if( (lTargetPos -lPos ).Norm() > 0.05 ){
			lDirWeight = 0.0;
		}
		else{
			lDirWeight = (0.05-(lTargetPos -lPos).Norm())/0.05;
			if( lDirWeight > 1.0 ) lDirWeight = 1.0;
			else if(lDirWeight < 0.0 ) lDirWeight = 0.0;
		}

		// set target velocity
		mTargetVelocity.SetSubVector(0, (lTargetPos -lPos ) / _dt*1.0 );
		mTargetVelocity.SetSubVector(3, (lTargetDirY-lDirY) / _dt*lDirWeight );
		mTargetVelocity.SetSubVector(6, (lTargetDirZ-lDirZ) / _dt*lDirWeight );

		lWeight.Identity();
		mJointDesVel.Zero();
		for(int ikrpt =0; ikrpt<1; ikrpt++)
		{
			lLocalCartVel = mTargetVelocity - mJacobian9*mJointDesVel;

			mIKSolver.SetJacobian(mJacobian9*lWeight);
			mIKSolver.SetTarget(lLocalCartVel, 0);
			mIKSolver.Solve();
			mJointDesVel += mIKSolver.GetOutput();

			for(int i=0;i<KUKA_DOF; i++){
				if( mJointDesVel(i) >=  mSKinematicChain->getMaxVel(i)*lVelMux ){
					mJointDesVel(i) = mSKinematicChain->getMaxVel(i)*lVelMux;
					lWeight(i,i) = 0.0;
				}
				else if( mJointDesVel(i) <= -mSKinematicChain->getMaxVel(i)*lVelMux ){
					mJointDesVel(i) = -mSKinematicChain->getMaxVel(i)*lVelMux;
					lWeight(i,i) = 0.0;
				}
				else{
					lWeight(i,i) = 1.0;
				}
			}
		}
		mJointDesPos = lJoints + mJointDesVel*_dt;
		mSKinematicChain->setJoints(mJointDesPos.Array());
		for(int i=0; i<7; i++) fprintf(fout, "%lf ", mJointDesPos(i) );
		fprintf(fout, "\n");
	}
	fclose(fout);

		fclose(flog);
	delete mSKinematicChain;
	return 0;
}
