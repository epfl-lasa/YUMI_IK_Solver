/*
 * CDDynamics.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Seungsu KIM
 */

/*
 * how to use

	CDDynamics *testDyn;
	testDyn = new CDDynamics(dim, dt, wn);

	testDyn->SetVelocityLimits(velLimits);
	testDyn->SetState(initial);
	testDyn->SetTarget(target);


	start loop
		// if new target is set
		testDyn->SetTarget(target);

		// update dynamics
		testDyn->Update();

		// get state
		testDyn->GetState(state);
	end loop
 */

#include "CDDynamics.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

CDDynamics::CDDynamics(int dim, double dt, double Wn)
{
	// set environment
	mDim = dim;
	mDT = dt;
	mWn = Wn;

	// resize variables
	mTarget.Resize(mDim);
	mTargetVelocity.Resize(mDim);
	mState.Resize(mDim);
	mStateVelocity.Resize(mDim);
	mPositionLimits.Resize(mDim);
	mVelocityLimits.Resize(mDim);

	// set initial values
	mState.Zero();
	mStateVelocity.Zero();
	mTarget.Zero();
	mTargetVelocity.Zero();

	mPositionLimits.Zero();
	mVelocityLimits.Zero();
}

void CDDynamics::SetState(const Vector & Position)
{
	if(mDim == Position.Size()){
		mState = Position - mTarget;
	}
	else{
		cout<<"Dimension error! @ CDDynamics::SetState() "<<endl;
	}
}

void CDDynamics::SetState(const Vector & Position, const Vector & Velocity)
{
	if( (mDim == Position.Size()) && (mDim == Velocity.Size())){
		mState = Position - mTarget;
		mStateVelocity = Velocity - mTargetVelocity;
	}
	else{
		cout<<"Dimension error! @ CDDynamics::SetState() "<<endl;
	}
}

void CDDynamics::SetTarget(const Vector & target)
{
	if(mDim == target.Size()){
		mState += (mTarget-target);
		mTarget = target;
	}
	else{
		cout<<"Dimension error! @ CDDynamics::SetTarget() "<<endl;
	}
}

void CDDynamics::SetStateTarget(const Vector & Position, const Vector & Target)
{
	SetState(Position);
	SetTarget(Target);
}

/*
void CDDynamics::SetTarget(const Vector & target, const Vector & targetVel)
{
	if( (mDim == target.Size()) && (mDim == targetVel.Size())){
		mState += (mTarget-target);
		mStateVelocity += (mTargetVelocity-targetVel);

		mTarget = target;
		mTargetVelocity = targetVel;
	}
	else{
		cout<<"Dimension error! @ CDDynamics::SetTarget() "<<endl;
	}
}
*/

void CDDynamics::SetDt(double dt)
{
	mDT = dt;
}

void CDDynamics::SetWn(double Wn)
{
	mWn = Wn;
}

void CDDynamics::SetVelocityLimits(const Vector & velLimits)
{
	if(mDim == velLimits.Size()){
		mVelocityLimits = velLimits;
	}
	else{
		cout<<"Dimension error! @ CDDynamics::SetVelocityLimits() "<<endl;
	}
}

void CDDynamics::RemoveVelocityLimits(void)
{
	mVelocityLimits.Zero();
}

void CDDynamics::SetPositionLimits(const Vector & posLimits)
{
	if(mDim == posLimits.Size()){
		mPositionLimits = posLimits;
	}
	else{
		cout<<"Dimension error! @ CDDynamics::SetPositionLimits() "<<endl;
	}
}

void CDDynamics::RemovePositionLimits(void)
{
	mPositionLimits.Zero();
}

void CDDynamics::GetTarget(Vector & target)
{
	target = mTarget;
}
/*
void CDDynamics::GetTarget(Vector & target, Vector & targetVel)
{
	target = mTarget;
	targetVel = mTargetVelocity;
}
*/
void CDDynamics::GetState(Vector & Position)
{
	Position = mState + mTarget;
}

void CDDynamics::GetState(Vector & Position, Vector & Velocity)
{
	Position = mState + mTarget;
	Velocity = mStateVelocity + mTargetVelocity;
}

void CDDynamics::Update()
{
	Update(mDT, 1.0);
}

void CDDynamics::Update(double dt)
{
	Update(dt, 1.0);
}

void CDDynamics::Update(double dt, double muxVel)
{
	// A      = x(0);
	// B      = x_d(0) + w*x(0)
	// x(t)   = (A+Bt)e^(-w*t)
	// x_d(t) = (-w*A+(1-w*t)B ) e^(-w*t)

	double B;
	double x;
	for(unsigned int i=0; i<mDim; i++ ){
		x = mState(i);
		B = mStateVelocity(i)+mWn*x;

		//mStateVelocity(i) = ( -mWn*mState(i) + (1.0-mWn*dt*muxVel)*B )* exp(-mWn*dt*muxVel);
		//mState(i)         = ( mState(i)+B*dt*muxVel )*exp(-mWn*dt*muxVel);
		mState(i)         = x+mStateVelocity(i)*dt;
		mStateVelocity(i) = ( -mWn*x + (1.0-mWn*dt)*B )* exp(-mWn*dt) *muxVel;

		if( mPositionLimits(i)>0){
			if     ( mState(i) >  mPositionLimits(i) ) mState(i) =  mPositionLimits(i);
			else if( mState(i) < -mPositionLimits(i) ) mState(i) = -mPositionLimits(i);
		}

		if( mVelocityLimits(i)>0){
			if     ( mStateVelocity(i) >  mVelocityLimits(i) ) mStateVelocity(i) =  mVelocityLimits(i);
			else if( mStateVelocity(i) < -mVelocityLimits(i) ) mStateVelocity(i) = -mVelocityLimits(i);
		}
	}
}
