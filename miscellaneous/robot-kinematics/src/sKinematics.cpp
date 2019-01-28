
#include <stdio.h> 
#include <stdlib.h> 
#include <math.h> 
#include "math_lib.h"
#include "sKinematics.h" 

/******************************************************************
 * Calculate forward kinematics
 ******************************************************************/
void sKinematics::calFwd(void)
{
	double alpha, theta, c_theta, s_theta, c_alpha, s_alpha;
	int link_index;
	for( int ai=0; ai<dof; ai++ ){
		link_index = active_index[ai];
		
		alpha = sDH[link_index].alpha;
		theta = sDH[link_index].theta+sDH[link_index].theta0;

		c_theta = cos(theta);
		s_theta = sin(theta);
		c_alpha = cos(alpha);
		s_alpha = sin(alpha);

		sDH[link_index].H[0][0]=c_theta;
		sDH[link_index].H[0][1]=-s_theta*c_alpha;
		sDH[link_index].H[0][2]=s_theta*s_alpha;
		sDH[link_index].H[0][3]=c_theta*sDH[link_index].a;

		sDH[link_index].H[1][0]=s_theta;
		sDH[link_index].H[1][1]=c_theta*c_alpha;
		sDH[link_index].H[1][2]=-c_theta*s_alpha;
		sDH[link_index].H[1][3]=s_theta*sDH[link_index].a;
	}

	matmul_c44(4, T0, sDH[0].H, sDH[0].H0i );
	for( int i=1; i<total_links; i++ ){
		matmul_c44(4, sDH[i-1].H0i, sDH[i].H, sDH[i].H0i );
	}
	matmul_c44(4, sDH[total_links-1].H0i, TF, H0F);
}

/******************************************************************
 * Creator
 *
 * @ num : number of links to define DH Parameters
 ******************************************************************/
sKinematics::sKinematics(int num, double dt)
{
	deltaT = dt;
	total_links = num;
	sDH = (struct DH *)malloc(sizeof(struct DH)*num);

	for(int i=0; i<num; i++)
	{
		sDH[i].mass   = 0.0;
		sDH[i].CoM[0] = 0.0;
		sDH[i].CoM[1] = 0.0;
		sDH[i].CoM[2] = 0.0;
	}

	matI_c4( T0 );
	matI_c4( TF );
}

/******************************************************************
 * Set DH Parameters
 *
 * @index  : index
 * @a, d, alpha, theta0 : D-H parameter values
 * @active : active joints -> 1
 * @min    : minimum joint limit
 * @max    : maximum joint limit
 * @maxVel : maximum joint velocity limit ( >0 )
 ******************************************************************/
int sKinematics::setDH(int index, double a, double d, double alpha, double theta0, int active, double min, double max, double maxVel)
{
	if( index >= total_links ) return -1;
	else{
		sDH[index].alpha  = alpha;
		sDH[index].a      = a;
		sDH[index].d      = d;
		sDH[index].theta0 = theta0;
		sDH[index].active = active;
		sDH[index].min    = min;
		sDH[index].max    = max;
		sDH[index].maxVel = maxVel;
		sDH[index].weight = 1.0;

		sDH[index].theta  = 0.0;		

		matzero_c4(4, sDH[index].H );
		sDH[index].H[2][1] = sin(alpha);
		sDH[index].H[2][2] = cos(alpha);
		sDH[index].H[2][3] = d;
		sDH[index].H[3][3] = 1.0;

		return 0;
	}
}

/******************************************************************
 * Initialize the kinematics variables
 *
 * : It should be called one time just after calling setDH for all links
 ******************************************************************/
void sKinematics::setT0(double T[][4])
{
	matcp_c4(4, T, T0 );
}
void sKinematics::setT0(MathLib::Matrix4 T)
{
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			 T0[i][j] = T(i,j);
}

void sKinematics::setT0(Eigen::Matrix4d T)
{
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			 T0[i][j] = T(i,j);
}

void sKinematics::getT0(MathLib::Matrix4& T)
{
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			 T(i,j) = T0[i][j];
}

void sKinematics::getT0(Eigen::Matrix4d& T)
{
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			 T(i,j) = T0[i][j];
}

/******************************************************************
 * Initialize the kinematics variables
 *
 * : It should be called one time just after calling setDH for all links
 ******************************************************************/
void sKinematics::setTF(double T[][4])
{
	matcp_c4(4, T, TF );
}

void sKinematics::setTF(MathLib::Matrix4 T)
{
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			TF[i][j] = T(i,j);
}

/******************************************************************
 * Initialize the kinematics variables
 *
 * : It should be called one time just after calling setDH for all links
 ******************************************************************/
void sKinematics::readyForKinematics(void)
{	
	// find degree of freedom
	dof = 0;
	int i, ai;
	for( i=0; i<total_links; i++ ){
		if( sDH[i].active ) dof++;
	}
	active_index = (int *)malloc(sizeof(int) * dof );

	// generate active_index
	ai = 0;
	for( i=0; i<total_links; i++ ){
		if( sDH[i].active ==1 ) active_index[ai++] = i;
	}

	// find inactive link trnsformation matrix
	double c_theta, s_theta, c_alpha, s_alpha;
	for( int i=0; i<total_links; i++ ){
		if( sDH[i].active == 0 ){			
			c_theta = cos(sDH[i].theta0);
			s_theta = sin(sDH[i].theta0);
			c_alpha = cos(sDH[i].alpha);
			s_alpha = sin(sDH[i].alpha);

			sDH[i].H[0][0]=c_theta;
			sDH[i].H[0][1]=-s_theta*c_alpha;
			sDH[i].H[0][2]=s_theta*s_alpha;
			sDH[i].H[0][3]=c_theta*sDH[i].a;

			sDH[i].H[1][0]=s_theta;
			sDH[i].H[1][1]=c_theta*c_alpha;
			sDH[i].H[1][2]=-c_theta*s_alpha;
			sDH[i].H[1][3]=s_theta*sDH[i].a;
		}
	}

	mTotalMass = 0.0;
	for( int i=0; i<total_links; i++ ){
		if( sDH[i].mass > 0.0 ) mTotalMass += sDH[i].mass;
	}

	MathLib::Vector lJoints(dof);
	lJoints.Zero();
	setJoints(lJoints.Array());

}

void sKinematics::checkVelocityLimit(double *q_old, double *q, double dt)
{
	int i;
	double delta_q, max_delta_q;
	for( int ai=0; ai<dof; ai++){
		i = active_index[ai];
		delta_q = q[ai] - q_old[ai];
		max_delta_q = sDH[i].maxVel*dt;

		if(      delta_q >  max_delta_q ) q[ai] = q_old[ai] + max_delta_q;
		else if( delta_q < -max_delta_q ) q[ai] = q_old[ai] - max_delta_q;
	}
}

void sKinematics::setCOM(int index, double CoM[], double mass)
{
	sDH[index].mass = mass;
	sDH[index].CoM[0] = CoM[0];
	sDH[index].CoM[1] = CoM[1];
	sDH[index].CoM[2] = CoM[2];
}

void sKinematics::getCOM(double CoM[])
{
	matcp_c1(3, mCoM, CoM);
}

void sKinematics::computeCOM(void)
{
	double lCoM[4];
	double lCoMTotal[3];
	double lOut[4];

	matzero_c1(3, lCoMTotal);
	for( int i=1; i<total_links; i++ ){
		if( sDH[i].mass > 0.0 ){
			lCoM[0] = sDH[i].CoM[0];
			lCoM[1] = sDH[i].CoM[1];
			lCoM[2] = sDH[i].CoM[2];
			lCoM[3] = 1.0;

			matmul_c41(4, sDH[i].H0i, lCoM, lOut);
			lCoMTotal[0] += sDH[i].mass*lOut[0];
			lCoMTotal[1] += sDH[i].mass*lOut[1];
			lCoMTotal[2] += sDH[i].mass*lOut[2];
		}
	}

	mCoM[0] = lCoMTotal[0]/mTotalMass;
	mCoM[1] = lCoMTotal[1]/mTotalMass;
	mCoM[2] = lCoMTotal[2]/mTotalMass;
}

/******************************************************************
 * Get active joint DOF
 * 
 ******************************************************************/
int sKinematics::getDOF(void)
{
	return dof;
}


/******************************************************************
 * Set joint angle values 
 * : ang[] is the angles for active joints
 ******************************************************************/
void sKinematics::setJoints(double ang[])
{
	int i;
	for( int ai=0; ai<dof; ai++){
		// min max check
		i = active_index[ai];
		sDH[i].theta = (ang[ai]<sDH[i].min) ? sDH[i].min : ((ang[ai]>sDH[i].max) ? sDH[i].max :ang[ai]);
		//sDH[i].theta = ang[ai];
	}

	calFwd();
}

/******************************************************************
 * Get active joint angles
 * 
 ******************************************************************/
void sKinematics::getJoints(double ang[])
{
	for( int ai=0; ai<dof; ai++){
		ang[ai] = sDH[ active_index[ai] ].theta;
	}
}

/******************************************************************
 * Get end effector position
 *
 * @return : 3DOF position 
 ******************************************************************/
void sKinematics::getEndPos(double pos[])
{
	for(int i=0; i<3; i++ ){
		pos[i] = H0F[i][3];
	}
}

/******************************************************************
 * Get end effector position
 *
 * @return : 3DOF position
 ******************************************************************/
void sKinematics::getEndPos(Eigen::Vector3d &pos)
{

	for(int i=0; i<3; i++ ){
		pos(i) = H0F[i][3];
	}

}


/******************************************************************
 * Get position of link_index
 *
 * @return : 3DOF position 
 ******************************************************************/
void sKinematics::getEndPos(int link_index, double pos[])
{
	for(int i=0; i<3; i++ ){
		pos[i] = sDH[link_index].H0i[i][3];
	}
}

void sKinematics::getEndPos(int link_index, Eigen::Vector3d& pos)
{
	for(int i=0; i<3; i++ ){
		pos(i) = sDH[link_index].H0i[i][3];
	}
}

/******************************************************************
 * Get end effector direction 
 *
 * @return : Axis direction vector
 ******************************************************************/
void sKinematics::getEndDirAxis(int axis, double dir[])
{
	for(int i=0; i<3; i++ ){
		//dir[i] = H0F[i][axis];
		dir[i] = H0F[i][axis];
	}
}

/******************************************************************
 * Get end effector direction
 *
 * @return : Axis direction vector
 ******************************************************************/
void sKinematics::getEndDirAxis(int axis, Eigen::Vector3d  &dir)
{

	for(int i=0; i<3; i++ ){
		//dir[i] = H0F[i][axis];
		dir(i) = H0F[i][axis];
	}
}

void sKinematics::getLinkTMatrix(int link_index, double T[][4])
{
	matcp_c4(4, sDH[link_index].H0i, T );
	//matcp_c4(4, H0F, T );
}

void sKinematics::getLinkTMatrix(int link_index, MathLib::Matrix4 &T)
{
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			T(i,j) = sDH[link_index].H0i[i][j];
}

void sKinematics::getEndTMatrix(MathLib::Matrix4 &T)
{
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			T(i,j) = H0F[i][j];
}

/******************************************************************
 * Get min value of joint limit for active_link
 ******************************************************************/
double sKinematics::getMin(int active_link)
{
	return sDH[ active_index[active_link] ].min;
}

/******************************************************************
 * Get max value of joint limit for active_link
 ******************************************************************/
double sKinematics::getMax(int active_link)
{
	return sDH[ active_index[active_link] ].max;
}

/******************************************************************
 * Get max value of joint velocity limit for active_link
 ******************************************************************/
double sKinematics::getMaxVel(int active_link)
{
	return sDH[ active_index[active_link] ].maxVel;
}

/******************************************************************
 * Get max value of joint velocity limits of active links
 ******************************************************************/
void sKinematics::getMaxVel(double maxVel[])
{
	for(int i=0; i<dof; i++ ){
		maxVel[i] = sDH[ active_index[i] ].maxVel;
	}
}


/******************************************************************
 * Get max value of joint limit for active_link
 ******************************************************************/
void sKinematics::setJointWeight(double weight[])
{
	for(int i=0; i<dof; i++ ){
		sDH[active_index[i] ].weight = weight[i];
	}
}

/******************************************************************
 * Get position jacobian
 *
 * @ J : return position jacobian (3 X dof)
 ******************************************************************/
void sKinematics::getJacobianPos(double **J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if( i == 0 ){
			for( j=0; j<3; j++){
				v1[j] = T0[j][2];
				v2[j] = H0F[j][3] - T0[j][3];
			}
		}
		else{
			for( j=0; j<3; j++){
				v1[j] = sDH[i-1].H0i[j][2];
				v2[j] = H0F[j][3] - sDH[i-1].H0i[j][3];
			}
		}
		CrossProduct(v1, v2, v3);
		J[0][ai] = v3[0];
		J[1][ai] = v3[1];
		J[2][ai] = v3[2];
	}
}

/******************************************************************
 * Get position jacobian
 *
 * @ J : return position jacobian (3 X dof)
 ******************************************************************/
void sKinematics::getJacobianPos(MathLib::Matrix &J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if( i == 0 ){
			for( j=0; j<3; j++){
				v1[j] = T0[j][2];
				v2[j] = H0F[j][3] - T0[j][3];
			}
		}
		else{
			for( j=0; j<3; j++){
				v1[j] = sDH[i-1].H0i[j][2];
				v2[j] = H0F[j][3] - sDH[i-1].H0i[j][3];
			}
		}
		CrossProduct(v1, v2, v3);
		J(0,ai) = v3[0];
		J(1,ai) = v3[1];
		J(2,ai) = v3[2];
	}
}


/******************************************************************
 * Get position jacobian for specific link index
 *
 * @ J : return position jacobian Transpose (3 X dof)
 ******************************************************************/
void sKinematics::getJacobianPos(int link_index, double **J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if( i> link_index ){
			J[0][ai] = 0.0;
			J[1][ai] = 0.0;
			J[2][ai] = 0.0;
		}
		else{
			if( i == 0 ){
				for( j=0; j<3; j++){
					v1[j] = T0[j][2];
					v2[j] = sDH[link_index].H0i[j][3] - T0[j][3];
				}
			}
			else{
				for( j=0; j<3; j++){
					v1[j] = sDH[i-1].H0i[j][2];
					v2[j] = sDH[link_index].H0i[j][3] - sDH[i-1].H0i[j][3];
				}
			}
			CrossProduct(v1, v2, v3);
			J[0][ai] = v3[0];
			J[1][ai] = v3[1];
			J[2][ai] = v3[2];
		}
	}
}
void sKinematics::getJacobianPos(Eigen::MatrixXd &J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	J.resize(3,dof);
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if( i == 0 ){
			for( j=0; j<3; j++){
				v1[j] = T0[j][2];
				v2[j] = H0F[j][3] - T0[j][3];
			}
		}
		else{
			for( j=0; j<3; j++){
				v1[j] = sDH[i-1].H0i[j][2];
				v2[j] = H0F[j][3] - sDH[i-1].H0i[j][3];
			}
		}
		CrossProduct(v1, v2, v3);
		J(0,ai) = v3[0];
		J(1,ai) = v3[1];
		J(2,ai) = v3[2];
	}
}

/******************************************************************
 * Get position jacobian for specific link index
 *
 * @ J : return position jacobian Transpose (3 X dof)
 ******************************************************************/
void sKinematics::getJacobianPos(int link_index, MathLib::Matrix &J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if( i> link_index ){
			J(0,ai) = 0.0;
			J(1,ai) = 0.0;
			J(2,ai) = 0.0;
		}
		else{
			if( i == 0 ){
				for( j=0; j<3; j++){
					v1[j] = T0[j][2];
					v2[j] = sDH[link_index].H0i[j][3] - T0[j][3];
				}
			}
			else{
				for( j=0; j<3; j++){
					v1[j] = sDH[i-1].H0i[j][2];
					v2[j] = sDH[link_index].H0i[j][3] - sDH[i-1].H0i[j][3];
				}
			}
			CrossProduct(v1, v2, v3);
			J(0,ai) = v3[0];
			J(1,ai) = v3[1];
			J(2,ai) = v3[2];
		}
	}
}


void sKinematics::getJacobianPos(int link_index, Eigen::MatrixXd &J)
{
	int i, j, ai;
		double v1[3], v2[3], v3[3];
		for(ai=0; ai<dof; ai++){
			i=active_index[ai];
			if( i> link_index ){
				J(0,ai) = 0.0;
				J(1,ai) = 0.0;
				J(2,ai) = 0.0;
			}
			else{
				if( i == 0 ){
					for( j=0; j<3; j++){
						v1[j] = T0[j][2];
						v2[j] = sDH[link_index].H0i[j][3] - T0[j][3];
					}
				}
				else{
					for( j=0; j<3; j++){
						v1[j] = sDH[i-1].H0i[j][2];
						v2[j] = sDH[link_index].H0i[j][3] - sDH[i-1].H0i[j][3];
					}
				}
				CrossProduct(v1, v2, v3);
				J(0,ai) = v3[0];
				J(1,ai) = v3[1];
				J(2,ai) = v3[2];
			}
		}
}

void sKinematics::getJacobianPos_fast(int link_index, Eigen::MatrixXd &J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	for(ai=0; ai<link_index; ai++){
		i=active_index[ai];
			if( i == 0 ){
				for( j=0; j<3; j++){
					v1[j] = T0[j][2];
					v2[j] = sDH[link_index].H0i[j][3] - T0[j][3];
				}
			}
			else{
				for( j=0; j<3; j++){
					v1[j] = sDH[i-1].H0i[j][2];
					v2[j] = sDH[link_index].H0i[j][3] - sDH[i-1].H0i[j][3];
				}
			}
			CrossProduct(v1, v2, v3);
			J(0,ai) = v3[0];
			J(1,ai) = v3[1];
			J(2,ai) = v3[2];
	}
}

/******************************************************************
 * Get direction jacobian
 *
 * @ J : return direction jacobian T (3 X dof)
 ******************************************************************/
void sKinematics::getJacobianDirection(int axis, double **J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if( i==0 ){
			for( j=0; j<3; j++){
				v1[j] = T0[j][2];
				v2[j] = H0F[j][axis];
			}
		}
		else{
			for( j=0; j<3; j++){
				v1[j] = sDH[i-1].H0i[j][2];
				v2[j] = H0F[j][axis];
			}
		}
		CrossProduct(v1, v2, v3);
		J[0][ai] = v3[0];
		J[1][ai] = v3[1];
		J[2][ai] = v3[2];
	}
}

/******************************************************************
 * Get direction jacobian
 *
 * @ J : return direction jacobian T (3 X dof)
 ******************************************************************/
void sKinematics::getJacobianDirection(int link_index, int axis, double **J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if( i==0 ){
			for( j=0; j<3; j++){
				v1[j] = T0[j][2];
				v2[j] = sDH[link_index].H0i[j][axis];
			}
		}
		else{
			for( j=0; j<3; j++){
				v1[j] = sDH[i-1].H0i[j][2];
				v2[j] = sDH[link_index].H0i[j][axis];
			}
		}
		CrossProduct(v1, v2, v3);
		J[0][ai] = v3[0];
		J[1][ai] = v3[1];
		J[2][ai] = v3[2];
	}
}


/******************************************************************
 * Get direction jacobian
 *
 * @ J : return direction jacobian T (3 X dof)
 ******************************************************************/
void sKinematics::getJacobianDirection(int link_index, int axis, MathLib::Matrix &J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if( i==0 ){
			for( j=0; j<3; j++){
				v1[j] = T0[j][2];
				v2[j] = sDH[link_index].H0i[j][axis];
			}
		}
		else{
			for( j=0; j<3; j++){
				v1[j] = sDH[i-1].H0i[j][2];
				v2[j] = sDH[link_index].H0i[j][axis];
			}
		}
		CrossProduct(v1, v2, v3);
		J(0,ai) = v3[0];
		J(1,ai) = v3[1];
		J(2,ai) = v3[2];
	}
}

void sKinematics::getJacobianDirection(int axis, Eigen::MatrixXd &J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	J.resize(3,dof);
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if( i==0 ){
			for( j=0; j<3; j++){
				v1[j] = T0[j][2];
				v2[j] = H0F[j][axis];
			}
		}
		else{
			for( j=0; j<3; j++){
				v1[j] = sDH[i-1].H0i[j][2];
				v2[j] = H0F[j][axis];
			}
		}
		CrossProduct(v1, v2, v3);
		J(0,ai) = v3[0];
		J(1,ai) = v3[1];
		J(2,ai) = v3[2];
	}
}

/******************************************************************
 * Get direction jacobian
 *
 * @ J : return direction jacobian T (3 X dof)
 ******************************************************************/
void sKinematics::getJacobianDirection(int axis, MathLib::Matrix &J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if( i==0 ){
			for( j=0; j<3; j++){
				v1[j] = T0[j][2];
				v2[j] = H0F[j][axis];
			}
		}
		else{
			for( j=0; j<3; j++){
				v1[j] = sDH[i-1].H0i[j][2];
				v2[j] = H0F[j][axis];
			}
		}
		CrossProduct(v1, v2, v3);
		J(0,ai) = v3[0];
		J(1,ai) = v3[1];
		J(2,ai) = v3[2];
	}
}


/******************************************************************
 * Get jacobian full direction
 *
 * @ J : return direction jacobian T (9 X dof)
 ******************************************************************/
void sKinematics::getJacobianFullDirection(int axis1, int axis2, MathLib::Matrix &J)
{
	int lColSize = J.ColumnSize();
	MathLib::Matrix lJ3(3,lColSize);

	getJacobianPos(lJ3);
	J.SetRow(lJ3.GetRow(0), 0);
	J.SetRow(lJ3.GetRow(1), 1);
	J.SetRow(lJ3.GetRow(2), 2);

	getJacobianDirection(axis1, lJ3);
	J.SetRow(lJ3.GetRow(0), 3);
	J.SetRow(lJ3.GetRow(1), 4);
	J.SetRow(lJ3.GetRow(2), 5);

	getJacobianDirection(axis2, lJ3);
	J.SetRow(lJ3.GetRow(0), 6);
	J.SetRow(lJ3.GetRow(1), 7);
	J.SetRow(lJ3.GetRow(2), 8);
}



/******************************************************************
 * Get Palm jacobian
 *
 * @ J : return direction jacobian T (6 X dof)
 ******************************************************************/
void sKinematics::getJacobianPalm(int axis, double **J)
{
	int i, j, ai;
	double v1[3], v21[3], v22[3], v3[3];
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if(i==0){
			for( j=0; j<3; j++){
				v1[j]  = T0[j][2];
				v21[j] = H0F[j][3] - T0[j][3];
				v22[j] = H0F[j][axis];
			}
		}
		else{
			for( j=0; j<3; j++){
				v1[j]  = sDH[i-1].H0i[j][2];
				v21[j] = H0F[j][3] - sDH[i-1].H0i[j][3];
				v22[j] = H0F[j][axis];
			}
		}
		CrossProduct(v1, v21, v3);
		J[0][ai] = v3[0];
		J[1][ai] = v3[1];
		J[2][ai] = v3[2];

		CrossProduct(v1, v22, v3);
		J[3][ai] = v3[0];
		J[4][ai] = v3[1];
		J[5][ai] = v3[2];
	}
}

/******************************************************************
 * Get direction jacobian
 *
 * @ J : return geo jacobian (6 X dof)
 ******************************************************************/
void sKinematics::getJacobian(double **J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if(i==0){
			for( j=0; j<3; j++){
				v1[j] = T0[j][2];
				v2[j] = H0F[j][3] - T0[j][3];
			}
			J[3][ai] = T0[0][2];
			J[4][ai] = T0[1][2];
			J[5][ai] = T0[2][2];

		}
		else{
			for( j=0; j<3; j++){
				v1[j] = sDH[i-1].H0i[j][2];
				v2[j] = H0F[j][3] - sDH[i-1].H0i[j][3];
			}
			J[3][ai] = sDH[i-1].H0i[0][2];
			J[4][ai] = sDH[i-1].H0i[1][2];
			J[5][ai] = sDH[i-1].H0i[2][2];
		}
		CrossProduct(v1, v2, v3);
		J[0][ai] = v3[0];
		J[1][ai] = v3[1];
		J[2][ai] = v3[2];
	}
}

/******************************************************************
 * Get direction jacobian
 *
 * @ J : return geo jacobian (6 X dof)
 ******************************************************************/
void sKinematics::getJacobian(MathLib::Matrix& J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if(i==0){
			for( j=0; j<3; j++){
				v1[j] = T0[j][2];
				v2[j] = H0F[j][3] - T0[j][3];
			}
			J(3,ai) = T0[0][2];
			J(4,ai) = T0[1][2];
			J(5,ai) = T0[2][2];

		}
		else{
			for( j=0; j<3; j++){
				v1[j] = sDH[i-1].H0i[j][2];
				v2[j] = H0F[j][3] - sDH[i-1].H0i[j][3];
			}
			J(3,ai) = sDH[i-1].H0i[0][2];
			J(4,ai) = sDH[i-1].H0i[1][2];
			J(5,ai) = sDH[i-1].H0i[2][2];
		}
		CrossProduct(v1, v2, v3);
		J(0,ai) = v3[0];
		J(1,ai) = v3[1];
		J(2,ai) = v3[2];
	}
}

/******************************************************************
 * Get direction jacobian
 *
 * @ J : return geo jacobian (6 X dof)
 ******************************************************************/
void sKinematics::getJacobian(Eigen::MatrixXd& J)
{
	int i, j, ai;
	double v1[3], v2[3], v3[3];
	J.resize(6,dof);
	for(ai=0; ai<dof; ai++){
		i=active_index[ai];
		if(i==0){
			for( j=0; j<3; j++){
				v1[j] = T0[j][2];
				v2[j] = H0F[j][3] - T0[j][3];
			}
			J(3,ai) = T0[0][2];
			J(4,ai) = T0[1][2];
			J(5,ai) = T0[2][2];

		}
		else{
			for( j=0; j<3; j++){
				v1[j] = sDH[i-1].H0i[j][2];
				v2[j] = H0F[j][3] - sDH[i-1].H0i[j][3];
			}
			J(3,ai) = sDH[i-1].H0i[0][2];
			J(4,ai) = sDH[i-1].H0i[1][2];
			J(5,ai) = sDH[i-1].H0i[2][2];
		}
		CrossProduct(v1, v2, v3);
		J(0,ai) = v3[0];
		J(1,ai) = v3[1];
		J(2,ai) = v3[2];
	}
}


/******************************************************************
 * Set Inverse points
 * 
 ******************************************************************/
void sKinematics::initIKpoints(int nPoints, int points_index[], double points_weight[])
{
	int i;
	int row = nPoints*3;

	ik_points_index  = (int *)malloc(nPoints*sizeof(int));
	ik_points_weight = svector(nPoints);
	vweight          = svector(row);

	this->nPoints = nPoints;
	for(i=0; i<nPoints; i++){
		ik_points_index[i]  = points_index[i];
		ik_points_weight[i] = points_weight[i];

		vweight[i*3+0] = points_weight[i];
		vweight[i*3+1] = points_weight[i];
		vweight[i*3+2] = points_weight[i];
	}

	J   = smatrix(3, dof);
	FJ  = smatrix(row, dof);
	FJi = smatrix(dof, row);
}


/******************************************************************
 * Calculate joint velocity
 *   Pseudo inverse matrix using SVD 
 ******************************************************************/
void sKinematics::solveIKpoints(double delta_x[], double delta_q[])
{
	int i, j, k;
	int dim;
	int rank;
	dim =3*nPoints;

	// get Full jacobian matrix for all contratints
	for( i=0; i<nPoints; i++){
		getJacobianPos(ik_points_index[i], J);
		for( j=0; j<3; j++ )
			for( k=0; k<dof; k++ )
				FJ[i*3+j][k] = J[j][k];
	}

	//SVD inverse
	//rank = matsvdinv(FJ, dim, dof, FJi );
	rank = matsvdinvDLS(FJ, 6, dof, 0.01, FJi );

	if( rank < dof ){		
		matmul_vec(FJi, dof, dim, delta_x, delta_q);
	}
	else{
		matmul_vec(FJi, dof, dim, delta_x, delta_q);
	}
}

double sKinematics::solveIKpointsItr(double x_dest[], double q_out[])
{
	int i, j, k;
	double *q_old, *q_curr, *q_delta, *q_init;
	double x[3];
	double *ox_delta;
	double error_old, error;
	int dim = 3*nPoints;
	
	// set maximum error;
	error_old = 100000.0; 

	q_old   = svector(dof);
	q_curr  = svector(dof);
	q_delta = svector(dof);
	q_init  = svector(dof);
	ox_delta= svector(dim);

	getJoints(q_old);	
	matcp_c1(dof, q_old, q_init );
	while(1){
		// get current joint
		getJoints(q_curr);

		// get jacobian and delta x
		for( i=0; i<nPoints; i++){
			getJacobianPos(ik_points_index[i], J);			
			for( j=0; j<3; j++ ){
				for( k=0; k<dof; k++ ){
					FJ[i*3+j][k] = J[j][k];
				}
			}

			getEndPos(ik_points_index[i], x);
			for( j=0; j<3; j++ ) {
				ox_delta[i*3+j] = x_dest[i*3+j]-x[j];
			}
		}

		// calculate error
		error = 0;
		for( i=0; i<dim; i++ ){
			error += ox_delta[i]*ox_delta[i]*vweight[i];
		}
		error = sqrt(error);	
		//printf("%f \n", error );
		
		// exit when the error reach a local minimum
		if( error < IK_TOLERANCE ){
			matcp_c1(dof, q_curr, q_out );
			return error;
		}
		else if( abs(error_old - error)  < IK_TOLERANCE*0.1 ){
			matcp_c1(dof, q_curr, q_out );
			return error;
		}
		else if( error > error_old ){
			matcp_c1(dof, q_old, q_out );
			return error_old;
		}

		// solve IK
		matsvdinvDLS(FJ, dim, dof, 0.5, FJi );
		matmul_vec(FJi, dof, dim, ox_delta, q_delta );

		matcp_c1(dof, q_curr, q_old );    // backup joint angles
		matadd_c1(dof, q_delta, q_curr ); // set current joint

		checkVelocityLimit(q_init, q_curr, deltaT);
		setJoints(q_curr);
		error_old = error;
	}
	return error;
}

/******************************************************************
 * Initialize IK Palm
 * @lamda : weight for null space motion
 ******************************************************************/
void sKinematics::initIKPalm(double lamda)
{
	JX  = smatrix(3, dof );
	JP  = smatrix(3, dof );
	OJ  = smatrix(6, dof );
	OJi = smatrix(dof, 6 );
	N   = smatrix(dof, dof);
	this->lamda = lamda;
}

/******************************************************************
 * Solve Inverse Kinematics
 * @deltaX : delta for end-effector position < R 3
 * @deltaP : delta for palm direction  < R 3
 * @restQ  : rest joint angles 
 * @deltaQ : output joint angles 
 ******************************************************************/
void sKinematics::solveIKPalm(double delta_x[], int axis, double delta_dir[], double q_rest[], double delta_q[])
{	
	int i;
	double in[6];
	double *q, *q_null;

	q      = svector(dof);
	q_null = svector(dof);	

	getJacobianPalm(axis, OJ );
	for( i=0; i<3; i++ ){
		in[i  ] = delta_x[i];
		in[i+3] = delta_dir[i];
	}
	
	//matsvdinv(OJ, 6, dof, OJi );
	matsvdinvDLS(OJ, 6, dof, 0.01, OJi );	
	matmul_vec(OJi, dof, 6, in, delta_q );

	// pseudo null space motion
	getJoints(q);
	matsub_c1(dof, q_rest, q, q_null );
	for(i=0; i<dof; i++ ) q_null[i] *= lamda;
	matnull(6, dof, OJ, OJi, N);
	matmul_vec( N, dof, dof, q_null, q ); 
	matadd_c1(dof, q, delta_q );
}


/******************************************************************
 * Solve Inverse Kinematics
 ******************************************************************/
double sKinematics::solveIKPalmItr(double x_dest[], int axis, double dir_dest[], double q_rest[], double q_out[])
{	
	int i,j;
	double *q_old, *q_curr, *q_delta, *q_null;
	double x[3], dir[3];
	double ox[6];
	double error_old, error;

	error_old = 1000.0; // set maximum error;

	q_old  = svector(dof);
	q_curr = svector(dof);
	q_delta= svector(dof);
	q_null = svector(dof);

	while(1){
		getJacobianPalm(axis, OJ );
		getEndPos(x);
		getEndDirAxis(axis, dir);
		getJoints(q_curr);

		for( i=0; i<3; i++ ){
			ox[i  ] = x_dest[i]-x[i];
			ox[i+3] = dir_dest[i] - dir[i];
		}
		error = vnorm(6, ox );
		//printf("%f \n", error );

		// stop when the error reach to a local minimum
		if( error < IK_TOLERANCE ){
			matcp_c1(dof, q_curr, q_out );
			return error;
		}
		else if( abs(error_old - error)  < IK_TOLERANCE ){
			matcp_c1(dof, q_curr, q_out );
			return error;
		}
		else if( error > error_old ){
			matcp_c1(dof, q_old, q_out );
			return error_old;
		}

		matcp_c1(dof, q_curr, q_old );

		// solve inverse kinematics
		matsvdinvDLS(OJ, 6, dof, 0.01, OJi ); 
		// set weight
		for( i=0; i<dof; i++ ){
			for( j=0; j<6; j++ ){
				OJi[i][j] *= sDH[active_index[i]].weight;
			}
		}

		matnull(6, dof, OJ, OJi, N );               
		matsub_c1(dof, q_rest, q_curr, q_null );
		for(i=0; i<dof; i++ ) q_null[i] *= lamda;

		matmul_vec(OJi, dof, 6, ox, q_delta );
		matadd_c1(dof, q_delta, q_curr );

		matmul_vec( N, dof, dof, q_null, q_delta ); 
		matadd_c1(dof, q_delta, q_curr );           


		checkVelocityLimit(q_old, q_curr, deltaT);
		setJoints(q_curr);
		
		getEndPos(x);
		getEndDirAxis(axis, dir);
		for( i=0; i<3; i++ ){
			ox[i  ] = x_dest[i]-x[i];
			ox[i+3] = dir_dest[i] - dir[i];
		}
		error = vnorm(6, ox );
		error_old = error;		
		
    }
}

MathLib::Vector3 sKinematics::calAngularVelocity(MathLib::Matrix3 &A, MathLib::Matrix3 &An, double dt)
{
	MathLib::Matrix3 W = (An-A)/dt * A.Transpose();
	MathLib::Vector3 omega;

	omega(2) = ( W(1,0)-W(0,1))/2.;
	omega(1) = (-W(2,0)+W(0,2))/2.;
	omega(0) = ( W(2,1)-W(1,2))/2.;
	return omega;
}

double sKinematics::CalAngle(MathLib::Vector3 vBase, MathLib::Vector3 vMeasure)
{
	double theta;
	MathLib::Vector3 vOrtho;
	MathLib::Vector3 vDir;
	theta = acos( vBase.Dot(vMeasure) / (vBase.Norm()*vMeasure.Norm()) );

	vOrtho = vBase.Cross(vMeasure);
	vOrtho.Normalize();
	vDir = vOrtho.Cross(vBase);

	if( vDir.Dot(vMeasure) > 0 ) return theta;
	else return -theta;
}
