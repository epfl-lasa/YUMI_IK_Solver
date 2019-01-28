/***********************************************************************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, ABB
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#include <boost/numeric/ublas/matrix.hpp>

#include "abb_egm_interface/egm_common_math.h"

namespace abb
{
namespace egm_interface
{

using namespace egm_common_values::conversions;

/****************************************************************************************
 * Math functions
 */

double dotProduct(const proto::Quaternion q1, const proto::Quaternion q2)
{
  return q1.u0()*q2.u0() + q1.u1()*q2.u1() + q1.u2()*q2.u2() + q1.u3()*q2.u3();
}

double euclideanNorm(const proto::Quaternion q)
{
  return sqrt(q.u0()*q.u0() + q.u1()*q.u1() + q.u2()*q.u2() + q.u3()*q.u3());
}

void normalizeQuaternion(proto::Quaternion* p_q)
{
  double norm = euclideanNorm(*p_q);
  p_q->set_u0(p_q->u0() / norm);
  p_q->set_u1(p_q->u1() / norm);
  p_q->set_u2(p_q->u2() / norm);
  p_q->set_u3(p_q->u3() / norm);
}

void eulerZYXToQuaternion(const proto::Euler e, proto::Quaternion* p_q)
{
  double z = e.z() * DEG_TO_RAD;
  double y = e.y() * DEG_TO_RAD;
  double x = e.x() * DEG_TO_RAD;

  double c1 = cos(z);
  double s1 = sin(z);
  double c2 = cos(y);
  double s2 = sin(y);
  double c3 = cos(x);
  double s3 = sin(x);

  /*
   * Convert ZYX Euler angles to a rotation matrix.
   * See https://en.wikipedia.org/wiki/Euler_angles for more informaiton.
   */
  boost::numeric::ublas::matrix<double> rm(3, 3);

  rm(0, 0) = c1*c2;
  rm(0, 1) = c1*s2*s3 - c3*s1;
  rm(0, 2) = s1*s3 + c1*c3*s2;

  rm(1, 0) = c2*s1;
  rm(1, 1) = c1*c3 + s1*s2*s3;
  rm(1, 2) = c3*s1*s2 - c1*s3;

  rm(2, 0) = -s2;
  rm(2, 1) = c2*s3;
  rm(2, 2) = c2*c3;

  /*
   * Rotaion matrix to quaternion.
   * See https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions for more information.
   * 
   * Handle cases to reduce numerical inaccuracy.
   * See http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/ for more information.
   */
  double trace = rm(0, 0) + rm(1, 1) + rm(2, 2);
  double u0 = 0.0;
  double u1 = 0.0;
  double u2 = 0.0;
  double u3 = 0.0;
  double a = 0.0;

  double eps = std::numeric_limits<double>::epsilon();
  if (trace > eps)
  {
    a = 2.0*sqrt(1.0 + trace);
    u0 = 0.25*a;
    u1 = (rm(2, 1) - rm(1, 2)) / a;
    u2 = (rm(0, 2) - rm(2, 0)) / a;
    u3 = (rm(1, 0) - rm(0, 1)) / a;
  }
  else if (rm(0, 0) > rm(1, 1) + eps && rm(0, 0) > rm(2, 2) + eps)
  {
    a = 2.0*sqrt(1.0 + rm(0, 0) - rm(1, 1) - rm(2, 2));
    u0 = (rm(2, 1) - rm(1, 2)) / a;
    u1 = 0.25*a;
    u2 = (rm(0, 1) + rm(1, 0)) / a;
    u3 = (rm(0, 2) + rm(2, 0)) / a;
  }
  else if (rm(1, 1) > rm(2, 2) + eps)
  {
    a = 2.0*sqrt(1.0 + rm(1, 1) - rm(0, 0) - rm(2, 2));
    u0 = (rm(0, 2) - rm(2, 0)) / a;
    u1 = (rm(0, 1) + rm(1, 0)) / a;
    u2 = 0.25*a;
    u3 = (rm(1, 2) + rm(2, 1)) / a;
  }
  else
  {
    a = 2.0*sqrt(1.0 + rm(2, 2) - rm(0, 0) - rm(1, 1));
    u0 = (rm(1, 0) - rm(0, 1)) / a;
    u1 = (rm(0, 2) + rm(2, 0)) / a;
    u2 = (rm(1, 2) + rm(2, 1)) / a;
    u3 = 0.25*a;
  }

  // Set the quaternion and normalize.
  p_q->set_u0(u0);
  p_q->set_u1(u1);
  p_q->set_u2(u2);
  p_q->set_u3(u3);

  normalizeQuaternion(p_q);
}

void quaternionToEulerZYX(const proto::Quaternion q, proto::Euler* p_e)
{
  /*
   * Convert a quaternion to ZYX Euler angles.
   * See http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
   * for more information.
   */

  double u0 = q.u0();
  double u1 = q.u1();
  double u2 = q.u2();
  double u3 = q.u3();

  double y = 0.0;
  double z = 0.0;
  double x = 0.0;

  double test = u0*u2 - u1*u3;

  if (test > 0.499)
  {
    x = 2.0*std::atan2(u1, u0);
    y = M_PI_2;
  }
  else if (test < -0.499)
  {
    x = -2.0*std::atan2(u1, u0);
    y = -M_PI_2;
  }
  else
  {
    x = std::atan2(2.0*(u0*u1 + u2*u3), 1.0 - 2.0*(u1*u1 + u2*u2));
    y = std::asin(2.0*(u0*u2 - u1*u3));
    z = std::atan2(2.0*(u0*u3 + u1*u2), 1.0 - 2.0*(u2*u2 + u3*u3));
  }

  p_e->set_x(x*RAD_TO_DEG);
  p_e->set_y(y*RAD_TO_DEG);
  p_e->set_z(z*RAD_TO_DEG);
}

} // end namespace egm_interface
} // end namespace abb
