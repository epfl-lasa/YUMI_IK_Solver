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

#ifndef EGM_COMMON_MATH_H
#define EGM_COMMON_MATH_H

#include "egm_common.h"

namespace abb
{
namespace egm_interface
{
/****************************************************************************************
 * Math functions
 */

/**
 * \brief A function to calculate the dot product between two quaternions.
 *
 * \param q1 for the first quaternion.
 * \param q2 for the second quaternion.
 *
 * \return double containing the dot product value.
 */
double dotProduct(const proto::Quaternion q1, const proto::Quaternion q2);

/**
 * \brief A function to calculate the Euclidean norm of a quaternion.
 *
 * \param q for the quaternion to calculate the norm of.
 *
 * \return double containing the Euclidean norm value.
 */
double euclideanNorm(const proto::Quaternion q);

/**
 * \brief A function to normalize a quaternion.
 *
 * \param p_q for the quaternion to normalize.
 */
void normalizeQuaternion(proto::Quaternion* p_q);

/**
 * \brief A function to convert ZYX Euler angles to a quaternion.
 *
 * \param e for the ZYX Euler angles.
 * \param p_q for the calculated quaternion.
 */
void eulerZYXToQuaternion(const proto::Euler e, proto::Quaternion* p_q);

/**
 * \brief A function to convert a quaternion to ZYX Euler angles.
 *
 * \param q for the quaternion.
 * \param p_e for the calculated ZYX Euler angles.
 */
void quaternionToEulerZYX(const proto::Quaternion q, proto::Euler* p_e);

} // end namespace egm_interface
} // end namespace abb

#endif // EGM_COMMON_MATH_H
