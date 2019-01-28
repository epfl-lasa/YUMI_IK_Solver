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

#ifndef EGM_COMMON_AUXILIARY_H
#define EGM_COMMON_AUXILIARY_H

#include "egm_common.h"

namespace abb
{
namespace egm_interface
{
/****************************************************************************************
 * Auxiliary functions
 */

/**
 * \brief Truncate robot joints.
 *
 * \param joints containing the joints to truncate.
 * \param size for the targeted number of joints.
 */
void truncate(proto::JointSpace* joints, const int size);

/**
 * \brief Fill out robot joints with position data.
 *
 * \param joints containing the joints to fill out.
 * \param size for the targeted number of joints.
 * \param feedback containing the feedback values from the robot controller.
 */
void fillPosition(proto::JointSpace* joints,
                  const int size,
                  const proto::JointSpace feedback);

/**
 * \brief Fill out robot joints with speed and acceleration data.
 *
 * \param joints containing the joints to fill out.
 * \param size for the targeted number of joints.
 */
void fillSpeedAndAcceleration(proto::JointSpace* joints, const int size);

/**
 * \brief Truncate external joints.
 *
 * \param joints containing the joints to truncate.
 * \param size for the targeted number of joints.
 */
void truncateExternal(proto::JointSpace* joints, const int size);

/**
 * \brief Fill out external joints with position data.
 *
 * \param joints containing the joints to fill out.
 * \param size for the targeted number of joints.
 * \param feedback containing the feedback values from the robot controller.
 */
void fillExternalPosition(proto::JointSpace* joints,
                          const int size,
                          const proto::JointSpace feedback);

/**
 * \brief Fill out external joints with speed and acceleration data.
 *
 * \param joints containing the joints to fill out.
 * \param size for the targeted number of joints.
 */
void fillExternalSpeedAndAcceleration(proto::JointSpace* joints, const int size);

/**
 * \brief Fill out Cartesian data with position data.
 *
 * \param cartesian containing the data to fill out.
 * \param feedback containing the feedback values from the robot controller.
 */
void fillPosition(proto::Cartesian* cartesian,
                  const proto::Cartesian feedback);

/**
 * \brief Fill out quaternion orientation data.
 *
 * \param quaternion containing the data to fill out.
 * \param feedback containing the feedback values from the robot controller.
 */
void fillQuaternion(proto::Quaternion* quaternion,
                    const proto::Quaternion feedback);

/**
 * \brief Fill out Cartesian speed and acceleration data.
 *
 * \param cartesian containing the data to fill out.
 */
void fillSpeedAndAcceleration(proto::CartesianSpace* cartesian);

/**
 * \brief Validate joint space data (fills out missing inputs).
 *
 * \param input_data the input data to validate.
 * \param feedback containing feedback data for filling out missing data.
 */
void validateInputData(proto::JointSpace* input_data, const proto::JointSpace feedback);

/**
 * \brief Validate cartesian space data (fills out missing inputs).
 *
 * \param input_data the input data to validate.
 * \param feedback containing feedback data for filling out missing data.
 */
void validateInputData(proto::CartesianSpace* input_data, const proto::CartesianSpace feedback);

/**
 * \brief Estimate joint space speed data.
 *
 * \param estimated_speed containg the estimated joint speed.
 * \param current_joints container for the current joint positions.
 * \param previous_joints container for the previous joint positions.
 * \param sample_time for the sample time.
 */
void estimateSpeed(proto::JointSpace* estimated_speed,
                   const proto::JointSpace current_joints,
                   const proto::JointSpace previous_joints,
                   const double sample_time);

/**
 * \brief Estimate Cartesian space speed data.
 *
 * \param estimated_speed containg the estimated TCP and angular velocities.
 * \param current_joints container for the current Cartesian position and orientation.
 * \param previous_joints container for the previous Cartesian position and orientation.
 * \param sample_time for the sample time.
 */
void estimateSpeed(proto::CartesianSpeed* estimated_speed,
                   const proto::CartesianSpace current_cartesian,
                   const proto::CartesianSpace previous_cartesian,
                   const double sample_time);

} // end namespace egm_interface
} // end namespace abb

#endif // EGM_COMMON_AUXILIARY_H
