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

#ifndef RWS_INTERFACE_YUMI_H
#define RWS_INTERFACE_YUMI_H

#include "rws_interface.h"
#include "rws_interface_yumi_common.h"

namespace abb
{
namespace rws_interface
{
/**
 * \brief A class for a Robot Web Services (RWS) interface specifically for dual armed robots (i.e. YuMi).
 *
 * Note: This class assumes that certain corresponding RAPID symbol(s) and IO signal(s)
 *       has been defined on the robot controller side. I.e. the robot controller has been
 *       properly configured.
 */
class RWSInterfaceYuMi : public RWSInterface
{
public:
  /**
   * \brief A constructor.
   *
   * \param ip_address specifying the robot controller's IP address.
   * \param port specifying the robot controller's RWS server's port number.
   */
  RWSInterfaceYuMi(const std::string ip_address, const std::string port = "80")
  :
  RWSInterface(ip_address, port)
  {
  }
  
  /**
   * \brief Waits until the RAPID tasks are idle.
   *
   * \param side specifying left, right or both arms.
   */
  void waitUntilIdle(Side side = BOTH_SIDES);
  
  /**
   * \brief Get a RAPID symbol containing joint data.
   *
   * \param task_name for specifying the RAPID task.
   * \param p_data for containing the retrived data.
   *
   * \return bool indicating that the data was retrived.
   */
  bool getData(const std::string task_name, JointData* p_data) { return RWSInterface::getData(task_name, p_data);}
  
  /**
   * \brief Get a RAPID symbol containing EGM data.
   *
   * \param task_name for specifying the RAPID task.
   * \param p_data for containing the retrived data.
   *
   * \return bool indicating that the data was retrived.
   */
  bool getData(const std::string task_name, EGMData* p_data) {return RWSInterface::getData(task_name, p_data);}
  
  /**
   * \brief Get RAPID symbols containing joint data.
   *
   * \param p_data for containing the retrived data.
   * \param side specifying left, right or both arms.
   *
   * \return bool indicating that the data was retrived.
   */
  bool getData(DualJointData* p_data, Side side = BOTH_SIDES);
  
  /**
   * \brief Get RAPID symbols containing EGM data.
   *
   * \param p_data for containing the retrived data.
   * \param side specifying left, right or both arms.
   *
   * \return bool indicating that the data was retrived.
   */
  bool getData(DualEGMData* p_data, Side side = BOTH_SIDES);
  
  /**
   * \brief Get RAPID symbols containing smart gripper (SG) data.
   *
   * \param p_data for containing the retrived data.
   * \param side specifying left, right or both arms.
   *
   * \return bool indicating that the data was retrived.
   */
  bool getData(DualSGData* p_data, Side side = BOTH_SIDES);
  
  /**
   * \brief Get RAPID symbols containing camera data.
   *
   * \param p_data for containing the retrived data.
   * \param side specifying left, right or both arms.
   *
   * \return bool indicating that the data was retrived.
   */
  bool getData(DualCameraData* p_data, Side side = BOTH_SIDES);
  
  /**
   * \brief Set a RAPID symbol containing EGM data.
   *
   * \param task_name for specifying the RAPID task.
   * \param data containing the data to set.
   */
  void setData(const std::string task_name, EGMData data) {RWSInterface::setData(task_name, data);}
  
  /**
   * \brief Set RAPID symbols containing EGM data.
   *
   * \param data containing the data to set.
   * \param side specifying left, right or both arms.
   */
  void setData(DualEGMData data, Side side = BOTH_SIDES);
  
  /**
   * \brief Set RAPID symbols containing smart gripper (SG) data.
   *
   * \param data containing the data to set.
   * \param side specifying left, right or both arms.
   */
  void setData(DualSGData data, Side side = BOTH_SIDES);

  /**
   * \brief Set RAPID symbols containing camera data.
   *
   * \param data containing the data to set.
   * \param side specifying left, right or both arms.
   */
  void setData(DualCameraData data, Side side = BOTH_SIDES);
  
  /**
   * \brief Get the calibration status of a smart gripper.
   *
   * \param gripper specifying left or right gripper.
   *
   * \return bool indicating if the gripper is calibrated.
   */
  bool getSGCalibratedStatus(Gripper gripper);
  
  /**
   * \brief Get the position [mm] of a smart gripper.
   *
   * \param gripper specifying left or right gripper.
   *
   * \return float contaning the position.
   */
  float getSGActualPosition(Gripper gripper);
  
  /**
   * \brief Get the speed [mm/s] of a smart gripper.
   *
   * \param gripper specifying left or right gripper.
   *
   * \return float contaning the speed.
   */
  float getSGActualSpeed(Gripper gripper);
  
  /**
   * \brief Trigger an initialization of the smart gripper(s).
   *
   * \param side specifying left, right or both arms.
   */
  void doSGInitialize(Side side = BOTH_SIDES);
  
  /**
   * \brief Trigger an calibration of the smart gripper(s).
   *
   * \param side specifying left, right or both arms.
   */
  void doSGCalibrate(Side side = BOTH_SIDES);
  
  /**
   * \brief Trigger a move for the smart gripper(s).
   *
   * \param side specifying left, right or both arms.
   */
  void doSGMove(Side side = BOTH_SIDES);
  
  /**
   * \brief Move the smart gripper(s).
   *
   * \param target specifying the target [mm].
   * \param wait indicating if the method should wait for the move to be finished.
   * \param side specifying left, right or both arms.
   */
  void doSGMoveTo(const float target, bool wait, Side side = BOTH_SIDES);
  
  /**
   * \brief Move the smart gripper(s).
   *
   * \param left_target specifying the  left target [mm].
   * \param right_target specifying the right target [mm].
   * \param wait indicating if the method should wait for the move to be finished.
   * \param side specifying left, right or both arms.
   */
  void doSGMoveTo(const float left_target, const float right_target, bool wait, Side side = BOTH_SIDES);
  
  /**
   * \brief Trigger a gripping for the smart gripper(s).
   *
   * \param side specifying left, right or both arms.
   */
  void doSGGrip(Side side = BOTH_SIDES);
  
  /**
   * \brief Grip with the smart gripper(s).
   *
   * \param target specifying the target [mm].
   * \param force indicating the gripping force [N].
   * \param side specifying left, right or both arms.
   */
  void doSGGripTo(const float target, const float force, Side side = BOTH_SIDES);
  
  /**
   * \brief Grip with the smart gripper(s).
   *
   * \param left_target specifying the left target [mm].
   * \param left_force indicating the left gripping force [N].
   * \param right_target specifying the right target [mm].
   * \param right_force indicating the right gripping force [N].
   * \param side specifying left, right or both arms.
   */
  void doSGGripTo(const float left_target,
                  const float left_force,
                  const float right_target,
                  const float right_force,
                  Side side = BOTH_SIDES);

  /**
   * \brief Trigger a request image from the smart gripper(s) camera.
   *
   * \param side specifying left, right or both arms.
   */
  void doCameraRequestImage(Side side = BOTH_SIDES);
  
  /**
   * \brief Set the camera(s) exposure time.
   *
   * \param data containing the camera data.
   * \param side specifying left, right or both arms.
   */
  void doCameraSetExposure(DualCameraData data, Side side = BOTH_SIDES);

protected:
  /**
   * \brief Get dual RAPID data.
   *
   * \param p_left_data containing the retrived left data.
   * \param p_right_data containing the retrived right data.
   * \param symbol specifying the RAPID symbol.
   * \param side specifying left, right or both arms.
   *
   * \return bool indicating if the data was retrived.
   */
  bool getDualData(RAPIDRecord* p_left_data, RAPIDRecord* p_right_data, RAPIDSymbol symbol, Side side);
  
  /**
   * \brief Set dual RAPID data.
   *
   * \param data containing the data.
   * \param symbol specifying the RAPID symbol.
   * \param side specifying left, right or both arms.
   */
  void setDualData(std::string data, RAPIDSymbol symbol, Side side);
  
  /**
   * \brief Set dual RAPID data.
   *
   * \param left_data containing the left data.
   * \param right_data containing the right data.
   * \param symbol specifying the RAPID symbol.
   * \param side specifying left, right or both arms.
   */
  void setDualData(std::string left_data, std::string right_data, RAPIDSymbol symbol, Side side);
  
  /**
   * \brief Set camera(s) to use.
   *
   * \param side specifying left, right or both arms.
   */
  void setUseCameraData(Side side);
  
  /**
   * \brief Convert a string to a float value.
   *
   * \param input_string containing the string to convert.
   *
   * \return float containing the float value.
   */
  float stringToFloat(std::string input_string);
};

} // end namespace rws_interface
} // end namespace abb

#endif