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

#ifndef RWS_INTERFACE_H
#define RWS_INTERFACE_H

#include "rws_client.h"
#include "rws_interface_common.h"

namespace abb
{
namespace rws_interface
{
/**
 * \brief A class for wrapping a Robot Web Services (RWS) client in a more user friendly interface.
 *
 * Note: This class assumes that certain corresponding RAPID symbol(s) and IO signal(s)
 *       has been defined on the robot controller side. I.e. the robot controller has been
 *       properly configured.
 */
class RWSInterface
{
public:
  /**
   * \brief A constructor.
   *
   * \param ip_address specifying the robot controller's IP address.
   * \param port specifying the robot controller's RWS server's port number.
   */
  RWSInterface(const std::string ip_address, const std::string port = "80")
  :
  rws_client_(ip_address, port)
  {
  }
  
  /**
   * \brief Check if RAPID is running on the robot controller.
   *
   * \return bool indicating if RAPID is running.
   */
  bool isRAPIDRunning();

  /**
   * \brief Check if robot controller is in auto mode.
   *
   * \return bool indicating if auto mode is active.
   */
  bool isModeAuto();
  
  /**
   * \brief Check if the specified robot controller's RAPID task is idle.
   *
   * \param task_name for specifying the RAPID task.
   *
   * \return bool indicating if the RAPID task is idle.
   */
  bool isRobotIdle(const std::string task_name);
  
  /**
   * \brief Get a RAPID symbol containing joint data.
   *
   * \param task_name for specifying the RAPID task.
   * \param p_data for containing the retrived data.
   *
   * \return bool indicating that the data was retrived.
   */
  bool getData(const std::string task_name, JointData* p_data);
  
  /**
   * \brief Get a RAPID symbol containing EGM data.
   *
   * \param task_name for specifying the RAPID task.
   * \param p_data for containing the retrived data.
   *
   * \return bool indicating that the data was retrived.
   */
  bool getData(const std::string task_name, EGMData* p_data);
  
  /**
   * \brief Set a RAPID symbol containing EGM data.
   *
   * \param task_name for specifying the RAPID task.
   * \param data containing the data to set.
   */
  void setData(const std::string task_name, EGMData data);
  
  /**
   * \brief Signal an EGM joint mode start.
   *
   * \return bool indicating if the signaling was successful.
   */
  bool doEGMStartJoint();

  /**
  * \brief Signal an EGM pose mode start.
  *
  * \return bool indicating if the signaling was successful.
  */
  bool doEGMStartPose();

  /**
   * \brief Signal a EGM stop.
   *
   * \return bool indicating if the signaling was successful.
   */
  bool doEGMStop();

  /**
   * \brief Signal the robot to go to the home position.
   *
   * \return bool indicating if the signaling was successful.
   */
  bool doGoToHomePosition();

protected:
  /**
   * \brief Get a RAPID symbol.
   *
   * \param task_name for specifying the RAPID task.
   * \param symbol for specifying the RAPID symbol.
   * \param p_data for containing the retrived data.
   *
   * \return bool indicating if the data was retrived. 
   */
  bool getData(const std::string task_name, RAPIDSymbol symbol, RAPIDRecord* p_data);
  
  /**
   * \brief Set a RAPID symbol.
   *
   * \param task_name for specifying the RAPID task.
   * \param symbol for specifying the RAPID symbol.
   * \param data containing the data to set.
   */
  void setData(const std::string task_name, RAPIDSymbol symbol, std::string data);
  
  /**
   * \brief Toggles an IO signal.
   *
   * \param io_signal specifying the IO signal to toggle.
   *
   * \return bool indicating if the toggling was successful.
   */
  bool toggleIOSignal(const std::string io_signal);
  
  /**
   * \brief The RWS client used to communicate with the robot controller.
   */
  RWSClient rws_client_;
};

} // end namespace rws_interface
} // end namespace abb

#endif