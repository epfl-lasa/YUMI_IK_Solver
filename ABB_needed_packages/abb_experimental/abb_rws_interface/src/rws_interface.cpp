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

#include <algorithm>

#include "abb_rws_interface/rws_interface.h"

using namespace abb::rws_interface::rws_strings;
using namespace abb::rws_interface::rws_strings::user_defined::rapid::symbols;

namespace abb
{
namespace rws_interface
{
/****************************************************************************************
 * Class definitions: RWSInterface
 */

/********************************************
 * Primary methods
 */

bool RWSInterface::isRAPIDRunning()
{
  return (rws_client_.getRAPIDExecution() == system_defined::controller_states::RAPID_EXECUTION_RUNNING);
}

bool RWSInterface::isModeAuto()
{
  return (rws_client_.getPanelOperationMode() == system_defined::controller_states::PANEL_OPERATION_MODE_AUTO);
}

bool RWSInterface::isRobotIdle(const std::string task_name)
{
  bool success = false;

  std::string server_reply = rws_client_.getRAPIDSymbolData(task_name, 
                                                            RAPID_SYMBOL_CURRENT_ACTION.module_name, 
                                                            RAPID_SYMBOL_CURRENT_ACTION.symbol_name);
  if (!server_reply.empty())
  {
    success = (server_reply == user_defined::rapid::values::ACTION_IDLE);
  }

  return success;
}

bool RWSInterface::getData(const std::string task_name, JointData* p_data)
{
  return getData(task_name, RAPID_SYMBOL_CURRENT_JOINTS, p_data);
}

bool RWSInterface::getData(const std::string task_name, EGMData* p_data)
{
  return getData(task_name, RAPID_SYMBOL_EGM_DATA, p_data);
}

void RWSInterface::setData(const std::string task_name, EGMData data)
{
  setData(task_name, RAPID_SYMBOL_EGM_DATA, data.constructString());
}

bool RWSInterface::doEGMStartJoint()
{
  return toggleIOSignal(user_defined::io::signals::IO_EGM_START_JOINT);
}

bool RWSInterface::doEGMStartPose()
{
  return toggleIOSignal(user_defined::io::signals::IO_EGM_START_POSE);
}

bool RWSInterface::doEGMStop()
{
  return toggleIOSignal(user_defined::io::signals::IO_EGM_STOP);
}

bool RWSInterface::doGoToHomePosition()
{
  return toggleIOSignal(user_defined::io::signals::IO_GO_TO_HOME_POSITION);
}

/********************************************
 * Auxiliary methods
 */

bool RWSInterface::getData(const std::string task_name, RAPIDSymbol symbol, RAPIDRecord* p_data)
{
  bool success = false;

  std::string server_reply = rws_client_.getRAPIDSymbolData(task_name, 
                                                            symbol.module_name, 
                                                            symbol.symbol_name);
  std::replace(server_reply.begin(), server_reply.end(), '[', ',');
  std::replace(server_reply.begin(), server_reply.end(), ']', ',');

  if (!server_reply.empty())
  {
    p_data->parseStringValues(rws_client_.extractDelimitedValues(server_reply));
    success = true;
  }

  return success;
}

void RWSInterface::setData(const std::string task_name, RAPIDSymbol symbol, std::string data)
{
  if (isModeAuto())
  {
    rws_client_.setRAPIDSymbolData(task_name,
                                   symbol.module_name,
                                   symbol.symbol_name,
                                   data);
  }
}

bool RWSInterface::toggleIOSignal(const std::string io_signal)
{
  int max_number_of_attempts = 5;
  bool success = false;

  if (isRAPIDRunning() && isModeAuto())
  {
    for (int i = 0; i < max_number_of_attempts && !success; ++i)
    {
      rws_client_.setIOSignal(io_signal, system_defined::io::values::DI_SIGNAL_LOW);
      if (rws_client_.getIOSignal(io_signal) == system_defined::io::values::DI_SIGNAL_LOW)
      {
        success = true;
      }
    }

    if (success)
    {
      success = false;

      for (int i = 0; i < max_number_of_attempts && !success; ++i)
      {
        rws_client_.setIOSignal(io_signal, system_defined::io::values::DI_SIGNAL_HIGH);
        if (rws_client_.getIOSignal(io_signal) == system_defined::io::values::DI_SIGNAL_HIGH)
        {
          success = true;
        }
      }
    }
  }

  return success;
}

} // end namespace rws_interface
} // end namespace abb
