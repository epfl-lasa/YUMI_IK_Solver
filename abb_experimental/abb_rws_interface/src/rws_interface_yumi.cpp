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

#include <cmath>
#include <sstream>

#include "abb_rws_interface/rws_interface_yumi.h"

using namespace abb::rws_interface::rws_strings;

namespace abb
{
namespace rws_interface
{
/****************************************************************************************
 * Class definitions: RWSInterfaceYuMi
 */

/********************************************
 * Primary methods
 */

void RWSInterfaceYuMi::waitUntilIdle(Side side)
{
  if (isModeAuto() && isRAPIDRunning())
  {
    switch (side)
    {
      case LEFT_SIDE:
      {
        while (!isRobotIdle(system_defined::rapid::task_names::RAPID_TASK_ROB_L))
        {}
      }
      break;
      
      case RIGHT_SIDE:
      {
        while (!isRobotIdle(system_defined::rapid::task_names::RAPID_TASK_ROB_R))
        {}
      }
      break;

      case BOTH_SIDES:
      {

        while (!isRobotIdle(system_defined::rapid::task_names::RAPID_TASK_ROB_L) || 
               !isRobotIdle(system_defined::rapid::task_names::RAPID_TASK_ROB_R))
        {};
      }
      break;
    }
  }
}

bool RWSInterfaceYuMi::getData(DualJointData* p_data, Side side)
{
  bool result = getDualData(&(p_data->left),
                            &(p_data->right),
                            user_defined::rapid::symbols::RAPID_SYMBOL_CURRENT_JOINTS,
                            side);

  // Remap the YuMi joints to the logical order.
  if (result)
  {
    float temp = p_data->left.getExt1();
    p_data->left.setExt1(p_data->left.getRob6());
    p_data->left.setRob6(p_data->left.getRob5());
    p_data->left.setRob5(p_data->left.getRob4());
    p_data->left.setRob4(p_data->left.getRob3());
    p_data->left.setRob3(temp);

    temp = p_data->right.getExt1();
    p_data->right.setExt1(p_data->right.getRob6());
    p_data->right.setRob6(p_data->right.getRob5());
    p_data->right.setRob5(p_data->right.getRob4());
    p_data->right.setRob4(p_data->right.getRob3());
    p_data->right.setRob3(temp);
  }

  return result;
}

bool RWSInterfaceYuMi::getData(DualEGMData* p_data, Side side)
{
  return getDualData(&(p_data->left),
                     &(p_data->right),
                     user_defined::rapid::symbols::RAPID_SYMBOL_EGM_DATA,
                     side);
}

bool RWSInterfaceYuMi::getData(DualSGData* p_data, Side side)
{
  return getDualData(&(p_data->left),
                     &(p_data->right),
                     user_defined::rapid::symbols::RAPID_SYMBOL_SG_DATA,
                     side);
}

bool RWSInterfaceYuMi::getData(DualCameraData* p_data, Side side)
{
  bool result = false;

  std::string task = user_defined::rapid::task_names::RAPID_TASK_CAMERA;

  switch (side)
  {
    case LEFT_SIDE:
    {
      result = RWSInterface::getData(task,
                                     user_defined::rapid::symbols::RAPID_SYMBOL_CAMERA_DATA_LEFT,
                                     &(p_data->left));
    }
    break;

    case RIGHT_SIDE:
    {
      result = RWSInterface::getData(task,
                                     user_defined::rapid::symbols::RAPID_SYMBOL_CAMERA_DATA_RIGHT,
                                     &(p_data->right));
    }
    break;

    case BOTH_SIDES:
    {
      if (RWSInterface::getData(task,
                                user_defined::rapid::symbols::RAPID_SYMBOL_CAMERA_DATA_LEFT,
                                &(p_data->left)))
      {
        result = RWSInterface::getData(task,
                                       user_defined::rapid::symbols::RAPID_SYMBOL_CAMERA_DATA_RIGHT,
                                       &(p_data->right));
      }
    }
    break;
  }

  return result;
}

void RWSInterfaceYuMi::setData(DualEGMData data, Side side)
{
  setDualData(data.left.constructString(),
              data.right.constructString(),
              user_defined::rapid::symbols::RAPID_SYMBOL_EGM_DATA,
              side);
}

void RWSInterfaceYuMi::setData(DualSGData data, Side side)
{
  setDualData(data.left.constructString(),
              data.right.constructString(),
              user_defined::rapid::symbols::RAPID_SYMBOL_SG_DATA,
              side);
}

void RWSInterfaceYuMi::setData(DualCameraData data, Side side)
{
  std::string task = user_defined::rapid::task_names::RAPID_TASK_CAMERA;

  switch (side)
  {
    case LEFT_SIDE:
    {
      RWSInterface::setData(task,
                            user_defined::rapid::symbols::RAPID_SYMBOL_CAMERA_DATA_LEFT,
                            data.left.constructString());
    }
    break;

    case RIGHT_SIDE:
    {
      RWSInterface::setData(task,
                            user_defined::rapid::symbols::RAPID_SYMBOL_CAMERA_DATA_RIGHT,
                            data.right.constructString());
    }
    break;

    case BOTH_SIDES:
    {
      RWSInterface::setData(task,
                            user_defined::rapid::symbols::RAPID_SYMBOL_CAMERA_DATA_LEFT,
                            data.left.constructString());
      RWSInterface::setData(task,
                            user_defined::rapid::symbols::RAPID_SYMBOL_CAMERA_DATA_RIGHT,
                            data.right.constructString());
    }
    break;
  }
}

bool RWSInterfaceYuMi::getSGCalibratedStatus(Gripper gripper)
{
  std::string io_signal = gripper == LEFT_GRIPPER ? system_defined::io::signals::IO_SG_HAND_STATUSCALIBRATED_L 
                                                    :
                                                    system_defined::io::signals::IO_SG_HAND_STATUSCALIBRATED_R;
 std::string value = rws_client_.getIOSignal(io_signal);

  return value == system_defined::io::values::DI_SIGNAL_HIGH;
}

float RWSInterfaceYuMi::getSGActualPosition(Gripper gripper)
{
  std::string io_signal = gripper == LEFT_GRIPPER ? system_defined::io::signals::IO_SG_HAND_ACTUALPOSITION_L 
                                                    :
                                                    system_defined::io::signals::IO_SG_HAND_ACTUALPOSITION_R;
  std::string value = rws_client_.getIOSignal(io_signal);

  return (float) (stringToFloat(value)/10.0);
}

float RWSInterfaceYuMi::getSGActualSpeed(Gripper gripper)
{
  std::string io_signal = gripper == LEFT_GRIPPER ? system_defined::io::signals::IO_SG_HAND_ACTUALSPEED_L 
                                                    :
                                                    system_defined::io::signals::IO_SG_HAND_ACTUALSPEED_R;
  std::string value = rws_client_.getIOSignal(io_signal);
  
  return (float) (stringToFloat(value)/10.0);
}

void RWSInterfaceYuMi::doSGInitialize(Side side)
{
  setDualData(user_defined::rapid::values::SG_COMMAND_INITIALIZE,
              user_defined::rapid::symbols::RAPID_SYMBOL_SG_COMMAND,
              side);
  toggleIOSignal(user_defined::io::signals::IO_RUN_SG_COMMAND);
}

void RWSInterfaceYuMi::doSGCalibrate(Side side)
{
  setDualData(user_defined::rapid::values::SG_COMMAND_CALIBRATE,
              user_defined::rapid::symbols::RAPID_SYMBOL_SG_COMMAND,
              side);
  toggleIOSignal(user_defined::io::signals::IO_RUN_SG_COMMAND);
}

void RWSInterfaceYuMi::doSGMove(Side side)
{
  setDualData(user_defined::rapid::values::SG_COMMAND_MOVE,
              user_defined::rapid::symbols::RAPID_SYMBOL_SG_COMMAND,
              side);
  toggleIOSignal(user_defined::io::signals::IO_RUN_SG_COMMAND);
}

void RWSInterfaceYuMi::doSGMoveTo(const float target, bool wait, Side side)
{
  doSGMoveTo(target, target, wait, side);
}

void RWSInterfaceYuMi::doSGMoveTo(const float left_target, const float right_target, bool wait, Side side)
{
  if (isRAPIDRunning())
  {
    DualSGData sg_data;
    if (getData(&sg_data))
    {
      sg_data.left.setTargetPosition(left_target);
      sg_data.right.setTargetPosition(right_target);
      setData(sg_data, side);
    }

    doSGMove(side);

    if (wait)
    {
      float current_position = 0;
      float current_speed = 0;
      bool done = false;

      float left_condition = left_target;
      if (left_condition < 0.0) left_condition = 0.0;
      if (left_condition > sg_data.left.getPhysicalLimit()) left_condition = sg_data.left.getPhysicalLimit();

      float right_condition = right_target;
      if (right_condition < 0.0) right_condition = 0.0;
      if (right_condition > sg_data.right.getPhysicalLimit()) right_condition = sg_data.right.getPhysicalLimit();

      while (!done && isRAPIDRunning() && isModeAuto())
      {
        if (side == LEFT_SIDE || side == BOTH_SIDES)
        { 
          current_position = getSGActualPosition(LEFT_GRIPPER);
          current_speed = getSGActualSpeed(LEFT_GRIPPER);

          done = (std::abs(current_position - left_condition) < 0.2 && current_speed == 0);
        }
      
        if (side == RIGHT_SIDE || side == BOTH_SIDES)
        {
          current_position = getSGActualPosition(RIGHT_GRIPPER);
          current_speed = getSGActualSpeed(RIGHT_GRIPPER);

          done = (std::abs(current_position - right_condition) < 0.2 && current_speed == 0);
        }
      }
    }
  }
}

void RWSInterfaceYuMi::doSGGrip(Side side)
{
  setDualData(user_defined::rapid::values::SG_COMMAND_GRIP,
              user_defined::rapid::symbols::RAPID_SYMBOL_SG_COMMAND,
              side);
  toggleIOSignal(user_defined::io::signals::IO_RUN_SG_COMMAND);
}

void RWSInterfaceYuMi::doSGGripTo(const float target, const float force, Side side)
{
  doSGGripTo(target, force, target, force, side);
}

void RWSInterfaceYuMi::doSGGripTo(const float left_target,
                                  const float left_force,
                                  const float right_target,
                                  const float right_force,
                                  Side side)
{
  if (isRAPIDRunning())
  {
    DualSGData sg_data;
    if (getData(&sg_data))
    {
      sg_data.left.setTargetPosition(left_target);
      sg_data.left.setHoldForce(left_force);
      sg_data.right.setTargetPosition(right_target);
      sg_data.right.setHoldForce(right_force);
      setData(sg_data, side);
    }

    doSGGrip(side);
  }
}

void RWSInterfaceYuMi::doCameraRequestImage(Side side)
{
  setUseCameraData(side);

  RWSInterface::setData(user_defined::rapid::task_names::RAPID_TASK_CAMERA,
                        user_defined::rapid::symbols::RAPID_SYMBOL_CAMERA_COMMAND,
                        user_defined::rapid::values::CAMERA_COMMAND_REQUEST_IMAGE);

  toggleIOSignal(user_defined::io::signals::IO_RUN_CAMERA_COMMAND);
}

void RWSInterfaceYuMi::doCameraSetExposure(DualCameraData data, Side side)
{
  setData(data, side);

  setUseCameraData(side);

  RWSInterface::setData(user_defined::rapid::task_names::RAPID_TASK_CAMERA,
                        user_defined::rapid::symbols::RAPID_SYMBOL_CAMERA_COMMAND,
                        user_defined::rapid::values::CAMERA_COMMAND_SET_EXPOSURE);

  toggleIOSignal(user_defined::io::signals::IO_RUN_CAMERA_COMMAND);
}

/********************************************
 * Auxiliary methods
 */

bool RWSInterfaceYuMi::getDualData(RAPIDRecord* p_left_data, RAPIDRecord* p_right_data, RAPIDSymbol symbol, Side side)
{
  bool result = false;

  switch (side)
  {
    case LEFT_SIDE:
    {
      result = RWSInterface::getData(system_defined::rapid::task_names::RAPID_TASK_ROB_L, symbol, p_left_data);
    }
    break;

    case RIGHT_SIDE:
    {
      result = RWSInterface::getData(system_defined::rapid::task_names::RAPID_TASK_ROB_R, symbol, p_right_data);
    }
    break;

    case BOTH_SIDES:
    {
      if (RWSInterface::getData(system_defined::rapid::task_names::RAPID_TASK_ROB_L, symbol, p_left_data))
      {
        result = RWSInterface::getData(system_defined::rapid::task_names::RAPID_TASK_ROB_R, symbol, p_right_data);
      }
    }
    break;
  }

  return result;
}

void RWSInterfaceYuMi::setDualData(std::string data, RAPIDSymbol symbol, Side side)
{
  setDualData(data, data, symbol, side);
}

void RWSInterfaceYuMi::setDualData(std::string left_data, std::string right_data, RAPIDSymbol symbol, Side side)
{
  switch (side)
  {
    case LEFT_SIDE:
    {
      RWSInterface::setData(system_defined::rapid::task_names::RAPID_TASK_ROB_L, symbol, left_data);
    }
    break;

    case RIGHT_SIDE:
    {
      RWSInterface::setData(system_defined::rapid::task_names::RAPID_TASK_ROB_R, symbol, right_data);
    }
    break;

    case BOTH_SIDES:
    {
      RWSInterface::setData(system_defined::rapid::task_names::RAPID_TASK_ROB_L, symbol, left_data);
      RWSInterface::setData(system_defined::rapid::task_names::RAPID_TASK_ROB_R, symbol, right_data);
    }
    break;
  }
}

void RWSInterfaceYuMi::setUseCameraData(Side side)
{
  std::string task = user_defined::rapid::task_names::RAPID_TASK_CAMERA;
  RAPIDSymbol symbol = user_defined::rapid::symbols::RAPID_SYMBOL_USE_CAMERA;

  switch (side)
  {
    case LEFT_SIDE:
    {
      RWSInterface::setData(task,
                            symbol,
                            user_defined::rapid::values::CAMERA_USE_LEFT_CAMERA);
    }
    break;

    case RIGHT_SIDE:
    {
      RWSInterface::setData(task,
                            symbol,
                            user_defined::rapid::values::CAMERA_USE_RIGHT_CAMERA);
    }
    break;

    case BOTH_SIDES:
    {
      RWSInterface::setData(task,
                            symbol,
                            user_defined::rapid::values::CAMERA_USE_BOTH_CAMERAS);
    }
    break;
  }
}

float RWSInterfaceYuMi::stringToFloat(std::string input_string)
{
  float float_value = 0;

  std::stringstream ss;
  ss << input_string;
  ss >> float_value;

  return float_value;
}

} // end namespace rws_interface
} // end namespace abb
