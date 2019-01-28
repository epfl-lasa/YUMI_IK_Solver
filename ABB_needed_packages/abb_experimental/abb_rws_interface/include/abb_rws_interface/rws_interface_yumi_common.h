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

#ifndef RWS_INTERFACE_YUMI_COMMON_H
#define RWS_INTERFACE_YUMI_COMMON_H

#include "rws_interface_common.h"

namespace abb
{
namespace rws_interface
{
/****************************************************************************************
 * Default values
 */

namespace rws_strings
{
namespace system_defined
{
namespace rapid
{
namespace task_names
{
/**
 * \brief Default name for the left arm robot RAPID motion task.
 */
const std::string RAPID_TASK_ROB_L = "T_ROB_L";
        
/**
 * \brief Default name for the right arm robot RAPID motion task.
 */
const std::string RAPID_TASK_ROB_R = "T_ROB_R";
}
}

namespace io
{
namespace signals
{
/**
 * \brief Name of defined IO signal for smart gripper left position.
 *
 * Note: Requires the Smart Gripper product.
 */
const std::string IO_SG_HAND_ACTUALPOSITION_L = "hand_ActualPosition_L";
        
/**
 * \brief Name of defined IO signal for smart gripper right position.
 *
 * Note: Requires the Smart Gripper product.
 */
const std::string IO_SG_HAND_ACTUALPOSITION_R = "hand_ActualPosition_R";
        
/**
 * \brief Name of defined IO signal for smart gripper left speed.
 *
 * Note: Requires the Smart Gripper product.
 */
const std::string IO_SG_HAND_ACTUALSPEED_L = "hand_ActualSpeed_L";
        
/**
 * \brief Name of defined IO signal for smart gripper right speed.
 *
 * Note: Requires the Smart Gripper product.
 */
const std::string IO_SG_HAND_ACTUALSPEED_R = "hand_ActualSpeed_R";
        
/**
 * \brief Name of defined IO signal for smart gripper left calibration status.
 *
 * Note: Requires the Smart Gripper product.
 */
const std::string IO_SG_HAND_STATUSCALIBRATED_L = "hand_StatusCalibrated_L";
        
/**
 * \brief Name of defined IO signal for smart gripper right calibration status.
 *
 * Note: Requires the Smart Gripper product.
 */
const std::string IO_SG_HAND_STATUSCALIBRATED_R = "hand_StatusCalibrated_R";
}
}
}

namespace user_defined
{
namespace rapid
{
namespace task_names
{
/**
 * \brief Name of special camera task.
 *
 * Note: Assumed to have been defined in the robot controller.
 */
const std::string RAPID_TASK_CAMERA = "T_CAMERA";
}

namespace module_names
{
/**
 * \brief Name of special main module.
 *
 * Note: Assumed to have been loaded into a RAPID motion task.
 */
const std::string RAPID_PROGRAM_MODULE_CAMERA = "TCameraMain";
        
/**
 * \brief Name of special smart gripper module.
 *
 * Note: Assumed to have been loaded into a RAPID motion task.
 */
const std::string RAPID_SYSTEM_MODULE_SMARTGRIPPER = "TRobSG";
}

namespace symbols
{
/**
 * \brief A defined RAPID symbol for smart gripper data.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const RAPIDSymbol RAPID_SYMBOL_SG_DATA(module_names::RAPID_SYSTEM_MODULE_SMARTGRIPPER, "sg_data");
        
/**
 * \brief A defined RAPID symbol for smart gripper commands.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const RAPIDSymbol RAPID_SYMBOL_SG_COMMAND(module_names::RAPID_SYSTEM_MODULE_SMARTGRIPPER, "sg_current_command");
        
/**
 * \brief A defined RAPID symbol for specifying which camera(s) to use.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const RAPIDSymbol RAPID_SYMBOL_USE_CAMERA(module_names::RAPID_PROGRAM_MODULE_CAMERA, "use_camera");
        
/**
 * \brief A defined RAPID symbol for left camera data.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const RAPIDSymbol RAPID_SYMBOL_CAMERA_DATA_LEFT(module_names::RAPID_PROGRAM_MODULE_CAMERA, "camera_data_left");
        
/**
 * \brief A defined RAPID symbol for right camera data.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const RAPIDSymbol RAPID_SYMBOL_CAMERA_DATA_RIGHT(module_names::RAPID_PROGRAM_MODULE_CAMERA, "camera_data_right");
        
/**
 * \brief A defined RAPID symbol for camera commands.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const RAPIDSymbol RAPID_SYMBOL_CAMERA_COMMAND(module_names::RAPID_PROGRAM_MODULE_CAMERA, "camera_current_command");
}

namespace values
{
/**
 * \brief Value of a defined RAPID value for an empty comand for the smart gripper(s).
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const std::string SG_COMMAND_NONE = "0";
        
/**
 * \brief Value of a defined RAPID value for an initialization command for the smart gripper(s).
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const std::string SG_COMMAND_INITIALIZE = "1";
        
/**
 * \brief Value of a defined RAPID value for a calibration command for the smart gripper(s).
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const std::string SG_COMMAND_CALIBRATE = "2";
        
/**
 * \brief Value of a defined RAPID value for a move command for the smart gripper(s).
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const std::string SG_COMMAND_MOVE = "3";
        
/**
 * \brief Value of a defined RAPID value for a grip command for the smart gripper(s).
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const std::string SG_COMMAND_GRIP = "4";
        
/**
 * \brief Value of a defined RAPID value for specifying no camera is used.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const std::string CAMERA_USE_NO_CAMERA = "0";
                
/**
 * \brief Value of a defined RAPID value for specifying left camera is used.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const std::string CAMERA_USE_LEFT_CAMERA = "1";
                
/**
 * \brief Value of a defined RAPID value for specifying right camera is used.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const std::string CAMERA_USE_RIGHT_CAMERA = "2";
                
/**
 * \brief Value of a defined RAPID value for specifying both cameras are used.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const std::string CAMERA_USE_BOTH_CAMERAS = "3";
        
/**
 * \brief Value of a defined RAPID value for an empty comamnd for the camera(s).
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const std::string CAMERA_COMMAND_NONE = "0";
        
/**
 * \brief Value of a defined RAPID value for a request command for the camera(s).
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const std::string CAMERA_COMMAND_REQUEST_IMAGE = "1";
        
/**
 * \brief Value of a defined RAPID value for a set exposure command for the camera(s).
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const std::string CAMERA_COMMAND_SET_EXPOSURE = "2";
}
}

namespace io
{
namespace signals
{
/**
 * \brief Name of defined IO signal for running smart gripper commands.
 *
 * Note: Assumed to have been setup in the robot controller.
 */
const std::string IO_RUN_SG_COMMAND = "RUN_SG_COMMAND";
        
/**
 * \brief Name of defined IO signal for running camera commands.
 *
 * Note: Assumed to have been setup in the robot controller.
 */
const std::string IO_RUN_CAMERA_COMMAND = "RUN_CAMERA_COMMAND";
}
}
}
}




/****************************************************************************************
 * Support enums
 */

/**
 * \brief An enum for specifying which side of the robot is used.
 */
enum Side
{
  LEFT_SIDE,  ///< \brief Left side.
  RIGHT_SIDE, ///< \brief Right side.
  BOTH_SIDES  ///< \brief Both sides.
};

/**
* \brief An enum for specifying which smart gripper of the robot is used.
*/
enum Gripper
{
  LEFT_GRIPPER, ///< \brief Left gripper.
  RIGHT_GRIPPER ///< \brief Right gripper.
};




/****************************************************************************************
 * Support classes
 */

/**
 * \brief A class for representing smart gripper (SG) data.
 */
class SGData : public RAPIDRecord
{
public:
  /**
   * \brief A constructor.
   */
  SGData()
  :
  ELEMENT_1("max_speed"),
  ELEMENT_2("hold_force"),
  ELEMENT_3("physical_limit"),
  ELEMENT_4("target_position")
  {
    components_.push_back(VectorElement(ELEMENT_1, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_2, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_3, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_4, RAPIDAtomic::NUM));
  }
  
  /**
   * \brief Get the max speed [mm/s].
   *
   * \return float containing the value.
   */
  float getMaxSpeed()       { return getComponentValue(ELEMENT_1, NUM_VALUE); }
  
  /**
   * \brief Get the hold force [N].
   *
   * \return float containing the value.
   */
  float getHoldForce()      { return getComponentValue(ELEMENT_2, NUM_VALUE); }
  
  /**
   * \brief Get the physical limit [mm].
   *
   * \return float containing the value.
   */
  float getPhysicalLimit()  { return getComponentValue(ELEMENT_3, NUM_VALUE); }
  
  /**
   * \brief Get the target position [mm].
   *
   * \return float containing the value.
   */
  float getTargetPosition() { return getComponentValue(ELEMENT_4, NUM_VALUE); }

  /**
   * \brief Set the max speed [mm/s].
   *
   * \param value containing the new value.
   */
  void setMaxSpeed(const float value)       { return setComponentValue(ELEMENT_1, value); }
  
  /**
   * \brief Set the hold force [N].
   *
   * \param value containing the new value.
   */
  void setHoldForce(const float value)      { return setComponentValue(ELEMENT_2, value); }
  
  /**
   * \brief Set the physical limit [mm].
   *
   * \param value containing the new value.
   */
  void setPhysicalLimit(const float value)  { return setComponentValue(ELEMENT_3, value); }
  
  /**
   * \brief Set the target position [mm].
   *
   * \param value containing the new value.
   */
  void setTargetPosition(const float value) { return setComponentValue(ELEMENT_4, value); }

private:
  /**
   * \brief First element.
   */
  const std::string ELEMENT_1;

  /**
   * \brief Second element.
   */
  const std::string ELEMENT_2;

  /**
   * \brief Third element.
   */
  const std::string ELEMENT_3;

  /**
   * \brief Fourth element.
   */
  const std::string ELEMENT_4;
};

/**
 * \brief A class for representing camera data.
 */
class CameraData : public RAPIDRecord
{
public:
  /**
   * \brief A constructor.
   */
  CameraData()
  :
  ELEMENT_1("exposure_time")
  {
    components_.push_back(VectorElement(ELEMENT_1, RAPIDAtomic::NUM));
  }
  
  /**
   * \brief Get the exposure time.
   *
   * \return float containing the value.
   */
  float getExposureTime() { return getComponentValue(ELEMENT_1, NUM_VALUE); }
  
  /**
   * \brief Set the exposure time.
   *
   * \param value containing the new value.
   */
  void setExposureTime(const float value) { return setComponentValue(ELEMENT_1, value); }

private:
  /**
   * \brief First element.
   */
  const std::string ELEMENT_1;
};




/****************************************************************************************
 * Support structs
 */

/**
 * \brief A struct for dual joint data.
 */
struct DualJointData
{
  /**
   * \brief Left data.
   */
  JointData left;
  
  /**
   * \brief Right data.
   */
  JointData right;
};

/**
 * \brief A struct for dual EGM data.
 */
struct DualEGMData
{
  /**
   * \brief Left data.
   */
  EGMData left;

  /**
   * \brief Right data.
   */
  EGMData right;
};

/**
 * \brief A struct for dual smart gripper (SG) data.
 */
struct DualSGData
{
  /**
   * \brief Left data.
   */
  SGData left;
  
  /**
   * \brief Right data.
   */
  SGData right;
};

/**
 * \brief A struct for dual camera data.
 */
struct DualCameraData
{
  /**
   * \brief Left data.
   */
  CameraData left;
  
  /**
   * \brief Right data.
   */
  CameraData right;
};

} // end namespace rws_interface
} // end namespace abb

#endif