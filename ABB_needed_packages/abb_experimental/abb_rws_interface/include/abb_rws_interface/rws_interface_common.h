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

#ifndef RWS_INTERFACE_COMMON_H
#define RWS_INTERFACE_COMMON_H

#include "rws_rapid_data.h"

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
 * \brief Default name for the first robot RAPID motion task.
 */
const std::string RAPID_TASK_ROB1 = "T_ROB1";

/**
 * \brief Default name for the second robot RAPID motion task.
 */
const std::string RAPID_TASK_ROB2 = "T_ROB2";

/**
 * \brief Default name for the third robot RAPID motion task.
 */
const std::string RAPID_TASK_ROB3 = "T_ROB3";

/**
 * \brief Default name for the fourth robot RAPID motion task.
 */
const std::string RAPID_TASK_ROB4 = "T_ROB4";
}

namespace values
{
/**
 * \brief RAPID boolean true.
 */
const std::string RAPID_TRUE  = "TRUE";

/**
 * \brief RAPID boolean false.
 */
const std::string RAPID_FALSE = "FALSE";
}
}

namespace io
{
namespace values
{
/**
 * \brief High IO signal.
 */
const std::string DI_SIGNAL_HIGH = "1";
        
/**
 * \brief Low IO signal.
 */
const std::string DI_SIGNAL_LOW  = "0";
}
}

namespace controller_states
{
/**
 * \brief RAPID running.
 */
const std::string RAPID_EXECUTION_RUNNING  = "running";

/**
 * \brief Robot controller auto mode.
 */
const std::string PANEL_OPERATION_MODE_AUTO = "AUTO";
}
}

namespace user_defined
{
namespace rapid
{
namespace module_names
{
/**
 * \brief Name of special main module.
 *
 * Note: Assumed to have been loaded into a RAPID motion task.
 */
const std::string RAPID_PROGRAM_MODULE_ROB = "TRobMain";
        
/**
 * \brief Name of special EGM module.
 *
 * Note: Assumed to have been loaded into a RAPID motion task.
 */
const std::string RAPID_SYSTEM_MODULE_EGM = "TRobEGM";
}

namespace symbols
{
/**
 * \brief A defined RAPID symbol for current joints.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const RAPIDSymbol RAPID_SYMBOL_CURRENT_JOINTS(module_names::RAPID_PROGRAM_MODULE_ROB, "current_joints");
        
/**
 * \brief A defined RAPID symbol for current action.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const RAPIDSymbol RAPID_SYMBOL_CURRENT_ACTION(module_names::RAPID_PROGRAM_MODULE_ROB, "current_action");
        
/**
 * \brief A defined RAPID symbol for EGM data.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const RAPIDSymbol RAPID_SYMBOL_EGM_DATA(module_names::RAPID_SYSTEM_MODULE_EGM, "egm_data");
}

namespace values
{
/**
 * \brief Value of a defined RAPID value for an idle action.
 * 
 * Note: Assumed to have been declared in a RAPID module.
 */
const std::string ACTION_IDLE = "0";
}
}

namespace io
{
namespace signals
{
/**
 * \brief Name of defined IO signal for EGM joint mode.
 *
 * Note: Assumed to have been setup in the robot controller.
 */
const std::string IO_EGM_START_JOINT = "EGM_START_JOINT";
        
/**
 * \brief Name of defined IO signal for EGM pose mode.
 *
 * Note: Assumed to have been setup in the robot controller.
 */
const std::string IO_EGM_START_POSE = "EGM_START_POSE";
        
/**
 * \brief Name of defined IO signal for EGM stop.
 *
 * Note: Assumed to have been setup in the robot controller.
 */
const std::string IO_EGM_STOP = "EGM_STOP";
        
/**
 * \brief Name of defined IO signal for go to home position.
 *
 * Note: Assumed to have been setup in the robot controller.
 */
const std::string IO_GO_TO_HOME_POSITION = "GO_TO_HOME_POSITION";
}
}
}
}




/****************************************************************************************
 * Support classes
 */

/**
 * \brief A class for representing a RAPID joint target.
 */
class JointData : public RAPIDRecord
{
public:
  /**
   * \brief A constructor.
   */
  JointData()
  :
  ELEMENT_1("Rob1"),
  ELEMENT_2("Rob2"),
  ELEMENT_3("Rob3"),
  ELEMENT_4("Rob4"),
  ELEMENT_5("Rob5"),
  ELEMENT_6("Rob6"),
  ELEMENT_7("Ext1"),
  ELEMENT_8("Ext2"),
  ELEMENT_9("Ext3"),
  ELEMENT_10("Ext4"),
  ELEMENT_11("Ext5"),
  ELEMENT_12("Ext6")
  {
    components_.push_back(VectorElement(ELEMENT_1, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_2, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_3, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_4, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_5, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_6, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_7, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_8, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_9, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_10, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_11, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_12, RAPIDAtomic::NUM));
  }

  /**
   * \brief Get robot axis 1.
   *
   * \return float containing the value.
   */
  float getRob1() { return getComponentValue(ELEMENT_1, NUM_VALUE); }

  /**
   * \brief Get robot axis 2.
   *
   * \return float containing the value.
   */
  float getRob2() { return getComponentValue(ELEMENT_2, NUM_VALUE); }

  /**
   * \brief Get robot axis 3.
   *
   * \return float containing the value.
   */
  float getRob3() { return getComponentValue(ELEMENT_3, NUM_VALUE); }

  /**
   * \brief Get robot axis 4.
   *
   * \return float containing the value.
   */
  float getRob4() { return getComponentValue(ELEMENT_4, NUM_VALUE); }

  /**
   * \brief Get robot axis 5.
   *
   * \return float containing the value.
   */
  float getRob5() { return getComponentValue(ELEMENT_5, NUM_VALUE); }

  /**
   * \brief Get robot axis 6.
   *
   * \return float containing the value.
   */
  float getRob6() { return getComponentValue(ELEMENT_6, NUM_VALUE); }

  /**
   * \brief Get external axis 1.
   *
   * \return float containing the value.
   */
  float getExt1() { return getComponentValue(ELEMENT_7, NUM_VALUE); }

  /**
   * \brief Get external axis 2.
   *
   * \return float containing the value.
   */
  float getExt2() { return getComponentValue(ELEMENT_8, NUM_VALUE); }

  /**
   * \brief Get external axis 3.
   *
   * \return float containing the value.
   */
  float getExt3() { return getComponentValue(ELEMENT_9, NUM_VALUE); }

  /**
   * \brief Get external axis 4.
   *
   * \return float containing the value.
   */
  float getExt4() { return getComponentValue(ELEMENT_10, NUM_VALUE); }

  /**
   * \brief Get external axis 5.
   *
   * \return float containing the value.
   */
  float getExt5() { return getComponentValue(ELEMENT_11, NUM_VALUE); }

  /**
   * \brief Get external axis 6.
   *
   * \return float containing the value.
   */
  float getExt6() { return getComponentValue(ELEMENT_12, NUM_VALUE); }
  
  /**
   * \brief Set robot axis 1.
   *
   * \param value containing the new value.
   */
  void setRob1(const float value) { setComponentValue(ELEMENT_1, value); }

  /**
   * \brief Set robot axis 2.
   *
   * \param value containing the new value.
   */
  void setRob2(const float value) { setComponentValue(ELEMENT_2, value); }

  /**
   * \brief Set robot axis 3.
   *
   * \param value containing the new value.
   */
  void setRob3(const float value) { setComponentValue(ELEMENT_3, value); }

  /**
   * \brief Set robot axis 4.
   *
   * \param value containing the new value.
   */
  void setRob4(const float value) { setComponentValue(ELEMENT_4, value); }

  /**
   * \brief Set robot axis 5.
   *
   * \param value containing the new value.
   */
  void setRob5(const float value) { setComponentValue(ELEMENT_5, value); }

  /**
   * \brief Set robot axis 6.
   *
   * \param value containing the new value.
   */
  void setRob6(const float value) { setComponentValue(ELEMENT_6, value); }

  /**
   * \brief Set external axis 1.
   *
   * \param value containing the new value.
   */
  void setExt1(const float value) { setComponentValue(ELEMENT_7, value); }

  /**
   * \brief Set external axis 2.
   *
   * \param value containing the new value.
   */
  void setExt2(const float value) { setComponentValue(ELEMENT_8, value); }

  /**
   * \brief Set external axis 3.
   *
   * \param value containing the new value.
   */
  void setExt3(const float value) { setComponentValue(ELEMENT_9, value); }

  /**
   * \brief Set external axis 4.
   *
   * \param value containing the new value.
   */
  void setExt4(const float value) { setComponentValue(ELEMENT_10, value); }

  /**
   * \brief Set external axis 5.
   *
   * \param value containing the new value.
   */
  void setExt5(const float value) { setComponentValue(ELEMENT_11, value); }

  /**
   * \brief Set external axis 6.
   *
   * \param value containing the new value.
   */
  void setExt6(const float value) { setComponentValue(ELEMENT_12, value); }

  /**
   * \brief Get all joint values in a vector.
   *
   * \return std::vector<double> containing the values.
   */
  std::vector<double> getJointValues()
  {
    float temp = 0.0;
    std::vector<double> result;

    for (size_t i = 0; i < components_.size(); ++i)
    {
      components_.at(i).second.getValue(&temp);
      if (temp < 9e+08)
      {
        result.push_back((double) temp);
      }
    }

    return result;
  }

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

  /**
   * \brief Fifth element.
   */
  const std::string ELEMENT_5;

  /**
   * \brief Sixth element.
   */
  const std::string ELEMENT_6;

  /**
   * \brief Seventh element.
   */
  const std::string ELEMENT_7;

  /**
   * \brief Eighth element.
   */
  const std::string ELEMENT_8;

  /**
   * \brief Ninth element.
   */
  const std::string ELEMENT_9;

  /**
   * \brief Tenth element.
   */
  const std::string ELEMENT_10;

  /**
   * \brief Eleventh element.
   */
  const std::string ELEMENT_11;

  /**
   * \brief Twelfth element.
   */
  const std::string ELEMENT_12;
};

/**
 * \brief A class for representing EGM data.
 */
class EGMData : public RAPIDRecord
{
public:
  /**
   * \brief A constructor.
   */
  EGMData()
  :
  ELEMENT_1("comm_timeout"),
  ELEMENT_2("tool_name"),
  ELEMENT_3("wobj_name"),
  ELEMENT_4("cond_min_max"),
  ELEMENT_5("lp_filter"),
  ELEMENT_6("max_speed_deviation"),
  ELEMENT_7("cond_time"),
  ELEMENT_8("ramp_in_time"),
  ELEMENT_9("pos_corr_gain")
  {
    components_.push_back(VectorElement(ELEMENT_1, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_2, RAPIDAtomic::STRING));
    components_.push_back(VectorElement(ELEMENT_3, RAPIDAtomic::STRING));
    components_.push_back(VectorElement(ELEMENT_4, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_5, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_6, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_7, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_8, RAPIDAtomic::NUM));
    components_.push_back(VectorElement(ELEMENT_9, RAPIDAtomic::NUM));
  }
  
  /**
   * \brief Get the communication timeout.
   *
   * \return float containing the value.
   */
  float getCommTimeout() { return getComponentValue(ELEMENT_1, NUM_VALUE); }
  
  /**
   * \brief Get the used tool's name.
   *
   * \return std::string containing the value.
   */
  std::string getToolName() { return getComponentValue(ELEMENT_2, STRING_VALUE); }
  
  /**
   * \brief Get the used work object's name.
   *
   * \return std::string containing the value.
   */
  std::string getWobjName() { return getComponentValue(ELEMENT_3, STRING_VALUE); }

  /**
   * \brief Get the condition min/max value.
   *
   * \return float containing the value.
   */
  float getCondMinMax() { return getComponentValue(ELEMENT_4, NUM_VALUE); }

  /**
   * \brief Get the low pass filter value.
   *
   * \return float containing the value.
   */
  float getLpFilter() { return getComponentValue(ELEMENT_5, NUM_VALUE); }

  /**
   * \brief Get the maximum speed deviation.
   *
   * \return float containing the value.
   */
  float getMaxSpeedDeviation() { return getComponentValue(ELEMENT_6, NUM_VALUE); }

  /**
   * \brief Get the condition time.
   *
   * \return float containing the value.
   */
  float getCondTime() { return getComponentValue(ELEMENT_7, NUM_VALUE); }

  /**
   * \brief Get the ramp in time.
   *
   * \return float containing the value.
   */
  float getRampInTime() { return getComponentValue(ELEMENT_8, NUM_VALUE); }

  /**
   * \brief Get the position correction gain.
   *
   * \return float containing the value.
   */
  float getPosCorrGain() { return getComponentValue(ELEMENT_9, NUM_VALUE); }
  
  /**
   * \brief Set the communication timeout.
   *
   * \param value containing the new value.
   */
  void setCommTimeout(const float value) { setComponentValue(ELEMENT_1, value); }

  /**
   * \brief Set the used tool's name (assumed to have been declared on the robot side).
   *
   * \param value containing the new value.
   */
  void setToolName(const std::string value) { setComponentValue(ELEMENT_2, value); }

  /**
   * \brief Set the used work object's name (assumed to have been declared on the robot side).
   *
   * \param value containing the new value.
   */
  void setWobjName(const std::string value) { setComponentValue(ELEMENT_3, value); }
  
  /**
   * \brief Set the condition min/max value.
   *
   * \param value containing the new value.
   */
  void setCondMinMax(const float value) { setComponentValue(ELEMENT_4, value); }

  /**
   * \brief Set the low pass filter value.
   *
   * \param value containing the new value.
   */
  void setLpFilter(const float value) { setComponentValue(ELEMENT_5, value); }

  /**
   * \brief Set the maximum speed deviation (affects the acceleration as well).
   *
   * \param value containing the new value.
   */
  void setMaxSpeedDeviation(const float value) { setComponentValue(ELEMENT_6, value); }

  /**
   * \brief Set the condition time.
   *
   * \param value containing the new value.
   */
  void setCondTime(const float value) { setComponentValue(ELEMENT_7, value); }
  
  /**
   * \brief Set the ramp in time.
   *
   * \param value containing the new value.
   */
  void setRampInTime(const float value) { setComponentValue(ELEMENT_8, value); }

  /**
   * \brief Set the position correction gain.
   *
   * \param value containing the new value.
   */
  void setPosCorrGain(const float value) { setComponentValue(ELEMENT_9, value); }

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

  /**
   * \brief Fifth element.
   */
  const std::string ELEMENT_5;

  /**
   * \brief Sixth element.
   */
  const std::string ELEMENT_6;

  /**
   * \brief Seventh element.
   */
  const std::string ELEMENT_7;

  /**
  * \brief Eighth element.
  */
  const std::string ELEMENT_8;

  /**
  * \brief Ninth element.
  */
  const std::string ELEMENT_9;
};

} // end namespace rws_interface
} // end namespace abb

#endif