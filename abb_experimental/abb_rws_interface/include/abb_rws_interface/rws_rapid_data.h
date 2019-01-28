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

#ifndef RWS_RAPID_DATA_H
#define RWS_RAPID_DATA_H

#include <string>
#include <vector>

namespace abb
{
namespace rws_interface
{
/****************************************************************************************
 * Support structures
 */

/**
 * \brief A struct for containing parts of a RAPID symbol path.
 */
struct RAPIDSymbol
{
  /**
   * \brief A constructor.
   *
   * \param arg_module_name specifying the name of the RAPID module containing the symbol.
   * \param arg_symbol_name specifying the name of the RAPID symbol.
   */
  RAPIDSymbol(const std::string arg_module_name, const std::string arg_symbol_name)
  :
  module_name(arg_module_name),
  symbol_name(arg_symbol_name)
  {}

  /**
   * \brief The RAPID module name.
   */
  std::string module_name;

  /**
   * \brief The RAPID symbol name.
   */
  std::string symbol_name;
};




/****************************************************************************************
 * Support classes
 */

/**
 * \brief A class for representing RAPID atomic data types.
 */
class RAPIDAtomic
{
public:
  /**
   * \brief An enum for different atomic data types.
   */
  enum Type
  {
    NUM,   ///< \brief A num (i.e. float).
    DNUM,  ///< \brief A dnum (i.e. double).
    BOOL,  ///< \brief A bool (i.e. bool).
    STRING ///< \brief A string (i.e. std::string).
  };

  /**
   * \brief A constructor.
   *
   * \param type specifying the atomic data type to use.
   */
  RAPIDAtomic(const Type type) : type_(type), num_(0), dnum_(0), bool_(false) {};

  /**
   * \brief A method to get the used data type.
   *
   * \return Type containing used data type.
   */
  Type getType() { return type_; }

  /**
   * \brief A method to get the float value (i.e. RAPID num).
   * 
   * \param p_value for storing the retrived value.
   */
  void getValue(float* p_value) { *p_value = num_; };

  /**
   * \brief A method to get the double value (i.e. RAPID dnum).
   * 
   * \param p_value for storing the retrived value.
   */
  void getValue(double* p_value) { *p_value = dnum_; };

  /**
   * \brief A method to get the bool value (i.e. RAPID bool).
   * 
   * \param p_value for storing the retrived value.
   */
  void getValue(bool* p_value) { *p_value = bool_; };

  /**
   * \brief A method to get the std::string value (i.e. RAPID string).
   * 
   * \param p_value for storing the retrived value.
   */
  void getValue(std::string* p_value) { *p_value = string_; };

  /**
   * \brief A method to set the float value (i.e. RAPID num).
   * 
   * \param value for the new value.
   */
  void setValue(const float value) { num_ = value; };

  /**
   * \brief A method to set the double value (i.e. RAPID dnum).
   * 
   * \param value for the new value.
   */
  void setValue(const double value) { dnum_ = value; };

  /**
   * \brief A method to set the bool value (i.e. RAPID bool).
   * 
   * \param value for the new value.
   */
  void setValue(const bool value) { bool_ = value; };

  /**
   * \brief A method to set the std::string value (i.e. RAPID string).
   * 
   * \param value for the new value.
   */
  void setValue(const std::string value) { string_ = value; };

  /**
   * \brief A method to parse a string value into the specified type (i.e. RAPID num, dnum, bool or string).
   * 
   * \param string_value containing the string to parse.
   */
  void parseStringValue(std::string string_value);

  /**
   * \brief A method to get a string value from the specified type (i.e. RAPID num, dnum, bool or string).
   * 
   * \return std::string containing the string.
   */
  std::string toString();

private:
  /**
   * \brief The used data type.
   */
  Type type_;

  /**
   * \brief The num value.
   */
  float num_;

  /**
   * \brief The dnum value.
   */
  double dnum_;

  /**
   * \brief The bool value.
   */
  bool bool_;

  /**
   * \brief The string value.
   */
  std::string string_;
};

/**
 * \brief A class for representing a RAPID RECORD.
 *
 * Note: Only for simple RAPID RECORDS (e.g. only containing atomic data types).
 */
class RAPIDRecord
{
public:
  /**
   * \brief A default constructor.
   */
  RAPIDRecord() : index_(0), NUM_VALUE(0.0), DNUM_VALUE(0.0), BOOL_VALUE(false), STRING_VALUE("") {}

  /**
   * \brief A method to parse a vector of strings, into the representation of the RAPID RECORD.
   * 
   * \param values containing the string  values to parse.
   *
   * \return bool indicating if the values were parsed or not.
   */
  bool parseStringValues(const std::vector<std::string> values);

  /**
   * \brief A method to construct a string representation of the RAPID RECORD.
   * 
   * \return std::string containing the string.
   */
  std::string constructString();

protected:
  /**
   * \brief Typedef for the elements in the vector container.
   */
  typedef std::pair<std::string, RAPIDAtomic> VectorElement;

  /**
   * \brief A method to find a component's index
   *
   * \param name containing the component name.
   * \param index for containing the found index.
   * 
   * \return bool indicating if the component was found.
   */
  bool findComponent(const std::string name, size_t* index);
  
  /**
   * \brief A method to get a component's float value
   *
   * \param name containing the component name.
   * \param for specifying that it is a float value.
   * 
   * \return float containing the retrived value.
   */
  float getComponentValue(const std::string name, float);

  /**
   * \brief A method to get a component's double value
   *
   * \param name containing the component name.
   * \param for specifying that it is a double value.
   * 
   * \return double containing the retrived value.
   */
  double getComponentValue(const std::string name, double);

  /**
   * \brief A method to get a component's bool value
   *
   * \param name containing the component name.
   * \param for specifying that it is a bool value.
   * 
   * \return bool containing the retrived value.
   */
  bool getComponentValue(const std::string name, bool);

  /**
   * \brief A method to get a component's string value
   *
   * \param name containing the component name.
   * \param for specifying that it is a string value.
   * 
   * \return std::string containing the retrived value.
   */
  std::string getComponentValue(const std::string name, std::string);
  
  /**
   * \brief A method to set a component's float value
   *
   * \param name containing the component name.
   * \param value containing the new value.
   */
  void setComponentValue(const std::string name, const float value);

  /**
   * \brief A method to set a component's double value
   *
   * \param name containing the component name.
   * \param value containing the new value.
   */
  void setComponentValue(const std::string name, const double value);

  /**
   * \brief A method to set a component's bool value
   *
   * \param name containing the component name.
   * \param value containing the new value.
   */
  void setComponentValue(const std::string name, const bool value);

  /**
   * \brief A method to set a component's string value
   *
   * \param name containing the component name.
   * \param value containing the new value.
   */
  void setComponentValue(const std::string name, const std::string value);

  /**
   * \brief A default float.
   */
  const float NUM_VALUE;

  /**
   * \brief A default double.
   */
  const double DNUM_VALUE;

  /**
   * \brief A default bool.
   */
  const bool BOOL_VALUE;

  /**
   * \brief A default std::string.
   */
  const std::string STRING_VALUE;

  /**
   * \brief An index.
   */
  size_t index_;

  /**
   * \brief The components in the RECORD.
   */
  std::vector<VectorElement> components_;
};

} // end namespace rws_interface
} // end namespace abb

#endif