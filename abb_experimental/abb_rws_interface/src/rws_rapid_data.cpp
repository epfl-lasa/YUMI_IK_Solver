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

#include <sstream>

#include "abb_rws_interface/rws_rapid_data.h"

namespace abb
{
namespace rws_interface
{
/****************************************************************************************
 * Class definitions: RAPIDAtomic
 */

/********************************************
 * Auxiliary methods
 */

void RAPIDAtomic::parseStringValue(std::string string_value)
{
  std::stringstream ss;
  ss.str(string_value);

  switch(type_)
  {
    case NUM:
      ss >> num_;
    break;
    case DNUM:
      ss >> dnum_;
    break;
    case BOOL:
      bool_ = (string_value.compare("TRUE") == 0 ? true : false);
    break;
    case STRING:
      ss >> string_;
    break;
  }
}

std::string RAPIDAtomic::toString()
{
  std::stringstream ss;

  switch(type_)
  {
    case NUM:
      ss << num_;
    break;
    case DNUM:
      ss << dnum_;
    break;
    case BOOL:
      ss << (bool_ ? "TRUE" : "FALSE");
    break;
    case STRING:
      ss << "\"" << string_ << "\"";
    break;
  }

  return ss.str();
}




/****************************************************************************************
 * Class definitions: RAPIDRecord
 */

/********************************************
 * Primary methods
 */

bool RAPIDRecord::parseStringValues(const std::vector<std::string> values)
{
  bool valid = false;

  if (values.size() == components_.size())
  {
    for (size_t i = 0; i < values.size(); ++i)
    {
      components_.at(i).second.parseStringValue(values.at(i));
    }

    valid = true;
  }

  return valid;
}

std::string RAPIDRecord::constructString()
{
  std::stringstream ss;

  ss << "[";

  for (size_t i = 0; i < components_.size(); ++i)
  {
    ss << components_.at(i).second.toString();

    if (i != components_.size() - 1)
    {
      ss << ",";
    }
  }

  ss << "]";

  return ss.str();
}

/********************************************
 * Auxiliary methods
 */

bool RAPIDRecord::findComponent(const std::string name, size_t* index)
{
  bool found = false;

  for (size_t i = 0; i < components_.size(); ++i)
  {
    if (components_.at(i).first.compare(name) == 0)
    {
      *index = i;
      found = true;
    }
  }

  return found;
}
  
float RAPIDRecord::getComponentValue(const std::string name, float)
{
  float value = 0;

  if (findComponent(name, &index_))
  {
    components_.at(index_).second.getValue(&value);
  }

  return value;
}

double RAPIDRecord::getComponentValue(const std::string name, double)
{
  double value = 0;

  if (findComponent(name, &index_))
  {
    components_.at(index_).second.getValue(&value);
  }

  return value;
}

bool RAPIDRecord::getComponentValue(const std::string name, bool)
{
  bool value = false;

  if (findComponent(name, &index_))
  {
    components_.at(index_).second.getValue(&value);
  }

  return value;
}

std::string RAPIDRecord::getComponentValue(const std::string name, std::string)
{
  std::string value = "";

  if (findComponent(name, &index_))
  {
    components_.at(index_).second.getValue(&value);
  }

  return value;
}
  
void RAPIDRecord::setComponentValue(const std::string name, const float value)
{
  if (findComponent(name, &index_))
  {
    components_.at(index_).second.setValue(value);
  }
}

void RAPIDRecord::setComponentValue(const std::string name, const double value)
{
  if (findComponent(name, &index_))
  {
    components_.at(index_).second.setValue(value);
  }
}

void RAPIDRecord::setComponentValue(const std::string name, const bool value)
{
  if (findComponent(name, &index_))
  {
    components_.at(index_).second.setValue(value);
  }
}

void RAPIDRecord::setComponentValue(const std::string name, const std::string value)
{
  if (findComponent(name, &index_))
  {
    components_.at(index_).second.setValue(value);
  }
}

} // end namespace rws_interface
} // end namespace abb
