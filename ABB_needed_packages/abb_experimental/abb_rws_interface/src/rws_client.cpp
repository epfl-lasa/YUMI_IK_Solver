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

#include "abb_rws_interface/rws_client.h"

namespace abb
{
namespace rws_interface
{
/****************************************************************************************
 * Class definitions: RWSClient
 */

/********************************************
 * Primary methods
 */

std::string RWSClient::getIOSignal(const std::string iosignal)
{
  std::string result = "";
  get("/rw/iosystem/signals/" + iosignal, &result);
  return findSubstringContent(result, "<span class=\"lvalue\">", "<");
}

std::string RWSClient::getRAPIDExecution()
{
  std::string result = "";
  get("/rw/rapid/execution", &result);
  return findSubstringContent(result, "<span class=\"ctrlexecstate\">", "<");
}

std::string RWSClient::getPanelOperationMode()
{
  std::string result = "";
  get("/rw/panel/opmode", &result);
  return findSubstringContent(result, "<span class=\"opmode\">", "<");
}

std::string RWSClient::getRAPIDSymbolData(const std::string rapid_task,
                                          const std::string rapid_module,
                                          const std::string symbol_name)
{
  std::string result = "";
  get("/rw/rapid/symbol/data/RAPID/" + rapid_task + "/" + rapid_module + "/" + symbol_name, &result);
  return findSubstringContent(result, "<span class=\"value\">", "<");
}

void RWSClient::setIOSignal(const std::string iosignal, const std::string value)
{
  post("/rw/iosystem/signals/" + iosignal + "?action=set",
       "lvalue=" + value);
}

void RWSClient::setRAPIDSymbolData(const std::string rapid_task,
                                   const std::string rapid_module,
                                   const std::string symbol_name,
                                   const std::string data)
{
  post("/rw/rapid/symbol/data/RAPID/" + rapid_task + "/" + rapid_module + "/" + symbol_name + "?action=set",
       "value=" + data);
}

std::vector<std::string> RWSClient::extractDelimitedSubstrings(const std::string input)
{
  std::vector<std::string> substrings;
  std::string temp = "";
  size_t position = 0;

  std::stringstream ss;
  ss << input;

  while (std::getline(ss, temp, '['))
  {
    if (!temp.empty())
    {
      position = temp.find(']');
      if (position != std::string::npos)
      {
        temp.erase(position, std::string::npos);
      }
      substrings.push_back(temp);
    }
  }

  return substrings;
}

std::vector<std::string> RWSClient::extractDelimitedValues(const std::string input)
{
  std::vector<std::string> values;
  std::string temp = "";

  std::stringstream ss;
  ss << input;

  while (std::getline(ss, temp, ','))
  {
    if (!temp.empty())
    {
      values.push_back(temp);
    }
  }

  return values;
}

} // end namespace rws_interface
} // end namespace abb
