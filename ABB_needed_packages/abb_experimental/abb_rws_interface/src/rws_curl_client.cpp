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

#include <cstring>

#include "abb_rws_interface/rws_curl_client.h"

namespace abb
{
namespace rws_interface
{
/****************************************************************************************
 * Class definitions: CURLClient
 */

/********************************************
 * Primary methods
 */

CURLcode CURLClient::get(const std::string url_path_and_query, std::string* result)
{
  // Lock the object's mutex. It is released when the method goes out of scope.
  boost::lock_guard<boost::mutex> lock(mutex_);

  CURLcode code = CURLE_FAILED_INIT;

  clearBuffers();

  if(curl_)
  {
    setupCURLOptions(curl_, GET, base_url_ + url_path_and_query);
    code = curl_easy_perform(curl_);
  }

  *result = writedata_buffer_;

  return code;
}

CURLcode CURLClient::post(const std::string url_path_and_query, const std::string post_data)
{
  // Lock the object's mutex. It is released when the method goes out of scope.
  boost::lock_guard<boost::mutex> lock(mutex_);

  CURLcode code = CURLE_FAILED_INIT;

  clearBuffers();

  if(curl_)
  {
    setupCURLOptions(curl_, POST, base_url_ + url_path_and_query, post_data.c_str());
    code = curl_easy_perform(curl_);
  }

  return code;
}

std::string CURLClient::findSubstringContent(const std::string whole_string,
                                             const std::string substring_start,
                                             const std::string substring_end)
{
  std::string result;
  size_t start_postion = whole_string.find(substring_start);

  if (start_postion != std::string::npos)
  {
    start_postion += substring_start.size();
    size_t end_postion = whole_string.find_first_of(substring_end, start_postion);

    if (end_postion != std::string::npos)
    {
      result = whole_string.substr(start_postion, end_postion-start_postion);
    }
  }

  std::string quot = "&quot;";
  size_t quot_position = 0;
  do
  {
    quot_position = result.find(quot);
    if (quot_position != std::string::npos)
    {
      result.replace(quot_position, quot.size(), "");
    }
  }
  while (quot_position != std::string::npos);

  return result;
}

/********************************************
 * Auxiliary methods
 */

size_t CURLClient::headerCallback(char* buffer, size_t size, size_t nitems, void* userdata)
{
  ((std::string*)userdata)->append(buffer, size * nitems);
  return size * nitems;
}

size_t CURLClient::writeCallback(char* ptr, size_t size, size_t nmemb, void* userdata)
{
  ((std::string*)userdata)->append(ptr, size * nmemb);
  return size * nmemb;
}

void CURLClient::setupCURLOptions(CURL* curl, const CURLCase curl_case, const std::string url, const char* post_data)
{
  if(port_.compare("80") == 0)
  {
    curl_easy_setopt(curl, CURLOPT_USERNAME, user_.c_str());
    curl_easy_setopt(curl, CURLOPT_PASSWORD, password_.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_DIGEST);
  }
  curl_easy_setopt(curl, CURLOPT_COOKIEJAR, "cookies.txt");

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, headerCallback);
  curl_easy_setopt(curl, CURLOPT_HEADERDATA, &headerdata_buffer_);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &writedata_buffer_);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 150L);

  switch (curl_case)
  {
    case GET:
    {
      curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
    }
    break;

    case POST:
    {
      curl_easy_setopt(curl, CURLOPT_POST, 1L);
      curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long) std::strlen(post_data));
      curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_data);
    }
    break;
  }
}

void CURLClient::clearBuffers()
{
  headerdata_buffer_.clear();
  writedata_buffer_.clear();
}

} // end namespace rws_interface
} // end namespace abb
