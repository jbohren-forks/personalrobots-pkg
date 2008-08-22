/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <string_utils/string_utils.h>

void string_utils::split(const std::string &s, std::vector<std::string> &t, const std::string &d)
{
  t.clear();
  std::size_t start = 0, end;
  while ((end = s.find_first_of(d, start)) != std::string::npos)
  {
    t.push_back(s.substr(start, end-start));
    start = end + 1;
  }
  t.push_back(s.substr(start));
}

void string_utils::split(const std::string &str, std::vector<std::string> &tokens)
{
  std::size_t start = 0, i;

  while (true)
  {
    while (start < str.size() && isspace(str[start]))
      ++start;
    if (start == str.size())
      return;

    i = start;
    while (i < str.size() && !isspace(str[i]))
      ++i;

    tokens.push_back(str.substr(start, i - start));
    if (i == str.size())
      return;

    start = i;
  }
}

std::string string_utils::trim(const std::string &str)
{
    std::string tmp = str;
    tmp = tmp.erase(tmp.find_last_not_of(" \t\r\n") + 1);
    return tmp.erase(0, tmp.find_first_not_of(" \t\r\n"));
}

std::string string_utils::tolower(const std::string& source)
{
    std::string result = source;
    for (unsigned int i = 0 ; i < result.size() ; i++)
	result[i] = ::tolower(result[i]);
    return result;    
}

std::string string_utils::toupper(const std::string& source)
{
    std::string result = source;
    for (unsigned int i = 0 ; i < result.size() ; i++)
	result[i] = ::toupper(result[i]);
    return result;    
}
