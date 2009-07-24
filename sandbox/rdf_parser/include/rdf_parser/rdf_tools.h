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

/* Author: Wim Meeussen */

#ifndef RDF_TOOLS_H
#define RDF_TOOLS_H

#include <string>
#include <map>
#include <tinyxml/tinyxml.h>
#include <boost/function.hpp>

using namespace std;

namespace rdf_parser{

bool getAttribute(TiXmlElement *xml, const string& name, string& attr)
{
  if (!xml) return false;
  const char *attr_char = xml->Attribute(name.c_str());
  if (!attr_char) return false;
  attr = string(attr_char);
  return true;
}

/* this is commented out because the check is not robust enough, look for better atof checkers
bool checkNumber(const char& c)
{
  return (c=='1' || c=='2' ||c=='3' ||c=='4' ||c=='5' ||c=='6' ||c=='7' ||c=='8' ||c=='9' ||c=='0' ||c=='.' ||c=='-' ||c==' ');
};

bool checkNumber(const std::string& s)
{
  for (unsigned int i=0; i<s.size(); i++)
    if (!checkNumber(s[i])) return false;
  return true;
};
*/




}

#endif
