/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef POLYGON_HPP
#define POLYGON_HPP

#include <string>
#include <set>
#include <vector>
#include <iostream>

#include "boost/algorithm/string.hpp"

template<class T>
class base_polygon
{
public:
  base_polygon(): side_length_(0) {};
protected:
  T side_length_;
};

class polygon : public base_polygon<double>{

public:
  polygon(){}

  virtual ~polygon() {}

  void set_side_length(double side_length) {
    side_length_ = side_length;
  }

  virtual double area() const = 0;
};


std::set<std::string> findPlugins(const std::string & package, const std::string& type)
{
  std::set<std::string> output;
    
  std::string cmd = "rospack depends-on1 plugin_user | xargs -L1 rospack export --lang=filter --attrib=" + type;
  std::cout << cmd << std::endl;
  FILE* pipe = popen(cmd.c_str(), "r");
  std::string output_str;
    
  if (pipe)
  {
    char rospack_output[1024];
    while (fgets(rospack_output, 1024, pipe))
    {
      output_str += rospack_output;
    }
      
    pclose(pipe);
  }

  std::cout << output_str << std::endl;
  
  std::vector<std::string> output_vec;
  
  boost::split(output_vec, output_str, boost::is_any_of("\n "));
  
  for (std::vector<std::string>::iterator it = output_vec.begin(); it != output_vec.end() ; it++)
  {
    if (it->size() == 0) continue;
    output.insert(*it);
    std::cout << "adding " << *it << std::endl;
  };
  return output;

};



#endif
