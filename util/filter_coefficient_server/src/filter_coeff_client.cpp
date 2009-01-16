/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

// Original version: Melonee Wise <mwise@willowgarage.com>

#include <string>
#include <vector>
#include "ros/node.h"
#include "filter_coefficient_server/Filter.h"

class GenFilter : public ros::Node
{
public:
  GenFilter() : ros::Node("filter_coeff_client")
  {
  }
  bool call_add(std::string name, std::vector<std::string> args, std::vector<double> &b, std::vector<double> &a)
  {
    filter_coefficient_server::Filter::request  req;
    filter_coefficient_server::Filter::response res;
    req.name = name;
    req.args = args;
    if (ros::service::call("filter_coeffs", req, res))
    {
      for(uint32_t i=0; i<res.a.size();i++)
      { 
        a.push_back(res.a[i]);
        b.push_back(res.b[i]);
      }
      return true;
    }
    else
    {
      a.push_back(0);
      b.push_back(0);
      return false;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  if (argc < 3)
  {
    printf("usage: filter_coeff_client name args\n");
    printf("usage: filter_coeff_client butter 2 .1\n");
    printf("usage: filter_coeff_client butter 2 .1 high\n");
    printf("usage: filter_coeff_client butter 2 .1 .5 stop\n");
    return 1;
  }
  GenFilter a;
  std::vector<double> tf_a;
  std::vector<double> tf_b;
  std::vector<std::string> args;
  for(int i=2; i<argc; i++)
  {
    args.push_back(argv[i]);
  }
  
  if (a.call_add(argv[1], args, tf_b, tf_a))
  {
    for(uint32_t i=0; i<tf_a.size();i++)
    {
      printf("a[%d]:%f b[%d]:%f \n",i,i,tf_b[i], tf_a[i]);
    }
  }
  else
    printf("The filter name does not exist or the wrong arguments were sent\n");
  ros::fini();
  return 0;
}

