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

#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "ros/node.h"
#include "filters/transfer_function.h"
#include "filter_coefficient_server/Filter.h"



class ExampleFilter : public ros::node
{
public:
  ExampleFilter() : ros::node("filtering_example")
  {
  }
  bool call_srv(std::string name, std::vector<std::string> args)
  {
    filter_coefficient_server::Filter::request  req;
    filter_coefficient_server::Filter::response res;
    req.name = name;
    req.args = args;
    std::vector<double> a;
    std::vector<double> b;
    std::vector<std::vector<double> > in;
    std::vector<std::vector<double> > out;
    std::vector<double> temp(2);
    
    //hack for wait for service
    while (!ros::service::call("generate_filter_coeffs", req, res))
    {
      sleep(1);
    }
    
    if (ros::service::call("generate_filter_coeffs", req, res))
    {

      std::ofstream outfile("output.txt");
      for(uint32_t i=0; i<res.a.size();i++)
      {
        a.push_back(res.a[i]);
        b.push_back(res.b[i]);
      }
      
      //create a filter using the coeffs
      filters::TransferFunctionFilter<double> filter(b,a,2);
      
      //pass in a simple sinewave
      for(uint32_t i=0; i<250; i++)
      {       
        temp[0]=sin(2*M_PI*i/25);
        temp[1]=cos(2*M_PI*i/50);
        in.push_back(temp);
        filter.update(&temp, &temp);
        out.push_back(temp);       
        outfile << in[i][0]<<", "<<out[i][0]<<", "<<in[i][1]<<", "<<out[i][1]<<"\n";
      }
      outfile.close();
      return true;
    }
    else
    {
      return false;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  ExampleFilter a;
  
  std::vector<std::string> args;
  for(int i=2; i<argc; i++)
  {
    args.push_back(argv[i]);
  }

  if (a.call_srv(argv[1], args))
  {
  }
  else
    printf("The filter name does not exist or the wrong arguments were sent\n");
  ros::fini();
  return 0;
}

