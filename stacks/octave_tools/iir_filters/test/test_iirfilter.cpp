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

#include <gtest/gtest.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include "ros/node.h"

#include "iir_filters/iir.h"
#include "filters/filter_base.h"

ros::Node* n;

using namespace iir_filters;


TEST(IIRFilter, Configure)
{
  iir_filters::Filter::Request  req;
  iir_filters::Filter::Response res;
  std::vector<std::string> args;
  
  args.push_back("2");
  args.push_back("0.1");
  req.name = "butter";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("filter_coeffs", req, res))
  {
    sleep(1);
  }

  TiXmlDocument doc;
  doc.Parse("<filter type=\"IIRFilter\" name=\"iirfilter_test\"> <params name=\"butter\"args=\"2 .1 high\"/></filter>"); 
  TiXmlElement *config = doc.RootElement();
	filters::FilterBase<double> * filter = new IIRFilter<double> ();
	EXPECT_TRUE(filter->configure(5,config));
}

TEST(IIRFilter, LowPass)
{

  iir_filters::Filter::Request  req;
  iir_filters::Filter::Response res;
  std::vector<std::string> args;
  
  args.push_back("2");
  args.push_back("0.1");
  req.name = "butter";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("filter_coeffs", req, res))
  {
    sleep(1);
  }
  
  double epsilon = 1e-4;
  
  TiXmlDocument doc;
  doc.Parse("<filter type=\"IIRFilter\" name=\"iirfilter_test\"> <params name=\"butter\" args=\"1 .2\"/></filter>"); 
  TiXmlElement *config = doc.RootElement();
	filters::FilterBase<double> * filter = new IIRFilter<double> ();
	EXPECT_TRUE(filter->configure(1,config));

  std::vector<double> in1,in2,in3,in4,in5,in6,in7;
  std::vector<double> out1;

  in1.push_back(10.0);
  in2.push_back(70.0);
  in3.push_back(10.0);
  in4.push_back(44.0);
  in5.push_back(10.0);
  in6.push_back(5.0);
  in7.push_back(6.0);
  out1.push_back(11.8008);
  EXPECT_TRUE(filter->update(in1, in1));
  filter->update(in2, in2);
  filter->update(in3, in3);
  filter->update(in4, in4);
  filter->update(in5, in5);
  filter->update(in6, in6);
  filter->update(in7, in7);

  EXPECT_NEAR(out1[0], in7[0], epsilon);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  
  ros::init(argc, argv);
  n = new ros::Node("test_iirfilter");

  int ret = RUN_ALL_TESTS();
  
  delete n;

  return ret;
}
