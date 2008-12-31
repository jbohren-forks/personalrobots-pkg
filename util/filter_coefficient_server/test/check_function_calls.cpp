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

/* Author: Melonee Wise */

/*
 * Check that the server is working
 */

#include <gtest/gtest.h>
#include <time.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include "ros/node.h"
#include "filter_coefficient_server/Filter.h"

ros::node* n;

TEST(CheckFunctionCalls, checkButterLow)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  req.name = "butter";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.020083365564211);
  exp_b.push_back(0.040166731128423);
  exp_b.push_back(0.020083365564211);
  exp_a.push_back(1);
  exp_a.push_back(-1.561018075800718);
  exp_a.push_back(0.641351538057563);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}

TEST(CheckFunctionCalls, checkButterHigh)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("high");
  req.name = "butter";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.80059240346457);
  exp_b.push_back(-1.601184806929141);
  exp_b.push_back(0.80059240346457);
  exp_a.push_back(1);
  exp_a.push_back(-1.561018075800718);
  exp_a.push_back(0.641351538057563);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}

TEST(CheckFunctionCalls, checkButterBandstop3)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.4");
  req.name = "butter";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.131106439916626);
  exp_b.push_back(0);
  exp_b.push_back(-0.262212879833252);
  exp_b.push_back(0);
  exp_b.push_back(0.131106439916626);

  exp_a.push_back(1);
  exp_a.push_back(-2.180657838602801);
  exp_a.push_back(2.020004116183514);
  exp_a.push_back(-1.02551084771342);
  exp_a.push_back(0.272214937925008);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}

TEST(CheckFunctionCalls, checkButterBandstop4)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.4");
  args.push_back("stop");
  req.name = "butter";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.505001029045881);
  exp_b.push_back(-1.60308434315812);
  exp_b.push_back(2.282216996016778);
  exp_b.push_back(-1.60308434315812);
  exp_b.push_back(0.505001029045881);

  exp_a.push_back(1);
  exp_a.push_back(-2.180657838602801);
  exp_a.push_back(2.020004116183514);
  exp_a.push_back(-1.02551084771342);
  exp_a.push_back(0.272214937925008);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}




TEST(CheckFunctionCalls, checkCheby1Low)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.2");
  req.name = "cheby1";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.163091272386807);
  exp_b.push_back(0.326182544773613);
  exp_b.push_back(0.163091272386807);
  exp_a.push_back(1);
  exp_a.push_back(-0.613126393211069);
  exp_a.push_back(0.273045514473463);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}

TEST(CheckFunctionCalls, checkCheby1High)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.2");
  args.push_back("high");
  req.name = "cheby1";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.781804630284764);
  exp_b.push_back(-1.563609260569528);
  exp_b.push_back(0.781804630284764);
  exp_a.push_back(1);
  exp_a.push_back(-1.531327486602936);
  exp_a.push_back(0.632102518362357);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}

TEST(CheckFunctionCalls, checkCheby1Bandstop4)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.2");
  args.push_back("0.4");
  req.name = "cheby1";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.163091272386807);
  exp_b.push_back(0);
  exp_b.push_back(-0.326182544773613);
  exp_b.push_back(0);
  exp_b.push_back(0.163091272386807);

  exp_a.push_back(1);
  exp_a.push_back(-1.615000927903865);
  exp_a.push_back(1.333579953321333);
  exp_a.push_back(-0.716435767244677);
  exp_a.push_back(0.273045514473463);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}

TEST(CheckFunctionCalls, checkCheby1Bandstop5)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.2");
  args.push_back("0.4");
  args.push_back("stop");
  req.name = "cheby1";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.781804630284765);
  exp_b.push_back(-1.932727336312121);
  exp_b.push_back(2.75810044539647);
  exp_b.push_back(-1.932727336312121);
  exp_b.push_back(0.781804630284765);

  exp_a.push_back(1);
  exp_a.push_back(-2.182480412127352);
  exp_a.push_back(2.739650227468426);
  exp_a.push_back(-1.727734116072243);
  exp_a.push_back(0.632102518362356);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}


TEST(CheckFunctionCalls, checkCheby2Low)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.2");
  req.name = "cheby2";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.93695);
  exp_b.push_back(-1.22052);
  exp_b.push_back(0.93695);
  exp_a.push_back(1);
  exp_a.push_back(-1.23844);
  exp_a.push_back(0.89181);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}

TEST(CheckFunctionCalls, checkCheby2High)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.2");
  args.push_back("high");
  req.name = "cheby2";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.96731);
  exp_b.push_back(-1.74062);
  exp_b.push_back(0.96731);
  exp_a.push_back(1);
  exp_a.push_back(-1.73950);
  exp_a.push_back(0.93575);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}

TEST(CheckFunctionCalls, checkCheby2Bandstop4)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.2");
  args.push_back("0.4");
  req.name = "cheby2";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.93695);
  exp_b.push_back(-1.91245);
  exp_b.push_back(2.40248);
  exp_b.push_back(-1.91245);
  exp_b.push_back(0.93695);
  
  exp_a.push_back(1);
  exp_a.push_back(-2.00146);
  exp_a.push_back(2.43408);
  exp_a.push_back(-1.86773);
  exp_a.push_back(0.89181);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}

TEST(CheckFunctionCalls, checkCheby2Bandstop5)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.2");
  args.push_back("0.4");
  args.push_back("stop");
  req.name = "cheby2";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.96731);
  exp_b.push_back(-2.27143);
  exp_b.push_back(3.14444);
  exp_b.push_back(-2.27143);
  exp_b.push_back(0.96731);
  exp_a.push_back(1);
  exp_a.push_back(-2.31114);
  exp_a.push_back(3.14332);
  exp_a.push_back(-2.23172);
  exp_a.push_back(0.93575);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}


TEST(CheckFunctionCalls, checkEllipLow)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.2");
  args.push_back("0.4");
  req.name = "ellip";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.78475);
  exp_b.push_back(-0.34528);
  exp_b.push_back(0.78475);
  exp_a.push_back(1);
  exp_a.push_back(-0.62130);
  exp_a.push_back(0.85970);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}

TEST(CheckFunctionCalls, checkEllipHigh)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.2");
  args.push_back("0.4");
  args.push_back("high");
  req.name = "ellip";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.84596);
  exp_b.push_back(-0.66482);
  exp_b.push_back(0.84596);
  exp_a.push_back(1);
  exp_a.push_back(-0.52660);
  exp_a.push_back(0.85743);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}

TEST(CheckFunctionCalls, checkEllipBandstop5)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.2");
  args.push_back("0.4");
  args.push_back("0.7");
  req.name = "ellip";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.76574);
  exp_b.push_back(0.40912);
  exp_b.push_back(0.87057);
  exp_b.push_back(0.40912);
  exp_b.push_back(0.76574);

  exp_a.push_back(1);
  exp_a.push_back(0.55112);
  exp_a.push_back(1.23210);
  exp_a.push_back(0.50908);
  exp_a.push_back(0.88026);
  
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}

TEST(CheckFunctionCalls, checkEllipBandstop6)
{
  filter_coefficient_server::Filter::request  req;
  filter_coefficient_server::Filter::response res;
  std::vector<std::string> args;
  std::vector<double> exp_b;
  std::vector<double> exp_a;
  std::string temp;
  
  args.push_back("2");
  args.push_back("0.1");
  args.push_back("0.2");
  args.push_back("0.4");
  args.push_back("0.7");
  args.push_back("stop");
  req.name = "ellip";
  req.args = args;
  //hack for wait for service
  while (!ros::service::call("generate_filter_coeffs", req, res))
  {
    sleep(1);
  }
  double epsilon = 1e-4;
  
  exp_b.push_back(0.88370);
  exp_b.push_back(0.51106);
  exp_b.push_back(1.23320);
  exp_b.push_back(0.51106);
  exp_b.push_back(0.88370);

  exp_a.push_back(1);
  exp_a.push_back(0.53866);
  exp_a.push_back(1.15884);
  exp_a.push_back(0.49530);
  exp_a.push_back(0.87651);
  
  if (ros::service::call("generate_filter_coeffs", req, res))
  {
    for(unsigned int i=0; i<res.a.size(); i++)
    {
      EXPECT_NEAR(res.a[i], exp_a[i], epsilon);
    }
    for(unsigned int i=0; i<res.b.size(); i++)
    {
      EXPECT_NEAR(res.b[i], exp_b[i], epsilon);
    }
  }
  else 
  {
    ASSERT_TRUE(0);
  }
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv);
  n = new ros::node("check_fucntion_calls");

  int ret = RUN_ALL_TESTS();
  ros::fini();
  delete n;

  return ret;
}
