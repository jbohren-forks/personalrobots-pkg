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
#include <vector>
#include "filters/transfer_function.h"

using namespace filters ;

TEST(TransferFunctionFilter, Compile)
{
  std::vector<double> a;
  std::vector<double> b;
  a.push_back(1.0);
  b.push_back(3.0);
  TransferFunctionFilter<double > compile(b,a,7);

}

TEST(TransferFunctionFilter, LowPass)
{
  double epsilon = 1e-4;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> in1,in2,in3,in4,in5,in6,in7;
  std::vector<double> out1;
  a.push_back(1.0);
  a.push_back(-0.509525449494429);
  b.push_back(0.245237275252786);
  b.push_back(0.245237275252786);
  TransferFunctionFilter<double> filter(b,a,1);
  in1.push_back(10.0);
  in2.push_back(70.0);
  in3.push_back(10.0);
  in4.push_back(44.0);
  in5.push_back(10.0);
  in6.push_back(5.0);
  in7.push_back(6.0);
  out1.push_back(11.8008);
  filter.update(&in1);
  filter.update(&in2);
  filter.update(&in3);
  filter.update(&in4);
  filter.update(&in5);
  filter.update(&in6);
  filter.update(&in7);

  EXPECT_NEAR(out1[0], in7[0], epsilon);
}

TEST(TransferFunctionFilter, LowPassNonUnity)
{
  double epsilon = 1e-4;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> in1,in2,in3,in4,in5,in6,in7;
  std::vector<double> out1;
  a.push_back(2.0);
  a.push_back(-0.509525449494429);
  b.push_back(0.245237275252786);
  b.push_back(0.245237275252786);
  TransferFunctionFilter<double> filter(b,a,1);
  in1.push_back(10.0);
  in2.push_back(70.0);
  in3.push_back(10.0);
  in4.push_back(44.0);
  in5.push_back(10.0);
  in6.push_back(5.0);
  in7.push_back(6.0);
  out1.push_back(2.4088);
  filter.update(&in1);
  filter.update(&in2);
  filter.update(&in3);
  filter.update(&in4);
  filter.update(&in5);
  filter.update(&in6);
  filter.update(&in7);

  EXPECT_NEAR(out1[0], in7[0], epsilon);
}

TEST(TransferFunctionFilter, LowPassMulti)
{
  double epsilon = 1e-4;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> in1,in2,in3,in4,in5,in6,in7;
  std::vector<double> out1;
  a.push_back(1.0);
  a.push_back(-1.760041880343169);
  a.push_back(1.182893262037831);
  a.push_back(-0.278059917634546);
  b.push_back(0.018098933007514);
  b.push_back(0.245237275252786);
  b.push_back(0.054296799022543);
  b.push_back(0.018098933007514);
  TransferFunctionFilter<double> filter(b,a,1);
  in1.push_back(10.0);
  in1.push_back(10.0);
  in1.push_back(10.0);
  //
  in2.push_back(70.0);
  in2.push_back(30.0);
  in2.push_back(8.0);
  //
  in3.push_back(-1.0);
  in3.push_back(5.0);
  in3.push_back(22.0);
  //
  in4.push_back(44.0);
  in4.push_back(23.0);
  in4.push_back(8.0);
  //
  in5.push_back(10.0);
  in5.push_back(10.0);
  in5.push_back(10.0);
  //
  in6.push_back(5.0);
  in6.push_back(-1.0);
  in6.push_back(5.0);
  //
  in7.push_back(6.0);
  in7.push_back(-30.0);
  in7.push_back(2.0);
  //
  out1.push_back(60.6216);
  out1.push_back(33.9829);
  out1.push_back(28.1027);
  filter.update(&in1);
  filter.update(&in2);
  filter.update(&in3);
  filter.update(&in4);
  filter.update(&in5);
  filter.update(&in6);
  filter.update(&in7);

  for(unsigned int i=0; i<out1.size(); i++)
  {
    EXPECT_NEAR(out1[i], in7[i], epsilon);
  }
}

TEST(TransferFunctionFilter, LowPassIrrational)
{
  double epsilon = 1e-4;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> in1,in2,in3,in4,in5,in6,in7;
  std::vector<double> out1;
  a.push_back(1.0);
  a.push_back(-1.760041880343169);
  a.push_back(1.182893262037831);
  b.push_back(0.018098933007514);
  b.push_back(0.054296799022543);
  b.push_back(0.054296799022543);
  b.push_back(0.018098933007514);
  TransferFunctionFilter<double> filter(b,a,3);
  in1.push_back(10.0);
  in1.push_back(10.0);
  in1.push_back(10.0);
  //
  in2.push_back(70.0);
  in2.push_back(30.0);
  in2.push_back(8.0);
  //
  in3.push_back(-1.0);
  in3.push_back(5.0);
  in3.push_back(22.0);
  //
  in4.push_back(44.0);
  in4.push_back(23.0);
  in4.push_back(8.0);
  //
  in5.push_back(10.0);
  in5.push_back(10.0);
  in5.push_back(10.0);
  //
  in6.push_back(5.0);
  in6.push_back(-1.0);
  in6.push_back(5.0);
  //
  in7.push_back(6.0);
  in7.push_back(-30.0);
  in7.push_back(2.0);
  //
  out1.push_back(17.1112);
  out1.push_back(9.0285);
  out1.push_back(8.3102);
  filter.update(&in1);
  filter.update(&in2);
  filter.update(&in3);
  filter.update(&in4);
  filter.update(&in5);
  filter.update(&in6);
  filter.update(&in7);

  for(unsigned int i=0; i<out1.size(); i++)
  {
    EXPECT_NEAR(out1[i], in7[i], epsilon);
  }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
