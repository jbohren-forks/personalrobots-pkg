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

#include <gtest/gtest.h>
#include <sys/time.h>
#include <vector>
#include "misc_utils/ring_buffer.h"


TEST(RingBuffer, Compile)
{
  RingBuffer<std::vector<double> > compile(5);

}


TEST(RingBuffer, SingleRing)
{
  double epsilon = 1e-6;
  RingBuffer<std::vector<double> > buffer(3);
  std::vector<double > in1,in2,in3,in4;
  in1.push_back(5.0);
  in2.push_back(0.0);
  in3.push_back(-1.5);
  in4.push_back(-28.0);
  buffer.push(in1);
  buffer.push(in2);
  buffer.push(in3);
  std::vector<double> out=buffer[1];
  EXPECT_NEAR(out[0], in2[0], epsilon);
}

TEST(RingBuffer, SingleRing2)
{
  double epsilon = 1e-6;
  RingBuffer<std::vector<double> > buffer(3);
  std::vector<double> in1,in2,in3,in4;
  in1.push_back(5.0);
  in2.push_back(0.0);
  in3.push_back(-1.5);
  in4.push_back(-28.0);
  buffer.push(in1);
  buffer.push(in2);
  buffer.push(in3);
  buffer.push(in4);
  buffer.push(in4);
  buffer.push(in2);
  std::vector<double> out=buffer[1];
  EXPECT_NEAR(out[0], in4[0], epsilon);
}

TEST(RingBuffer, MultiRing)
{
  double epsilon = 1e-6;
  RingBuffer<std::vector<double> > buffer(3);
  std::vector<double> in1,in2,in3,in4;
  in1.push_back(5.0);
  in1.push_back(8.0);
  in1.push_back(12.0);
  in2.push_back(0.0);
  in2.push_back(0.0);
  in2.push_back(9.0);
  in3.push_back(-1.5);
  in3.push_back(-5.5);
  in3.push_back(5.5);
  in4.push_back(-28.0);
  in4.push_back(32.0);
  in4.push_back(12.0);
  buffer.push(in1);
  buffer.push(in2);
  buffer.push(in3);
  buffer.push(in4);
  buffer.push(in4);
  buffer.push(in2);
  std::vector<double> out=buffer[1];
  for(int i=0; i<out.size();i++)
  {
    EXPECT_NEAR(out[i], in4[i], epsilon);
  }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
