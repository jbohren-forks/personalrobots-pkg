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

#include "filters/median.h"
#include "filters/mean.h"

void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};

void generate_rand_vectors(double scale, uint64_t runs, std::vector<double>& xvalues, std::vector<double>& yvalues, std::vector<double>&zvalues)
{
  seed_rand();
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
  }
}

TEST(MedianFilter, ConfirmIdentityNRows)
{
  double epsilon = 1e-6;
  int length = 5;
  int rows = 5;
  MedianFilter<float> filter(rows,length);
  float input1[] = {1,2,3,4,5};
  float input1a[] = {1,2,3,4,5};

  for (uint32_t i =0; i < rows*10; i++)
  {
    filter.update(input1, input1a);
    
    for (int i = 1; i < length; i++)
    {
      EXPECT_NEAR(input1[i], input1a[i], epsilon);
    }
  }
}

TEST(MedianFilter, ThreeRows)
{
  double epsilon = 1e-6;
  int length = 5;
  int rows = 5;
  MedianFilter<float> filter(rows,length);
  float input1[] = {0,1,2,3,4};
  float input2[] = {1,2,3,4,5};
  float input3[] = {2,3,4,5,6};
  float input1a[] = {1,2,3,4,5};

  filter.update(input1, input1a);
  filter.update(input2, input1a);
  filter.update(input3, input1a);
  
  for (int i = 1; i < length; i++)
  {
    EXPECT_NEAR(input2[i], input1a[i], epsilon);
  }
  
}

TEST(MeanFilter, ConfirmIdentityNRows)
{
  double epsilon = 1e-6;
  int length = 5;
  int rows = 5;
  MeanFilter filter(rows,length);
  double input1[] = {1,2,3,4,5};
  double input1a[] = {1,2,3,4,5};

  for (uint32_t i =0; i < rows*10; i++)
  {
    filter.update(input1, input1a);
    
    for (int i = 1; i < length; i++)
    {
      EXPECT_NEAR(input1[i], input1a[i], epsilon);
    }
  }
}

TEST(MeanFilter, ThreeRows)
{
  double epsilon = 1e-6;
  int length = 5;
  int rows = 5;
  MeanFilter filter(rows,length);
  double input1[] = {0,1,2,3,4};
  double input2[] = {1,2,3,4,5};
  double input3[] = {2,3,4,5,6};
  double input1a[] = {1,2,3,4,5};

  filter.update(input1, input1a);
  filter.update(input2, input1a);
  filter.update(input3, input1a);
  
  for (int i = 1; i < length; i++)
  {
    EXPECT_NEAR(input2[i], input1a[i], epsilon);
  }
  
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
