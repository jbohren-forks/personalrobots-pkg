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

#include "gtest/gtest.h"
#include "filters/filter_chain.h"
#include "filters/median.h"
#include "filters/mean.h"

template <typename T>
class TestFilter : public filters::FilterBase<T>
{
public:
  TestFilter() { printf("Constructor\n");};
  
  ~TestFilter() { printf("Destructor\n");};

  virtual bool configure(unsigned int number_of_channels, TiXmlElement *config) 
  {
    printf("Configured with %d \n", number_of_channels);
    
    return true;
  };

  virtual bool update(const T & data_in, T& data_out)
  {
    printf("Update called\n");
    return true;
  };



};

ROS_REGISTER_FILTER(TestFilter, double)
ROS_REGISTER_FILTER(TestFilter, float)



TEST(FilterChain, configuring){
  double epsilon = 1e-9;
  printf("Chain test starting\n");
  filters::FilterChain<std_vector_float > chain;
  //filters::FilterChain<float> chain;


  //  chain.add("TestFilter", "");
  chain.add("<filter type=\"MeanFilter\" name=\"mean_test\"> <params number_of_observations=\"5\"/></filter>");
  printf("second\n");
  chain.add("<filter type=\"MedianFilter\" name=\"median_test\"> <params number_of_observations=\"5\"/></filter>");

  chain.configure(5);

  float input1[] = {1,2,3,4,5};
  float input1a[] = {9,9,9,9,9};//seed w/incorrect values
  std::vector<float> v1 (input1, input1 + sizeof(input1) / sizeof(float));
  std::vector<float> v1a (input1a, input1a + sizeof(input1a) / sizeof(float));

  
  EXPECT_TRUE(chain.update(v1, v1a));

  chain.clear();

  for (unsigned int i = 1; i < v1.size(); i++)
  {
    EXPECT_NEAR(input1[i], v1a[i], epsilon);
  }
  
}

TEST(FilterChain, Misconfigured){
  printf("Chain test starting\n");
  filters::FilterChain<std_vector_float > chain;
  //filters::FilterChain<float> chain;


  //  chain.add("TestFilter", "");
  chain.add("<filter type=\"MeanFilter\" name=\"mean_test\"> <params number_of_observations=\"5\"/></filter>");
  chain.add("<filter type=\"MedianFilter\" name=\"median_test\"> <params number_of_observations=\"5\"/></filter>");

  chain.configure(10);

  float input1[] = {1,2,3,4,5};
  float input1a[] = {1,2,3,4,5};
  std::vector<float> v1 (input1, input1 + sizeof(input1) / sizeof(float));
  std::vector<float> v1a (input1a, input1a + sizeof(input1a) / sizeof(float));

  
  EXPECT_FALSE(chain.update(v1, v1a));

  chain.clear();

}




int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
