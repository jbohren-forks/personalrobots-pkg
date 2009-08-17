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
#include "filters/filter_plugin_chain.h"
#include "filters/median.h"
#include "filters/mean.h"
#include <sstream>


static std::string mean_filter_5 = "<filter type=\"DoubleMeanFilter\" name=\"mean_test_5\"> <params number_of_observations=\"5\"/></filter>";
static std::string median_filter_5 = "<filter type=\"MedianFilter\" name=\"median_test_5\"> <params number_of_observations=\"5\"/></filter>";


TEST(FilterChain, configuring){
  double epsilon = 1e-9;
  printf("Chain test starting\n");
  filters::FilterPluginChain<double> chain("filters", "double_filter");
  //filters::FilterChain<float> chain;

  // EXPECT_TRUE(chain.add(mean_filter_5));
 
  //  EXPECT_TRUE(chain.add(median_filter_5));
  TiXmlDocument chain_def = TiXmlDocument();
  chain_def.Parse(mean_filter_5.c_str());
  TiXmlElement * config = chain_def.RootElement();
  EXPECT_TRUE(chain.configure(config));
 
  double input = 1;
  double output = 9;
  
  EXPECT_TRUE(chain.update(input, output));

  chain.clear();

  EXPECT_NEAR(input, output, epsilon);
  
}

#if 0
TEST(FilterChain, MisconfiguredNumberOfChannels){
  filters::FilterChain<float> chain;


  //  EXPECT_TRUE(chain.add(mean_filter_5));
  //EXPECT_TRUE(chain.add(median_filter_5));
  TiXmlDocument chain_def = TiXmlDocument();
  chain_def.Parse(median_filter_5.c_str());
  TiXmlElement * config = chain_def.RootElement();
  EXPECT_TRUE(chain.configure(10, config));

  //  EXPECT_TRUE(chain.configure(10));

  float input1[] = {1,2,3,4,5};
  float input1a[] = {1,2,3,4,5};
  std::vector<float> v1 (input1, input1 + sizeof(input1) / sizeof(float));
  std::vector<float> v1a (input1a, input1a + sizeof(input1a) / sizeof(float));

  
  EXPECT_FALSE(chain.update(v1, v1a));

  chain.clear();

}
TEST(FilterChain, OverlappingNames){
  filters::FilterChain<float> chain;


  std::string bad_xml = "<filters> <filter type=\"MeanFilter\" name=\"mean_test\"> <params number_of_observations=\"5\"/></filter><filter type=\"MedianFilter\" name=\"mean_test\"> <params number_of_observations=\"5\"/></filter></filters>";

  TiXmlDocument chain_def = TiXmlDocument();
  chain_def.Parse(bad_xml.c_str());
  TiXmlElement * config = chain_def.RootElement();

  EXPECT_FALSE(chain.configure(5, config));

}

/**
 * This test tests the execution of filter chains of varying sizes from 1 to 10
 * to exercise the different cases in FilterChain::update()
 */
TEST(FilterChain, LongChainExecution) {

  for (int num_filters=1; num_filters<=10; num_filters++)
  {
    filters::FilterChain<int> chain;
    std::ostringstream sstr;

    sstr << "<filters>";
    for (int i=0; i<num_filters; i++)
    {
      sstr << "<filter type=\"IncrementFilter\" name=\"inc_" <<  i << "\"/>";
    }
    sstr << "</filters>";

    chain.configureFromXMLString(1, sstr.str());

    int input = 1;
    int output = 0;

    int expected = input+num_filters;

    chain.update(input, output);
    EXPECT_EQ(expected, output);
  }
}
#endif //0
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
