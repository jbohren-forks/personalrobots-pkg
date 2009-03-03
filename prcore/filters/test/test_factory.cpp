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
#include "filters/filter_base.h"

template <typename T>
class TestFilter : public filters::FilterBase<T>
{
public:
  TestFilter() { printf("Constructor\n");};
  
  ~TestFilter() { printf("Destructor\n");};

  virtual bool configure() 
  {
    printf("Configured with %d\n", this->number_of_channels_);
    return true;
  };

  virtual bool update(const T& data_in, T& data_out)
  {
    printf("Update called\n");
    return true;
  };
  virtual bool update(const std::vector<T>& data_in, std::vector<T>& data_out)
  {
    printf("Update called\n");
    return true;
  };



};

ROS_REGISTER_FILTER(TestFilter, double)
ROS_REGISTER_FILTER(TestFilter, float)
ROS_REGISTER_FILTER(TestFilter, int)



TEST(FilterChain, configuring)
{
  std::string name = filters::getFilterID<float>("TestFilter");
  filters::FilterBase<float> * a_filter = filters::FilterFactory<float>::Instance().CreateObject(name);
  name = filters::getFilterID<float>("TestFilter");
  filters::FilterBase<float> * a1_filter = filters::FilterFactory<float>::Instance().CreateObject(name);
  //filters::FilterBase<int> * a1_filter = filters::FilterFactory<int>::Instance().CreateObject("TestFilter<int>");
  name = filters::getFilterID<double>("TestFilter");
  filters::FilterBase<double> * b_filter = filters::FilterFactory<double>::Instance().CreateObject(name);

  printf("a is of type: %s\na1 is of type: %s\n b is of type: %s \n",
         a_filter->getType().c_str(), 
         a1_filter->getType().c_str(), 
         b_filter->getType().c_str());

  TiXmlDocument doc;
  doc.Parse("<filter type=\"MeanFilter\" name=\"mean_test\"> <params number_of_observations=\"5\"/></filter>"); 
  TiXmlElement *config = doc.RootElement();

  a_filter->configure(4, config);

  delete a_filter;
  delete a1_filter;
  delete b_filter;

  EXPECT_TRUE("SUCCESS");
};




int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
