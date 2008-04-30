/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "ros/ros_slave.h"
#include "common_flows/FlowFloat32.h"

class TemplateNode : public ROS_Slave
{
public:
  FlowFloat32 *float_in;
  FlowFloat32 *float_out;

  double param;

  TemplateNode() : ROS_Slave()
  {
    register_source(float_out = new FlowFloat32("float_out"));
    register_sink(float_in = new FlowFloat32("float_in"), ROS_CALLBACK(TemplateNode, float_callback));
    register_with_master();
    if (!get_double_param(".some_param", &param))
      param = 0;
    printf("package path is [%s]\n", get_my_package_path().c_str());
  }

  virtual ~TemplateNode()
  { 

  }

  void float_callback() {
    printf("Look, I got my own value: %g\n", float_in->data);
  }

  void publish_float() {
    float_out->data = param;
    float_out->publish();
  }
  

};

int main(int argc, char **argv)
{
  TemplateNode a;
  while (a.happy()) {
    a.publish_float();
    usleep(1000);
  }
  return 0;
}

