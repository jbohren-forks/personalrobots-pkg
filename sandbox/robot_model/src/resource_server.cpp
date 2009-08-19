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

#include <ros/ros.h>
#include <robot_model/GetResource.h>
#include <fstream>

class ResourceServer
{
public:
  ResourceServer(void)
  {
    serveRes_ = nh_.advertiseService("get_resource", &ResourceServer::getResource, this);
  }

  ~ResourceServer(void)
  {
  }

  void run(void)
  {
    ros::spin();
  }

private:

  bool getResource(robot_model::GetResource::Request &req, robot_model::GetResource::Response &res)
  {
    bool result = false;

    ROS_DEBUG("Serving '%s'", req.identifier.c_str());
    std::ifstream in(req.identifier.c_str(), std::ios::in | std::ios::binary);

    if (in.good())
    {
      in.seekg(0, std::ios::end);
      std::streampos sz = in.tellg();
      in.seekg(0, std::ios::beg);
      if (sz > 0)
      {
        res.content.resize(sz);
        in.read(reinterpret_cast<char*> (&res.content[0]), sz);
        if (in.gcount() != sz)
          res.content.clear();
        else
          result = true;
      }
      else if (sz == 0)
        result = true;
      in.close();
    }

    if (!result)
      ROS_ERROR("Errors encountered while loading '%s'", req.identifier.c_str());
    return result;
  }

  ros::NodeHandle nh_;
  ros::ServiceServer serveRes_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "resource_server");

  ResourceServer rs;
  rs.run();

  return 0;
}
