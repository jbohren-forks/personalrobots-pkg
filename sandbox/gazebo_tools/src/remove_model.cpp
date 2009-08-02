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

#include <cstdio>
#include <cstdlib>

#include <string>
#include <sstream>

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

#include "ros/ros.h"

void usage(const char *progname)
{
    printf("\nUsage: %s gazebo_model_name\n", progname);
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        usage(argv[0]);
        exit(1);
    }

    std::string robot_model_name(argv[1]);

    // connect to gazebo
    gazebo::Client *client = new gazebo::Client();
    gazebo::FactoryIface *factoryIface = new gazebo::FactoryIface();

    int serverId = 0;

    bool connected_to_server = false;
    /// Connect to the libgazebo server
    while (!connected_to_server)
    {
      try
      {
        client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
        connected_to_server = true;
      }
      catch (gazebo::GazeboError e)
      {
        ROS_ERROR("Gazebo error: Unable to connect\n %s\n",e.GetErrorStr().c_str());
        usleep(1000000);
        connected_to_server = false;
      }
    }

    /// @todo: hack, waiting for system to startup, find out why ConnectWait goes through w/o locking in gazebo
    usleep(2000000);

    /// Open the Factory interface
    try
    {
      factoryIface->Open(client, "default");
    }
    catch (gazebo::GazeboError e)
    {
      ROS_ERROR("Gazebo error: Unable to connect to the factory interface\n%s\n",e.GetErrorStr().c_str());
      return -1;
    }

    bool writing_iface = true;
    while (writing_iface)
    {
      factoryIface->Lock(1);
      if (strcmp((char*)factoryIface->data->deleteModel,"")==0)
      {
        ROS_INFO("Deleting Robot Model Name:%s in Gazebo\n",robot_model_name.c_str());
        // don't overwrite data, only write if iface data is empty
        strcpy((char*)factoryIface->data->deleteModel, robot_model_name.c_str());
        writing_iface = false;
      }
      factoryIface->Unlock();
    }

    return 0;
}

