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
    printf("\nUsage: %s xml_param_name [initial x y z roll pitch yaw gazebo_model_name]\n", progname);
    printf("  For example: xml2factory robot_description 0 0 1 0 0 90 pr3_model\n\n");
    printf("  Note: gazebo_model_name defaults to the robot description parameter.\n\n");
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"xml2factory",ros::init_options::AnonymousName);

    if (argc < 2)
    {
        usage(argv[0]);
        exit(1);
    }

    double initial_x = 0;
    double initial_y = 0;
    double initial_z = 0;
    if (argc >= 5)
    {
        initial_x = atof(argv[2]);
        initial_y = atof(argv[3]);
        initial_z = atof(argv[4]);
    }
    double initial_rx = 0;
    double initial_ry = 0;
    double initial_rz = 0;
    if (argc >= 8)
    {
        initial_rx = atof(argv[5]);
        initial_ry = atof(argv[6]);
        initial_rz = atof(argv[7]);
    }

    std::string robot_model_name(argv[1]);
    // make sure this is not the ros-generated commandline log filename
    if (argc >= 9)
    {
        robot_model_name = std::string(argv[8]);
    }
    // get rid of slahses
    std::replace(robot_model_name.begin(),robot_model_name.end(),'/','_');


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
        ROS_INFO("xml2factory waiting for gazebo factory, usually launched by 'roslaunch `rospack find gazebo`/launch/empty_world.launch'");
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

    // Load parameter server string for pr2 robot description
    ros::NodeHandle rosnode;
    ROS_INFO("-------------------- starting node for pr2 param server factory \n");

    std::string xml_param_name = std::string(argv[1]);
    std::string full_xml_param_name;
    rosnode.searchParam(xml_param_name,full_xml_param_name);
    std::string xml_content;
    rosnode.getParam(full_xml_param_name.c_str(),xml_content);
    ROS_DEBUG("%s content\n%s\n", full_xml_param_name.c_str(), xml_content.c_str());

    if (xml_content.c_str()==NULL)
    {
        ROS_ERROR("Unable to load robot model from param server robot_description\n");  
        exit(2);
    }

    // strip <? ... xml version="1.0" ... ?> from xml_content
    std::string open_bracket("<?");
    std::string close_bracket("?>");
    int pos1 = xml_content.find(open_bracket,0);
    int pos2 = xml_content.find(close_bracket,0);
    xml_content.replace(pos1,pos2-pos1+2,std::string(""));

    // replace initial pose of robot
    // find first instance of xyz and rpy, replace with initial pose
    if (argc >= 5)
    {
      std::ostringstream xyz_stream, rpy_stream;
      xyz_stream << "<xyz>" << initial_x << " " << initial_y << " " << initial_z << "</xyz>";
      int xyz_pos1 = xml_content.find("<xyz>" ,0);
      int xyz_pos2 = xml_content.find("</xyz>",0);
      if (xyz_pos1 != -1 && xyz_pos2 != -1)
        xml_content.replace(xyz_pos1,xyz_pos2-xyz_pos1+6,std::string(xyz_stream.str()));
      if (argc == 8)
      {
        rpy_stream << "<rpy>" << initial_rx << " " << initial_ry << " " << initial_rz << "</rpy>";
        int rpy_pos1 = xml_content.find("<rpy>" ,0);
        int rpy_pos2 = xml_content.find("</rpy>",0);
        if (rpy_pos1 != -1 && rpy_pos2 != -1)
          xml_content.replace(rpy_pos1,rpy_pos2-rpy_pos1+6,std::string(rpy_stream.str()));
      }
    }

    bool writing_iface = true;
    while (writing_iface)
    {
      factoryIface->Lock(1);
      if (strcmp((char*)factoryIface->data->newModel,"")==0)
      {
        ROS_INFO("Creating Robot Model Name:%s in Gazebo\n",robot_model_name.c_str());
        // don't overwrite data, only write if iface data is empty
        strcpy((char*)factoryIface->data->newModel, xml_content.c_str());
        writing_iface = false;
      }
      factoryIface->Unlock();
    }

    return 0;
}

