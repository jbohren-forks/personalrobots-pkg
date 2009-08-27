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

#include <vector>
#include <string>
#include <sstream>

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

#include "ros/ros.h"
#include <boost/program_options.hpp>

#include <gazebo_tools/urdf2gazebo.h>

void usage(const char *progname)
{
    printf("\nUsage: %s urdf_param_name [options]\n", progname);
    printf("  Note: model name in Gazebo defaults to the robot description parameter name.\n");
    printf("  Example: spawn_urdf --help\n\n");
    printf("  Example: spawn_urdf robot_description_new -x 10\n\n");
}

namespace po = boost::program_options;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"spawn_urdf",ros::init_options::AnonymousName);

    double initial_x = 0;
    double initial_y = 0;
    double initial_z = 0;
    double initial_rx = 0;
    double initial_ry = 0;
    double initial_rz = 0;

    // parse options
    po::options_description v_desc("Allowed options");
    v_desc.add_options()
      ("help,h" , "produce this help message")
      ("model-name,m" , po::value<std::string>() , "name of the model in simulation, defaults to the parameter name of the urdf.")
      ("init-x,x" , po::value<double>() , "set initial x position of model.")
      ("init-y,y" , po::value<double>() , "set initial y position of model.")
      ("init-z,z" , po::value<double>() , "set initial z position of model.")
      ("yaw,w" , po::value<double>() , "set initial yaw (rz) of model.  application orders are r-p-y.")
      ("pitch,p" , po::value<double>() , "set initial pitch (ry) of model.  application orders are r-p-y.")
      ("roll,r" , po::value<double>() , "set initial roll (rx) of model.  application orders are r-p-y.");

    po::options_description h_desc("Hidden options");
    h_desc.add_options()
      ("urdf-param" , po::value< std::vector<std::string> >(), "ROS param name containing URDF as a string.");

    po::options_description desc("Allowed options");
    desc.add(v_desc).add(h_desc);

    po::positional_options_description p_desc;
    p_desc.add("urdf-param", -1);

    po::variables_map vm;
    //po::store(po::parse_command_line(argc, argv, desc), vm);
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p_desc).run(), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
      usage(argv[0]);
      std::cout << v_desc << std::endl;
      exit(1);
    }

    std::string robot_model_name;
    if (vm.count("urdf-param"))
    {
      std::vector<std::string> str_vec = vm["urdf-param"].as<std::vector<std::string> >();
      if (str_vec.size() > 1)
      {
        ROS_ERROR("multiple urdf not supported");
        exit(1);
      }
      else
      {
        ROS_DEBUG("URDF parameter names are: ");
        for (std::vector<std::string>::iterator str = str_vec.begin(); str != str_vec.end(); str++)
        {
          ROS_DEBUG("  %s",str->c_str());
          robot_model_name = *str;
          // get rid of slahses
          std::replace(robot_model_name.begin(),robot_model_name.end(),'/','_');
        }
      }
    }

    if (vm.count("model-name"))
    {
      robot_model_name = vm["model-name"].as<std::string>();
    }
    ROS_DEBUG("model name: %s",robot_model_name.c_str());

    if (vm.count("init-x"))
    {
      initial_x = vm["init-x"].as<double>();
      ROS_DEBUG("x: %f",initial_x);
    }
    if (vm.count("init-y"))
    {
      initial_y = vm["init-y"].as<double>();
      ROS_DEBUG("y: %f",initial_y);
    }
    if (vm.count("init-z"))
    {
      initial_z = vm["init-z"].as<double>();
      ROS_DEBUG("z: %f",initial_z);
    }
    if (vm.count("roll"))
    {
      initial_rx = vm["roll"].as<double>();
      ROS_DEBUG("roll: %f",initial_rx);
    }
    if (vm.count("pitch"))
    {
      initial_ry = vm["pitch"].as<double>();
      ROS_DEBUG("pitch: %f",initial_ry);
    }
    if (vm.count("yaw"))
    {
      initial_rz = vm["yaw"].as<double>();
      ROS_DEBUG("yaw: %f",initial_rz);
    }

    if (argc < 2)
    {
        usage(argv[0]);
        exit(1);
    }

    urdf::Vector3 initial_xyz(initial_x,initial_y,initial_z);
    urdf::Vector3 initial_rpy(initial_rx,initial_ry,initial_rz);

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
        ROS_INFO("spawn_urdf waiting for gazebo factory, usually launched by 'roslaunch `rospack find gazebo`/launch/empty_world.launch'");
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
    std::string urdf_param_name = std::string(argv[1]);
    std::string full_urdf_param_name;
    rosnode.searchParam(urdf_param_name,full_urdf_param_name);
    std::string xml_content;
    rosnode.getParam(full_urdf_param_name.c_str(),xml_content);
    ROS_DEBUG("%s content\n%s\n", full_urdf_param_name.c_str(), xml_content.c_str());

    // set flag to enforce joint limits, this is hardcoded to true because we do want a model with
    // joint limits enforced.
    bool enforce_limits = true;
    //
    // init a parser library
    //
    urdf2gazebo::URDF2Gazebo u2g(robot_model_name);
    // do the number crunching to make gazebo.model file
    TiXmlDocument urdf_in, xml_out;
    urdf_in.Parse(xml_content.c_str());
    u2g.convert(urdf_in, xml_out, enforce_limits, initial_xyz, initial_rpy);

    // copy model to a string
    std::ostringstream stream;
    stream << xml_out;
    std::string xml_string = stream.str();
    ROS_DEBUG("Gazebo XML\n\n%s\n\n ",xml_string.c_str());

    bool writing_iface = true;
    while (writing_iface)
    {
      factoryIface->Lock(1);
      if (strcmp((char*)factoryIface->data->newModel,"")==0)
      {
        ROS_INFO("Creating Robot Model Name:%s in Gazebo\n",robot_model_name.c_str());
        // don't overwrite data, only write if iface data is empty
        strcpy((char*)factoryIface->data->newModel, xml_string.c_str());
        writing_iface = false;
      }
      factoryIface->Unlock();
    }

    return 0;
}

