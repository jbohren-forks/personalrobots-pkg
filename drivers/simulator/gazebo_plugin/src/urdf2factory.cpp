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

#include "ros/node.h"

#include <urdf/URDF.h>

#include <gazebo_plugin/urdf2gazebo.h>

using namespace urdf2gazebo;

void usage(const char *progname)
{
    printf("\nUsage: %s param_name [initial x y z roll pitch yaw]\n", progname);
    printf("  e.g. read robotdesc/pr2 from param server and send to gazebo factory to spawn robot\n\n");
}

int main(int argc, char **argv)
{
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
    if (argc == 8)
    {
        initial_rx = atof(argv[5]);
        initial_ry = atof(argv[6]);
        initial_rz = atof(argv[7]);
    }

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
        std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
        usleep(1000000);
        connected_to_server = false;
      }
    }

    /// Open the Factory interface
    try
    {
      factoryIface->Open(client, "factory_iface");
    }
    catch (gazebo::GazeboError e)
    {
      std::cout << "Gazebo error: Unable to connect to the factory interface\n"
      << e << "\n";
      return -1;
    }


    // Load parameter server string for pr2 robot description
    ros::init(argc,argv);
    ros::Node* rosnode = new ros::Node(argv[1],ros::Node::DONT_HANDLE_SIGINT);
    printf("-------------------- starting node for pr2 param server factory \n");
    std::string xml_content;
    rosnode->getParam(argv[1],xml_content);

    // Parse URDF from param server
    bool enforce_limits = true;
    robot_desc::URDF wgxml;
    if (!wgxml.loadString(xml_content.c_str()))
    {
        printf("Unable to load robot model from param server robotdesc/pr2\n");  
        exit(2);
    }

    //
    // init a parser library
    //
    URDF2Gazebo u2g;
    // do the number crunching to make gazebo.model file
    TiXmlDocument doc;
    u2g.convert(wgxml, doc, enforce_limits);


    //std::cout << " doc " << doc << std::endl << std::endl;

    // copy model to a string
    std::ostringstream stream;
    stream << doc;
    std::string xml_string = stream.str();

    // strip <? ... xml version="1.0" ... ?> from xml_string
    std::string open_bracket("<?");
    std::string close_bracket("?>");
    int pos1 = xml_string.find(open_bracket,0);
    int pos2 = xml_string.find(close_bracket,0);
    xml_string.replace(pos1,pos2-pos1+2,std::string(""));

    // replace initial pose of robot
    // find first instance of xyz and rpy, replace with initial pose
    if (argc >= 5)
    {
      std::ostringstream xyz_stream, rpy_stream;
      xyz_stream << "<xyz>" << initial_x << " " << initial_y << " " << initial_z << "</xyz>";
      int xyz_pos1 = xml_string.find("<xyz>" ,0);
      int xyz_pos2 = xml_string.find("</xyz>",0);
      if (xyz_pos1 != -1 && xyz_pos2 != -1)
        xml_string.replace(xyz_pos1,xyz_pos2-xyz_pos1+6,std::string(xyz_stream.str()));
      if (argc == 8)
      {
        rpy_stream << "<rpy>" << initial_rx << " " << initial_ry << " " << initial_rz << "</rpy>";
        int rpy_pos1 = xml_string.find("<rpy>" ,0);
        int rpy_pos2 = xml_string.find("</rpy>",0);
        if (rpy_pos1 != -1 && rpy_pos2 != -1)
          xml_string.replace(rpy_pos1,rpy_pos2-rpy_pos1+6,std::string(rpy_stream.str()));
      }
    }

    //std::cout << " ------------------- xml ------------------- " << std::endl;
    //std::cout << xml_string << std::endl;
    //std::cout << " ------------------- xml ------------------- " << std::endl;

    factoryIface->Lock(1);
    printf("Creating Robot in Gazebo\n");
    strcpy((char*)factoryIface->data->newModel, xml_string.c_str());
    factoryIface->Unlock();
    usleep(1000000);

    rosnode->shutdown();

    return 0;
}

