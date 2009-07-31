/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include <getopt.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#include <mechanism_controller_test/null_hardware.h>
#include <mechanism_control/mechanism_control.h>
#include <urdf/parser.h>


static struct
{
  char *program_;
  char *xml_;
} g_options;

void controlLoop()
{
  NullHardware hw;
  controller::MechanismControl mc(hw.hw_);


  // Load robot description
  TiXmlDocument xml;
  struct stat st;
  if (0 == stat(g_options.xml_, &st))
  {
    xml.LoadFile(g_options.xml_);
  }
  else
  {
    ROS_INFO("Xml file not found, reading from parameter server\n");
    assert(ros::Node::instance());
    std::string result;
    if (ros::Node::instance()->getParam(g_options.xml_, result))
      xml.Parse(result.c_str());
    else
    {
      ROS_FATAL("Could not load the xml from parameter server: %s\n", g_options.xml_);
      exit(1);
    }
  }
  TiXmlElement *root_element = xml.RootElement();
  TiXmlElement *root = xml.FirstChildElement("robot");
  if (!root || !root_element)
  {
    ROS_FATAL("Could not parse the xml from %s\n", g_options.xml_);
    exit(1);
  }

  //Initialize null hardware interface
  hw.initXml(root);

  //Initialize mechanism control from robot description
  mc.initXml(root);

  //Start running controller updates (and measure update time)
  int count = 0;
  while(1){
    hw.hw_->current_time_ = count / 1.0e-3;
    mc.update();
    if(count % 1000000 == 0)
      printf("%d seconds simulated \n", count / 1000);
    count++;
  }

  //Done - return
}

int main(int argc, char *argv[]){
  ros::init(argc, argv);

  // Parse options
  g_options.program_ = argv[0];
  while (1)
  {
    static struct option long_options[] = {
      {"xml", required_argument, 0, 'x'},
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "x:", long_options, &option_index);
    if (c == -1) break;
    switch (c)
    {
    case 'x':
      g_options.xml_ = optarg;
      break;
    }
  }

  ros::Node *node = new ros::Node("pr2_etherCAT", ros::Node::DONT_HANDLE_SIGINT);

  controlLoop();

  delete node;

  return 0;
}
