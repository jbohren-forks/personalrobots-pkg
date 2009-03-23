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

/*
 * Author: Rob Wheeler
 */

#include "robot_mechanism_controllers/dynamic_loader_controller.h"
#include "robot_srvs/SpawnController.h"
#include "robot_srvs/KillController.h"
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/algorithm/string/trim.hpp>

namespace controller {

ROS_REGISTER_CONTROLLER(DynamicLoaderController)

DynamicLoaderController::DynamicLoaderController() : handle_(0)
{
}

DynamicLoaderController::~DynamicLoaderController()
{
  // Shutdown controller cleanly
  boost::thread killThread(boost::bind(DynamicLoaderController::unloadLibrary, names_, handle_));
}

void DynamicLoaderController::unloadLibrary(std::vector<std::string> names, lt_dlhandle handle)
{
  robot_srvs::KillController::Request req;
  robot_srvs::KillController::Response resp;

  BOOST_FOREACH(std::string &name, names) {
    req.name = name;

    ros::service::call("kill_controller", req, resp);
  }

  lt_dlclose(handle);
}

void DynamicLoaderController::loadLibrary(std::string &xml)
{
  robot_srvs::SpawnController::Request req;
  robot_srvs::SpawnController::Response resp;

  req.xml_config = xml;

  ros::service::call("spawn_controller", req, resp);

  if (resp.ok[0]) {
    names_ = resp.name;
  }
}

bool DynamicLoaderController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  std::string package = config->Attribute("package") ? config->Attribute("package") : "";
  if (package == "") {
    ROS_ERROR("No package given to DynamicLoaderController");
    return false;
  }

  std::string lib = config->Attribute("lib") ? config->Attribute("lib") : "";
  if (lib == "") {
    ROS_ERROR("No lib given to DynamicLoaderController");
    return false;
  }

  std::string command = "rospack find " + package;
  std::string package_dir;
  char buffer[256];
  FILE *pipe = popen(command.c_str(), "r");
  if (pipe == NULL) {
    ROS_ERROR("Unable to run command: %s", command.c_str());
    return false;
  }
  while (fgets(buffer, sizeof(buffer), pipe) != NULL)
    package_dir.append(buffer);
  pclose(pipe);
  
  int errors = lt_dlinit();
  if (errors) {
    ROS_ERROR("Unable to initialize dynamic loader (%s)", lt_dlerror());
    return false;
  }

  std::stringstream path;
  path << boost::algorithm::trim_copy(package_dir) << "/lib/" << lib;

  if ((handle_ = lt_dlopenext(path.str().c_str())) == NULL) {
    ROS_ERROR("Unable to dlopen %s: %s", path.str().c_str(), lt_dlerror());
    return false;
  }
  
  TiXmlElement *controller = config->FirstChildElement();

  if (controller) {
    std::stringstream str;
    str << *controller;
    boost::thread spawnThread(boost::bind(&DynamicLoaderController::loadLibrary, this, str.str()));
  }

  return true;
}

void DynamicLoaderController::update()
{
}

}
