/*
 * TREX Process
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

/**

@mainpage

@htmlinclude manifest.html

@b exec_trex is a demonstration of using the TREX hybrid executive
for planning and control. It uses the EUROPA-2 constraint-based temporal planning library
to represent plan state, do planning, write models etc. Right now this is
a crude integration which will evolve to include more generalized integration patterns to
ROS nodes.

<hr>

@section usage Usage
@verbatim
$ exec_trex_g_rt cfgFile
@endverbatim

@param cfgFile Is the TREX configuration file which defines initial inputs and goals.

@todo

@par Example

@verbatim
$ exec_trex_g_rt exec.cfg
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "state"/Planner2DState : current planner state (e.g., goal reached, no
path)

Publishes to (name/type):
- @b "goal"/Planner2DGoal : goal for the robot.

@todo

<hr>

@section parameters ROS parameters

- None

 **/

// For integration of testing. Ros console needs to be defined early to avoid conflict in warning
// declaration somewhere in the include tree.
#include <gtest/gtest.h>
#include <ros/console.h>

#include "TestMonitor.hh"
#include "Nddl.hh"
#include "executive_trex_pr2/components.hh"
#include "Logger.hh"
#include "Agent.hh"
#include "executive_trex_pr2/logclock.hh"
#include "Debug.hh"
#include "Utilities.hh"
#include <signal.h>
#include <unistd.h>
#include <signal.h>

#include "Executive.hh"



//NDDL includes
#include "Nddl.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Utilities.hh"

// Requirements for watchdog
#include <sys/time.h>
#include <boost/thread.hpp>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

bool g_playback = false;

TREX::ExecutiveId node;
ros::Node* g_ros_node;

namespace executive_trex_pr2 {

  /**
   * @brief Utilty to check for an argument in an arg list
   */
  bool isArg(int argc, char** argv, const char* argName){
    for(int i=0;i<argc;i++){
      std::string toCheck(argv[i]);
      if(toCheck.find(argName) == 0)
	return true;
    }

    return false;
  }


}

/**
 * @brief Handle cleanup on exit
 */
void cleanup(){
  if(node.isId()){
    g_ros_node->shutdown();
    delete (ros::Node*) g_ros_node;
    delete (TREX::Executive*) node;
    node = TREX::ExecutiveId::noId();
  }
  exit(0);
}

/**
 * Test for validating expected output. This will have to evolve
 */
TEST(trex, validateOutput){
  bool success = TREX::TestMonitor::success();
  if(!success){
    ROS_ERROR("\n%s", TREX::TestMonitor::toString().c_str());
  }

  ASSERT_EQ(success, true);
}

int main(int argc, char **argv)
{
  signal(SIGINT,  &TREX::signalHandler);
  signal(SIGTERM, &TREX::signalHandler);
  signal(SIGQUIT, &TREX::signalHandler);
  signal(SIGKILL, &TREX::signalHandler);

  ros::init(argc, argv);
  g_ros_node = new ros::Node("trex");

  if(executive_trex_pr2::isArg(argc, argv, "--help")){
    std::cout << "\n";
    std::cout << "Welcome! TREX is an executive for supervisory  control of an autonomous system. TREX requires the following ROS parameters:\n";
    std::cout << "* trex/input_file: An xml file that defines the agent control configuration.\n";
    std::cout << "* trex/path:       A search path for locating input files. This should include a location for\n";
    std::cout << "                   the input configuration file, as well as locations for agent initialization files (nddl files)\n";
    std::cout << "* trex/start_dir   A directory from which to start the search for input files";
    std::cout << "* trex/log_dir:    An output directory for TREX log files.\n";
    std::cout << "\n";
    std::cout << "Usage: trexfast  [--help | --playback]\n";
    std::cout << "--help:            Provides this message!\n";
    std::cout << "--playback:        Use if debugging a previous run. Expects an xml observation log file as input named <your_agent_name>.log\n";
    std::cout << "                   and a clock log file names clock.log.\n";
    std::cout << "--warp:            Use unlimited steps per tick in playback, ignore clock.log.\n";
    return 0;
  }

  int success = 0;

  bool playback = executive_trex_pr2::isArg(argc, argv, "--playback");
  bool warp = executive_trex_pr2::isArg(argc, argv, "--warp");

  try{
    node = TREX::Executive::request(playback, warp);
    node->run();
  }
  catch(char* e){
    ROS_INFO("Caught %s. Shutting down.\n", e);
    success = -1;
  }
  catch(...){
    ROS_INFO("Caught Unknown Exception!. Shutting down.\n");
    success = -1;
  }

  g_ros_node->shutdown();
  delete (ros::Node*) g_ros_node;
  delete (TREX::Executive*) node;
  node = TREX::ExecutiveId::noId();


  // Parse command line arguments to see if we must apply test case validation
  if(executive_trex_pr2::isArg(argc, argv, "--gtest")){
    testing::InitGoogleTest(&argc, argv);
    success = RUN_ALL_TESTS();
  }

  return success;
}

