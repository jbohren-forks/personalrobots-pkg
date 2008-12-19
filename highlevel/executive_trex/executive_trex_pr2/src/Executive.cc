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
#include <rosconsole/rosconsole.h>

#include "TestMonitor.hh"
#include "Nddl.hh"
#include "Components.hh"
#include "Logger.hh"
#include "Agent.hh"
#include "LogClock.hh"
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
#include <highlevel_controllers/Ping.h>
#include <sys/time.h>
#include <rosthread/member_thread.h>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


TREX::ExecutiveId node;

namespace TREX {

  ExecutiveId Executive::s_id;

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

  /**
   * Executive class implementation.
   */


  /**
   * @brief Singleton accessor
   */
  ExecutiveId Executive::request(){
    if(s_id == ExecutiveId::noId()){
      new Executive();
    }
    return s_id;
  }

  /**
   * @brief Executive constructor sets up the trex agent instance
   */
  Executive::Executive() 
    : ros::node("trex"), m_id(this), watchDogCycleTime_(0.1), agent_clock_(NULL), debug_file_("Debug.log"), input_xml_root_(NULL), playback_(isArg(argc, argv, "--playback"))
  {
    s_id = m_id; 
    m_refCount = 0;
  
    // ROS Parameters for running the agent
    double ping_frequency(1.0);
    std::string input_file;
    std::string path;
    int time_limit(0);
    std::string log_dir;
    param("/trex/ping_frequency", ping_frequency, ping_frequency);
    param("/trex/input_file", input_file, input_file);
    param("/trex/path", path, path);
    param("/trex/time_limit", time_limit, time_limit);
    param("/trex/log_dir", log_dir, log_dir);

    // Bind the watchdog loop sleep time from input controller frequency
    if(ping_frequency > 0)
      watchDogCycleTime_ = 1 / ping_frequency;

    // Bind TREX environment variables
    setenv("TREX_PATH", path.c_str(), 1);
    setenv("TREX_LOG_DIR", log_dir.c_str(), 1);


    // When not running in playback mode, we enable generation of a log file
    if (!playback_) {
      logger_ = TREX::Logger::request();
      logger_->setEnabled(true);
    }

    // When running in playback mode, we use the playback version of the given input file
    if(playback_){
      input_file = "playback." + input_file;
    }

    // Read input XML configuration data
    input_xml_root_ = TREX::LogManager::initXml( TREX::findFile(input_file) );

    // This initialization looks after registration of factories required for TREX plug-ins
    TREX::initROSExecutive(playback_);

    // Redirect debug output
    DebugMessage::setStream(debug_file_);

    // Set up the clock
    int finalTick = 1;
    input_xml_root_->Attribute("finalTick", &finalTick);
    if (playback_) {
      agent_clock_ = new TREX::PlaybackClock((time_limit == 0 ? finalTick : time_limit), input_xml_root_);
    } else {
      // Allocate a real time clock with 1 second per tick
      agent_clock_ = new TREX::LogClock(1.0);
    }

    // Allocate the agent
    TREX::Agent::initialize(*input_xml_root_, *agent_clock_, time_limit);

    // Set up  watchdog thread message generation
    ros::node::advertise<highlevel_controllers::Ping>("trex/ping", 1);
    ros::thread::member_thread::startMemberFunctionThread(this, &Executive::watchDogLoop); 

    ROS_INFO("Executive created.\n");
  }


  Executive::~Executive() {
    ROS_INFO("Shutting down at tick(%d)\n", TREX::Agent::instance()->getCurrentTick());

    // Terminate the agent
    TREX::Agent::terminate();
    TREX::Clock::sleep(3);
    TREX::Agent::reset();

    TREX::Agent::cleanupLog();

    // Deallocate clock
    if(agent_clock_ != NULL)
      delete agent_clock_;

    // Deallocate logger, with a clean shutdown
    if (logger_ != TREX::LoggerId::noId()) {
      FILE* file = logger_->getFile();
      if (file) { fprintf(file, "\n</log>\n"); }
      logger_->release();
    }

    m_id.remove();
  }

  /**
   * @broef Kicks off all the work to do.
   */
  void Executive::run(){
    if (playback_) {
      ROS_INFO("Stepping the executive.\n");
      agent_clock_->doStart();    
      TREX::LogManager::instance().handleInit();
      while (!((TREX::PlaybackClock*)agent_clock_)->isTimedOut()) {
	if (((TREX::PlaybackClock*)agent_clock_)->isAtGoalTick()) {
	  ((TREX::PlaybackClock*)agent_clock_)->consolePopup();
	}
	TREX::Agent::instance()->doNext();
      }
    } else {
      try{
	ROS_INFO("Running the executive.\n");
	TREX::Agent::instance()->run();
      }
      catch(const char * str){
	std::cout << str << std::endl;
      }
    }

    ROS_INFO("Agent has finished running.\n");
  }

  void Executive::watchDogLoop(){
    highlevel_controllers::Ping pingMsg;
    while(!Agent::terminated()){
      publish<highlevel_controllers::Ping>("trex/ping", pingMsg);
      usleep((unsigned int) rint(watchDogCycleTime_ * 1e6));
    }
  }

}

/**
 * @brief Handle cleanup on exit
 */
void cleanup(){
  if(node.isId()){
    node->shutdown();
    delete (ros::node*) node;
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

  if(TREX::isArg(argc, argv, "--help")){
    std::cout << "\n";
    std::cout << "Welcome! TREX is an executive for supervisory  control of an autonomous system. TREX requires the following ROS parameters:\n";
    std::cout << "* trex/input_file: An xml file that defines the agent control configuration.\n";
    std::cout << "* trex/path:       A search path for locating input files. This should include a location for\n";
    std::cout << "                   the input configuration file, as well as locations for agent initialization files (nddl files)\n";
    std::cout << "* trex/log_dir:    An output directory for TREX log files.\n";
    std::cout << "\n";
    std::cout << "Usage: trexfast  [--help | --playback]\n";
    std::cout << "--help:            Provides this message!\n";
    std::cout << "--playback:        Use if debugging a previous run. Expects an xml observation log file as input named <your_agent_name>.log\n";
    std::cout << "                   and a clock log file names clock.log.\n";
    return 0;
  }

  int success = 0;

  try{
    node = TREX::Executive::request();
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

  node->shutdown();
  delete (ros::node*) node;
  node = TREX::ExecutiveId::noId();


  // Parse command line arguments to see if we must apply test case validation
  if(TREX::isArg(argc, argv, "--gtest")){
    testing::InitGoogleTest(&argc, argv);
    success = RUN_ALL_TESTS();
  }

  return success;
}

