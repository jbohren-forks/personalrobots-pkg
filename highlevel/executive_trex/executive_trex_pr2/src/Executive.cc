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

#include <ros/console.h>

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
#include <std_msgs/Empty.h>
#include <sys/time.h>
#include <boost/thread.hpp>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

TREX::ExecutiveId node;

namespace TREX {

  ExecutiveId Executive::s_id;

  /**
   * Executive class implementation.
   */


  /**
   * @brief Singleton accessor
   */
  ExecutiveId Executive::request(bool playback, bool warp){
    if(s_id == ExecutiveId::noId()){
      new Executive(playback, warp);
    }
    return s_id;
  }

  /**
   * @brief Executive constructor sets up the trex agent instance
   */
  Executive::Executive(bool playback, bool warp)
    : m_id(this), watchDogCycleTime_(0.1), agent_clock_(NULL), debug_file_("Debug.log"), input_xml_root_(NULL), playback_(playback), warp_(warp)
  {
    s_id = m_id;
    m_refCount = 0;

    // ROS Parameters for running the agent
    double ping_frequency(1.0);
    std::string input_file;
    std::string path;
    int time_limit(0);
    std::string start_dir;
    std::string log_dir;
    ros::Node::instance()->param("/trex/ping_frequency", ping_frequency, ping_frequency);
    ros::Node::instance()->param("/trex/input_file", input_file, input_file);
    ros::Node::instance()->param("/trex/path", path, path);
    ros::Node::instance()->param("/trex/time_limit", time_limit, time_limit);
    ros::Node::instance()->param("/trex/start_dir", start_dir, start_dir);
    ros::Node::instance()->param("/trex/log_dir", log_dir, log_dir);

    // Bind the watchdog loop sleep time from input controller frequency
    if(ping_frequency > 0)
      watchDogCycleTime_ = 1 / ping_frequency;

    // Bind TREX environment variables
    setenv("TREX_PATH", path.c_str(), 1);
    setenv("TREX_START_DIR", start_dir.c_str(), 1);
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

    // Read debug configuration information if available
    std::string debug_config_file = TREX::findFile("Debug.cfg");
    std::ifstream debug_config_stream(debug_config_file.c_str());
    if(debug_config_stream.good())
      DebugMessage::readConfigFile(debug_config_stream);

    // Redirect debug output
    DebugMessage::setStream(debug_file_);

    // This initialization looks after registration of factories required for TREX plug-ins
    TREX::initROSExecutive(playback_);

    // Set up the clock
    int finalTick = 1;
    input_xml_root_->Attribute("finalTick", &finalTick);
    if (playback_) {
      agent_clock_ = new TREX::PlaybackClock((time_limit == 0 ? finalTick : time_limit), warp, input_xml_root_);
    } else {
      // Allocate a real time clock with 1 second per tick
      agent_clock_ = new TREX::LogClock(1.0);
    }

    // Allocate the agent
    TREX::Agent::initialize(*input_xml_root_, *agent_clock_, time_limit);

    // Set up  watchdog thread message generation
    ros::Node::instance()->ros::Node::advertise<std_msgs::Empty>("trex/ping", 1);
    new boost::thread(boost::bind(&Executive::watchDogLoop, this));

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
	if (!TREX::Agent::instance()->doNext()) {
	  std::cout << "Agent has completed its mission." << std::endl;
	  ((TREX::PlaybackClock*)agent_clock_)->consolePopup();
	}
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
    std_msgs::Empty pingMsg;
    while(!Agent::terminated()){
      ros::Node::instance()->publish("trex/ping", pingMsg);
      usleep((unsigned int) rint(watchDogCycleTime_ * 1e6));
    }
  }
}

