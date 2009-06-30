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
#include "trex_ros/components.h"
#include "trex_ros/logger.h"
#include "Agent.hh"
#include "trex_ros/logclock.h"
#include "Debug.hh"
#include "Utilities.hh"
#include <signal.h>
#include <unistd.h>
#include <signal.h>

#include "trex_ros/executive.h"



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
  ExecutiveId Executive::request(bool playback, bool hyper){
    if(s_id == ExecutiveId::noId()){
      new Executive(playback, hyper);
    }
    return s_id;
  }

  /**
   * @brief Executive constructor sets up the trex agent instance
   */
  Executive::Executive(bool playback, bool hyper)
    : m_id(this), watchDogCycleTime_(0.1), agent_clock_(NULL), debug_file_("Debug.log"), input_xml_root_(NULL), playback_(playback), hyper_(hyper)
  {
    s_id = m_id;
    m_refCount = 0;

    // ROS Parameters for running the agent
    ros::NodeHandle node_handle;

    double ping_frequency(1.0);
    double update_rate(1.0);
    std::string input_file = "";
    std::string path = "";
    int time_limit(0);
    std::string start_dir = "";
    std::string log_dir = "";
    double broadcast_plan_rate(0.0);
    node_handle.param("/trex/ping_frequency", ping_frequency, ping_frequency);
    node_handle.param("/trex/update_rate", update_rate, update_rate);
    node_handle.param("/trex/input_file", input_file, input_file);
    node_handle.param("/trex/path", path, path);
    node_handle.param("/trex/time_limit", time_limit, time_limit);
    node_handle.param("/trex/start_dir", start_dir, start_dir);
    node_handle.param("/trex/log_dir", log_dir, log_dir);
    node_handle.param("/trex/broadcast_plan_rate", broadcast_plan_rate, broadcast_plan_rate);

    checkError(log_dir != "", "You must set /trex/log_dir");
    checkError(start_dir != "", "You must set /trex/start_dir");
    checkError(input_file != "", "You must set /trex/input_file");

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
    std::string debug_config_file = TREX::findFile("debug.cfg");
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
      // Run the agent off of a clock that reads from the log file "clock.log"
      agent_clock_ = new TREX::PlaybackClock(true);
    } else if (hyper_) {
      // Run the agent off of a clock that runs as fast as possible
      agent_clock_ = new TREX::PseudoClock(0, 50, true);
    } else {
      // Allocate a real time clock based on input of update_rate, and write out to the log file "clock.log"
      const double tick_duration = (update_rate <= 0 ? 1.0 : 1.0 / update_rate);
      agent_clock_ = new TREX::LogClock(tick_duration);
    }

    // Allocate the agent
    TREX::Agent::initialize(*input_xml_root_, *agent_clock_, time_limit);

    // Set up  watchdog thread message generation
    new boost::thread(boost::bind(&Executive::watchDogLoop, this));

    ROS_INFO("Executive created.\n");
  }


  Executive::~Executive() {
    ROS_INFO("Shutting down at tick(%d)\n", TREX::Agent::instance()->getCurrentTick());

    // Terminate the agent
    ROS_INFO("Terminating agent...\n");
    TREX::Agent::terminate();
    TREX::Clock::sleep(5);
    ROS_INFO("Cleaning up log...\n");
    TREX::Agent::cleanupLog();
    ROS_INFO("Resetting agent...\n");
    TREX::Agent::reset(); //FIXME: This does not work.
    ROS_INFO("Destructing Executive...\n");


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
   * @brief Kicks off all the work to do.
   */
  void Executive::run(){
    ROS_INFO("Running the executive in nonstop mode.\n");
    TREX::Agent::instance()->run();
    ROS_INFO("Agent has finished running.\n");
  }


  /**
   * @brief Initializes interactive mode. This is some work usually done by Agent::run()
   */
  void Executive::interactiveInit() {
    // Start the clock
    agent_clock_->doStart();

    // Initialize agent logging
    TREX::LogManager::instance().handleInit();

    ROS_INFO("Running the executive in interactive mode.\n");
  }

  /**
   * @brief Steps the executive agent by one step
   */
  bool Executive::step(){
    return TREX::Agent::instance()->doNext();
  }


  void Executive::reset(){
    checkError(s_id.isNoId() || s_id.isValid(), "Bad Executive Id.");

    if(s_id.isId()) {
      delete (Executive*) s_id;
    }

    s_id = ExecutiveId::noId();
  }

  void Executive::watchDogLoop(){
    ros::NodeHandle node_handle;
    ros::Publisher pub = node_handle.advertise<std_msgs::Empty>("trex/ping", 1);

    std_msgs::Empty pingMsg;
    while(!Agent::terminated()){
      pub.publish(pingMsg);
      usleep((unsigned int) rint(watchDogCycleTime_ * 1e6));
    }
  }
}

