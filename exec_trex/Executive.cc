/*
 * wavefront_player
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

@b exec_trex is a bare bones demonstration of using the TREX hybrid executive
for planning and control. It uses the EUROPA-2 constraint-based temporal planning library
to represent plan state, do planning, write models etc. Right now this is
a crude integration which will evolve to include more generalized integration patterns to
ROS nodes. Also, the build structure uses jam, and that allows builds for a variety
of versions. For now we will use the default development build.

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

#include "Nddl.hh"
#include "Executive.hh"
#include "RCSAdapter.hh"
#include "Token.hh"
#include <signal.h>
#include "WavefrontPlanner.hh"
#include <std_srvs/StaticMap.h>

#include <std_msgs/BaseVel.h>

TiXmlElement* root = NULL;
Clock* agentClock = NULL;
std::ofstream dbgFile("Debug.log");

using namespace std_msgs;




namespace TREX {


  /**
   * @brief Singleton accessor
   */
  ExecutiveId Executive::instance(){
    return s_id;
  }

  Executive::Executive() : m_id(this) {
  }
  Executive::~Executive(){
    m_id.remove();
    s_id = ExecutiveId::noId();
  }
  bool Executive::isInitialized() const{
    return  m_initialized;
  }
};


/**
 * @brief Handle cleanup on process termination signals.
 */
void signalHandler(int signalNo){
  debugMsg("Executive:signal", "Handling signal...");
  exit(0);
}

/**
 * @brief Handle cleanup on exit
 */
void cleanup(){
    debugMsg("Executive", "Cleaning up...");

  // Terminate the agent
  Agent::terminate();

  Clock::sleep(3);

  Agent::cleanupLog();
}

ExecutiveId Executive::s_id;

int main(int argc, char **argv)
{
  //signal(SIGINT,  &signalHandler);
  //signal(SIGTERM, &signalHandler);
  //signal(SIGQUIT, &signalHandler);
  //signal(SIGKILL, &signalHandler);
  //atexit(&cleanup);

  if (argc != 2) {
    std::cerr << "Invalid argument list: Usage: ros_exec configFile" << std::endl;
    return -1;
  }

  
  LogManager::instance();

  initROSExecutive();

  Executive t;

  NDDL::loadSchema();

  char * configFile = argv[1];
  root = LogManager::initXml( configFile );
  DebugMessage::setStream(dbgFile);

  // Allocate a real time clock with 1 second per tick
  agentClock = new RealTimeClock(0.25);

  // Allocate the agent
  debugMsg("Executive", "Initializing the agent");
  Agent::initialize(*root, *agentClock);


  debugMsg("Executive", "Starting TREX");
  try{
    debugMsg("ALWAYS", "Executing the agent");
    Agent::instance()->run();
  }
  catch(void*){
    debugMsg("Executive", "Caught unexpected exception.");
  }

  
  //delete WavefrontPlanner::Instance();

  return 0;
}

