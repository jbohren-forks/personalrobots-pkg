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
#include "executive_trex_pr2/components.h"
#include "executive_trex_pr2/logger.h"
#include "Agent.hh"
#include "executive_trex_pr2/logclock.h"
#include "Debug.hh"
#include "Utilities.hh"
#include <signal.h>
#include <unistd.h>
#include <signal.h>

#include "executive_trex_pr2/executive.h"



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

TREX::ExecutiveId executive;

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

  // Run parameters
  bool g_playback,
       g_hyper,
       g_console;
  
  // Tick position management
  TREX::TICK consoleStopTick,
    currentTick;

  /**
   * @brief Live documentation for interactive mode
   */
  void printConsoleHelp() {
    std::cout << "Interactive mode." << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << " Q :- Quit" << std::endl;
    std::cout << " N :- Next" << std::endl;
    std::cout << " G :- Goto <tick> e.g. g100" << std::endl;
    std::cout << " R :- Reload Debug.cfg" << std::endl;
    std::cout << " + :- Enable pattern e.g. '+Agent'" << std::endl;
    std::cout << " - :- Disable pattern e.g. '-Agent'" << std::endl;
    std::cout << " ! :- Disable all debug messages" << std::endl;
    std::cout << " PW :- Start PlanWorks output" << std::endl;
    std::cout << " EPW :- End PlanWorks output" << std::endl;
    std::cout << " H :- Help" << std::endl;
  }

  
  void printConsolePopup() {
    std::string cmdString, cmdStringRaw;
    bool cmdValid = false;

    std::cout << "At tick " << currentTick << ", H for help." << std::endl;
    while (!cmdValid) {
      std::cout << "> ";
      getline(std::cin, cmdStringRaw);
      cmdString = cmdStringRaw;
      std::transform(cmdString.begin(), cmdString.end(), cmdString.begin(), toupper);
      const char cmd = cmdString.length() == 0 ? 0 : cmdString.at(0);
      if (!cmd) {
	cmdValid = false;
      } else if(cmd == 'H' || cmd == 'h'){
	// Print help message
	printConsoleHelp();
	cmdValid = false;
      } else if(cmd == 'Q' || cmd == 'q'){
	// Quit
	std::cout << "Goodbye :-)" << std::endl;
	cmdValid = true;
	exit(0);
      } else if(cmd == 'N' || cmd == 'n'){
	// Step to the next tick
	consoleStopTick = currentTick + 1;
	cmdValid = true;
      } else if(cmd == 'G' || cmd == 'g'){
	// Go to an absolute tick step
	
	std::string tickStr = cmdString.substr(1);

	if (atoi(tickStr.c_str()) > 0) {
	  // Set which tick to step to
	  consoleStopTick = (TREX::TICK)atoi(tickStr.c_str());
	  cmdValid = true;
	  
	  // Check if the desired tick is in the past
	  if (consoleStopTick < currentTick) {
	    cmdValid = true;
	    std::cout << "Doing a rewind." << std::endl;

	    // Reset the executive
	    executive->reset();

	    // Re-initialize the executive with the initial arguments
	    executive = TREX::Executive::request(g_playback, g_hyper);
	  } else if (currentTick == consoleStopTick) {
	    cmdValid = false;
	    std::cout << "Already at that tick." << std::endl;
	  }
	} else {
	  // No number entered
	  std::cout << "Please enter a number." << std::endl;
	  cmdValid = false;
	}
      } else if(cmd == 'R' || cmd == 'r'){
	// Reload the debug configuration
	std::cout << "Reading debug.cfg." << std::endl;
	std::ifstream config(TREX::findFile("Debug.cfg").c_str());
	DebugMessage::readConfigFile(config);
	cmdValid = false;
      } else if(cmdString == "PW"){
	std::cout << "Enable plan works." << std::endl;
	DebugMessage::enableMatchingMsgs("", "PlanWorks");
      } else if(cmdString == "EPW"){
	std::cout << "Disable plan works." << std::endl;
	DebugMessage::disableMatchingMsgs("", "PlanWorks");
      } else if(cmd == '+'){
	std::string pattern = cmdStringRaw.substr(1);
	std::cout << "Enable pattern: " << pattern << std::endl;
	DebugMessage::enableMatchingMsgs("", pattern);
	cmdValid = false;
      } else if(cmd == '-'){
	std::string pattern = cmdStringRaw.substr(1);
	std::cout << "Disable pattern: " << pattern << std::endl;
	DebugMessage::disableMatchingMsgs("", pattern);
	cmdValid = false;
      } else if(cmd == '!'){
	std::cout << "Disable all." << std::endl;
	DebugMessage::disableAll();
	cmdValid = false;
      } else {
	std::cout << "Invalid command. Type \"H\" for help." << std::endl;
	cmdValid = false;
      }
    }
  }

  /**
   * @brief Console interaction loop
   */
  void interactiveRun() {

    while (true) {
      // Grab the current tick
      currentTick = TREX::Agent::instance()->getCurrentTick();

      // If this is the stop tick, display prompt
      if (currentTick == consoleStopTick) {
	printConsolePopup();
      }

      // Step agent forward
      if (!executive->step()) {
	// Mission complete
	std::cout << "Agent has completed its mission." << std::endl;
	printConsolePopup();
      }
    }
  }
}

/**
 * @brief Handle cleanup on exit
 */
void cleanup(){
  if(executive.isId()){
    delete (TREX::Executive*) executive;
    executive = TREX::ExecutiveId::noId();
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
  using namespace executive_trex_pr2;

  signal(SIGINT,  &TREX::signalHandler);
  signal(SIGTERM, &TREX::signalHandler);
  signal(SIGQUIT, &TREX::signalHandler);
  signal(SIGKILL, &TREX::signalHandler);

  ros::init(argc, argv, "trex");
  ros::NodeHandle node_handle;

  // Display help if requested
  if(executive_trex_pr2::isArg(argc, argv, "--help")){
    std::cout << "\n";
    std::cout << "Welcome! TREX is an executive for supervisory  control of an autonomous system. TREX requires the following ROS parameters:\n";
    std::cout << "* trex/input_file: An xml file that defines the agent control configuration.\n";
    std::cout << "* trex/path:       A search path for locating input files. This should include a location for\n";
    std::cout << "                   the input configuration file, as well as locations for agent initialization files (nddl files)\n";
    std::cout << "* trex/start_dir   A directory from which to start the search for input files\n";
    std::cout << "* trex/log_dir:    An output directory for TREX log files.\n";
    std::cout << "\n";
    std::cout << "Usage: trexfast  [--help | [--playback | --hyper ] [--console]]\n";
    std::cout << "--help:            Provides this message!\n";
    std::cout << "--playback:        Use if debugging a previous run. Expects an xml observation log file as input named <your_agent_name>.log\n";
    std::cout << "                   and a clock log file names clock.log. In each tick, this will enforce that the Agent is allowed only as many\n";
    std::cout << "                   execution steps that occured during the original run.\n";
    std::cout << "--hyper:           Run at 100% CPU for debugging. This enforces no limit on execution steps during each tick, and does not\n";
    std::cout << "                   sleep if there is time left before the next tick.\n";
    std::cout << "--console:         Run in interactive mode. This will have no effect on the number of execution steps during each tick.\n";
    return 0;
  }

  int success = 0;

  // Process input arguments
  g_playback = executive_trex_pr2::isArg(argc, argv, "--playback");
  g_hyper = executive_trex_pr2::isArg(argc, argv, "--hyper");
  g_console = executive_trex_pr2::isArg(argc, argv, "--console");

  // Hyper and playback make different assumptions about the execution steps in each tick
  if (g_playback && g_hyper) {
    ROS_ERROR("--hyper should not be used in playback. Disabling --hyper.\n");
    g_hyper = false;
  }

  try{
    // Get executive singleton
    executive = TREX::Executive::request(g_playback, g_hyper);

    if(g_console) {
      // Run with interactive stepping
      executive->interactiveInit();
      interactiveRun();
    } else {
      // Run in nonstop mode
      executive->run();
    }
  }
  catch(char* e){
    ROS_INFO("Caught %s. Shutting down.\n", e);
    success = -1;
  }
  catch(std::runtime_error e){
    ROS_ERROR(e.what());
  }
  catch(...){
    ROS_INFO("Caught Unknown Exception!. Shutting down.\n");
    success = -1;
  }

  if (success == -1){
    ROS_ERROR("Terminating execution. See logs/latest/TREX.log for clues and see cfg/debug.cfg to enable useful debugging messages to aid in analysis.");
    ROS_ERROR("You can also grep for trex:error and trex:warn in logs/latest directory");
  }

  delete (TREX::Executive*) executive;
  executive = TREX::ExecutiveId::noId();

  // Parse command line arguments to see if we must apply test case validation
  if(executive_trex_pr2::isArg(argc, argv, "--gtest")){
    testing::InitGoogleTest(&argc, argv);
    success = RUN_ALL_TESTS();
  }

  return success;
}

