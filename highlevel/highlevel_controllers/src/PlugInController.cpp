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

#include <HighlevelController.hh>

#include <highlevel_controllers/PlugInState.h>
#include <highlevel_controllers/PlugInGoal.h>
#include <robot_msgs/BatteryState.h>

namespace highlevel_controllers {

  class PlugIn : public HighlevelController<PlugInState, PlugInGoal> {
    
  private:
    std::string m_addresses, m_plugin_subject, m_unplug_subject, m_plugin_body, m_unplug_body, m_mail_client;
    
    enum {STATE_UNPLUGGED, STATE_PLUGGED_IN} m_state;
    enum {GOAL_UNPLUGGED, GOAL_PLUGGED_IN, GOAL_NONE} m_goal;
    bool m_mailRequired;

    robot_msgs::BatteryState batteryStateMsg;
  public:
    
    /**
     * @brief Constructor
     */
    PlugIn()
      : HighlevelController<PlugInState, PlugInGoal>("plugin_controller", "plugin_state", "plugin_goal"),
	m_addresses(""), m_plugin_subject("Robot Needs to Be Plugged In"), 
	m_unplug_subject("Robot Needs to Be Unplugged"), m_plugin_body("Hello, could you please plug me in?\nThanks, PR2"),
	m_unplug_body("Hello, could you please unplug me?\nThanks, PR2"), m_mail_client("mailx -s"),
	m_state(STATE_UNPLUGGED), m_goal(GOAL_NONE), m_mailRequired(false) {
      
      param("plugin/email_addresses", m_addresses, m_addresses);
      param("plugin/subject_plugin", m_plugin_subject, m_plugin_subject);
      param("plugin/subject_unplug", m_unplug_subject, m_unplug_subject);
      param("plugin/body_plugin", m_plugin_body, m_plugin_body);
      param("plugin/body_unplug", m_unplug_body, m_unplug_body);
      param("plugin/mail_client", m_mail_client,  m_mail_client);

      subscribe("battery_state", batteryStateMsg, &PlugIn::batteryStateCallback, QUEUE_MAX());
      
      if (m_addresses == "") {
	printf("There are no email addresses.\n");
	FILE* email = fopen("email_file.txt", "r");
	if (email) {
	  char buff[1024];
	  fgets(buff, 1020, email);
	  for (int i = 0; i < 1024; i++) {
	    if (buff[i] == '\r' || buff[i] == '\n') { 
	      buff[i] = 0; 
	      break;
	    }
	  }
	  m_addresses = buff;
	  fclose(email);
	} else {
	  printf("Could not open email alert file: email_file.txt\n");
	}
      }
      printf("Emails: %s\n", m_addresses.c_str());
    }
    
    ~PlugIn(){
    }

    void batteryStateCallback() {
      lock();
      m_state = (batteryStateMsg.power_consumption < 0) ? STATE_UNPLUGGED : STATE_PLUGGED_IN;
      unlock();

      printf("Got battery state: %s.\n", (m_state == STATE_UNPLUGGED) ? "unplugged" : "plugged in");

      // Now initialize
      initialize();
    }
    
    bool goalReached() {
      return (m_state == STATE_UNPLUGGED && m_goal == GOAL_UNPLUGGED)
	|| (m_state == STATE_PLUGGED_IN && m_goal == GOAL_PLUGGED_IN) || (m_goal == GOAL_NONE);
    }
    

    bool makePlan() {
      if (!goalReached()) {
	m_mailRequired = true;
      }
      return true;
    }

    
    void updateGoalMsg(){
      lock();
      if (goalMsg.goal) {
	m_goal = GOAL_PLUGGED_IN;
      } else {
	m_goal = GOAL_UNPLUGGED;
      }
      unlock();
      
      printf("Received new goal %s.\n", (m_goal == GOAL_PLUGGED_IN) ? "plugin" : "unplug");
    }
    
    void updateStateMsg(){
      
      // Assign state data 
      lock();
      stateMsg.done = goalReached();
      stateMsg.goal = (m_goal == GOAL_PLUGGED_IN);
      stateMsg.state = (m_state == STATE_PLUGGED_IN);
      unlock();
      
    }

    bool dispatchCommands() {
      //Does nothing.
      if (m_mailRequired && m_addresses != "") {
	printf("Sending mail...\n");
	std::string command = "echo \"";
	command += (m_goal == GOAL_PLUGGED_IN) ? m_plugin_body : m_unplug_body;
	command += "\" | " + m_mail_client + " \"";
	command += (m_goal == GOAL_PLUGGED_IN) ? m_plugin_subject : m_unplug_subject;
	command += "\" \"";
	for (unsigned int i = 0; i <  m_addresses.length(); i++) {
	  if (m_addresses[i] == ' ') {
	    command += "\" \"";
	  } else {
	    command += m_addresses[i];
	  }
	}
	command += "\"";
	
	
	printf("Command: %s\n", command.c_str());
        system(command.c_str());
	m_mailRequired = false;
      }
      return true;
    }
  };
}


int main(int argc, char** argv) {

  if(argc != 1){
    std::cout << "Usage: ./" << argv[0];
    return -1;
  }

  ros::init(argc,argv);
  
  highlevel_controllers::PlugIn node;
  node.run();
  ros::fini();
 

  return(0);
}
