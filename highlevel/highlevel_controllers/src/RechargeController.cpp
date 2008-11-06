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

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 *
 * @b recharge is a node which will handle plugging in and unpluggin the robot, assuming the plug location
 * is within the immediate vicinity.
 *
 * <hr>
 *
 *  @section usage Usage
 *  @verbatim
 *  $ recharge
 *  @endverbatim
 *
 * <hr>
 *
 * @section topic ROS topics
 *
 * Subscribes to (name/type):
 * - @b 
 *
 * Publishes to (name / type):
 * - @b 
 *
 *  <hr>
 *
 * @section parameters ROS parameters
 *
 * - None
 **/

#include <HighlevelController.hh>
#include <highlevel_controllers/RechargeGoal.h>
#include <highlevel_controllers/RechargeState.h>
#include <robot_msgs/BatteryState.h>
#include <cstdlib>

namespace highlevel_controllers {

  class RechargeController : public HighlevelController<RechargeState, RechargeGoal> {

  public:

    /**
     * @brief Constructor
     */
    RechargeController(const std::string& stateTopic, const std::string& goalTopic);

    virtual ~RechargeController();


  private:
    enum MailType {UNPLUG, PLUG};

    enum State {INACTIVE, CONNECT, CHARGE, DISCONNECT};

    /**
     * HighlevelController interface
     */
    virtual void updateGoalMsg();
    virtual void updateStateMsg();
    virtual bool makePlan();
    virtual bool goalReached();
    virtual bool dispatchCommands();

    void batteryStateCallback();

    void sendMail(MailType type);
    bool charged() const;
    bool connected() const;

    // Mail parameters
    std::string m_addresses, m_pluginSubject, m_unplugSubject, m_pluginBody, m_unplugBody, m_mailClient;
    

    State controlState_; // The state of the controller
    robot_msgs::BatteryState batteryStateMsg_;
  };

  RechargeController::RechargeController(const std::string& stateTopic, const std::string& goalTopic)
    : HighlevelController<RechargeState, RechargeGoal>("recharge_controller", stateTopic, goalTopic),
      m_addresses("mcgann@willowgarage.com"), m_pluginSubject("Robot Needs to Be Plugged In"), 
      m_unplugSubject("Robot Needs to Be Unplugged"), m_pluginBody("Hello, could you please plug me in?\nThanks, PR2"),
      m_unplugBody("Hello, could you please unplug me?\nThanks, PR2"), m_mailClient("mailx -s"),
      controlState_(INACTIVE){

    param("recharge/email_addresses", m_addresses, m_addresses);
    param("recharge/subject_plugin", m_pluginSubject, m_pluginSubject);
    param("recharge/subject_unplug", m_unplugSubject, m_unplugSubject);
    param("recharge/body_plugin", m_pluginBody, m_pluginBody);
    param("recharge/body_unplug", m_unplugBody, m_unplugBody);
    param("recharge/mail_client", m_mailClient,  m_mailClient);
    param("recharge/body_unplug", m_unplugBody, m_unplugBody);
    param("recharge/mail_client", m_mailClient,  m_mailClient);

    if (m_addresses == "") {
      ROS_INFO("There are no email addresses in the param server. Opening the text file.\n");
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
	ROS_INFO("Could not open email alert file: email_file.txt\n");
      }
    }

    ROS_INFO("Emails: %s\n", m_addresses.c_str());


    // We will listen to battery state messages to monitor for transitions. We only care about the latest one
    subscribe("battery_state", batteryStateMsg_, &RechargeController::batteryStateCallback, 1);

    lock();
    stateMsg.recharge_level = 0.0;
    stateMsg.goal_recharge_level = 0.0;
    unlock();
  }

  RechargeController::~RechargeController(){}

  void RechargeController::batteryStateCallback(){
    // Set to initalized (benign if already true)
    initialize();
  }

  void RechargeController::sendMail(MailType type) {
    std::string subject = "Error: bad mail.", body = "Bad, bug in recharge controller.";
    if (type == UNPLUG) {
      subject = m_unplugSubject;
      body = m_unplugBody;
    } else if (type == PLUG) {
      subject = m_pluginSubject;
      body = m_pluginBody;
    }
    
    ROS_INFO("Sending mail...\n");
    std::string command = "echo \"";
    command += body + "\" | " + m_mailClient + " \"";
    command += subject + "\" \"";
    for (unsigned int i = 0; i <  m_addresses.length(); i++) {
      if (m_addresses[i] == ' ') {
	command += "\" \"";
      } else {
	command += m_addresses[i];
      }
    }
    command += "\"";
    system(command.c_str());
    ROS_INFO("Mail command sent: %s\n", command.c_str());
  }
  
  void RechargeController::updateGoalMsg(){
    stateMsg.lock();
    stateMsg.goal_recharge_level = goalMsg.recharge_level;
    stateMsg.unlock();
  }

  void RechargeController::updateStateMsg(){
    batteryStateMsg_.lock();
    stateMsg.recharge_level = (batteryStateMsg_.energy_remaining / batteryStateMsg_.energy_capacity);
    batteryStateMsg_.unlock();
  }

  bool RechargeController::makePlan(){
    if(!isValid())
      controlState_ = CONNECT;

    return true;
  }

  /**
   * @brief When the state machine has transitioned back into an inactive state, we think we are done
   */
  bool RechargeController::goalReached(){return controlState_ == INACTIVE;}

  /**
   * @brief Commands involve sending mail to plug and unplug, as well as waiting to charge. This state machine is strictly linear.
   */
  bool RechargeController::dispatchCommands(){
    switch(controlState_){
    case CONNECT:
      ROS_DEBUG("Transitioning to charge\n");
      if(!connected())
	sendMail(PLUG);
      controlState_ = CHARGE;
      break;
    case CHARGE:
      ROS_DEBUG("Charged up to %f percent, target is %f\n", stateMsg.recharge_level, stateMsg.goal_recharge_level);
      if(charged()){
	sendMail(UNPLUG);
	ROS_DEBUG("Transitioning to disconnect\n");
	controlState_ = DISCONNECT;
      }
      break;
    case DISCONNECT:
      if(!connected()){
	ROS_DEBUG("Transitioning to inactive\n");
	controlState_ = INACTIVE;
      }
    default:
      ROS_ASSERT(0); // Should never get here. Should be benign though. If we do the logic is incorrect somewhere
    }

    return true;
  }

  bool RechargeController::charged() const{
    return stateMsg.recharge_level >= stateMsg.goal_recharge_level;
  }

  /**
   * If published power consumption is positive, then we must be connected to a power source
   */
  bool RechargeController::connected() const {
    return batteryStateMsg_.power_consumption > 0;
  }
}

int
main(int argc, char** argv)
{
  ros::init(argc,argv);
  try{
    highlevel_controllers::RechargeController node("recharge_state", "recharge_goal");
    node.run();
  }
  catch(...){
    ROS_DEBUG("Caught expection running node. Cleaning up.\n");
  }

  ros::fini();

  return(0);
}
