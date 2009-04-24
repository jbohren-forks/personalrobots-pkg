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

#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>
#include <pr2_robot_actions/RechargeState.h>
#include <robot_msgs/BatteryState.h>
#include <cstdlib>

namespace highlevel_controllers {

  class RechargeController : public robot_actions::Action<std_msgs::Float32, std_msgs::Float32> {

  public:

    /**
     * @brief Constructor
     */
    RechargeController(const std::string& name);

    virtual ~RechargeController();


  private:
    enum MailType {UNPLUG, PLUG};


    /**
     * @brief Will update goal data.
     */
    virtual robot_actions::ResultStatus execute(const std_msgs::Float32& goalMsg, std_msgs::Float32& feedback);

    /**
     * @brief Will do what dispatch commands used to to
     */
    bool handleUpdate(std_msgs::Float32& feedback);

    void batteryStateCallback();

    void sendMail(MailType type);
    bool charged();
    bool connected() const;

    // Mail parameters
    std::string m_addresses, m_pluginSubject, m_unplugSubject, m_pluginBody, m_unplugBody, m_mailClient;
    std_msgs::Float32 _recharge_level;
    robot_msgs::BatteryState batteryStateMsg_;
    bool pluginNotified_;
    bool unplugNotified_;
    unsigned int connectionCount_; // Use to make sure we connect to a socket at least once. Handly for forcing a rest
  };

  RechargeController::RechargeController(const std::string& name)
    : robot_actions::Action<std_msgs::Float32, std_msgs::Float32>(name),
      m_addresses("mcgann@willowgarage.com"), m_pluginSubject("Robot Needs to Be Plugged In"), 
      m_unplugSubject("Robot Needs to Be Unplugged"), m_pluginBody("Hello, could you please plug me in?\nThanks, PR2"),
      m_unplugBody("Hello, could you please unplug me?\nThanks, PR2"), m_mailClient("mailx -s"),
      pluginNotified_(false), unplugNotified_(false), connectionCount_(0){

    ros::Node::instance()->param("recharge/email_addresses", m_addresses, m_addresses);
    ros::Node::instance()->param("recharge/subject_plugin", m_pluginSubject, m_pluginSubject);
    ros::Node::instance()->param("recharge/subject_unplug", m_unplugSubject, m_unplugSubject);
    ros::Node::instance()->param("recharge/body_plugin", m_pluginBody, m_pluginBody);
    ros::Node::instance()->param("recharge/body_unplug", m_unplugBody, m_unplugBody);
    ros::Node::instance()->param("recharge/mail_client", m_mailClient,  m_mailClient);
    ros::Node::instance()->param("recharge/body_unplug", m_unplugBody, m_unplugBody);
    ros::Node::instance()->param("recharge/mail_client", m_mailClient,  m_mailClient);

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
    ros::Node::instance()->subscribe("battery_state", batteryStateMsg_, &RechargeController::batteryStateCallback, this, 1);
  }

  RechargeController::~RechargeController(){}

  void RechargeController::batteryStateCallback(){

    // If not activated, do nothing
    if(!isActive())
      return;

    // If connected, we can reset notification to plug in. We also clear it to be releasable, meaning
    if(connected()){
      pluginNotified_ = false;
      connectionCount_++;
    }

    // If !connected, we can reset notification to unplug
    if(!connected() )
      unplugNotified_ = false;
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
  
  robot_actions::ResultStatus RechargeController::execute(const std_msgs::Float32& goalMsg, std_msgs::Float32& feedback){
    _recharge_level.lock();
    _recharge_level = goalMsg;
    pluginNotified_ = false;
    unplugNotified_ = false;
    connectionCount_ = 0;
    _recharge_level.unlock();

    // Will update at 10 Hz
    ros::Duration d; d.fromSec(0.10);
    while(!isPreemptRequested() && !handleUpdate(feedback))
      d.sleep();

    if(isPreemptRequested())
      return robot_actions::PREEMPTED;

    return robot_actions::SUCCESS;
  }

  bool RechargeController::handleUpdate(std_msgs::Float32& feedback){

    if(!connected() && charged() && connectionCount_ > 0){
      feedback = _recharge_level;
      return true;
    }
    else {
      if(!connected() && (connectionCount_ == 0 || !charged()) && !pluginNotified_){
	sendMail(PLUG);
	pluginNotified_ = true;
	ROS_DEBUG("Requested help to plug in.");
      }
      else if(charged() && connected() && !unplugNotified_){
	sendMail(UNPLUG);
	unplugNotified_ = true;
	ROS_DEBUG("Requested help to unplug.");
      }
    }

    return false;
  }

  bool RechargeController::charged() {
    batteryStateMsg_.lock();
    bool result = (batteryStateMsg_.energy_remaining / batteryStateMsg_.energy_capacity);
    batteryStateMsg_.unlock();
    return result;
  }

  /**
   * If published power consumption is positive, then we must be connected to a power source
   */
  bool RechargeController::connected() const {
    return batteryStateMsg_.power_consumption > -5;
  }
}

int
main(int argc, char** argv)
{
  ros::init(argc,argv);

  ros::Node node("highlevel_controller/rechargeController");

  // Allocate an action runner with an update rate of 10 Hz
  robot_actions::ActionRunner runner(10.0);

  highlevel_controllers::RechargeController recharge_controller("rechargeController");

  runner.connect<std_msgs::Float32, pr2_robot_actions::RechargeState, std_msgs::Float32>(recharge_controller);

  runner.run();

  node.spin();

  return(0);
}
