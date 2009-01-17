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
#include <algorithm>
#include <pr2_mechanism_controllers/laser_scanner_controller.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(LaserScannerController)

LaserScannerController::LaserScannerController()
{
  robot_ = NULL;
  joint_ = NULL;

  command_ = 0;
  last_time_ = 0;
  profile_index_ = 0;
  profile_length_ = 0;

  //Clear arrays
  profile_locations_ = NULL;
  profile_dt_ = NULL;
  current_mode_ = MANUAL;
}

LaserScannerController::~LaserScannerController()
{
  //Free memory in profile if needed
  if(profile_locations_!=NULL) delete[] profile_locations_;
  if(profile_dt_!=NULL) delete[] profile_dt_;

}

void LaserScannerController::init(double p_gain, double i_gain, double d_gain, double windup, double time, std::string name, mechanism::RobotState *robot)
{
  robot_ = robot;
  joint_ = robot->getJointState(name);

  // TODO
  abort(); //joint_position_controller_.init( p_gain,  i_gain,  d_gain,  windup, time, name, robot);
  command_= 0;
  last_time_= time;
}

bool LaserScannerController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;
  last_time_ = robot->hw_->current_time_;

  //Perform checks at highest level to give the most informative error message possible
  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "LaserScannerController was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "LaserScannerController could not find joint named \"%s\"\n", joint_name);
    return false;
  }
/*
 TiXmlElement *p = j->FirstChildElement("pid");
  if (!p)
    fprintf(stderr, "LaserScannerController's config did not specify the default pid parameters.\n");
*/
  joint_position_controller_.initXml(robot,config); //Pass down XML snippet to encapsulated joint_position_controller_
  return true;


 }

// Set the joint position command
void LaserScannerController::setCommand(double command)
{
  command_ = command;
  current_mode_ = MANUAL;
}

// Return the current position command
double LaserScannerController::getCommand()
{
  return command_;
}

// Return the measured joint position
double LaserScannerController::getMeasuredPosition()
{
  return joint_->position_;
}

double LaserScannerController::getTime()
{
  return robot_->hw_->current_time_;
}

void LaserScannerController::update()
{
  if (!joint_->calibrated_)
    return;

  double time = robot_->hw_->current_time_;

  switch(current_mode_)
  {
    case MANUAL:
      joint_position_controller_.setCommand(command_);
      break;
    case SAWTOOTH:
    case SINEWAVE:
      joint_position_controller_.setCommand(profile_locations_[profile_index_]); //Issue position command

      //Check if enough time has elapsed to move to next set point
      if(time-time_of_last_point_ >= profile_dt_[profile_index_])
      {
        #ifdef DEBUG
        printf("DLL : %f %f\n",profile_locations_[profile_index_],time- time_of_last_point_);
        #endif
        time_of_last_point_ = time;

        //Advance time index
        if(profile_index_ == (profile_length_-1))
        {
          profile_index_ = 0; //Restart profile
          cycle_start_time_ = time;
        } else profile_index_++;
      }
      break;
    case DYNAMIC_SAWTOOTH:
      //Advance to next period
      if(time-cycle_start_time_>period_) cycle_start_time_ = time;

      //Issue command based on time from start of cycle
      setDynamicSawtooth(time-cycle_start_time_);
      break;
    case DYNAMIC_SINEWAVE:
      //Advance to next period
      if(time-cycle_start_time_>period_) cycle_start_time_ = time;

      //Issue command based on time from start of cycle
      setDynamicSinewave(time-cycle_start_time_);
      break;
    case AUTO_LEVEL:
      break;
    default:
      break;
  }
  joint_position_controller_.update(); //Update lower controller

  last_time_ = time; //Keep track of last time for update

}

//Set mode to use sawtooth profile
void LaserScannerController::setSawtoothProfile(double period, double amplitude, int num_elements, double offset)
{
  int smaller_num_elements = num_elements/4; //Number of elements in a single quadrant
  int total_elements = smaller_num_elements*4; //track actual number of elements after int truncation
  double delta = amplitude/smaller_num_elements; //Scale first, then determine step size
  double dt = period/total_elements;
  double current = 0;
  double newvalue = 0;

  //Error checking
  if(total_elements<=0) return;

  //Clear arrays
  if(profile_locations_ !=NULL) delete[] profile_locations_;
  if(profile_dt_ !=NULL) delete[] profile_dt_;

  profile_locations_ = new double[total_elements];
  profile_dt_ = new double[total_elements];

  //Construct evenly spaced elements in distance along sine wave
  for(int i = 0;i<total_elements;i++)
  {
    profile_locations_[i] = current + offset; //set current point
    profile_dt_[i] = dt; //Constant dt because of linear relationship

    newvalue = current + delta; //Calculate next value
    if(i == smaller_num_elements) //Shift from quadrant 1 to 2
    {
      delta = -delta;
      newvalue = current + delta;
    }
    else if (i == smaller_num_elements*3)//Shift from quadrant 3-4
    {
      delta = -delta;
      newvalue = current + delta;
    }
    current = newvalue;
  }

 //Reset profile settings
  profile_length_ = total_elements; //Keep track of profile length
  profile_index_= 0; //Start at beginning
  time_of_last_point_ = robot_->hw_->current_time_;

  current_mode_ = SAWTOOTH;
}

//Set mode to use sawtooth profile
void LaserScannerController::setSawtoothProfile(double period, double amplitude, double offset)
{
  period_ = period;
  amplitude_ = amplitude;
  offset_ = offset;

   //Reset profile settings
  profile_length_ = 0;
  profile_index_= 0;
  cycle_start_time_ = robot_->hw_->current_time_;

  current_mode_ = DYNAMIC_SAWTOOTH;
}

//Set mode to use Sinewave profile
void LaserScannerController::setSinewaveProfile(double period, double amplitude, int num_elements, double offset)
{
  int smaller_num_elements = num_elements/4; //Number of elements in a single quadrant
  int total_elements = smaller_num_elements*4; //track actual number of elements after int truncation
  double delta = 1.0/smaller_num_elements;
  double current = 0;
  double newvalue = 0;
  double temp_value = 0.0;
  double last_temp_value = 0.0;

  //Error checking
  if(total_elements<=0) return;

  //Clear arrays
  if(profile_locations_ !=NULL) delete[] profile_locations_;
  if(profile_dt_ !=NULL) delete[] profile_dt_;

  profile_locations_ = new double[total_elements];
  profile_dt_ = new double[total_elements];

  //Construct evenly spaced elements in distance along sine wave
  for(int i = 0;i<total_elements;i++)
  {
    profile_locations_[i] = current; //set current point
    newvalue = current + delta; //Calculate next value
    if(i == smaller_num_elements) //Shift from quadrant 1 to 2
    {
      delta = -delta;
      newvalue = current + delta;
    }
    else if (i == smaller_num_elements*3)//Shift from quadrant 3-4
    {
      delta = -delta;
      newvalue = current + delta;
    }

    current = newvalue;
    current = min(max(current, -1.0), 1.0); //Make sure asin doesn't fail
  }

  profile_locations_[0] = offset; //set first value

   //At time 0, we wish for our location to be at offset. Start indexing at 1, but associate dt with previous value
  for(int i = 1;i<total_elements;i++)
  {
    temp_value = asin(profile_locations_[i]); //Calculate time
    profile_dt_[i-1] = fabs(temp_value-last_temp_value)*period/(2*M_PI); //Calculate dt, scale by period
    profile_locations_[i] = temp_value*amplitude + offset; //Scale goal location by amplitude
    last_temp_value = temp_value;

    #ifdef DEBUG
    printf("*** test %u %f %f\n",i,profile_dt_[i-1],profile_locations_[i]);
    #endif
  }

  profile_dt_[total_elements-1] = profile_dt_[0]; //Make symmetric

  #ifdef DEBUG
  for(int i = 0;i<total_elements;i++)
  {
    printf("** test %u %f %f\n",i,profile_dt_[i],profile_locations_[i]);
  }
  #endif

 //Reset profile settings
  profile_length_ = total_elements; //Keep track of profile length
  profile_index_= 0; //Start at beginning
  time_of_last_point_ = robot_->hw_->current_time_;
  cycle_start_time_ = robot_->hw_->current_time_;

  current_mode_ = SINEWAVE;
}

//Set mode to use Sinewave profile
void LaserScannerController::setSinewaveProfile(double period, double amplitude,double offset)
{
   //Reset profile settings
  profile_length_ = 0;
  profile_index_= 0;
  cycle_start_time_ = robot_->hw_->current_time_;
  current_mode_ = DYNAMIC_SINEWAVE;

  period_ = period;
  amplitude_ = amplitude;
  offset_ = offset;
}

void LaserScannerController::startAutoLevelSequence()
{
  current_mode_=AUTO_LEVEL;
}

bool LaserScannerController::checkAutoLevelStatus()
{
  return (current_mode_==AUTO_LEVEL);
}

bool LaserScannerController::checkAutoLevelResult()
{
  return auto_level_result_;
}

void LaserScannerController::setJointEffort(double effort)
{
  joint_->commanded_effort_ = effort;
}

//Get sinewave based on current time
void LaserScannerController::setDynamicSinewave(double time_from_start)
{
  //double command = sin(2*M_PI*time_from_start/period_)*amplitude_+offset_;

  // Switched to cosine, so that the first half and second half are each a sweep (as opposed to 2/4+3/4 and 1/4+4/4 being sweeps)
  double command = cos(2*M_PI*time_from_start/period_)*amplitude_+offset_;
  joint_position_controller_.setCommand(command);

}

//Set mode to use sawtooth profile
void LaserScannerController::setDynamicSawtooth(double time_from_start)
{
  double time_from_peak = fmod(time_from_start,(period_/4));
  double command = (time_from_peak)/(period_/4)*amplitude_;

  if(time_from_start<period_/4.0) //Quadrant I
  {
    command = command + offset_;
  }
  else if (time_from_start>period_/4.0 && time_from_start<period_/2.0) //Quadrant II
  {
    command = amplitude_-command + offset_;
  }
  else if (time_from_start>period_/2.0 && time_from_start<period_*3/4.0) //Quadrant III
  {
    command = -command + offset_;
  }
  else if (time_from_start>period_*3/4.0 && time_from_start<period_) //Quadrant IV
  {
    command = -amplitude_ + command + offset_;
  }

  joint_position_controller_.setCommand(command);
}

LaserScannerController::ProfileExecutionState LaserScannerController::getProfileExecutionState()
{
  switch (current_mode_)
  {
    case SINEWAVE :
    case SAWTOOTH :
    {
      int halfway_point = profile_length_ / 2 ;             // We should be able to get away with doing integer math here.
      if (profile_index_ <= halfway_point)
        return FIRST_HALF ;
      else
        return SECOND_HALF ;
    }

    case DYNAMIC_SINEWAVE :
    case DYNAMIC_SAWTOOTH :
    {
      double time = robot_->hw_->current_time_ ;
      double time_from_start = time - cycle_start_time_ ;

      if ( time_from_start < period_ / 2.0 )
        return FIRST_HALF ;
      else
        return SECOND_HALF ;                                 // It is possible that (time_from_start > period). These should return as FIRST_HALF, but at this point, we don't care.
    }

    case MANUAL :
    case AUTO_LEVEL :
      return NOT_APPLICABLE ;
    default:
      return NOT_APPLICABLE ;
  }
}

ROS_REGISTER_CONTROLLER(LaserScannerControllerNode)
LaserScannerControllerNode::LaserScannerControllerNode(): node_(ros::Node::instance())
{
  c_ = new LaserScannerController();
  prev_profile_exec_state_ = LaserScannerController::NOT_APPLICABLE ;
  need_to_send_msg_ = false ;                                           // Haven't completed a sweep yet, so don't need to send a msg
  publisher_ = NULL ;                                                   // We don't know our topic yet, so we can't build it
}


LaserScannerControllerNode::~LaserScannerControllerNode()
{
  node_->unadvertiseService(service_prefix_ + "/set_command");
  node_->unadvertiseService(service_prefix_ + "/get_command");
  node_->unadvertiseService(service_prefix_ + "/set_profile");
  //node_->unadvertise(service_prefix_ + "/laser_scanner_signal") ;

  publisher_->stop() ;
  delete publisher_ ;    // Probably should wait on publish_->is_running() before exiting. Need to
                         //   look into shutdown semantics for realtime_publisher
  delete c_;
}

void LaserScannerControllerNode::update()
{
  c_->update();

  LaserScannerController::ProfileExecutionState cur_profile_exec_state ;
  cur_profile_exec_state = c_->getProfileExecutionState() ;
  switch (prev_profile_exec_state_)
  {
    case LaserScannerController::FIRST_HALF :
      if (cur_profile_exec_state == LaserScannerController::SECOND_HALF)   // We just transitioned from 1st->2nd, so send msg saying half-scan is done
      {
        m_scanner_signal_.signal = 0 ;                                     // 0 -> Half profile complete

        //if (need_to_send_msg_)                                             // Is there a message we were supposed to send, but never got to?
        //  printf("LaserScannerController:: Missed sending a msg\n") ;      // Is there a better way to output this error msg?

        need_to_send_msg_ = true ;
      }
      break ;
    case LaserScannerController::SECOND_HALF :                            // Send msg saying full scan is done
      if (cur_profile_exec_state == LaserScannerController::FIRST_HALF)   // We just transitioned from 2nd->1st, so send msg saying full-scan is done
      {
        m_scanner_signal_.signal = 1 ;                                    // 1 -> Full profile complete

        //if (need_to_send_msg_)                                             // Is there a message we were supposed to send, but never got to?
        //  printf("LaserScannerController:: Missed sending a msg\n") ;      // Is there a better way to output this error msg?

        need_to_send_msg_ = true ;
      }
      break ;
    case LaserScannerController::NOT_APPLICABLE :                         // Don't do anything
      break ;
    default :
      break ;
  }
  prev_profile_exec_state_ = cur_profile_exec_state ;

  // Use the realtime_publisher to try to send the message.
  //   If it fails sending, it's not a big deal, since we can just try again 1 ms later. No one will notice.
  if (need_to_send_msg_)
  {
    if (publisher_->trylock())
    {
      publisher_->msg_.header.stamp = ros::Time((uint64_t) (c_->getTime()*1000000000)) ;
      publisher_->msg_.signal = m_scanner_signal_.signal ;
      publisher_->unlockAndPublish() ;
      need_to_send_msg_ = false ;
    }
    //printf("tilt_laser: Signal trigger (%u)\n", m_scanner_signal_.signal) ;
    //std::cout << std::flush ;
  }
}


bool LaserScannerControllerNode::setCommand(
  robot_mechanism_controllers::SetCommand::request &req,
  robot_mechanism_controllers::SetCommand::response &resp)
{
  c_->setCommand(req.command);
  resp.command = c_->getCommand();

  //FIXME: Backdoor method to issue command set
  if(req.command==41)c_->setSawtoothProfile(1,0.5,100,0);
  else if(req.command==42)c_->setSawtoothProfile(2,0.5,100,0);
  else if(req.command==43)c_->setSawtoothProfile(2,0.5,100,0.5);
  else if(req.command==44)c_->setSawtoothProfile(0.5,0.5,100,0);
  else if(req.command==45)c_->setSawtoothProfile(1,0.5,0);
  else if(req.command==46)c_->setSawtoothProfile(20,0.872,100,0.3475); // JMH- latest running numbers from David with proper range considered
  else if(req.command==-41)c_->setSinewaveProfile(2,0.5,100,0.5);
  else if(req.command==-42)c_->setSinewaveProfile(2,0.5,100,0);
  else if(req.command==-43)c_->setSinewaveProfile(1,0.5,100,0);
  else if(req.command==-44)c_->setSinewaveProfile(4,0.5,100,0);
  else if (req.command==-45)c_->setSinewaveProfile(1,0.5,0);
  else if (req.command==-46)c_->setSinewaveProfile(3,0.5,0);
  else if (req.command==-47)c_->setSinewaveProfile(20,0.7,0);
  else if (req.command==-48)c_->setSinewaveProfile(15,1.2,0.25);
  else if (req.command==-49)c_->setSinewaveProfile(10,1.2,0.25);
  else if (req.command==-50)c_->setSinewaveProfile(8,1.2,0.25);
  else if (req.command==-51)c_->setSinewaveProfile(5,1.2,0.25);
  return true;
}


bool LaserScannerControllerNode::setProfileCall(
  pr2_mechanism_controllers::SetProfile::request &req,
  pr2_mechanism_controllers::SetProfile::response &resp)
{
  const double num_elem = -1.0 ;     // We should only be using the dynamicProfiles, so we don't need num_elem

  setProfile(LaserScannerController::LaserControllerMode(req.profile),req.period,req.amplitude,num_elem,req.offset);
  resp.time = c_->getTime();
  return true;
}

bool LaserScannerControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{

  service_prefix_ = config->Attribute("name");

  if (!c_->initXml(robot, config))
    return false;
  node_->advertiseService(service_prefix_ + "/set_command", &LaserScannerControllerNode::setCommand, this);
  node_->advertiseService(service_prefix_ + "/set_profile", &LaserScannerControllerNode::setProfileCall, this);


  if (publisher_ != NULL)               // Make sure that we don't memory leak if initXml gets called twice
    delete publisher_ ;
  publisher_ = new misc_utils::RealtimePublisher <pr2_mechanism_controllers::LaserScannerSignal> (service_prefix_ + "/laser_scanner_signal", 1) ;

  return true;
}


void LaserScannerControllerNode::setProfile(LaserScannerController::LaserControllerMode profile, double period, double amplitude, int num_elements, double offset)
{
  //Instruct the controller to perform an automatic profile
  switch(profile)
  {
    case LaserScannerController::SINEWAVE:
      c_->setSinewaveProfile(period, amplitude, num_elements, offset);
      break;
    case LaserScannerController::DYNAMIC_SINEWAVE:
      c_->setSinewaveProfile(period, amplitude,offset);
      break;
    case LaserScannerController::SAWTOOTH:
      std::cout << " setting sawtooth with these numbers " << period << " " << amplitude << " " << num_elements << " " << offset << std::endl;
      c_->setSawtoothProfile(period, amplitude, num_elements, offset);
      break;
    case LaserScannerController::DYNAMIC_SAWTOOTH:
      c_->setSawtoothProfile(period, amplitude,offset);
      break;
    default:
      break;
  }
}




