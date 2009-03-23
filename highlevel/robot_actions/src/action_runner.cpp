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

// Author Conor McGann (mcgann@willowgarage.com)

#include <robot_actions/action_runner.h>

namespace robot_actions {

  ActionRunner::ActionRunner(double update_rate): _initialized(false), _terminated(false), _update_rate(update_rate), _update_thread(NULL){

    ROS_ASSERT(_update_rate > 0);

    // Start the action_runner_thread
    _update_thread = new boost::thread(boost::bind(&ActionRunner::updateLoop, this));
  }


  ActionRunner::~ActionRunner(){
    terminate();
    _update_thread->join();

    // Now deallocate all adapters
    for(std::vector<AbstractAdapter*>::const_iterator it = _adapters.begin(); it != _adapters.end(); ++it){
      AbstractAdapter* adapter = *it;
      delete adapter;
    }
  }

  void ActionRunner::terminate() {
    _terminated = true;
  }
  
  bool ActionRunner::isTerminated() const {
    return _terminated;
  }

  void ActionRunner::run(){
    // Initialize all actions
    for(std::vector<AbstractAdapter*>::const_iterator it = _adapters.begin(); it != _adapters.end(); ++it){
      AbstractAdapter* adapter = *it;
      adapter->initialize();
    }

    // Mark ready to run
    _initialized = true;
  }

  void ActionRunner::updateLoop(){
    
    while(ros::Node::instance()->ok()) {
      ros::Time curr = ros::Time::now();
      bool done(true);

      if(_initialized){

	// Iterate through adapters to ping each one for an update
	for(std::vector<AbstractAdapter*>::const_iterator it = _adapters.begin(); it != _adapters.end(); ++it){
	  AbstractAdapter* adapter = *it;

	  if(!isTerminated()){
	    adapter->update();
	  }
	  else if(adapter->isOk()){
	    done = false;
	    adapter->terminate();
	  }
	}
      }
      else {
	ROS_DEBUG("Action Runner pending initialization");
      }

      // If we are terminated and done then we can quit the thread
      if(isTerminated() && done){
	ROS_DEBUG("Terminating action runner update loop");
	break;
      }

      sleep(curr, 1 / _update_rate);
    }
  }
  
  void ActionRunner::sleep(ros::Time loopstart, double loopDuration){
    ros::Time curr = ros::Time::now();
    ros::Duration cycleTime;
    cycleTime = cycleTime.fromSec(loopDuration);
    ros::Time desiredCycleEnd = loopstart + cycleTime;
    
    ros::Duration diff = desiredCycleEnd - curr;
    
    if(diff <= ros::Duration()){
      ROS_DEBUG("Missed deadline and not sleeping; check machine load. Started %f, ended %f, wanted end %f. Wanted delta %f, got %f. Tryed to correct %f sec.\n", 
		loopstart.toSec(), curr.toSec(), desiredCycleEnd.toSec(), cycleTime.toSec(), (curr - loopstart).toSec(), diff.toSec());
    } else {
      diff.sleep();
    }
  }
}
