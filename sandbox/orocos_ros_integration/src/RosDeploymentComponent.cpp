/*
 * Copyright (c) 2008, Ruben Smits
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
 *     * Neither the name of Ruben Smits nor the names of its
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

/*
 * Author: Ruben Smits
 */

#include "RosDeploymentComponent.hpp"

namespace OCL{
  RosDeploymentComponent::RosDeploymentComponent(const std::string& name)
    : DeploymentComponent(name)
  {
    this->methods()->addMethod(method("rosnode",&RosDeploymentComponent::createRosNode,this),
			       "Creates a RosNode exporting the dataports for the given component",
			       "tc","Name of the TaskContext (must be a peer).");
  }
   
  RosDeploymentComponent::~RosDeploymentComponent(){
    log(Warning)<<"Deleting all ROS nodes"<<endlog();
    for(unsigned int i=0;i<rosnodes.size();i++)
      delete rosnodes[i].ros_;
  }

  bool RosDeploymentComponent::createRosNode(const std::string& tc){
    log(Debug)<<"Trying to create RosNode for component "<<tc<<endlog();
    TaskContext* peer = this->getPeer(tc);
    if(!peer){
      log(Error)<<"No such peer: "<< tc <<endlog();
      return false;
    }
    TaskContext* rosnode = RosNode::createRosNode(peer);
    if(rosnode!=0){
      rosnodes.push_back(couple(peer,rosnode));
      RTT::connectPorts(peer,rosnode);
      return true;
    }
    return false;
  }

  bool RosDeploymentComponent::componentLoaded(TaskContext* c){
    bool ros = comps[c->getName()].rosnode;
    if(ros){
      log(Info)<<"CreateRosNode flag was true"<<endlog();
      TaskContext* rosnode = RosNode::createRosNode(c);
      if(rosnode!=0){
	rosnodes.push_back(couple(c,rosnode));
	return true;
      }
      return false;
    }
    return true;
  }

  bool RosDeploymentComponent::connectRosPorts(){
    bool return_ = true;
    for(unsigned int i=0;i<rosnodes.size();i++){
      return_&=RTT::connectPorts(rosnodes[i].rtt_,rosnodes[i].ros_);
      rosnodes[i].ros_->start();
    }
    return return_;
  }
}
