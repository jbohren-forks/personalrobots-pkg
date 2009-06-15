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

#include "Orocos2Ros.hpp"
#include <ros/callback_queue.h>
#include <rtt/Logger.hpp>

RTT::TaskContext* RosNode::createRosNode(RTT::TaskContext* tc){
  RTT::log(RTT::Info)<<"Creating RosNode for component "<<tc->getName()<<RTT::endlog();
  RosNode* ros_node = new RosNode(tc->getName());
  
  /*************************************
   * Create the corresponding ROS node *
   *************************************/
  //iterate over all ports of tc
  const RTT::DataFlowInterface::Ports& ports = tc->ports()->getPorts();
  //for(RTT::DataFlowInterface::Ports::const_iterator it = tc->ports()->getPorts().begin();it!=tc->ports()->getPorts().end();++it){
  for(unsigned int i=0;i<ports.size();i++){
    if(ports[i]->getPortType()==RTT::PortInterface::WritePort){
      log(RTT::Info)<<"Creating RosWritePort for "<<tc->getName()+"."+ports[i]->getName()<<RTT::endlog();
      //Create anticlone for the port
      RTT::PortInterface* anticlone(ports[i]->antiClone());
      //Add the anticlone to the ports of the node
      ros_node->ports()->addPort(anticlone);
      //connect the port and its anticlone
      //ports[i]->connectTo(anticlone);
      //Create the RosWritePort
      ros_node->write_ports_.push_back(RTT::RosPortCreator::Instance()->createWritePort(&ros_node->n,anticlone,ros_node));
    }
    
    else if(ports[i]->getPortType()==RTT::PortInterface::ReadPort){
      log(RTT::Info)<<"Creating RosReadPort for "<<tc->getName()+"."+ports[i]->getName()<<RTT::endlog();
      //Create anticlone for the port
      RTT::PortInterface* anticlone(ports[i]->antiClone());
      //Add the anticlone to the ports of the node
      ros_node->ports()->addPort(anticlone);
      //connect the port and its anticlone
      //ports[i]->connectTo(anticlone);
      //create the subscribe object
      ros_node->read_ports_.push_back(RTT::RosPortCreator::Instance()->createReadPort(&ros_node->n,anticlone));
    }
    else{
      log(RTT::Error)<<"No RosPort created for "<<tc->getName()+"."+ports[i]->getName()<<", ReadWritePort is not supported"<<RTT::endlog();
    }
  }
  tc->addPeer(ros_node);
  return ros_node;
}

bool RosNode::startHook(){
  log(RTT::Debug)<<"RosNode start"<<RTT::endlog();
  //this->doTrigger();
  return true;
}

void RosNode::updateHook(){
  if(!n.ok()){
    log(RTT::Error)<<"RosNode is dead :("<<RTT::endlog();
    this->error();
    return;
  }
  //this->doTrigger();
}

void RosNode::stopHook(){
  log(RTT::Debug)<<"RosNode stop"<<RTT::endlog();
  
}

RosNode::~RosNode(){
  for(unsigned int i=0;i<write_ports_.size();i++)
    delete write_ports_[i];
  for(unsigned int i=0;i<read_ports_.size();i++)
    delete read_ports_[i];

  const RTT::DataFlowInterface::Ports& ports = this->ports()->getPorts();
  for(unsigned int i=0;i<ports.size();i++)
    delete ports[i];
}
