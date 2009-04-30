/*
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

/** \author Wim Meeussen */


#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <string>

using namespace tf;
using namespace ros;
using namespace std;


class TFMonitor
{
public:
  std::string framea_, frameb_;
  
  Node node_;
  std::vector<std::string> chain_;
  std::map<std::string, std::vector<double> > delay_map;
  
  TransformListener tf_;

  tf::tfMessage message_;

  boost::mutex map_lock_;
  void callback()
  {
    boost::mutex::scoped_lock my_lock(map_lock_);  
    for (unsigned int i = 0; i < message_.transforms.size(); i++)
    {
      std::map<std::string, std::vector<double> >::iterator it = delay_map.find(message_.transforms[i].header.frame_id);
      if (it == delay_map.end())
        delay_map[message_.transforms[i].header.frame_id] = std::vector<double>(1,(ros::Time::now() - message_.transforms[i].header.stamp).toSec());
      else
        it->second.push_back((ros::Time::now() - message_.transforms[i].header.stamp).toSec());
    } 
  };

  TFMonitor(std::string framea, std::string frameb):
    framea_(framea), frameb_(frameb),
    node_("tf_monitor", ros::Node::ANONYMOUS_NAME),
    tf_(node_)
  {
    
  cout << "Waiting for first transfrom to become available" << flush;
  while (node_.ok() && !tf_.canTransform(framea_, frameb_, Time(), Duration(1.0)))
    cout << "." << flush;
  cout << endl;
  
  tf_.chainAsVector(framea_, ros::Time(), frameb_, ros::Time(), frameb_, chain_);
  cout << "Chain currently is:" <<endl;
  for (unsigned int i = 0; i < chain_.size(); i++)
  {
    cout << chain_[i] <<", ";
  }
  cout <<endl;

  node_.subscribe("tf_message", message_, &TFMonitor::callback, this, 100);
    
  }

  
  void spin()
  {  
    
    // create tf listener
    double max_diff = 0;
    double avg_diff = 0;
    double lowpass = 0.01;
    unsigned int counter = 0;
    
  

  while (node_.ok()){
    Stamped<Transform> tmp;

    tf_.lookupTransform(framea_, frameb_, Time(), tmp);
    double diff = (Time::now() - tmp.stamp_).toSec();
    avg_diff = lowpass * diff + (1-lowpass)*avg_diff;
    if (diff > max_diff) max_diff = diff;
    Duration(0.01).sleep();
    counter++;
    if (counter > 100){
      counter = 0;
      cout << "Net delay " << framea_ << " to " << frameb_ << "    avg = " << avg_diff <<": max = " << max_diff << endl;
      boost::mutex::scoped_lock lock(map_lock_);  
      for ( unsigned int i = 0 ; i < chain_.size(); i++)
      {
        std::map<std::string, std::vector<double> >::iterator it = delay_map.begin();
        for ( ; it != delay_map.end() ; ++it)
        {
          if (it->first != chain_[i])
            continue;
          double average_delay = 0;
          double max_delay = 0;
          for (unsigned int i = 0; i < it->second.size(); i++)
          {
            average_delay += it->second[i];
            max_delay = std::max(max_delay, it->second[i]);
          }
          average_delay /= it->second.size();
          cout << "Frame: " << it->first << " Average Delay: " << average_delay << " Max Delay: " << max_delay << std::endl;
        }
      }
      
    }
  }

  }
};


int main(int argc, char ** argv)
{
  //Initialize ROS
  init(argc, argv);

  string framea, frameb;
  if (argc == 3){
    framea = argv[1];
    frameb = argv[2];
  }
  else{
    ROS_INFO("TF_Monitor: usage: tf_monitor framea frameb");
    return -1;
  }
  TFMonitor monitor(framea, frameb);
  monitor.spin();

  return 0;

}
