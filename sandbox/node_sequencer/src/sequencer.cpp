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


/**
 * \file
 *
 * Node that sequences events across other nodes
 *
 */


#include <fstream>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <std_msgs/String.h>


#define foreach BOOST_FOREACH

using namespace std;
using std_msgs::String;
namespace po=boost::program_options;


namespace Statuses { enum Status { CANNOT_RUN, CAN_RUN, COMPLETED }; }
typedef Statuses::Status Status;

typedef vector<string> StringVector;
typedef map<string, StringVector> StringMap;
typedef map<string, Status> StatusMap;
typedef pair<string, StringVector> StringMapPair;
typedef pair<string, Status> StatusMapPair;




/************************************************************
 * Class
 ************************************************************/



class NodeSequencer
{
public:
  NodeSequencer (const string& input_file);
  void run();

private:

  void changeStatus (const string& task, Status new_status);
  Status currentStatus (const string& task) const;
  void completionCallback (const String::ConstPtr& msg);
  bool isIncomplete (const string& task) const;
  const StringVector& parents (const string& task) const;
  void print() const;
  void addTaskIfNecessary(const string& name);

  StringMap parents_;
  StatusMap statuses_;
  StringVector tasks_;
  ros::NodeHandle node_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  ros::ServiceServer service_;
};


/************************************************************ 
 * Methods
 ************************************************************/

NodeSequencer::NodeSequencer (const string& input_file)
{
  ifstream str(input_file.c_str());
  if (str.is_open()) {
    
    typedef boost::char_separator<char> Separator;
    typedef boost::tokenizer<Separator> Tokenizer;
    Separator sep(" ");

    string line;
    while (!str.eof()) {
      getline(str, line);
      Tokenizer tok(line, sep);
      Tokenizer::iterator iter=tok.begin();

      if (distance(tok.begin(), tok.end())==2) {
        string parent = *iter++;
        string task = *iter;
        addTaskIfNecessary(task);
        addTaskIfNecessary(parent);
        parents_[task].push_back(parent);
      }
    }

    publisher_ = node_.advertise<String>("~can_start", 1000);
    subscriber_ = node_.subscribe("~completed", 1000, &NodeSequencer::completionCallback, this);
  }

  else {
    ROS_FATAL_STREAM ("Could not open file " << input_file);
    exit(1);
  }
}

void NodeSequencer::addTaskIfNecessary(const string& name)
{
  if (find(tasks_.begin(), tasks_.end(), name)==tasks_.end()) {
    tasks_.push_back(name);
    StringVector v;
    parents_[name] = v;
    statuses_[name] = Statuses::CANNOT_RUN;
  }
}

void NodeSequencer::run ()
{
  // Main loop
  ros::Duration d(.1);
  while (node_.ok()) {

    foreach (string task, tasks_) {
      const StringVector& pars = parents(task);

      if (currentStatus(task) == Statuses::CANNOT_RUN && 
          find_if(pars.begin(), pars.end(), bind(&NodeSequencer::isIncomplete, this, _1)) == pars.end())
        changeStatus (task, Statuses::CAN_RUN);

      if (currentStatus(task) == Statuses::CAN_RUN) {
        String msg;
        msg.data = task;
        publisher_.publish(msg);
      }
      ros::spinOnce();
      d.sleep();
    }
  }
}

bool NodeSequencer::isIncomplete (const string& task) const
{
  return currentStatus(task)!=Statuses::COMPLETED;
}

const StringVector& NodeSequencer::parents (const string& task) const
{
  StringMap::const_iterator pos = parents_.find(task);
  if (pos==parents_.end()) {
    ROS_FATAL_STREAM ("Could not find parents of " << task);
    exit(1);
  }
  return pos->second;
}

string statusString(const Status status)
{
  if (status == Statuses::CANNOT_RUN) 
    return string("CANNOT_RUN");
  else if (status == Statuses::CAN_RUN) 
    return string("CAN_RUN");
  else 
    return string("COMPLETED");
}

void NodeSequencer::print() const
{
  foreach (string task, tasks_) 
    ROS_INFO_STREAM ("task " << task);

  foreach (StringMapPair pair, parents_) {
    ROS_INFO_STREAM ("Parents of task " << pair.first);
    foreach (string parent, pair.second) {
      ROS_INFO_STREAM (" " << parent);
    }
  }

  foreach (StatusMapPair pair, statuses_) 
    ROS_INFO_STREAM ("Status of " << pair.first << " is " << statusString(pair.second));
}

 
void NodeSequencer::changeStatus (const string& task, const Status new_status)
{
  const Status current_status = currentStatus(task);
  string current_str = statusString(current_status);
  string new_str = statusString(new_status);

  if (current_status > new_status) {
    ROS_FATAL_STREAM ("Tried to change status of " << task << " from " << current_str << " to " << new_str);
    exit(1);
  }
  ROS_INFO_STREAM ("Sequencer changing status of " << task << " from " << current_str << " to " << new_str);
  statuses_[task] = new_status;
}

Status NodeSequencer::currentStatus (const string& task) const
{
  StatusMap::const_iterator pos = statuses_.find(task);
  if (pos==statuses_.end()) {
    ROS_FATAL_STREAM ("Couldn't look up task " << task);
    exit(1);
  }
  return pos->second;
}
        
void NodeSequencer::completionCallback (const String::ConstPtr& msg)
{
  if (statuses_.find(msg->data)==statuses_.end()) 
    ROS_ERROR_STREAM ("Returned completion notification for unknown task " << msg->data);
  else if (currentStatus(msg->data)==Statuses::CANNOT_RUN) 
    ROS_ERROR_STREAM ("Tried to set " << msg->data << " to COMPLETED, but its current status is " << statusString(currentStatus(msg->data)));
  else if (currentStatus(msg->data)==Statuses::CAN_RUN) {
    ROS_DEBUG_STREAM ("In completion callback, setting status of " << msg->data << " to completed");
    changeStatus(msg->data, Statuses::COMPLETED);
  }
}

  





/************************************************************
 * Main
 ************************************************************/

int main (int argc, char** argv)
{
  string input_file;

  // Process command-line arguments
  {
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help,h", "produce help message")
      ("constraint_file,c", po::value<string>(&input_file), "File containing constraints");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") || !vm.count("constraint_file")) {
      cout << desc << "\n";
      return 1;
    }
  }

  ros::init(argc, argv, "node_sequencer");

  NodeSequencer seq(input_file);
  seq.run();

  return 0;
}

