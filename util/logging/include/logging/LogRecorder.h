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

#ifndef LOGRECORDER_H
#define LOGRECORDER_H

#include "ros/time.h"
#include "rosthread/mutex.h"
#include "logging/LogFunctor.h"

#include <fstream>
#include <iostream>
#include <iomanip>


class LogRecorder
{

  typedef std::vector<std::pair<std::string, std::string> > TopicList;

  class LogHelper : public ros::msg
  {

    LogRecorder* log_recorder_;

    std::string topic_name_;
    std::string datatype_;
    std::string md5sum_;

    AbstractLogFunctor* callback_;

    ros::msg* msg_;

    unsigned int max_queue_;

    bool first_;

  public:

  LogHelper(LogRecorder* log_recorder, std::string topic_name, std::string datatype, std::string md5sum, unsigned int max_queue)
    : log_recorder_(log_recorder), 
      topic_name_(topic_name), datatype_(datatype), md5sum_(md5sum),
      callback_(NULL),
      msg_(NULL),
      max_queue_(max_queue),
      first_(true)
    {}
    
    virtual ~LogHelper()
    {
      if (msg_)
        delete msg_;

      if (callback_)
        delete callback_;
    }

    void addHandler(AbstractLogFunctor* callback)
    {
      if (msg_ == NULL)
        msg_ = callback->allocateMsg();
      else
        msg_ = this;

      callback_ = callback;
    }

    void callHandler()
    {
      if (callback_)
      {
        ros::Time now = ros::Time::now();
        callback_->call(topic_name_, msg_, now);
      }
    }

    std::string get_topic_name() {return topic_name_;}

    int get_max_queue() {return max_queue_;}

    virtual const std::string __get_datatype() const { return datatype_; }

    virtual const std::string __get_md5sum()   const { return md5sum_; }

     virtual uint32_t serialization_length()    { return __serialized_length; }

    virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t) { assert(0); return NULL; }

    virtual uint8_t *deserialize(uint8_t *read_ptr)
    {
      if (first_)
      {
        std::string mapped_topic_name = log_recorder_->node_->map_name(topic_name_);

        if (datatype_ == "*")
        {
          TopicList topics;
          
          log_recorder_->node_->get_published_topics(&topics);
            
          for (TopicList::iterator i = topics.begin(); i != topics.end(); i++)
            if (i->first == mapped_topic_name)
            {
              datatype_ = i->second;
              break;
            }
        }
        
        std::cout << "LogRecorder is saving topic: \"" << get_topic_name() << "\" with datatype: " << __get_datatype() << std::endl;

        first_ = false;
      }
          
      uint8_t* sz = read_ptr + __serialized_length;

      sz = log_recorder_->log(get_topic_name(), __get_datatype(), __get_md5sum(), read_ptr, __serialized_length);

      if (msg_)
      {
        msg_->__serialized_length = __serialized_length;
        msg_->deserialize(read_ptr);
      }

      return sz;
    }
  };

  std::ofstream log_file_;

  ros::thread::mutex log_mutex_;

  ros::Time start_time_;

  std::vector<LogHelper*> topics_;

  bool started_;

public:

  LogRecorder(ros::node* node) : started_(false), node_(node) {}

  virtual ~LogRecorder()
  {
    for (std::vector<LogHelper*>::iterator topic_it = topics_.begin(); 
         topic_it != topics_.end(); 
         topic_it++)
    {
      if (*topic_it)
        delete *topic_it;
    }

    log_file_.close();
  }

  bool open(const std::string &file_name, ros::Time start_time)
  {
    start_time_ = start_time;

    log_file_.open(file_name.c_str());
    if (log_file_.fail())
    {
      std::cerr << "Failed to open log file: " << file_name <<  std::endl;
      return false;
    }

    log_file_ << "#ROSLOG V1.1" << std::endl;

    return true;
  }

  template <class M>
  LogHelper*  addTopic_(std::string topic_name, int max_queue)
  {
    if (!started_)
    {

      std::string datatype = M::__s_get_datatype();
      std::string md5sum   = M::__s_get_md5sum();

      LogHelper* l = new LogHelper(this, topic_name, datatype, md5sum, max_queue);

      topics_.push_back(l);

      return l;

    } else {
      std::cerr << "Tried to add topic \"" << topic_name << "\" after logging started." << std::endl;
      return NULL;
    }
  }

  template <class M>
  void addTopic(std::string topic_name, int max_queue)
  {
    addTopic_<M>(topic_name, max_queue);
  }

  template <class M>
  void addTopic(std::string topic_name, int max_queue, void (*fp)(std::string, ros::msg*, ros::Time, void*), void* ptr = NULL, bool inflate = false)
  {
    LogHelper* l = addTopic_<M>(topic_name, max_queue);

    if (!l)
      return;
    
    if (fp != NULL)
    {
      AbstractLogFunctor* lf = new LogFunctor<M>(fp, ptr, inflate);
      l->addHandler(lf);
    }
  }

  template <class M>
  void addTopic(std::string topic_name, int max_queue, void (*fp)(std::string, M*, ros::Time, void*), void* ptr = NULL)
  {
    LogHelper* l = addTopic_<M>(topic_name, max_queue);

    if (!l)
      return;
    
    if (fp != NULL)
    {
      AbstractLogFunctor* lf = new LogFunctor<M>(fp, ptr);
      l->addHandler(lf);
    }
  }

  void start() {
    for (std::vector<LogHelper*>::iterator topic_it = topics_.begin();
         topic_it != topics_.end();
         topic_it++)
      node_->subscribe((*topic_it)->get_topic_name(), *(*topic_it), &LogRecorder::loggerCb, this, (*topic_it), (*topic_it)->get_max_queue());
  }

  void loggerCb(void* log_helper)
  {
    ((LogHelper*)(log_helper))->callHandler();
  }


protected:

  ros::node* node_;

  uint8_t* log(std::string topic_name, std::string datatype, std::string md5sum, uint8_t *read_ptr, uint32_t length)
  {
    ros::Duration elapsed = ros::Time::now() - start_time_;
    log_mutex_.lock();

    log_file_ << topic_name << std::endl;
    log_file_ << md5sum << std::endl;
    log_file_ << datatype << std::endl;

    log_file_.write((char*)&elapsed.sec, 4);
    log_file_.write((char*)&elapsed.nsec, 4);

    log_file_.write((char*)&(length), 4);

    log_file_.write((char*)read_ptr, length);
    
    log_mutex_.unlock();

    return read_ptr + length;
  }

};

#endif
