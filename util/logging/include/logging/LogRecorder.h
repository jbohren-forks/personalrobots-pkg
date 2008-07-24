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
#include "logging/AnyMsg.h"

#include <fstream>
#include <iostream>
#include <iomanip>


class LogRecorder
{

  class LogHelper : public ros::msg
  {

    typedef std::pair<void (*)(std::string, ros::msg*, void*), void*> Callback;

    LogRecorder* log_recorder_;

    std::string topic_name_;
    std::string datatype_;
    std::string md5sum_;

    Callback* callback_;

    ros::msg* msg_;

  public:

    LogHelper(LogRecorder* log_recorder, std::string topic_name, std::string datatype, std::string md5sum)
      : log_recorder_(log_recorder), 
        topic_name_(topic_name), datatype_(datatype), md5sum_(md5sum),
        callback_(NULL),
        msg_(NULL) {}

    virtual ~LogHelper()
    {
      if (msg_)
        delete msg_;

      if (callback_)
        delete callback_;
    }


    void addHandler(ros::msg* msg, void (*fp)(std::string, ros::msg*, void*), void* ptr)
    {
      if (msg)
      {
        if (msg_)
          delete msg_;
        msg_ = msg;
      }

      callback_ = new Callback(fp, ptr);
    }

    void callHandler()
    {
      ros::msg* msg = this;

      if (msg_)
        msg = msg_;

      if (callback_)
        (*(callback_->first))(topic_name_, msg, (callback_->second));
    }


    std::string get_topic_name() {return topic_name_;}

    virtual const std::string __get_datatype() const { return datatype_; }

    virtual const std::string __get_md5sum()   const { return md5sum_; }

     virtual uint32_t serialization_length()    { return __serialized_length; }

    virtual uint8_t *serialize(uint8_t *write_ptr) { assert(0); return NULL; }

    virtual uint8_t *deserialize(uint8_t *read_ptr)
    {
      uint8_t* sz = log_recorder_->log(get_topic_name(), read_ptr, __serialized_length);

      if (msg_)
      {
        msg_->__serialized_length = __serialized_length;
        msg_->deserialize(read_ptr);
      }

      return sz;
    }
  };

  ros::node* node_;

  std::ofstream log_file_;

  ros::thread::mutex log_mutex_;

  ros::Time start_time_;

  std::vector<LogHelper*> topics_;

public:

  LogRecorder(ros::node* node) : node_(node) {}

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

    log_file_ << "#ROSLOG V1.0" << std::endl << std::setw(4) << std::setfill('0') << 0 << std::endl;

    return true;
  }


  template <class M>
  void addTopic(std::string topic_name, void (*fp)(std::string, ros::msg*, void*) = NULL, void* ptr = NULL, bool inflate = false)
  {
    LogHelper* l = new LogHelper(this, topic_name, M::__s_get_datatype(), M::__s_get_md5sum());
    topics_.push_back(l);

    log_mutex_.lock();
    
    log_file_.seekp(0, std::ios_base::beg);
    log_file_ << "#ROSLOG V1.0" << std::endl << std::setw(4) << std::setfill('0') << topics_.size() << std::endl;
    log_file_.seekp(0, std::ios_base::end);

    log_file_ << topic_name << std::endl;
    log_file_ << l->__get_datatype() << std::endl;
    log_file_ << l->__get_md5sum() << std::endl;

    log_mutex_.unlock();
    
    if (fp != NULL)
    {
      M* msg = NULL;

      if (inflate)
        msg = new M;

      l->addHandler(msg, fp, ptr);
    }

  }

  void start() {
    for (std::vector<LogHelper*>::iterator topic_it = topics_.begin();
         topic_it != topics_.end();
         topic_it++)
      node_->subscribe((*topic_it)->get_topic_name(), *(*topic_it), &LogRecorder::dummyCb, this, (*topic_it), 100);
  }

  void dummyCb(void* log_helper)
  {
    ((LogHelper*)(log_helper))->callHandler();
  }

protected:
  uint8_t* log(std::string topic_name, uint8_t *read_ptr, uint32_t length)
  {
    ros::Duration elapsed = ros::Time::now() - start_time_;
    log_mutex_.lock();

    log_file_ << topic_name << std::endl;

    log_file_.write((char*)&elapsed.sec, 4);
    log_file_.write((char*)&elapsed.nsec, 4);

    log_file_.write((char*)&(length), 4);

    log_file_.write((char*)read_ptr, length);
    
    log_mutex_.unlock();

    return read_ptr + length;
  }

};

#endif
