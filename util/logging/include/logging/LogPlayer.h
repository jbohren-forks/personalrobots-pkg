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

#ifndef LOGPLAYER_H
#define LOGPLAYER_H

#include "ros/node.h"
#include "logging/AnyMsg.h"
#include <fstream>
#include <sstream>

class LogPlayer
{
 
  class LogHelper : public ros::msg
  {
    typedef std::pair<void (*)(std::string, ros::msg*, ros::Time, void*), void*> Callback;

    std::string topic_name_;
    std::string datatype_;
    std::string md5sum_;

    std::vector<Callback> callbacks_;

    ros::msg* msg_;
    uint8_t* next_msg_;
    uint32_t next_msg_size_;

  public:

    LogHelper(std::string topic_name, std::string datatype, std::string md5sum) 
      : topic_name_(topic_name), datatype_(datatype), md5sum_(md5sum), 
        msg_(NULL), next_msg_(NULL), next_msg_size_(0) {}

    virtual ~LogHelper()
    {
      if (msg_)
        delete msg_;
    }

    void addHandler(ros::msg* msg, void (*fp)(std::string, ros::msg*, ros::Time, void*), void* ptr)
    {
      if (msg)
      {
        if (msg_)
          delete msg_;
        msg_ = msg;
      }

      callbacks_.push_back(Callback(fp, ptr));
    }

    void callHandlers(uint8_t* next_msg, uint32_t next_msg_size, ros::Time time)
    {
      next_msg_ = next_msg;
      next_msg_size_ = next_msg_size;

      __serialized_length = next_msg_size_;

      ros::msg* msg = this;

      if (msg_)
      {
        msg_->__serialized_length = next_msg_size_;
        msg_->deserialize(next_msg_);
        msg = msg_;
      }

      for (std::vector<Callback>::iterator cb_it = callbacks_.begin();
           cb_it != callbacks_.end();
           cb_it++)
        (*(cb_it->first))(topic_name_, msg, time, (cb_it->second));
    }

    std::string get_topic_name() {return topic_name_;}

    virtual const std::string __get_datatype() const { return datatype_; }

    virtual const std::string __get_md5sum()   const { return md5sum_; }

    virtual uint8_t *deserialize(uint8_t *read_ptr) { assert(0); return NULL; }

    virtual uint32_t serialization_length()    { return __serialized_length; }

    virtual uint8_t *serialize(uint8_t *write_ptr)
    { 
      assert(next_msg_);
      memcpy(write_ptr, next_msg_, next_msg_size_);
      return write_ptr + next_msg_size_;
    }
  };

  std::ifstream log_file_;

  bool old_format_;

  ros::Time start_time_;

  std::map<std::string, LogHelper*> topics_;

  std::string next_msg_name_;
  uint8_t *next_msg_;
  uint32_t next_msg_size_, next_msg_alloc_size_;
  ros::Time next_msg_time_;

  bool done_;

public:

  LogPlayer() : old_format_(false), next_msg_(NULL), next_msg_size_(0), next_msg_alloc_size_(0), done_(false) {}

  virtual ~LogPlayer()
  {
    for (std::map<std::string, LogHelper*>::iterator topic_it = topics_.begin(); 
         topic_it != topics_.end(); 
         topic_it++)
    {
      if (topic_it->second)
        delete topic_it->second;
    }

    log_file_.close();
  }

  bool isDone() {return done_;}

  bool open(const std::string &file_name, ros::Time start_time)
  {
    start_time_ = start_time;

    log_file_.open(file_name.c_str());

    if (log_file_.fail())
    {
      std::cerr << "Failed to open log file: " << file_name <<  std::endl;
      return false;
    }

    std::string comment_line;
    getline(log_file_, comment_line);

    if (comment_line[0] != '#')
      log_file_.seekg(0, std::ios_base::beg);
    else
      std::cout << "Read comment line: " << comment_line << std::endl;

    std::string quantity_line;
    getline(log_file_,quantity_line);

    std::istringstream iss(quantity_line);

    int quantity = -1;
    iss >> quantity;
    
    std::cout << "Read quantity: " << quantity << std::endl;

    if (quantity == -1)
    {
      old_format_ = true;
      quantity = 1;
      log_file_.seekg(-(int)(quantity_line.size() + 1), std::ios_base::cur);
    }

    std::string topic_name;
    std::string md5sum;
    std::string datatype;

    for (int i = 0; i < quantity; i++)
    {
      getline(log_file_,topic_name);
      getline(log_file_,md5sum);
      getline(log_file_,datatype);

      std::cout << "Log " << i << ": " << topic_name << ", " << md5sum << ", " << datatype << std::endl;

      LogHelper* l = new LogHelper(topic_name, md5sum, datatype);

      topics_[topic_name] = l;
    }

    readNextMsg();
    
    return true;
  }


  std::vector<std::string> getNames() {
    std::vector<std::string> names;
    for (std::map<std::string, LogHelper*>::iterator topic_it = topics_.begin(); 
         topic_it != topics_.end();
         topic_it++)
      names.push_back(topic_it->first);

    return names;
  }

  template <class M>
  bool addHandler(std::string topic_name, void (*fp)(std::string, ros::msg*, ros::Time, void*), void* ptr, bool inflate)
  {

    for (std::map<std::string, LogHelper*>::iterator topic_it = topics_.begin();
         topic_it != topics_.end();
         topic_it++)
    {
      if (topic_name == std::string("*") ||
          topic_name == (topic_it->second)->get_topic_name())
      {

        if (M::__s_get_md5sum() != (topic_it->second)->__get_md5sum() &&
            (topic_it->second)->__get_md5sum() != std::string("*"))
          return false;

        if (M::__s_get_datatype() != (topic_it->second)->__get_datatype() && 
            M::__s_get_datatype() != std::string("*") && (topic_it->second)->__get_datatype() != std::string("*"))
          return false;
        
        M* msg = NULL;

        if (inflate)
          msg = new M;

        (topic_it->second)->addHandler(msg, fp, ptr);
      }
    }
    return true;
  }


  ros::Time get_next_msg_time()
  {
    if (!done_)
      return next_msg_time_;
    else 
      return ros::Time(0);
  }

  bool nextMsg()
  {
    if (done_)
      return false;

    if (topics_.find(next_msg_name_) != topics_.end())
      topics_[next_msg_name_]->callHandlers(next_msg_, next_msg_size_, next_msg_time_);

    readNextMsg();

    return true;
  }

protected:

  bool readNextMsg()
  {
    if (!log_file_.good())
    {
      done_ = true;
      return false;
    }

    if (old_format_)
    {
      next_msg_name_ = (topics_.begin())->first;
    } else {
      getline(log_file_, next_msg_name_);
    }

    ros::Duration next_msg_dur;
      
    log_file_.read((char*)&next_msg_dur.sec, 4);
    log_file_.read((char*)&next_msg_dur.nsec, 4);
    log_file_.read((char*)&next_msg_size_, 4);

    next_msg_time_ = start_time_ + next_msg_dur;

    if (log_file_.eof())
    {
      done_ = true;
      return false;
    }

    if (next_msg_size_ > next_msg_alloc_size_)
    {
      if (next_msg_)
        delete[] next_msg_;
      next_msg_alloc_size_ = next_msg_size_ * 2;
      next_msg_ = new uint8_t[next_msg_alloc_size_];
    }

    log_file_.read((char*)next_msg_, next_msg_size_);

    if (log_file_.eof())
    {
      done_ = true;
      return false;
    }

    return true;
  }

};

#endif
