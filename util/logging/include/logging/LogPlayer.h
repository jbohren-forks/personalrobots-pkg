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
#include "ros/time.h"
#include "logging/AnyMsg.h"
#include "logging/LogFunctor.h"
#include <fstream>
#include <sstream>
#include <cstdio>

class LogPlayer
{
  struct FilteredLogFunctor
  {
    std::string topic_name;
    std::string md5sum;
    std::string datatype;
    bool inflate;
    AbstractLogFunctor* f;
  };


  class LogHelper : public ros::msg
  {
    LogPlayer* log_player_;

    std::string topic_name_;
    std::string md5sum_;
    std::string datatype_;

    ros::msg* msg_;
    uint8_t* next_msg_;
    uint32_t next_msg_size_;

  public:

    LogHelper(LogPlayer* log_player, std::string topic_name, std::string md5sum, std::string datatype) 
      : log_player_(log_player), topic_name_(topic_name),  md5sum_(md5sum), datatype_(datatype),
        msg_(NULL), next_msg_(NULL), next_msg_size_(0) {}

    virtual ~LogHelper()
    {
      if (msg_)
        delete msg_;
    }

    void callHandlers()
    {

      next_msg_ = log_player_->next_msg_;
      next_msg_size_ = log_player_->next_msg_size_;

      __serialized_length = next_msg_size_;

      if (msg_)
      {
        msg_->__serialized_length = next_msg_size_;
        msg_->deserialize(next_msg_);
      }

      for (std::vector<FilteredLogFunctor>::iterator flf_it = log_player_->callbacks_.begin();
           flf_it != log_player_->callbacks_.end();
           flf_it++)
      {

        if (topic_name_  == flf_it->topic_name || 
            flf_it->topic_name == std::string("*"))
        {

          if (flf_it->md5sum != md5sum_ &&
              md5sum_ != std::string("*"))
            break;
          
          if (flf_it->datatype != datatype_ && 
              flf_it->datatype != std::string("*") && 
              datatype_ != std::string("*"))
            break;
          
          if (flf_it->inflate && msg_ == NULL)
          {
            msg_ = flf_it->f->allocateMsg();

            if (msg_)
            {
              msg_->__serialized_length = next_msg_size_;
              msg_->deserialize(next_msg_);
            }
          }
          
          if (flf_it->inflate)
            flf_it->f->call(topic_name_, msg_, log_player_->next_msg_time_);
          else
            flf_it->f->call(topic_name_, this, log_player_->next_msg_time_);
        }
      }
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

  int version_;

  ros::Time start_time_;

  std::map<std::string, LogHelper*> topics_;

  std::string next_msg_name_;

  ros::Time next_msg_time_;

  bool done_;

public:

  LogPlayer() : version_(0),  done_(false), next_msg_(NULL), next_msg_size_(0), next_msg_alloc_size_(0) {}

  virtual ~LogPlayer()
  {
    for (std::vector<FilteredLogFunctor>::iterator flf_it = callbacks_.begin();
         flf_it != callbacks_.end();
         flf_it++)
      if (flf_it->f)
        delete (flf_it->f);

    close();
  }

  bool isDone() {return done_;}

  void close() {
    log_file_.close();

    for (std::map<std::string, LogHelper*>::iterator topic_it = topics_.begin(); 
         topic_it != topics_.end(); 
         topic_it++)
    {
      if (topic_it->second)
        delete topic_it->second;
    }

    topics_.clear();
    done_ = false;
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

    char logtypename[100];
    int version_major = 0;
    int version_minor = 0;

    std::string version_line;
    getline(log_file_, version_line);

    sscanf(version_line.c_str(), "#ROS%s V%d.%d", logtypename, &version_major, &version_minor);

    if (version_major == 0 && version_line[0] == '#')
    {
      version_major = 1;
    }

    version_ = version_major * 100 + version_minor;

    int quantity = 0;

    if (version_ == 0)
    {
      std::cout << "Opening ROSLOG version 0" << std::endl;

      log_file_.seekg(0, std::ios_base::beg);

      quantity = 1;

    }
    else if (version_ == 100)
    {
      std::cout << "Opening ROSLOG version 1.0" << std::endl;

      std::string quantity_line;
      getline(log_file_,quantity_line);
      sscanf(quantity_line.c_str(), "%d", &quantity);
    }
    else if (version_ == 101)
    {
      std::cout << "Opening ROSLOG version 1.1" << std::endl;
    }

    if (version_ == 0 || version_ == 100)
    {
      //      std::cout << "Reading " << quantity << " topics from " << file_name << std::endl;

      std::string topic_name;
      std::string md5sum;
      std::string datatype;

      for (int i = 0; i < quantity; i++)
      {
        getline(log_file_,topic_name);
        getline(log_file_,md5sum);
        getline(log_file_,datatype);
        
        //        std::cout << " (" << i << "): " << topic_name << ", " << md5sum << ", " << datatype << std::endl;
        
        LogHelper* l = new LogHelper(this, topic_name, md5sum, datatype);
        
        topics_[topic_name] = l;
      }
    }

    readNextMsg();
    
    return true;
  }

  template <class M>
  void addHandler(std::string topic_name, void (*fp)(std::string, ros::msg*, ros::Time, void*), void* ptr, bool inflate)
  {
    FilteredLogFunctor flf;

    flf.topic_name = topic_name;
    flf.md5sum   = M::__s_get_md5sum();
    flf.datatype = M::__s_get_datatype();
    flf.inflate = inflate;
    flf.f = new LogFunctor<M>(fp, ptr, inflate);

    callbacks_.push_back(flf);
  }

  template <class M>
  void addHandler(std::string topic_name, void (*fp)(std::string, M*, ros::Time, void*), void* ptr)
  {
    FilteredLogFunctor flf;

    flf.topic_name = topic_name;
    flf.md5sum   = M::__s_get_md5sum();
    flf.datatype = M::__s_get_datatype();
    flf.inflate = true;
    flf.f = new LogFunctor<M>(fp, ptr);

    callbacks_.push_back(flf);
  }

  template <class M, class T>
  void addHandler(std::string topic_name, void (T::*fp)(std::string, ros::msg*, ros::Time, void*), T* obj, void* ptr, bool inflate)
  {
    FilteredLogFunctor flf;

    flf.topic_name = topic_name;
    flf.md5sum   = M::__s_get_md5sum();
    flf.datatype = M::__s_get_datatype();
    flf.inflate = inflate;
    flf.f = new LogFunctor<M, T>(obj, fp, ptr, inflate);

    callbacks_.push_back(flf);
  }

  template <class M, class T>
  void addHandler(std::string topic_name, void (T::*fp)(std::string, M*, ros::Time, void*), T* obj, void* ptr)
  {
    FilteredLogFunctor flf;

    flf.topic_name = topic_name;
    flf.md5sum   = M::__s_get_md5sum();
    flf.datatype = M::__s_get_datatype();
    flf.inflate = true;
    flf.f = new LogFunctor<M, T>(obj, fp, ptr);

    callbacks_.push_back(flf);
  }
    
  ros::Time get_next_msg_time()
  {
    if (!done_)
      return next_msg_time_;
    else 
      return ros::Time(0.0);
  }

  bool nextMsg()
  {
    if (done_)
      return false;
    
    if (topics_.find(next_msg_name_) != topics_.end())
    {
      topics_[next_msg_name_]->callHandlers();
    }
    
    readNextMsg();
    
    return true;
  }
  
protected:

  uint8_t *next_msg_;
  uint32_t next_msg_size_, next_msg_alloc_size_;
  
  std::vector<FilteredLogFunctor> callbacks_;
  
  bool readNextMsg()
  {
    static ros::Duration first_duration(0,0);

    if (!log_file_.good())
    {
      done_ = true;
      return false;
    }
    
    if (version_ <= 100)
    {
      if (version_ == 0)
        next_msg_name_ = (topics_.begin())->first;
      else
        getline(log_file_, next_msg_name_);
      
      ros::Duration next_msg_dur;


    } else {
      
      std::string topic_name;
      std::string md5sum;
      std::string datatype;
      
      getline(log_file_,topic_name);
      getline(log_file_,md5sum);
      getline(log_file_,datatype);

      next_msg_name_ = topic_name;
      
      if (topics_.find(topic_name) == topics_.end())
      {
        LogHelper* l = new LogHelper(this, topic_name, md5sum, datatype);
        topics_[topic_name] = l;
      }
      
    }

    if (log_file_.eof())
    {
      done_ = true;
      return false;
    }

    ros::Duration next_msg_dur;

    log_file_.read((char*)&next_msg_dur.sec, 4);
    log_file_.read((char*)&next_msg_dur.nsec, 4);
    log_file_.read((char*)&next_msg_size_, 4);

    if (log_file_.eof())
    {
      done_ = true;
      return false;
    }

    if(first_duration == ros::Duration(0,0))
      first_duration = next_msg_dur;
      
    next_msg_time_ = start_time_ + (next_msg_dur - first_duration);
      
      
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

class MultiLogPlayer
{
  std::vector<LogPlayer*> players_;

public:

  MultiLogPlayer() { }

  ~MultiLogPlayer()
  {
    for (std::vector<LogPlayer*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      if (*player_it)
        delete (*player_it);
    }
  }

  bool open(std::vector<std::string> file_names, ros::Time start)
  {
    for (std::vector<std::string>::iterator name_it = file_names.begin();
         name_it != file_names.end();
         name_it++)
    {
      LogPlayer* l = new LogPlayer;

      if (l->open(*name_it, start))
        players_.push_back(l);
      else
      {
        delete l;
        return false;
      }
    }

    return true;
  }

  bool nextMsg()
  {
    LogPlayer* next_player = 0;

    ros::Time min_t = ros::Time((uint64_t)-1); // This should be the maximum unsigned int;

    bool remaining = false;

    for (std::vector<LogPlayer*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      if ((*player_it)->isDone())
      {
        continue;
      }
      else
      {
        remaining = true;
        ros::Time t = (*player_it)->get_next_msg_time();
        if (t < min_t)
        {
          next_player = (*player_it);
          min_t = (*player_it)->get_next_msg_time();
        }
      }
    }

    if (next_player)
      next_player->nextMsg();
    
    return remaining;
  }

  template <class M>
  void addHandler(std::string topic_name, void (*fp)(std::string, ros::msg*, ros::Time, void*), void* ptr, bool inflate)
  {
    for (std::vector<LogPlayer*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      (*player_it)->addHandler<M>(topic_name, fp, ptr, inflate);
    }
  }

  template <class M>
  void addHandler(std::string topic_name, void (*fp)(std::string, M*, ros::Time, void*), void* ptr)
  {
    for (std::vector<LogPlayer*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      (*player_it)->addHandler<M>(topic_name, fp, ptr);
    }
  }

  template <class M, class T>
  void addHandler(std::string topic_name, void (T::*fp)(std::string, ros::msg*, ros::Time, void*), T* obj, void* ptr, bool inflate)
  {
    for (std::vector<LogPlayer*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      (*player_it)->addHandler<M>(topic_name, fp, obj, ptr, inflate);
    }
  }

  template <class M, class T>
  void addHandler(std::string topic_name, void (T::*fp)(std::string, M*, ros::Time, void*), T* obj, void* ptr)
  {
    for (std::vector<LogPlayer*>::iterator player_it = players_.begin();
         player_it != players_.end();
         player_it++)
    {
      (*player_it)->addHandler<M>(topic_name, fp, obj, ptr);
    }
  }
};

#endif

