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

class LogPlayerBase
{
public:
  bool done;

  LogPlayerBase() : done(false) {}

  virtual bool open_log(const std::string &log_file, ros::Time _start, bool _deserialize) = 0;
  virtual ros::Time time_of_msg() = 0;
  virtual bool read_next_msg(bool deserialize = false) = 0;
};

template <class M = AnyMsg>
class LogPlayer : public LogPlayerBase, public M
{
public:
  std::ifstream log;
  ros::Time start;
  ros::Duration next_msg_dur;
  std::string topic_name;

  uint8_t *next_msg;
  uint32_t next_msg_size, next_msg_alloc_size;

  LogPlayer() : M(), next_msg(NULL), next_msg_alloc_size(0) { }

  virtual ~LogPlayer() { log.close(); if (next_msg) delete[] next_msg; }

  virtual bool open_log(const std::string &log_file, ros::Time _start, bool deserialize=false)
  {
    start = _start;
    log.open(log_file.c_str());
    if (log.fail())
    {
      done = true;
      return false;
    }
    getline(log,topic_name);

    std::string md5sum;
    std::string datatype;
    getline(log,md5sum);
    getline(log,datatype);

    if (M::__get_md5sum() != md5sum && 
        md5sum != std::string("*"))
    {
      return false;
    }

    if (M::__get_datatype() != datatype && 
        M::__get_datatype() != std::string("*") && datatype != std::string("*"))
    {
      return false;
    }

    
    if (!read_next_msg(deserialize))
      return false;
    return true;
  }

  virtual ros::Time time_of_msg()
  {
    if (done)
      return ros::Time(0);

    return start + next_msg_dur;
  }

  virtual bool read_next_msg(bool deserialize = false)
  {
    if (!log.good())
    {
      done = true;
      return false;
    }

    log.read((char*)&next_msg_dur.sec, 4);
    log.read((char*)&next_msg_dur.nsec, 4);
    log.read((char*)&next_msg_size, 4);
    if (log.eof())
    {
      done = true;
      return false;
    }

    if (next_msg_size > next_msg_alloc_size)
    {
      if (next_msg)
        delete[] next_msg;
      next_msg_alloc_size = next_msg_size * 2;
      next_msg = new uint8_t[next_msg_alloc_size];
    }

    log.read((char*)next_msg, next_msg_size);

    if (deserialize)
    {
      M::deserialize(next_msg);
    }

    if (log.eof())
    {
      done = true;
      return false;
    }
    return true;
  }

  virtual std::string __get_topic_name() { return topic_name; }

  uint32_t serialization_length() { return next_msg_size; }

  virtual uint8_t *serialize(uint8_t *write_ptr)
  { 
    memcpy(write_ptr, next_msg, next_msg_size);
    return write_ptr + next_msg_size;
  }
};


#endif
