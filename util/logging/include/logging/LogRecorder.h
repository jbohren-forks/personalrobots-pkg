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
#include <iomanip>

class LogWriter : public std::ofstream
{
  ros::thread::mutex m;
  uint32_t count;

public:

  LogWriter() : std::ofstream() , count(0) { }

  bool openOld(std::string fname, std::string topic_name, std::string data_type, std::string md5sum) 
  {
    std::ofstream::open(fname.c_str());
    if (std::ofstream::fail())
    {
      printf("Failed to open log\n");
      return false;
    }

    *this << topic_name << std::endl;
    *this << data_type  << std::endl;
    *this << md5sum     << std::endl;

    return true;
  }

  bool open(std::string fname) 
  {
    std::ofstream::open(fname.c_str());
    if (std::ofstream::fail())
    {
      fprintf(stderr, "Failed to open log\n");
      return false;
    }

    *this << "##MULTILOG##" << std::endl << std::setw(4) << std::setfill('0') << 0 << std::endl;

    return true;
  }

  void addLog(std::string topic_name, std::string data_type, std::string md5sum)
  {
    seekp(0, ios_base::beg);

    *this << "##MULTILOG##" << std::endl << std::setw(4) << std::setfill('0') << ++count << std::endl;

    seekp(0, ios_base::end);

    *this << topic_name << std::endl;
    *this << data_type  << std::endl;
    *this << md5sum     << std::endl;
  }

  void lock() {
    m.lock();
  }

  void unlock() {
    m.unlock();
  }
};

template <class M = AnyMsg>
class LogRecorder : public M
{

  bool oldFormat;
  LogWriter* log;
  std::string topic_name;
  ros::Time start;
public:

  LogRecorder() : M(), oldFormat(false), log(NULL) { }
  virtual ~LogRecorder()
  {
    if (oldFormat)
      delete log;
  }
  bool open_log(const std::string &file_name, std::string _topic_name, ros::Time _start)
  {
    oldFormat = true;

    topic_name = _topic_name;
    start = _start;
    log = new LogWriter;

    if (!log->openOld(file_name, __get_topic_name(), M::__get_datatype(), M::__get_md5sum()))
      return false;

    return true;
  }

  bool open_log(LogWriter* _log, std::string _topic_name, ros::Time _start)
  {
    log = _log;

    topic_name = _topic_name;
    start = _start;

    log->lock();
    log->addLog(__get_topic_name(), M::__get_datatype(), M::__get_md5sum());
    log->unlock();

    return true;
  }

  virtual std::string __get_topic_name() { return topic_name; }

  virtual uint8_t *serialize(uint8_t *write_ptr) { assert(0); return NULL; }
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::Duration elapsed = ros::Time::now() - start;
    log->lock();

    if (!oldFormat)
      *log << topic_name << std::endl;

    log->write((char*)&elapsed.sec, 4);
    log->write((char*)&elapsed.nsec, 4);

    uint32_t tmp = M::__serialized_length;
    log->write((char*)&(tmp), 4);

    log->write((char*)read_ptr, M::__serialized_length);
    log->unlock();
    return read_ptr + M::__serialized_length;
  }
};


#endif
