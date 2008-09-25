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

#ifndef LOGFUNCTOR_H
#define LOGFUNCTOR_H

#include "ros/time.h"
#include <string>

class AbstractLogFunctor
{  
public:
  virtual void call(std::string, ros::msg*, ros::Time) = 0;
  virtual ros::msg* allocateMsg() = 0;
  virtual ~AbstractLogFunctor() {}
};

class EmptyObject
{

};

template<class M, class T = EmptyObject>
class LogFunctor : public AbstractLogFunctor
{
public:
  LogFunctor(void (*fp)(std::string, M*, ros::Time, void*), void* user_data)
    : inflate_(true), obj_(NULL), fp_(NULL), fp_typed_(fp), fp_obj_(NULL), fp_typed_obj_(NULL), user_data_(user_data) { }

  LogFunctor(void (*fp)(std::string, ros::msg*, ros::Time, void*), void* user_data, bool inflate)
    : inflate_(inflate), obj_(NULL), fp_(fp), fp_typed_(NULL), fp_obj_(NULL), fp_typed_obj_(NULL), user_data_(user_data) { }

  LogFunctor(T* obj, void (T::*fp)(std::string, M*, ros::Time, void*), void* user_data)
    : inflate_(true), obj_(obj), fp_(NULL), fp_typed_(NULL), fp_obj_(NULL), fp_typed_obj_(fp), user_data_(user_data) { }

  LogFunctor(T* obj, void (T::*fp)(std::string, ros::msg*, ros::Time, void*), void* user_data, bool inflate)
    : inflate_(inflate), obj_(obj), fp_(NULL), fp_typed_(NULL), fp_obj_(fp), fp_typed_obj_(NULL), user_data_(user_data) { }

  virtual void call(std::string topic_name, ros::msg* msg, ros::Time time)
  {
    if (obj_)
    {
      if (fp_obj_)
        (*obj_.*fp_obj_)(topic_name, msg, time, user_data_);
      else if(fp_typed_obj_)
        (*obj_.*fp_typed_obj_)(topic_name, (M*)msg, time, user_data_);
    } else {
      if (fp_)
        (*fp_)(topic_name, msg, time, user_data_);
      else if (fp_typed_)
        (*fp_typed_)(topic_name, (M*)msg, time, user_data_);
    }
  }

  virtual ros::msg* allocateMsg()
  {
    if (fp_typed_ || fp_typed_obj_)
      return new M;
    else
      return NULL;
  }
private:
  bool inflate_;
  T* obj_;
  void (*fp_)(std::string, ros::msg*, ros::Time, void*);
  void (*fp_typed_)(std::string, M*, ros::Time, void*);
  void (T::*fp_obj_)(std::string, ros::msg*, ros::Time, void*);
  void (T::*fp_typed_obj_)(std::string, M*, ros::Time, void*);
  void *user_data_;
};

#endif
