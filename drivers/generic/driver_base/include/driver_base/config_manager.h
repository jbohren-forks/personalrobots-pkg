/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef __CONFIG_MANAGER_H__
#define __CONFIG_MANAGER_H__

#include "ros/common.h"

namespace driver_base
{

enum
{
  OK,
  ERROR
};

class ConfigManager
{
private:
  class OptionBase
  {
    unsigned int level;
  public:
  };

  template <class OT>
  class Option : public OptionBase
  {
    T &value;
  };

  std::map<std::string,OptionBase> options_;
  std::auto_ptr<ros::AbstractFunctor> reconfigCallback_;
  ros::Node &node_;
  boost::mutex &mainlock_;

  bool reconfigureSrv(Reconfigure::Request &req, Reconfigure::Response &res);
  bool setConfigSrv(SetConfig::Request &req, SetConfig::Response &res);

public:
  template <class T> 
  void declareOption(std::string &name, T &value, T &defaultvalue, unsigned int level = 0);
  
  setReconfigCallback(AbstractFunctor *f);

  template <class T>
  setReconfigCallback(void(T::*f)(), T *obj)
  {
    setReconfigCallback(new MethodFunctor<T>(obj, f));
  }

  template <class T>
  setReconfigCallback(void(T::*f)(void *user_data), T *obj, void *user_data)
  {
    setReconfigCallback(new MethodFunctor<T>(obj, f, user_data));
  }

  template <class T>
  setReconfigCallback(void(*f)())
  {
    setReconfigCallback(new FunctionFunctor<T>(f));
  }

  template <class T>
  setReconfigCallback(void(*f)(void *), void *user_data)
  {
    setReconfigCallback(new FunctionFunctor<T>(f, user_data));
  }

  void triggerReconfig();
  ConfigManager(ros::node &n);
};

// These methods have been separated out to ease making this into a library
// later.

ConfigManager::ConfigManager(ros::node &n) :
  node_(n)
{
  n.advertiseService("~reconfigure", &reconfigureSrv);
  n.advertiseService("~set_configuration", &setConfigSrv);
}

void ConfigManager::triggerReconfig()  
{
  boost::mutex::scoped_lock(mainlock_)
  
  if (reconfigCallback_)
    reconfigCallback_();
}
  
void ConfigManager::setReconfigCallback(AbstractFunctor *f)
{
  boost::mutex::scoped_lock(mainlock_);
  
  reconfigCallback_ = f;
}
  
bool ConfigManager::reconfigureSrv(Reconfigure::Request &req, Reconfigure::Response &res)
{
  res = OK;
  return true;
}

bool ConfigManager::setConfigSrv(SetConfig::Request &req, SetConfig::Response &res)
{
  res = OK;
  return true;
}
}
#endif
