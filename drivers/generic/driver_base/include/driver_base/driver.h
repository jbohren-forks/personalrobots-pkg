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

// Author: Blaise Gassend
#ifndef __DRIVER_BASE__DRIVER_H__
#define __DRIVER_BASE__DRIVER_H__

#include <diagnostic_updater/diagnostic_updater.h>
#include <self_test/self_test.h>

namespace driver_base
{

{
  static const 
public:      
  report_error();
};


class Driver
{
public:
  void Driver()
  {                   

    prepare_diagnostics();
    prepare_self_tests();
    read_config();
  }
  
  void spin()
  {
    while (node_handle_.ok())
    {
      if (parameters_.autostart && !started())
      {
        go_running();
      }

      /// Will need some locking here or in diagnostic_updater?
      diagnostic_.update();
      self_test_.checkTest();
    }


  }
  
  void go_running()
  {
    if (dev->getState() != Device::RUNNING)
      go_opened();

    if (dev->getState() == Device::OPENED)
      dev->start();
  }

  void go_opened()
  {
    if (dev->getState() == Device::RUNNING)
      dev->stop();
    else if (dev->getState != Device::OPENED)
    {
      go_closed();

      if (dev->getState == Device::CLOSED)
        go_opened();
    }
  }

  void go_closed()
  {
    if (dev->getState() == Device::RUNNING)
      dev->stop();
  
    if (dev->getState() != Device::CLOSED)
      dev->close(); 
  }

  int main(std::string name, int argc, char **argv)
  {
    ros::init(argc, argv, name, ros::init_options::NoSigIntHandler, Device *dev);
    signal(SIGINT, Driver::sigCalled);
    signal(SIGTERM, Driver::sigCalled);
    Driver drv(dev);
    ros::spin();
    return drv.getExitStatus();
  }

private:
  static int ctrl_c_hit_count_;
  Device &dev_;
  diagnostic_updater::Updater diagnostic_;
  self_test::Dispatcher self_test_;
};

static int Driver::ctrl_c_hit_count_ = 0;
  
// @todo exit status.

};

#endif
