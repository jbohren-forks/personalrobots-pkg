#error Do not use this file. It is scheduled for deletion.

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
#ifndef __DRIVER_BASE_H__
#define __DRIVER_BASE_H__

namespace driver_base
{
  
enum DeviceState
{
  DEV_UNCONFIGURED = 0,
  DEV_READY = 1,
  DEV_RUNNING = 2,
  DEV_FAILED = 3
};

char *DeviceStateStr[] = 
{
  "DEV_UNCONFIGURED",
  "DEV_READY",
  "DEV_RUNNING",
  "DEV_FAILED"
};

class DeviceBase
{
  protected:
    virtual void handleRun() = 0;
    virtual void handleStop() = 0;
    virtual void handleConfigure() = 0;
    virtual void handleUnconfigure() = 0;

  public:
    void run();
    void stop();
    void configure();
    void unconfigure();

  private:

  DeviceState state_;
  
private:
  virtual int configure()

  virtual int unconfigure();


}

enum DriverState
{
  DRV_UNCONFIGURED = 0,
  DRV_READY = 1,
  DRV_RUNNING = 2,
};

char *DriverStateStr[]
{
  "DRV_UNCONFIGURED",
  "DRV_READY",
  "DRV_RUNNING",
};

class DriverNode
{
  DriverState state_;
  bool self_test_;
  bool failed_;

private:
  int configureCallback(int level)
  {
    if (self_test_)
      return 
  }

  int configure(int level)
  {
    assert(level >= 0 && level < 3);
    
    if (isSelfTesting())
      return false;

    if (isRunning() && level < DRV_RUNNING)
      stop();

    if (isConfigured() && level < DRV_CONFIGURED)
      unconfigure();

    if (level == state_)
    {
      int ret;

      switch (level)
      {
        case DRV_RUNNING:
          ret = handleHotReconfigure();
          break;

        case DRV_CONFIGURED:
          ret = handleWarmReconfigure();
          break;

        case DRV_UNCONFIGURED:
          ret = handleColdReconfigure():
          break;
      }
    }
      reconfigure();

    activate();  
  }

  int stop()
  {
    if (state_ == DRV_RUNNING)
    {
      if (handleStop())
    }
    else
      fail();
  }

  void unconfigure()
  {
    if (state_ == DRV_READY)
      state_ = dev_.unconfigure();
  }

  void activate()
  {
    if (state == DRV_

  }

};

}

#endif
