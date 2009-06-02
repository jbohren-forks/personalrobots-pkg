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
