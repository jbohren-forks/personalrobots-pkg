#include <ros/node.h>
#include <highlevel_controllers/Ping.h>
#include <sys/time.h>

namespace TREX{

  class Watchdog: public ros::node {
  public:
    Watchdog(): ros::node("trex_watch_dog"), watchdogOK_(true), watchdogTimeLimit_(1.0){
      param("trex/ping_frequency", watchdogTimeLimit_, watchdogTimeLimit_);
      subscribe<highlevel_controllers::Ping>("trex/ping", pingMsg_, &Watchdog::pingCallback, 1);
      advertise<highlevel_controllers::Ping>("highlevel_controllers/shutdown", 1);
    }

    void run(){
      while(true){
	if(!watchdogOK_){
	  highlevel_controllers::Ping shutdownMsg;
	  publish<highlevel_controllers::Ping>("highlevel_controllers/shutdown", shutdownMsg);
	}

	watchdogOK_ = false;
	usleep((unsigned int) rint(watchdogTimeLimit_ * 1e6));
      }
    }

    /**
     * Callback will reset the watchdog
     */
    void pingCallback(){
      watchdogOK_ = true;
    }

  private:
    bool watchdogOK_;
    double watchdogTimeLimit_;
    highlevel_controllers::Ping pingMsg_;
  };

}

int main(int argc, char** argv)
{
  ros::init(argc,argv); 

  TREX::Watchdog node;

  try {
    node.run();
  }
  catch(char const* e){
    std::cout << e << std::endl;
  }

  ros::fini();

  return(0);
}
