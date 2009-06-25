#include <ros/node.h>
#include <std_msgs/Empty.h>
#include <sys/time.h>

namespace TREX{

  class Watchdog: public ros::Node {
  public:
    Watchdog(): ros::Node("trex_watch_dog"), watchdogOK_(true), watchdogTimeLimit_(1.0){
      param("trex/ping_frequency", watchdogTimeLimit_, watchdogTimeLimit_);
      subscribe<std_msgs::Empty>("trex/ping", pingMsg_, &Watchdog::pingCallback, 1);
      advertise<std_msgs::Empty>("robot_actions/shutdown", 1);
    }

    void run(){
      while(true){
	if(!watchdogOK_){
	  std_msgs::Empty shutdownMsg;
	  publish("robot_actions/shutdown", shutdownMsg);
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
    std_msgs::Empty pingMsg_;
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

  

  return(0);
}
