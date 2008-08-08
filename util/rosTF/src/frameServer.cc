#include "rosTF/rosTF.h"

class FrameServer : public ros::node
{
public:
  //constructor
  FrameServer() : ros::node("frameServer"){
    pTFServer = new rosTFServer(*this);
  };
  //Clean up ros connections
  ~FrameServer() { }

  //A pointer to the rosTFServer class
  rosTFServer * pTFServer;


  // A function to call to send data periodically
  void loop () {
    pTFServer->sendEuler("FRAMEID_TILT_BASE","base",0,0,0,0,1.57,0,ros::Time::now());
    pTFServer->sendEuler("FRAMEID_STEREO_BLOCK","base",0,0,0,0,0,0,ros::Time::now());
  };

private:

};

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv);

  //Construct/initialize the server
  FrameServer myTestServer;
  
  while(myTestServer.ok())
  {
      //Send some data
      myTestServer.loop();
      usleep(100000);
  }
  ros::fini();

  return 0;
};

