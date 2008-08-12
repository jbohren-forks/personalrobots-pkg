#include "rosTF/rosTF.h"

class FrameServer : public ros::node
{
public:
  //constructor
  FrameServer() : ros::node("frameServer"){
    pTFServer = new rosTFServer(*this);
  };
  //Clean up ros connections
  ~FrameServer() { 
      if (pTFServer)
	  delete pTFServer;
  }

  //A pointer to the rosTFServer class
  rosTFServer * pTFServer;


  // A function to call to send data periodically
  void loop () {
    pTFServer->sendEuler("FRAMEID_SMALLV", "FRAMEID_STEREO_BLOCK", 0.0, 0.0, 0.0, -M_PI/2, 0.0, -M_PI/2, ros::Time::now());
    pTFServer->sendEuler("base", "FRAMEID_STEREO_BLOCK",0,0,0,0,0,0,ros::Time::now());

    NEWMAT::Real trnsele[] = { 0.08830,  0.00158,  0.99609, 2.51721e-3, //
			       -0.00580,  0.99998, -0.00107, 100.66109e-3, //
			       -0.99608, -0.00568,  0.08831, 48.23300e-3, //  
			       0.0, 0.0, 0.0, 1.0}; 

    NEWMAT::Matrix trns(4,4);
    trns << trnsele;
    pTFServer->sendMatrix("FRAMEID_STEREO_BLOCK","FRAMEID_TILT_BASE",  trns.i(), ros::Time::now());
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

