#include "rosTF/rosTF.h"

class testServer : public ros::node
{
public:
  //constructor
  testServer() : ros::node("server"),count(2){
    pTFServer = new rosTFServer(*this);
  };

  //A pointer to the rosTFServer class
  rosTFServer * pTFServer;


  // A function to call to send data periodically
  void test () {
    pTFServer->sendEuler(5,count++,1,1,1,1,1,1,100000,100000);
    pTFServer->sendDH(5,count++,1,1,1,1,100000,100000);
    pTFServer->sendQuaternion(5,count++,1,1,1,1,1,1,1,100000,100000);
  };

private:
  int count;

};

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv);

  //Construct/initialize the server
  testServer myTestServer;
  
  while(myTestServer.ok())
    {
      //Send some data
      myTestServer.test();
      sleep(1);
    }

  return 0;
};

