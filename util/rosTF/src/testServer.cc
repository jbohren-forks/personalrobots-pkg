#include "rosTF/rosTF.h"

class testServer : public ros::node
{
public:
  //constructor
  testServer() : ros::node("server"),count(2){
    pTFServer = new rosTFServer(*this);
  };
  //Clean up ros connections
  ~testServer() { }

  //A pointer to the rosTFServer class
  rosTFServer * pTFServer;


  // A function to call to send data periodically
  void test () {
    NEWMAT::Matrix mat(4,4);
    mat << 1 << 0 << 0 << 1
        << 0 << 1 << 0 << 2
        << 0 << 0 << 1 << 3
        << 0 << 0 << 0 << 1;
    
    pTFServer->sendEuler(5,count++,1,1,1,1,1,1,100000,100000);
    pTFServer->sendInverseEuler(5,count++,1,1,1,1,1,1,100000,100000);
    pTFServer->sendDH(5,count++,1,1,1,1,100000,100000);
    pTFServer->sendQuaternion(5,count++,1,1,1,1,1,1,1,100000,100000);
    pTFServer->sendMatrix(5,count++,mat, ros::Time::now());
    if (count > 9000)
      count = 0;
    std::cerr<<count<<std::endl;
    pTFServer->lookup("asdf");
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
      usleep(1000);
  }
  ros::fini();

  return 0;
};

