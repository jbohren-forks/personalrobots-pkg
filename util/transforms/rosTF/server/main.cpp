#include "rosTF/rosTF.h"

class testServer : ros::node
{
public:
  //constructor
  testServer() : ros::node("server"){
    pTFServer = new rosTFServer(*this);
  };

  rosTFServer * pTFServer;

  void test () {pTFServer->sendEuler(5,2,1,1,1,1,1,1,383838838ULL);};

};

int main(int argc, char ** argv)
{
  ros::init(argc, argv);
  testServer myTestServer;
  
  while(1)
    {
      myTestServer.test();
      sleep(1);
    }

  return 0;
};

