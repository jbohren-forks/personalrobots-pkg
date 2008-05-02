#include "rosTF/rosTF.h"

class testClient : ros::node
{
public:
  //constructor
  testClient() : ros::node("client") {
    pClient = new rosTFClient(*this);
  };
  
  rosTFClient * pClient;

};


int main(int argc, char ** argv)
{
  ros::init(argc, argv);
  testClient testClient;
  
  while(1)
    {
      sleep(1);
    }

  return 0;
};

