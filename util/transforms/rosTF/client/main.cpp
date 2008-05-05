#include "rosTF/rosTF.h"

class testClient : public ros::node
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
  
  while(testClient.ok())
    {
      sleep(1);
    }

  return 0;
};

