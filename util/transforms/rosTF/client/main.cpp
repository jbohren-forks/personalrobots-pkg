#include "rosTF/rosTF.h"

class testListener : public ros::node
{
public:

  rosTFClient tf;
  
  //constructor with name
  testListener() : 
    ros::node("client"),  
    tf(*this) 
  {
  
  };
  
  ~testListener()
  {
    
    ros::fini();
  };

};


int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv);

  //Instantiate a local client
  testListener testListener;
  
  //Nothing needs to be done except wait for a quit
  //The callbacks withing the listener class 
  //will take care of everything
  while(testListener.ok())
    {
      sleep(1);
    }

  return 0;
};

