#include "tf/transform_listener.h"



class testListener : public ros::node
{
public:

  tf::TransformListener tf;
  
  //constructor with name
  testListener() : 
    ros::node("listener"),  
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

  //Instantiate a local listener
  testListener testListener;
  
  //Nothing needs to be done except wait for a quit
  //The callbacks withing the listener class 
  //will take care of everything
  while(testListener.ok())
    {
      std::cout << "The current list of frames is:" <<std::endl;
      std::cout << testListener.tf.allFramesAsString()<<std::endl;
      sleep(1);
    }

  return 0;
};

