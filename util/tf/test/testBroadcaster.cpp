#include "tf/transform_broadcaster.h"

class testBroadcaster : public ros::node
{
public:
  //constructor
  testBroadcaster() : ros::node("broadcaster"),broadcaster(*this),count(2){};
  //Clean up ros connections
  ~testBroadcaster() { }

  //A pointer to the rosTFServer class
  tf::TransformBroadcaster broadcaster;


  // A function to call to send data periodically
  void test () {
    NEWMAT::Matrix mat(4,4);
    mat << 1 << 0 << 0 << 1
        << 0 << 1 << 0 << 2
        << 0 << 0 << 1 << 3
        << 0 << 0 << 0 << 1;

    broadcaster.sendTransform(btTransform(btQuaternion(0,0,0), btVector3(1,2,3)), 1000000000ULL, "frame1", "frame2");
    /*    pTFServer->sendEuler("count","count++",1,1,1,1,1,1,ros::Time(100000,100000));
    pTFServer->sendInverseEuler("count","count++",1,1,1,1,1,1,ros::Time(100000,100000));
    pTFServer->sendDH("count","count++",1,1,1,1,ros::Time(100000,100000));
    pTFServer->sendQuaternion("count","count++",1,1,1,1,1,1,1,ros::Time(100000,100000));
    pTFServer->sendMatrix("count","count++",mat, ros::Time::now());
*/
    if (count > 9000)
      count = 0;
    std::cerr<<count<<std::endl;
  };

private:
  int count;

};

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv);

  //Construct/initialize the server
  testBroadcaster myTestBroadcaster;
  
  while(myTestBroadcaster.ok())
  {
      //Send some data
      myTestBroadcaster.test();
      usleep(1000);
  }
  ros::fini();

  return 0;
};

