#include <iostream>
#include "ros/node.h"               //All roscpp nodes will need this
#include "time.h"
#include <std_msgs/BaseVel.h>
// Header created from the message
#include <rosControllers/RotaryJointState.h>
#include <string>

static const std::string joint_name = std::string("DUMMY_JOINT");
static const std::string joint_name_l = std::string("DUMMY_JOINT_listener");

class Pinger : public ros::node
{
public:
  rosControllers::RotaryJointState in;  // Our input flow
  std_msgs::BaseVel out;
//   pingpong_cpp::ppball out; // Our output flow
//   std::string msg;      // A string containing our message to echo
  double freq;     // Our frequency of operation.
  Pinger() : ros::node(joint_name_l)
  {
    // Advertise our output
    advertise<std_msgs::BaseVel>(joint_name+"_cmd");
    //Register our in flow as a sink
    subscribe(joint_name+"_state", in, &Pinger::pong_callback);
    // Attempt to retrieve some parameters from the server:
    // Retrieve ponger/message, or else set default to 'pong! '
//     param("pinger/message", msg, std::string("ping! "));
    // Retriever freq or else set default to 1.0
//     param("freq", freq, 1.0);
  }
  void pong_callback() {
    std::cout << "Received msg " << in.VelAct << ": " << in.VelCmd << std::endl;
    // Copy over input to output, append our own msg, and increment counter
    out.vx = 0.5;
    out.vw = 0.1;
    usleep(1000000);
    publish(joint_name+"_cmd", out);
//     out.msg = in.msg;
//     out.msg.append(msg);
//     out.counter = in.counter + 1;
//     std::cout << "Sending msg " << out.counter << ": " << out.msg << std::endl;
    //Sleep for a bit:
//     usleep(1000000/freq);
    // Publish output
//     publish("ping_bus", out);
//   }
  // Function call to start play in the first place
//   void serve() {
//     out.msg = msg;
//     out.counter = 1;
//     publish("ping_bus", out);
  }
};
int main(int argc, char **argv)
{
  // Initialize ros
  ros::init(argc, argv);
  // Create a new instance of pinger
  Pinger p;
  // Send first message
//   p.serve();
  // Wait for pinger to finish using a different wait method.
  while (p.ok()) {
    usleep(100000);
  }
  // Done
  printf("Pinger is done!\n");
  return 0;
}
