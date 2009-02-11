#include <mechanism_controller_test/null_hardware.h>
#include <mechanism_control/mechanism_control.h>
#include <urdf/parser.h>

void controlLoop()
{
  char *filename = "robot.xml";

  //Create null hardware
  NullHardware hw;

  //Create mechanism control
  MechanismControl mc(hw.hw_);

  MechanismControlNode mcn(&mc);

  TiXmlDocument xml;
  xml.LoadFile("robot.xml");
  urdf::normalizeXml(xml.RootElement());
  TiXmlElement *root = xml.FirstChildElement("robot");
  if(!root)
  {
    ROS_FATAL("Could not load xml file: %s\n", filename);
    exit(1);
  }
  
  //Initialize null hardware interface
  hw.initXml(root);
    
  //Initialize mechanism control from robot description
  mcn.initXml(root);
  
  //Spawn controller (and measure initialization time)

  //Start running controller updates (and measure update time)
  int count = 0;
  while(1){
    count++;
    mcn.update();
    if(count % 1000 == 0)
      printf("%d seconds simulated \n", count / 1000);
  }
  
  //Done - return
}

int main(int argc, char *argv[]){
  ros::init(argc, argv);

  //Parse options here? TODO

  ros::Node *node = new ros::Node("pr2_etherCAT", ros::Node::DONT_HANDLE_SIGINT);

  controlLoop();
    
  delete node;

  return 0;
}
