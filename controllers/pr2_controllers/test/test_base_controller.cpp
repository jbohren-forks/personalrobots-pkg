
#include <pr2_controllers/base_controller.h>
#include <mechanism_model/robot.h>

#include <ros/node.h>

int main( int argc, char** argv )
{
  mechanism::Robot *robot_model = new mechanism::Robot;
  controller::BaseController bc;
  HardwareInterface hw(0);

  if(argc != 3)
  {
    printf("Usage: %s <robot_xml> <controller_xml>\n",argv[0]);
    exit(-1);
  }

  ros::init(argc,argv);
  ros::node *node = new ros::node("test_base_controller"); //Can be found by ros::node *ros::g_node for now.  Will eventually be superceded by ros::peer interface

  printf("robot file:: %s, controller file:: %s\n",argv[1],argv[2]);
  char *xml_robot_file = argv[1];
  TiXmlDocument xml(xml_robot_file);   // Load robot description
  xml.LoadFile();
  TiXmlElement *root = xml.FirstChildElement("robot");
  robot_model->initXml(root);


  char *xml_control_file = argv[2];
  TiXmlDocument xml_control(xml_control_file);   // Load robot description
  xml_control.LoadFile();
  TiXmlElement *root_control = xml_control.FirstChildElement("robot");
  TiXmlElement *root_controller = root_control->FirstChildElement("controller");

  mechanism::RobotState *robot_state = new mechanism::RobotState(robot_model, &hw);

  bc.initXml(robot_state,root_controller);
  ros::fini();
  delete robot_model;
  delete robot_state;
//void BaseController::initXml(mechanism::Robot *robot, TiXmlElement *config)
}


/*class BaseControllerTest : public testing::Test {
 protected:
  virtual void SetUp() {
  }

  // virtual void TearDown() {}

  Queue<int> q0_;
  Queue<int> q1_;
  Queue<int> q2_;
};


TEST (BaseControllerTests, constructionDestruction)
{
  controller::BaseController *bc = new controller::BaseController();
  delete bc;
}

TEST (BaseControllerTests, loadXML)
{
  mechanism::Robot *robot = new mechanism::Robot("r2d2");
  controller::BaseController bc;

  const string xml_controller_file = "controller_base.xml";
  const string xml_robot_file = "pr2_base.xml"

  TiXmlDocument xml(xml_robot_file);   // Load robot description
  xml.LoadFile();
  TiXmlElement *root = xml.FirstChildElement("robot");
  robot->initXml(root);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
*/
