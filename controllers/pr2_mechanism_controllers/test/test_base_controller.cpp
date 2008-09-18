
#include <pr2_mechanism_controllers/base_controller.h>
#include <mechanism_model/robot.h>
#include <hardware_interface/hardware_interface.h>
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

  NEWMAT::Matrix A(16,3);

  A.Row(1) << 10 << 8.95 << 0.05;
  A.Row(2) << 0 <<  -2 << 0;
  A.Row(3) << 1 << -0.1 << 0.01;
  A.Row(4) << 2 << 1.1 << 0;

  A.Row(5) << 3 << 2 << -0.05;
  A.Row(6) << 4 << 3 << 0.01;
  A.Row(7) << 5 << 4.1 << 0.05;
  A.Row(8) << -1 << -2 << 0.025;

  A.Row(9) << 6.15 << 5.05 << 0.01;
  A.Row(10) << 6.985 << 6.02 << 0.01;
  A.Row(11) << 8.01 << 8.05 << -0.05;
  A.Row(12) << 9.03 << 8.1 << -0.01;

  A.Row(13) << -8.03 << -9.1 << 0.01;
  A.Row(14) << -10.03 << -13.1 << 0.05;
  A.Row(15) << -15.03 << -16.1 << -0.015;
  A.Row(16) << -16.03 << -17.1 << -0.01;

  NEWMAT::Matrix B(16,1);
  B << 1.1 << 1 << 1.1 << 1.15 << 0.95 << 0.99 << 0.98 << 0.95 << 1.05 << 1.1 << 1.05 << 1 << 1.13 << 0.995 << 1.035 << 1.08;

  NEWMAT::Matrix xfit(3,1);

  xfit = bc.iterativeLeastSquares(A,B,"Gaussian",10);

  cout << "done" << xfit << endl;
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
