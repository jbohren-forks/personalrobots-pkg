#include <cstdio>
#include <stdexcept>
#include <vector>
//#include "kniBase.h"
#include "katana/katana.h"
#include "ros/common.h"
#include "ros/node.h"
#include "std_srvs/StringString.h"
#include "std_srvs/StringArmCSpace.h"
#include "std_srvs/ArmCSpaceSeqString.h"
#include "std_srvs/ArmCSpaceString.h"
#include "std_srvs/ArmCSpaceSeqString.h"
#include "std_srvs/UInt32String.h"

using std::vector;
using std::cout;
using std::endl;

class KatanaServer : public ros::node
{
  public:
    KatanaServer() : ros::node("katana_server")
    {
      advertise_service("katana_calibrate_service", &KatanaServer::calibrateSrv);
      advertise_service("katana_move_upright_service", &KatanaServer::move_to_upright);
      advertise_service("katana_back_to_upright_service", &KatanaServer::move_back_to_upright);
      advertise_service("katana_move_for_camera_service", &KatanaServer::move_for_camera);
      advertise_service("katana_get_joint_angles_service", &KatanaServer::get_current_joint_angles);
      advertise_service("katana_move_joints_single_service", &KatanaServer::moveJointsSingle);
      advertise_service("katana_move_joint_sequence_rad_service", &KatanaServer::moveJointsSeqRad);
      advertise_service("katana_move_joint_sequence_deg_service", &KatanaServer::moveJointsSeqDeg);
      advertise_service("katana_gripper_cmd_service", &KatanaServer::gripperCmd);
    }

    bool calibrateSrv(std_srvs::StringString::request &req,
                   std_srvs::StringString::response &res)
    {
      Katana *katana = new Katana();
      bool success = katana->calibrate();
      if (success) {
        res.str = "Done";
				cout << "Done!" << endl;
      } else {
				res.str = "Error";
				cout << "Error!" << endl;
			}
      delete katana;
      return(success);
    }

    bool move_to_upright(std_srvs::StringString::request &req,
                   std_srvs::StringString::response &res)
    {
      Katana *katana = new Katana();
      bool success = katana->goto_upright();
      if (success) {
        res.str = "Done";
				cout << "Done!" << endl;
      } else {
				res.str = "Error";
				cout << "Error!" << endl;
			}
      delete katana;
      return(success);
    }
    
    bool move_for_camera(std_srvs::StringString::request &req,
                   std_srvs::StringString::response &res)
    {
      Katana *katana = new Katana();
      bool success = katana->move_for_camera();
      if (success) {
        res.str = "Done";
				cout << "Done!" << endl;
      } else {
				res.str = "Error";
				cout << "Error!" << endl;
			}
      delete katana;
      return(success);
    }
    
    bool move_back_to_upright(std_srvs::StringString::request &req,
                   std_srvs::StringString::response &res)
    {
      Katana *katana = new Katana();
      bool success = katana->move_back_to_upright();
      if (success) {
        res.str = "Done";
				cout << "Done!" << endl;
      } else {
				res.str = "Error";
				cout << "Error!" << endl;
			}
      delete katana;
      return(success);
    }

    bool get_current_joint_angles(std_srvs::StringArmCSpace::request &req,
                   std_srvs::StringArmCSpace::response &res)
    {
      Katana *katana = new Katana();
      vector<double> jointPositions = katana->get_joint_positions();
      res.jointAngles.set_angles_size(jointPositions.size());
      for (unsigned int i=0; i<jointPositions.size(); i++) {
        res.jointAngles.angles[i] = jointPositions.at(i);
      }
      delete katana;
      return (jointPositions.size() > 0);
    }
    
    bool moveJointsSingle(std_srvs::ArmCSpaceString::request &req,
                   std_srvs::ArmCSpaceString::response &res)
    {
      Katana *katana = new Katana();
      bool success = katana->goto_joint_position_deg(req.jointAngles.angles[0], req.jointAngles.angles[1],
        req.jointAngles.angles[2], req.jointAngles.angles[3], req.jointAngles.angles[4]);
      if (success) {
        res.str = "Done";
				cout << "Done!" << endl;
      } else {
				res.str = "Error";
				cout << "Error!" << endl;
			}
      delete katana;
      return (success);
    }
    
    bool gripperCmd(std_srvs::UInt32String::request &req,
                   std_srvs::UInt32String::response &res)
    {
      Katana *katana = new Katana();
      bool success = katana->gripper_fullstop(req.value);
      if (success) {
        res.str = "Done";
				cout << "Done!" << endl;
      } else {
				res.str = "Error";
				cout << "Error!" << endl;
			}
      delete katana;
      return (success);
    }
    
    bool moveJointsSeqDeg(std_srvs::ArmCSpaceSeqString::request &req,
                   std_srvs::ArmCSpaceSeqString::response &res)
    {
      Katana *katana = new Katana();
      bool success = false;
      for (size_t i=0; i<req.jointAngles.angles_size; i++) {
        success = katana->goto_joint_position_deg(req.jointAngles.angles[i].angles[0], 
          req.jointAngles.angles[i].angles[1], req.jointAngles.angles[i].angles[2],
          req.jointAngles.angles[i].angles[3], req.jointAngles.angles[i].angles[4]);
        if (!success) break;
      }
      if (success) {
        res.str = "Done";
				cout << "Done!" << endl;
      } else {
				res.str = "Error";
				cout << "Error!" << endl;
			}
      delete katana;
      return (success);
    }
    
    bool moveJointsSeqRad(std_srvs::ArmCSpaceSeqString::request &req,
                   std_srvs::ArmCSpaceSeqString::response &res)
    {
      Katana *katana = new Katana();
      bool success = false;
      for (size_t i=0; i<req.jointAngles.angles_size; i++) {
        success = katana->goto_joint_position_rad(req.jointAngles.angles[i].angles[0], 
          req.jointAngles.angles[i].angles[1], req.jointAngles.angles[i].angles[2],
          req.jointAngles.angles[i].angles[3], req.jointAngles.angles[i].angles[4]);
        if (!success) break;
      }
      if (success) {
        res.str = "Done";
				cout << "Done!" << endl;
      } else {
				res.str = "Error";
				cout << "Error!" << endl;
			}
      delete katana;
      return (success);
    }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  KatanaServer katanaSrv;
  katanaSrv.spin();
  ros::fini();
  return 0;
}
