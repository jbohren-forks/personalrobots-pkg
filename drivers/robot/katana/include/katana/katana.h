#ifndef KATANA_KATANA_H
#define KATANA_KATANA_H

#include <string>
#include <vector>
#include "kniBase.h"
#include "KNI/kmlFactories.h"
#include "common/MathHelperFunctions.h"
#include "KNI_LM/lmBase.h"
#include "common/Timer.h"
#include "KNI_InvKin/KatanaKinematics6M180.h"
#include "katana/ik_hook.h"

class Katana : public IKHook
{
public:
  Katana();
  ~Katana();

  bool calibrate();
  bool set_motor_power(bool on);
  std::vector<double> get_joint_positions();
  std::vector<int> get_joint_encoders();
	std::vector<double> get_pose();
  bool goto_joint_position_deg(double j1, double j2, double j3, 
                               double j4, double j5);
  bool goto_joint_position_deg(int motor_idx, double motor_position);
	bool goto_joint_position_rad(double j1, double j2, double j3, 
                               double j4, double j5);
  bool goto_joint_position_rad(int motor_idx, double motor_position);
  bool gripper_fullstop(bool open_gripper); // for binary gripper control
  bool move_gripper(double fraction_open); // for opening gripper to specified value
  bool goto_upright();
  bool move_for_camera();
  bool move_back_to_upright();
  bool move_along_trajectory(std::vector<std::vector<double> > &jointAngles, double timeperspline);
	int get_number_of_motors();
  bool linear_move(std::vector<double> dstPose, int waitTimeout);
  std::vector<TMotInit> get_motor_parameters();
	std::vector<double> rad_to_enc(std::vector<double> joints_rad);
	bool ik_calculate(double x, double y, double z, double psi, double theta, double psi, 
    std::vector<double> &solution);
	bool ik_calculate(double x, double y, double z, double psi, double theta, double psi, 
    std::vector<double> &solution, std::vector<int> currentEncoders);
	bool ik_joint_solution(double x, double y, double z, double theta_init, double psi, 
		double max_theta_dev, std::vector<double> &solution);
  bool allow_crash_limits(bool allow);

private:
  CCdlCOM *device;
  CCplSerialCRC *protocol;
  CLMBase *kni_lm;
};

#endif

