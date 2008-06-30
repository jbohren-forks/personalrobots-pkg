#ifndef OPEN_DOOR_IK_HOOK_H
#define OPEN_DOOR_IK_HOOK_H

#include <vector>

class IKHook
{
public:
  virtual bool ik_joint_solution(double x, double y, double z, double theta_init, 
                                 double psi, double max_theta_dev, 
                                 std::vector<double> &solution) = 0;
  virtual ~IKHook() { }
};

#endif

