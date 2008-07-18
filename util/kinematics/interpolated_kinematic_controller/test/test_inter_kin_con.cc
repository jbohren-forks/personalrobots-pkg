#include <ros/node.h>
#include <libpr2API/pr2API.h>
#include <rosgazebo/EndEffectorState.h>

using namespace KDL;

int main(int argc, char **argv)
{  
  ros::init(argc, argv);

  ros::node mynode("easy_kin_con_test");
  mynode.advertise<rosgazebo::EndEffectorState>("right_pr2arm_set_end_effector");

  //In shoulder frame x 0.562689
  //In shoulder frame y -0.367447
  //In shoulder frame z -0.369594
  Rotation r = Rotation::RotZ(DTOR(0));
  Vector v(.562689,-.367447,-.369594);

  sleep(1);

  rosgazebo::EndEffectorState efs;
  efs.set_rot_size(9);
  efs.set_trans_size(3);
  for(int i = 0; i < 9; i++) {
    efs.rot[i] = r.data[i];
  }
  for(int i = 0; i < 3; i++) {
    efs.trans[i] = v.data[i];
  }

  mynode.publish("right_pr2arm_set_end_effector",efs);
  
  sleep(1);
  
  mynode.shutdown();

  return 0;    
}
