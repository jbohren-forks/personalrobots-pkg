#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"

#include <sys/time.h>
#include <iostream>
#include "tf/time_cache.h"
#include "tf/tf.h"

using namespace std;

int main(void)
{
  btQuaternion quat;
  quat.setEuler(0,1,1);
  cout << quat.x() << std::endl << quat.y() << std::endl << quat.z() << std::endl << quat.w() << std::endl;
  quat.setEuler(0,1,0);
  cout << quat.x() << std::endl << quat.y() << std::endl << quat.z() << std::endl << quat.w() << std::endl;
  quat.setEuler(0,0,1);
  cout << quat.x() << std::endl << quat.y() << std::endl << quat.z() << std::endl << quat.w() << std::endl;
  quat.setEuler(0,1,2);
  cout << quat.x() << std::endl << quat.y() << std::endl << quat.z() << std::endl << quat.w() << std::endl;


  btTransform tran(quat, btVector3(btScalar(1), btScalar(1), btScalar(1)));

  std::cout << sizeof(btScalar)<<std::endl;

  tf::TimeCache myCache;

  tf::Transformer myTR;
    
return 0;
};
