

#include "Quaternion3D.hh"


int main()
{
  Quaternion3D myquat(1,1,1,1,1,1,1);

  Quaternion3D my2ndquat(myquat.asMatrix());
  std::cout << "Quat1"<<std::endl;
  myquat.printMatrix();
  std::cout << "Quat2"<<std::endl;
  my2ndquat.printMatrix();

  return 0;
}
