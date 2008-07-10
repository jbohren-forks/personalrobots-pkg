#include "libTF/libTF.h"

int main(int argc, char ** argv)
{
  double yaw = 0.0;
  double roll = 0.0;
  double pitch = 0.0;

  printf("in: %.3f %.3f %.3f\n",
         yaw, roll, pitch);

  NEWMAT::Matrix m = libTF::Pose3D::matrixFromEuler(0.0,
                                                    0.0,
                                                    0.0,
                                                    yaw,
                                                    pitch,
                                                    roll);

  libTF::Pose3D::Euler out = libTF::Pose3D::eulerFromMatrix(m);

  printf("out: %.3f %.3f %.3f\n",
         out.yaw, out.roll, out.pitch);

  return(0);
}
