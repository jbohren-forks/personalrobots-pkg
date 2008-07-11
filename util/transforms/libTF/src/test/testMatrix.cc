#include "libTF/libTF.h"

int main(int argc, char ** argv)
{
  bool success = true;
  for (unsigned int j = 0; j < 3;j ++)
    {
      for (int i = -5; i < 5; i++)
	{

	  double yaw, pitch, roll;
	  if (j ==0){
	    yaw = 0.1*i ;
	    roll = 0.0;
	    pitch = 0.0;
	  }
	  else if (j == 1){
	    yaw = 0.0;
	    roll = 0.1*i ;
	    pitch = 0.0;
	  }
	  else {
	    yaw = 0.0 ;
	    roll = 0.0;
	    pitch = 0.1*i;
	  }






	  printf("in: %.3f %.3f %.3f\n",
		 yaw, roll, pitch);

	  NEWMAT::Matrix m = libTF::Pose3D::matrixFromEuler(0.0,
							    0.0,
							    0.0,
							    yaw,
							    pitch,
							    roll);

	  libTF::Pose3D::Euler out = libTF::Pose3D::eulerFromMatrix(m);

	  printf("out: %.3f %.3f %.3f\n\n",
		 out.yaw, out.roll, out.pitch);
	  
	  //TODO add +- 2PI checking/redundant angles checking
	  if ((out.yaw != yaw || out.pitch != pitch || out.roll !=roll))
	      success = false;

	}
    }
  if (success)
    printf("Success\n");
  else
    printf("Failure\n");

  return(0);
}
