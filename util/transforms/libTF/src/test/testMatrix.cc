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
	    roll = 0.0;
	    pitch = 0.1*i;
	  }
	  else {
	    yaw = 0.0 ;
	    roll = 0.1*i ;
	    pitch = 0.0;
	  }


	  	      
	  NEWMAT::Matrix m = libTF::Pose3D::matrixFromEuler(0.0,
							    0.0,
							    0.0,
							    yaw,
							    pitch,
							    roll);

	  libTF::Pose3D::Euler out = libTF::Pose3D::eulerFromMatrix(m);

	  //TODO add +- 2PI checking/redundant angles checking
	  if (fabs(out.yaw - yaw) > 0.001 || fabs(out.pitch - pitch) > 0.001 || fabs(out.roll -roll) > 0.0001)
	    {

	      printf("in: %.3f %.3f %.3f\n",
		     yaw, pitch, roll);

	      std::cout << m;


	      printf("out: %.3f %.3f %.3f\n\n",
		     out.yaw, out.pitch, out.roll);
	  

	      success = false;
	      printf("FAILURE!!!!!!!!!!\n\n");
	    }

	}
    }
  if (success)
    printf("Success\nAll tests Passed\n");
 
  return(0);
}
