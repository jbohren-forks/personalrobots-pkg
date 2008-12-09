#include "libTF/libTF.h"

#include <sys/time.h>
#include <cstdlib>

int main(int argc, char ** argv)
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);

  bool success = true; //Flag of whether to say Success at the end

  //Test many different premutations
  for (unsigned int j = 0; j < 3;j ++)
    {
      for (int i = -5; i < 5; i++)
	{

	  double yaw, pitch, roll;
	  if (j ==0){//yaw only
	    yaw = 0.1*i ;
	    roll = 0.0;
	    pitch = 0.0;
	  }
	  else if (j == 1){ //pitch only
	    yaw = 0.0;
	    roll = 0.0;
	    pitch = 0.1*i;
	  }
	  else if ( j== 3){ //roll only
	    yaw = 0.0 ;
	    roll = 0.1*i ;
	    pitch = 0.0;
	  }
	  else { //random combinations
	    yaw = ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
	    pitch = ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
	    roll = ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
	  }

	  	      
	  NEWMAT::Matrix m = libTF::Pose3D::matrixFromEuler(0.0,
							    0.0,
							    0.0,
							    yaw,
							    pitch,
							    roll);

	  libTF::Euler out = libTF::Pose3D::eulerFromMatrix(m);

	  //TODO add +- 2PI checking/redundant angles checking
	  // see if input is the same as output (accounting for floating point errors)
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
