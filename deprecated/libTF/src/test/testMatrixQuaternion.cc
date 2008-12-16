#include "libTF/libTF.h"

#include <angles/angles.h>
#include <sys/time.h>
#include <cstdlib>

int main(int argc, char ** argv)
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);

  libTF::Pose3D aPose;

  bool success = true; //Flag of whether to say Success at the end

  int i_max = 100;

  //Test many different premutations
  for (unsigned int j = 0; j < 3;j ++)
    {
      for (int i = -i_max; i < i_max; i++)
	{

          	  double yaw, pitch, roll;
                  /*if (j ==0){//yaw only
	    yaw = M_PI/2*(double)i/(double)i_max;
	    roll = 0.0;
	    pitch = 0.0;
	  }
	  else if (j == 1){ //pitch only
	    yaw = 0.0;
	    roll = 0.0;
	    pitch = M_PI/2*(double)i/(double)i_max;
	  }
	  else if ( j== 3){ //roll only
	    yaw = 0.0 ;
            roll = M_PI/2*(double)i/(double)i_max;
            pitch = 0.0;
	  }
	  else*/ { //random combinations
	    yaw = 2* M_PI * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
	    pitch = 2 * M_PI * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
	    roll = 2 *M_PI * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
	  }

	  NEWMAT::Matrix m = libTF::Pose3D::matrixFromEuler(0.0,
							    0.0,
							    0.0,
							    yaw,
							    pitch,
							    roll);
          
          //          std::cout << m <<std::endl;
          aPose.setFromMatrix(m);
          m = aPose.asMatrix();

	  libTF::Euler out = libTF::Pose3D::eulerFromMatrix(m);
	  libTF::Euler out2 = libTF::Pose3D::eulerFromMatrix(m,2);

	  // see if input is the same as output (accounting for floating point errors)
	  if ((fabs(angles::modNPiBy2(out.yaw) - angles::modNPiBy2(yaw)) > 0.001 || fabs(angles::modNPiBy2(out.pitch) - angles::modNPiBy2(pitch)) > 0.001 || fabs(angles::modNPiBy2(out.roll) -angles::modNPiBy2(roll)) > 0.0001) &&
              (fabs(angles::modNPiBy2(out2.yaw) - angles::modNPiBy2(yaw)) > 0.001 || fabs(angles::modNPiBy2(out2.pitch) - angles::modNPiBy2(pitch)) > 0.001 || fabs(angles::modNPiBy2(out2.roll) -angles::modNPiBy2(roll)) > 0.0001))
	    {

	      printf("in: %.3f %.3f %.3f\n",
		     yaw, pitch, roll);

	      std::cout << m;


	      printf("out: %.3f %.3f %.3f\n",
		     out.yaw, out.pitch, out.roll);
	      printf("out2: %.3f %.3f %.3f\n\n",
		     out2.yaw, out2.pitch, out2.roll);
	  

	      success = false;
	      printf("FAILURE!!!!!!!!!!\n\n");
	    }

	}
    }


  NEWMAT::Matrix matrix(4,4);  
  double temp[16] = {0.855923,  0.516655,  -0.021430,  0.618588,  0.516977,  -0.855859,  0.013713,  0.077414,  -0.011257,  -0.022816,  -0.999648,  -0.110394,  0.000000,  0.000000,  0.000000,  1.000000};
  matrix << temp;
   
  std::cout<<"Before: "<<std::endl;
  std::cout << matrix;

  aPose.setFromMatrix(matrix);

  libTF::Quaternion myquat;
  aPose.getQuaternion(myquat);
  std::cout << " MY Quaternion" <<  myquat.x << " " << myquat.y << " " << myquat.z  << " " << myquat.w << std::endl;

  NEWMAT::Matrix matrix2 = aPose.asMatrix();


  std::cout<<"After: "<<std::endl;
  std::cout << matrix2;
  

  std::cout<<"Difference: "<<std::endl;
  std::cout << matrix - matrix2;

            


  if (success)
    {
      printf("Success\nAll tests Passed\n");
      return(0);
    }
  else return -1;
}
