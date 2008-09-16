#include <vector>
#include <sys/time.h>

#include "LinearMath/btTransform.h"


void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};


int main(int argc, char **argv){
  
  unsigned int runs = 400;
  seed_rand();
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
  }
  
  
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    btMatrix3x3 mat;
    btQuaternion q_baseline(xvalues[i],yvalues[i],zvalues[i]);
    mat.setRotation(q_baseline);
    btQuaternion q_from_m;
    mat.getRotation(q_from_m);
    std::printf("%f, angle between quaternions\n", q_from_m.angle(q_baseline));
  } 
  
  return 0;  
}


