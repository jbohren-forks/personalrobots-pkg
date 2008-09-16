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
    
  //Useful Operator Overload
  for ( unsigned int i = 0; i < runs ; i++ )
  {    
    btTransform transform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i]));
    btQuaternion initial(xvalues[i],yvalues[i],zvalues[i]);
    btQuaternion final(xvalues[i],yvalues[i],zvalues[i]);
    final = transform * initial;
    std::printf("Useful Operator Overload: %f, angle between quaternions\n", initial.angle(final));
  } 

  
  return 0;  
}


