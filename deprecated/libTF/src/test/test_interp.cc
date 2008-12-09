#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <libTF/libTF.h>

int
main(void)
{
  libTF::TransformReference mTR(true);
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  uint64_t atime = temp_time_struct.tv_sec * 1000000000ULL + (uint64_t)temp_time_struct.tv_usec * 1000ULL;
  uint64_t btime = atime + 1 * 1000000000ULL;
  uint64_t ctime = btime + 1 * 1000000000ULL;

  for(int i=0;i<10;i++)
  {
    mTR.setWithEulers("2",
                      "1",
                      i*0.2,
                      i*-0.1,
                      0.0,
                      i*M_PI/12.0,
                      0.0,
                      0.0,
                      atime-i*1000000000ULL);
  }
  mTR.setWithEulers("2",
                   "1",
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    atime);

  mTR.setWithEulers("2",
                    "1",
                    1.0,
                    1.0,
                    0.0,
                    M_PI,
                    0.0,
                    0.0,
                    ctime);


  libTF::TFPose2D inpose;
  inpose.x = 0.0;
  inpose.y = 0.0;
  inpose.yaw = 0.0;
  inpose.frame = "2";
  inpose.time = atime;

  libTF::TFPose2D outpose = mTR.transformPose2D("1", inpose);

  printf("in:  %.3f %.3f %.3f\n",
         inpose.x, inpose.y, inpose.yaw);
  printf("out: %.3f %.3f %.3f\n",
         outpose.x, outpose.y, outpose.yaw);

  return(0);
}

