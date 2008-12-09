#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <libTF/libTF.h>

#define DTOR(a) ((a) * M_PI/180.0)
#define RTOD(a) ((a) * 180.0/M_PI)

using namespace libTF;

int
main(void)
{
  libTF::TransformReference mTR(true, 0);
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  uint64_t atime = temp_time_struct.tv_sec * 1000000000ULL + (uint64_t)temp_time_struct.tv_usec * 1000ULL;

  libTF::TFPose2D mappose;
  libTF::TFPose2D odompose;
  libTF::TFPose2D apose;

  odompose.x = 1;
  odompose.y = 0;
  odompose.yaw = DTOR(60);
  odompose.frame = "3";
  odompose.time = atime;

  mappose.x = 30.0;
  mappose.y = 40.0;
  //  mappose.yaw = DTOR(-36.0);
  mappose.yaw = DTOR(90);
  mappose.frame = "1";
  mappose.time = atime;

  apose.x = 0;
  apose.y = 0;
  apose.yaw = DTOR(0);
  apose.frame = "3";
  apose.time = atime;

  libTF::TFPose diffpose;
  diffpose.x = mappose.x-odompose.x;
  diffpose.y = mappose.y-odompose.y;
  diffpose.yaw = mappose.yaw-odompose.yaw;
  //diffpose.x = 10;
  //diffpose.y = 100;
  diffpose.z = 1000;
  //diffpose.yaw = DTOR(0);
  diffpose.pitch = DTOR(0);
  //diffpose.roll = DTOR(90);
  diffpose.roll = DTOR(0);
  //  diffpose.yaw = 0;
  diffpose.frame = "1";
  diffpose.time = atime;

  libTF::TFPose2D diffpose2;
  //diffpose.x = mappose.x-odompose.x;
  //diffpose.y = mappose.y-odompose.y;
  //diffpose.yaw = mappose.yaw-odompose.yaw;
  diffpose2.x = diffpose.x;
  diffpose2.y = diffpose.y;
  diffpose2.yaw = diffpose.yaw;
  diffpose2.frame = "1";
  diffpose2.time = atime;

  mTR.setWithEulers("2",
                    mappose.frame,
                    mappose.x,
                    mappose.y,
                    0.0,
                    mappose.yaw,
                    0.0,
                    0.0,
                    atime);

  mTR.setWithMatrix("3",
  		    "2",
		    Pose3D::matrixFromEuler(odompose.x, odompose.y, 0.0, odompose.yaw, 0.0, 0.0).i(),
		    atime);

  // See the list of transforms to get between the frames
  std::cout<<"Viewing (1,3):"<<std::endl;  
  std::cout << mTR.viewChain("1","3");
  std::cout<<"Viewing (2,3):"<<std::endl;  
  std::cout << mTR.viewChain("2","3");


  libTF::TFPose2D result = mTR.transformPose2D(mappose.frame,odompose);
  libTF::TFPose2D result2 = mTR.transformPose2D(mappose.frame,apose);

  libTF::TFPoint point_in;
  point_in.frame = odompose.frame;
  point_in.x = odompose.x;
  point_in.y = odompose.y;
  point_in.z = 0;
  point_in.time = atime;

  libTF::TFPoint point = mTR.transformPoint(mappose.frame, point_in);

  puts("--------------------------");
  printf("Odom: %.3f %.3f %.3f\n",
         odompose.x, odompose.y, RTOD(odompose.yaw));
  printf("Map : %.3f %.3f %.3f\n",
         mappose.x, mappose.y, RTOD(mappose.yaw));
  printf("Diff : %.3f %.3f %.3f %.3f %.3f %.3f\n",
         diffpose.x, diffpose.y, diffpose.z, RTOD(diffpose.yaw), RTOD(diffpose.pitch), RTOD(diffpose.roll));
  puts("--------------------------");
  printf("2D out Out odompose: %.3f %.3f %.3f\n",
         result.x, result.y, RTOD(result.yaw));
  printf("2D out Out apose : %.3f %.3f %.3f\n",
         result2.x, result2.y, RTOD(result2.yaw));
  printf("Point : %.3f %.3f %.3f\n",
         point.x, point.y,point.z);

  std::cout<<"1,2:"<<std::endl<<mTR.getMatrix("1","2",atime);
  std::cout<<"2,3"<<std::endl<<mTR.getMatrix("2","3",atime);
  std::cout<<"1,3:"<<std::endl<<mTR.getMatrix("1","3",atime);

  NEWMAT::Matrix matPoint(4,1);
  matPoint(1,1) = point_in.x;
  matPoint(2,1) = point_in.y;
  matPoint(3,1) = point_in.z;
  matPoint(4,1) = 1;


  //  std::cout <<"point from, to:"<<std::endl;
  // std::cout<<matPoint<<std::endl;
  //  std::cout<<mTR.getMatrix(1,2,atime)*matPoint;
  //std::cout<<mTR.getMatrix(1,point_in.frame,atime)*matPoint;

  // std::cout <<"vector from, to:"<<std::endl;
  //matPoint(4,1)=0;
  //std::cout<<matPoint<<std::endl;
  //std::cout<<mTR.getMatrix(1,point_in.frame,atime)*matPoint;


  //  libTF::TFPoint point2 = mTR.transformPoint(1,point_in);

  libTF::TFEulerYPR ypr_in;
  ypr_in.frame = odompose.frame;
  ypr_in.yaw = 0;
  ypr_in.pitch = 0;
  ypr_in.roll = 0;
  ypr_in.time = atime;

  libTF::TFEulerYPR ypr = mTR.transformEulerYPR("1", ypr_in);

  printf("Ypr : %.3f %.3f %.3f\n",
         ypr.yaw, ypr.pitch,ypr.roll);


  TFVector2D vec2_in;
  vec2_in.x = 1;
  vec2_in.y = 1;
  vec2_in.frame = "2";
  vec2_in.time = atime;

  TFVector2D vec2 = mTR.transformVector2D("1",vec2_in);
  
  printf("Vector : %.3f %.3f\n",
         vec2.x, vec2.y);
  

  TFVector xaxis;
  xaxis.x = 1;
  xaxis.y = 0;
  xaxis.z = 0;
  xaxis.frame = "9";
  xaxis.time = atime;

  TFVector yaxis;
  yaxis.x = 0;
  yaxis.y = 1;
  yaxis.z = 0;
  yaxis.frame = "9";
  yaxis.time = atime;

  TFVector zaxis;
  zaxis.x = 0;
  zaxis.y = 0;
  zaxis.z = 1;
  zaxis.frame = "9";
  zaxis.time = atime;


  printf("--------------------------TESTING VECTORS\n");
  printf("From Frame 9 into frame 1\n");
  for (int ind = 0; ind < 2; ind++)
    for (int ind1 = 0; ind1 < 2; ind1++)
      for (int ind2 = 0; ind2 < 2; ind2++)
	{	
	  
	  mTR.setWithEulers("9",
			    "1",
			    diffpose.x,
			    diffpose.y,
			    diffpose.z,
			    DTOR(90*ind),
			    DTOR(90*ind1),
			    DTOR(90*ind2),
			    atime++);


	  printf("Ypr : %.3f %.3f %.3f\n",
		 90.0*ind, 90.0*ind1, 90.0*ind2);


  TFVector vec_out = mTR.transformVector("1",xaxis);
  printf("X Axis rotated = ");
  printf("Vector : %.3f %.3f %.3f\n",
         vec_out.x, vec_out.y, vec_out.z);
  
  vec_out = mTR.transformVector("1",yaxis);
  printf("Y Axis rotated = ");
  printf("Vector : %.3f %.3f %.3f\n",
         vec_out.x, vec_out.y, vec_out.z);

  vec_out = mTR.transformVector("1",zaxis);
  printf("Z Axis rotated = ");
  printf("Vector : %.3f %.3f %.3f\n",
         vec_out.x, vec_out.y, vec_out.z);
  
	}
  



  TFPoint xunit;
  xunit.x = 1;
  xunit.y = 0;
  xunit.z = 0;
  xunit.frame = "9";
  xunit.time = atime;

  TFPoint yunit;
  yunit.x = 0;
  yunit.y = 1;
  yunit.z = 0;
  yunit.frame = "9";
  yunit.time = atime;

  TFPoint zunit;
  zunit.x = 0;
  zunit.y = 0;
  zunit.z = 1;
  zunit.frame = "9";
  zunit.time = atime;


  printf("-------------------TESTING POINTS\n");
	 printf("From Frame 9 into frame 1\n");

  printf("Fixed offset:\n %.3f %.3f %.3f \n", 
	 diffpose.x, diffpose.y, diffpose.z);

  for (int ind = 0; ind < 2; ind++)
    for (int ind1 = 0; ind1 < 2; ind1++)
      for (int ind2 = 0; ind2 < 2; ind2++)
	{	
	  
	  mTR.setWithEulers("9",
			    "1",
			    diffpose.x,
			    diffpose.y,
			    diffpose.z,
			    DTOR(90*ind),
			    DTOR(90*ind1),
			    DTOR(90*ind2),
			    atime++);


	  printf("Ypr : %.3f %.3f %.3f\n",
		 90.0*ind, 90.0*ind1, 90.0*ind2);


  TFPoint vec_out = mTR.transformPoint("1",xunit);
  printf("X Unit rotated = ");
  printf("Point : %.3f %.3f %.3f\n",
         vec_out.x, vec_out.y, vec_out.z);
  
  vec_out = mTR.transformPoint("1",yunit);
  printf("Y Unit rotated = ");
  printf("Point : %.3f %.3f %.3f\n",
         vec_out.x, vec_out.y, vec_out.z);

  vec_out = mTR.transformPoint("1",zunit);
  printf("Z Unit rotated = ");
  printf("Point : %.3f %.3f %.3f\n",
         vec_out.x, vec_out.y, vec_out.z);
  
	}
  

}
