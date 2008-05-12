#include "libTF/libTF.h"
#include <time.h>

using namespace std;
using namespace libTF;

int main(void)
{
  double dx,dy,dz,dyaw,dp,dr;
  TransformReference mTR;
  
      
  
  dx = dy= dz = 0;
  dyaw = dp = dr = 0.1;
  
  unsigned long long atime = mTR.gettime();

  
  //Fill in some transforms
  //  mTR.setWithEulers(10,2,1,1,1,dyaw,dp,dr,atime); //Switching out for DH params below
  mTR.setWithDH(10,2,1.0,1.0,1.0,dyaw,atime);
    //mTR.setWithEulers(2,3,1-1,1,1,dyaw,dp,dr,atime-1000);
   mTR.setWithEulers(2,3,1,1,1,dyaw,dp,dr,atime-100);
   mTR.setWithEulers(2,3,1,1,1,dyaw,dp,dr,atime-50);
   mTR.setWithEulers(2,3,1,1,1,dyaw,dp,dr,atime-1000);
   //mTR.setWithEulers(2,3,1+1,1,1,dyaw,dp,dr,atime+1000);
  mTR.setWithEulers(3,5,dx,dy,dz,dyaw,dp,dr,atime);
  mTR.setWithEulers(5,1,dx,dy,dz,dyaw,dp,dr,atime);
  mTR.setWithEulers(6,5,dx,dy,dz,dyaw,dp,dr,atime);
  mTR.setWithEulers(6,5,dx,dy,dz,dyaw,dp,dr,atime);
  mTR.setWithEulers(7,6,1,1,1,dyaw,dp,dr,atime);
  mTR.setWithDH(8,7,1.0,1.0,1.0,dyaw,atime);
  //mTR.setWithEulers(8,7,1,1,1,dyaw,dp,dr,atime); //Switching out for DH params above
  
  
  //Demonstrate InvalidFrame LookupException
  try
    {
      std::cout<< mTR.viewChain(10,9);
    }
  catch (TransformReference::LookupException &ex)
    {
      std::cout << "Caught " << ex.what()<<std::endl;
    }
  
  
  // See the list of transforms to get between the frames
  std::cout<<"Viewing (10,8):"<<std::endl;  
  std::cout << mTR.viewChain(10,8);
  
  
  //See the resultant transform
  std::cout <<"Calling getMatrix(10,8)"<<std::endl;
  //      NEWMAT::Matrix mat = mTR.getMatrix(1,1);
  NEWMAT::Matrix mat = mTR.getMatrix(10,8,atime);
  
  std::cout << "Result of getMatrix(10,8,atime):" << std::endl << mat<< std::endl;

  TFPoint mPoint;
  mPoint.x = 1;
  mPoint.y = 1;
  mPoint.z = 1;
  mPoint.frame = 10;

  TFPoint nPoint = mPoint;

  std::cout <<"Point 1,1,1 goes like this:" <<std::endl;
  std::cout << "(" << mPoint.x <<","<< mPoint.y << "," << mPoint.z << ") in frame " << mPoint.frame << std::endl;
  mPoint =  mTR.transformPoint(2, mPoint);
  std::cout << "(" << mPoint.x <<","<< mPoint.y << "," << mPoint.z << ") in frame " << mPoint.frame << std::endl;
  mPoint =  mTR.transformPoint(3, mPoint);
  std::cout << "(" << mPoint.x <<","<< mPoint.y << "," << mPoint.z << ") in frame " << mPoint.frame << std::endl;
  mPoint =  mTR.transformPoint(5, mPoint);
  std::cout << "(" << mPoint.x <<","<< mPoint.y << "," << mPoint.z << ") in frame " << mPoint.frame << std::endl;
  mPoint =  mTR.transformPoint(6, mPoint);
  std::cout << "(" << mPoint.x <<","<< mPoint.y << "," << mPoint.z << ") in frame " << mPoint.frame << std::endl;
  mPoint =  mTR.transformPoint(7, mPoint);
  std::cout << "(" << mPoint.x <<","<< mPoint.y << "," << mPoint.z << ") in frame " << mPoint.frame << std::endl;
  mPoint =  mTR.transformPoint(8, mPoint);
  std::cout << "(" << mPoint.x <<","<< mPoint.y << "," << mPoint.z << ") in frame " << mPoint.frame << std::endl;
  mPoint =  mTR.transformPoint(10, mPoint);
  std::cout << "(" << mPoint.x <<","<< mPoint.y << "," << mPoint.z << ") in frame " << mPoint.frame << std::endl;

  //Break the graph, making it loop and demonstrate catching MaxDepthException
  mTR.setWithEulers(6,7,dx,dy,dz,dyaw,dp,dr,atime);
  
  try {
    std::cout<<mTR.viewChain(10,8);
  }
  catch (TransformReference::MaxDepthException &ex)
    {
      std::cout <<"caught loop in graph"<<std::endl;
    }
  
  //Break the graph, making it disconnected, and demonstrate catching ConnectivityException
  mTR.setWithEulers(6,0,dx,dy,dz,dyaw,dp,dr,atime);
  
  try {
    std::cout<<mTR.viewChain(10,8);
  }
  catch (TransformReference::ConnectivityException &ex)
    {
      std::cout <<"caught unconnected frame"<<std::endl;
    }

  //Testing clearing the history with parent change
  mTR.setWithEulers(7,5,1,1,1,dyaw,dp,dr,atime);
  //todo display this somehow


  std::cout << " Testing accessors" <<std::endl;

  double x,y,z,yaw,pitch,roll;

  x = 30.0;
  y = 10.0;
  z = 0.0;
  yaw = 0.0;
  pitch = 0.0;
  roll = 0.0;

  mTR.setWithEulers(2,1,x,y,z,yaw,pitch,roll,atime);

  libTF::TFPose2D in, out;

  in.x = 0.0;
  in.y = 0.0;
  in.yaw = 0.0;
  in.frame = 2;
  in.time = atime;

  out = mTR.transformPose2D(1,in);

  printf("%.3f %.3f %.3f\n",
         out.x, out.y, out.yaw*180.0/M_PI);

  
  return 0;
};
