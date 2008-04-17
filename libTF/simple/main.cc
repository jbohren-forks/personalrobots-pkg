#include "Quaternion3D.hh"
#include "libTF.hh"
#include <time.h>

using namespace std;

int main(void)
{
  double dx,dy,dz,dyaw,dp,dr;
  TransformReference mTR;
  
      
  
  dx = dy= dz = 0;
  dyaw = dp = dr = .1;
  
  unsigned long long atime = Quaternion3D::Qgettime();

  
  //Fill in some transforms
  //  mTR.set(10,2,1,1,1,dyaw,dp,dr,atime); //Switching out for DH params below
  mTR.set(10,2,1,1,1,dyaw,atime);
  mTR.set(2,3,1,1,1,dyaw,dp,dr+1,atime-1000);
  mTR.set(2,3,1,1,1,dyaw,dp,dr-1,atime+1000);
  mTR.set(3,5,dx,dy,dz,dyaw,dp,dr,atime);
  mTR.set(5,1,dx,dy,dz,dyaw,dp,dr,atime);
  mTR.set(6,5,dx,dy,dz,dyaw,dp,dr,atime);
  mTR.set(7,6,1,1,1,dyaw,dp,dr,atime);
  mTR.set(8,7,1,1,1,dyaw,atime);
  //  mTR.set(8,7,1,1,1,dyaw,dp,dr,atime); //Switching out for DH params above
  
  
  //Demonstrate InvalidFrame LookupException
  try
    {
      mTR.view(10,9);
    }
  catch (TransformReference::LookupException &ex)
    {
      std::cout << "Caught " << ex.what()<<std::endl;
    }
  
  
  // See the list of transforms to get between the frames
  std::cout<<"Viewing (10,8):"<<std::endl;  
  mTR.view(10,8);
  
  
  //See the resultant transform
  std::cout <<"Calling get(10,8)"<<std::endl;
  //      NEWMAT::Matrix mat = mTR.get(1,1);
  NEWMAT::Matrix mat = mTR.get(10,8,atime);
  
  std::cout << "Result of get(10,8,atime):" << std::endl << mat<< std::endl;

  
  
  //Break the graph, making it loop and demonstrate catching MaxDepthException
  mTR.set(6,7,dx,dy,dz,dyaw,dp,dr,atime);
  
  try {
    mTR.view(10,8);
  }
  catch (TransformReference::MaxDepthException &ex)
    {
      std::cout <<"caught loop in graph"<<std::endl;
    }
  
  //Break the graph, making it disconnected, and demonstrate catching ConnectivityException
  mTR.set(6,0,dx,dy,dz,dyaw,dp,dr,atime);
  
  try {
    mTR.view(10,8);
  }
  catch (TransformReference::ConnectivityException &ex)
    {
      std::cout <<"caught unconnected frame"<<std::endl;
    }
  
  return 0;
};
