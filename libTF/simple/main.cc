#include "libTF.hh"
#include <time.h>

using namespace std;

int main(void)
{
  double dx,dy,dz,dyaw,dp,dr;
  TransformReference mTR;
  
      
  //  while (true)
    {
      /*      cout << "delta x:";
      cin >> dx;
      cout << "delta y:";
      cin >> dy;
      cout << "delta z:";
      cin >> dz;
      cout << "delta yaw:";
      cin >> dyaw;
      cout << "delta pitch:";
      cin >> dp;
      cout << "delta roll:";
      cin >> dr;
      */
      dx = dy= dz = 0;
      dyaw = dp = dr = .1;
           
      
      
      mTR.set(1,2,1,1,1,dyaw,dp,dr);
      mTR.set(2,3,1,1,1,dyaw,dp,dr);
      mTR.set(3,5,dx,dy,dz,dyaw,dp,dr);
      mTR.set(5,0,dx,dy,dz,dyaw,dp,dr);
      mTR.set(6,5,dx,dy,dz,dyaw,dp,dr);
      mTR.set(7,6,1,1,1,dyaw,dp,dr);
      mTR.set(8,7,1,1,1,dyaw,dp,dr);
      mTR.view(1,8);


      std::cout <<"Calling get(1,8)"<<std::endl;
      //      NEWMAT::Matrix mat = mTR.get(1,1);
      NEWMAT::Matrix mat = mTR.get(1,8);

      std::cout << "Result" << std::endl << mat<< std::endl;
#ifdef DONOTUSE

      //      NEWMAT::Matrix mat(4,4);
      NEWMAT::Matrix MatI(4,4);
      NEWMAT::Matrix MatI2(4,4);
      

      //      double vec[] = {dx,dy,dz,dyaw,dp,dr};
      // mat <<vec;
      double init_time = (double)clock()/(double) CLOCKS_PER_SEC;
      int iterations = 1;
      for (int i = 0; i < iterations; i++)
	{
	  MatI = mat.i();
	}
      std::cout <<"Straight Inverse" <<std::endl<< MatI;
      std::cout << "This should be Identity if Inverse is right: "<<std::endl << MatI * mat;
      std::cout << "Time Elapsed for "<<iterations<<"iterations is:"<<(double)clock()/(double) CLOCKS_PER_SEC - init_time <<std::endl;
      std::cout << "Net: "<<((double)clock()/(double) CLOCKS_PER_SEC -init_time)/(double)iterations <<std::endl;


      NEWMAT::Matrix trans_mask(4,4);
      trans_mask << 0 << 0 << 0 << -1
		 << 0 << 0 << 0 << -1
		 << 0 << 0 << 0 << -1
		 << 0 << 0 << 0 << 1;

      NEWMAT::Matrix trans_fix(4,4);
      trans_fix << 1 << 0 << 0 << 0
		<< 0 << 1 << 0 << 0
		<< 0 << 0 << 1 << 0
		<< 0 << 0 << 0 << 0;
      

      NEWMAT::Matrix rot_mask(4,4);
      rot_mask << 1 << 1 << 1 << 0
	       << 1 << 1 << 1 << 0
	       << 1 << 1 << 1 << 0
	       << 0 << 0 << 0 << 1;

      init_time = (double)clock()/(double) CLOCKS_PER_SEC;
      for (int i = 0; i < iterations; i++)
	{
	  MatI2 =   SP(mat, rot_mask).t();
	  MatI2 *= (SP(mat,trans_mask) + trans_fix);
	}
      std::cout <<"My Inverse:" <<std::endl<< MatI2;
      std::cout << "Product: " <<std::endl<< MatI2 * mat;
      std::cout << "Time Elapsed for method 2 "<<iterations<<" iterations is:"<<(double)clock()/(double) CLOCKS_PER_SEC - init_time <<std::endl;
      std::cout << "Net: "<<((double)clock()/(double) CLOCKS_PER_SEC - init_time) /(double)iterations <<std::endl;
      

      std::cout <<"difference: "<<std::endl<<MatI - MatI2;
#endif //DONOTUSE

    }
  return 0;
};
