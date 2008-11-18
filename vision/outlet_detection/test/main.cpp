// Main file to run outlet detection
// Gary Bradski 11/13/08   (c) Willow Garage, 2008
//
#include <cstdio>
#include <iterator>
#include <algorithm>
#include <cv.h>
#include <highgui.h>
#include <cvwimage.h>
#include "TrainBase.h"
#include "star_detector/detector.h"
#include "star_detector/keypoint.h"
#include "boost/foreach.hpp"
#include <fstream>
#include "mcutils/mcutils.h"
#include <climits>
#include <string> 

using namespace features;
using namespace std;

int main( int argc, char** argv )
{
  assert(argc > 1);

  TrainBase tbase;
  printf("Simple view 1\n");
  tbase.view(std::string(argv[1]),1000);
/*
 printf("Simple view 2\n");
  tbase.view(std::string(argv[1]),1000);

 printf("Train base\n");

  float ret = tbase.train(std::string(argv[1]), 100, 20, 50, 1000, 0.2, 0.3, 0.2); 
  std::cout << "ret = " << ret << std::endl;
  */
return 0;
}
 /*float TrainBase::train(std::string img_url, int num_trees, 
                       int depth, int views, int base_sz,
                       float phi_minmax, float theta_minmax, float lambda_plusminus)*/
