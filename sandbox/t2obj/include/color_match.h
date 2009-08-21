#include <math.h>
#include <algorithm>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv/cvaux.h"
#include "opencv_latest/CvBridge.h"
#include "robot_msgs/PointCloud.h"
#include "robot_msgs/Point32.h"
#include "robot_msgs/PointStamped.h"

#include <fstream>
#include <sstream>
using namespace std;
using namespace cv;

bool read2DFloatText( string filename, vector< vector<float> > & data);
float DistGRB(float R, float G, float B, vector<float> exemplar);
void color_match_neighbor( vector<vector<float> > exemplars, IplImage* rcalimage, vector<robot_msgs::Point> candidates, vector< int> cand_ids, double threshold, int radius);
