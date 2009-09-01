#include <iostream>
#include <fstream>
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include "cvaux.h"
#include <math.h>
#include "RandomForest.h"
#include "point_cloud_mapping/cloud_io.h"
#include "point_cloud_mapping/kdtree/kdtree_ann.h"
#include "descriptors_3d/all_descriptors.h"
#include "write.h"

using namespace cv;
using namespace std;

class shape{
public:
    shape(): NRadius(0.02){}
    shape( double NRadius_ ):
        NRadius( NRadius_){ }
    ~shape(){}

    // methods
    // for detection task
    void compute( const sensor_msgs::PointCloud& pcd_pc, Vector< geometry_msgs::Point32>& interest_pts, 
	cv::Vector<cv::Vector<float> >& all_descriptor_results);

    // properties
    double NRadius;

    // descriptor 3d stuffs
    SpectralAnalysis::SpectralAnalysis sa;
    ShapeSpectral::ShapeSpectral shape_spectral;
    sensor_msgs::PointCloud pcd_pc;
    Vector< geometry_msgs::Point32> interest_pts;
};
