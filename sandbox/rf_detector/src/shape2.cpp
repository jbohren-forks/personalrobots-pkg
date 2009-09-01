#include "shape2.h"
#include "write.h"

using namespace cv;
using namespace std;

void shape::compute( const sensor_msgs::PointCloud& pcd_pc, Vector< geometry_msgs::Point32>& interest_pts, 
	cv::Vector<cv::Vector<float> >& all_descriptor_results)
{
    unsigned int num_interest_pts = interest_pts.size();
    cout << "num_interest_pts" << num_interest_pts <<endl;
    all_descriptor_results.resize(num_interest_pts);
    cloud_kdtree::KdTreeANN pcd_pc_kdtree( pcd_pc);

    Vector<const geometry_msgs::Point32*> interest_pts_tmp(num_interest_pts);
    for (size_t i = 0 ; i < num_interest_pts ; i++)
    {
        interest_pts_tmp[i] = &(interest_pts[i]);
    }

    SpectralAnalysis::SpectralAnalysis sa(NRadius);
    ShapeSpectral::ShapeSpectral shape_spectral(sa);

    Descriptor3D* descriptors_3d = &shape_spectral;
    double t = (double)cv::getTickCount();
    descriptors_3d->compute(pcd_pc, pcd_pc_kdtree, interest_pts_tmp, all_descriptor_results);
    t = (double)cv::getTickCount() - t;
    printf("extract shape spectral feature time = %gms\n", t*1000./cv::getTickFrequency());
}
