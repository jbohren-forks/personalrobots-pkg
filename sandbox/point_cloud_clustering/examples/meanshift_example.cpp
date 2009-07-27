#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/cxcore.hpp>
#include <opencv/cvaux.hpp>

#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/kdtree/kdtree_flann.h>

#include <point_cloud_clustering/meanshift_clustering.h>

using namespace std;

// --------------------------------------------------------------
/*!
 * \brief Creates PointCloud datastructure from file
 *
 * File format: x y z label 2
 */
// --------------------------------------------------------------
int loadPointCloud(string filename, robot_msgs::PointCloud& pt_cloud, vector<unsigned int>& labels)
{
  // ----------------------------------------------
  // Open file
  ifstream infile(filename.c_str());
  if (infile.is_open() == false)
  {
    ROS_ERROR("Could not open filename: %s", filename.c_str());
    return -1;
  }

  // ----------------------------------------------
  // Count number of lines in the file
  unsigned int nbr_samples = 0;
  char line[256];
  while (infile.getline(line, 256))
  {
    nbr_samples++;
  }

  infile.clear();
  infile.seekg(ios::beg);

  // ----------------------------------------------
  // Resize containers appropriately
  pt_cloud.pts.resize(nbr_samples);
  labels.resize(nbr_samples);

  // ----------------------------------------------
  // Read file
  // file format: x y z label 2
  unsigned int tempo;
  for (unsigned int i = 0 ; i < nbr_samples ; i++)
  {
    infile >> pt_cloud.pts[i].x;
    infile >> pt_cloud.pts[i].y;
    infile >> pt_cloud.pts[i].z;
    infile >> labels[i];
    infile >> tempo;
  }

  infile.close();
  return 0;
}

int main()
{
  /*
   int n = 100000;
   int d = 3;
   double poop[n * d];
   for (int i = 0 ; i < n ; i++)
   {
   for (int j = 0 ; j < d ; j++)
   {
   poop[i*d + j] = static_cast<double>(rand() % 10);
   }
   }
   */

  robot_msgs::PointCloud pt_cloud;
  vector<unsigned int> poop;
  loadPointCloud("pt_cloud_260.xyz_label_conf", pt_cloud, poop);
  cloud_kdtree::KdTreeFLANN pt_cloud_kdtree(pt_cloud);

  //double labels[n];
  vector<double> labels;

  double bandwidth = 0.37;
  int max_iter = 2;
  double rate = 0.7;

  map<unsigned int, vector<int> > cluster_pt_indices;
  time_t start, end;
  time(&start);
  point_cloud_clustering::pcMeanshift(pt_cloud, pt_cloud_kdtree, bandwidth, rate, max_iter,
      cluster_pt_indices);
  time(&end);
  cout << "clustering took this long: " << difftime(end, start) << endl;

}
