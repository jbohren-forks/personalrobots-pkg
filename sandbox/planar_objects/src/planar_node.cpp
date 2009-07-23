/*
 * planar_node.cpp
 *
 *  Created on: Jul 7, 2009
 *      Author: sturm
 */
#include "assert.h"
#include "planar_node.h"
#include "find_planes.h"
#include "vis_utils.h"

#include "opencv_latest/CvBridge.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "vis_utils.h"

using namespace ros;
using namespace std;
using namespace robot_msgs;
using namespace vis_utils;

#define CVWINDOW(a) cvNamedWindow(a,CV_WINDOW_AUTOSIZE); cvMoveWindow(a,(cvWindows /3 )* 500,(cvWindows%3)*500); cvWindows++;

// Constructor
PlanarNode::PlanarNode() :
  sync_(&PlanarNode::syncCallback, this)
{
  int cvWindows=0;
  CVWINDOW("left");
//  CVWINDOW("right");
  CVWINDOW("disparity");

  CVWINDOW("occupied");
  CVWINDOW("free");
  CVWINDOW("unknown");


  CVWINDOW("canny");
  CVWINDOW("free-dilated");
  CVWINDOW("canny-filtered");

  CVWINDOW("canny-filtered-dilated");
  CVWINDOW("hough");


  nh_.param("~n_planes_max", n_planes_max_, 4);
  nh_.param("~point_plane_distance", point_plane_distance_, 0.015);

  // subscribe to topics
  cloud_sub_ = nh_.subscribe("stereo/cloud", 1, sync_.synchronize(&PlanarNode::cloudCallback, this));
  disp_sub_ = nh_.subscribe("stereo/disparity", 1, sync_.synchronize(&PlanarNode::dispCallback, this) );
  dinfo_sub_ = nh_.subscribe("stereo/disparity_info", 1, sync_.synchronize(&PlanarNode::dinfoCallback, this) );
  limage_sub_ = nh_.subscribe("stereo/left/image_rect", 1, sync_.synchronize(&PlanarNode::limageCallback, this) );

  // advertise topics
  cloud_planes_pub_ = nh_.advertise<PointCloud> ("~planes", 1);
  cloud_outliers_pub_ = nh_.advertise<PointCloud> ("~outliers", 1);
  visualization_pub_ = nh_.advertise<visualization_msgs::Marker> ("visualization_marker", 1);

  currentTime = Time::now();
  lastTime = Time::now();
  lastDuration = Duration::Duration(0);
}

void PlanarNode::cloudCallback(const robot_msgs::PointCloud::ConstPtr& point_cloud)
{
  cloud_ = point_cloud;
}

void PlanarNode::dinfoCallback(const sensor_msgs::DisparityInfo::ConstPtr& dinfo) {
  dinfo_ = dinfo;
}

void PlanarNode::dispCallback(const sensor_msgs::Image::ConstPtr& disp_img)
{
  dimage_ = disp_img;

  if (dbridge_.fromImage(*dimage_))
  {
    // Disparity has to be scaled to be be nicely displayable
    IplImage* disp = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
    cvCvtScale(dbridge_.toIpl(), disp, 4.0/dinfo_->dpp);
    cvShowImage("disparity", disp);
    cvReleaseImage(&disp);
  }
}

void PlanarNode::limageCallback(const sensor_msgs::Image::ConstPtr& left_img)
{
  limage_ = left_img;

  if (lbridge_.fromImage(*limage_))
  {
    // Disparity has to be scaled to be be nicely displayable
    IplImage* disp = cvCreateImage(cvGetSize(lbridge_.toIpl()), IPL_DEPTH_8U, 1);
    cvShowImage("left", lbridge_.toIpl());
    cvReleaseImage(&disp);
  }
}

double inner_prod(std::vector<double> vec1,std::vector<double> vec2) {
  double sum=0;
  for(size_t i=0;i<3;i++) {
    sum += vec1[i]*vec2[i];
  }
  return(sum);
}

void PlanarNode::syncCallback()
{
  ROS_INFO("PlanarNode::syncCallback(), %d points in cloud",cloud_->get_pts_size());

  currentTime = Time::now();
  if(lastTime + lastDuration*1.5 > currentTime) {
    ROS_INFO("skipping frame..");
    return;
  }

  vector<PointCloud> plane_cloud;
  vector<vector<double> > plane_coeff;
  vector<vector<int> > plane_indices;
  PointCloud outside;

  find_planes::findPlanes(*cloud_, n_planes_max_, point_plane_distance_, plane_indices, plane_cloud, plane_coeff,outside);

  int n=plane_coeff.size();
  int backplane = 0; // assume that the largest plane belongs to the background
  int frontplane = 1;
  double ip3=inner_prod(plane_coeff[backplane],plane_coeff[backplane]);
  ROS_INFO("backplane %d, ip %f", backplane, ip3);

  for(int i=0;i<n;i++) {
    ROS_INFO ("plane %d: %d inliers: [%g, %g, %g, %g]", i, plane_indices[i].size(),
              plane_coeff[i][0], plane_coeff[i][1], plane_coeff[i][2], plane_coeff[i][3]);
//    ROS_INFO("i=%d, prod=%f",i,inner_prod(plane_coeff[backplane],plane_coeff[i]));
  }

  double ip=inner_prod(plane_coeff[backplane],plane_coeff[frontplane]);
  int i=frontplane+1;
  ROS_INFO("backplane %d, frontplane %d, ip %f", backplane, frontplane,ip);
  while(i<n) {
    double ip2 = inner_prod(plane_coeff[backplane],plane_coeff[i]);
    if(ip2 > ip && plane_coeff[backplane][3] > plane_coeff[i][3]) {
      frontplane = i;
      ip = ip2;
    }
    ROS_INFO("backplane %d, frontplane %d, ip %f, i %d, ip2 %f", backplane, frontplane,ip,i,ip2);
    i++;
  }
  ROS_INFO("backplane %d, frontplane %d", backplane, frontplane);

  std::vector<float> plane_color;
  plane_color.resize(n);
  for(int i=0;i<n;i++) {
    plane_color[i] = HSV_to_RGBf(0.7,0,0.3);
  }
  plane_color[backplane] = HSV_to_RGBf(0.3,0.3,1);
  plane_color[frontplane] = HSV_to_RGBf(0.,0.3,1);

  vis_utils::visualizePlanes(*cloud_,plane_indices,plane_cloud,plane_coeff,plane_color,outside,cloud_planes_pub_,visualization_pub_);

  // occupied pixels in plane
  IplImage* pixOccupied = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);

  // free pixels in plane (because something sensed behind plane)
  IplImage* pixFree = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);

  // unknown/undefined pixels in plane
  IplImage* pixUnknown = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);

  find_planes::createPlaneImage(*cloud_, plane_indices[frontplane], plane_coeff[frontplane],
                                pixOccupied,pixFree,pixUnknown);

  cvShowImage( "occupied", pixOccupied );
  cvShowImage( "free", pixFree );
  cvShowImage( "unknown", pixUnknown );

//  IplImage* pixMerged = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 3);
//  cvMerge(pixOccupied,pixFree,pixUnknown,NULL,pixMerged);
//  cvShowImage( "merged", pixMerged);

  // canny edge image of occupied pixels
  IplImage* pixCanny = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  cvCanny( pixOccupied, pixCanny, 50, 200, 3 );
  cvShowImage( "canny", pixCanny );

  // dilate (increase) free pixels
  IplImage* pixFreeDilated = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  cvDilate(pixFree,pixFreeDilated,NULL,10);
  cvShowImage( "free-dilated", pixFreeDilated );

  // only keep edges that lie substantially close to free pixels
  IplImage* pixCannyFiltered = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  cvAnd(pixCanny,pixFreeDilated,pixCannyFiltered);
  cvShowImage( "canny-filtered", pixCannyFiltered );

  // dilate filtered canny edge image..
  IplImage* pixCannyFilteredAndDilated = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  cvDilate(pixCannyFiltered,pixCannyFilteredAndDilated,NULL,2);
  cvShowImage( "canny-filtered-dilated", pixCannyFilteredAndDilated );

  // find edges via hough transform
  IplImage* pixHough = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 3);
  cvCvtColor( lbridge_.toIpl(), pixHough, CV_GRAY2BGR );

  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* lines = 0;
  lines = cvHoughLines2( pixCannyFilteredAndDilated, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 5, 30, 15 );
  for(int i = 0; i < lines->total; i++ )
  {
      CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
      cvLine( pixHough, line[0], line[1], CV_RGB(rand()%255,rand()%255,rand()%255), 1, 8 );
  }
////  cvClearMemStorage(storage);
////  cvReleaseMemStorage(&storage);
  cvShowImage("hough",pixHough);

  cvReleaseImage(&pixOccupied);
  cvReleaseImage(&pixFree);
  cvReleaseImage(&pixUnknown);
  cvReleaseImage(&pixCanny);
  cvReleaseImage(&pixFreeDilated);
  cvReleaseImage(&pixCannyFiltered);
  cvReleaseImage(&pixCannyFilteredAndDilated);
  cvReleaseImage(&pixHough);

  lastDuration = Time::now() - currentTime;
  lastTime = currentTime;

  ROS_INFO("processing took %g s",lastDuration.toSec());
}

bool PlanarNode::spin()
{
  while (nh_.ok())
  {
    int key = cvWaitKey(100) & 0x00FF;
    if (key == 27) //ESC
      break;

    ros::spinOnce();
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planar_node");

  PlanarNode node;
  node.spin();

  return 0;
}


/*
 *


  // the depth image
  IplImage* imgInPlane = cvCreateImage( cvGetSize(planeImage), planeImage->depth, planeImage->nChannels );


  // Reduce the image by 2
//  IplImage* out = cvCreateImage( cvSize(planeImage->width/2,planeImage->height/2), planeImage->depth, planeImage->nChannels );
  IplImage* out = cvCreateImage( cvGetSize(planeImage), planeImage->depth, planeImage->nChannels );
//  cvPyrDown( planeImage, out );

//  cvShowImage("plane2", planeImage);
//  int rows=3;
//  int columns=3;
//  IplConvKernel* structuringElement; // open/close structuring element
//
//  structuringElement = cvCreateStructuringElementEx(rows, columns,
//                cvFloor(rows / 2), cvFloor(columns / 2), CV_SHAPE_RECT, NULL);
//
//  int iterations=10;
//
//  cvMorphologyEx(planeImage, planeImage, NULL, structuringElement,
//                                                                                        CV_MOP_OPEN, iterations);
//
////  cvMorphologyEx(planeImage, planeImage, NULL, structuringElement,
////                                                                                        CV_MOP_CLOSE, iterations);
//
//  cvReleaseStructuringElement(&structuringElement);

  cvSmooth( planeImage, out, CV_GAUSSIAN, 1, 1 );

  IplImage* dst = cvCreateImage( cvGetSize(out), 8, 1 );
  IplImage* color_dst = cvCreateImage( cvGetSize(out), 8, 3 );
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* lines = 0;

  cvCanny( out, dst, 50, 200, 3 );
  cvDilate(dst,dst);

  IplImage* dist = cvCreateImage( cvGetSize(out), 8, 1 );
  uchar *data;

  cvCvtColor( dst, color_dst, CV_GRAY2BGR );
  lines = cvHoughLines2( dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 5, 30, 15 );
  for(int i = 0; i < lines->total; i++ )
  {
      CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
      cvLine( color_dst, line[0], line[1], CV_RGB(255,0,0), 3, 8 );
  }
  data      = (uchar *)dst->imageData;
  for(int i=0;i<dst->height;i++) for(int j=0;j<dst->width;j++) for(int k=0;k<dst->nChannels;k++)
    data[i*dst->widthStep+j*dst->nChannels+k]=255-data[i*dst->widthStep+j*dst->nChannels+k];

  cvDistTransform(dst, dist, CV_DIST_L1,3);

  for(int i = 0; i < lines->total; i++ )
  {
      CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);

      CvLineIterator iterator;
      int max_buffer = cvInitLineIterator(dist,line[0],line[1],&iterator,8,0);
      double d_sum=0;
      double d_max=0;
      for(int j=0;j<max_buffer;j++) {
        d_sum += iterator.ptr[0];
        d_max = MAX(d_max,iterator.ptr[0]);
        CV_NEXT_LINE_POINT(iterator);
      }
      cvLine( color_dst, line[0], line[1], CV_RGB(255-MIN(255,d_max*20),0,0), 3, 8 );

      cout << "line" << i <<" has d_sum="<<d_sum <<" d_max="<<d_max<<" and max_buffer="<<max_buffer<< endl;

  }

//
//
//  char outFileName[50];
//  sprintf (outFileName, "/tmp/plane-%.5d.png", cloud_->header.seq);
//  printf("'%s'\n",outFileName);
//  if(!cvSaveImage(outFileName,planeImage)) printf("Could not save: %s\n",outFileName);
//  sprintf (outFileName, "/tmp/canny-%.5d.png", cloud_->header.seq);
//  if(!cvSaveImage(outFileName,dst)) printf("Could not save: %s\n",outFileName);
//  sprintf (outFileName, "/tmp/hough-%.5d.png", cloud_->header.seq);
//  if(!cvSaveImage(outFileName,color_dst)) printf("Could not save: %s\n",outFileName);

  cvShowImage("plane", planeImage);
  cvShowImage( "Canny", dst );
  cvShowImage( "Hough", color_dst );
  cvShowImage( "Dist", dist );

  cvReleaseImage(&dst);
  cvReleaseImage(&color_dst);
  cvReleaseImage(&planeImage);


 */
