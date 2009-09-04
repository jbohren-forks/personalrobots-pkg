/*
 * box_detector.cpp
 *
 *  Created on: Jul 7, 2009
 *      Author: sturm
 */
#include "assert.h"
#include "box_detector.h"
#include "find_planes.h"
#include "vis_utils.h"

#include "opencv_latest/CvBridge.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "vis_utils.h"

using namespace ros;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "box_detector");

  planar_objects::BoxDetector node;
  node.spin();

  return 0;
}

namespace planar_objects
{

#define CVWINDOW(a) cvNamedWindow(a,CV_WINDOW_AUTOSIZE); cvMoveWindow(a,(cvWindows /3 )* 500,(cvWindows%3)*500); cvWindows++;
#define SQR(a) ((a)*(a))

// Constructor
BoxDetector::BoxDetector() :
  sync_(&BoxDetector::syncCallback, this)
{
  nh_.param("~n_planes_max", n_planes_max_, 5);
  nh_.param("~n_seeds_per_plane", n_seeds_per_plane_, 5);
  nh_.param("~point_plane_distance", point_plane_distance_, 0.01);

  nh_.param("~cost_pix_in_front", cost_pix_in_front, 0.1);
  nh_.param("~cost_pix_unknown", cost_pix_unknown, 0.2);

  nh_.param("~show_colorized_planes", show_colorized_planes, true);
  nh_.param("~show_convex_hulls", show_convex_hulls, false);
  nh_.param("~show_lines", show_lines, false);
  nh_.param("~show_images", show_images, true);
  nh_.param("~show_corners", show_corners, true);
  nh_.param("~show_rectangles", show_rectangles, true);
  nh_.param("~save_images", save_images, false);
  nh_.param("~save_images_matching", save_images_matching, false);

  nh_.param("~verbose", verbose, false);
  nh_.param("~scaling", scaling, 1.00);

  nh_.param("~select_frontplane", select_frontplane, -1);
//  cout << "select_frontplane="<<select_frontplane<<endl;
  nh_.param("~max_lines", max_lines, 200);
  nh_.param("~max_corners", max_corners, 500 );

  nh_.param("~min_precision", min_precision, 0.7);
  nh_.param("~min_recall", min_recall, 0.7);

  nh_.param("~rect_min_size", rect_min_size, 0.05);
  nh_.param("~rect_max_size", rect_max_size, 1.50);
  nh_.param("~rect_max_displace", rect_max_displace, 0.3);

  int cvWindows = 0;
  if(show_images) {
	    CVWINDOW("left");
    CVWINDOW("disparity");
    CVWINDOW("debug");
  }
  if(show_images && verbose) {
    CVWINDOW("right");

    CVWINDOW("occupied");
    CVWINDOW("distance");
    CVWINDOW("free");
    CVWINDOW("unknown");
    CVWINDOW("plane");

    CVWINDOW("canny");
    CVWINDOW("free-dilated");
    CVWINDOW("canny-filtered");

    CVWINDOW("canny-filtered-dilated");
    CVWINDOW("hough");
  }

  string stereo_ns = nh_.resolveName("stereo");
// subscribe to topics
  cloud_sub_ = nh_.subscribe(stereo_ns+"/cloud", 1, sync_.synchronize(&BoxDetector::cloudCallback, this));
  disp_sub_ = nh_.subscribe(stereo_ns+"/disparity", 1, sync_.synchronize(&BoxDetector::dispCallback, this));
  dinfo_sub_ = nh_.subscribe(stereo_ns+"/disparity_info", 1, sync_.synchronize(&BoxDetector::dinfoCallback, this));
  limage_sub_ = nh_.subscribe(stereo_ns+"/left/image_rect", 1, sync_.synchronize(&BoxDetector::limageCallback, this));
  rimage_sub_ = nh_.subscribe(stereo_ns+"/right/image_rect", 1, sync_.synchronize(&BoxDetector::rimageCallback, this));
  linfo_sub_ = nh_.subscribe(stereo_ns+"/left/cam_info", 1, sync_.synchronize(&BoxDetector::linfoCallback, this));
  rinfo_sub_ = nh_.subscribe(stereo_ns+"/right/cam_info", 1, sync_.synchronize(&BoxDetector::rinfoCallback, this));

  // advertise topics
  cloud_planes_pub_ = nh_.advertise<sensor_msgs::PointCloud> ("~planes", 1);
  visualization_pub_ = nh_.advertise<visualization_msgs::Marker> ("~visualization_marker", 100);
  observations_pub_ = nh_.advertise<BoxObservations> ("~observations", 100);

  currentTime = Time::now();
  lastTime = Time::now();
  lastDuration = Duration::Duration(0);
  frame=0;
}

void BoxDetector::cloudCallback(const sensor_msgs::PointCloud::ConstPtr& point_cloud)
{
  cloud_ = point_cloud;
}

void BoxDetector::dinfoCallback(const stereo_msgs::DisparityInfo::ConstPtr& dinfo)
{
  dinfo_ = dinfo;
}

void BoxDetector::linfoCallback(const sensor_msgs::CameraInfo::ConstPtr& linfo)
{
  linfo_ = linfo;
}

void BoxDetector::rinfoCallback(const sensor_msgs::CameraInfo::ConstPtr& rinfo)
{
  rinfo_ = rinfo;
}

void BoxDetector::dispCallback(const sensor_msgs::Image::ConstPtr& disp_img)
{
  dimage_ = disp_img;

  if (dbridge_.fromImage(*dimage_))
  {
    // Disparity has to be scaled to be be nicely displayable
    IplImage* disp = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
    cvCvtScale(dbridge_.toIpl(), disp, 4.0 / dinfo_->dpp);
    cvShowImage("disparity", disp);
    cvReleaseImage(&disp);
  }
}

void BoxDetector::limageCallback(const sensor_msgs::Image::ConstPtr& left_img)
{
  limage_ = left_img;

  if (lbridge_.fromImage(*limage_))
  {
    cvShowImage("left", lbridge_.toIpl());
  }
}

void BoxDetector::rimageCallback(const sensor_msgs::Image::ConstPtr& right_img)
{
  rimage_ = right_img;

  if (rbridge_.fromImage(*rimage_))
  {
    cvShowImage("right", rbridge_.toIpl());
  }
}

double inner_prod(std::vector<double> vec1, std::vector<double> vec2)
{
  double sum = 0;
  for (size_t i = 0; i < 3; i++)
  {
    sum += vec1[i] * vec2[i];
  }
  return (sum);
}

void BoxDetector::buildRP()
{
  double offx = 0;

  // reprojection matrix
  double Tx = rinfo_->P[0] / rinfo_->P[3];
  // first column
  RP[0] = 1.0;
  RP[4] = RP[8] = RP[12] = 0.0;

  // second column
  RP[5] = 1.0;
  RP[1] = RP[9] = RP[13] = 0.0;

  // third column
  RP[2] = RP[6] = RP[10] = 0.0;
  RP[14] = -Tx;

  // fourth column
  RP[3] = -linfo_->P[2]; // cx
  RP[7] = -linfo_->P[6]; // cy
  RP[11] = linfo_->P[0]; // fx, fy
  RP[15] = (linfo_->P[2] - rinfo_->P[2] - (double)offx) / Tx;

  for (int i = 0; i < 16; i++)
    P[i] = linfo_->P[i];

  //  for(int i=0;i<16;i++)
  //    printf("%d -- %f \n",i,P[i]);

}

btVector3 BoxDetector::calcPt(int x, int y, std::vector<double>& coeff)
{
  double cx = RP[3];
  double cy = RP[7];
  double f = RP[11];

  double ax = (double)x + cx;
  double ay = (double)y + cy;
  double aw = -coeff[3] / (ax * coeff[0] + ay * coeff[1] + f * coeff[2]);

  return (btVector3(ax * aw, ay * aw, f * aw));
}

bool sortCornersByAngle(const CornerCandidate& d1, const CornerCandidate& d2)
{
  return fabs(d1.angle - M_PI_2) < fabs(d2.angle - M_PI_2);
}

#define MEASURE(timeStamp,s) cout<<"["<<s<<"] took "<<(Time::now() - timeStamp).toSec()<<"s"<<endl; timeStamp = Time::now();

void BoxDetector::syncCallback()
{
  setVisualization(&visualization_pub_, &cloud_planes_pub_, cloud_->header);

  ROS_INFO("BoxDetector::syncCallback(), %d points in cloud, seq=%d",cloud_->get_points_size(),cloud_->header.seq);

  Time timeStamp = Time::now();

  currentTime = Time::now();
  if (lastTime + lastDuration * 0.1 > currentTime)
  {
    ROS_INFO("skipping frame..");
    return;
  }

  buildRP();

  findPlanes(*cloud_, n_planes_max_, point_plane_distance_*scaling, plane_indices, plane_cloud, plane_coeff,
                          outside);
  MEASURE(timeStamp,"findPlanes");

  if (show_colorized_planes)
    visualizePlanes(*cloud_, plane_indices, plane_cloud, plane_coeff, outside, show_convex_hulls);
  MEASURE(timeStamp,"visualizePlanes");

  pixOccupied = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  pixFree = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  pixUnknown = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  pixPlane = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  pixDist = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  pixDebug = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 3);
  cvCvtColor(lbridge_.toIpl(), pixDebug, CV_GRAY2BGR);

  std::vector<CornerCandidate> corners;
//  cout << "frontplane="<<select_frontplane<<endl;
  cout << "planes="<<plane_coeff.size()<<endl;
  for (size_t frontplane = 0; frontplane < plane_coeff.size(); frontplane++)
  {
    if (select_frontplane != -1)
    {
      if (select_frontplane >= 0 || select_frontplane < (int)plane_coeff.size())
      {
        ROS_INFO("selected frontplane %d.. ",select_frontplane);
        frontplane = select_frontplane;
      }
      else
      {
        ROS_ERROR("selected frontplane %d does not exist (n=%d planes)",select_frontplane,plane_coeff.size());
      }
    }
    cvSetZero(pixOccupied);
    cvSetZero(pixFree);
    cvSetZero(pixUnknown);
    cvSetZero(pixPlane);
    cvSetZero(pixDist);
    //  int frontplane=1;
    createPlaneImage(*cloud_, plane_indices[frontplane], plane_coeff[frontplane], pixOccupied, pixFree,
                                  pixUnknown,pixPlane,(int)((1-cost_pix_in_front)*255),(int)((1-cost_pix_unknown)*255));
    cvShowImage("occupied", pixOccupied);
    cvShowImage("free", pixFree);
    cvShowImage("unknown", pixUnknown);
    cvShowImage("plane", pixPlane);

    current_plane = frontplane;

    std::vector<CornerCandidate> corner;
    findCornerCandidates(pixOccupied, pixFree, pixUnknown, pixDist, plane_coeff[frontplane], plane_indices[frontplane],corner, frontplane * 1000);

    findRectangles(corner);

    corner = filterRectanglesBySupport2d(corner, pixOccupied, min_precision);
    corner = filterRectanglesBySupport3d(corner, *cloud_, plane_indices[frontplane], min_recall);

    //    cout << "corners in plane="<<frontplane<<" with high support: "<<corner.size()<<endl;;
    if (show_images)
      visualizeRectangles2d(corner,CV_RGB(0,255,0));
    if (show_corners)
      visualizeCorners(corner, frontplane * 1000);
    if (show_rectangles)
      visualizeRectangles3d(corner, frontplane * 1000);

    for (size_t i = 0; i < corner.size(); i++)
    {
      corner[i].precision = corner[i].computeSupport2d(pixOccupied,pixDebug);
      corner[i].recall = corner[i].computeSupport3d(*cloud_, plane_indices[frontplane]);
      corner[i].plane_id = frontplane;
    }

    corners.insert(corners.end(), corner.begin(), corner.end());

    MEASURE(timeStamp,"detectionLoop");

    if (select_frontplane != -1)
      break;
  }

  cout << "corner observations: " << corners.size() << endl;
  BoxObservations obs;
  obs.header = cloud_->header;
  obs.set_obs_size(corners.size());
  for (size_t i = 0; i < corners.size(); i++)
  {
    obs.obs[i].transform.rotation.w = corners[i].tf.getRotation().w();
    obs.obs[i].transform.rotation.x = corners[i].tf.getRotation().x();
    obs.obs[i].transform.rotation.y = corners[i].tf.getRotation().y();
    obs.obs[i].transform.rotation.z = corners[i].tf.getRotation().z();
    obs.obs[i].transform.translation.x = corners[i].tf.getOrigin().x();
    obs.obs[i].transform.translation.y = corners[i].tf.getOrigin().y();
    obs.obs[i].transform.translation.z = corners[i].tf.getOrigin().z();
    obs.obs[i].w = corners[i].w;
    obs.obs[i].h = corners[i].h;
    obs.obs[i].precision = corners[i].precision;
    obs.obs[i].recall = corners[i].recall;
    obs.obs[i].plane_id = corners[i].plane_id;
  }
  observations_pub_.publish(obs);

  if(save_images) {
    char buf[50];
    sprintf(buf,"/tmp/debug-%05d.png",cloud_->header.seq); cvSaveImage(buf,pixDebug);

    sprintf(buf,"/tmp/left-%05d.png",cloud_->header.seq); cvSaveImage(buf,lbridge_.toIpl());

    IplImage* disp = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
    cvCvtScale(dbridge_.toIpl(), disp, 4.0 / dinfo_->dpp);
    sprintf(buf,"/tmp/disparity-%05d.png",cloud_->header.seq); cvSaveImage(buf,disp);
    cvReleaseImage(&disp);
  }

  cvReleaseImage(&pixDist);
  cvReleaseImage(&pixOccupied);
  cvReleaseImage(&pixFree);
  cvReleaseImage(&pixUnknown);
  cvReleaseImage(&pixPlane);
  cvShowImage("debug", pixDebug);
  cvReleaseImage(&pixDebug);

  lastDuration = Time::now() - currentTime;
  ROS_INFO("processing took %g s",lastDuration.toSec());
  lastTime = currentTime;
}

bool BoxDetector::spin()
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

void BoxDetector::visualizePlanes(const sensor_msgs::PointCloud& cloud, std::vector<std::vector<int> >& plane_indices,
                                 std::vector<sensor_msgs::PointCloud>& plane_cloud,
                                 std::vector<std::vector<double> >& plane_coeff, sensor_msgs::PointCloud& outside,
                                 bool convexHull)
{
  std::vector<float> plane_color;
  plane_color.resize(plane_coeff.size());
  for (size_t i = 0; i < plane_coeff.size(); i++)
  {
    plane_color[i] = HSV_to_RGBf(i / (double)plane_coeff.size(), 0.3, 1.0);
  }
  visualizePlanes2(*cloud_, plane_indices, plane_cloud, plane_coeff, plane_color, outside,
                             convexHull);

}

void BoxDetector::visualizeCorners(std::vector<CornerCandidate> &corner, int id)
{
  std::vector<std::pair<btVector3, btVector3> > lines;

  lines.resize(corner.size());
  for (int j = 0; j < 3; j++)
  {
    btVector3 vec(j == 0 ? 1.00 : 0.00, j == 1 ? 1.00 : 0.00, j == 2 ? 1.00 : 0.00);
    for (size_t i = 0; i < corner.size(); i++)
    {
      lines[i].first = corner[i].tf.getOrigin();
      lines[i].second = corner[i].tf * (vec * 0.05);
    }
    visualizeLines(lines, id + 10 + j, j == 0 ? 1.00 : 0.00, j == 1 ? 1.00 : 0.00, j == 2 ? 1.00 : 0.00);
  }
}

void BoxDetector::visualizeRectangles3d(std::vector<CornerCandidate> &corner, int id)
{
  std::vector<std::pair<btVector3, btVector3> > lines;

  lines.resize(4);
  for (size_t i = 0; i < MIN(100, corner.size()); i++)
  {
    corner[i].updatePoints3d();
    lines[0].first = corner[i].points3d[0];
    lines[0].second = corner[i].points3d[1];

    lines[1].first = corner[i].points3d[1];
    lines[1].second = corner[i].points3d[2];

    lines[2].first = corner[i].points3d[2];
    lines[2].second = corner[i].points3d[3];

    lines[3].first = corner[i].points3d[3];
    lines[3].second = corner[i].points3d[0];

    visualizeLines(lines, id + 100 + i, MIN(i*0.1,1.00), MAX(0.00, 1.00-i*.01), 0.00,0.002 * scaling);
  }
  lines.resize(0);
  for (size_t i = corner.size(); i < 100; i++)
    visualizeLines(lines, id + 100 + i,1.0,1.0,1.0);
}

void BoxDetector::visualizeRectangles2d(std::vector<CornerCandidate> &corner,CvScalar col)
{
  for (size_t i = 0; i < corner.size(); i++)
  {
    visualizeRectangle2d(corner[i],col);
  }
}

void BoxDetector::visualizeRectangle2d(CornerCandidate &corner,CvScalar col)
{
  corner.updatePoints3d();
  corner.updatePoints2d();

  cvCircle(pixDebug,corner.points2d[0],5,col);
  cvCircle(pixDebug,corner.points2d[2],3,col);
  for (int j = 0; j < 4; j++)
  {
    cvLine(pixDebug, corner.points2d[j], corner.points2d[(j + 1) % 4], col, 1, 8);
    //      cout << "j="<<j<<" x="<< corner[i].points2d[j].x <<" y="<<corner[i].points2d[j].y << endl;
  }
}

void BoxDetector::findRectangles(std::vector<CornerCandidate> &corner)
{
  IplImage* pixSave;
  if(save_images_matching) {
    pixSave = cvCreateImage(cvGetSize(pixDebug), IPL_DEPTH_8U, 3);
    cvCopy(pixDebug,pixSave);
  }

  for(int x=0;x<pixDebug->width;x++) {
    for(int y=0;y<pixDebug->height;y++) {
      if(((uchar*)pixOccupied->imageData)[y * pixOccupied->widthStep + x * pixOccupied->nChannels]>0) {
        ((uchar*)pixDebug->imageData)[y * pixDebug->widthStep + x * pixDebug->nChannels] =
          255 - (255 - ((uchar*)pixDebug->imageData)[y * pixDebug->widthStep + x * pixDebug->nChannels])/2;
      }
    }
  }

  for (size_t i = 0; i < corner.size(); i++)
  {
    initializeRectangle(corner[i]);

    CvScalar col;
    if( (corner[i].computeSupport2d(pixOccupied) >= min_precision)
        & (corner[i].computeSupport3d(*cloud_,plane_indices[current_plane]) >= min_recall)) {
      col = CV_RGB(0,255,0);
    } else {
      col = CV_RGB(255,0,0);
    }


    for (int j = 0; j < 4; j++)
      cvLine(pixDebug, corner[i].points2d[j], corner[i].points2d[(j + 1) % 4], col, 1, 8);

    if(save_images_matching) {
      char buf[50];
      sprintf(buf,"/tmp/debug-%05d.png",frame++); cvSaveImage(buf,pixDebug);
    }
  }
  if(save_images_matching) {
    cvCopy(pixSave,pixDebug);
    cvReleaseImage(&pixSave);
  }
}

void BoxDetector::initializeRectangle(CornerCandidate &corner)
{
  corner.w = 0.2*scaling;
  corner.h = 0.2*scaling;
  visualizeRectangle2d(corner,CV_RGB(255,255,255));
  for (int i = 0; i < 20; i++)
  {
    for(int c=0;c<2;c++) {
      corner.optimizeWidth2(pixPlane, -0.05*scaling,+0.05*scaling, 5,c);
      corner.optimizeHeight2(pixPlane, -0.05*scaling,+0.05*scaling, 5,c);
    }
    corner.optimizePhi(pixPlane, -M_PI / (16), +M_PI / (16), 5);
    visualizeRectangle2d(corner,CV_RGB(255-3*i,255-3*i,255-3*i));

    if(save_images_matching) {
      char buf[50];
      sprintf(buf,"/tmp/debug-%05d.png",frame++); cvSaveImage(buf,pixDebug);
    }
  }
}

std::vector<CornerCandidate> BoxDetector::filterRectanglesBySupport2d(std::vector<CornerCandidate> &corner,
                                                                     IplImage* pixOccupied, double min_support)
{
  std::vector<CornerCandidate> result;
  for (size_t i = 0; i < corner.size(); i++)
  {
    if (corner[i].computeSupport2d(pixOccupied) >= min_support)
      result.push_back(corner[i]);
  }
  return (result);
}

std::vector<CornerCandidate> BoxDetector::filterRectanglesBySupport3d(std::vector<CornerCandidate> &corner,
                                                                     const sensor_msgs::PointCloud& cloud, std::vector<
                                                                         int> & plane_indices, double min_support)
{
  std::vector<CornerCandidate> result;
  for (size_t i = 0; i < corner.size(); i++)
  {
    if (corner[i].computeSupport3d(cloud, plane_indices) >= min_support)
      result.push_back(corner[i]);
  }
  return (result);
}

void BoxDetector::findCornerCandidates(IplImage* pixOccupied, IplImage *pixFree, IplImage* pixUnknown,
                                      IplImage* &pixCannyDist, vector<double>& plane_coeff, vector<int>& plane_indices,
                                      std::vector<CornerCandidate> &corner, int id)
{
  // dilate (increase) free pixels
  IplImage* pixFreeDilated = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  cvDilate(pixFree, pixFreeDilated, NULL, 10);
  cvShowImage("free-dilated", pixFreeDilated);

  // canny edge image of occupied pixels
  IplImage* pixCanny = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  cvCanny(pixOccupied, pixCanny, 50, 200, 3);
  cvShowImage("canny", pixCanny);

  // only keep edges that lie substantially close to free pixels
  IplImage* pixCannyFiltered = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  cvAnd(pixCanny, pixFreeDilated, pixCannyFiltered);
  cvShowImage("canny-filtered", pixCannyFiltered);

  // dilate filtered canny edge image..
  IplImage* pixCannyFilteredAndDilated = cvCreateImage(cvGetSize(dbridge_.toIpl()), IPL_DEPTH_8U, 1);
  cvDilate(pixCannyFiltered, pixCannyFilteredAndDilated, NULL, 1);
  cvShowImage("canny-filtered-dilated", pixCannyFilteredAndDilated);

  // compute corner candidates
  btVector3 vecPlane(plane_coeff[0], plane_coeff[1], plane_coeff[2]);
  CornerCandidate candidate;
  candidate.RP = RP;
  candidate.P = P;
  candidate.rect_max_displace = rect_max_displace;
  candidate.rect_min_size = rect_min_size*scaling;
  candidate.rect_max_size = rect_max_size*scaling;
  candidate.w = rect_min_size*scaling;
  candidate.h = rect_min_size*scaling;

  for(int iter=0;iter<n_seeds_per_plane_;iter++) {
    cout << iter << endl;
    // pick 3 inliers
    int p[3];
    p[0] = p[1] = p[2] = plane_indices[random() % plane_indices.size()];
    while(p[0]==p[1]) p[1] = plane_indices[random() % plane_indices.size()];
    while(p[0]==p[2] || p[0]==p[2]) p[2] = plane_indices[random() % plane_indices.size()];

    // now shift inliers to maximize surface
    btVector3 position(cloud_->points[p[0]].x,cloud_->points[p[0]].y,cloud_->points[p[0]].z);
    btVector3 vec1(cloud_->points[p[1]].x,cloud_->points[p[1]].y,cloud_->points[p[1]].z);
    vec1 -= position;
    vec1 = vec1.normalize();

    vec1 -= vecPlane * vecPlane.dot(vec1);

    btVector3 vec2 = vecPlane.cross(vec1);

    // orientation
    btQuaternion orientation;
    btMatrix3x3 rotation;
    rotation[0] = vec1; // x
    rotation[1] = vec2; // y
    rotation[2] = vecPlane; // z

    rotation = rotation.transpose();
    rotation.getRotation(orientation);
    rotation.setRotation(orientation);
    candidate.tf = btTransform(orientation, position);

    candidate.updatePoints3d();
    candidate.updatePoints2d();

    candidate.plane_id = current_plane;

    corner.push_back(candidate);
  }

  cvNot(pixCanny, pixCanny);
  cvDistTransform(pixCanny, pixCannyDist, CV_DIST_L1);
  cvShowImage("distance", pixCannyDist);

  cvReleaseImage(&pixFreeDilated);
  cvReleaseImage(&pixCanny);
  cvReleaseImage(&pixCannyFiltered);
  cvReleaseImage(&pixCannyFilteredAndDilated);
}



}
