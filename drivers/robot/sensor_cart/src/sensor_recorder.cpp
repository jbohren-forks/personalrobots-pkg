#include <cstdio>
#include <vector>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "std_msgs/LaserImage.h"
#include "std_msgs/PointCloud.h"
#include "image_utils/cv_bridge.h"

#include <sys/stat.h>
#include <fstream>

using namespace std;
using namespace std_msgs;

class SensorRecorder : public ros::node
{
public:
  LaserImage laser_img;
  Image cam_img;
  PointCloud cloud;

  LaserImage laser_img_sv;
  Image cam_img_sv;
  PointCloud cloud_sv;

  CvBridge<Image> cam_bridge;
  CvBridge<Image> laser_int_bridge;
  CvBridge<Image> laser_rng_bridge;

  CvBridge<Image> cam_bridge_sv;
  CvBridge<Image> laser_int_bridge_sv;
  CvBridge<Image> laser_rng_bridge_sv;

  ros::thread::mutex cv_mutex;

  IplImage *cv_image;
  IplImage *cv_image_sc;

  char dir_name[256];
  int img_cnt;
  bool made_dir;

  SensorRecorder() : node("sensor_recorder"),
		     cam_bridge(&cam_img), laser_int_bridge(&laser_img.intensity_img), laser_rng_bridge(&laser_img.range_img),
		     cam_bridge_sv(&cam_img_sv), laser_int_bridge_sv(&laser_img_sv.intensity_img), laser_rng_bridge_sv(&laser_img_sv.range_img),
		     cv_image(0), cv_image_sc(0), img_cnt(0), made_dir(false)
  { 
    cvNamedWindow("cam_view", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("intensity_view", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("range_view", CV_WINDOW_AUTOSIZE);

    subscribe("laser_image", laser_img, &SensorRecorder::laser_cb, 1);
    subscribe("cam_image", cam_img, &SensorRecorder::cam_cb, 1);
    subscribe("full_cloud", cloud, &SensorRecorder::cloud_cb, 1);

    time_t rawtime;
    struct tm* timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    sprintf(dir_name, "recordings_%.2d%.2d%.2d_%.2d%.2d%.2d", timeinfo->tm_mon + 1, timeinfo->tm_mday,timeinfo->tm_year - 100,timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
  }

 ~SensorRecorder()
  {
    free_images();
  }

  void free_images()
  {
    if (cv_image)
      cvReleaseImage(&cv_image);
    if (cv_image_sc)
      cvReleaseImage(&cv_image_sc);
  }


  void do_scale() {
      cv_image_sc = cvCreateImage(cvSize(cv_image->width, cv_image->height),
				  IPL_DEPTH_8U, 1);
      if (cv_image->depth == IPL_DEPTH_16U)
	cvCvtScale(cv_image, cv_image_sc, 0.0625, 0);
      else
	cvCvtScale(cv_image, cv_image_sc, 1, 0);
  }

  void cam_cb()
  {
    cv_mutex.lock();

    cam_img_sv = cam_img;

    free_images();
    if (cam_bridge.to_cv(&cv_image))
    {
      do_scale();
      cvShowImage("cam_view", cv_image_sc);
    }
    cv_mutex.unlock();
  }

  void laser_cb()
  {
    cv_mutex.lock();

    laser_img_sv = laser_img;

    free_images();
    if (laser_int_bridge.to_cv(&cv_image))
    {
      do_scale();
      cvShowImage("intensity_view", cv_image_sc);
    }

    free_images();
    if (laser_rng_bridge.to_cv(&cv_image))
    {
      do_scale();
      cvShowImage("range_view", cv_image_sc);
    }
    cv_mutex.unlock();
  }

  void cloud_cb()
  {
    cv_mutex.lock();

    cloud_sv = cloud;

    cv_mutex.unlock();
  }

  void check_keys() {
    cv_mutex.lock();
    if (cvWaitKey(3) == 10)
      save_stuff();
    cv_mutex.unlock();
  }

  void save_stuff() {

    if (!made_dir) {
      if (mkdir(dir_name, 0755)) {
	printf("Failed to make directory: %s\n", dir_name);
	return;
      } else {
	made_dir = true;
      }
    }

    img_cnt++;

    save_laser();
    save_cam();
    save_cloud();    
    save_cloud_vrml();
  }

  void save_laser() {
    printf("Saving laser data.\n");

    laser_img_sv = laser_img;

    free_images();
    if (laser_rng_bridge_sv.to_cv(&cv_image))
    {
      do_scale();
      std::ostringstream oss;
      oss << dir_name << "/Range" << img_cnt << ".png";
      cvSaveImage(oss.str().c_str(), cv_image_sc);
    }    

    free_images();
    if (laser_int_bridge_sv.to_cv(&cv_image))
    {
      do_scale();
      std::ostringstream oss;
      oss << dir_name << "/Intensity" << img_cnt << ".png";
      cvSaveImage(oss.str().c_str(), cv_image_sc);
    }    

    {      
      ostringstream oss;
      oss << dir_name << "/VertAngles" << img_cnt << ".dat";
      ofstream out(oss.str().c_str());
     
      out.setf(ios::fixed, ios::floatfield);
      out.setf(ios::showpoint);
      out.precision(5);
 
      out << "#Vertical Angles"
	  << "#" << laser_img_sv.get_vert_angles_size() << endl;
      
      for (int i = 0; i < laser_img_sv.get_vert_angles_size(); i++)
	out << laser_img_sv.vert_angles[i] << endl;

      out.close();
    }

    {      
      ostringstream oss;
      oss << dir_name << "/HorizAngles" << img_cnt << ".dat";
      ofstream out(oss.str().c_str());
     
      out.setf(ios::fixed, ios::floatfield);
      out.setf(ios::showpoint);
      out.precision(5);
 
      out << "#Horizontal Angles"
	  << "#" << laser_img_sv.get_horiz_angles_size() << endl;
      
      for (int i = 0; i < laser_img_sv.get_horiz_angles_size(); i++)
	out << laser_img_sv.horiz_angles[i] << endl;

      out.close();
    }

    
  }
  void save_cam() {
    printf("Saving camera data.\n");

    free_images();
    if (cam_bridge_sv.to_cv(&cv_image))
    {
      do_scale();
      std::ostringstream oss;
      oss << dir_name << "/Cam" << img_cnt << ".png";
      cvSaveImage(oss.str().c_str(), cv_image_sc);
    }    
  }

  void save_cloud() {
    printf("Saving cloud data.\n");

    ostringstream oss;
    oss << dir_name << "/Cloud" << img_cnt << ".dat";
    ofstream out(oss.str().c_str());
    
    out.setf(ios::fixed, ios::floatfield);
    out.setf(ios::showpoint);
    out.precision(5);

    out << "#Point Cloud" << endl;
    out << "#Format: X [meters], Y [meters], Z [meters], Intensity" << endl;
    out << "#Points: " << cloud_sv.get_pts_size() << endl;

    for (int i = 0; i < cloud_sv.get_pts_size(); i++) {
      out << cloud_sv.pts[i].x << " " 
	  << cloud_sv.pts[i].y << " "
	  << cloud_sv.pts[i].z << " " 
	  << cloud_sv.chan[0].vals[i] << endl;
    }
    
    out.close();

  }


  void save_cloud_vrml() {
    printf("Saving vrml file.\n");

    ostringstream oss;
    oss << dir_name << "/Cloud" << img_cnt << ".wrl";
    ofstream out(oss.str().c_str());
    
    out.setf(ios::fixed, ios::floatfield);
    out.setf(ios::showpoint);
    out.precision(5);

    out << "#VRML V2.0 utf8" << endl
	<< "DEF InitView Viewpoint" << endl
	<< "{" << endl
	<< "position -2 0 0" << endl
	<< "orientation 0 1 0 -1.57079633" << endl
	<< "" << endl
	<< "}DEF World Transform " << endl
	<< "{" << endl
	<< "translation 0 0 0" << endl
	<< "children  [" << endl
	<< "DEF Scene Transform" << endl
	<< "{" << endl
	<< "translation 0 0 0" << endl
	<< "children [" << endl
	<< "Shape" << endl
	<< "{" << endl
	<< "geometry PointSet" << endl
	<< "{" << endl
	<< "coord Coordinate" << endl
	<< "{" << endl
	<< "point[" << endl;

    for (int i = 0; i < cloud_sv.get_pts_size(); i++) {
      out << cloud_sv.pts[i].x << " " 
	  << cloud_sv.pts[i].z << " "
	  << -cloud_sv.pts[i].y << endl;
    }

    out << "]"
	<< "}" << endl
	<< "color Color" << endl
	<< "{" << endl
	<< "color[" << endl;

    for (int i = 0; i < cloud_sv.get_pts_size(); i++) {
      out << cloud_sv.chan[0].vals[i] / (4080.0) << " "
	  << cloud_sv.chan[0].vals[i] / (4080.0) << " "
	  << cloud_sv.chan[0].vals[i] / (4080.0) << endl;
    }

    out << "]"
	<< "}" << endl
	<< "}" << endl
	<< "}" << endl
	<< "" << endl
	<< "]" << endl
	<< "} # end of scene transform" << endl
	<< "]" << endl
	<< "} # end of world transform" << endl;

    out.close();
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  SensorRecorder rec;
  while (rec.ok()) {
    usleep(10000);
    rec.check_keys();
  }
  ros::fini();
  return 0;
}

