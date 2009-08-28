/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv_latest/CvBridge.h>
#include <std_msgs/String.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

#include <dorylus/dorylus.h>
#include <descriptors_2d/descriptors_2d.h>


USING_PART_OF_NAMESPACE_EIGEN;
using namespace std;
using namespace cv;



// TODO: Make this not copy data.  
MatrixXf* cvVector2Eigen(const Vector<float>& v) {
  MatrixXf* m = new MatrixXf(v.size(), 1);
  for(size_t i=0; i<v.size(); i++) {
    (*m)(i,0) = v[i];
  }
  return m;
}

vector<ImageDescriptor*> setupImageDescriptors() {
  vector<ImageDescriptor*> d;

// HogWrapper::HogWrapper(Size winSize, Size blockSize, Size blockStride, Size cellSize,
// 			int num_bins, int derivAperture, double winSigma,
// 			int histogramNormType, double L2HysThreshold, bool gammaCorrection) : 


  // -- Hog basic
//   d.push_back(new HogWrapper(Size(16,16), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(32,32), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(16,16), 7, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(128,128), Size(64,64), Size(32,32), Size(32,32), 7, 1, -1, 0, 0.2, true));
 
  // -- Hog for hands.
  d.push_back(new HogWrapper(Size(100,100), Size(50,50), Size(25,25), Size(25,25), 9, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(100,100), Size(50,50), Size(50,50), Size(25,25), 9, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(16,16), 9, 1, -1, 0, 0.2, true));


  // -- Hog extended
//   d.push_back(new HogWrapper(Size(16,16), Size(16,16), Size(8,8), Size(8,8), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(32,32), Size(16,16), Size(8,8), Size(8,8), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(16,16), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(128,128), Size(64,64), Size(32,32), Size(32,32), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(128,128), Size(64,64), Size(32,32), Size(16,16), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(8,8), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(32,32), Size(16,16), Size(8,8), Size(4,4), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(16,16), Size(16,16), Size(8,8), Size(4,4), 9, 1, -1, 0, 0.2, true));
 
  SuperpixelColorHistogram* sch1 = new SuperpixelColorHistogram(20, 0.5, 10);
  SuperpixelColorHistogram* sch2 = new SuperpixelColorHistogram(5, 0.5, 10, NULL, sch1);
  SuperpixelColorHistogram* sch3 = new SuperpixelColorHistogram(5, 1, 10, NULL, sch1);
  SuperpixelColorHistogram* sch4 = new SuperpixelColorHistogram(5, .25, 10, NULL, sch1);
  d.push_back(sch1);
  d.push_back(sch2);
  d.push_back(sch3);
  d.push_back(sch4);
 
//   d.push_back(new SurfWrapper(true, 150));
//   d.push_back(new SurfWrapper(true, 100));
//   d.push_back(new SurfWrapper(true, 50));
//   d.push_back(new SurfWrapper(true, 25));
//   d.push_back(new SurfWrapper(true, 10));
   
//   Daisy* base_daisy = new Daisy(25, 3, 8, 8, NULL);
//   d.push_back(base_daisy);
//   d.push_back(new Daisy(50, 3, 8, 8, base_daisy));
//   d.push_back(new Daisy(75, 3, 8, 8, base_daisy));
//   d.push_back(new Daisy(100, 3, 8, 8, base_daisy));
//   d.push_back(new Daisy(150, 3, 8, 8, base_daisy));

  return d;
}


class ThumbnailHandDetector
{
private:
  ros::NodeHandle node_handle_;
  ros::V_Subscriber subs_;
  ros::Publisher pub_;

  sensor_msgs::ImageConstPtr last_msg_;
  sensor_msgs::CvBridge img_bridge_;
  boost::mutex image_mutex_;
  
  std::string window_name_;
  boost::format filename_format_;
  int save_count_;
  int detection_count_;
  
  bool visualize_;
  bool save_;
  Dorylus classifier_;
  vector<ImageDescriptor*> descriptors_;

public:
  ThumbnailHandDetector(const ros::NodeHandle& node_handle, string classifier_filename)
    : node_handle_(node_handle), filename_format_(""), save_count_(0), detection_count_(0)
  {
    node_handle_.param("~window_name", window_name_, node_handle_.resolveName("image"));
    
    //node_handle_.param<bool>("~save", save_, false); // Why doesn't this work?
    node_handle_.param<bool>("/headcart/thumbnail_hand_detector/save", save_, false);
    node_handle_.param<bool>("/headcart/thumbnail_hand_detector/visualize", visualize_, false);

    cout << "Save flag: " << save_ << ", visualize flag: " << visualize_ << endl;

    bool autosize;
    node_handle_.param("~autosize", autosize, false);
    
    std::string format_string;
    node_handle_.param("~filename_format", format_string, std::string("frame%04i.jpg"));
    filename_format_.parse(format_string);

    if(visualize_) {    
      ROS_INFO_STREAM("Starting visualization.");
//       cvNamedWindow(window_name_.c_str(), autosize ? CV_WINDOW_AUTOSIZE : 0);
//       cvNamedWindow("small", CV_WINDOW_AUTOSIZE);
      cvNamedWindow("Classification", CV_WINDOW_AUTOSIZE);
      cvStartWindowThread();
    }

    subs_.push_back( node_handle_.subscribe("image", 1, &ThumbnailHandDetector::image_cb, this) );
    pub_ = node_handle_.advertise<std_msgs::String>("hands", 100);

    classifier_.load(classifier_filename);
    descriptors_ = setupImageDescriptors();
    
    vector<string> files;
    mkdir("hands", S_IRWXG | S_IRWXU | S_IRWXO);
    getDir("hands", files);
    int id, max_id = 0;
    for(size_t i=0; i<files.size(); ++i) {
      size_t end = files[i].find(".jpg");
      if(end == string::npos) 
	continue;

      
      id = atoi(files[i].substr(4, end).c_str());
      if(id > max_id)
	max_id = id;
    }
    save_count_ = max_id + 1;
  }

  ~ThumbnailHandDetector()
  {
    cvDestroyWindow(window_name_.c_str());
  }

  void image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    boost::lock_guard<boost::mutex> guard(image_mutex_);
    
    // Hang on to message pointer for sake of mouse_cb
    //last_msg_ = msg;

    /** @todo: restore raw bayer display, ideally without copying the whole msg... */
#if 0
    // May want to view raw bayer data
    if (msg->encoding.find("bayer") != std::string::npos)
      msg->encoding = "mono";
#endif

    if(img_bridge_.fromImage(*msg, "bgr8")) {
      double t_vis = (double)cvGetTickCount();
      IplImage* img = img_bridge_.toIpl();

      // -- Crop and resize to 128x128.
      int h = img->height;
      int w = img->width;
      assert(w>h);
      CvRect rect = cvRect((w-h)/2, 0, h, h); // Crop to square. 
      cvSetImageROI(img, rect);
      IplImage* small = cvCreateImage(cvSize(128, 128), img->depth, img->nChannels);
      cvResize(img, small, CV_INTER_NN);
      cvResetImageROI(img);
      //cvShowImage("small", small);

      IplImage* vis = NULL;
      if(visualize_) 
	vis = cvCloneImage(small);


      // -- Collect features. 
      Vector<KeyPoint> keypoints;
      vector<object*> objects;
      collectRandomFeatures(small, 40, &keypoints, &objects);

      // -- Make predictions.
      double total_response = 0;
      for(size_t j=0; j<objects.size(); ++j) {
	MatrixXf response = classifier_.classify(*objects[j]);
	total_response += response(0,0);
	int size = ceil(log(ceil(abs(response(0,0))+.001)));
	int x = keypoints[j].pt.x;
	int y = keypoints[j].pt.y;
	
	if(visualize_) {
	  if(response(0,0) > 0)
	    cvCircle(vis, cvPoint(x, y), size, cvScalar(0,255,0), -1);
	  else
	    cvCircle(vis, cvPoint(x, y), size, cvScalar(0,0,255), -1);
	}
      }

      if(total_response > 0) {
	detection_count_++;

	std::stringstream ss;
	ss << "Found hand.  Detection count: " << detection_count_ << endl;
	std_msgs::String msg;
	msg.data = ss.str();
	pub_.publish(msg);

	ROS_INFO_STREAM("Found hand.  Response: " << total_response);
	if(save_) {
	  char buf[100];
	  sprintf(buf, "hands/hand%05d.jpg", save_count_);
	  cvSaveImage(buf, small);
	  cout << "   Saved " << buf;
	  save_count_++;
	}
	cout << endl;

	// -- Make the image green.
	for(int r=0; r<img->height; r++) {
	  uchar* ptr = (uchar*)(img->imageData + r * img->widthStep);
	  for(int c=0; c<img->width; c++) {
	    ptr[3*c+1] = 200;
	  }
	}
      }
      
      if(visualize_) {
	//cvShowImage(window_name_.c_str(), img);
	cvShowImage("Classification", img);
      }

      // -- Cleanup.
      cvReleaseImage(&small);
      if(vis)
	cvReleaseImage(&vis);
      for(size_t i=0; i<objects.size(); ++i)
	delete objects[i];
      

      t_vis = (double)cvGetTickCount() - t_vis;
      //printf("Took %g ms for this image.\n", t_vis/(cvGetTickFrequency()*1000.) );
    }
    else
      ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
  }


  void collectRandomFeatures(IplImage* img, int num_samples, Vector<KeyPoint>* keypoints, vector<object*>* objects) {
    getRandomKeypoints(img, num_samples, keypoints);
    
    // -- Collect features.
    Vector<vvf> results(descriptors_.size());
    for(size_t j=0; j<descriptors_.size(); ++j) {
      descriptors_[j]->compute(img, *keypoints, results[j]);
    }

    // -- Put into objects
    assert((int)keypoints->size() == num_samples);
    Vector<KeyPoint> actual_keypoints;
    actual_keypoints.reserve(keypoints->size());
    for(size_t k=0; k<(size_t)num_samples; k++)  {
      object* obj = new object;
      
      // -- Only accept those that have all valid descriptors. 
      bool success = true;
      for(size_t j=0; j<descriptors_.size(); j++) {
	if(results[j][k].empty()) {
	  success = false;
	  break;
	}
	obj->features[descriptors_[j]->getName()] = cvVector2Eigen(results[j][k]);
      }
      if(success) {
	objects->push_back(obj);
	actual_keypoints.push_back((*keypoints)[k]);
      }
      else
	delete obj;      
    }

    *keypoints = actual_keypoints; 
  }


  // Hacked for speed: assuming we only want to sample the middle 28x28 pixels.
  void getRandomKeypoints(IplImage* img, int num_samples, Vector<KeyPoint>* keypoints) { 
    assert(keypoints->empty());
    
    keypoints->reserve(num_samples);
    for(int i=0; i<num_samples; i++)  {
//       int x = rand() % img->width;
//       int y = rand() % img->height;
      int x = 50 + rand() % 28;
      int y = 50 + rand() % 28;
      
      int size = 1;
    
      keypoints->push_back(KeyPoint(x, y, size));
    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thumbnail_hand_detector");
  ros::NodeHandle n;
  if (n.resolveName("image") == "/image") {
    ROS_WARN("image_view: image has not been remapped! Example command-line usage:\n"
             "\t$ rosrun image_view image_view image:=/forearm/image_color");
  }

  if(argc != 2) {
    cout << "usage: " << argv[0] << " CLASSIFIER_FILENAME image:=<image topic>" << endl;
    return 1;
  }


  ThumbnailHandDetector view(n, argv[1]);
  ros::spin();
  return 0;
}
