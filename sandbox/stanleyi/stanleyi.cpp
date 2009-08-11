/*********************************************************************
*
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
*
* Author: Alex Teichman
*********************************************************************/

#include <dorylus.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <rosrecord/Player.h>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <opencv_latest/CvBridge.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <fstream>
#include <tinyxml/tinyxml.h>


#include <descriptors_2d/descriptors_2d.h>
#include <descriptors_2d_gpl/descriptors_2d_gpl.h>

USING_PART_OF_NAMESPACE_EIGEN
using namespace std;
using namespace cv;

// -- Utility functions.
vector<ImageDescriptor*> setupImageDescriptors();
void releaseImageDescriptors(vector<ImageDescriptor*>* desc);
template <class T> void copyMsg(string name, T* m, ros::Time t, ros::Time t_no_use, void* n);
int getdir (string dir, vector<string> &files);
IplImage* findLabelMask(double stamp, string results_dir);
void findLabelPolys(double stamp, string results_dir, Vector< Vector<Point> >& polys, vector<int>& poly_labels);
map<string, int> createLabelMap();
void showLabelPolys(IplImage* img, const Vector< Vector<Point> >& polys);
void createLabelMaps(map<string, int>* str2int, vector<string>* int2str);
  
/***************************************************************************
***********  Stanleyi
****************************************************************************/


class Stanleyi
{
public:
  Stanleyi();
  ~Stanleyi();
  void testContours(string bagfile, string label_dir);
  void sanityCheck(string bagfile, string results_dir);
  DorylusDataset* collectDataset(string bagfile, int samples_per_img, string results_dir);
  DorylusDataset* collectDatasetXML(string bagfile, int samples_per_img, string results_dir);
  void viewLabels(string bagfile, string results_dir);
  void makeClassificationVideo(string bagfile, Dorylus& d, int samples_per_img, string results_dir);

private:
  ros::record::Player lp_;
  sensor_msgs::Image img_msg_;
  sensor_msgs::CvBridge img_bridge_;
  IplImage* img_;
  IplImage* mask_;
  vector<ImageDescriptor*> descriptor_;

  void drawResponse(IplImage* img, float response, CvPoint pt);
  void collectObjectsFromImageVectorized(int samples_per_img, vector<object*>* objects, Vector<KeyPoint>* keypoints);
  void collectObjectsFromImageVectorizedXML(int samples_per_img, const Vector< Vector<Point> >& polys, const vector<int>& poly_labels, vector<object*>* objects, Vector<KeyPoint>* keypoints);
};

Stanleyi::Stanleyi()
  : img_(NULL), mask_(NULL)
{
  lp_.addHandler<sensor_msgs::Image>(string("/narrow_stereo/left/image_rect_color"), &copyMsg<sensor_msgs::Image>, (void*)(&img_msg_));
  descriptor_ = setupImageDescriptors();
  if(getenv("DEBUG") != NULL) {
    for(size_t i=0; i<descriptor_.size(); i++) { 
      descriptor_[i]->debug_ = true;
    }
  }
  //srand(time(NULL)); //For randomly selecting points.
}

Stanleyi::~Stanleyi() {
  releaseImageDescriptors(&descriptor_);
  if(mask_)
    cvReleaseImage(&mask_);
  //img_ is released by the CvBridge.
}

DorylusDataset* Stanleyi::collectDataset(string bagfile, int samples_per_img, string results_dir) {
  bool debug = false;
  vector<object*> objs;

  if(getenv("DEBUG") != NULL) {
    cout << "Entering debug mode." << endl;
    debug = true;
  }

  lp_.open(bagfile, ros::Time());

  // -- Get data for all labeled images.
  int frame_id = 0;
  while(lp_.nextMsg()) {
    frame_id++;

    // -- Get the next img with a label mask.
    if (!img_bridge_.fromImage(img_msg_, "bgr"))  {
       ROS_ERROR("Could not convert message to ipl.");
      continue;
    }

    img_ = img_bridge_.toIpl();
    mask_ = findLabelMask(img_msg_.header.stamp.toSec(), results_dir);
    if(mask_ == NULL)
      continue;
    
    ROS_ASSERT(img_ != NULL);
    
    Vector<KeyPoint> keypoints;
    collectObjectsFromImageVectorized(samples_per_img, &objs, &keypoints);

    cvReleaseImage(&mask_);
  }

  DorylusDataset* dd = new DorylusDataset();
  dd->setObjs(objs);
  return dd;
}

DorylusDataset* Stanleyi::collectDatasetXML(string bagfile, int samples_per_img, string results_dir) {
  bool debug = false;
  vector<object*> objs;

  if(getenv("DEBUG") != NULL) {
    cout << "Entering debug mode." << endl;
    debug = true;
  }

  lp_.open(bagfile, ros::Time());

  // -- Get data for all labeled images.
  int frame_id = 0;
  while(lp_.nextMsg()) {
    frame_id++;
    cout << frame_id << endl;

    // -- Get the next img with a label mask.
    if (!img_bridge_.fromImage(img_msg_, "bgr"))  {
      ROS_ERROR("Could not convert message to ipl.");
      continue;
    }

    img_ = img_bridge_.toIpl();
    ROS_ASSERT(img_ != NULL);

    Vector< Vector<Point> > polys;
    vector<int> poly_labels;
    findLabelPolys(img_msg_.header.stamp.toSec(), results_dir, polys, poly_labels);    
    showLabelPolys(img_, polys);
    
    Vector<KeyPoint> keypoints;
    collectObjectsFromImageVectorizedXML(samples_per_img, polys, poly_labels, &objs, &keypoints);
  }

  DorylusDataset* dd = new DorylusDataset();
  dd->setObjs(objs);
  return dd;
}

// TODO: Make this not copy data.  
MatrixXf* cvVector2Eigen(const Vector<float>& v) {
  MatrixXf* m = new MatrixXf(v.size(), 1);
  for(size_t i=0; i<v.size(); i++) {
    (*m)(i,0) = v[i];
  }
  return m;
}


  //! Appends to objects.  If an object is invalid, a NULL pointer is used so that keypoints[i] corresponds to objects[i].
void Stanleyi::collectObjectsFromImageVectorized(int samples_per_img, vector<object*>* objects, Vector<KeyPoint>* keypoints) {
  vector<vvf> results(descriptor_.size());
  Vector<KeyPoint> desired;

  // -- Choose random locations and make keypoints.
  keypoints->clear();
  keypoints->reserve(samples_per_img);
  desired.reserve(samples_per_img);
  vector<int> labels(samples_per_img);
  for(int i=0; i<samples_per_img; i++)  {
    int r = rand() % img_->height;
    int c = rand() % img_->width;
    int size = 1;
    desired.push_back(KeyPoint(c, r, size));

    if(mask_) {
      CvScalar s = cvGet2D(mask_, r, c);
      labels[i] = s.val[0];
      if(labels[i] != 0) {
	labels[i] = 1;
      }
    }
    else
      labels[i] = -1;
      
  }
  
  // -- Call all descriptors, get vectorized results.
  for(size_t i=0; i<descriptor_.size(); i++) {
    descriptor_[i]->compute(img_, desired, results[i]);
  }

  // -- Copy into objects.
  for(int i=0; i<samples_per_img; i++)  {
    object* obj = new object;
    obj->label = labels[i];

    // -- Only accept those that have all valid descriptors. 
    bool success = true;
    for(size_t j=0; j<descriptor_.size(); j++) {

      if(results[j][i].empty()) {
	success = false;
	break;
      }
      obj->features[descriptor_[j]->getName()] = cvVector2Eigen(results[j][i]);
    }
    if(success) {
      objects->push_back(obj);
      keypoints->push_back(desired[i]);
    }
    else {
      delete obj;
    }
  }
}


//! Appends to objects.  If an object is invalid, a NULL pointer is used so that keypoints[i] corresponds to objects[i].
void Stanleyi::collectObjectsFromImageVectorizedXML(int samples_per_img, const Vector< Vector<Point> >& polys, const vector<int>& poly_labels, vector<object*>* objects, Vector<KeyPoint>* keypoints) {
  assert(polys.size() == poly_labels.size());

  vector<vvf> results(descriptor_.size());
  Vector<KeyPoint> desired;

  // -- Choose random locations and make keypoints.
  keypoints->clear();
  keypoints->reserve(samples_per_img);
  desired.reserve(samples_per_img);
  vector<int> labels(samples_per_img);
  for(int i=0; i<samples_per_img; i++)  {
    int r = rand() % img_->height;
    int c = rand() % img_->width;
    int size = 1;
    desired.push_back(KeyPoint(c, r, size));

    // -- Find the label.  (0 for bg).
    for(size_t j=0; j<polys.size(); ++j) {
      if(pointPolygonTest(polys[j], Point2f(c, r), 0) >= 0) {
	labels[i] = poly_labels[j];
      }
    }

    // -- Show the labels.
    vector<string> label_int2str;
    createLabelMaps(NULL, &label_int2str);
    cout << "This point is labeled " << labels[i] << ", " << label_int2str[labels[i]] << endl;
    IplImage* vis = cvCloneImage(img_);
    cvLine(vis, cvPoint(c-10, r), cvPoint(c+10, r), cvScalar(0,0,255));
    cvLine(vis, cvPoint(c, r-10), cvPoint(c, r+10), cvScalar(0,0,255));
    CVSHOW("Label", vis);
    cvWaitKey(0);
  }
  
  // -- Call all descriptors, get vectorized results.
  for(size_t i=0; i<descriptor_.size(); i++) {
    descriptor_[i]->compute(img_, desired, results[i]);
  }

  // -- Copy into objects.
  for(int i=0; i<samples_per_img; i++)  {
    object* obj = new object;
    obj->label = labels[i];

    // -- Only accept those that have all valid descriptors. 
    bool success = true;
    for(size_t j=0; j<descriptor_.size(); j++) {

      if(results[j][i].empty()) {
	success = false;
	break;
      }
      obj->features[descriptor_[j]->getName()] = cvVector2Eigen(results[j][i]);
    }
    if(success) {
      objects->push_back(obj);
      keypoints->push_back(desired[i]);
    }
    else {
      delete obj;
    }
  }
}

//! Train and make predictions on a single labeled image.  
//! This is mostly to make sure nothing totally wrong is happening, as almost any feature should make this task work perfectly. 
void Stanleyi::sanityCheck(string bagfile, string results_dir) {

  int samples_per_img = 1000;
  if(getenv("NSAMPLES") != NULL) 
    samples_per_img = atoi(getenv("NSAMPLES"));

  int skip_frames = 184;
  if(getenv("SKIP_FRAMES") != NULL) 
    skip_frames = atoi(getenv("SKIP_FRAMES"));

  // -- Skip frames.
  lp_.open(bagfile, ros::Time());
  for(int i=0; i<skip_frames; i++) {
    lp_.nextMsg();
  }

  while(lp_.nextMsg()) {
    // -- Get next image from the bag.
    if (!img_bridge_.fromImage(img_msg_, "bgr")) {
      ROS_ERROR("Could not convert message to ipl.");
      break;
    }  
    img_ = img_bridge_.toIpl();

    // -- Stop at the first labeled image.
    mask_ = findLabelMask(img_msg_.header.stamp.toSec(), results_dir);
    if(mask_)
      break;
  }

  // -- Collect dataset on that image.
  Vector<KeyPoint> keypoints;
  vector<object*> objs;
  collectObjectsFromImageVectorized(samples_per_img, &objs, &keypoints);
  DorylusDataset dd;
  dd.setObjs(objs);
  cout << dd.status() << endl;

  // -- Train classifier.
  Dorylus d;
  d.useDataset(&dd);
  int nCandidates = 10;
  int max_secs = 60*5;
  int max_wcs = 1000;
  d.train(nCandidates, max_secs, max_wcs);

  // -- Make predictions on that image.
  while(true) {
    IplImage* vis_img = cvCloneImage(img_);
    IplImage* vis_mask = cvCloneImage(mask_);

    // -- Draw predictions.
    for(size_t i=0; i<objs.size(); ++i)
      delete objs[i];
    objs.clear();
    keypoints.clear();
    collectObjectsFromImageVectorized(samples_per_img, &objs, &keypoints);
    for(size_t i=0; i<objs.size(); i++) {
      if(objs[i] == NULL) 
	continue;

      MatrixXf response = d.classify(*objs[i]);
      int col = keypoints[i].pt.x;
      int row = keypoints[i].pt.y;

      drawResponse(vis_img, response(0,0), cvPoint(col, row));
      drawResponse(vis_mask, response(0,0), cvPoint(col, row));
    }

    // -- Display
    CVSHOW("Image", vis_img);
    CVSHOW("Mask", vis_mask);
    if(cvWaitKey(33) == 'q') 
      break;

    // -- Clean up.
    cvReleaseImage(&vis_img);
    cvReleaseImage(&vis_mask);
  }
}

void Stanleyi::drawResponse(IplImage* img, float response, CvPoint pt) {
  int size = 1;
  if(response != 0)
    size = ceil(log(ceil(abs(response))));

  if(response > 0)
    cvCircle(img, pt, size, cvScalar(0,255,0), -1);
  else 
    cvCircle(img, pt, size, cvScalar(0,0,255), -1);
}

void Stanleyi::makeClassificationVideo(string bagfile, Dorylus& d, int samples_per_img, string results_dir) {
  //vector<ImageDescriptor*> desc = setupImageDescriptors();
  //d.exclude_descriptors_.push_back(desc[4]->name_);
  if(getenv("MAX_WC") != NULL)
    d.max_wc_ = atoi(getenv("MAX_WC"));
  int max_frames = 0;
  if(getenv("MAX_FRAMES") != NULL)
    max_frames = atoi(getenv("MAX_FRAMES"));
  

  cvNamedWindow("Classification Visualization", CV_WINDOW_AUTOSIZE);
  

  int row = 0, col = 0;
  IplImage* vis = NULL;
  MatrixXf response;
  int frame_num = 0;
  CvVideoWriter* writer = NULL;
  vector<object*> objects;

  int skip_frames = 184;
  if(getenv("SKIP_FRAMES") != NULL) 
    skip_frames = atoi(getenv("SKIP_FRAMES"));


  lp_.open(bagfile, ros::Time());
  for(int i=0; i<skip_frames; i++) {
    lp_.nextMsg();
    frame_num++;
  }

  int nFrames = 0;
  while(lp_.nextMsg()) {
    frame_num++;
    nFrames++;

    // -- Get next image from the bag.
    if (!img_bridge_.fromImage(img_msg_, "bgr")) {
      ROS_ERROR("Could not convert message to ipl.");
      break;
    }  
    img_ = img_bridge_.toIpl();
    vis = cvCloneImage(img_);

    if(writer == NULL) {
      writer = cvCreateVideoWriter("output.mpg", CV_FOURCC('P','I','M','1'), 30, cvGetSize(img_));
    }

    IplImage* mask = findLabelMask(img_msg_.header.stamp.toSec(), results_dir);
    if(getenv("DEBUG_POSITIVES") != NULL &&  mask == NULL)
      continue;


    Vector<KeyPoint> keypoints;

    // -- Collect features.
    objects.clear();
    collectObjectsFromImageVectorized(samples_per_img, &objects, &keypoints);

    int tp=0, fp=0, tn=0, fn=0;
    for(size_t i=0; i<objects.size(); i++) {
      if(objects[i] == NULL) 
	continue;
      //cout << objects[i]->status(false) << endl;
      response = d.classify(*objects[i]);
      col = keypoints[i].pt.x;
      row = keypoints[i].pt.y;

      if(mask) {
	CvScalar s = cvGet2D(mask, row, col);
	//cout << s.val[0] << " " << s.val[1] << " " << s.val[2] << " " << s.val[3] << endl;
	int label = s.val[0];
	if(label != 0 && response(0,0) > 0)
	  tp++;
	else if(label == 0 && response(0,0) > 0)
	  fp++;
	else if(label == 0 && response(0,0) <= 0)
	  tn++;
	else// if(label == 0 && response(0,0) > 0)
	  fn++;
      }

      //cout << response_nm.Nrows() << " " << response_nm.Ncols() << endl;

      int size = ceil(log(ceil(abs(response(0,0)))));
      if(response(0,0) > 0) {
	cvCircle(vis, cvPoint(col, row), size, cvScalar(0,255,0), -1);
	if(mask)
	  cvCircle(mask, cvPoint(col, row), size, cvScalar(0,255,0), -1);
      }
      else {
	cvCircle(vis, cvPoint(col, row), size, cvScalar(0,0,255), -1);
	if(mask)
	  cvCircle(mask, cvPoint(col, row), size, cvScalar(0,0,255), -1);
      }
    }

    if(mask) {
      cvNamedWindow("Mask", CV_WINDOW_AUTOSIZE);
      cvShowImage("Mask", mask);
      printf("TP: %d, TN: %d, FP: %d, FN: %d\n", tp, tn, fp, fn);
    }

    cvWriteFrame(writer, vis);

    cout << "Showing results for frame " << frame_num << endl;
    cvShowImage("Classification Visualization", vis);

    // -- Clean up.
    cvReleaseImage(&vis);
    for(size_t i=0; i<objects.size(); ++i)
      delete objects[i];

    if(cvWaitKey(500) == 'q' || nFrames == max_frames) {
      break;
    }
  }
  cvReleaseVideoWriter(&writer);
}


void test_watershed(IplImage *img) {
  int seed_step = 10;

  cout << img->depth << " " << img->nChannels << endl;
    

  // -- Create a grid of seed points.
  IplImage* markers = cvCreateImage( cvGetSize(img), IPL_DEPTH_32S, 1 );
  cvZero(markers);
  int label = 0;
  for(int r=0; r<markers->height; r++) {
    long* ptr = (long*)(markers->imageData + r * markers->widthStep);
    for(int c=0; c<markers->width; c++) {
      if(c%seed_step==0 && r%seed_step==0) {
	*ptr = label;
	label++;
      }
      ptr++;

    }
  }


  double t = (double)cvGetTickCount();
  cvWatershed( img, markers );
  t = (double)cvGetTickCount() - t;
  printf( "exec time = %gms\n", t/(cvGetTickFrequency()*1000.) );
  

  // -- Display the results.
  CvMat* color_tab;
  color_tab = cvCreateMat( 1, label, CV_8UC3 );
  CvRNG rng = cvRNG(-1);
  for(int i = 0; i < label; i++ )
    {
      uchar* ptr = color_tab->data.ptr + i*3;
      ptr[0] = (uchar)(cvRandInt(&rng)%180 + 50);
      ptr[1] = (uchar)(cvRandInt(&rng)%180 + 50);
      ptr[2] = (uchar)(cvRandInt(&rng)%180 + 50);
    }
  
  // paint the watershed image
  IplImage* wshed = cvCloneImage( img );
  IplImage* img_gray = cvCloneImage( img );
  cvZero( wshed );
  IplImage* marker_mask = cvCreateImage( cvGetSize(img), 8, 1 );
  cvCvtColor( img, marker_mask, CV_BGR2GRAY );
  cvCvtColor( marker_mask, img_gray, CV_GRAY2BGR );

  for(int i = 0; i < markers->height; i++ )
    for(int j = 0; j < markers->width; j++ )
      {
	int idx = CV_IMAGE_ELEM( markers, int, i, j );
	uchar* dst = &CV_IMAGE_ELEM( wshed, uchar, i, j*3 );
	if( idx == -1 )
	  dst[0] = dst[1] = dst[2] = (uchar)255;
	else if( idx <= 0 || idx > label )
	  dst[0] = dst[1] = dst[2] = (uchar)0; // should not get here
	else
	  {
	    uchar* ptr = color_tab->data.ptr + (idx-1)*3;
	    dst[0] = ptr[0]; dst[1] = ptr[1]; dst[2] = ptr[2];
	  }
      }

  cvAddWeighted( wshed, 0.5, img_gray, 0.5, 0, wshed );
  cvNamedWindow("Wshed", CV_WINDOW_AUTOSIZE);
  cvShowImage( "Wshed", wshed );



  cvShowImage("Feature", markers);
  cvWaitKey(0);
}  
  

void Stanleyi::viewLabels(string bagfile, string results_dir) {
  cvNamedWindow("Image", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Mask", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Feature", CV_WINDOW_AUTOSIZE);

  if(!lp_.open(bagfile, ros::Time())) {
    ROS_FATAL("Failed to open bag file %s.", bagfile.c_str());
    return;
  }
  else {
    ROS_DEBUG("Opened bag file %s.",  bagfile.c_str());
  }
    
  while(lp_.nextMsg()) {
    cout << "Next msg." << endl;
    if (img_bridge_.fromImage(img_msg_, "bgr")) {
      img_ = img_bridge_.toIpl();
      mask_ = findLabelMask(img_msg_.header.stamp.toSec(), results_dir);
      if(mask_ != NULL) {
	cvShowImage("Image", img_);
	cvShowImage("Mask", mask_);
	test_watershed(img_);
	cvWaitKey(0);
      }
    }
  }
}

void Stanleyi::testContours(string bagfile, string label_dir) {
  vector<IplImage*> masks;
  vector<IplImage*> imgs;

  lp_.open(bagfile, ros::Time());
  while(lp_.nextMsg()) {
    if (!img_bridge_.fromImage(img_msg_, "bgr")) {
      ROS_ERROR("Could not convert message to ipl.");
      break;
    }
    img_ = img_bridge_.toIpl();

    IplImage* mask = findLabelMask(img_msg_.header.stamp.toSec(), label_dir);
    if(!mask)
      continue;

    masks.push_back(mask);
    imgs.push_back(cvCloneImage(img_));
  }
  ContourFragmentManager cfc(10);
  cfc.learnContours(imgs, masks);
  cfc.saveContours("contour_fragments");
}
  
int main(int argc, char** argv) 
{
  // int numCPU = sysconf( _SC_NPROCESSORS_ONLN );
//   cout << numCPU << endl;
//   cvSetNumThreads(numCPU);
  //  cout << cvGetNumThreads() << endl;

  Stanleyi s;

  // -- Get env var options.
  int samples_per_img = 1000;
  if(getenv("NSAMPLES") != NULL) 
    samples_per_img = atoi(getenv("NSAMPLES"));
  int nCandidates = 2;
  if(getenv("NCAND") != NULL) 
    nCandidates = atoi(getenv("NCAND"));
  int max_secs = 0;
  if(getenv("MAX_SECS") != NULL) 
    max_secs = atoi(getenv("MAX_SECS"));
  int max_wcs = 0;
  if(getenv("MAX_WCS") != NULL) 
    max_wcs = atoi(getenv("MAX_WCS"));


  // -- Parse args.
  if(argc > 3 && !strcmp(argv[1], "--viewLabels"))
    {
      cout << "Showing labels for bag " << argv[2] << " using the label masks in " << argv[3] << endl;  
      s.viewLabels(argv[2], argv[3]);
    }

  else if(argc > 4 && !strcmp(argv[1], "--collectDataset"))
    {
      cout << "Collecting a dataset for bag " << argv[2] << ", saving with name " << argv[4] << " using the label masks in " << argv[3] <<  endl;  
      DorylusDataset* dd = s.collectDataset(argv[2], samples_per_img, argv[3]);
      cout << dd->status() << endl;
      dd->save(argv[4]);
      delete dd;
    }

  else if(argc > 4 && !strcmp(argv[1], "--collectDatasetXML"))
    {
      cout << "Collecting a multiclass dataset for bag " << argv[2] << ", saving with name " << argv[4] << " using the label masks in " << argv[3] <<  endl;  
      DorylusDataset* dd = s.collectDatasetXML(argv[2], samples_per_img, argv[3]);
      cout << dd->status() << endl;
      dd->save(argv[4]);
      delete dd;
    }
  
  else if(argc > 2 && !strcmp(argv[1], "--status")) {
    cout << "Examining " << argv[2] << endl;

    // -- Figure out what it is.
    ifstream file(argv[2]);
    if(!file.is_open()) {
      cout << "Could not open " << argv[2] << endl;
      return 1;
    }
    string line;
    getline(file, line);

    if(line.find("#DORYLUS CLASSIFIER LOG") != string::npos) {
      Dorylus d;
      if(!d.load(argv[2]))
	return 1;
      cout << d.status() << endl;
    }

    if(line.find("#DORYLUS DATASET LOG") != string::npos) {
      DorylusDataset dd;
      if(!dd.load(argv[2]))
	return 1;
      cout << dd.status() << endl;
    }
  }

  else if(argc > 2 && !strcmp(argv[1], "--train")) {

    cout << "Training new classifier on " << argv[2] << ", saving with name " << argv[3] << " using " << nCandidates << " candidates, training for " << max_secs << "s or " << max_wcs << " wcs." << endl;
    Dorylus d;
    DorylusDataset dd;
    if(!dd.load(argv[2])) 
      return 1;
    d.useDataset(&dd);
    d.train(nCandidates, max_secs, max_wcs);
    d.save(argv[3]);
  }

  else if(argc > 4 && !strcmp(argv[1], "--collectAndTrain")) {
    cout << "Collecting a dataset for bag " << argv[2]  << " using the label masks in " << argv[3] << " then training with " << 
      samples_per_img << "samples per image and saving in " << argv[4] << endl;  
    DorylusDataset* dd = s.collectDataset(argv[2], samples_per_img, argv[3]);
    cout << dd->status() << endl;
    Dorylus d;
    d.useDataset(dd);
    d.train(nCandidates, max_secs, max_wcs);
    d.save(argv[4]);
  }
  else if(argc > 4 && !strcmp(argv[1], "--makeClassificationVideo")) {
    cout << "Showing video classification for classifier " << argv[2] << " on bag " << argv[3] << " using the label masks in " << argv[4] <<  endl;
    Dorylus d;
    if(!d.load(argv[2]))
      return 1;
    s.makeClassificationVideo(argv[3], d, samples_per_img, argv[4]);
  }
  else if(argc > 3 && !strcmp(argv[1], "--cf")) {
    cout << "Learning contour fragments for bagfile " << argv[2] << " and labels in " << argv[3] << endl;
    s.testContours(argv[2], argv[3]);
  }
  else if(argc > 3 && !strcmp(argv[1], "--sanityCheck")) {
    cout << "Doing sanity check on " << argv[2] << " with labels in " << argv[3] << endl;
    s.sanityCheck(argv[2], argv[3]);
  }


  else {
    cout << "usage: " << endl;
    cout << argv[0] << " --makeClassificationVideo CLASSIFIER BAGFILE [LABELS]" << endl;
    cout << argv[0] << " --collectDataset BAGFILE LABELS DATASET" << endl;
    cout << argv[0] << " --collectDatasetXML BAGFILE LABELS DATASET" << endl;
    cout << argv[0] << " --collectAndTrain BAGFILE LABELS CLASSIFIER" << endl;
    cout << argv[0] << " --viewLabels BAGFILE LABELS" << endl;
    cout << argv[0] << " --status DATASET" << endl;
    cout << argv[0] << " --status CLASSIFIER" << endl;
    cout << argv[0] << " --cf BAGFILE LABELS" << endl;
    cout << argv[0] << " --sanityCheck BAGFILE LABELS" << endl;
    cout << "  where LABELS might take the form of `rospack find cv_mech_turk`/results/single-object-s " << endl;
    cout << "Environment variable options: " << endl;
  }
  
}

vector<ImageDescriptor*> setupImageDescriptors() {
  vector<ImageDescriptor*> d;

//   d.push_back(new HogWrapper(Size(16,16), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(32,32), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(16,16), 7, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(128,128), Size(64,64), Size(32,32), Size(32,32), 7, 1, -1, 0, 0.2, true));

  SuperpixelColorHistogram* sch1 = new SuperpixelColorHistogram(20, 0.5, 10);
//   SuperpixelColorHistogram* sch2 = new SuperpixelColorHistogram(5, 0.5, 10, NULL, sch1);
//   SuperpixelColorHistogram* sch3 = new SuperpixelColorHistogram(5, 1, 10, NULL, sch1);
//   SuperpixelColorHistogram* sch4 = new SuperpixelColorHistogram(5, .25, 10, NULL, sch1);
  d.push_back(sch1);
//   d.push_back(sch2);
//   d.push_back(sch3);
//   d.push_back(sch4);
 
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
//  d.push_back(new Daisy(150, 3, 8, 8, base_daisy));

  // d.push_back(new ContourFragmentDescriptor(0, "contour_fragments"));

  return d;
}

void releaseImageDescriptors(vector<ImageDescriptor*>* desc) {
  for(size_t i=0; i<desc->size(); i++) {
    delete (*desc)[i];
  }
  desc->clear();
}

template <class T>
void copyMsg(string name, T* m, ros::Time t, ros::Time t_no_use, void* n)
{
  if (m != 0) {
    *((T*)(n)) = *((T*)(m));
  }
}

IplImage* findLabelMask(double stamp, string results_dir)
{
  vector<string> files;
  IplImage *mask = NULL;

  string masks_dir = results_dir + "/masks/";
  char buf[100];
  sprintf(buf, "%f", stamp); 
  string sbuf(buf);
  sbuf = sbuf.substr(0, sbuf.length()-1);   //Remove the last digit - it's been rounded.

  getDir(masks_dir, files);
  for(size_t i=0; i<files.size(); i++)
    {
      if(files[i].find(sbuf) != string::npos && 
	 files[i].find(string(".png")) != string::npos)
	{
	  cout << "Found label for " << sbuf << ": " << files[i] << endl;
	  mask = cvLoadImage((masks_dir + files[i]).c_str());
	  break;
	}
    }
  return mask;
}

void getContoursFromTinyXML(string filename, Vector< Vector<Point> >& contours, vector<int>& poly_labels) {

  assert(poly_labels.empty());
  assert(contours.empty());

  map<string, int> label_str2int;
  vector<string> label_int2str;
  createLabelMaps(&label_str2int, &label_int2str);


  // -- Setup XML.
  TiXmlDocument XMLdoc(filename);
  bool loadOkay = XMLdoc.LoadFile();
  if (!loadOkay) {
    cout << "Could not load " << filename << endl;
    return;
  } 

  TiXmlElement *annotations, *annotation, *polygon, *results, *pt;
  annotations = XMLdoc.FirstChildElement("annotations");
  results = annotations->FirstChildElement("results");
  annotation = results->FirstChildElement("annotation");
  polygon = annotation->FirstChildElement("polygon");

  // -- For each poly, get the label and the contour.
  while(polygon) {
    poly_labels.push_back(label_str2int[polygon->Attribute("name")]);
    //cout << "polygon " << polygon->Attribute("name") << " " << polygon->Attribute("sqn") << endl;
    pt = polygon->FirstChildElement("pt");
    Vector<Point> contour;
    while(pt) {
      //cout << "pt " << pt->Attribute("x") << " " << pt->Attribute("y") << endl;
      contour.push_back(Point(atoi(pt->Attribute("x")), atoi(pt->Attribute("y"))));
      pt = pt->NextSiblingElement("pt");
    }
    contours.push_back(contour);
    polygon = polygon->NextSiblingElement("polygon");
  }
}

void findLabelPolys(double stamp, string results_dir, Vector< Vector<Point> >& polys, vector<int>& poly_labels) {
  assert(polys.empty());
  assert(poly_labels.empty());

  vector<string> files;

  string polys_dir = results_dir + "/annotations/";
  char buf[100];
  sprintf(buf, "%f", stamp); 
  string sbuf(buf);
  sbuf = sbuf.substr(0, sbuf.length()-1);   //Remove the last digit - it's been rounded.

  getDir(polys_dir, files);
  for(size_t i=0; i<files.size(); i++)
    {
      //cout << "searching for " << sbuf << " in " << files[i] << endl;
      if(files[i].find(sbuf) != string::npos && 
	 files[i].find(string(".xml")) != string::npos)
	{
	  cout << "Found polys for " << sbuf << ": " << files[i] << endl;
	  getContoursFromTinyXML(results_dir + "/annotations/" + files[i], polys, poly_labels);
	  break;
	}
    }
  
  if(polys.size() > 0) { 
    cout << " Found " << polys.size() << " polygon labels in " << results_dir << endl;
  }
}

void drawPointsInOrder(IplImage* img, const Vector<Point>& poly) {
  IplImage* vis = cvCreateImage(cvGetSize(img), 8, 1);
  for(size_t i=0; i<poly.size(); ++i) {
    CV_IMAGE_ELEM(vis, uchar, poly[i].y, poly[i].x) = 255;
    CVSHOW("points", vis);
    cvWaitKey(0);
  }
  cvReleaseImage(&vis);
}

void showLabelPolys(IplImage* img, const Vector< Vector<Point> >& polys) {
  CVSHOW("img", img);
  for(size_t l=0; l<polys.size(); ++l) {

    // -- Draw points in order.
    //drawPointsInOrder(img, polys[l]);

    IplImage* label = cvCreateImage(cvGetSize(img), 8, 1);
    cout << "Showing polygon " << l << endl;
    for(size_t p=0; p<polys[l].size(); ++p) {
      cout << "x " << polys[l][p].x << " y " << polys[l][p].y << endl;
    }

    for(int x=0; x<img->width; ++x) {
      for(int y=0; y<img->height; ++y) {
	bool inside; 
	inside = cv::pointPolygonTest(polys[l], Point2f(x, y), false) >= 0;
	if(inside) 
	  CV_IMAGE_ELEM(label, uchar, y, x) = 255;
      }
    }
    CVSHOW("polygon", label);
    cvWaitKey(0);
    cvReleaseImage(&label);
  }
}

void createLabelMaps(map<string, int>* str2int, vector<string>* int2str) {
  if(str2int) {
    map<string, int>& s2i = *str2int;
    s2i["Unlabeled"] = 0;
    s2i["Odwalla"] = 1;
    s2i["Naked"] = 2;
    s2i["Water"] = 3;
  }

  if(int2str) {
    vector<string>& i2s = *int2str;
    i2s.push_back("Unlabeled");
    i2s.push_back("Odwalla");
    i2s.push_back("Naked");
    i2s.push_back("Water");
  }
}
