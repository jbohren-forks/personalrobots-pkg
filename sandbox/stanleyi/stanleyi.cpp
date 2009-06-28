#include <iostream>
#include <image_msgs/Image.h>
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
#include "descriptors.h"
#include <dorylus.h>

using namespace NEWMAT;
using namespace std;


template <class T>
void copyMsg(string name, T* m, ros::Time t, ros::Time t_no_use, void* n)
{
  if (m != 0) {
    *((T*)(n)) = *((T*)(m));
  }
}
  

int getdir (string dir, vector<string> &files)
{
  DIR *dp;
  struct dirent *dirp;
  if((dp  = opendir(dir.c_str())) == NULL) {
    cout << "Error(" << errno << ") opening " << dir << endl;
    return errno;
  }

  while ((dirp = readdir(dp)) != NULL) {
    files.push_back(string(dirp->d_name));
  }
  closedir(dp);
  return 0;
}


IplImage* findLabelMask(double stamp) 
{
  vector<string> files;
  IplImage *mask = NULL;

  
  char buf[100];
  sprintf(buf, "%f", stamp); 
  string sbuf(buf);
  sbuf = sbuf.substr(0, sbuf.length()-1);   //Remove the last digit - it's been rounded.
  string dir = string("/tmp/turk_submit_images/images/");


  getdir(dir, files);
  for(unsigned int i=0; i<files.size(); i++)
    {
      if(files[i].find(sbuf) != string::npos && 
	 files[i].find(string(".png")) != string::npos)
	{
	  cout << "Found label for " << sbuf << ": " << files[i] << endl;
	  mask = cvLoadImage((dir + files[i]).c_str());
	  break;
	}
    }
  return mask;
}


class Stanleyi
{
public:
  ros::record::Player lp_;
  image_msgs::Image img_msg_;
  image_msgs::CvBridge img_bridge_;
  IplImage* img_;
  IplImage* mask_;
  vector<ImageDescriptor*> descriptor_;

  object* computeFeaturesAtRandomPoint(int* row=NULL, int* col=NULL, bool debug=false);


  Stanleyi()
    : img_(NULL), mask_(NULL)
  {
    lp_.addHandler<image_msgs::Image>(string("/forearm/image_rect_color"), &copyMsg<image_msgs::Image>, (void*)(&img_msg_));
    descriptor_ = setupImageDescriptors();
  }

  void collectDataset(string bagfile, int samples_per_img, string save_name);
  void viewLabels(string bagfile);
  void classifyVideoVis(string bagfile, Dorylus& d, int samples_per_img);
};

object* Stanleyi::computeFeaturesAtRandomPoint(int* row, int* col, bool debug) {
  int r = rand() % img_->height;
  int c = rand() % img_->width;
  Matrix* result = NULL;
  object* obj = new object;

  // -- Aim the descriptor functions at the new point.
  for(unsigned int j=0; j<descriptor_.size(); j++) {
    descriptor_[j]->setPoint(r, c);
  }

  // -- Set the label.
  if(mask_ != NULL) {
    CvScalar s = cvGet2D(mask_, r, c);
    obj->label = s.val[0];
    if(obj->label != 0) {
      obj->label = 1;
    }
  }

  // -- Compute all the descriptors.
  bool success = false;
  for(unsigned int j=0; j<descriptor_.size(); j++) {
    // -- For now, only accept points for which all features are computable.
    if(obj->label == 1 && getenv("DEBUG_POSITIVES") != NULL) 
      success = descriptor_[j]->compute(&result, true);
    else
      success = descriptor_[j]->compute(&result, debug);

    if(!success) {
      ROS_WARN("Descriptor %s failed.", descriptor_[j]->name_.c_str());
      return NULL;
    }

    obj->features[descriptor_[j]->name_] = result;
    ROS_ASSERT(result != NULL);
  }


  if(row!=NULL && col!=NULL) {
    *row = r;
    *col = c;
  }
  return obj;
}

void Stanleyi::collectDataset(string bagfile, int samples_per_img, string save_name) {
  bool debug = false;
  vector<object> objs;

  if(getenv("DEBUG") != NULL) {
    cout << "Entering debug mode." << endl;
    debug = true;
  }

  lp_.open(bagfile, ros::Time());

  // -- Get data for all labeled images.
  while(lp_.nextMsg()) {
    // -- Get the next img with a label mask.
    if (!img_bridge_.fromImage(img_msg_, "bgr")) 
      continue;
    img_ = img_bridge_.toIpl();
    mask_ = findLabelMask(img_msg_.header.stamp.toSec());
    if(mask_ == NULL)
      continue;
    
    ROS_ASSERT(img_ != NULL);
    
    // -- Aim the descriptor functions at the new image.
    for(unsigned int j=0; j<descriptor_.size(); j++) {
      descriptor_[j]->setImage(img_);
    }

    // -- Randomly sample points from the image and get the features.
    srand ( time(NULL) );
    for(int i=0; i<samples_per_img; i++) {
      object* obj = computeFeaturesAtRandomPoint(NULL, NULL, debug);

      // -- Add the object to the dataset.
      if(obj != NULL)
	objs.push_back(*obj);
	
    }
  }

  DorylusDataset dd;
  dd.setObjs(objs);
  dd.save(save_name);
}
  

void Stanleyi::classifyVideoVis(string bagfile, Dorylus& d, int samples_per_img) {
  cvNamedWindow("Classification Visualization", CV_WINDOW_AUTOSIZE);
  
  int row = 0, col = 0;
  IplImage* vis = NULL;
  object* obj = NULL;
  Matrix response;
  int frame_num = 0;

  int skip_frames = 184;
  if(getenv("SKIP_FRAMES") != NULL) 
    skip_frames = atoi(getenv("SKIP_FRAMES"));


  lp_.open(bagfile, ros::Time());
  for(int i=0; i<skip_frames; i++) {
    lp_.nextMsg();
    frame_num++;
  }

  while(lp_.nextMsg()) {
    frame_num++;
    if (!img_bridge_.fromImage(img_msg_, "bgr"))
      break;
  
    
    img_ = img_bridge_.toIpl();
    vis = cvCloneImage(img_);

    IplImage* mask = findLabelMask(img_msg_.header.stamp.toSec());
    if(getenv("DEBUG_POSITIVES") != NULL &&  mask == NULL)
      continue;

    // -- Aim the descriptor functions at the new image.
    for(unsigned int j=0; j<descriptor_.size(); j++) {
      descriptor_[j]->setImage(img_);
    }

    int tp=0, fp=0, tn=0, fn=0;
    for(int i=0; i<samples_per_img; i++) {
      obj = computeFeaturesAtRandomPoint(&row, &col);
      if(!obj)
	continue;
      response = d.classify(*obj);

      if(mask) {
	CvScalar s = cvGet2D(mask, row, col);
	cout << s.val[0] << " " << s.val[1] << " " << s.val[2] << " " << s.val[3] << endl;
	int label = s.val[0];
	if(label != 0 && response(1,1) > 0)
	  tp++;
	else if(label == 0 && response(1,1) > 0)
	  fp++;
	else if(label == 0 && response(1,1) <= 0)
	  tn++;
	else// if(label == 0 && response(1,1) > 0)
	  fn++;
      }

      if(response(1,1) > 0) {
	cvCircle(vis, cvPoint(col, row), 2, cvScalar(0,255,0), 2);
	if(mask)
	  cvCircle(mask, cvPoint(col, row), 2, cvScalar(0,255,0), 2);
      }
      else {
	cvCircle(vis, cvPoint(col, row), 2, cvScalar(0,0,255), 2);
	if(mask)
	  cvCircle(mask, cvPoint(col, row), 2, cvScalar(0,0,255), 2);
      }
    }
    if(mask) {
      cvNamedWindow("Mask", CV_WINDOW_AUTOSIZE);
      cvShowImage("Mask", mask);
      printf("TP: %d, TN: %d, FP: %d, FN: %d\n", tp, tn, fp, fn);
    }

    cout << "Showing results for frame " << frame_num << endl;
    cvShowImage("Classification Visualization", vis);
    if(cvWaitKey(0) == 'q') {
      return;
    }
  }
   
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
  

void Stanleyi::viewLabels(string bagfile) {
  cvNamedWindow("Image", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Mask", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Feature", CV_WINDOW_AUTOSIZE);

  lp_.open(bagfile, ros::Time());
  while(lp_.nextMsg()) {
    if (img_bridge_.fromImage(img_msg_, "bgr")) {
      img_ = img_bridge_.toIpl();
      mask_ = findLabelMask(img_msg_.header.stamp.toSec());
      if(mask_ != NULL) {
	cvShowImage("Image", img_);
	cvShowImage("Mask", mask_);
	test_watershed(img_);
	cvWaitKey(0);
      }
    }
  }
}
  
int main(int argc, char** argv) 
{
  Stanleyi s;
  
  // -- Get env var options.
  int samples_per_img = 1000;
  int nCandidates = 2;
  int max_secs = 0;
  int max_wcs = 2;
  if(getenv("NSAMPLES") != NULL) 
    samples_per_img = atoi(getenv("NSAMPLES"));
  if(getenv("NCAND") != NULL) 
    nCandidates = atoi(getenv("NCAND"));
  if(getenv("MAX_SECS") != NULL) 
    max_secs = atoi(getenv("MAX_SECS"));
  if(getenv("MAX_WCS") != NULL) 
    max_wcs = atoi(getenv("MAX_WCS"));


  // -- Parse args.
  if(argc > 2 && !strcmp(argv[1], "--viewlabels"))
    {
      cout << "Showing labels for " << argv[2] << endl;  
      s.viewLabels(string(argv[2]));
    }

  else if(argc > 3 && !strcmp(argv[1], "--collectDataset"))
    {
      cout << "Collecting a dataset for " << argv[2] << ", saving with name " << argv[3] << endl;  
      s.collectDataset(string(argv[2]), samples_per_img, string(argv[3]));
    }
  
  else if(argc > 2 && !strcmp(argv[1], "--statusDataset")) {
    cout << "Examining " << argv[2] << endl;
    DorylusDataset dd;
    dd.load(string(argv[2]));
    //    cout << dd.displayFeatures() << endl;
    cout << dd.status() << endl;
  }

  else if(argc > 2 && !strcmp(argv[1], "--statusClassifier")) {
    cout << "Examining " << argv[2] << endl;
    Dorylus d;
    d.load(string(argv[2]));
    //    cout << dd.displayFeatures() << endl;
    cout << d.status() << endl;
  }

  else if(argc > 2 && !strcmp(argv[1], "--train")) {

    cout << "Training new classifier on " << argv[2] << ", saving with name " << argv[3] << " using " << nCandidates << " candidates, training for " << max_secs << "s or " << max_wcs << " wcs." << endl;
    Dorylus d;
    DorylusDataset dd;
    dd.load(string(argv[2]));
    d.useDataset(&dd);
    d.train(nCandidates, max_secs, max_wcs);
    d.save(string(argv[3]));
  }
  else if(argc > 2 && !strcmp(argv[1], "--classifyVideoVis")) {
    cout << "Showing video classification for " << argv[2] << " on " << argv[3] << endl;
    Dorylus d;
    d.load(string(argv[2]));
    s.classifyVideoVis(string(argv[3]), d, samples_per_img);
  }

  else {
    cout << "usage: " << endl;
  }
  
}

