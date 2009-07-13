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
#include "descriptors.h"


USING_PART_OF_NAMESPACE_EIGEN
//using namespace NEWMAT;
using namespace std;
using namespace cv;

vector<ImageDescriptor*> setupImageDescriptors() {
  vector<ImageDescriptor*> d;

// Size winSize, Size blockSize, Size blockStride, Size cellSize,
//   int nbins, int derivAperture=1, double winSigma=-1,
//   int histogramNormType=L2Hys, double L2HysThreshold=0.2, bool gammaCorrection=false)
//  d.push_back(new HogWrapper());
  d.push_back(new HogWrapper(Size(16,16), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(32,32), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(16,16), 7, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(128,128), Size(64,64), Size(32,32), Size(32,32), 7, 1, -1, 0, 0.2, true));

  SuperpixelColorHistogram* sch1 = new SuperpixelColorHistogram(20, 0.5, 10, string("hue"));
  SuperpixelColorHistogram* sch2 = new SuperpixelColorHistogram(5, 0.5, 10, string("hue"), NULL, sch1);
  SuperpixelColorHistogram* sch3 = new SuperpixelColorHistogram(5, 1, 10, string("hue"), NULL, sch1);
  SuperpixelColorHistogram* sch4 = new SuperpixelColorHistogram(5, .25, 10, string("hue"), NULL, sch1);
  d.push_back(sch1);
  d.push_back(sch2);
  d.push_back(sch3);
  d.push_back(sch4);

//   IntegralImageTexture* iit = new IntegralImageTexture(1);
//   d.push_back(iit);
//   d.push_back(new IntegralImageTexture(2, iit));
//   d.push_back(new IntegralImageTexture(3, iit));

//   d.push_back(new IntensityPatch(40, .5, true));
//   d.push_back(new IntensityPatch(20, 1, true));
//   d.push_back(new IntensityPatch(80, .25, true));
//   d.push_back(new IntensityPatch(120, 1.0/6.0, true));
  return d;
}

vector<ImageDescriptor*> setupImageDescriptorsTest() {
  vector<ImageDescriptor*> d;

  d.push_back(new IntegralImageTexture());
  return d;
}


void releaseImageDescriptors(vector<ImageDescriptor*>* desc) {
  for(unsigned int i=0; i<desc->size(); i++) {
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


IplImage* findLabelMask(double stamp, string results_dir) 
{
  vector<string> files;
  IplImage *mask = NULL;

  string masks_dir = results_dir + "/masks/";
  char buf[100];
  sprintf(buf, "%f", stamp); 
  string sbuf(buf);
  sbuf = sbuf.substr(0, sbuf.length()-1);   //Remove the last digit - it's been rounded.

  getdir(masks_dir, files);
  for(unsigned int i=0; i<files.size(); i++)
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


class Stanleyi
{
public:
  ros::record::Player lp_;
  sensor_msgs::Image img_msg_;
  sensor_msgs::CvBridge img_bridge_;
  IplImage* img_;
  IplImage* mask_;
  vector<ImageDescriptor*> descriptor_;

  object* computeObjectAtRandomPoint(int* row=NULL, int* col=NULL);
  vector<object*> collectFeaturesFromImage(int samples_per_img);
  void induct(string bagfile, string results_dir);
  void collectObjectsFromImageVectorized(int samples_per_img, vector<object*>* objects, Vector<Keypoint>* keypoints);
  void sanityCheck(string bagfile, string results_dir);

  Stanleyi()
    : img_(NULL), mask_(NULL)
  {
    lp_.addHandler<sensor_msgs::Image>(string("/forearm/image_rect_color"), &copyMsg<sensor_msgs::Image>, (void*)(&img_msg_));
    descriptor_ = setupImageDescriptors();
    if(getenv("DEBUG") != NULL) {
      for(unsigned int i=0; i<descriptor_.size(); i++) { 
	descriptor_[i]->setDebug(true);
      }
    }
    srand ( time(NULL) ); //For randomly selecting points.
  }

  ~Stanleyi() {
    cout << "Destroying stanleyi." << endl;
    releaseImageDescriptors(&descriptor_);
//     if(img_)
//       cvReleaseImage(&img_);
    if(mask_)
      cvReleaseImage(&mask_);
  }

  void collectDataset(string bagfile, int samples_per_img, string save_name, string results_dir);
  void viewLabels(string bagfile, string results_dir);
  void makeClassificationVideo(string bagfile, Dorylus& d, int samples_per_img, string results_dir);
};

object* Stanleyi::computeObjectAtRandomPoint(int* row, int* col) {
  int r = rand() % img_->height;
  int c = rand() % img_->width;
  MatrixXf* result = NULL;
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
    if(mask_ != NULL && obj->label == 1 && getenv("DEBUG_POSITIVES") != NULL)  {
      descriptor_[j]->setDebug(true);
      success = descriptor_[j]->compute(&result);
      descriptor_[j]->setDebug(false);
    }
    else
      success = descriptor_[j]->compute(&result);

    if(!success) {
      //ROS_WARN("Descriptor %s failed.", descriptor_[j]->name_.c_str());
      delete obj;
      return NULL;
    }

    ROS_ASSERT(result != NULL);
    obj->features[descriptor_[j]->name_] = result;
  }


  if(row!=NULL && col!=NULL) {
    *row = r;
    *col = c;
  }
  return obj;
}

void Stanleyi::collectDataset(string bagfile, int samples_per_img, string save_name, string results_dir) {
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
    
    Vector<Keypoint> keypoints;
    collectObjectsFromImageVectorized(samples_per_img, &objs, &keypoints);

    cvReleaseImage(&mask_);
  }

  DorylusDataset dd;
  dd.setObjs(objs);
  cout << dd.status() << endl;
  dd.save(save_name);
}

MatrixXf* cvVector2Eigen(const Vector<float>& v) {
  MatrixXf* m = new MatrixXf(v.size(), 1);
  for(unsigned int i=0; i<v.size(); i++) {
    (*m)(i,0) = v[i];
  }
  return m;
}


  //! Appends to objects.  If an object is invalid, a NULL pointer is used so that keypoints[i] corresponds to objects[i].
void Stanleyi::collectObjectsFromImageVectorized(int samples_per_img, vector<object*>* objects, Vector<Keypoint>* keypoints) {
  vector<vvf> results(descriptor_.size());
  Vector<Keypoint> desired;

  // -- Choose random locations and make keypoints.
  keypoints->clear();
  keypoints->reserve(samples_per_img);
  desired.reserve(samples_per_img);
  vector<int> labels(samples_per_img);
  for(int i=0; i<samples_per_img; i++)  {
    int r = rand() % img_->height;
    int c = rand() % img_->width;
    desired.push_back(Keypoint(c, r));

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
  for(unsigned int i=0; i<descriptor_.size(); i++) {
    Mat m = cvarrToMat(img_);
    descriptor_[i]->compute(img_, desired, results[i]);
    //cout << "Got " << results[i].size() << " features" << endl;
    //cout << "with " << results[i][0].size() << " elements each. " << endl;
  }


  // -- Copy into objects.
  for(int i=0; i<samples_per_img; i++)  {
    object* obj = new object;
    obj->label = labels[i];

    bool success = true;
    //cout << descriptor_.size() << " descriptors.  " << results.size() << endl;
    for(unsigned int j=0; j<descriptor_.size(); j++) {
      //cout << samples_per_img << " objects.  " << results[j].size() << endl;
      //cout << descriptor_[j]->result_size_ << " elem.  " << results[j][i].size() << endl;

      if(results[j][i].empty()) {
	success = false;
	break;
      }
      obj->features[descriptor_[j]->name_] = cvVector2Eigen(results[j][i]);
    }
    if(success) {
      objects->push_back(obj);
      keypoints->push_back(desired[i]);
    }
    else {
      delete obj;
    }
  }


  cout << "sizes: " <<  objects->size() << " " << keypoints->size() << endl;
}

vector<object*> Stanleyi::collectFeaturesFromImage(int samples_per_img) {
  vector<object*> objs;

  // -- Aim the descriptor functions at the new image.
  for(unsigned int j=0; j<descriptor_.size(); j++) {
    descriptor_[j]->setImage(img_);
  }
  
  // -- Randomly sample points from the image and get the features.
  for(int i=0; i<samples_per_img; i++) {
    object* obj = computeObjectAtRandomPoint(NULL, NULL);
    
    // -- Add the object to the dataset.
    if(obj != NULL) {
      objs.push_back(obj);
    }
  }
  return objs;
}

void Stanleyi::sanityCheck(string bagfile, string results_dir) {
  // -- Find first labeled image.
  // -- Collect dataset on that image.
  // -- Train classifier.
  // -- Make predictions on that image.
}

void Stanleyi::makeClassificationVideo(string bagfile, Dorylus& d, int samples_per_img, string results_dir) {
  //vector<ImageDescriptor*> desc = setupImageDescriptors();
  //d.exclude_descriptors_.push_back(desc[4]->name_);
  if(getenv("MAX_WC") != NULL)
    d.max_wc_ = atoi(getenv("MAX_WC"));
  

  cvNamedWindow("Classification Visualization", CV_WINDOW_AUTOSIZE);
  

  int row = 0, col = 0;
  IplImage* vis = NULL;
  NEWMAT::Matrix response_nm;
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

  while(lp_.nextMsg()) {
    frame_num++;
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


    Vector<Keypoint> keypoints;
    objects.clear();
    collectObjectsFromImageVectorized(samples_per_img, &objects, &keypoints);

    int tp=0, fp=0, tn=0, fn=0;
    for(int i=0; i<objects.size(); i++) {
      if(objects[i] == NULL) 
	continue;
      //cout << objects[i]->status(false) << endl;
      response_nm = d.classify(*objects[i]);
      col = keypoints[i].pt.x;
      row = keypoints[i].pt.y;

      if(mask) {
	CvScalar s = cvGet2D(mask, row, col);
	//cout << s.val[0] << " " << s.val[1] << " " << s.val[2] << " " << s.val[3] << endl;
	int label = s.val[0];
	if(label != 0 && response_nm(1,1) > 0)
	  tp++;
	else if(label == 0 && response_nm(1,1) > 0)
	  fp++;
	else if(label == 0 && response_nm(1,1) <= 0)
	  tn++;
	else// if(label == 0 && response_nm(1,1) > 0)
	  fn++;
      }

      //cout << response_nm.Nrows() << " " << response_nm.Ncols() << endl;

      if(response_nm(1,1) > 0) {
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

    cvWriteFrame(writer, vis);

    cout << "Showing results for frame " << frame_num << endl;
    cvShowImage("Classification Visualization", vis);
    if(cvWaitKey(30) == 'q') {
      break;
    }

    cvReleaseImage(&vis);
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
  

void Stanleyi::induct(string bagfile, string results_dir) {
  int row = 0, col = 0;
  IplImage* vis = NULL;
  object* obj = NULL;
  NEWMAT::Matrix response_nm;
  CvVideoWriter* writer = NULL;
  int frame_num = 0;

  int samples_per_img = 3000;
  if(getenv("NSAMPLES") != NULL) 
    samples_per_img = atoi(getenv("NSAMPLES"));
  int nCandidates = 10;
  if(getenv("NCAND") != NULL) 
    nCandidates = atoi(getenv("NCAND"));
  int max_secs = 60;
  if(getenv("MAX_SECS") != NULL) 
    max_secs = atoi(getenv("MAX_SECS"));
  int max_wcs = 0;
  if(getenv("MAX_WCS") != NULL) 
    max_wcs = atoi(getenv("MAX_WCS"));


  cvNamedWindow("Inductive Classification Visualization", CV_WINDOW_AUTOSIZE);
  lp_.open(bagfile, ros::Time());
  
  // -- Find first labeled frame.
  while(lp_.nextMsg()) {
    img_ = img_bridge_.toIpl();
    if (!img_bridge_.fromImage(img_msg_, "bgr")) {
      ROS_ERROR("Could not convert message to ipl.");
      break;
    }

    mask_ = findLabelMask(img_msg_.header.stamp.toSec(), results_dir);
    if(mask_)
      break;
  }
  ROS_DEBUG("Found mask.");

  // -- Collect a dataset from it and train a classifier.
  vector<object*> objs = collectFeaturesFromImage(samples_per_img);

  // -- Main loop.
  while(lp_.nextMsg()) {
    DorylusDataset dd;
    dd.setObjs(objs);

    // -- Train a classifier on dd.
    Dorylus d;  
    d.useDataset(&dd);
    d.train(nCandidates, max_secs, max_wcs);

    // -- Get the next image.
    img_ = img_bridge_.toIpl();
    vis = cvCloneImage(img_);
    if (!img_bridge_.fromImage(img_msg_, "bgr"))
      break;
    if(writer == NULL) {
      writer = cvCreateVideoWriter("output.mpg", CV_FOURCC('P','I','M','1'), 30, cvGetSize(img_));
    }

    // -- Aim the descriptor functions at the new image.
    for(unsigned int j=0; j<descriptor_.size(); j++) {
      descriptor_[j]->setImage(img_);
    }

    // -- Compute features across the image.
    for(int i=0; i<samples_per_img; i++) {
      obj = computeObjectAtRandomPoint(&row, &col);
      if(!obj)
	continue;

      // -- Get predictions and create a dataset from them for this image.
      response_nm = d.classify(*obj);
      if(response_nm(1,1) > 0) 
	obj->label = 1;
      else
	obj->label = 0;
      objs.push_back(obj);

      // -- Draw the visualization.
      if(response_nm(1,1) > 0)
	cvCircle(vis, cvPoint(col, row), 2, cvScalar(0,255,0), 2);
      else
	cvCircle(vis, cvPoint(col, row), 2, cvScalar(0,0,255), 2);
    }

    cvWriteFrame(writer, vis);

    cout << "Showing results for frame " << frame_num << endl;
    cvShowImage("Inductive Classification Visualization", vis);
    if(cvWaitKey(30) == 'q') {
      break;
    }

    cvReleaseImage(&vis);
  }
  cvReleaseVideoWriter(&writer);
}

int main(int argc, char** argv) 
{
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
      s.collectDataset(argv[2], samples_per_img, argv[4], argv[3]);
    }
  
  else if(argc > 2 && !strcmp(argv[1], "--statusD")) {
    cout << "Examining " << argv[2] << endl;
    DorylusDataset dd;
    dd.load(argv[2]);
    //    cout << dd.displayFeatures() << endl;
    cout << dd.status() << endl;
  }

  else if(argc > 2 && !strcmp(argv[1], "--statusClassifier")) {
    cout << "Examining " << argv[2] << endl;
    Dorylus d;
    d.load(argv[2]);
    //    cout << dd.displayFeatures() << endl;
    cout << d.status() << endl;
  }

  else if(argc > 2 && !strcmp(argv[1], "--train")) {

    cout << "Training new classifier on " << argv[2] << ", saving with name " << argv[3] << " using " << nCandidates << " candidates, training for " << max_secs << "s or " << max_wcs << " wcs." << endl;
    Dorylus d;
    DorylusDataset dd;
    dd.load(argv[2]);
    d.useDataset(&dd);
    d.train(nCandidates, max_secs, max_wcs);
    d.save(argv[3]);
  }
  else if(argc > 4 && !strcmp(argv[1], "--makeClassificationVideo")) {
    cout << "Showing video classification for classifier " << argv[2] << " on bag " << argv[3] << " using the label masks in " << argv[4] <<  endl;
    Dorylus d;
    d.load(argv[2]);
    s.makeClassificationVideo(argv[3], d, samples_per_img, argv[4]);
  }
  else if(argc > 3 && !strcmp(argv[1], "--induct")) {
    cout << "Running inductive video classification on bag " << argv[2] << " using the label masks in " << argv[3] <<  endl;
    s.induct(argv[2], argv[3]);
  }

  else {
    cout << "usage: " << endl;
    cout << argv[0] << " --makeClassificationVideo CLASSIFIER BAGFILE [LABELS]" << endl;
    cout << argv[0] << " --collectDataset BAGFILE LABELS DATASET" << endl;
    cout << argv[0] << " --viewLabels BAGFILE LABELS" << endl;
    cout << argv[0] << " --statusD DATASET" << endl;
    cout << argv[0] << " --induct BAGFILE LABELS [CLASSIFIER] [DATASET]" << endl;
    cout << "  where LABELS might take the form of `rospack find cv_mech_turk`/results/single-object-s " << endl;
  }
  
}

