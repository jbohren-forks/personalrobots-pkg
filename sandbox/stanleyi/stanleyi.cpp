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
#include <fstream>

USING_PART_OF_NAMESPACE_EIGEN
using namespace std;
using namespace cv;

// -- Utility functions.
vector<ImageDescriptor*> setupImageDescriptors();
void releaseImageDescriptors(vector<ImageDescriptor*>* desc);
template <class T> void copyMsg(string name, T* m, ros::Time t, ros::Time t_no_use, void* n);
int getdir (string dir, vector<string> &files);
IplImage* findLabelMask(double stamp, string results_dir);


  
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
  void collectObjectsFromImageVectorized(int samples_per_img, vector<object*>* objects, Vector<Keypoint>* keypoints);
};

Stanleyi::Stanleyi()
  : img_(NULL), mask_(NULL)
{
  lp_.addHandler<sensor_msgs::Image>(string("/forearm/image_rect_color"), &copyMsg<sensor_msgs::Image>, (void*)(&img_msg_));
  descriptor_ = setupImageDescriptors();
  if(getenv("DEBUG") != NULL) {
    for(size_t i=0; i<descriptor_.size(); i++) { 
      descriptor_[i]->debug_ = true;
    }
  }
  srand ( time(NULL) ); //For randomly selecting points.
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
    
    Vector<Keypoint> keypoints;
    collectObjectsFromImageVectorized(samples_per_img, &objs, &keypoints);

    cvReleaseImage(&mask_);
  }

  DorylusDataset* dd = new DorylusDataset();
  dd->setObjs(objs);
  return dd;
}

MatrixXf* cvVector2Eigen(const Vector<float>& v) {
  MatrixXf* m = new MatrixXf(v.size(), 1);
  for(size_t i=0; i<v.size(); i++) {
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
    int size = 1;
    desired.push_back(Keypoint(c, r, size));

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

    bool success = true;
    //cout << descriptor_.size() << " descriptors.  " << results.size() << endl;
    for(size_t j=0; j<descriptor_.size(); j++) {
      //cout << samples_per_img << " objects.  " << results[j].size() << endl;
      //cout << descriptor_[j]->result_size_ << " elem.  " << results[j][i].size() << endl;

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
  Vector<Keypoint> keypoints;
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
    objs.clear();
    keypoints.clear();
    collectObjectsFromImageVectorized(samples_per_img, &objs, &keypoints);
    for(size_t i=0; i<objs.size(); i++) {
      if(objs[i] == NULL) 
	continue;

      NEWMAT::Matrix response_nm = d.classify(*objs[i]);
      int col = keypoints[i].pt.x;
      int row = keypoints[i].pt.y;

      drawResponse(vis_img, response_nm(1,1), cvPoint(col, row));
      drawResponse(vis_mask, response_nm(1,1), cvPoint(col, row));
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


    Vector<Keypoint> keypoints;
    objects.clear();
    collectObjectsFromImageVectorized(samples_per_img, &objects, &keypoints);

    int tp=0, fp=0, tn=0, fn=0;
    for(size_t i=0; i<objects.size(); i++) {
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

      int size = ceil(log(ceil(abs(response_nm(1,1)))));
      if(response_nm(1,1) > 0) {
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

// Size winSize, Size blockSize, Size blockStride, Size cellSize,
//   int nbins, int derivAperture=1, double winSigma=-1,
//   int histogramNormType=L2Hys, double L2HysThreshold=0.2, bool gammaCorrection=false)
//  d.push_back(new HogWrapper());
//  d.push_back(new HogWrapper(Size(16,16), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(32,32), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(16,16), 7, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(128,128), Size(64,64), Size(32,32), Size(32,32), 7, 1, -1, 0, 0.2, true));

//   SuperpixelColorHistogram* sch1 = new SuperpixelColorHistogram(20, 0.5, 10, string("hue"));
//   SuperpixelColorHistogram* sch2 = new SuperpixelColorHistogram(5, 0.5, 10, string("hue"), NULL, sch1);
//   SuperpixelColorHistogram* sch3 = new SuperpixelColorHistogram(5, 1, 10, string("hue"), NULL, sch1);
//   SuperpixelColorHistogram* sch4 = new SuperpixelColorHistogram(5, .25, 10, string("hue"), NULL, sch1);
//   d.push_back(sch1);
//   d.push_back(sch2);
//   d.push_back(sch3);
//   d.push_back(sch4);
 
  
//  d.push_back(new ContourFragmentDescriptor(0, "contour_fragments"));
//   IntegralImageTexture* iit = new IntegralImageTexture(1);
//   d.push_back(iit);
//   d.push_back(new IntegralImageTexture(2, iit));
//   d.push_back(new IntegralImageTexture(3, iit));

// -- SURF.
  d.push_back(new SurfWrapper(true, 150));
  d.push_back(new SurfWrapper(true, 100));
  d.push_back(new SurfWrapper(true, 50));
  d.push_back(new SurfWrapper(true, 25));
  d.push_back(new SurfWrapper(true, 10));

// -- Haar.
//   vector<ImageDescriptor*> haar = setupDefaultHaarDescriptors();
//   d.insert(d.end(), haar.begin(), haar.end());

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

  getdir(masks_dir, files);
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
