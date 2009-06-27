#include "descriptors.h"
#include "ros/console.h"
#include "ros/assert.h"

using namespace std;
using namespace NEWMAT;

/****************************************************************************
*************  Generally Useful Functions
*****************************************************************************/


vector<ImageDescriptor*> setupImageDescriptors() {
  vector<ImageDescriptor*> d;
//   d.push_back(new IntensityPatch(50, 1, true));
//   d.push_back(new PatchStatistic(string("variance"), (Patch*)d.back()));
//   d.push_back(new IntensityPatch(40, .5, true));
//   d.push_back(new IntensityPatch(20, 1, true));
//   d.push_back(new IntensityPatch(80, .25, true));
//   d.push_back(new IntensityPatch(120, 1.0/6.0, true));
  SuperpixelColorHistogram* sch1 = new SuperpixelColorHistogram(20, 0.5, 25, string("hue"));
  SuperpixelColorHistogram* sch2 = new SuperpixelColorHistogram(5, 0.5, 25, string("hue"), NULL, sch1);
  d.push_back(sch1);
  d.push_back(sch2);
//   d.push_back(new SuperpixelColorHistogram(20, 0.5, 25, string("sat"), sch1, sch1));
//   d.push_back(new SuperpixelColorHistogram(20, 0.5, 25, string("val"), sch1, sch1));
//   d.push_back(new SuperpixelColorHistogram(5, 0.5, 25, string("sat"), sch2, sch1));
//   d.push_back(new SuperpixelColorHistogram(5, 0.5, 25, string("val"), sch2, sch1));
//   d.push_back(new SuperpixelColorHistogram(20, 0.5, 10, string("hue"), sch1, sch1));
//   d.push_back(new SuperpixelColorHistogram(20, 0.5, 10, string("sat"), sch1, sch1));
//   d.push_back(new SuperpixelColorHistogram(20, 0.5, 10, string("val"), sch1, sch1));
//   d.push_back(new SuperpixelColorHistogram(5, 0.5, 10, string("hue"), sch2, sch1));
//   d.push_back(new SuperpixelColorHistogram(5, 0.5, 10, string("sat"), sch2, sch1));
//   d.push_back(new SuperpixelColorHistogram(5, 0.5, 10, string("val"), sch2, sch1));

  //  d.push_back(new SuperpixelColorHistogram(5, 0.5, 16));
  //d.push_back(new SuperpixelColorHistogram(20, 0.25, 10));
  return d;
}


void whiten(Matrix* m) {
  float var=0.0;
  float mean = m->Sum() / m->Nrows();
  if(mean == 0.0) //This should only happen if the feature is the zero vector.
    return;

  for(int i=1; i<=m->Nrows(); i++) {
    (*m)(i,1) = (*m)(i,1) - mean;
    var += pow((*m)(i,1), 2);
  }
  var /= m->Nrows();

  Matrix div(1,1); div = 1/(sqrt(var));
  *m = KP(*m, div);
}


/****************************************************************************
*************  ImageDescriptor
****************************************************************************/

void ImageDescriptor::setImage(IplImage* img) {
  img_ = img;
  clearImageCache();
}

void ImageDescriptor::setPoint(int row, int col) {
  row_ = row;
  col_ = col;
  clearPointCache();
}

void ImageDescriptor::commonDebug() {
  IplImage* display = cvCloneImage(img_);
  cvResetImageROI(display);
  CvFont* numbFont = new CvFont();
  cvInitFont( numbFont, CV_FONT_VECTOR0, 1.0f, 1.0f, 0, 1);
  cvPutText(display, "+", cvPoint(col_,row_), numbFont, cvScalar(0,0,255));
  cvNamedWindow("Input Image");
  cvShowImage("Input Image", display);
  cvWaitKey(0);
  cvReleaseImage(&display);
}

bool ImageDescriptor::compute(Matrix** result, bool debug) {
  return false;
}

void ImageDescriptor::display(const NEWMAT::Matrix& result) {
  cout << "Displaying..." << endl;
}


/****************************************************************************
*************  ImageDescriptor::Patch
****************************************************************************/


Patch::Patch(int raw_size, float scale) 
  : raw_size_(raw_size), scale_(scale), final_patch_(NULL), scaled_patch_(NULL)
{
  //Common patch constructor computation.
  cout << "Doing common constructor computation." << endl;
  size_ = (int) ((float)raw_size * scale);
  if(size_%2==0)
    size_ -= 1;

  
  char buf[100];
  sprintf(buf, "Patch_sz%d_scale%g", raw_size_, scale_);
  name_.assign(buf);
}

bool Patch::preCompute(bool debug) {
  if(debug)
    cout << "Computing " << name_ << endl;
  
  // -- final_patch_ should have been deallocated on clearPointCache(), which must be called by the user.
  if(final_patch_ != NULL || scaled_patch_ != NULL)
    cout << "WARNING: final_patch_  or scaled_patch_ has not been deallocated.  Have you called clearPointCache()?" << endl;

  // -- Catch edge cases.
  int half = ceil((float)raw_size_ / 2.0);
  if(row_-half < 0 || row_+half >= img_->height || col_-half < 0 || col_+half >= img_->width) {
    cout << "Out of bounds; size: " << raw_size_ << ", row: " << row_ << ", col_: " << col_ << endl;
    return false;
  }
  
  // -- Get the scaled patch.
  //cout << "Setting roi to " << row_-half << " " << col_-half << " " << raw_size_ << endl;
  cvSetImageROI(img_, cvRect(col_-half, row_-half, raw_size_, raw_size_));
  scaled_patch_ = cvCreateImage(cvSize(size_, size_), img_->depth, img_->nChannels);
  cvResize(img_, scaled_patch_,CV_INTER_AREA);
  
  return true;
}
     
void Patch::clearPointCache() {
  cvReleaseImage(&scaled_patch_);
  cvReleaseImage(&final_patch_);
  final_patch_ = NULL;
  scaled_patch_ = NULL;
}

//void Patch::clearImageCache() {}

/****************************************************************************
*************  ImageDescriptor::Patch::IntensityPatch
****************************************************************************/

IntensityPatch::IntensityPatch(int raw_size, float scale, bool whiten)
  : Patch(raw_size, scale), whiten_(whiten)
{
  char buf[100];
  sprintf(buf, "_whiten%d_Intensity", whiten_);
  name_.append(buf);
  cout << "Creating " << name_ << endl;

  result_size_ = size_ * size_;
}

bool IntensityPatch::compute(NEWMAT::Matrix** result, bool debug) {
  //Do common patch processing.  
  if(!preCompute(debug))
    return false;


  final_patch_ = cvCreateImage(cvSize(size_, size_), IPL_DEPTH_8U, 1);
  cvCvtColor(scaled_patch_, final_patch_, CV_BGR2GRAY);
  Matrix* res = new Matrix(result_size_,1);

  // -- Convert ipl to Newmat.
  int idx=0;
  for(int r=0; r<final_patch_->height; r++) {
    uchar* ptr = (uchar*)(final_patch_->imageData + r * final_patch_->widthStep);
    for(int c=0; c<final_patch_->width; c++) {
      (*res)(idx+1, 1) = *ptr;
      ptr++;
      idx++;
    }
  }

  // -- Set mean to 0, variance to 1 if appropriate and desired.
  if(whiten_)
    whiten(res);
  
  *result = res;

  // -- Display for debugging.
  if(debug) {
    cout << name_ << " dump: ";
    cout << (**result).t() << endl;
      
    IplImage* final_patch_rescaled = cvCreateImage(cvSize(500,500), IPL_DEPTH_8U, 1);
    cvResize(final_patch_, final_patch_rescaled, CV_INTER_NN);
    cvNamedWindow(name_.c_str());
    cvShowImage(name_.c_str(), final_patch_rescaled);
    commonDebug();

    cvReleaseImage(&final_patch_rescaled);
  }
   
  // -- Clean up.
  cvResetImageROI(img_);
  return true;
}

     
/****************************************************************************
*************  ImageDescriptor::PatchStatistic
****************************************************************************/

PatchStatistic::PatchStatistic(string type, Patch* patch) :
  type_(type), patch_(patch)
{
  char buf[200];
  sprintf(buf, "%s_PatchStatistic_%s", patch_->name_.c_str(), type_.c_str());
  name_.assign(buf);
  cout << "Creating " << name_ << endl;

  if(type_.compare("variance") == 0) {  
    result_size_ = 1;
  }
}

bool PatchStatistic::compute(NEWMAT::Matrix** result, bool debug) {

  if(patch_ == NULL) {
    cout << "patch_ was null" << endl;
    return false; 
  }
  if(patch_->final_patch_ == NULL) {
    cout << "final_patch_ was null" << endl;
    return false; 
  }

  IplImage* fp = patch_->final_patch_;
  if(type_.compare("variance") == 0) {  
    (*result) = new Matrix(1,1);

    // -- Get the mean.
    double mean = 0.0;
    for(int r=0; r<fp->height; r++) {
      uchar* ptr = (uchar*)(fp->imageData + r * fp->widthStep);
      for(int c=0; c<fp->width; c++) {
	mean += (double)*ptr;
      }
    }
    mean /= (double)(fp->height * fp->width);
    //cout << "Mean is " << mean << endl;    

    // -- Compute variance.
    double var = 0.0;
    double tmp = 0.0;
    for(int r=0; r<fp->height; r++) {
      uchar* ptr = (uchar*)(fp->imageData + r * fp->widthStep);
      for(int c=0; c<fp->width; c++) {
	tmp = (double)(*ptr) - mean;
	var += tmp * tmp;
	//cout << "tmp " << tmp << " var " << var << endl;
      }
    }
    //cout << "Total pts " << (double)(fp->height * fp->width) << endl;
    var /= (double)(fp->height * fp->width);
    
    (**result)(1,1) = var;
  }

  if(debug) {
    display(**result);
//     cvNamedWindow("fp");
//     cvShowImage("fp", fp);
    commonDebug();
  }

  return true;
}

void PatchStatistic::display(const NEWMAT::Matrix& result) {
  cout << name_ << " is " << result << endl;
}

/***************************************************************************
***********  ImageDescriptor::SuperpixelStatistic
****************************************************************************/

SuperpixelStatistic::SuperpixelStatistic(int seed_spacing, float scale, SuperpixelStatistic* seg_provider) :
  seed_spacing_(seed_spacing), scale_(scale), seg_provider_(seg_provider), seg_(NULL)
{
  char buf[100];
  sprintf(buf, "SuperpixelStatistic_seedSpacing%d_scale%g", seed_spacing_, scale_);
  name_ = string(buf);
}

void SuperpixelStatistic::segment(bool debug) {
  ROS_DEBUG_COND(seg_ != NULL, "seg_ is not null, but is being recreated anyway!");
  
  // -- Downsample image.
  IplImage* img_small = cvCreateImage(cvSize(((float)img_->width)*scale_, ((float)img_->height)*scale_), img_->depth, img_->nChannels);
  cvResize(img_, img_small, CV_INTER_AREA);

  // -- Create a grid of seed points.
  IplImage* seg_small = cvCreateImage( cvGetSize(img_small), IPL_DEPTH_32S, 1 );
  cvZero(seg_small);
  long label = 1;
  for(int r=0; r<seg_small->height; r++) {
    long* ptr = (long*)(seg_small->imageData + r * seg_small->widthStep);
    for(int c=0; c<seg_small->width; c++) {
      if(c%seed_spacing_==0 && r%seed_spacing_==0) {
	*ptr = label;
	label++;
      }
      ptr++;
    }
  }

  // -- Compute segmentation.
  double t = (double)cvGetTickCount();
  cvWatershed( img_small, seg_small );
  t = (double)cvGetTickCount() - t;

  // -- Remove -1 (boundary) entries by nearest neighbor.
  for(int r=0; r<seg_small->height; r++) {
    long* ptr = (long*)(seg_small->imageData + r * seg_small->widthStep);
    for(int c=0; c<seg_small->width; c++) {
      if(*ptr < 1) {
	if(c<seg_small->width-1 && ptr[1] > 0) {
	  *ptr = ptr[1];
	  ptr++; 
	  continue;
	}
	else if(c>0 && ptr[-1] > 0) {
	  *ptr = ptr[-1];
	  ptr++; 
	  continue;
	}
	else if(r>0) {
	  long* ptr2 = (long*)(seg_small->imageData + r-1 * seg_small->widthStep);
	  if(ptr2[c] > 0) {
	    *ptr = ptr2[c];
	    ptr++; 
	    continue;
	  }
	}
	else if(r<seg_small->height-1) {
	  long* ptr2 = (long*)(seg_small->imageData + r+1 * seg_small->widthStep);
	  if(ptr2[c] > 0) {
	    *ptr = ptr2[c];
	    ptr++; 
	    continue;
	  }
	}
	else {
	  cout << "j" << endl;
	  ROS_DEBUG("This should not happen!");
	}
      }
      ptr++;
    }
  }

  // -- Enlarge segmentation back to size of image.
  seg_ = cvCreateImage( cvGetSize(img_), IPL_DEPTH_32S, 1 );
  cvResize(seg_small, seg_, CV_INTER_NN);

  // -- Compute the index.
  for(int r=0; r<seg_->height; r++) {
    long* ptr = (long*)(seg_->imageData + r * seg_->widthStep);
    for(int c=0; c<seg_->width; c++) {
      index_[*ptr].push_back(cvPoint(c,r));
      ptr++;
    }
  }

  if(debug) {
    cout << "Debugging " << name_.c_str() << endl;
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
    IplImage* wshed = cvCloneImage( img_ );
    IplImage* img_gray = cvCloneImage( img_ );
    cvZero( wshed );
    IplImage* marker_mask = cvCreateImage( cvGetSize(img_), 8, 1 );
    cvCvtColor( img_, marker_mask, CV_BGR2GRAY );
    cvCvtColor( marker_mask, img_gray, CV_GRAY2BGR );

    for(int i = 0; i < seg_->height; i++ )
      for(int j = 0; j < seg_->width; j++ )
	{
	  int idx = CV_IMAGE_ELEM( seg_, int, i, j );
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
    cvNamedWindow(name_.c_str(), CV_WINDOW_AUTOSIZE);
    cvShowImage( name_.c_str(), wshed );

    cvWaitKey(0);
    cvReleaseMat(&color_tab);
    cvReleaseImage(&wshed);
    cvReleaseImage(&marker_mask);
  }

  cvReleaseImage(&img_small);
  cvReleaseImage(&seg_small);
}
  
/***************************************************************************
***********  ImageDescriptor::SuperpixelStatistic::SuperpixelColorHistogram
****************************************************************************/

SuperpixelColorHistogram::SuperpixelColorHistogram(int seed_spacing, float scale, int nBins, string type, SuperpixelStatistic* seg_provider, SuperpixelColorHistogram* hsv_provider) :
  SuperpixelStatistic(seed_spacing, scale, seg_provider), nBins_(nBins), type_(type), hsv_provider_(hsv_provider)
{
  char buf[100];
  sprintf(buf, "_colorHistogram_type:%s_nBins%d", type_.c_str(), nBins);
  name_.append(buf);
  cout << "Creating " << name_ << endl;

  result_size_ = nBins_;

  // -- Set up ranges_ for histogram computation.
//   float* range = (float*) malloc(sizeof(float)*2);
//   range[0] = 0;   
//   range[1] = 255;
//   ranges_ = &range;
}

bool SuperpixelColorHistogram::compute(NEWMAT::Matrix** result, bool debug) {
  // -- Make sure we have access to a segmentation and other per-image computation.
  if(seg_provider_ == NULL && seg_ == NULL)
    segment(debug);
  else if(seg_provider_ != NULL && seg_ == NULL) {
    seg_ = seg_provider_->seg_;
  }

  // -- Make sure we have access to HSV image.
  else if(hsv_provider_ != NULL && hsv_ == NULL) {
     hsv_ = hsv_provider_->hsv_;
     hue_ = hsv_provider_->hue_;
     sat_ = hsv_provider_->sat_;
     val_ = hsv_provider_->val_;
  }
  if(hsv_provider_ == NULL && hsv_ == NULL) {
    hsv_ = cvCreateImage( cvGetSize(img_), 8, 3 );
    hue_ = cvCreateImage( cvGetSize(img_), 8, 1 );
    sat_ = cvCreateImage( cvGetSize(img_), 8, 1 );
    val_ = cvCreateImage( cvGetSize(img_), 8, 1 );
    cvCvtColor(img_, hsv_, CV_BGR2HSV);
    cvSplit(hsv_, hue_, sat_, val_, 0);
  }

  // -- Get the label at this point.
  long label = CV_IMAGE_ELEM(seg_, long, row_, col_);


  // -- Choose which channel we wil use.
  IplImage* channel=NULL;
  float max = 255;
  if(type_.compare("hue") == 0) {
    channel = hue_;
    max = 180;
  }
  else if(type_.compare("sat") == 0)
    channel = sat_;
  else if(type_.compare("val") == 0)
    channel = val_;
  else 
    ROS_ERROR("Bad SuperpixelColorHistogram type.");

  // -- Compute the histogram by hand if we need to.
  float max_val=0;
  map<int, Histogram*>::iterator hit = histograms_.find(label);
  if(hit == histograms_.end()) {
    Histogram* h = new Histogram(nBins_, 0, max);
    for(int r=0; r<channel->height; r++) {
      uchar* ptr = (uchar*)(channel->imageData + r * channel->widthStep);
      long* seg_ptr = (long*)(seg_->imageData + r * seg_->widthStep);
      for(int c=0; c<channel->width; c++) {
	//cout << (int)*ptr << " ";
	if(max_val < *ptr) max_val = *ptr;
	if(*seg_ptr == label) {
	  if(!h->insert(*ptr)) {
	    ROS_WARN("Insertion failed for val = %d.", *ptr);
	    cout << "Hello?" << endl;
	  }
	}
	seg_ptr++;
	ptr++;
      }
    }
    h->normalize();
    histograms_[label] = h;
  }
 

  // -- Copy into result.
  Matrix* res = new Matrix(result_size_,1);
  for(unsigned int i=1; i<=result_size_; i++) {
    (*res)(i,1) = histograms_[label]->bins_[i-1];
  }

  // -- Opencv histograms are broken! --
  // -- Create the mask for the histogram.
//   IplImage* mask = cvCreateImage(cvGetSize(img_), IPL_DEPTH_8U, 1);
//   cvZero(mask);
//   for(int r=0; r<mask->height; r++) {
//     uchar* mask_ptr = (uchar*)(mask->imageData + r * mask->widthStep);
//     long* seg_ptr = (long*)(seg_->imageData + r * seg_->widthStep);
//     for(int c=0; c<mask->width; c++) {
//       if(*seg_ptr == label)
// 	*mask_ptr = 255;
//       seg_ptr++;
//       mask_ptr++;
//     }
//   }

//   // -- Compute the histogram.
//   float range[] = {0, 180}; //hsv
//   float* ranges[] = {range, range};
//   int sizes[] = {nBins_, 1};
//   CvHistogram* hist = cvCreateHist(2, sizes, CV_HIST_ARRAY, ranges, 1);
//   cout << "nBins " << nBins_ << "&hue_ " << &hue_ << "hue_ " << hue_ << "hist " << hist << "ranges " << ranges << "mask " << mask << endl;
//   cvCalcHist(&hue_, hist, 0, mask);
//   cvNormalizeHist(hist, 1);
  
//   // -- Put the histogram into the result.
//   for(int i=0; i<nBins_; i++) {
//     cout << cvQueryHistValue_2D(hist, i, 1) << endl;
//   }

  


  if(debug) {
    // -- Show the mask.
//     cvNamedWindow("Mask");
//     cvShowImage("Mask", mask);
    cout << "Max val for " << type_ << " is " << max_val << endl;
    histograms_[label]->print();
    commonDebug();
  }

  return true;
}

//! Release seg_, etc., if this object is the one that computes them; otherwise, just nullify the pointers.
void SuperpixelColorHistogram::clearImageCache() {
  ROS_DEBUG("Clearing the image cache for %s", name_.c_str());
  

  if(hsv_provider_ == NULL) {
    cvReleaseImage(&hsv_);
    cvReleaseImage(&hue_);
    cvReleaseImage(&sat_);
    cvReleaseImage(&val_);
  }
  else {
    hsv_ = NULL;
    hue_ = NULL;
    sat_ = NULL;
    val_ = NULL;
  }

  if(seg_provider_ == NULL)
    cvReleaseImage(&seg_);
  else
    seg_ = NULL;

  map<int, Histogram*>::iterator hit;
  for(hit = histograms_.begin(); hit != histograms_.end(); hit++) {
    delete hit->second;
  }
  histograms_.clear();
}


Histogram::Histogram(int nBins, float min, float max) 
  : nBins_(nBins), min_(min), max_(max)
{
  bin_size_ = (max_ - min_) / (float)nBins_;
  boundaries_.reserve(nBins_+1);

  float b = min_;
  while(b <= max_) {
    boundaries_.push_back(b);
    b += bin_size_;
  } 

  bins_.reserve(nBins_);
  for(int i=0; i<nBins; i++) {
    bins_.push_back(0);
  }

//   cout << "Boundaries: ";
//   for(unsigned int i=0; i<boundaries_.size(); i++) {
//     cout << boundaries_[i] << " ";
//   }
//   cout << endl;
}

//! Temporary, slow version.
bool Histogram::insert(float val) {
  for(unsigned int i=0; i<boundaries_.size() - 1; i++) {
    if (boundaries_[i] <= val && boundaries_[i+1] + .001 >= val) {
      bins_[i]++;
      return true;
    }
  }
  return false;
}

void Histogram::normalize() {
  float total = 0;
  for(unsigned int i=0; i<bins_.size(); i++) {
    total += bins_[i];
  }
  if(total == 0) {
    ROS_WARN("Empty histogram!");
    return;
  }
  for(unsigned int i=0; i<bins_.size(); i++) {
    bins_[i] /= total;
  }
}

void Histogram::print() {
  cout << "Histogram: ";
  for(unsigned int i=0; i<bins_.size(); i++) {
    cout << bins_[i] << " ";
  }
  cout << endl;
}

void Histogram::clear() {
  for(unsigned int i=0; i<bins_.size(); i++) {
    bins_[i] = 0;
  }
}
  
  

      
    
  
  


/***************************************************************************
***********  ImageDescriptor::Patch::EdgePatch
****************************************************************************/

/*

EdgePatch::compute(NEWMAT::Matrix** result, bool debug) {
  Patch::compute(IplImage* img_, row_, col_, result, debug);
  IplImage* gray = cvCreateImage(cvGetSize(img_), IPL_DEPTH_8U, 1);
  IplImage* detail_edge = cvCloneImage(gray);
  cvCvtCol_or(img_, gray, CV_BGR2GRAY);
  cvCanny(gray, detail_edge, thresh1_, thresh2_);
  cvNamedWindow("detail_edge");
  cvShowImage("detail_edge", detail_edge);
  
  cvResize(detail_edge, final_patch_, CV_INTER_AREA);
  cvReleaseImage(&gray);
  cvReleaseImage(&detail_edge);
  
  // -- Convert ipl to Newmat.
  int idx=0;
  for(int r=0; r<final_patch_->height; r++) {
    uchar* ptr = (uchar*)(final_patch_->imageData + r * final_patch_->widthStep);
    for(int c=0; c<final_patch_->width; c++) {
      (*res)(idx+1, 1) = *ptr;
      ptr++;
      idx++;
    }
  }
  result = res;

  // -- Set mean to 0, variance to 1 if appropriate and desired.
  if(whiten_)
    whiten(result);
  
  // -- Display for debugging.
  if(debug) {
    cout << name_ << " dump: ";
    cout << (*result).t() << endl;
      
    IplImage* final_patch_rescaled = cvCreateImage(cvSize(500,500), IPL_DEPTH_8U, 1);
    cvResize(final_patch_, final_patch_rescaled, CV_INTER_NN);
    cvNamedWindow(name_.c_str());
    cvShowImage(name_.c_str(), final_patch_rescaled);
    commonDebug(img_, row_, col_);

    cvReleaseImage(&final_patch_rescaled);
  }
   
  // -- Clean up.
  cvResetImageROI(img_);
  return true;
}

*/


/****************************************************************************
*************  ImageDescriptor::Hog
****************************************************************************/

/*

Hog::Hog(Patch* patch) {
  int blocksz = 16;
  p_ = patch->final_patch_;
  ROS_DEBUG_COND((fp->height % blocksz != 0 || fp->width % blocksz != 0), "Hog patch must be divisible by blocksize.");
  cvHog = HOGDescriptor(cvGetSize(fp), cvSize(blocksz, blocksz), cvSize(8,8), cvSize(8,8), 1);
}

bool Hog::compute(NEWMAT::Matrix** result, bool debug);
*/


