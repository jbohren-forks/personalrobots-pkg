#include "descriptors.h"

using namespace std;
using namespace NEWMAT;

//***************************************************************************
//***********  Generally Useful Functions
//***************************************************************************


vector<ImageDescriptor*> setupImageDescriptors() {
  vector<ImageDescriptor*> d;
  d.push_back(new IntensityPatch(50, 1, true));
  d.push_back(new PatchStatistic(string("variance"), (Patch*)d.back()));
  d.push_back(new IntensityPatch(40, .5, true));
  d.push_back(new IntensityPatch(20, 1, true));
  d.push_back(new IntensityPatch(80, .25, true));
  d.push_back(new IntensityPatch(120, 1.0/6.0, true));
  return d;
}


void whiten(Matrix* m) {
  float var=0.0;
  float mean = m->Sum() / m->Nrows();
  if(mean == 0.0) //This should only happen if the feature is identically 0.
    return;

  for(int i=1; i<=m->Nrows(); i++) {
    (*m)(i,1) = (*m)(i,1) - mean;
    var += pow((*m)(i,1), 2);
  }
  var /= m->Nrows();

  Matrix div(1,1); div = 1/(sqrt(var));
  *m = KP(*m, div);
}


//***************************************************************************
//***********  ImageDescriptor
//***************************************************************************

void ImageDescriptor::commonDebug(IplImage* img, int row, int col) {
  IplImage* display = cvCloneImage(img);
  cvResetImageROI(display);
  CvFont* numbFont = new CvFont();
  cvInitFont( numbFont, CV_FONT_VECTOR0, 1.0f, 1.0f, 0, 1);
  cvPutText(display, "+", cvPoint(col,row), numbFont, cvScalar(0,0,255));
  cvNamedWindow("Input Image");
  cvShowImage("Input Image", display);
  cvWaitKey(0);
  cvReleaseImage(&display);
}

bool ImageDescriptor::compute(IplImage* img, int row, int col, Matrix** result, bool debug) {
  return false;
}

void ImageDescriptor::display(const NEWMAT::Matrix& result) {
  cout << "Displaying..." << endl;
}


//***************************************************************************
//***********  ImageDescriptor::Patch
//***************************************************************************


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

bool Patch::preCompute(IplImage* img, int row, int col, bool debug) {
  if(debug)
    cout << "Computing " << name_ << endl;
  
  // -- final_patch_ should have been deallocated on clearPointCache(), which must be called by the user.
  if(final_patch_ != NULL || scaled_patch_ != NULL)
    cout << "WARNING: final_patch_  or scaled_patch_ has not been deallocated.  Have you called clearPointCache()?" << endl;

  // -- Catch edge cases.
  int half = ceil((float)raw_size_ / 2.0);
  if(row-half < 0 || row+half >= img->height || col-half < 0 || col+half >= img->width) {
    cout << "Out of bounds; size: " << raw_size_ << ", row: " << row << ", col: " << col << endl;
    return false;
  }
  
  // -- Get the scaled patch.
  //cout << "Setting roi to " << row-half << " " << col-half << " " << raw_size_ << endl;
  cvSetImageROI(img, cvRect(col-half, row-half, raw_size_, raw_size_));
  scaled_patch_ = cvCreateImage(cvSize(size_, size_), img->depth, img->nChannels);
  cvResize(img, scaled_patch_,CV_INTER_AREA);
  
  return true;
}
     
void Patch::clearPointCache() {
  cvReleaseImage(&scaled_patch_);
  cvReleaseImage(&final_patch_);
  final_patch_ = NULL;
  scaled_patch_ = NULL;
}

//void Patch::clearImageCache() {}

//***************************************************************************
//***********  ImageDescriptor::Patch::IntensityPatch
//***************************************************************************

IntensityPatch::IntensityPatch(int raw_size, float scale, bool whiten)
  : Patch(raw_size, scale), whiten_(whiten)
{
  char buf[100];
  sprintf(buf, "_whiten%d_Intensity", whiten_);
  name_.append(buf);
  cout << "Creating " << name_ << endl;

  result_size_ = size_ * size_;
}

bool IntensityPatch::compute(IplImage* img, int row, int col, NEWMAT::Matrix** result, bool debug) {
  //Do common patch processing.  
  if(!preCompute(img, row, col, debug))
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
    commonDebug(img, row, col);

    cvReleaseImage(&final_patch_rescaled);
  }
   
  // -- Clean up.
  cvResetImageROI(img);
  return true;
}

     
//***************************************************************************
//***********  ImageDescriptor::PatchStatistic
//***************************************************************************

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

bool PatchStatistic::compute(IplImage* img, int row, int col, NEWMAT::Matrix** result, bool debug) {

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
    commonDebug(img, row, col);
  }

  return true;
}

void PatchStatistic::display(const NEWMAT::Matrix& result) {
  cout << name_ << " is " << result << endl;
}




/*

//***************************************************************************
//***********  ImageDescriptor::Patch::EdgePatch
//***************************************************************************



EdgePatch::compute(IplImage* img, int row, int col, NEWMAT::Matrix** result, bool debug) {
  Patch::compute(IplImage* img, row, col, result, debug);
  IplImage* gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
  IplImage* detail_edge = cvCloneImage(gray);
  cvCvtColor(img, gray, CV_BGR2GRAY);
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
    commonDebug(img, row, col);

    cvReleaseImage(&final_patch_rescaled);
  }
   
  // -- Clean up.
  cvResetImageROI(img);
  return true;
}




//***************************************************************************
//***********  ImageDescriptor::Hog
//***************************************************************************

Hog::Hog(Patch* patch) {
  int blocksz = 16;
  p_ = patch->final_patch_;
  ROS_DEBUG_COND((fp->height % blocksz != 0 || fp->width % blocksz != 0), "Hog patch must be divisible by blocksize.");
  cvHog = HOGDescriptor(cvGetSize(fp), cvSize(blocksz, blocksz), cvSize(8,8), cvSize(8,8), 1);
}

bool Hog::compute(IplImage* img, int r, int c, NEWMAT::Matrix** result, bool debug);
*/
