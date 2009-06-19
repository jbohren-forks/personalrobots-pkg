#include "descriptors.h"

using namespace std;
using namespace NEWMAT;

Patch::Patch(int raw_size, string type, float scale) 
  : raw_size_(raw_size), scale_(scale)
{
  char buf[100];
  sprintf(buf, "Patch_sz%d_scale%f_%s", raw_size, scale, type.c_str());
  name_.assign(buf);
  type_ = type;
  cout << "Creating " << name_ << endl;

  size_ = (int) ((float)raw_size * scale);
  if(size_%2==0)
    size_ -= 1;
  
  result_size_ = size_ * size_;
  if(type_.compare("color") == 0) {
    result_size_ *= 3;
  }
}

bool Patch::compute(IplImage* img, int row, int col, Matrix* result, bool debug) {
  Matrix* res = new Matrix(result_size_,1); 


  // -- Catch edge cases.
  int half = ceil((float)raw_size_ / 2.0);
  if(row-half < 0 || row+half >= img->height || col-half < 0 || col+half >= img->width) {
    cout << "Out of bounds; size: " << raw_size_ << ", row: " << row << ", col: " << col << endl;
    return false;
  }
  
  
  cvSetImageROI(img, cvRect(row-half, col-half, raw_size_, raw_size_));
  IplImage* patch_scaled = cvCreateImage(cvSize(size_, size_), img->depth, img->nChannels);
  cvResize(img, patch_scaled);
  

  IplImage* patch_scaled_gray = cvCreateImage(cvSize(size_, size_), IPL_DEPTH_8U, 1);
  if(type_.compare("intensity") == 0) {
    cvCvtColor(patch_scaled, patch_scaled_gray, CV_BGR2GRAY);
  }
  else if(type_.compare("edge") == 0) {
  }
  else {
    cerr << "Bad patch type: " << type_ << endl;
  }


  int idx=0;
  for(int r=0; r<patch_scaled_gray->height; r++) {
    uchar* ptr = (uchar*)(patch_scaled_gray->imageData + r * patch_scaled_gray->widthStep);
    for(int c=0; c<patch_scaled_gray->width; c++) {
      (*res)(idx+1, 1) = *ptr;
      ptr++;
      idx++;
    }
  }
  result = res;
  
  if(debug) {
    //cout << "Debugging for " << name_ << endl; //Why does this produce garbage only in this if statement?
    cout << (*result).t() << endl;
    
    IplImage* patch_rescaled_gray = cvCreateImage(cvSize(500,500), IPL_DEPTH_8U, 1);
    cvResize(patch_scaled_gray, patch_rescaled_gray, CV_INTER_NN);

    IplImage* orig = cvCloneImage(img);
    cvResetImageROI(orig);
    CvFont* numbFont = new CvFont();
    cvInitFont( numbFont, CV_FONT_VECTOR0, 0.4f, 0.4f, 0, 1);
    cvPutText(orig, "+", cvPoint(row,col), numbFont, cvScalar(0,1,0));

    cvNamedWindow("Debug");
    cvNamedWindow("Orig");
    cvShowImage("Debug", patch_rescaled_gray);
    cvShowImage("Orig", orig);
    cvWaitKey(0);
  }
  
    
  cvResetImageROI(img);
  return true;
}
     
     
