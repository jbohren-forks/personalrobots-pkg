/*********************************************************************
* This class takes OpenCV images and tracks blobs using OpenCV's cvCamShift function.
* Basically, this is mean shift tracking with some scale space search. 
* Right now the tracking is happening for hue features. 
* In the future, the feature computation should be moved outside so that the 
* caller can choose which features to track.
*
**********************************************************************
*
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Caroline Pantofaru
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

#include "blob_tracker.h"

#define VMIN 10 //10
#define VMAX 250 //250
#define SMIN 10 //30
#define SMAX 256 //256

//#define CHI2MAX 1000 // The threshold above which tracking is declared lost.
#define CHI2MAX 750 // The threshold above which tracking is declared lost.
#define BADFRAMESMAX 30 // The number of frames of bad window similarity above which tracking is declared lost.

#define DEBUG 0

static int hbins = 16; // Number of histogram bins for the hue channel.
static float hranges[] = {0,180}; // Range of values for the hue channel.
float *hranges_ptr = hranges;
 
/* !
 * /brief Constructor.
 */
BTracker::BTracker()
{
 
#if 0
  hmin_ = hranges[0];
  hmax_ = hranges[1];
#endif
  feat_image_ptr_= NULL;
  hsv_image_ptr_ = NULL;
  mask_ptr_=NULL;
  backproject_ptr_=NULL;

  num_bad_frames_ = 0;

  // Initialize the current blob.
  blob_.hist = NULL;
  temp_hist_ = NULL;

  instructions(); // Display the instructions at the prompt.

}


/* !
 * /brief Destructor.
 */
BTracker::~BTracker()
{
  cvReleaseHist(&blob_.hist);
  cvReleaseHist(&temp_hist_);

  // Delete images
  cvReleaseImage(&feat_image_ptr_);
  cvReleaseImage(&hsv_image_ptr_);
  cvReleaseImage(&mask_ptr_);
  cvReleaseImage(&backproject_ptr_);
}


/* !
 * \brief Process one frame. Either allows for blob creation or tracks the blobs in the new frame.
 */
bool BTracker::processFrame(IplImage* rec_cv_image_ptr, bool is_new_blob, CvRect old_window, CvRect* new_window, const IplImage* priorProbMap, bool useCamShift) 
{

  new_window->x = old_window.x;
  new_window->y = old_window.y;
  new_window->width = old_window.width;
  new_window->height = old_window.height;

  CvSize l_s = cvGetSize(rec_cv_image_ptr);
  blob_.window = old_window;

  // Allocate the images
  if (hsv_image_ptr_==NULL) {
    hsv_image_ptr_  = cvCreateImage(l_s, IPL_DEPTH_8U, 3);
    feat_image_ptr_ = cvCreateImage(l_s, IPL_DEPTH_8U, 1);
    mask_ptr_ = cvCreateImage(l_s,IPL_DEPTH_8U,1);
    backproject_ptr_ = cvCreateImage(l_s,IPL_DEPTH_8U,1);
  }


  // Create the feature image and the mask of pixels within a reasonable range.
  cvCvtColor( rec_cv_image_ptr, hsv_image_ptr_, CV_BGR2HSV );
  cvSplit( hsv_image_ptr_, feat_image_ptr_, 0, 0, 0);
  cvInRangeS( hsv_image_ptr_, cvScalar(0,SMIN,VMIN,0), cvScalar(180,SMAX,VMAX,0), mask_ptr_);

  // Check that at least some of the pixels in the window are valid in the mask. 
  // If not, ask for a new window.
  cvSetImageROI( mask_ptr_, old_window);
  CvScalar avg = cvAvg( mask_ptr_ );
  cvResetImageROI(mask_ptr_);
  if (avg.val[0] < 0.25) {
    printf("Too many pixels in the selected window are out of range.\n");
    return false;
  }

  // Compute the blob histogram.
  // For now, we're just using a hue histogram. Could use other features in the future.
  float max_val = 0.f;


  if (blob_.hist == NULL) {
    blob_.hist = cvCreateHist(1, &hbins, CV_HIST_ARRAY, &hranges_ptr, 1);
  }

  if (temp_hist_ == NULL) {
    temp_hist_ = cvCreateHist(1, &hbins, CV_HIST_ARRAY, &hranges_ptr, 1);
  }

  // If this is a new blob, recompute the histogram.
#if 0
  if (is_new_blob) {
    cvMinMaxLoc(feat_image_ptr_, &hmin_, &hmax_);
  }
  // scale hue channel
  double scale = 180./(hmax_-hmin_);
  double shift = 180.*hmin_/(hmax_-hmin_);
  printf("cvtscale hue: %f, %f, %f, %f\n", hmin_, hmax_, scale, shift);
  cvConvertScale(feat_image_ptr_, feat_image_ptr_, scale, shift);
#endif

  if (is_new_blob) {
    cvSetImageROI( feat_image_ptr_, old_window );
    cvSetImageROI( mask_ptr_, old_window );
    cvCalcHist( &feat_image_ptr_, blob_.hist, 0, mask_ptr_);

    cvGetMinMaxHistValue( blob_.hist, 0, &max_val, 0, 0);
    // The next step puts everything between 0 and 255, but doesn't normalize the whole histogram. Interesting.
    cvConvertScale( blob_.hist->bins, blob_.hist->bins, max_val ? 255./max_val : 0.0, 0);

    cvResetImageROI( feat_image_ptr_ );
    cvResetImageROI( mask_ptr_ );
  }

  // The actual tracking using CamShift.
  cvCalcBackProject( &feat_image_ptr_, backproject_ptr_, blob_.hist );
  // the mask in cvAnd does not seem to work
  //cvAnd( backproject_ptr_, mask_ptr_, backproject_ptr_, priorProbMap);
  cvAnd( backproject_ptr_, mask_ptr_, backproject_ptr_, 0);
  if (priorProbMap)
    cvAnd( backproject_ptr_, priorProbMap, backproject_ptr_, 0);

  if (useCamShift) {
    cvCamShift( backproject_ptr_, blob_.window, 
		cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1),
		&blob_.comp, &blob_.box );
  } else {
    cvMeanShift( backproject_ptr_, blob_.window, 
		 cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1),
		 &blob_.comp );
  }
  

  // Check if tracking was actually lost because the new window is too different from the old window.
  // Chi2 difference between original and current histograms.
  cvSetImageROI( feat_image_ptr_, blob_.comp.rect );
  cvSetImageROI( mask_ptr_, blob_.comp.rect );
  cvCalcHist( &feat_image_ptr_, temp_hist_, 0, mask_ptr_ );
  cvGetMinMaxHistValue( temp_hist_, 0, &max_val, 0, 0 );
  cvConvertScale( temp_hist_->bins, temp_hist_->bins, max_val ? 255./max_val : 0.0, 0);
  double track_error = 0.0;
  track_error = cvCompareHist(blob_.hist, temp_hist_, CV_COMP_CHISQR);

#if 0
  // Average probability in the new window backprojection. 0.3 is a good threshold.
  cvSetImageROI( backproject_ptr_, blob_.comp.rect );
  CvScalar track_prob_scalar;
  track_prob_scalar = cvAvg(backproject_ptr_, mask_ptr_);
  printf("Average prob in the new window is %g\n", track_prob_scalar.val[0]/255.0);
#endif

  cvResetImageROI( feat_image_ptr_ );
  cvResetImageROI( mask_ptr_ );
  cvResetImageROI( backproject_ptr_ );

  if (track_error > CHI2MAX) {
    printf("Histogram difference is too large at %g\n",track_error);
    num_bad_frames_++;
    if (num_bad_frames_ > BADFRAMESMAX) {
      return false;
    }
  }
  else {
    num_bad_frames_ = 0;
  }

  blob_.window = blob_.comp.rect;
  blob_.center = cvPointFrom32f(blob_.box.center);

  new_window->x = blob_.window.x;
  new_window->y = blob_.window.y;
  new_window->width = blob_.window.width;
  new_window->height = blob_.window.height;

  return true;

}

/* !
 * \brief Display the instructions.
 */
void BTracker::instructions()
{

  printf("To start tracking, either select the object window with the mouse \n"
	 "OR double click to select the object in the center of the image.\n");
  printf("To kill the tracker, type: [Esc], 'q' or 'Q' on the tracker window.\n");

  return;
}


 
