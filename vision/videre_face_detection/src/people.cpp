/*********************************************************************
* People-specific computer vision algorithms.
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

#include "people.h"


People::People():
  list_(NULL),
  cascade_(NULL),
  storage_(NULL),
  cv_image_gray_(NULL) {
}

People::~People() {

  for (uint i=0; i < list_.size(); i++) {
    cvReleaseHist(&list_[i].face_color_hist_);
    cvReleaseHist(&list_[i].shirt_color_hist_);
    cvReleaseImage(&list_[i].body_mask_);
    cvReleaseImage(&list_[i].face_mask_);
  }

  cvReleaseImage(&cv_image_gray_);
  cvReleaseMemStorage(&storage_);
  cvReleaseHaarClassifierCascade(&cascade_);

}


/********
 * Detect all faces in an image.
 * Input:
 * image - The image in which to detect faces.
 * haar_classifier_filename - Path to the xml file containing the trained haar classifier cascade.
 * threshold - Detection threshold. Currently unused.
 * disparity_image - Image of disparities (from stereo). To avoid using depth information, set this to NULL.
 * cam_model - The camera model created by CvStereoCamModel.
 * do_draw - If true, draw a box on `image' around each face.
 * Output:
 * A vector of CvRects containing the bounding boxes around found faces.
 *********/ 
vector<CvRect> People::detectAllFaces(IplImage *image, const char* haar_classifier_filename, double threshold, IplImage *disparity_image, CvStereoCamModel *cam_model, bool do_draw) {
  vector<CvRect> faces;

  if (!cascade_) {
    cascade_ = (CvHaarClassifierCascade*)cvLoad(haar_classifier_filename);
    if (!cascade_) {
      std::cerr << "Cascade file " << haar_classifier_filename << " doesn't exist.\n" << std::endl;
      return faces;
    }
  }
  if (!storage_) {
    storage_ = cvCreateMemStorage(0);
  }
  CvSize im_size;
  im_size = cvGetSize(image);

  // Convert the image to grayscale, if necessary.
  if (cv_image_gray_==NULL) {
    cv_image_gray_ = cvCreateImage(im_size, 8, 1);
  }
  if (image->nChannels == 1) {
    cvCopy(image, cv_image_gray_, NULL);
  }
  else if (image->nChannels == 3) {
    cvCvtColor(image, cv_image_gray_, CV_BGR2GRAY);
  } 
  else {
    std::cerr << "Unknown image type"<<std::endl;
    return faces;
  }

  // Find the faces using OpenCV's haar cascade object detector.
  CvSeq* face_seq = cvHaarDetectObjects(cv_image_gray_, cascade_, storage_, 1.2, 2, CV_HAAR_DO_CANNY_PRUNING);
 

  // Filter the faces using depth information, if available. Currently checks that the actual face size is within the given limits.
  CvScalar color = cvScalar(0,255,0);
  CvRect one_face;
  double avg_disp = 0.0, real_size;
  int good_pix = 0;
  int r,c;
  bool good_face;
  // For each face...
  for (int iface = 0; iface < face_seq->total; iface++) {

    good_face = true;

    one_face = *(CvRect*)cvGetSeqElem(face_seq, iface);

    // Get the average disparity in the middle half of the bounding box.
    if (disparity_image && cam_model) {
      for (r = floor(one_face.y+one_face.height/4.); r < floor(one_face.y+3.*one_face.height/4.); r++) {
	uchar* ptr = (uchar*)(disparity_image->imageData + r*disparity_image->widthStep);
	for (c = floor(one_face.x+one_face.width/4.); c < floor(one_face.x+3.*one_face.width/4.); c++) {
	  if (ptr[c] > 0) {
	    avg_disp += ptr[c];
	    good_pix++;
	  }
	}
      }
      avg_disp /= (double)good_pix; // Take the average.

      // If the disparity exists but the size is out of bounds, mark it red on the image and don't add it to the output vector.
      if (avg_disp > 0 && cam_model->getZ(avg_disp) < MAX_Z_MM) {
	real_size = cam_model->getDeltaX(one_face.width,avg_disp);
	if (real_size < FACE_SIZE_MIN_MM || real_size > FACE_SIZE_MAX_MM) {
	  color = cvScalar(0,0,255);
	  good_face = false;
	}
      }
      else {
	color = cvScalar(255,0,0);
      }
    }

    // Draw the bounding boxes.
    if (do_draw) {
      cvRectangle(image, cvPoint(one_face.x,one_face.y), cvPoint(one_face.x+one_face.width, one_face.y+one_face.height), color, 4);
      if (disparity_image) {
	cvRectangle(disparity_image, cvPoint(one_face.x,one_face.y), cvPoint(one_face.x+one_face.width, one_face.y+one_face.height), cvScalar(255), 4);
      }
    }

    // Add good faces to the output vector.
    if (good_face) {
      faces.push_back(one_face);
    }
  }
  return faces;
}
