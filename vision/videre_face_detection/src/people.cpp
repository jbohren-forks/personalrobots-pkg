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
 
  CvScalar color = cvScalar(0,255,0);
  CvRect one_face;
  double avg_disp = 0.0, real_size;
  int good_pix = 0;
  int r,c;
  bool good_face;
  for (int iface = 0; iface < face_seq->total; iface++) {

    good_face = true;

    one_face = *(CvRect*)cvGetSeqElem(face_seq, iface);

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
      avg_disp /= (4.0*good_pix); // Disparity is in 1/4 pixel units. Take the average.

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

    if (do_draw) {
      cvRectangle(image, cvPoint(one_face.x,one_face.y), cvPoint(one_face.x+one_face.width, one_face.y+one_face.height), color, 4);
      if (disparity_image) {
	cvRectangle(disparity_image, cvPoint(one_face.x,one_face.y), cvPoint(one_face.x+one_face.width, one_face.y+one_face.height), cvScalar(255), 4);
      }
    }

    if (good_face) {
      faces.push_back(one_face);
    }
  }
  return faces;
}
