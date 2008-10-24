#ifndef PEOPLE_H
#define PEOPLE_H

#include <stdio.h>
#include <iostream>
#include <vector>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>

#include "CvStereoCamModel.h";

#define FACE_SIZE_MIN_MM 100
#define FACE_SIZE_MAX_MM 400
#define MAX_Z_MM 10000

using namespace std;

struct Person {
  CvHistogram *face_color_hist_;
  CvHistogram *shirt_color_hist_;
  double body_height_;
  double body_width_;
  CvRect body_bbox_;
  IplImage *body_mask_;
  double face_size_;
  CvRect face_bbox_;
  IplImage *face_mask_;
};

class People
{
 public:

  People();

  ~People();

  void addPerson(){}

  void removePerson(){}

  void recognizePerson(){}

  vector<CvRect> detectAllFaces(IplImage *image, const char* haar_classifier_filename, double threshold, IplImage *disparity_image, CvStereoCamModel *cam_model, bool do_draw);

  void detectKnownFaces(){}

  void track(){}

 ////////////////////
 private:

  vector<Person> list_;
  CvHaarClassifierCascade *cascade_;
  CvMemStorage *storage_;
  IplImage *cv_image_gray_;


};

#endif
