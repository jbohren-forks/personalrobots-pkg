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

#ifndef PEOPLE_H
#define PEOPLE_H

#include <stdio.h>
#include <iostream>
#include <vector>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>

#include "CvStereoCamModel.h"

// Thresholds for the face detection algorithm
#define FACE_SIZE_MIN_MM 100
#define FACE_SIZE_MAX_MM 500
#define MAX_Z_MM 10000

using namespace std;

struct Person {
  CvHistogram *face_color_hist;
  CvHistogram *shirt_color_hist;
  double body_height_3d;
  double body_width_3d;
  CvRect body_bbox_2d;
  IplImage *body_mask_2d;
  double face_size_3d;
  CvRect face_bbox_2d;
  IplImage *face_mask_2d;
  CvScalar face_center_3d;
  int id;
  string name;
};

class People
{
 public:

  // Create an empty list of people.
  People();

  // Destroy a list of people.
  ~People();

  // Add a person to the list of people.
  void addPerson();

  // Return the number of people.
  int getNumPeople();

  // Return the face size in 3D
  double getFaceSize3D(int iperson);

  // Set a person's 3D face size.
  void setFaceSize3D(double face_size, int iperson);

  // Set a person's face.
  void setFaceBbox2D(CvRect face_rect, int iperson );

  // Set a person's position.
  void setFaceCenter3D(double cx, double cy, double cz, int iperson);

  // Takes in a rectangle center and size and outputs the four corners in the order (TL; TR; BL; BR)
  void centerSizeToFourCorners( CvMat *centers, CvMat *sizes, CvMat *four_corners);

  // Remove a person from the list of people.
  void removePerson(){}

  // Use the list of people to recognize a face image.
  void recognizePerson(){}

  // Set a person's id
  void setID(int id, int iperson);

  // Get a person's id
  int getID(int iperson);

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
  vector<CvRect> detectAllFaces(IplImage *image, const char* haar_classifier_filename, double threshold, IplImage *disparity_image, CvStereoCamModel *cam_model, bool do_draw);

  // Detect only known faces in an image.
  void detectKnownFaces(){}

  // Track a face.
  void track(){}

  // Track a face based on the face colour histogram.
  bool track_color_3d_bhattacharya(const IplImage *image, IplImage *disparity_image, CvStereoCamModel *cam_model, int npeople,  int* which_people, CvMat* start_points, CvMat* end_points);

 ////////////////////
 private:


  // The list of people.
  vector<Person> list_;
  // Classifier cascade for face detection.
  CvHaarClassifierCascade *cascade_;
  // Storage for OpenCV functions.
  CvMemStorage *storage_;
  // Grayscale image (to avoid reallocating an image each time an OpenCV function is run.)
  IplImage *cv_image_gray_;

  // Structures for the color face tracker (cft).
  // Color planes and normalized color planes.
  IplImage *cft_r_plane_;
  IplImage *cft_g_plane_;
  IplImage *cft_b_plane_;
  IplImage *cft_r_plane_norm_;
  IplImage *cft_g_plane_norm_;
  CvMat *rbins_;
  CvMat *gbins_;
  // The 3d coords for each point.
  IplImage *cft_X_;
  IplImage *cft_Y_;
  IplImage *cft_Z_;
  CvMat *cft_xyz_;
  CvMat *cft_uvd_;
  CvHistogram *cft_start_hist_;
  CvHistogram *cft_ratio_hist_;


};

#endif

