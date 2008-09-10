/*
 * CvPathReconDisplay.cpp
 *
 *  Created on: Sep 8, 2008
 *      Author: jdchen
 */

#include "CvPathReconDisplay.h"

CvPathReconDisplay::CvPathReconDisplay() {
#if 0
  // TODO Auto-generated constructor stub
  // create a list of windows to display results
  cvNamedWindow(poseEstWinName.c_str(), CV_WINDOW_AUTOSIZE);
  cvNamedWindow(leftCamWinName.c_str(), CV_WINDOW_AUTOSIZE);
  cvNamedWindow(dispWindowName.c_str(), CV_WINDOW_AUTOSIZE);
  cvNamedWindow(lastTrackedLeftCam.c_str(), CV_WINDOW_AUTOSIZE);

  cvMoveWindow(poseEstWinName.c_str(), 0, 0);
  cvMoveWindow(leftCamWinName.c_str(), 650, 0);
  cvMoveWindow(dispWindowName.c_str(), 650, 530);
  cvMoveWindow(lastTrackedLeftCam.c_str(), 0, 530);

  cvSetMouseCallback(leftCamWinName.c_str(), MyMouseCallback, (void*)this);

  // input and output directory
  string dirname       = "Data/indoor1";
  string outputDirname = "Output/indoor1";

  // prepare for the text in the pose estimate window
  char info[256];
  CvPoint org = cvPoint(0, 475);
  CvFont font;
  cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX, .5, .4);

#endif
}

CvPathReconDisplay::~CvPathReconDisplay() {
  // TODO Auto-generated destructor stub
}
