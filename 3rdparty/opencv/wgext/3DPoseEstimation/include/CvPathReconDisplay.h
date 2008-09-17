/*
 * CvPathReconDisplay.h
 *
 *  Created on: Sep 8, 2008
 *      Author: jdchen
 */

#ifndef CVPATHRECONDISPLAY_H_
#define CVPATHRECONDISPLAY_H_

/**
 * A wrapper class to perform path reconstruction and display images
 * a long the way (Under reconstruction).
 */
class CvPathReconDisplay {
public:
  CvPathReconDisplay();
  virtual ~CvPathReconDisplay();
#if 0
  // filenames for saving debugging/analysis images
  char leftCamWithMarks[PATH_MAX];
  char rightCamWithMarks[PATH_MAX];
  char poseEstFilename[PATH_MAX];

  string poseEstWinName("Pose Estimated");
  string leftCamWinName("Left  Cam");
  string lastTrackedLeftCam("Last Tracked Left Cam");
  string dispWindowName("Disparity Map");
#endif
};

#endif /* CVPATHRECONDISPLAY_H_ */
