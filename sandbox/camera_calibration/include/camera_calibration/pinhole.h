// license

#ifndef _CAMERA_CALIBRATION_INTRINSICS_H_
#define _CAMERA_CALIBRATION_INTRINSICS_H_

#include <string>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>

namespace camera_calibration {

class PinholeCameraModel
{
public:
  double K[3*3];
  double D[5];

  std::string camera_name;

  // No-initialization constructor
  PinholeCameraModel();
  // Constructor/setter with params fx, fy, ...?

  // getters for focal length, principal point
  double fx() const;
  double fy() const;
  double cx() const;
  double cy() const;
  
  // to CvMat functions
  CvMat K_cv();
  const CvMat K_cv() const;
  CvMat D_cv();
  const CvMat D_cv() const;

  // move principal point, update distortion matrices...
  // should be ROI? stick this in subclass?
  //virtual void moveImageOrigin(int dx, int dy);
  // reset?

  // rescale
  //virtual void setResolution(int new_width, int new_height);

  // undistort (& rectify)
  //virtual void undistort(IplImage* src, IplImage* dst);
  
  // file I/O
  virtual bool load(const std::string& file_name);
  virtual bool save(const std::string& file_name);
  virtual bool parse(const std::string& buffer, const std::string& format = "ini");

protected:
  int image_width_;
  int image_height_;
  int origin_x_, origin_y_;

  bool distorted_;
  cv::WImageBuffer1_f full_undistort_map_x_, full_undistort_map_y_;
  cv::WImageBuffer1_f region_undistort_map_x_, region_undistort_map_y_;
  
};


inline double PinholeCameraModel::fx() const { return K[0]; }
inline double PinholeCameraModel::fy() const { return K[4]; }
inline double PinholeCameraModel::cx() const { return K[2]; }
inline double PinholeCameraModel::cy() const { return K[5]; }

inline CvMat PinholeCameraModel::K_cv()
{
  return cvMat(3, 3, CV_64FC1, K);
}

inline const CvMat PinholeCameraModel::K_cv() const
{
  return cvMat(3, 3, CV_64FC1, const_cast<double*>(K));
}

inline CvMat PinholeCameraModel::D_cv()
{
  return cvMat(1, 5, CV_64FC1, D);
}

inline const CvMat PinholeCameraModel::D_cv() const
{
  return cvMat(1, 5, CV_64FC1, const_cast<double*>(D));
}

} //namespace camera_calibration

#endif
