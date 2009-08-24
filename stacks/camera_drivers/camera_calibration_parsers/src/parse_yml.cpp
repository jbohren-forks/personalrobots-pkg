#include "camera_calibration_parsers/parse_yml.h"
#include <opencv/cv.h>
#include <ctime>

namespace camera_calibration_parsers {

static const char CAM_YML_NAME[]    = "camera_name";
static const char WIDTH_YML_NAME[]  = "image_width";
static const char HEIGHT_YML_NAME[] = "image_height";
static const char K_YML_NAME[]      = "camera_matrix";
static const char D_YML_NAME[]      = "distortion_coefficients";
static const char R_YML_NAME[]      = "rectification_matrix";
static const char P_YML_NAME[]      = "projection_matrix";

static const double D_default[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
static const double R_default[9] = {1.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0,
                                    0.0, 0.0, 1.0};

// @todo: use yaml-cpp to get rid of opencv dependency
bool writeCalibrationYml(const std::string& file_name, const std::string& camera_name,
                         int width, int height,
                         const double* K, const double* D,
                         const double* R, const double* P)
{
  CvFileStorage* fs = cvOpenFileStorage(file_name.c_str(), 0, CV_STORAGE_WRITE);
  if (!fs)
    return false;

  if (!D) D = D_default;
  if (!R) R = R_default;
  double P_default[12];
  if (!P) {
    for (int i = 0; i < 3; ++i) {
      memcpy(P_default + 4*i, K + 3*i, 3*sizeof(double));
      P_default[4*i + 3] = 0.0;
    }
    P = P_default;
  }
  
  time_t raw_time;
  time( &raw_time );
  cvWriteString(fs, "save_time", asctime( localtime(&raw_time) ));
  cvWriteInt(fs, WIDTH_YML_NAME, width);
  cvWriteInt(fs, HEIGHT_YML_NAME, height);
  
  cvWriteString(fs, CAM_YML_NAME, camera_name.c_str());
  const CvMat K_mat = cvMat(3, 3, CV_64FC1, const_cast<double*>(K));
  cvWrite(fs, K_YML_NAME, &K_mat);
  const CvMat D_mat = cvMat(1, 5, CV_64FC1, const_cast<double*>(D));
  cvWrite(fs, D_YML_NAME, &D_mat);
  const CvMat R_mat = cvMat(3, 3, CV_64FC1, const_cast<double*>(R));
  cvWrite(fs, R_YML_NAME, &R_mat);
  const CvMat P_mat = cvMat(3, 4, CV_64FC1, const_cast<double*>(P));
  cvWrite(fs, P_YML_NAME, &P_mat);

  cvReleaseFileStorage(&fs);
  return true;
}

bool readCalibrationYml(const std::string& file_name, std::string& camera_name,
                        int &width, int &height,
                        double* K, double* D, double* R, double* P)
{
  CvFileStorage* fs = cvOpenFileStorage(file_name.c_str(), 0, CV_STORAGE_READ);
  if (!fs)
    return false;

  const char* name = cvReadStringByName(fs, NULL, CAM_YML_NAME, NULL);
  if (name)
    camera_name = name;

  width = cvReadIntByName(fs, NULL, WIDTH_YML_NAME);
  height = cvReadIntByName(fs, NULL, HEIGHT_YML_NAME);
  if (width == 0 || height == 0)
    return false;

  CvMat* K_mat = (CvMat*)cvReadByName(fs, 0, K_YML_NAME);
  if (!K_mat)
    return false;
  memcpy(K, K_mat->data.db, 9*sizeof(double));

  if (D) {
    CvMat* D_mat = (CvMat*)cvReadByName(fs, 0, D_YML_NAME);
    if (D_mat) {
      memcpy(D, D_mat->data.db, 5*sizeof(double));
      cvReleaseMat(&D_mat);
    } else {
      // Default to zeros
      memcpy(D, D_default, 5*sizeof(double));
    }
  }

  if (R) {
    CvMat* R_mat = (CvMat*)cvReadByName(fs, 0, R_YML_NAME);
    if (R_mat) {
      memcpy(R, R_mat->data.db, 9*sizeof(double));
      cvReleaseMat(&R_mat);
    } else {
      // Default to identity matrix
      memcpy(R, R_default, 9*sizeof(double));
    }
  }

  if (P) {
    CvMat* P_mat = (CvMat*)cvReadByName(fs, 0, P_YML_NAME);
    if (P_mat) {
      memcpy(P, P_mat->data.db, 12*sizeof(double));
      cvReleaseMat(&P_mat);
    } else {
      // Default to K augmented with column of zeros
      for (int i = 0; i < 3; ++i) {
        memcpy(P + 4*i, K + 3*i, 3*sizeof(double));
        P[4*i + 3] = 0.0;
      }
    }
  }
  
  return true;
}

} //namespace camera_calibration_parsers
