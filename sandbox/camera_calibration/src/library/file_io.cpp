#include "camera_calibration/file_io.h"
#include <ros/console.h>
#include <opencv/cv.h>
#include <boost/tokenizer.hpp>
#include <boost/shared_ptr.hpp>
#include <cstdio>
#include <ctime>

namespace camera_calibration {

static void print3x3(FILE* file, const double* M, const char* endline = "\n")
{
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      fprintf(file, "%.5f ", M[3*i+j]);
    }
    fprintf(file, endline);
  }
}

bool writeIntrinsicsIni(const std::string& file_name, const std::string& camera_name,
                        int width, int height,
                        const double* K, const double* D,
                        const double* R, const double* P)
{
  FILE* file = fopen(file_name.c_str(), "w");
  if (!file)
    return false;
  boost::shared_ptr<FILE> guard(file, fclose);

  fprintf(file, "# %s camera intrinsics\n\n", camera_name.c_str());
  fprintf(file, "[image]\n\n");
  fprintf(file, "width\n%d\n\n", width);
  fprintf(file, "height\n%d\n\n", height);

  fprintf(file, "[%s]\n\n", camera_name.c_str());
  fprintf(file, "camera matrix\n");
  print3x3(file, K);
  
  fprintf(file, "\ndistortion\n");
  if (D) {
    for (int i = 0; i < 5; ++i)
      fprintf(file, "%.5f ", D[i]);
  } else {
    fprintf(file, "0.00000 0.00000 0.00000 0.00000 0.00000\n");
  }
  
  fprintf(file, "\n\nrectification\n");
  if (R) {
    print3x3(file, R);
  } else {
    fprintf(file, "1.00000 0.00000 0.00000\n");
    fprintf(file, "0.00000 1.00000 0.00000\n");
    fprintf(file, "0.00000 0.00000 1.00000\n");
  }

  fprintf(file, "\nprojection\n");
  if (P) {
    print3x3(file, P, "0.00000\n");
  } else {
    print3x3(file, K, "0.00000\n");
  }

  return true;
}

bool readIntrinsicsIni(const std::string& file_name, const std::string& camera_name,
                       int &width, int &height,
                       double* K, double* D, double* R, double* P)
{
  FILE* file = fopen(file_name.c_str(), "r");
  if (!file)
    return false;
  boost::shared_ptr<FILE> guard(file, fclose);

  fseek(file, 0, SEEK_END);
  long size = ftell(file);
  std::string buffer(size, '\0');
  rewind(file);
  fread(&buffer[0], 1, size, file);

  return parseIntrinsicsIni(buffer, camera_name, width, height, K, D, R, P);
}

bool parseIntrinsicsIni(const std::string& buffer, const std::string& camera_name,
                        int &width, int &height,
                        double* K, double* D, double* R, double* P)
{
  /** @todo: actually parse only what's requested */
  double ignore[12];
  if (!D) D = ignore;
  if (!R) R = ignore;
  if (!P) P = ignore;
  
  // Separate into lines
  typedef boost::tokenizer<boost::char_separator<char> > Tok;
  boost::char_separator<char> sep("\n");
  Tok tok(buffer, sep);

  // Check "header"
  Tok::iterator iter = tok.begin();
  std::string header = "# " + camera_name + " camera intrinsics";
  if (*iter++ != header) {
    ROS_WARN("Camera intrinsics header doesn't match");
    return false;
  }

  // Read calibration matrices
  width = height = 0;
  int items_read = 0;
  static const int EXPECTED_ITEMS = 9 + 5 + 9 + 12;
  for (Tok::iterator ie = tok.end(); iter != ie; ++iter) {
    if (*iter == "width") {
      ++iter;
      width = atoi(iter->c_str());
    }
    else if (*iter == "height") {
      ++iter;
      height = atoi(iter->c_str());
    }
    else if (*iter == "camera matrix")
      for (int i = 0; i < 3; ++i) {
        ++iter;
        items_read += sscanf(iter->c_str(), "%lf %lf %lf",
                             &K[3*i], &K[3*i+1], &K[3*i+2]);
      }
    else if (*iter == "distortion") {
      ++iter;
      items_read += sscanf(iter->c_str(), "%lf %lf %lf %lf %lf",
                           D, D+1, D+2, D+3, D+4);
    }
    else if (*iter == "rectification")
      for (int i = 0; i < 3; ++i) {
        ++iter;
        items_read += sscanf(iter->c_str(), "%lf %lf %lf",
                             &R[3*i], &R[3*i+1], &R[3*i+2]);
      }
    else if (*iter == "projection")
      for (int i = 0; i < 3; ++i) {
        ++iter;
        items_read += sscanf(iter->c_str(), "%lf %lf %lf %lf",
                             &P[4*i], &P[4*i+1], &P[4*i+2], &P[4*i+3]);
      }
  }

  // Check we got everything
  return items_read == EXPECTED_ITEMS && width != 0 && height != 0;
}

static const char K_YML_NAME[] = "camera_matrix";
static const char D_YML_NAME[] = "distortion_coefficients";
static const char R_YML_NAME[] = "rectification_matrix";
static const char P_YML_NAME[] = "projection_matrix";

bool writeIntrinsicsYml(const std::string& file_name, const std::string& camera_name,
                        int width, int height,
                        const double* K, const double* D,
                        const double* R, const double* P)
{
  static const double D_default[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  static const double R_default[9] = {1.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0};
  
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
  
  cvWriteString(fs, "camera_name", camera_name.c_str());
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

bool readIntrinsicsYml(const std::string& file_name, const std::string& camera_name,
                       int &width, int &height,
                       double* K, double* D, double* R, double* P)
{
  CvFileStorage* fs = cvOpenFileStorage(file_name.c_str(), 0, CV_STORAGE_READ);
  if (!fs)
    return false;

  CvMat* K_mat = (CvMat*)cvReadByName(fs, 0, K_YML_NAME);
  memcpy(K, K_mat->data.db, 9*sizeof(double));

  /** @todo: default values, error checking */
  if (D) {
    CvMat* D_mat = (CvMat*)cvReadByName(fs, 0, D_YML_NAME);
    memcpy(D, D_mat->data.db, 5*sizeof(double));
    cvReleaseMat(&D_mat);
  }

  if (R) {
    CvMat* R_mat = (CvMat*)cvReadByName(fs, 0, R_YML_NAME);
    memcpy(R, R_mat->data.db, 9*sizeof(double));
    cvReleaseMat(&R_mat);
  }

  if (P) {
    CvMat* P_mat = (CvMat*)cvReadByName(fs, 0, P_YML_NAME);
    memcpy(P, P_mat->data.db, 12*sizeof(double));
    cvReleaseMat(&P_mat);
  }
  
  return false;
}

} //namespace camera_calibration
