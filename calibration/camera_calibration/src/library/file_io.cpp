#include "camera_calibration/file_io.h"
#include <ros/console.h>
#include <opencv/cv.h>
#include <boost/shared_ptr.hpp>
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_file_iterator.hpp>
#include <boost/spirit/include/classic_confix.hpp>
#include <boost/spirit/include/classic_loops.hpp>
#include <boost/typeof/typeof.hpp>
#include <cstdio>
#include <ctime>

using namespace BOOST_SPIRIT_CLASSIC_NS;

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

  fprintf(file, "# Camera intrinsics\n\n");
  /** @todo: time? */
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

// Semantic action to store a sequence of values in an array
template <typename T>
struct ArrayAssignActor
{
  ArrayAssignActor(T* start)
    : ptr_(start)
  {}

  void operator()(T val) const
  {
    *ptr_++ = val;
  }

  mutable T* ptr_;
};

// Semantic action generator
template <typename T>
ArrayAssignActor<T> array_assign_a(T* start)
{
  return ArrayAssignActor<T>(start);
}

template <typename Iterator>
bool parseIntrinsicsIniRange(Iterator first, Iterator last,
                             std::string& camera_name, int& width, int& height,
                             double* K, double* D, double* R, double* P)
{
  /** @todo: actually parse only what's requested? */
  double ignore[12];
  if (!D) D = ignore;
  if (!R) R = ignore;
  if (!P) P = ignore;
  
  bool have_externals = false;
  double trans[3], rot[3];

  /** @todo: separate grammar out into separate function */
  
  // Image section (width, height)
  BOOST_AUTO(image,
      str_p("[image]")
      >> "width"
      >> uint_p[assign_a(width)]
      >> "height"
      >> uint_p[assign_a(height)]
     );

  // Optional externals section
  BOOST_AUTO(externals,
      str_p("[externals]")
      >> "translation"
      >> repeat_p(3)[real_p[array_assign_a(trans)]]
      >> "rotation"
      >> repeat_p(3)[real_p[array_assign_a(rot)]]
     );

  // Parser to save name of camera section
  BOOST_AUTO(name, confix_p('[', (*anychar_p)[assign_a(camera_name)], ']'));

  // Camera section (intrinsics)
  BOOST_AUTO(camera,
      name
      >> "camera matrix"
      >> repeat_p(9)[real_p[array_assign_a(K)]]
      >> "distortion"
      >> repeat_p(5)[real_p[array_assign_a(D)]]
      >> "rectification"
      >> repeat_p(9)[real_p[array_assign_a(R)]]
      >> "projection"
      >> repeat_p(12)[real_p[array_assign_a(P)]]
     );

  // Full grammar
  BOOST_AUTO(ini_grammar,
      image
      >> !externals[assign_a(have_externals, true)]
      >>  camera);

  // Skip whitespace and line comments
  BOOST_AUTO(skip, space_p | comment_p('#'));

  parse_info<Iterator> info = parse(first, last, ini_grammar, skip);
  /** @todo: do anything with externals? */
  return info.hit;
}

bool readIntrinsicsIni(const std::string& file_name, std::string& camera_name,
                       int &width, int &height,
                       double* K, double* D, double* R, double* P)
{
  typedef file_iterator<char> Iterator;

  Iterator first(file_name);
  Iterator last = first.make_end();

  return parseIntrinsicsIniRange(first, last, camera_name, width, height, K, D, R, P);
}

bool parseIntrinsicsIni(const std::string& buffer, std::string& camera_name,
                        int &width, int &height,
                        double* K, double* D, double* R, double* P)
{
  return parseIntrinsicsIniRange(buffer.begin(), buffer.end(), camera_name,
                                 width, height, K, D, R, P);
}

static const char CAM_YML_NAME[]    = "camera_name";
static const char WIDTH_YML_NAME[]  = "image_width";
static const char HEIGHT_YML_NAME[] = "image_height";
static const char K_YML_NAME[]      = "camera_matrix";
static const char D_YML_NAME[]      = "distortion_coefficients";
static const char R_YML_NAME[]      = "rectification_matrix";
static const char P_YML_NAME[]      = "projection_matrix";

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

bool readIntrinsicsYml(const std::string& file_name, std::string& camera_name,
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

  /** @todo: default values? */
  if (D) {
    CvMat* D_mat = (CvMat*)cvReadByName(fs, 0, D_YML_NAME);
    if (!D_mat)
      return false;
    memcpy(D, D_mat->data.db, 5*sizeof(double));
    cvReleaseMat(&D_mat);
  }

  if (R) {
    CvMat* R_mat = (CvMat*)cvReadByName(fs, 0, R_YML_NAME);
    if (!R_mat)
      return false;
    memcpy(R, R_mat->data.db, 9*sizeof(double));
    cvReleaseMat(&R_mat);
  }

  if (P) {
    CvMat* P_mat = (CvMat*)cvReadByName(fs, 0, P_YML_NAME);
    if (!P_mat)
      return false;
    memcpy(P, P_mat->data.db, 12*sizeof(double));
    cvReleaseMat(&P_mat);
  }
  
  return true;
}

} //namespace camera_calibration
