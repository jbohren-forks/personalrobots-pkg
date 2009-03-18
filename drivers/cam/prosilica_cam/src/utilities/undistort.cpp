#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <boost/algorithm/string.hpp>
#include <cstdio>
#include <cassert>

int main(int argc, char** argv)
{
  if (argc < 2) {
    printf("Usage: %s intrinsics.yml [FILES]\n", argv[0]);
    return 0;
  }

  // Read camera matrix and distortion from YML file
  CvFileStorage* fs = cvOpenFileStorage(argv[1], 0, CV_STORAGE_READ);
  assert(fs);
  CvMat* K = (CvMat*)cvReadByName(fs, 0, "camera_matrix");
  CvMat* D = (CvMat*)cvReadByName(fs, 0, "distortion_coefficients");
  int width = cvReadIntByName(fs, 0, "image_width");
  int height = cvReadIntByName(fs, 0, "image_height");
  cvReleaseFileStorage(&fs);

  // Set up undistort maps
  cv::WImageBuffer1_f undistortX(width, height), undistortY(width, height);
  cvInitUndistortMap(K, D, undistortX.Ipl(), undistortY.Ipl());

  // Undistort images
  cv::WImageBuffer3_b undistorted(width, height);
  for (int i = 2; i < argc; ++i) {
    cv::WImageBuffer3_b img( cvLoadImage(argv[i]) );
    cvRemap( img.Ipl(), undistorted.Ipl(),
             undistortX.Ipl(), undistortY.Ipl(),
             CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS );
    std::string name(argv[i]);
    boost::replace_last(name, ".", "_rect.");
    if (cvSaveImage(name.c_str(), undistorted.Ipl()))
      printf("Saved %s\n", name.c_str());
  }
  
  return 0;
}
