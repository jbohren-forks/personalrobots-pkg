
#include <opencv/cxcore.h>
//#include <opencv/cvwimage.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <stdio.h>

void useChessBoard(IplImage* templ, IplImage* image) {
  int rows = 9;
  int cols = 6;
//  int rows = 6;
//  int cols = 8;
  CvSize pattern_size = cvSize(cols, rows);
  CvPoint2D32f templ_corners[cols*rows]; // we do not need that many right
  CvPoint2D32f image_corners[cols*rows];
  int templ_corner_count=0;
  int image_corner_count=0;
  //int flags=CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS;
  int flags=CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE;
  printf("looking for corner of %d rows by %d cols\n", rows, cols);
  int foundCornerInTempl = cvFindChessboardCorners(templ, pattern_size,
        templ_corners, &templ_corner_count, flags);

  // do subpixel calculation, if corners have been found
  if (foundCornerInTempl && false) {
    cvFindCornerSubPix(templ, templ_corners, templ_corner_count,
        cvSize(5,5),cvSize(-1,-1),
        cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
  }

  int foundCornerInImage = cvFindChessboardCorners(image, pattern_size,
      image_corners, &image_corner_count, flags);
  // do subpixel calculation, if corners have been found
  if (foundCornerInImage && false) {
    cvFindCornerSubPix(image, image_corners, image_corner_count,
        cvSize(5,5),cvSize(-1,-1),
        cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
  }


  if (foundCornerInTempl) {
    printf("succeeded. found %d corners on template.\n", templ_corner_count);
  } else {
    printf("failed. found %d corners on template.\n", templ_corner_count);
  }
  if (foundCornerInImage) {
    printf("succeeded. found %d corners on image.\n", image_corner_count);
  } else {
    printf("failed. found %d corner in image.\n", image_corner_count);
  }

  cvDrawChessboardCorners(templ, pattern_size, templ_corners, templ_corner_count, foundCornerInTempl);
  cvDrawChessboardCorners(image, pattern_size, image_corners, image_corner_count, foundCornerInImage);
}

int main(int argc, char** argv) {
  char* templ_filename = argv[1];
  char* image_filename = argv[2];
  IplImage* templ = cvLoadImage(templ_filename);
  IplImage* image = cvLoadImage(image_filename);
  // making sure it is grayscale
  IplImage* templ0 = cvCreateImage(cvGetSize(templ),IPL_DEPTH_8U,1);
  IplImage* image0 = cvCreateImage(cvGetSize(image),IPL_DEPTH_8U,1);
  cvConvertImage(templ, templ0);
  cvConvertImage(image, image0);

  printf("checking image %s against template %s\n", image_filename, templ_filename);

  useChessBoard(templ0, image0);

  cvNamedWindow("template");
  cvNamedWindow("test image");
  cvShowImage("template",   templ0);
  cvShowImage("test image", image0);
  cvWaitKey(0);
}
