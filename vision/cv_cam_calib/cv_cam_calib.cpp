#include <cstdio>
#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace std;

//#define SHOW_IMAGES
//#define SHOW_UNDISTORTED
//#define SAVE_UNDISTORTED

int main(int argc, char **argv)
{
  if (argc <= 1)
  {
    printf("usage: cv_cam_calib IMG1 IMG2 IMG3 ...\n");
    return 1;
  }
  vector< vector<CvPoint2D32f> > img_corners;
  const int horiz = 7, vert = 10;
  const int num_corners = horiz * vert;
  const double square_size = 22.5; // in millimeters
  CvSize img_size;
  for (int i = 1; i < argc; i++)
  {
    IplImage *img = cvLoadImage(argv[i], CV_LOAD_IMAGE_GRAYSCALE);
    if (!img)
    {
      printf("couldn't load image\n");
      return 1;
    }
    img_size = cvSize(img->width, img->height);
#ifdef SHOW_IMAGES
    cvNamedWindow("checkers", CV_WINDOW_AUTOSIZE);
#endif
    CvSize pattern_size = cvSize(horiz, vert);
    CvPoint2D32f corners[num_corners];
    int corner_count;
    int ok = cvFindChessboardCorners(img, pattern_size,
                                     corners, &corner_count, 
                                     CV_CALIB_CB_ADAPTIVE_THRESH | 
                                     CV_CALIB_CB_NORMALIZE_IMAGE | 
                                     CV_CALIB_CB_FILTER_QUADS);
    if (ok)
    {
      cvFindCornerSubPix(img, corners, corner_count, cvSize(10, 10), 
                         cvSize(-1, -1), 
                         cvTermCriteria(CV_TERMCRIT_ITER, 10, 0.1f));
      vector<CvPoint2D32f> c;
      for (int j = 0; j < corner_count; j++)
        c.push_back(corners[j]);
      img_corners.push_back(c);
    }
#ifdef SHOW_IMAGES
    cvDrawChessboardCorners(img, pattern_size, corners, corner_count, ok);
    cvShowImage("checkers", img);
    cvWaitKey(0);
#endif
    cvReleaseImage(&img);
  }
  const int num_img = (int)img_corners.size();
  printf("%d of %d images had all corners detected\n", num_img, argc - 1);
  if (!num_img)
  {
    printf("I need more good images.\n");
    return 1;
  }
  CvMat *img_points = cvCreateMat(num_img * num_corners, 2, CV_32FC1);
  CvMat *obj_points = cvCreateMat(num_img * num_corners, 3, CV_32FC1);
  CvMat *num_points = cvCreateMat(num_img, 1, CV_32SC1);

  for (size_t i = 0; i < img_corners.size(); i++)
  {
    cvSetReal1D(num_points, i, num_corners);
    for (int y = 0; y < vert; y++)
      for (int x = 0; x < horiz; x++)
      {
        cvSetReal2D(img_points, x + y * horiz + i * num_corners, 
              0, img_corners[i][x + y * horiz].x);
        cvSetReal2D(img_points, x + y * horiz + i * num_corners, 
              1, img_corners[i][x + y * horiz].y);
        cvSetReal2D(obj_points, x + y * horiz + i * num_corners, 
              0, square_size * x);
        cvSetReal2D(obj_points, x + y * horiz + i * num_corners, 
              1, square_size * y);
        cvSetReal2D(obj_points, x + y * horiz + i * num_corners, 2, 0);
      }
  }

  CvMat *intrinsics = cvCreateMat(3, 3, CV_32FC1);
  CvMat *distortion = cvCreateMat(4, 1, CV_32FC1);

  cvCalibrateCamera2(obj_points, img_points, num_points, img_size, 
                     intrinsics, distortion);

  printf("%10f %10f %10f\n%10f %10f %10f\n%10f %10f %10f\n",
         cvGet2D(intrinsics, 0, 0).val[0],
         cvGet2D(intrinsics, 0, 1).val[0],
         cvGet2D(intrinsics, 0, 2).val[0],
         cvGet2D(intrinsics, 1, 0).val[0],
         cvGet2D(intrinsics, 1, 1).val[0],
         cvGet2D(intrinsics, 1, 2).val[0],
         cvGet2D(intrinsics, 2, 0).val[0],
         cvGet2D(intrinsics, 2, 1).val[0],
         cvGet2D(intrinsics, 2, 2).val[0]);

  printf("\ndistortion:\n%10f %10f\n%10f %10f\n",
         cvGet2D(distortion, 0, 0).val[0],
         cvGet2D(distortion, 1, 0).val[0],
         cvGet2D(distortion, 2, 0).val[0],
         cvGet2D(distortion, 3, 0).val[0]);

#ifdef SHOW_UNDISTORTED
  for (int i = 1; i < argc; i++) //argc; i++) //argc; i++)
  {
    IplImage *img = cvLoadImage(argv[i]);
    if (!img)
      continue;
    IplImage *undist = cvCreateImage(cvSize(img->width, img->height), 
                                     img->depth, img->nChannels);
    cvUndistort2(img, undist, intrinsics, distortion);
    cvNamedWindow("orig", CV_WINDOW_AUTOSIZE);
    cvShowImage("orig", img);
    cvNamedWindow("undist", CV_WINDOW_AUTOSIZE);
    cvShowImage("undist", undist);
    cvWaitKey(5);
#ifdef SAVE_UNDISTORTED
/*
    IplImage *undist_rgb = cvCreateImage(cvSize(img->width, img->height),
                                         IPL_DEPTH_8U, 3);
    cvCvtColor(undist, undist_rgb, CV_GRAY2BGR);
*/
    char fnbuf[100];
    snprintf(fnbuf, sizeof(fnbuf), "undist%d.ppm", i);
    cvSaveImage(fnbuf, undist);
//    cvReleaseImage(&undist_rgb);
#endif
    cvReleaseImage(&undist);
    cvReleaseImage(&img);
  }
#endif

  cvReleaseMat(&intrinsics);
  cvReleaseMat(&distortion);
  cvReleaseMat(&img_points);
  cvReleaseMat(&obj_points);
  cvReleaseMat(&num_points);
  return 0;
}

