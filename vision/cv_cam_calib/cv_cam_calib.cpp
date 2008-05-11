#include <cstdio>
#include "opencv/cv.h"
#include "opencv/highgui.h"

int main(int argc, char **argv)
{
  if (argc <= 1)
  {
    printf("usage: cv_cam_calib IMG1 IMG2 IMG3 ...\n");
    return 1;
  }
  for (int i = 1; i < 2; i++) //argc; i++)
  {
    IplImage *img = cvLoadImage(argv[i]);
    if (!img)
    {
      printf("couldn't load image\n");
      return 1;
    }
    cvNamedWindow("checkers", CV_WINDOW_AUTOSIZE);
    const int horiz_corners = 6, vert_corners = 7;
    const int num_corners = horiz_corners * vert_corners;
    CvSize pattern_size = cvSize(horiz_corners, vert_corners);
    CvPoint2D32f corners[num_corners];
    int corner_count;
    int ok = cvFindChessboardCorners(img, pattern_size,
                                     corners, &corner_count, 0);
    cvDrawChessboardCorners(img, pattern_size, corners, corner_count, ok);
    cvShowImage("checkers", img);
/*
    if (corner_count == num_corners)
    {
      printf("found all %d corners\n", corner_count);
    }
    else
      printf("couldn't find all corners\n");
*/
    cvWaitKey(0);
    cvReleaseImage(&img);
  }
  return 0;
}

