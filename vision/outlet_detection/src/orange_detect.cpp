#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include "BlobResult.h"
#include "nonmax_suppress.h"
#include <vector>
#include <iterator>
#include <cstdio>
#include <cmath>

using namespace cv;

char wndname[] = "Image";
char rect_wndname[] = "Rectified";
WImageBuffer3_b image, image_hsv, display;
WImageBuffer1_b mask;
static const int DIM = 101;
WImageBuffer3_b rectified;
int W, H;
int hue_min = 11, hue_max = 21;
int sat_min = 180, sat_max = 255;
int val_min = 0, val_max = 255;

void find_holes(CBlob *outlet)
{
  typedef NonmaxSuppressWxH<9, 9, uchar, std::greater_equal<uchar> > FindHoles;

  int min_x = CBlobGetMinX()(*outlet) - 10;
  int max_x = CBlobGetMaxX()(*outlet) + 10;
  int min_y = CBlobGetMinY()(*outlet) - 10;
  int max_y = CBlobGetMaxY()(*outlet) + 10;

  WImageView3_b patch_rgb = image.View(min_x, min_y, max_x - min_x, max_y - min_y);
  WImageBuffer1_b patch_g(patch_rgb.Width(), patch_rgb.Height());
  cvCvtColor(patch_rgb.Ipl(), patch_g.Ipl(), CV_BGR2GRAY);

  std::vector<CvPoint> holes;
  int num_holes = FindHoles(100)(patch_g.Ipl(), std::back_inserter(holes), 5);
  printf("\t%d holes found in (%d,%d), (%d,%d)\n", num_holes, min_x, min_y, max_x, max_y);
  for (int i = 0; i < num_holes; ++i) {
    //printf("Hole %d: (%d, %d)\n", i, holes[i].x, holes[i].y);
    cvCircle(display.Ipl(), cvPoint(holes[i].x + min_x, holes[i].y + min_y),
             3, CV_RGB(255,255,255));
  }
  //cvSaveImage("patch.png", patch_g.Ipl());
}

void on_trackbar(int)
{
  static const CvScalar colors[] = {{{255, 255, 0, 0}},
                                    {{255, 0, 255, 0}},
                                    {{0, 255, 255, 0}},
                                    {{255, 0, 0, 0}},
                                    {{0, 255, 0, 0}}};
  static const int num_colors = sizeof(colors) / sizeof(CvScalar);
  
  display.CopyFrom(image);

  if (hue_min <= hue_max && sat_min <= sat_max && val_min <= val_max) {
    // Calculate mask of "orange" pixels
    cvInRangeS(image_hsv.Ipl(), cvScalar(hue_min, sat_min, val_min),
               cvScalar(hue_max, sat_max, val_max), mask.Ipl());
    
    // Find outlet-sized blobs
    CBlobResult blobs(mask.Ipl(), NULL, 100, true);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 150);
    
    // Draw blobs on display image, excepting first (background) blob
    for (int i = 1; i < blobs.GetNumBlobs(); ++i) {
      CBlob *blob = blobs.GetBlob(i);
      //blob->FillBlob(display.Ipl(), colors[(i-1) % num_colors]);
      double orientation = CBlobGetOrientation()(*blob);
      double major = CBlobGetMajorAxisLength()(*blob);
      double minor = CBlobGetMinorAxisLength()(*blob);
      CvPoint center = cvPoint(CBlobGetXCenter()(*blob),
                               CBlobGetYCenter()(*blob));
      cvEllipse(display.Ipl(), center, cvSize(major/2, minor/2),
                -orientation*(180/M_PI), 0, 360, colors[(i-1) % num_colors]);
      printf("Blob %d:\n", i);
      printf("\tCenter = (%d, %d)\n", center.x, center.y);
      printf("\tOrientation = %f radians\n", orientation);
      printf("\tMajor axis length = %f\n", major);
      printf("\tMinor axis length = %f\n", minor);
      // Find and draw holes
      find_holes(blobs.GetBlob(i));
    }
    //cvSet(display.Ipl(), CV_RGB(0, 255, 255), mask.Ipl());
    
    // Rectify and display first blob
    
    static const double E = 1.5;
    CBlob *blob = blobs.GetBlob(1);
    double orientation = CBlobGetOrientation()(*blob);
    double major = CBlobGetMajorAxisLength()(*blob);
    double minor = CBlobGetMinorAxisLength()(*blob);
    CvPoint center = cvPoint(CBlobGetXCenter()(*blob),
                             CBlobGetYCenter()(*blob));
    double sin_o = sin(orientation - M_PI/2);
    double cos_o = cos(orientation - M_PI/2);
    double M = minor * E;
    double m = major * E;
    double d_M = DIM / M;
    double d_m = DIM / m;
    float buffer[6];
    buffer[0] = d_M * cos_o;
    buffer[1] = d_M * sin_o;
    buffer[2] = -d_M * (center.x*cos_o + center.y*sin_o) + DIM/2;
    buffer[3] = -d_m * sin_o;
    buffer[4] = d_m * cos_o;
    buffer[5] = d_m * (center.x*sin_o - center.y*cos_o) + DIM/2;
    printf("%f\t%f\t%f\n%f\t%f\t%f\n", buffer[0], buffer[1], buffer[2],
           buffer[3], buffer[4], buffer[5]);
    CvMat transform = cvMat(2, 3, CV_32F, buffer);
    cvWarpAffine(image.Ipl(), rectified.Ipl(), &transform);
    cvShowImage(rect_wndname, rectified.Ipl());
    
  }
  
  cvShowImage(wndname, display.Ipl());
}

int main(int argc, char** argv)
{
  if (argc < 2) {
    printf("Usage: %s image\n", argv[0]);
    return 0;
  }

  image.SetIpl( cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR) );
  W = image.Width();
  H = image.Height();
  image_hsv.Allocate(W, H);
  display.Allocate(W, H);
  mask.Allocate(W, H);
  rectified.Allocate(DIM, DIM);
  cvCvtColor(image.Ipl(), image_hsv.Ipl(), CV_BGR2HSV);

  cvNamedWindow(wndname);
  cvNamedWindow(rect_wndname);
  cvCreateTrackbar("Hue min", wndname, &hue_min, 180, on_trackbar);
  cvCreateTrackbar("Hue max", wndname, &hue_max, 180, on_trackbar);
  cvCreateTrackbar("Sat min", wndname, &sat_min, 255, on_trackbar);
  cvCreateTrackbar("Sat max", wndname, &sat_max, 255, on_trackbar);
  cvCreateTrackbar("Val min", wndname, &val_min, 255, on_trackbar);
  cvCreateTrackbar("Val max", wndname, &val_max, 255, on_trackbar);

  on_trackbar(0);
  
  for (;;) {
    int c = cvWaitKey(0);
    switch((char)c) {
      case '\x1b':
      case 'q':
        printf("Exiting...\n");
        goto exit_main;
      case 's':
        cvSaveImage("orange.jpg", display.Ipl());
        printf("Saved orange.jpg\n");
      case 'r':
        cvSaveImage("rectified.jpg", rectified.Ipl());
        printf("Saved rectified.jpg\n");
    }
  }

exit_main:
  cvDestroyWindow(wndname);
  cvDestroyWindow(rect_wndname);

  return 0;
}
