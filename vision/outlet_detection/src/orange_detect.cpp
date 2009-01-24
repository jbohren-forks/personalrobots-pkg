#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include "BlobResult.h"
#include <cstdio>

using namespace cv;

char wndname[] = "Image";
WImageBuffer3_b image, image_hsv, display;
WImageBuffer1_b mask;
int W, H;
int hue_min = 11, hue_max = 19;
int sat_min = 180, sat_max = 255;
int val_min = 0, val_max = 255;

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
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 200);
    
    // Draw blobs on display image, excepting first (background) blob
    for (int i = 1; i < blobs.GetNumBlobs(); ++i) {
      blobs.GetBlob(i)->FillBlob(display.Ipl(), colors[(i-1) % num_colors]);
    }
    //cvSet(display.Ipl(), CV_RGB(0, 255, 255), mask.Ipl());
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
  cvCvtColor(image.Ipl(), image_hsv.Ipl(), CV_BGR2HSV);

  cvNamedWindow(wndname);
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
        cvSaveImage("orange.png", display.Ipl());
        printf("Saved orange.png\n");
    }
  }

exit_main:
  cvDestroyWindow(wndname);

  return 0;
}
