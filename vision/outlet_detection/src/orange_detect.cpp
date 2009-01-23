#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cstdio>

using namespace cv;

char wndname[] = "Image";
WImageBuffer3_b image, image_hsv, display;
int W, H;
int hue_min = 11, hue_max = 18;
int sat_min = 180, sat_max = 255;
int val_min = 0, val_max = 255;

void on_trackbar(int)
{
  display.CopyFrom(image);

  if (hue_min <= hue_max && sat_min <= sat_max && val_min <= val_max) {
    for (int i = 0; i < H; ++i) {
      for (int j = 0; j < W; ++j) {
        uchar hue = image_hsv(j, i)[0];
        uchar sat = image_hsv(j, i)[1];
        uchar val = image_hsv(j, i)[2];
        if (hue >= hue_min && hue <= hue_max &&
            sat >= sat_min && sat <= sat_max &&
            val >= val_min && val <= val_max) {
          display(j, i)[0] = 255;
          display(j, i)[1] = 255;
          display(j, i)[2] = 0;
        }
      }
    }
  }
  
  cvShowImage(wndname, display.Ipl());
}

int main(int argc, char** argv)
{
  if (argc < 2)
    printf("Usage: %s image\n", argv[0]);

  image.SetIpl( cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR) );
  W = image.Width();
  H = image.Height();
  image_hsv.Allocate(W, H);
  display.Allocate(W, H);
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
    }
  }

exit_main:
  cvDestroyWindow(wndname);

  return 0;
}
