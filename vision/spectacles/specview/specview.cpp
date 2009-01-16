#include <string>
#include <cstdio>
#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"

const char *win_name = "specview";

using namespace std;
#if 0
class CvView : public Node
{
public:
  MsgImage image_msg;
  CvBridge<MsgImage> cv_bridge;

  CvView() : Node("cv_view"), cv_bridge(&image_msg)
  { 
    cvNamedWindow("cv_view", CV_WINDOW_AUTOSIZE);
    subscribe("image", image_msg, &CvView::image_cb);
  }
  void image_cb()
  {
    IplImage *cv_image;
    if (cv_bridge.to_cv(&cv_image))
    {
      cvShowImage("cv_view", cv_image);
      cvWaitKey(3);
      cvReleaseImage(&cv_image);
    }
  }
};
#endif
int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("usage: specview IMAGE DEPTHIMAGE\n");
    return 1;
  }

  IplImage *img = cvLoadImage(argv[1]);
  IplImage *depth = cvCreateImage(cvSize(img->width, img->height), 
                                  IPL_DEPTH_32F, 1);
  FILE *f = fopen(argv[2], "r");
  if (!f)
  {
    printf("couldn't open depth map %s\n", argv[2]);
    return 2;
  }
  const int linebuf_len = 64000;
  char linebuf[linebuf_len];
  for (int row = 0; row < img->height; row++)
  {
    if (!fgets(linebuf, linebuf_len, f))
    {
      printf("woah! error reading file\n");
      exit(6);
    }
    if (strlen(linebuf) >= linebuf_len - 2)
    {
      printf("linebuf was too short.\n");
      exit(3);
    }
    char *token = strtok(linebuf, " \n");
    if (!token)
    {
      printf("woah! couldn't parse a token from the line buffer\n");
      exit(4);
    }
    cvSetReal2D(depth, row, 0, atof(token));
    for (int col = 1; col < img->width; col++)
    {
      token = strtok(NULL, " \n");
      if (!token)
      {
        printf("woah! couldn't parse a token from the line buffer\n");
        exit(5);
      }
      double d = atof(token);
      d = (d-.5) * 2;
      if (d > 1) d = 1;
      if (d < 0) d = 0;
      cvSetReal2D(depth, row, col, 1.0 - d);
    }
  }
  fclose(f);
  cvNamedWindow(win_name, CV_WINDOW_AUTOSIZE);
  cvShowImage(win_name, depth);
  while(1)
  {
    char c = cvWaitKey();
    if (c == 27)
      break;
    else if (c == 'd')
      cvShowImage(win_name, depth);
    else if (c == 'i')
      cvShowImage(win_name, img);
  }
  cvReleaseImage(&depth);
  cvReleaseImage(&img);
  return 0;
}

