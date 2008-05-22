#include <string>
#include <cstdio>
#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace std;

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("usage: cloud2depth CLOUDFILE DEPTHFILE\n");
    return 1;
  }

  FILE *in = fopen(argv[1], "r");
  if (!in)
  {
    printf("couldn't open cloud file for input: %s\n", argv[1]);
    return 2;
  }

  FILE *out = fopen(argv[2], "w");
  if (!out)
  {
    printf("couldn't open cloud file for output: %s\n", argv[2]);
    return 3;
  }

  IplImage *depth = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F, 1);
  IplImage *counts = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32S, 1);
  cvSetZero(depth);
  cvSetZero(counts);

  const int linebuf_len = 1000;
  char linebuf[linebuf_len];
  while (!feof(in))
  {
    if (!fgets(linebuf, linebuf_len, in))
    {
      printf("woah! error reading file\n");
      exit(6);
    }
    if (strlen(linebuf) >= linebuf_len - 2)
    {
      printf("linebuf was too short.\n");
      exit(3);
    }
    double x, y, z, r, g, b, row, col;
    if (8 != fscanf(in, "%lf %lf %lf %lf %lf %lf %lf %lf\n", 
                    &x, &y, &z, &r, &g, &b, &row, &col))
    {
      printf("error parsing line\n");
      exit(8);
    }
    double d = cvGetReal2D(depth, (int)row, (int)col);
    cvSetReal2D(depth, (int)row, (int)col, d + z);
    double c = cvGetReal2D(counts, (int)row, (int)col);
    cvSetReal2D(counts, (int)row, (int)col, round(c) + 1);
  }
  for (int row = 0; row < 480; row++)
  {
    for (int col = 0; col < 640; col++)
    {
      int c = int(round(cvGetReal2D(counts, row, col)));
      double d = 0;
      if (c)
      {
        d = cvGetReal2D(depth, row, col) / c;
        if (d > 3) d = 3;
        else if (d < 0) d = 0;
        cvSetReal2D(depth, row, col, 1.0 - d / 3);
      }
      fprintf(out, "%f ", d);
    }
    fprintf(out, "\n");
  }
  fclose(in);
  fclose(out);
  const char *win_name = "cloud2depth";
  cvNamedWindow(win_name, CV_WINDOW_AUTOSIZE);
  cvShowImage(win_name, depth);
  cvWaitKey();
  cvReleaseImage(&depth);
  return 0;
}

