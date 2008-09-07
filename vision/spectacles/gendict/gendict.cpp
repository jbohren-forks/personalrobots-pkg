///////////////////////////////////////////////////////////////////////////////
// The spectacles package is a playground for vision experiments
//
// Copyright (C) 2008, Morgan Quigley, Stanford Univerity
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names 
//     of its contributors may be used to endorse or promote products derived 
//     from this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#include <string>
#include <cstdio>
#include <vector>
#include <string>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "tinyxml-2.5.3/tinyxml.h"
#include "spectacles/spectacles.h"

const char *win_name = "labelview";

using namespace std;
using namespace spectacles;

CvRect rect_intersect(const CvRect &r1, const CvRect &r2)
{
  CvRect r;
  r.x =  r1.x < r2.x ? r2.x : r1.x;
  r.y =  r1.y < r2.y ? r2.y : r1.y;
  r.w = (r1.x + r1.w < r2.x + r2.w ? r1.x + r1.w : r2.x + r2.w) - r.x;
  r.h = (r1.y + r1.h < r2.y + r2.h ? r1.y + r1.h : r2.y + r2.h) - r.y;
  if (r.w < 0 || r.h < 0)
    r.w = r.h = 0;
  return r;
}

int main(int argc, char **argv)
{
  srand(time(NULL));
  if (argc < 3)
  {
    printf("usage: labelview LABELFILE IMAGES\n");
    return 1;
  }
  labeled_imageset imgset;
  imgset.load(argv[1], argc, argv, 2);
  cvNamedWindow(win_name, CV_WINDOW_AUTOSIZE);
  bool done = false;
  vector<IplImage *> positives, negatives;
  for (size_t i = 0; !done && i < imgset.images.size(); i++)
  {
    int width = imgset.images[i]->image->width;
    int height = imgset.images[i]->image->height;
    for (size_t l = 0; !done && l < imgset.images[i]->labels.size(); l++)
    {
      labelrect r = imgset.images[i]->labels[l];
      if (r.name != string("mug"))
        continue;
      int diff = fabs(r.h - r.w);
      if (r.w > r.h)
      {
        r.y -= diff / 2;
        r.h += diff;
      }
      else if (r.w < r.h)
      {
        r.x -= diff / 2;
        r.w += diff;
      }
      if (r.y < 0 || r.x < 0 || r.y + r.h > height || r.x + r.w > width)
        continue; 
      CvMat mat;
      cvGetSubRect(imgset.images[i]->image, &mat, cvRect(r.x, r.y, r.w, r.h));
      IplImage *positive = cvCreateImage(cvSize(32, 32), IPL_DEPTH_8U, 3);
      cvResize(&mat, positive);
      IplImage *pos_1ch = cvCreateImage(cvSize(32, 32), IPL_DEPTH_8U, 1);
      cvConvertImage(positive, pos_1ch, 0);
      IplImage *pos_norm = cvCreateImage(cvSize(32, 32), IPL_DEPTH_8U, 1);
      cvEqualizeHist(pos_1ch, pos_norm);
      cvReleaseImage(&pos_1ch);
      cvReleaseImage(&positive);
      positives.push_back(pos_norm);
    }
    // now, generate some random negatives
    for (int n = 0; n < 10; n++)
    {
      bool bogus = true;
      do
      {
        CvRect nr(rand() % width, rand() % height, 
                  rand() % 100 + 10, rand() % 100 + 10);
        if (nr.x + nr.w >= width || nr.y + nr.h >= height)
          continue;
          
      } while (bogus);
    }
  }
  printf("found %d positives\n", positives.size());
  int np = (int)positives.size();
  int tile_cols = (int)ceil(sqrt(np));
  printf("%d tile cols\n", tile_cols);
  IplImage *poster = cvCreateImage(cvSize(tile_cols * 32, tile_cols * 32), 
                                   IPL_DEPTH_8U, 1);
  for (int y = 0; y < tile_cols; y++)
    for (int x = 0; x < tile_cols && y*tile_cols+x < (int)positives.size(); x++)
    {
      cvSetImageROI(poster, cvRect(x*32, y*32, 32, 32));
      cvCopy(positives[y*tile_cols+x], poster);
    }
  
  cvResetImageROI(poster);
  cvShowImage(win_name, poster);
  char c;
  while ((c = cvWaitKey()) != 27) ;
  cvReleaseImage(&poster);


  for (size_t i = 0; i < positives.size(); i++)
    cvReleaseImage(&positives[i]);
  positives.clear();
  return 0;
}

