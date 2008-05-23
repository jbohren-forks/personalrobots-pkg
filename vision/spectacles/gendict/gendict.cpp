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

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printf("usage: labelview LABELFILE IMAGES\n");
    return 1;
  }
  labeled_imageset imgset;
  imgset.load(argv[1], argc, argv, 2);
  cvNamedWindow(win_name, CV_WINDOW_AUTOSIZE);
  bool done = false;
  vector<IplImage *> positives;
  for (size_t i = 0; !done && i < imgset.images.size(); i++)
  {
    for (size_t l = 0; !done && l < imgset.images[i]->labels.size(); l++)
    {
      labelrect r = imgset.images[i]->labels[l];
      if (r.name != string("mug"))
        continue;
      int diff = fabs(r.h - r.w);
      int width = imgset.images[i]->image->width;
      int height = imgset.images[i]->image->height;
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
      /*
      cvShowImage(win_name, positive);
      if ((char)cvWaitKey() == 27)
        done = true;
        */
      /*
      double minval, maxval;
      cvMinMaxLoc(positive, &minval, &maxval);
      if (minval != maxval)
        cvConvertScale(positive, positive, 255.0 / (maxval - minval), -minval);
        */
      //cvNormalize(positive, positive, 0, 255, CV_MINMAX, NULL);
      positives.push_back(positive);
      //cvReleaseImage(&positive);
    }
  }
  printf("found %d positives\n", positives.size());
  int np = (int)positives.size();
  int tile_cols = (int)ceil(sqrt(np));
  printf("%d tile cols\n", tile_cols);
  IplImage *poster = cvCreateImage(cvSize(tile_cols * 32, tile_cols * 32), 
                                   IPL_DEPTH_8U, 3);
  for (int y = 0; y < tile_cols; y++)
    for (int x = 0; x < tile_cols && y*tile_cols+x < (int)positives.size(); x++)
    {
      cvSetImageROI(poster, cvRect(x*32, y*32, 32, 32));
      cvCopy(positives[y*tile_cols+x], poster);
    }
  
  cvResetImageROI(poster);
  cvShowImage(win_name, poster);
  cvWaitKey();
  cvReleaseImage(&poster);


  for (size_t i = 0; i < positives.size(); i++)
    cvReleaseImage(&positives[i]);
  positives.clear();
  return 0;
}

