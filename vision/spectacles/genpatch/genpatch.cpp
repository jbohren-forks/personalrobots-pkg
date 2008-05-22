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

const char *win_name = "genpatch";

using namespace std;
using namespace spectacles;

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printf("usage: labelview LABELFILE IMAGES\n");
    return 1;
  }
  TiXmlDocument labeldoc;
  if (!labeldoc.LoadFile(argv[1]))
  {
    printf("couldn't parse label document\n");
    return 2;
  }
  vector<labeled_image> labels;
  for (TiXmlElement *label_ele = 
         labeldoc.RootElement()->FirstChildElement("Object2dFrame");
       label_ele;
       label_ele = label_ele->NextSiblingElement("Object2dFrame"))
  {
    labeled_image li;
    for (TiXmlElement *obj_ele =
           label_ele->FirstChildElement("Object");
         obj_ele; obj_ele = obj_ele->NextSiblingElement("Object"))
    {
      labelrect r;
      obj_ele->QueryDoubleAttribute("x", &r.x);
      obj_ele->QueryDoubleAttribute("y", &r.y);
      obj_ele->QueryDoubleAttribute("w", &r.w);
      obj_ele->QueryDoubleAttribute("h", &r.h);
      r.name = obj_ele->Attribute("name");
      li.labels.push_back(r);
    }
    labels.push_back(li);
  }

  CvFont font;
  cvInitFont(&font, CV_FONT_VECTOR0, 0.7, 0.8, 0.0, 2.0);
  
  bool done = false;
  for (int i = 2; i < argc && !done; i++)
  {
    IplImage *img = cvLoadImage(argv[i]);
    if (!img)
    {
      printf("couldn't load [%s]\n", argv[i]);
      continue;
    }

    for (size_t l = 0; l < labels[i-2].labels.size(); l++)
    {
      labelrect r = labels[i-2].labels[l];
      cvRectangle(img, cvPoint(r.x, r.y), cvPoint(r.x+r.w, r.y+r.h), 
                  CV_RGB(0,255,0));
      cvPutText(img, r.name.c_str(), cvPoint(r.x, r.y), &font, CV_RGB(0,255,0));
    }

    cvNamedWindow(win_name, CV_WINDOW_AUTOSIZE);
    cvShowImage(win_name, img);

    char c = cvWaitKey();
    if (c == 27)
      done = true;

    cvReleaseImage(&img);
  }

  return 0;
}

