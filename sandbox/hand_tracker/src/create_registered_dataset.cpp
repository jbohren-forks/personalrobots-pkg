/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* This program reads in the bounding boxes output by the Mechanical Turk
 * annotation pipeline, and crops the corresponding patches out of the
 * images, resizing them to a standard size.
 * It also extracts patches of the same size that do not overlap with the
 * selected box, to serve as a negative training set.
 *
 * Romain Thibaux 07/07/09
 * Modified from convert_session_results_to_mask.cpp by Gary Bradski
 */

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>


using namespace std;

void help(char *program) {
  cout << "\n" << program << " <session_results_dir> " << " <widthxheight>" << endl;
  cout << "     session_results_dir      e.g. results/hand\n" << endl;
  cout << "     widthxheight  for example: 32x32\n" << endl;
}

//-----------------------------------------------------------------
//                        SUPPORT FUNCTIONS
//-----------------------------------------------------------------

// List the contents of a directory, except for '.' and '..'
bool getdir (string dir, vector<string> &files) {
  DIR *dp;
  struct dirent *dirp;
  if((dp  = opendir(dir.c_str())) == NULL) {
    cout << "Error opening " << dir << endl;
    return false;
  }

  while ((dirp = readdir(dp)) != NULL) {
    string file_name(dirp->d_name);
    if(file_name.compare(".") == 0 || file_name.compare("..") == 0) {
      continue;
    }
    files.push_back(file_name);
  }
  closedir(dp);
  return true;
}			

// Parse a string with format AxB where A and B are integers
void read_x_by_y(string x_by_y, int &x, int &y) {
  string::size_type xmark = x_by_y.find("x");
  if(xmark == string::npos) {
    cout << "Error in param " << x_by_y << ", widthxheight (example 64x64)" << endl;
    exit(-1);
  }
  x = atoi((x_by_y.substr(0,xmark)).c_str());
  y = atoi((x_by_y.substr(xmark+1, string::npos)).c_str());
}

// Parse a line from a csv file describing rectangles, one per line
bool read_rectangle(ifstream &file, CvRect &rectangle) {
  file.ignore(256, ' ');  // Skip 'xywh##, ' where ## is a number
  file >> rectangle.x;
  file.ignore(256, ' ');  // Skip ', '
  file >> rectangle.y;
  file.ignore(256, ' ');
  file >> rectangle.width;
  file.ignore(256, ' ');
  file >> rectangle.height;
  // The format of this file has a trailing comma and new line, so after reading
  // the last rectangle the status should still be "good" (in particular we have
  // not reached eof.
  return file.good();
}

// Outputs a new rectangle that contains as much of rect as possible and has the
// specified aspect ratio. The new rectangle will fit inside the image size. It
// may not contain the input rectangle entirely if the aspect ratios are very
// different and the size of the new rectangle is limited by the size of the image.
// In other words, we find the rectangle satisfying, in order of priority:
// 1) Fits in the image and has correct aspect ratio (always satisfied)
// 2) Covers rect as much as possible
// 3) Is smallest possible
// 4) Is centered as close to the center of rect as possible
//   rect: the rectangle to return a resized version of
//   image_size: size of the image into which rect will point, and where it has to fit
//   width, height: desired aspect ratio for rect
CvRect adjust_aspect_ratio(CvRect rect, CvSize image_size, int W, int H) {
  // Adjust rectangle aspect ratio
  if (rect.width*H > rect.height*W) {
    // Too wide: increase the height
    CvRect result;
    result.height = (rect.width*H) / W;
    result.y = rect.y - (result.height-rect.height)/2;  // recenter
    result.width = rect.width;
    result.x = rect.x;
    // Push back into the image if it sticks out
    if (result.y < 0) {
      result.y = 0;
    }
    if (result.y + result.height > image_size.height) {
      result.y = image_size.height - result.height;
    }
    if (result.y < 0) {
      // The rectangle still sticks out because it is bigger than the image.
      // Reduce until it fits while keeping the aspect ratio.
      // TODO: write unit test since some cases will rarely be used
      result.y = 0;
      result.height = image_size.height;
      result.width = (result.height*W) / H;
      result.x += (rect.width - result.width)/2;
    }
    return result;
  } else if (rect.width*H < rect.height*W) {
    // Too narrow: increase the width (call with transpose parameters)
    CvRect transpose = adjust_aspect_ratio(cvRect(rect.y, rect.x, rect.height, rect.width),
					   cvSize(image_size.height, image_size.width), H, W);
    return cvRect(transpose.y, transpose.x, transpose.height, transpose.width);
  } else {
    return rect;
  }
}


//-----------------------------------------------------------------
//                            MAIN
//-----------------------------------------------------------------

int main(int argc, char* argv[])
{
  // Command line stuff
  if (argc < 3) { help(argv[0]); return 1; }
  string dir = argv[1];
  string rect_dir = dir + "/CvRects";
  string image_dir = dir + "/images";
  string positive_dir = dir + "/positives";
  string negative_dir = dir + "/negatives";
  int W, H;
  read_x_by_y(argv[2], W, H);
  cout << "Width = " << W << ", height= " << H << endl;

  // FOR EACH image file
  vector<string> image_files;
  getdir(image_dir, image_files);
  for(int i=0; i<(int)image_files.size(); ++i) {
    // Check that we have a file of rectangles for this image
    int sz = image_files[i].size();
    string base_name = image_files[i].substr(0, sz - 4);    
    string rect_file_name = rect_dir + "/" + base_name + "_CvRects.csv"; 
    ifstream rect_file(rect_file_name.c_str());
    if (!rect_file.good()) {
      // This file may not exist if annotations for this image have not been received
      continue;
    }

    // Load image
    string image_file_name = image_dir + "/" + image_files[i];
    IplImage *image = cvLoadImage(image_file_name.c_str());
    CvSize image_size = cvGetSize(image);

    // W-by-H patch we will save to files
    IplImage *output = cvCreateImage(cvSize(W,H), image->depth, image->nChannels);

    // FOR EACH rectangle 
    CvRect rectangle;
    int nRectangles = 0;
    while (read_rectangle(rect_file, rectangle)) {
      // Adjust rectangle aspect ratio
      rectangle = adjust_aspect_ratio(rectangle, image_size, W, H);
      
      // Extract selected patch
      cvSetImageROI(image, rectangle);
      IplImage *patch = cvCreateImage(cvSize(rectangle.width, rectangle.height), image->depth, image->nChannels);
      cvCopy(image, patch, NULL);
      cvResetImageROI(image);  // Not really needed, but safer
      
      // Resize patch
      cvResize(patch, output);

      // Save output
      nRectangles++;
      stringstream output_file_name;
      output_file_name << positive_dir << "/" << base_name << "_" << nRectangles << ".png";
      cvSaveImage(output_file_name.str().c_str(), output);

      // Collect negative examples
      //for (int j=1; j<3; j++) {
      //int x = rand() % (rectangle.width
      //}
      cvReleaseImage(&patch);	
    }
    cvReleaseImage(&output);
    cvReleaseImage(&image);

    cout << nRectangles << " positive example(s) from " << base_name << endl;
  }
  
  cout << "DONE" << endl;  	
  return 0;
}



