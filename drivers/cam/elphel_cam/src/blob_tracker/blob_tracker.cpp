///////////////////////////////////////////////////////////////////////////////
// The axis_cam package provides a library that talks to Axis IP-based cameras
// as well as ROS nodes which use these libraries
//
// Copyright (C) 2008, Jimmy Sastra
//                     
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University, Willow Garage, nor the names 
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

#include <sstream>
#include <iostream>
#include "blob_tracker/blob_tracker.h"


IplImage *image = 0, *hsv = 0, *hue = 0, *mask = 0, *backproject = 0, *histimg = 0;
CvHistogram *hist = 0;

int select_object = 0;
int track_object = 0;

CvPoint origin;
CvRect selection;



int vmin = 10, vmax = 256, smin = 30;

// constructor
Blob_Tracker::Blob_Tracker() {}

void on_mouse( int event, int x, int y, int flags, void* param )
{
    if( !image )
        return;

    if( image->origin )
        y = image->height - y;

    if( select_object )
    {
        selection.x = MIN(x,origin.x);
        selection.y = MIN(y,origin.y);
        selection.width = selection.x + CV_IABS(x - origin.x);
        selection.height = selection.y + CV_IABS(y - origin.y);
        
        selection.x = MAX( selection.x, 0 );
        selection.y = MAX( selection.y, 0 );
        selection.width = MIN( selection.width, image->width );
        selection.height = MIN( selection.height, image->height );
        selection.width -= selection.x;
        selection.height -= selection.y;
    }

    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        origin = cvPoint(x,y);
        selection = cvRect(x,y,0,0);
        select_object = 1;
        break;
    case CV_EVENT_LBUTTONUP:
        select_object = 0;
        if( selection.width > 0 && selection.height > 0 )
            track_object = -1;
        break;
    }
  printf("mouse event\n");

}

void Blob_Tracker::init() {
  printf( "Hot keys: \n"
      "\tESC - quit the program\n"
      "\tc - stop the tracking\n"
      "\tb - switch to/from backprojection view\n"
      "\th - show/hide object histogram\n"
      "To initialize tracking, select the object with mouse\n" );
printf("inside init\n");
 
  cvNamedWindow( "Histogram", 1 );
  cvNamedWindow( "CamShiftDemo", 1 );
  cvSetMouseCallback( "CamShiftDemo", on_mouse, 0 );
printf("creating Trackbar\n");
  cvCreateTrackbar( "Vmin", "CamShiftDemo", &vmin, 256, 0 );
  cvCreateTrackbar( "Vmax", "CamShiftDemo", &vmax, 256, 0 );
  cvCreateTrackbar( "Smin", "CamShiftDemo", &smin, 256, 0 );

}

void Blob_Tracker::processFrame(IplImage** cv_image) {
printf("inside processFrame\n");
    cvNamedWindow("blobtracker", CV_WINDOW_AUTOSIZE);
      cvMoveWindow("blobtracker", 100, 100);
printf("showing image\n");
      cvShowImage("blobtracker", *cv_image);
printf("wait key\n");
      cvWaitKey(10);
      cvReleaseImage(cv_image);
}


void Blob_Tracker::foo() {}

