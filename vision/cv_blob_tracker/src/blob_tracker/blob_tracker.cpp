///////////////////////////////////////////////////////////////////////////////
// This package provides a library to take OpenCV images and track any number of blobs.
// It follows the camshiftdemo.c from the OpenCV samples.
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
#include <string.h>
#include <cstdlib>



IplImage *image = 0, *hsv = 0, *hue = 0, *mask = 0, *backproject = 0, *histimg = 0;
CvHistogram *hist = 0;

// Different modes:
int backproject_mode = 0;
int squares_mode = 0;
int select_object = 0;
int track_object = 0;
int show_hist = 1;
bool homogIsSet = false;
bool printPropsToCMD = false;


CvPoint origin;
CvRect selection;
CvRect track_window;
CvBox2D track_box;
CvConnectedComp track_comp;
int hdims = 16;
float hranges_arr[] = {0,180};
float* hranges = hranges_arr;
int vmin = 10, vmax = 256, smin = 30;

const int nChessPointsWidth = 3;
const int nChessPointsHeight = 5;
CvPoint2D32f corners[nChessPointsWidth*nChessPointsHeight];
CvMat* M_homog_src = cvCreateMat(2,nChessPointsWidth*nChessPointsHeight, CV_64FC1);
CvMat* M_homog_dest = cvCreateMat(2,nChessPointsWidth*nChessPointsHeight, CV_64FC1);
CvMat* M_homog = cvCreateMat(3,3,CV_64FC1);



int pattern_was_found;

struct Blob {
  float x, y;
  CvHistogram *histBlob;
  CvBox2D track_boxBlob;
  CvConnectedComp track_compBlob;
  CvRect track_windowBlob;
};

vector<Blob> blobs;
Blob property;

// constructor
Blob_Tracker::Blob_Tracker() {}


void create_M_homog_dest(const int nCornersWidth, const int nCornersHeight, CvMat* Ma) {
  for(int i=0; i < nCornersWidth*nCornersHeight; i++) {
    cvmSet(Ma,0,i,i%nCornersWidth);
    cvmSet(Ma,1,i,(int)(i/nCornersWidth));
  }

}

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

}

CvScalar hsv2rgb( float hue )
{
    int rgb[3], p, sector;
    static const int sector_data[][3]=
        {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
    hue *= 0.033333333333333333333333333333333f;
    sector = cvFloor(hue);
    p = cvRound(255*(hue - sector));
    p ^= sector & 1 ? 255 : 0;

    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;

    return cvScalar(rgb[2], rgb[1], rgb[0],0);
}

void Blob_Tracker::init() {
  printf( "Hot keys: \n"
      "\tESC - quit the program\n"
      "\tc - stop the tracking\n"
      "\tb - switch to/from backprojection view\n"
      "\th - show/hide object histogram\n"
      "To initialize tracking, select the object with mouse\n" );
 
  cvNamedWindow( "Histogram", 1 );
  cvNamedWindow( "BlobTracker", 1 );
  cvSetMouseCallback( "BlobTracker", on_mouse, 0 );
  cvCreateTrackbar( "Vmin", "BlobTracker", &vmin, 256, 0 );
  cvCreateTrackbar( "Vmax", "BlobTracker", &vmax, 256, 0 );
  cvCreateTrackbar( "Smin", "BlobTracker", &smin, 256, 0 );

  create_M_homog_dest(nChessPointsWidth,nChessPointsHeight,M_homog_dest); 
  

}

bool Blob_Tracker::processFrame(IplImage** cv_image) {
  IplImage* frame = 0;
  int i, bin_w, c;
  frame = *cv_image;
  if( !frame )
    return false;

  if( !image )

  {
    printf("!image\n");
      /* allocate all the buffers */
      image = cvCreateImage( cvGetSize(frame), 8, 3 );
      image->origin = frame->origin;
      hsv = cvCreateImage( cvGetSize(frame), 8, 3 );
      hue = cvCreateImage( cvGetSize(frame), 8, 1 );
      mask = cvCreateImage( cvGetSize(frame), 8, 1 );
      backproject = cvCreateImage( cvGetSize(frame), 8, 1 );
      //hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
      histimg = cvCreateImage( cvSize(320,200), 8, 3 );
      cvZero( histimg );
  }

  cvCopy( frame, image, 0 );
  cvCvtColor( image, hsv, CV_BGR2HSV );

  if( track_object )
  {
      int _vmin = vmin, _vmax = vmax;

      cvInRangeS( hsv, cvScalar(0,smin,MIN(_vmin,_vmax),0),
                  cvScalar(180,256,MAX(_vmin,_vmax),0), mask );
      cvSplit( hsv, hue, 0, 0, 0 );

      if( track_object < 0 ) // first time round calc hist
      {
	
	blobs.push_back(property);
	blobs.back().histBlob = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1);

          float max_val = 0.f;
          cvSetImageROI( hue, selection );
          cvSetImageROI( mask, selection );
          cvCalcHist( &hue, blobs.back().histBlob, 0, mask );
          cvGetMinMaxHistValue( blobs.back().histBlob, 0, &max_val, 0, 0 );
          cvConvertScale( blobs.back().histBlob->bins, blobs.back().histBlob->bins, max_val ? 255. / max_val : 0., 0 );
          cvResetImageROI( hue );
          cvResetImageROI( mask );
          blobs.back().track_windowBlob = selection;
          track_object = 1;

          cvZero( histimg );
          bin_w = histimg->width / hdims;
          for( i = 0; i < hdims; i++ )
          {
	    int val = cvRound( cvGetReal1D(blobs.back().histBlob->bins,i)*histimg->height/255 );
              CvScalar color = hsv2rgb(i*180.f/hdims);
              cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
                           cvPoint((i+1)*bin_w,histimg->height - val),
                           color, -1, 8, 0 );
          }
      }

      for(vector<Blob>::size_type i = 0; i != blobs.size(); ++i) {
	
      
	cvCalcBackProject( &hue, backproject, blobs[i].histBlob );

	cvAnd( backproject, mask, backproject, 0 );
	cvCamShift( backproject, blobs[i].track_windowBlob,
		    cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
		    &blobs[i].track_compBlob, &blobs[i].track_boxBlob );
	blobs[i].track_windowBlob = blobs[i].track_compBlob.rect;
	



	if(homogIsSet) {
	  CvMat *p = cvCreateMat(3,1,CV_64FC1);
	  CvMat *r = cvCreateMat(3,1,CV_64FC1);
	  cvmSet(p,0,0,blobs[i].track_boxBlob.center.x);
	  cvmSet(p,1,0,blobs[i].track_boxBlob.center.y);
	  cvmSet(p,2,0,1);
	  cvMatMul(M_homog,p,r);
	  blobs[i].x = cvmGet(r,0,0);
	  blobs[i].y = -cvmGet(r,1,0); // want to use farther away as y positive 
	}
	else {
            blobs[i].x =  blobs[i].track_boxBlob.center.x;
	    blobs[i].y =  blobs[i].track_boxBlob.center.y;
	}
	if( backproject_mode )
          cvCvtColor( backproject, image, CV_GRAY2BGR );

	if( !image->origin )
	  blobs[i].track_boxBlob.angle = -blobs[i].track_boxBlob.angle;
	cvEllipseBox( image, blobs[i].track_boxBlob, CV_RGB(255,0,0), 3, CV_AA, 0 );
      }
  }
  


  if( select_object && selection.width > 0 && selection.height > 0 )
  {
      cvSetImageROI( image, selection );
      cvXorS( image, cvScalarAll(255), image, 0 );
      cvResetImageROI( image );
  }

  if( squares_mode) { 
    CvSize pattern_size = cvSize(nChessPointsWidth,nChessPointsHeight);
    int corner_count = 0;
    pattern_was_found = cvFindChessboardCorners(image, pattern_size, corners, &corner_count, 0);
    cvDrawChessboardCorners(image, pattern_size, corners, corner_count, pattern_was_found);
  }
  
  if( printPropsToCMD ) {
    for(vector<Blob>::size_type i = 0; i != blobs.size(); ++i) {
      printf("i: %i, x: %f, y: %f\n", i, blobs[i].x, blobs[i].y);
    }
  }
  cvShowImage( "BlobTracker", image );
  cvShowImage( "Histogram", histimg );

  c = cvWaitKey(10);
  if( (char) c == 27 )
      return false;
  switch( (char) c )
  {
  case 'p': //print blob properties to CMD
    printPropsToCMD = true;
    break;
  case 'r':
    if(!blobs.empty())
      blobs.pop_back(); // remove blobs
    printf("blob size: %i\n", blobs.size());
    break;
   case 'b':
      backproject_mode ^= 1;
      break;
  case 'c':
      track_object = 0;
      cvZero( histimg );
      break;
  case 'h':
      show_hist ^= 1;
      if( !show_hist )
          cvDestroyWindow( "Histogram" );
      else
          cvNamedWindow( "Histogram", 1 );
      break;
  case 's':
    squares_mode ^= 1;
    break;
  case 'f':
    if(squares_mode && (pattern_was_found != 0)) {
	printf("Corners:\n");
      for(int i = 0; i < nChessPointsWidth*nChessPointsHeight; i++) {
	cvmSet(M_homog_src,0,i,corners[i].x);
	cvmSet(M_homog_src,1,i,corners[i].y);
      }

#ifdef DEBUG
#define  
     printf("src: \n");
     for(int i = 0; i < nChessPointsWidth*nChessPointsHeight; i++)
       printf("[ %f %f]\n", cvmGet(M_homog_src,0,i),  cvmGet(M_homog_src,1,i));

    
     printf("dest: \n");
     for(int i = 0; i < nChessPointsWidth*nChessPointsHeight; i++)
       printf("[ %f %f]\n", cvmGet(M_homog_dest,0,i),  cvmGet(M_homog_dest,1,i));
#endif
      cvFindHomography(M_homog_src,
                       M_homog_dest,
                       M_homog);
      homogIsSet = true;
      
    }
    break;
  case 'q':
    return false;
    break;
  default:

    break;
    
    
  }
  return true;
}

void Blob_Tracker::showFrame(IplImage** cv_image) {
  cvNamedWindow("BlobTracker", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("BlobTracker", 100, 100);
  cvShowImage("BlobTracker", *cv_image);
  cvWaitKey(10);
  cvReleaseImage(cv_image);
}


void Blob_Tracker::saveFrame(const char* fileName, IplImage* cv_image) {
        std::stringstream ss;
	static int iFrame;
        ss << "test" << iFrame++ << ".png";    
	cvSaveImage(ss.str().c_str(), cv_image);
}


int Blob_Tracker::getNumBlobs(){
  return blobs.size();
}

void Blob_Tracker::getBlobCoords(int i, float* x, float* y) {
  *x = blobs[i].x;
  *y = blobs[i].y;
}
