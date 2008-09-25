//
//  pg 37, testing function sum_rgb,
//
//  changed ipl fo Ipl for r,g,b
//  cvSplit( img, r, g, b ); --> img to src
// void sum_rgb( IplImage* src, IplImage dst ) { --> void sum_rgb( IplImage* src, IplImage* dst ) {
//  cvSplit( src, r, g, b ); --> need more inputs add NULL
//  dst should have single channel so cvThreshold works
//
//  AND A BUNCH OF 8U -> 16U CONVERSION CHANGES!
//
//  add "&0x7f" to cvWaitKey:  while( 1 ) { if( (cvWaitKey( 10 )&0x7f) == 27 ) break; }
//  FIXME:  should do this to all cvWaitKey.
//

#include <stdio.h>
#include <cv.h>
#include <highgui.h>


void sum_rgb( IplImage* src, IplImage* dst ) {
  // Allocate individual image planes
  IplImage* r8 = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1 );
  IplImage* g8 = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1 );
  IplImage* b8 = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1 );
  IplImage* s8 = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1 );

  // Use IPL_DEPTH_16S so we do not overflow pixels.
  IplImage* r = cvCreateImage( cvGetSize(src), IPL_DEPTH_16U, 1 );
  IplImage* g = cvCreateImage( cvGetSize(src), IPL_DEPTH_16U, 1 );
  IplImage* b = cvCreateImage( cvGetSize(src), IPL_DEPTH_16U, 1 );
  IplImage* s = cvCreateImage( cvGetSize(src), IPL_DEPTH_16U, 1 );

  // Split image onto the color planes   
  cvSplit( src, r8, g8, b8, NULL );

  // Convert image scale to 16U
  cvConvertScale( r8, r, 1, 0 );
  cvConvertScale( g8, g, 1, 0 );
  cvConvertScale( b8, b, 1, 0 );
     
  // Add equally weighted rgb values
  cvAddWeighted( r, 1./3., g, 1./3., 0.0, s );
  cvAddWeighted( s, 2./3., b, 1./3., 0.0, s );

  // convert image scale back to 8U
  cvConvertScale( s, s8 , 1, 0 );

  // Make a binary image with threshold value at 50
  cvThreshold( s8, dst, 50, 255, CV_THRESH_BINARY );

  cvReleaseImage( &r8 );
  cvReleaseImage( &g8 );   
  cvReleaseImage( &b8 );   
  cvReleaseImage( &s8 );
  cvReleaseImage( &r );   
  cvReleaseImage( &g );   
  cvReleaseImage( &b );   
  cvReleaseImage( &s );
}

int main(int argc, char** argv)
{

  // Create a named window with a the name of the file.
  cvNamedWindow( argv[1], 1 );

  // Load the image from the given file name.
  IplImage* src = cvLoadImage( argv[1] );
  IplImage* dst = cvCreateImage( cvGetSize(src), src->depth, 1);
  sum_rgb( src, dst);

  // Show the image in the named window
  cvShowImage( argv[1], dst );

  // Idle until the user hits the "Esc" key.
  while( 1 ) { if( (cvWaitKey( 10 )&0x7f) == 27 ) break; }

  // Clean up and don’t be piggies
  cvDestroyWindow( argv[1] );
  cvReleaseImage( &src );
  cvReleaseImage( &dst );

}
