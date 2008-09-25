//
//  this is testing cvSmooth for CvArr*, but I am using images
//  pass, no corrections
//

#include <stdio.h>
#include <cv.h>
#include <highgui.h>

int main(int argc, char** argv)
{
  // Create a named window with a the name of the file.
  cvNamedWindow( argv[1], 1 );

  // Load the image from the given file name.
  IplImage* src = cvLoadImage( argv[1] );
  IplImage* dst = cvCreateImage( cvGetSize(src), src->depth, src->nChannels);
  cvSmooth( src, dst, CV_GAUSSIAN, 3, 0, 0);

  // Show the image in the named window
  cvShowImage( argv[1], dst );

  // Idle until the user hits the "Esc" key.
  while( 1 ) { if( cvWaitKey( 100 ) == 27 ) break; }

  // Clean up and don’t be piggies
  cvDestroyWindow( argv[1] );
  cvReleaseImage( &src );
  cvReleaseImage( &dst );

}
