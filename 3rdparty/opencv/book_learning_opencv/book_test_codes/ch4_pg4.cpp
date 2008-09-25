#include <cv.h>
#include <highgui.h>
int main(int argc, char** argv)
{

  // Create a named window with a the name of the file.

  cvNamedWindow( argv[1], 1 );

  // Load the image from the given file name.
  IplImage* img = cvLoadImage( argv[1] );

  // Show the image in the named window
  cvShowImage( argv[1], img );

  // Idle until the user hits the "Esc" key.
  while( 1 ) { if( cvWaitKey( 100 ) == 27 ) break; }

  // Clean up and don’t be piggies
  cvDestroyWindow( argv[1] );
  cvReleaseImage( &img );

}
