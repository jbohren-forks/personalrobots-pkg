#include "highgui.h"

main( int argc, char** argv )
{
    CvCapture* capture = 0;
    IplImage *frame;
    if( argc==1 ) {
        capture = cvCreateCameraCapture(0);
    } else {
        capture = cvCreateFileCapture( argv[1] );
    }
    if(!capture) return -1;
    cvNamedWindow( "Vid", 0 );
    for(;;){
        if(!(frame = cvQueryFrame( capture )))
            break;
        cvShowImage("Vid", frame );
        if( cvWaitKey(10) >= 0 )
            break;
    }
    cvReleaseCapture( &capture );
    cvDestroyWindow("Vid");
}

