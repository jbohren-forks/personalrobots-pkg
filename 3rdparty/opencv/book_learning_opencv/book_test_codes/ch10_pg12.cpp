//
//  added includes
//  added:   #define CVX_GRAY50 100
//  added:   #define CVX_WHITE  255
//  removed: CvxMultiWin mw;
//  removed: mw.initialize( "OpticalFlow", imgC, 3, 1, 0 );
//  removed: mw.paint( imgA );
//  removed: mw.paint( imgB );
//  removed: mw.paint( imgC );
//  removed: mw.redraw();
//  added:    cvNamedWindow( "A" );
//  added:    cvNamedWindow( "B" );
//  added:    cvNamedWindow( "C" );
//  added:    cvShowImage( "A",imgA );
//  added:    cvShowImage( "B",imgB );
//  added:    cvShowImage( "C",imgC );
//  mod:      IplImage* imgA = cvLoadImage(argv[1],0);
//  mod:      IplImage* imgB = cvLoadImage(argv[2],0);
//  added:    cvDestroydWindow( "A" );
//  added:    cvDestroydWindow( "B" );
//  added:    cvDestroydWindow( "C" );
//  added:    cvReleaseImage( &imgA );
//  added:    cvReleaseImage( &imgB );
//  added:    cvReleaseImage( &imgC );
//

#include "cv.h"
#include "highgui.h"
#include <math.h>
#define CVX_GRAY50 cvScalar(100)
#define CVX_WHITE  cvScalar(255)

int main(int argc, char** argv)
{
    // Initialize, load two images from the file system, and
    // allocate the images and other structures we will need for
    // results.

    // exit if no input images
    if (argc < 3)
      return -1;

    IplImage* imgA = cvLoadImage(argv[1],0);
    IplImage* imgB = cvLoadImage(argv[2],0);

    IplImage* velx = cvCreateImage(cvGetSize(imgA),IPL_DEPTH_32F,1);
    IplImage* vely = cvCreateImage(cvGetSize(imgA),IPL_DEPTH_32F,1);

    IplImage* imgC = cvCreateImage(cvGetSize(imgA),IPL_DEPTH_8U,1);

    cvNamedWindow( "A" );
    cvNamedWindow( "B" );
    cvNamedWindow( "C" );

    cvShowImage( "A",imgA );
    cvShowImage( "B",imgB );

    // Call the actual Horn and Schunck algorithm
    //
    cvCalcOpticalFlowHS( 
        imgA, 
        imgB, 
        0,
        velx,
        vely,
        .10,
        cvTermCriteria( 
            CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,
            imgA->width,
            1e-6
        )
    );

    // Now make some image of what we are looking at:
    //
    cvZero( imgC );
    int step = 4;
    for( int y=0; y<imgC->height; y += step ) {
        float* px = (float*) ( velx->imageData + y * velx->widthStep );
        float* py = (float*) ( vely->imageData + y * vely->widthStep );
        for( int x=0; x<imgC->width; x += step ) {
            if( px[x]>1 && py[x]>1 ) {
                cvCircle(
                    imgC,
                    cvPoint( x, y ),
                    2,
                    CVX_GRAY50,
                    -1
                );
                cvLine(
                    imgC,
                    cvPoint( x, y ),
                    cvPoint( x+px[x]/2, y+py[x]/2 ),
                    CVX_WHITE,
                    1,
                    0
                );
            }
        }
    }
    // show tracking
    cvShowImage( "C",imgC );
    
    cvWaitKey(0);

    // destroy windows
    cvDestroyWindow( "A" );
    cvDestroyWindow( "B" );
    cvDestroyWindow( "C" );
    // release memory
    cvReleaseImage( &imgA );
    cvReleaseImage( &imgB );
    cvReleaseImage( &imgC );

    return 0;
}
