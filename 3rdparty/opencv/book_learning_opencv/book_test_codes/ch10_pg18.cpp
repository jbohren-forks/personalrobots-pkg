//
//  added:     #define CVX_RED 100
//  mod:       Int           corner_count = MAX_CORNERS;  --> int
//  added:     CvPoint2D32f* corners      = new CvPoint2D32f[ MAX_CORNERS ];
//  mod:     cvGoodFeaturesToTrack( 
//  mod:         imgA, 
//  mod:         eig_image,
//  mod:         tmp_image,
//  mod:         corners, 
//  mod:         corner_count, --> &corner_count,   <--  missing reference
//  mod:         0.01, 
//  mod:         2.0 
//  mod:     );
//  mod:   void cvCalcOpticalFlowPyrLK( --> void cvCalcOpticalFlowPyrLK( <-- removed void
//  mod:   CvPoint p0 = cvPoint( cvRound( cornersA[i].x ), cvRound( cornersA[i].y ) ); <-- capitalize cvPoint
//  mod:   CvPoint p1 = cvPoint( cvRound( cornersB[i].x ), cvRound( cornersB[i].y ) ); <-- capitalize cvPoint
//  removed:   CvxMultiWin mw;
//  removed:   mw.initialize( "OpticalFlowPyrLK", imgC, 3, 1, 0 );
//  removed:   mw.paint( imgA );
//  removed:   mw.paint( imgB );
//  removed:   mw.paint( imgC );
//  removed:   mw.redraw();
//  added:     } at the end
//
//  for imgA,imgB,imgC:
//  added cvNamedWindow
//  added cvShowImage
//  added cvDestroyWindow
//  added cvReleaseImage
//

#include <cv.h>
#include <highgui.h>
#define CVX_RED cvScalar(100)

const int MAX_CORNERS = 400;

int main(int argc, char** argv)
{
    // Initialize, load two images from the file system, and
    // allocate the images and other structures we will need for
    // results.
    //
    IplImage* imgA = cvLoadImage( "image1.bmp", 0 );
    IplImage* imgB = cvLoadImage( "image2.bmp", 0 );

    CvSize img_sz = cvGetSize( imgA );

    IplImage* imgC = cvCreateImage( img_sz, IPL_DEPTH_8U, 1 );

    cvNamedWindow( "A" );
    cvNamedWindow( "B" );
    cvNamedWindow( "C" );



    // The first thing we need to do is get the features
    // we want to track.
    //
    IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
    IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );

    int           corner_count = MAX_CORNERS; 
    CvPoint2D32f* corners      = new CvPoint2D32f[ MAX_CORNERS ];
    CvPoint2D32f* cornersA     = new CvPoint2D32f[ MAX_CORNERS ];

    cvGoodFeaturesToTrack( 
        imgA, 
        eig_image,
        tmp_image,
        corners, 
        &corner_count,
        0.01, 
        2.0 
    );


    // Call the Lukas Kanade algorithm
    //
    char  features_found[ MAX_CORNERS ];
    float feature_errors[ MAX_CORNERS ];

    CvSize pyr_sz = cvSize( imgA->width+8, imgB->height );

    IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
    IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 ); 

    CvPoint2D32f* cornersB     = new CvPoint2D32f[ MAX_CORNERS ];

    cvCalcOpticalFlowPyrLK( 
        imgA, 
        imgB, 
        pyrA, 
        pyrB,
        cornersA, 
        cornersB,
        corner_count,
        cvSize( 3, 3 ), 
        5,
        features_found,
        feature_errors, 
        cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ), 
        0 
    );

    // Now make some image of what we are looking at:
    //
    cvZero( imgC );
    for( int i=0; i<corner_count; i++ ) {
        if( features_found[i]==0 || feature_errors[i]>20 ) {
            continue;
        }
        CvPoint p0 = cvPoint( cvRound( cornersA[i].x ), cvRound( cornersA[i].y ) );
        CvPoint p1 = cvPoint( cvRound( cornersB[i].x ), cvRound( cornersB[i].y ) );

        cvLine( imgC, p0, p1, CVX_RED );
    }
    cvShowImage  ( "A" , imgA );
    cvShowImage  ( "B" , imgB );
    cvShowImage  ( "C" , imgC );
    
    cvWaitKey(0);

    cvDestroyWindow ( "A" );
    cvDestroyWindow ( "B" );
    cvDestroyWindow ( "C" );
    cvReleaseImage  ( &imgA );
    cvReleaseImage  ( &imgB );
    cvReleaseImage  ( &imgC );
    return 0;
}
