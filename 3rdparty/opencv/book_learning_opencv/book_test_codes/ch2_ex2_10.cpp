#include "cv.h"
#include "highgui.h"

// Convert a video to grayscale
// argv[1]: input video file
// argv[2]: name of new output file
//
main( int argc, char* argv[] ) {
    CvCapture* capture = cvCreateFileCapture( argv[1] );
    if (!capture){
        return -1;
    }
    IplImage* bgr_frame;
    double fps = cvGetCaptureProperty (
        capture,
        CV_CAP_PROP_FPS
    );

    CvSize size = cvSize(
        (int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH),
        (int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT)
    );
    
    CvVideoWriter* writer = cvCreateVideoWriter(
        argv[2],
        CV_FOURCC('M','J','P','G'),
        fps,
        size
    );
    
    IplImage* logpolar_frame = cvCreateImage(
        size,
        IPL_DEPTH_8U,
        3
    );

    IplImage* gray_frame = cvCreateImage(
        size,
        IPL_DEPTH_8U,
        1
    );

    while( (bgr_frame=cvQueryFrame(capture)) != NULL ) {
        cvConvertImage(
            bgr_frame,
            gray_frame,
            CV_RGB2GRAY
        );
        cvLogPolar( bgr_frame, logpolar_frame, 
                    cvPoint2D32f(bgr_frame->width/2,
                    bgr_frame->height/2), 
                    40, 
                    CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );
        cvWriteToAVI( writer, logpolar_frame );
    }
    cvReleaseVideoWriter( &writer );
    cvReleaseImage( &gray_frame );
    cvReleaseImage( &logpolar_frame );
    cvReleaseImage( &bgr_frame );
    cvReleaseCapture( &capture );
}
