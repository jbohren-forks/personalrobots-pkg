//
// example 6-1
//
//  removed spaces: cvNamedWindow( " cvHoughCircles ", 1 ); -->  cvNamedWindow( "cvHoughCircles", 1 );
//  CV_LOAD_IMAGE_GRAY --> CV_LOAD_IMAGE_GRAYSCALE
//  gray->width/10 --> image->width/10 
//

#include <cv.h>
#include <highgui.h>
#include <math.h>

int main(int argc, char** argv) {
  IplImage* image = cvLoadImage( 
    argv[1],
    CV_LOAD_IMAGE_GRAYSCALE
  );

  CvMemStorage* storage = cvCreateMemStorage(0);
  cvSmooth(image, image, CV_GAUSSIAN, 5, 5 );
  CvSeq* results = cvHoughCircles( 
    image, 
    storage, 
    CV_HOUGH_GRADIENT, 
    2, 
    image->width/10 
  );
  
  for( int i = 0; i < results->total; i++ ) {
    float* p = (float*) cvGetSeqElem( results, i );
    CvPoint pt = cvPoint( cvRound( p[0] ), cvRound( p[1] ) );
    cvCircle( 
      image,
      pt, 
      cvRound( p[2] ),
      CV_RGB(0xff,0xff,0xff) 
    );
  }
  cvNamedWindow( "cvHoughCircles", 1 );
  cvShowImage( "cvHoughCircles", image);
  cvWaitKey(0);
}

