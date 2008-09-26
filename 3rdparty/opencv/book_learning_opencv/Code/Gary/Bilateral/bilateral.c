#include "cv.h"
#include "highgui.h"

IplImage *I = 0, *Ic = 0;

int main( int argc, char** argv )
{
    char* filename = argc == 2 ? argv[1] : (char*)"fruits.jpg";
    
    if( (I = cvLoadImage( filename, 1)) == 0 )
        return -1;

    // Create the output image
    Ic = cvCreateImage(cvSize(I->width,I->height), IPL_DEPTH_8U, 3);

    //Bilateral filter
    cvSmooth( I, Ic, CV_BILATERAL, 3, 50, 3, 50);

	//PUT UP 2 WINDOWS
	cvNamedWindow("Raw",1);
        cvNamedWindow("Smoothed",1);

	//Show the results
	cvShowImage("Raw",I);
	cvShowImage("Smoothed",Ic);

	cvWaitKey(0);

	//Clean up
	cvReleaseImage(&I);
	cvReleaseImage(&Ic);
	cvDestroyWindow("Raw");
	cvDestroyWindow("Smoothed");

	return(0);
}


