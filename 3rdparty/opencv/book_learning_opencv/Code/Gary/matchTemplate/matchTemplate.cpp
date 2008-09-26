#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <stdio.h>


/*
#define  CV_TM_SQDIFF        0
#define  CV_TM_SQDIFF_NORMED 1
#define  CV_TM_CCORR         2
#define  CV_TM_CCORR_NORMED  3
#define  CV_TM_CCOEFF        4
#define  CV_TM_CCOEFF_NORMED 5
cvMatchTemplate( const CvArr* image, const CvArr* templ,
                              CvArr* result, int method );
image
    Image where the search is running. It should be 8-bit or 32-bit floating-point. 
templ
    Searched template; must be not greater than the source image and the same data type as the image. 
result
    A map of comparison results; single-channel 32-bit floating-point. 
	If image is W×H and templ is w×h then result must be W-w+1×H-h+1.						  
*/

// Call is
// matchTemplate image template method
//
// Will Display the results of the match
// 
int main( int argc, char** argv ) {

    IplImage *src, *templ,*ftmp[6]; //ftmp is what to display on
	int i,method = 0;
    if( argc == 4){ 
		method = atoi(argv[3]); //method of matching
		if(method > 5) method = 0; 
		if(method < 0) method = 0;
		printf("method = %d\n",method);
		//Read in the source image to be searched:
		if((src=cvLoadImage(argv[1], 1))== 0) {
				printf("Error on reading src image %s\n",argv[i]);
				return(-1);
		}
		//Read in the template to be used for matching:
		if((templ=cvLoadImage(argv[2], 1))== 0) {
				printf("Error on reading template %s\n",argv[2]);
				return(-1);
		}

 		int patchx = templ->width;
		int patchy = templ->height;
		int iwidth = src->width - patchx + 1;
		int iheight = src->height - patchy + 1;
		for(i=0; i<6; ++i){
//			printf("i=%d\n",i);
			ftmp[i] = cvCreateImage( cvSize(iwidth,iheight),32,1);
		}
//		cvZero(ftmp);
		//DO THE MATCHING OF THE TEMPLATE WITH THE IMAGE
		for(i=0; i<6; ++i){
			cvMatchTemplate( src, templ, ftmp[i], i); //, method);
//		double min,max;
//		cvMinMaxLoc(ftmp,&min,&max);
			cvNormalize(ftmp[i],ftmp[i],1,0,CV_MINMAX);
		}
        //DISPLAY
		cvNamedWindow( "Template", 0 );
        cvShowImage(   "Template", templ );
        cvNamedWindow( "Image", 0 );
        cvShowImage(   "Image", src );

		cvNamedWindow( "SQDIFF", 0 );
        cvShowImage(   "SQDIFF", ftmp[0] );

		cvNamedWindow( "SQDIFF_NORMED", 0 );
        cvShowImage(   "SQDIFF_NORMED", ftmp[1] );

		cvNamedWindow( "CCORR", 0 );
        cvShowImage(   "CCORR", ftmp[2] );

		cvNamedWindow( "CCORR_NORMED", 0 );
        cvShowImage(   "CCORR_NORMED", ftmp[3] );

		cvNamedWindow( "CCOEFF", 0 );
        cvShowImage(   "CCOEFF", ftmp[4] );

		cvNamedWindow( "CCOEFF_NORMED", 0 );
        cvShowImage(   "CCOEFF_NORMED", ftmp[5] );

/*
#define  CV_TM_SQDIFF        0
#define  CV_TM_SQDIFF_NORMED 1
#define  CV_TM_CCORR         2
#define  CV_TM_CCORR_NORMED  3
#define  CV_TM_CCOEFF        4
#define  CV_TM_CCOEFF_NORMED 5
*/

		//LET USER VIEW RESULTS:
        cvWaitKey(0);
    }
	else { printf("Call should be:  matchTemplate image template method\n");}
}
