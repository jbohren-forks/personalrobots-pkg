// Frame Difference sample code
// Gary Bradski 3/8/08
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "cv_yuv_codebook.h"

//VARIABLES for CODEBOOK METHOD:

void help() {
	printf("\nFRAME DIFFERENCING:\n"
		"\nUSAGE: // frameDiff lag_between_frames [movie filename, else from camera]\n"
		"INTERACTIVE PARAMETERS:\n"
		"\tESC,q,Q  - quit the program\n"
		"\th	- print this help\n"
		"\tp	- pause toggle\n"
		"\ts	- single step\n"
		"\tr	- run mode (single step off)\n"
		"=== AVG PARAMS ===\n"
		"\t[    - bump threshold UP by 1\n"
		"\t]    - bump threshold DOWN by 1\n"
		);
}

int main(int argc, char** argv)
{
 	IplImage* rawImage = 0, *grayImage = 0, *grayImageLag; 
    IplImage *Imask = 0;
    CvCapture* capture = 0;

	int c,n;
	int threshold =  15;
	int lag = 1;
	
	if(argc < 2) {
		printf("ERROR: Too few parameters\n");
		help();
	}else{
		if(argc == 2){
			printf("Capture from Camera\n");
			capture = cvCaptureFromCAM( 0 );
		}
		else {
			printf("Capture from file %s\n",argv[2]);
			capture = cvCaptureFromFile( argv[2] );
		}
		if(isdigit(argv[1][0])) { //Start from of background capture
			lag = atoi(argv[1]);
			if(lag <= 0)
				lag = 1;
		}
		printf("Frame difference lab = %d\n",lag);
	}

	//MAIN PROCESSING LOOP:
	bool pause = false;
	bool singlestep = false;

    if( capture )
    {
        cvNamedWindow( "Raw", 1 );
		cvNamedWindow( "Lag",1);
		cvNamedWindow( "Diff",1);
		cvNamedWindow( "Diff_ConnectComp",1);
        int i = -1;
        
        for(;;)
        {
			if(!pause){
        		if( !cvGrabFrame( capture ))
                	break;
            	rawImage = cvRetrieveFrame( capture );
				++i; //count it
			}
			//First time:
			if(0 == i) {
				//ALLOCATION
				Imask = cvCreateImage( cvGetSize(rawImage), IPL_DEPTH_8U, 1 );
				grayImage = cvCreateImage( cvGetSize(rawImage), IPL_DEPTH_8U, 1 );
				grayImageLag = cvCloneImage(grayImage);
				cvCvtColor( rawImage, grayImageLag, CV_BGR2GRAY); //To gray
			}
			
       		// printf("%d ",i);
			//If we've got an rawImage and are good to go:                
        	if( rawImage )
        	{
				cvCvtColor( rawImage, grayImage, CV_BGR2GRAY );//Convert to gray
				cvAbsDiff(grayImage,grayImageLag,Imask); //Do lagged subtraction
				if(singlestep){
					pause = true;
				}
				cvThreshold(Imask, Imask, threshold,255, CV_THRESH_BINARY);
				cvShowImage("Raw", grayImage);
				cvShowImage("Lag", grayImageLag);
				cvShowImage("Diff", Imask);
				cvconnectedComponents(Imask); //Clean mask
				cvShowImage("Diff_ConnectComp",Imask);

				//USER INPUT:
	         	c = cvWaitKey(10);
				//End processing on ESC, q or Q
				if(c == 27 || c == 'q' | c == 'Q')
					break;
				//Esle check for user input
				switch(c)
				{
					case 'h':
						help();
						break;
					case 'p':
						pause ^= 1;
						break;
					case 's':
						singlestep = 1;
						pause = false;
						break;
					case 'r':
						pause = false;
						singlestep = false;
						break;
					//THRESHOLD CONTROL
					case '[':
							threshold += 1;
							if(threshold > 255) threshold = 255;
							printf("threshold=%d\n",threshold);
						break;
					case ']':
							threshold -= 1;
							if(threshold < 0) threshold = 0;
							printf("threshold=%d\n",threshold);
						break;
				}
				if((!(i%lag))&&(!pause)) { //every lagth frame, update the difference
						cvCopy(grayImage,grayImageLag);
				}
            }
		}		
        cvReleaseCapture( &capture );
        cvDestroyWindow( "Raw" );
		cvDestroyWindow( "Lag" );
		cvDestroyWindow( "Diff_ConnectComp");
		cvDestroyWindow( "Diff");
		if(grayImage) cvReleaseImage(&grayImage);
		if(Imask) cvReleaseImage(&Imask);
		if(grayImageLag) cvReleaseImage(&grayImageLag);
    }
	else{ printf("Darn\n");
	}
    return 0;
}


