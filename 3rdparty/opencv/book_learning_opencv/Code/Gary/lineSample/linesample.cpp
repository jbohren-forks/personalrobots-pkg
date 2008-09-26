// Line sampling code (this will be morphed into a line sampler code ...
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>


void help() {
	printf("\nKeep statistics on a line through the image:\n"
		"\nUSAGE: ./linesample startFrameCollection# endFrameCollection# movie_file x1 y1 x2 y2\n\n"
		);
}

int main(int argc, char** argv)
{
    CvCapture* capture = 0;

	int startcapture = 1;
	int endcapture = 30;
	CvPoint pt1,pt2;
	int c,n;


	if(argc < 8) {
		printf("ERROR: Too few parameters\n");
		help();
		return -1;
	}	
	printf("Capture from file %s\n",argv[3]);
	capture = cvCaptureFromFile( argv[3] );
	if(!capture) {printf("Couldn't capture from %s\n",argv[3]); help(); return -1;}

	if(isdigit(argv[1][0])) { //Start from of background capture
		startcapture = atoi(argv[1]);
	}else {printf("%s not a digit\n",argv[1]); help(); return -1;}

	if(isdigit(argv[2][0])) { //End frame of background capture
		endcapture = atoi(argv[2]);
	}else {printf("%s not a digit\n",argv[2]); help(); return -1;}

	if(isdigit(argv[4][0])){
		pt1.x = atoi(argv[4]);
	}else {printf("%s not a digit\n",argv[4]); help(); return -1;}

	if(isdigit(argv[5][0])){
		pt1.y = atoi(argv[5]);
	}else {printf("%s not a digit\n",argv[5]); help(); return -1;}

	if(isdigit(argv[6][0])){
		pt2.x = atoi(argv[6]);
	}else {printf("%s not a digit\n",argv[6]); help(); return -1;}

	if(isdigit(argv[7][0])){
		pt2.y = atoi(argv[7]);
	}else {printf("%s not a digit\n",argv[7]); help(); return -1;}

	int i=0;
	int max_buffer;
	IplImage *rawImage;
	//MAIN PROCESSING LOOP:
    cvNamedWindow( "Image", 0 );
	int r[10000],g[10000],b[10000];
	CvLineIterator iterator;
	FILE *fptrb = fopen("blines.csv","w");
	FILE *fptrg = fopen("glines.csv","w");
	FILE *fptrr = fopen("rlines.csv","w");
    for(;;){
        if( !cvGrabFrame( capture ))
              	break;
        rawImage = cvRetrieveFrame( capture );
		max_buffer = cvInitLineIterator(rawImage,pt1,pt2,&iterator,8,0);
		for(int j=0; j<max_buffer; j++){
			b[j] = iterator.ptr[0];
			g[j] = iterator.ptr[1];
			r[j] = iterator.ptr[2];
			iterator.ptr[2] = 255;

			fprintf(fptrb,"%d,",b[j]);
			fprintf(fptrg,"%d,",g[j]);
			fprintf(fptrr,"%d,",r[j]);
			
			CV_NEXT_LINE_POINT(iterator);
		}
		fprintf(fptrb,"\n");
		fprintf(fptrg,"\n");
		fprintf(fptrr,"\n");

		++i; //count it
//		cvLine(rawImage,pt1,pt2,cvScalar(255,100,0),1);
		cvShowImage("Image",rawImage);
		c=cvWaitKey(125);
		if(27 == c)
			break;
		printf("%d \n",i);
		if('p' == c) {
            c = 0;
            while(c != 'p' && c != 27){
                    c = cvWaitKey(250);
            }
		}
	}
	fclose(fptrb);
	fclose(fptrg);
	fclose(fptrr);

	cvReleaseCapture( &capture );
    cvDestroyWindow( "Image" );
    return 0;
}


