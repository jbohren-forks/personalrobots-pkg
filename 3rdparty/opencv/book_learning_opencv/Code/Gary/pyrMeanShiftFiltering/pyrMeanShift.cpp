#ifdef _CH_
#pragma package <opencv>
#endif

#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <stdlib.h>

IplImage *src = 0, *dst = 0;

void help(){
    printf( "\t\tUsage: pyrMeanShift <image_file_name>\n" 
			"Hot keys: \n"
            "\tESC,q,Q - quit the program\n"
			"\th   - print help\n"
			"\ts   - save images\n"
       		"\tp or ENTER - run pyramid segmentation algorithm\n"
			"------------------------\n"
			"\tm,M - max_level down,Up\n"
			"\tc,C - colorRadius down,Up\n"
			"\tr,R - spacial radius spRadius down,Up\n"
                );
}

int main( int argc, char** argv )
{
    char* filename = argc >= 2 ? argv[1] : (char*)"fruits.jpg";
     CvTermCriteria termcrit=cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,5,1);
	double spRadius = 20.0, colorRadius = 40.0;
	int max_level = 2;

    if( (src = cvLoadImage(filename,-1)) == 0 )
        return 0;

	help();   
    cvNamedWindow( "Image", 1 );
	cvNamedWindow( "Segmented", 1);

    dst = cvCloneImage( src );
    cvShowImage( "Image", src );

    for(;;)
    {
        char c;
		do {//In linux, we get rid of shift codes which turn c negative.
			c = cvWaitKey(0);
			if(c < 0) c = -c;
		} while( (c == 30) || (c == 31)); //Shift codes
//		printf("c=%d\n",c);

        if(( (char)c == 27) || ((char)c == 'q') || ((char)c == 'Q') )//Quit
            break;

		switch(c){
			case 'H':
			case 'h':
				help();
				break;
			case's': //save image as watershed.jpg	
            	cvSaveImage("pyrMeanShiftInput.jpg",src);
             	cvSaveImage("pyrMeanShiftOutput.jpg",dst);
             	printf("Saved image pyrMeanShift*.jpg\n");
				break;
			case 'm':
				max_level -= 1;
				if(max_level <0) max_level = 0;
				printf("max_level = %d\n",max_level);
				break;
			case 'M'://"
				max_level += 1;
				printf("max_level = %d\n",max_level);
				break;
			case 'r':
				spRadius -= 1.0;
				if(spRadius < 1.0) spRadius = 1.0;
				printf("spRadius = %lf\n",spRadius);
				break;
			case 'R':
				spRadius += 1.0;
				printf("spRadius = %lf\n",spRadius);
				break;
			case 'c':
				colorRadius -= 1.0;
				if(colorRadius < 1.0) colorRadius = 1.0;
				printf("colorRadius = %lf\n",colorRadius);
				break;
			case 'C':
				colorRadius += 1.0;
				printf("colorRadius = %lf\n",colorRadius);
				break;
			case 'p':
			case '\n':
	           	double t = (double)cvGetTickCount();
				cvPyrMeanShiftFiltering( src, dst, spRadius, colorRadius, max_level, termcrit);
	           	t = (double)cvGetTickCount() - t;
				printf("For params image(x,y)=(%d,%d) max_level=%d, Radius color=%lf, space=%lf\n",
						src->width,src->height,max_level,colorRadius,spRadius);
            	printf( "exec time = %gms\n", t/(cvGetTickFrequency()*1000.) );
    	        cvShowImage( "Segmented", dst );
  				break;
		}
    }

    return 1;
}
