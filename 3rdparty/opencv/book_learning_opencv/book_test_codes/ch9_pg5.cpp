//
// chapter 9 corrected, page 5
//
//  added the beginninng part
//
//  assigned arbitrary constants to pt1 and pt2
//
//  works without changing, with addition of basic video reading
//
// STORE TO DISK A LINE SEGMENT OF BGR PIXELS FROM pt1 to pt2.  
//…

#include <stdio.h>
#include <cv.h>
#include <highgui.h>

int main( int argc, char** argv  )
{

    cvNamedWindow( "Example2", CV_WINDOW_AUTOSIZE );
    CvCapture* capture = cvCreateFileCapture( argv[1] );

    CvPoint pt1 = cvPoint(10,10);
    CvPoint pt2 = cvPoint(20,20);

    int max_buffer;
    IplImage *rawImage;
    int r[10000],g[10000],b[10000];
    FILE *fptrb = fopen("blines.csv","w"); //Store the data here
    FILE *fptrg = fopen("glines.csv","w"); // for each color channel
    FILE *fptrr = fopen("rlines.csv","w");
    CvLineIterator iterator;
    //MAIN PROCESSING LOOP:
    for(;;){
        if( !cvGrabFrame( capture ))
              break;
        rawImage = cvRetrieveFrame( capture );
        max_buffer = cvInitLineIterator(rawImage,pt1,pt2,&iterator,8,0);
        cvShowImage( "Example2", rawImage );
        int c = cvWaitKey(10);
        for(int j=0; j<max_buffer; j++){
            fprintf(fptrb,"%d,", iterator.ptr[0]); //Write blue value
            fprintf(fptrg,"%d,", iterator.ptr[1]); //green
            fprintf(fptrr,"%d,", iterator.ptr[2]); //red
            iterator.ptr[2] = 255;  //Mark this sample in red
            CV_NEXT_LINE_POINT(iterator); //Step to the next pixel
        }
        //OUTPUT THE DATA IN ROWS:
        fprintf(fptrb,"\n");fprintf(fptrg,"\n");fprintf(fptrr,"\n");
    }
    //CLEAN UP:
    fclose(fptrb); fclose(fptrg); fclose(fptrr);
    cvReleaseCapture( &capture );
    cvDestroyWindow( "Example2" );
}
