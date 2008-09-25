#include <stdio.h>
#include <iostream>
#include <fstream>
#include "cv.h"
#include "highgui.h"
using namespace std;

int        g_slider_position = 0;
CvCapture* g_capture         = NULL;

void onTrackbarSlide(int pos) {
    cvSetCaptureProperty(
        g_capture,
        CV_CAP_PROP_POS_FRAMES,
        pos
    );
}

int getAVIFrames(char * fname) {
    char tempSize[4];
    // Trying to open the video file
    ifstream  videoFile( fname , ios::in | ios::binary );
    // Checking the availablity of the file
    if ( !videoFile ) {
      cout << "Couldn’t open the input file " << fname << endl;
      exit( 1 );
    }
    // get the number of frames
    videoFile.seekg( 0x30 , ios::beg );
    videoFile.read( tempSize , 4 );
    int frames = (unsigned char ) tempSize[0] + 0x100*(unsigned char ) tempSize[1] + 0x10000*(unsigned char ) tempSize[2] +    0x1000000*(unsigned char ) tempSize[3];
    videoFile.close(  );
    return frames;
}


int main( int argc, char** argv ) {
    cvNamedWindow( "Example3", CV_WINDOW_AUTOSIZE );
    g_capture = cvCreateFileCapture( argv[1] );


    int frames = (int) cvGetCaptureProperty(
        g_capture,
        CV_CAP_PROP_FRAME_COUNT
    );
    int tmpw = (int) cvGetCaptureProperty(
        g_capture,
        CV_CAP_PROP_FRAME_WIDTH
    );

    int tmph = (int) cvGetCaptureProperty(
        g_capture,
        CV_CAP_PROP_FRAME_HEIGHT
    );

    printf("opencv frames %d w %d h %d\n",frames,tmpw,tmph);

    frames = getAVIFrames(argv[1]);

    printf("hacked frames %d w %d h %d\n",frames,tmpw,tmph);

    cvCreateTrackbar(
        "Position",
        "Example3",
        &g_slider_position,
        frames,
        onTrackbarSlide
    );
    IplImage* frame;

    while(1) {
        frame = cvQueryFrame( g_capture );
        if( !frame ) break;
        cvShowImage( "Example3", frame );
        char c = (char)cvWaitKey(10);
        if( c == 27 ) break;
    }
    cvReleaseCapture( &g_capture );
    cvDestroyWindow( "Example3" );
    return(0);
}
