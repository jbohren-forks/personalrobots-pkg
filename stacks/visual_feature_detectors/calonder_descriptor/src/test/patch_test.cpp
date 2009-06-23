#include "calonder_descriptor/patch_generator.h"
#include <highgui.h>
#include <ctime>

using namespace features;

char wndname[] = "Patch";

int main( int argc, char** argv )
{
  IplImage* source = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  IplImage* patch = cvCreateImage(cvSize(128, 128), IPL_DEPTH_8U, 1);
  Rng rng(std::time(NULL));
  PatchGenerator make_patch(source, rng);
  make_patch.addWhiteNoise(true);

  cvNamedWindow(wndname);

  for(;;)
  {
    make_patch(cvPoint(300, 300), patch);
    cvShowImage(wndname, patch);
    
    char c = cvWaitKey(0);

    if (c == '\x1b')
      goto exit_main;
  }

exit_main:
  
  cvReleaseImage(&source);
  cvReleaseImage(&patch);
  cvDestroyWindow(wndname);
  
  return 0;
}
