#include "calonder_descriptor/randomized_tree.h"
#include <cv.h>
#include <highgui.h>
#include <boost/foreach.hpp>
#include <map>
#include <cassert>
#include <cstdio>

using namespace features;
using std::string;

char wndname[] = "Base keypoints";

void DrawPoints(IplImage* dst, std::vector<BaseKeypoint> const& pts,
                CvScalar color = CV_RGB(0,255,0))
{
  BOOST_FOREACH( BaseKeypoint const& pt, pts )
    cvCircle(dst, cvPoint(pt.x, pt.y), 3, color, 1);
}

typedef std::vector<BaseKeypoint> BaseVec;
typedef std::pair< IplImage*, BaseVec > MapValue;
typedef std::map< const string, MapValue > KeypointMap;

void DisplayImage(MapValue const& value)
{
  IplImage *image = value.first;
  IplImage* color_img = cvCreateImage(cvSize(image->width, image->height),
                                      IPL_DEPTH_8U, 3);
  cvCvtColor(image, color_img, CV_GRAY2BGR);
  DrawPoints(color_img, value.second);
  cvShowImage(wndname, color_img);
  cvReleaseImage(&color_img);
}

int main( int argc, char** argv )
{
  assert(argc > 1);

  // Load base set data and images
  KeypointMap keypt_map;
  FILE* file = fopen(argv[1], "r");
  if (file) {
    int x, y;
    char image_file[128];

    while ( !feof(file) ) {
      fscanf(file, "%d %d %s\n", &x, &y, image_file);
      string file_string(image_file);
      MapValue &value = keypt_map[file_string];
      if (value.first == NULL)
        value.first = cvLoadImage(file_string.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
      value.second.push_back( BaseKeypoint(x, y, value.first) );
    }
    fclose(file);
  }
  else {
    printf("Error: could not open data file\n");
    return 1;
  }

  cvNamedWindow(wndname);
  KeypointMap::iterator keymap_it = keypt_map.begin(), old;
  DisplayImage(keymap_it->second);

  for(;;){
    int c = cvWaitKey(0);
    switch( (char) c ) {
      case '\x1b':
        printf("Exiting...\n");
        goto exit_main;
      case 'n':
        old = keymap_it++;
        if (keymap_it == keypt_map.end())
          keymap_it = old;
        DisplayImage(keymap_it->second);
        break;
      case 'p':
        if (keymap_it != keypt_map.begin())
          --keymap_it;
        DisplayImage(keymap_it->second);
        break;
    }
  }

  // Clean up
exit_main:
  for (keymap_it == keypt_map.begin(); keymap_it != keypt_map.end();
       ++keymap_it) {
    cvReleaseImage(&keymap_it->second.first);
  }

  return 0;
}
