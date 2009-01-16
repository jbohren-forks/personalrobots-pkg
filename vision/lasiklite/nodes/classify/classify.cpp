/*****************************************************************************
** STAIR VISION PROJECT
** Copyright (c) 2007-2008, Stephen Gould
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the Stanford University nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
*****************************************************************************
**
** FILENAME:    classifyImage.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**              Morgan Quigley <mquigley@cs.stanford.edu>
** DESCRIPTION:
**  Application for classifying images.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include "ros/node.h"
#include "std_msgs/Image.h"
#include "image_utils/cv_bridge.h"

#include "svlLib.h"

using namespace std;

#define WINDOW_NAME "lasiklite-classify"

svlObject2dFrame classifyImage(IplImage *image, svlObjectDetector *classifier,
    vector<CvRect>& rects);

static bool read_rects_from_file( vector<CvRect> &, char*);

bool rect_is_shorter( const CvRect& a, const CvRect& b ) { return a.height < b.height; }
bool  rect_is_taller( const CvRect& a, const CvRect& b ) { return a.height > b.height; }

int getIndexFromFilename(const char *filename);

void usage()
{
    cerr << "USAGE: ./classifyImage [OPTIONS] [<dictionary>] <model> <image>*" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -b                 :: only search at base scale" << endl
	 << "  -c <type>          :: classifier type: GABOR, PATCH (default)" << endl
	 << "  -map <filename>    :: save response map to file <filename>" << endl
	 << "  -n <object name>   :: name of object" << endl
	 << "  -o <filename>      :: output to file" << endl
	 << "  -p                 :: turn on SVL code profiling" << endl
	 << "  -r <x> <y> <w> <h> :: scan subregion of image" << endl
	 << "  -R <filename>      :: scan rectangle file" <<endl
	 << "  -s <w> <h>         :: resize image to <w>-by-<h>" << endl
	 << "  -t <threshold>     :: classifier threshold (default: 0.5)" << endl
	 << "  -x                 :: display image and matches" << endl
	 << endl;
    cerr << "Copyright (c) 2007-2008, Stephen Gould" << endl << endl;
}

IplImage *image = NULL; // the input image coming across the wire
ros::thread::mutex image_mutex;

class Classify : public ros::Node
{
public:
  std_msgs::Image image_msg;
  CvBridge<std_msgs::Image> cv_bridge;
  Classify() : ros::Node("classify"), cv_bridge(&image_msg)
  {
    subscribe("image", image_msg, &Classify::image_cb);
  }
  void image_cb()
  {
    image_mutex.lock();
    cv_bridge.to_cv(&image);
    image_mutex.unlock();
  }
};

int main(int argc, char *argv[])
{		
  ros::init(argc, argv);

  // read commandline parameters
  const char *dictionaryFilename = NULL;
  const char *classifierFilename = NULL;
  const char *outputFilename = NULL;
  const char *objectName = "[unknown]";
  const char *classifierType = "patch";
  const char *responseMapFilename = NULL;
  double threshold = 0.5;
  bool bDisplayImage = false;
  int width = -1;
  int height = -1;
  bool bBaseScaleOnly = false;

  char *rectFile = NULL;
  vector<CvRect> rects; // one vector per image

  CvRect region = cvRect(0, 0, 0, 0);

  char **args = argv + 1;
  while (--argc) 
  {
    if (!strcmp(*args, "-n")) {
      objectName = *(++args);                   argc--;
    } else if (!strcmp(*args, "-b")) {
      bBaseScaleOnly = true;
    } else if (!strcmp(*args, "-c")) {	    
      classifierType = *(++args);               argc--;
    } else if (!strcmp(*args, "-map")) {	    
      responseMapFilename = *(++args);          argc--;	    
    } else if (!strcmp(*args, "-o")) {
      outputFilename = *(++args);               argc--;
    } else if (!strcmp(*args, "-p")) {
      cerr << "NOT IMPLEMENTED" << endl;
      assert(false);
    } else if (!strcmp(*args, "-r")) {
      region.x = atoi(*(++args));               argc--;
      region.y = atoi(*(++args));               argc--;
      region.width = atoi(*(++args));           argc--;
      region.height = atoi(*(++args));          argc--;
    } else if (!strcmp(*args, "-s")) {
      width = atoi(*(++args));                  argc--;
      height = atoi(*(++args));                 argc--;
    } else if (!strcmp(*args, "-t")) {
      threshold = atof(*(++args));              argc--;
    } else if (!strcmp(*args, "-R")) {
      //read_rects_from_file( rects, *(++args) ); argc--;
      rectFile = *(++args);                       argc--;
    } else if (!strcmp(*args, "-x")) {
      bDisplayImage = true;
    } else if ((*args)[0] == '-') {
      cerr << "ERROR: unrecognized option " << *args << endl;
      return -1;
    } else {
      break;
    }
    args++;
  }

  if (argc < (!strcasecmp(classifierType, "gabor") ? 1 : 2)) {
    usage();
    return -1;
  }

  if (!strcasecmp(classifierType, "patch")) {
    dictionaryFilename = *args++; argc--;
  }
  classifierFilename = *args++; argc--;

  // argc now is the number of image files to process
  if (rectFile)
    read_rects_from_file(rects, rectFile);

  int total_rects_initial = 0;
  if (rectFile) {
    total_rects_initial = rects.size();
  }

  // initialize object detector
  svlObjectDetector *classifier = NULL;
  if (!strcasecmp(classifierType, "gabor")) {
    cerr << "NOT IMPLEMENTED" << endl;
    assert(false);
  } else if (!strcasecmp(classifierType, "patch")) {
    classifier = new svlPatchBasedClassifier(objectName);
    cerr << "Loading dictionary " << dictionaryFilename << "..." << endl;
    ((svlPatchBasedClassifier *)classifier)->readDictionary(dictionaryFilename);
  }
  assert(classifier != NULL);
  cerr << "Loading classifier " << classifierFilename << "..." << endl;
  classifier->readModel(classifierFilename);
  classifier->setThreshold(threshold);

  // run classifier on all supplied images
  IplImage *resultsImage = NULL;

  if (bDisplayImage) {
    cvNamedWindow(WINDOW_NAME, 0);
  }

  Classify classify;
  while (classify.ok())
  {
    image_mutex.lock();
    if (!image)
    {
      image_mutex.unlock();
      continue;
    }

    //cvFlip(image, NULL, 1); 
    if (image->nChannels != 1) 
    {
      IplImage *grey = cvCreateImage(cvSize(image->width, image->height), 
                                     IPL_DEPTH_8U, 1);
      cvCvtColor(image, grey, CV_RGB2GRAY);
      cvReleaseImage(&image);
      image = grey;	       
    }
    // resize image if -s commandline argument given
    if ((width > 0) && (height > 0)) 
    {
      cerr << "Resizing image from " << image->width << " x " << image->height 
        <<	" to " << width << " x " << height << "..." << endl;
      IplImage *img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
      cvResize(image, img);
      cvReleaseImage(&image);
      image = img;
    }
    // classify image
    svlObject2dFrame objects;
    if ((region.width != 0) && (region.height != 0)) {
      if (bBaseScaleOnly) {
        vector<CvPoint> locations;
        classifier->createWindowLocations(region.width, region.height, 
                                          locations);
        for (vector<CvPoint>::iterator it = locations.begin(); 
             it != locations.end(); ++it) {
          it->x += region.x;
          it->y += region.y;
        }
        objects = classifier->classifyImage(image, locations);
      } else {
        objects = classifier->classifyRegion(image, region);
      }
    } else {
      if (!rectFile) {
        if (bBaseScaleOnly) 
        {
          vector<CvPoint> locations;
          classifier->createWindowLocations(image->width, image->height, 
                                            locations);
          objects = classifier->classifyImage(image, locations);
        } else {
          objects = classifier->classifyImage(image);
        }
      } else {
        objects = classifyImage(image, classifier, rects);
      }
    }

    if (bDisplayImage) {
      IplImage *colourImage = cvCreateImage(cvSize(image->width, image->height),
                                            IPL_DEPTH_8U, 3);
      cvCvtColor(image, colourImage, CV_GRAY2RGB);
      if (resultsImage != NULL) {		
        cvReleaseImage(&resultsImage);
      }

      /*
         if (rectFile) {
         for (vector<CvRect>::iterator it=rects[index].begin(); it!=rects[index].end(); ++it ) {
         cvRectangle(colourImage, cvPoint( it->x, it->y ),
         cvPoint( it->x + it->width - 1, it->y + it->height - 1),
         CV_RGB(0, 64, 0), 1);
         }
         }*/

      for (unsigned i = 0; i < objects.size(); i++) {
        cvRectangle(colourImage, cvPoint((int)objects[i].x, (int)objects[i].y),
            cvPoint((int)(objects[i].x + objects[i].w - 1), 
              (int)(objects[i].y + objects[i].h - 1)),
            CV_RGB(0, 255, 0), 1);
      }

      cvRectangle(colourImage, cvPoint(region.x, region.y),
          cvPoint(region.x + region.width - 1, 
            region.y + region.height - 1), 
          CV_RGB(0, 0, 255), 1);

      resultsImage = colourImage;
      cvShowImage(WINDOW_NAME, resultsImage);
      cvWaitKey(5);
      //int c = cvWaitKey(index != argc - 1 ? 100 : 0);
      //int c = cvWaitKey(0);
    }
    cvReleaseImage(&image);
    image_mutex.unlock();
  }
  // free classifier and results
  delete classifier;
  if (resultsImage != NULL)
    cvReleaseImage(&resultsImage);
  cvDestroyAllWindows();
  ros::fini();
}

bool extendRectInOneDimension(int &loc, int &size,
			      int new_size, int max_size) {
  if (new_size > max_size)
    return false; // probably nothing there anyway, too big

  int new_loc  = loc - (new_size - size) / 2;

  // fix boundary conditions if any
  if (new_loc < 0) {
    new_loc = 0;
  } else if (new_loc + new_size > max_size) {
    new_loc = max_size - new_size;
  }

  size = new_size;
  loc = new_loc;

  return true;
}

bool checkRectangleValid(CvRect &r, int image_width, int image_height) {
  if (r.x < 0) return false;
  if (r.y < 0) return false;
  if (r.x + r.width - 1 > image_width - 1) return false;
  if (r.y + r.height - 1 > image_height - 1) return false;
  return true;
}

// Will discard the rectangles that are too different from the desired
// height/width ratio, and expand the rest so the ratio is perfect
void standardizeRectangles(vector<CvRect> &rects,
			   int classifier_width, int classifier_height,
			   int image_width, int image_height) {

  double desired_ratio = (double) classifier_width / classifier_height;
  double deviation = 2.0;

  vector<CvRect> rectsToUse;

  for (unsigned i = 0; i < rects.size(); i++) {
    CvRect rect = rects[i];
    
    double curr_ratio = (double)rect.width / rect.height;

    if ((desired_ratio / deviation <= curr_ratio) && 
	(curr_ratio <= desired_ratio * deviation)) {
      
      // work on extending this rectangle
      if (curr_ratio < desired_ratio) {
	
	// extend the width
	int new_width = (int)(desired_ratio * rect.height);

	if (!extendRectInOneDimension(rect.x, rect.width,
				      new_width, image_width))
	  continue;
      }

      else if (curr_ratio > desired_ratio) {
	
	// extend the height
	int new_height = (int)(rect.width / desired_ratio);

	if (!extendRectInOneDimension(rect.y, rect.height,
				      new_height, image_height))
	  continue;
      }

      if(!checkRectangleValid(rect, image_width, image_height)) {
	cerr<< "Improper rectangle: " << rect.x << " " << rect.y << " " << rect.width << " " << rect.height << endl;
      }
      rectsToUse.push_back(rect);
    }
  }

  rects = rectsToUse;

}

svlObject2dFrame classifyImage(IplImage *image,
    svlObjectDetector *classifier, vector<CvRect>& rects)
{
    svlObject2dFrame objects;
    double scale = 1.0;
    double scale_step = 1.2;
    //    int x_step = 4;
    //int y_step = 4;
    int x, y;

    cerr << "Analyzing the given rectangles" << endl;

    // first, discard all that have an inappropriate ratio, and expand the others
    // to have the same proportions as expected by the classifier
    standardizeRectangles(rects, classifier->width(), classifier->height(), image->width, image->height);
    
    if(rects.size() == 0) return objects;
    
    // If the caller gave us a rectangle list, then we will sort that list
    // so that the tall rectangles are first.
    sort( rects.begin(), rects.end(), rect_is_taller );

    IplImage *scaledImage = cvCloneImage(image);
    while((scaledImage->width > (int)classifier->width()) &&
	(scaledImage->height > (int)classifier->height())) {
	
	// create sliding window locations
	// These are the upper left hand corners of the corresponding
	// window locations.
	vector<CvPoint> locations;

	// in this case, just use locations which seem to correspond to rectangles
	// which are about the "correct" size.
	double sqrt_step = sqrt( scale_step );
	int i = 0;
	for( vector<CvRect>::iterator it=rects.begin(); it!=rects.end(); ++it, ++i ) {
	  double Rh_over_Splus  = it->height / ( scale * sqrt_step );
	  double Rh_over_Sminus = it->height / ( scale / sqrt_step );
	  
	  // double Rh_over_Splus = it->height / (scale);                // at current level
	  //double Rh_over_Sminus = it->height / (scale / scale_step);  // at previous level

	  // FIX: The idea is to discard windows which are not even close to the
	  // correct ratio up front, then extend the rest to be perfectly sized,
	  // so now can just check the size without worrying about the ratio

	  //if ((Rh_over_Splus <= classifier->width()) &&
	  //   (classifier->width() < Rh_over_Sminus)) {
	  if ((Rh_over_Splus <= classifier->height()) &&
	      (classifier->height() < Rh_over_Sminus)) {
	    x = min((int)(it->x / scale), scaledImage->width - classifier->width());
	    y = min((int)(it->y / scale), scaledImage->height - classifier->height());

	    if (x < 0 || y < 0) {
	      cerr << "Improper location: " << x << " " << y
		   << " when initially " << (int)it->x/scale << " " << (int)it->y/scale
		   << ", classifier is " << classifier->width() << " x " << classifier->height()
		   << " and image is " << scaledImage->width << " x " << scaledImage->height << endl;
	      assert(false);
	    }

	    locations.push_back( cvPoint(x, y) );
	  }
	}
 
    
	// run classifier
	svlObject2dFrame newObjects = classifier->classifyImage(scaledImage, locations);
	for (unsigned i = 0; i < newObjects.size(); i++) {
	  newObjects[i].x *= scale;
	  newObjects[i].y *= scale;
	  newObjects[i].w *= scale;
	  newObjects[i].h *= scale;
	  objects.push_back(newObjects[i]);
	}
	
	// scale down image
	scale *= scale_step;
	cvReleaseImage(&scaledImage);
	scaledImage = cvCreateImage(cvSize((int)(image->width / scale), (int)(image->height / scale)),
				    IPL_DEPTH_8U, 1);
	cvResize(image, scaledImage);
    }
    
    cvReleaseImage(&scaledImage);

    return objects;
}

// Here is some really dodgy code to read a file full of 
// rectangles.  It expects every line to be either blank
// or to have a rectangle of the form:
//   [image number] [x] [y] [width] [height]
// Every rectangle found gets dumped into the vector of
// rectangles provided by the caller.
//

bool read_rects_from_file(vector<CvRect>& rects,
			  char* filename)
{
  FILE* fp = fopen( filename, "r" );

  if( fp==NULL ) return false;
  
  char buf[512];
  CvRect rect;
  char* p = NULL;
  char delim[] = "\t\n ";
  while( fgets( buf, 512, fp ) != NULL ) {
    if( strlen( buf ) < 2 ) continue;
    p = strtok( buf, delim );  
    int layer = atoi( p );
    p = strtok( NULL, delim ); 
    rect.x = atoi( p );
    p = strtok( NULL, delim ); 
    rect.y = atoi( p );
    p = strtok( NULL, delim ); 
    rect.width = atoi( p );
    p = strtok( NULL, delim ); 
    rect.height = atoi( p );
    
    if (layer < 0 || layer >= 1) {
      cerr << "Improper rectangle found: "
	   << layer << " "
	   << rect.x << " "
	   << rect.y << " "
	   << rect.width << " "
	   << rect.height << " " << endl;
      continue;
    }
    
    rects.push_back( rect );
  }
  
  return true;
}

int getIndexFromFilename(const char *filename) {
  string name(filename);
  unsigned index = name.find_last_of("/");
  if (index != string::npos) {
    filename += index + 1;
  }

  int size = string(filename).size();
  for (int i = 0; i < size; i++, filename++) {
    if (isdigit(filename[0])) {
      return atoi(filename);
    }
  }
  cerr << "ERROR: " << filename << " has no index" << endl;
  return -1;
}


