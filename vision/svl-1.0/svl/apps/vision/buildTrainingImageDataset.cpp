/****************************************************************************
** STAIR VISION LIBRARY
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
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
******************************************************************************
** FILENAME:    buildTrainingImageDataset.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Constructs a database of positive and negative training images for
**  training an object detector.
**
*****************************************************************************/

#include <cstdlib>
#include <string>
#include <limits>
#include <iomanip>
#include <map>

// OpenCV library
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

// stair vision library
#include "svlBase.h"
#include "svlVision.h"

using namespace std;

// randRect -------------------------------------------------------------------
// Given a base size and frame size, this returns a rectangle in the frame that
// is sampled from the distribution consistent with what the classifier will
// see in the sliding windows (essentially, more small, fewer large images).

// TODO: use in main

// TODO: aspect ratio input is redundant, given base size.  Which is better for
// caller?
CvRect randRect(double aspectRatio, CvSize base, CvSize frame, CvRNG *rngState)
{
  const double SEL_FACTOR = 100000000.0;
  const double SEL_FACTOR_SQRT = 10000.0;
  int minSel, maxSel;
  CvRect rect;

  // TODO: minSel and maxSel calculation can be done once, given the frame
  // size and base size
  if (frame.width / frame.height > aspectRatio) {
    // Frame is wider than aspect, rectangle is constrained by height
    minSel = (int) ceil(SEL_FACTOR / pow(frame.height, 2.0));
    maxSel = (int) floor(SEL_FACTOR / pow(base.height, 2.0));

    float sel = (float) (cvRandInt(rngState) % (maxSel - minSel) + minSel);
    rect.height = (int) (SEL_FACTOR_SQRT / sqrt(sel));
    rect.width = (int) (rect.height * aspectRatio);
  } else {
    // Frame is thinner than aspect, rectangle is constrained by width
    minSel = (int) ceil(SEL_FACTOR / pow(frame.width, 2.0));
    maxSel = (int) floor(SEL_FACTOR / pow(base.width, 2.0));

    float sel = (float) (cvRandInt(rngState) % (maxSel - minSel) + minSel);
    rect.width = (int) (SEL_FACTOR_SQRT / sqrt(sel));
    rect.height = (int) (rect.width / aspectRatio);
  }

  rect.x = cvRandInt(rngState) % (frame.width - rect.width);
  rect.y = cvRandInt(rngState) % (frame.height - rect.height);

  return rect;
}

// Main ----------------------------------------------------------------------

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./buildTrainingImageDataset [OPTIONS] <imageSequence> <objectLabels>" << endl;
    cerr << "OPTIONS:" << endl
         << "  -object <name>      :: object extract (default: all)" << endl
         << "  -skipPositives      :: Does not write positive objects to file"<<endl
         << "  -negName <name>     :: Name for the folder which contains the negatives (default: negative)"<< endl
         << "  -negHeight <height> :: Used to fix the size of the negative patch in combination with the aspect ratio (default: 32)"<<endl  
         << "  -aspectRatio <num>  :: aspect ratio (w/h) for clipping negative examples (default : 1)" << endl
         << "  -resize <w> <h>     :: resize images to <w>-by-<h>" << endl
         << "  -includeFlipped     :: include horizontally flipped positive examples" << endl
         << "  -negatives <n>      :: number of negatives per image (default: 100)" << endl
         << "  -fponly             :: processes only the false positives from the given detections and does not add any new negatives (default: inactive)" << endl
         << "  -detections <file>  :: include false-positive detections from <file>" << endl
         << "  -createDirs         :: Creates sub directories for objectc names & one for negative images" << endl
         << "  -directory <dir>    :: output base directory " << endl
         << "  -posPrefix <string> :: prefix for the final names of the positive images (default : blank)" << endl
         << "  -negPrefix <string> :: prefix for the final names of the negative images (default : blank)" << endl
         << "  -imageExt <ext>     :: extension of the image data"<< endl
         << "  -depthMapExt <ext>  :: extension of the depth map data"<< endl
         << "  -depthChannel       :: indicates the presence of a depth channel for all images in the image seq"<<endl
         << "  -dmsize <w> <h>     :: dimensions of the depth map"<< endl
         << "  -v                  :: verbose (default: false)" << endl
         << endl;
}

int main(int argc, char *argv[])
{
    string imageExt = string(".jpg");
    string depthMapExt = string(".reconstructed.z.txt");

    CvRNG rngState(0xffffffff);
    const int minWidth = 32;
    const int minHeight = 32;
    int isDepthChannel = 0;
    int depthMapWidth = 640;
    int depthMapHeight = 480;
    int negHeight = 32;
    int createDirs = 1;
    int skipPositives = 0;

    // read command line parameters
    const int NUM_REQUIRED_PARAMETERS = 2;
    
    const char *imageSequenceFile = NULL;
    const char *objectLabelsFile = NULL;
    const char *negName = "negative";
    const char *posPrefix = NULL;
    const char *negPrefix = NULL;

    const char *objectName = NULL;
    int resizeWidth = -1;
    int resizeHeight = -1;
    int fponly = 0;
    bool bIncludeFlipped = false;
    int numNegatives = 100;
    double negAspectRatio = 1.0;
    const char *detectionsFilename = NULL;
    const char *baseDirectory = ".";
    svlLogger::setLogLevel(SVL_LOG_MESSAGE);
    
    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        if (!strcmp(*args, "-object")) {
            objectName = *(++args);
            argc--;
        } else if (!strcmp(*args, "-aspectRatio")) {
            negAspectRatio = atof(*(++args));
            argc--;
        } 
        else if (!strcmp(*args, "-skipPositives")) {
            skipPositives = 1;
        }
        else if (!strcmp(*args, "-fponly")) {
            fponly = 1;
        }
        else if (!strcmp(*args, "-resize")) {
            resizeWidth = atoi(*(++args));
            resizeHeight = atoi(*(++args));
            argc -= 2;
        } else if (!strcmp(*args, "-negatives")) {
            numNegatives = atoi(*(++args));
            argc--;
        } else if (!strcmp(*args, "-includeFlipped")) {
            bIncludeFlipped = true;
        } else if (!strcmp(*args, "-negHeight")) {
            negHeight = atoi(*(++args));
            argc--;
        }
        else if (!strcmp(*args, "-negName")) 
        {
            negName = *(++args);
            argc--;
        } 
        else if (!strcmp(*args, "-posPrefix")) 
        {
            posPrefix = *(++args);
            argc--;
        } 
        else if (!strcmp(*args, "-negPrefix")) 
        {
            negPrefix = *(++args);
            argc--;
        }
        else if (!strcmp(*args, "-depthChannel")) {
           isDepthChannel = 1;
        }
         else if (!strcmp(*args, "-createDirs")) {
             createDirs = 1;
        }
        else if (!strcmp(*args, "-detections")) {
            detectionsFilename = *(++args);
            argc--;
        } else if (!strcmp(*args, "-directory")) {
            baseDirectory = *(++args);
            argc--;
        } else if (!strcmp(*args, "-imageExt")) {
	    imageExt = string(*(++args)); argc--;
	    } else if (!strcmp(*args, "-depthMapExt")) {
            depthMapExt = string(*(++args)); argc--;
	    }
        else if (!strcmp(*args, "-dmsize")) 
	    {
	    	depthMapWidth = atoi(*(++args)); argc--;
	    	depthMapHeight = atoi(*(++args)); argc--;
        }
        else if (!strcmp(*args, "-v")) {
            svlLogger::setLogLevel(SVL_LOG_VERBOSE);
	} else {
	    SVL_LOG(SVL_LOG_FATAL, "unrecognized option " << *args);
	}
	args++;
    }
    
    if (argc != NUM_REQUIRED_PARAMETERS) 
    {
    	usage();
    	return -1;
    }

    imageSequenceFile = args[0];
    objectLabelsFile = args[1];

    // image sequence and object labels
    SVL_LOG(SVL_LOG_VERBOSE, "Reading image sequence from " << imageSequenceFile << "...");
    svlImageSequence imageSequence;
    imageSequence.load(imageSequenceFile);
    SVL_LOG(SVL_LOG_VERBOSE, "..." << imageSequence.size() << " images read");
    
    SVL_LOG(SVL_LOG_VERBOSE, "Reading object labels from " << objectLabelsFile << "...");
    svlObject2dSequence objectLabels;
    readObject2dSequence(objectLabelsFile, objectLabels);
    SVL_LOG(SVL_LOG_VERBOSE, "..." << objectLabels.size() << " labeled frames read");

    svlObject2dSequence objectDetections;

    if (detectionsFilename != NULL) 
    {
        SVL_LOG(SVL_LOG_VERBOSE, "Reading detections from " << detectionsFilename << "...");
        readObject2dSequence(detectionsFilename, objectDetections);
        SVL_LOG(SVL_LOG_VERBOSE, "..." << objectDetections.size() << " labeled frames read");
	    int total = countObjects(objectDetections);
	    int removed = nonMaximalSuppression(objectDetections, 0.25, 0.25, 0.64);
	    SVL_LOG(SVL_LOG_VERBOSE, "..." << removed << " out of " << total << " non-maximal objects suppressed");
    }

    // counters for image names
    map<string, int> positiveCounts;
    int negativeCounts = 0;


    if (createDirs && !fponly) {
        string dirName = string(baseDirectory) + string("/") + string(negName);
        svlCreateDirectory(dirName.c_str());
    }

    // load images and snip regions
    for (unsigned i = 0; i < imageSequence.size(); i++) 
    {
        
        SVL_LOG(SVL_LOG_VERBOSE, "Processing image " << imageSequence[i] << "...");
        
        if (imageSequence.image(i) == NULL) 
        {
            SVL_LOG(SVL_LOG_WARNING, "could not load image " << imageSequence[i]);
            continue;
        }

        IplImage *image = cvCloneImage(imageSequence.image(i));
        IplImage *depthMap = NULL;

        //Read depth map information if availible
        if( isDepthChannel )
        {
            string filename = imageSequence.directory() + string("/") + imageSequence[i];

            filename.replace(filename.length() - imageExt.size() , imageExt.size(), depthMapExt );
            depthMap = readMatrixAsIplImage(filename.c_str(),depthMapWidth,depthMapHeight);
        }
     
        
        if (image->nChannels != 1) 
        {
            IplImage *tmpImage = greyImage(image);
            cvReleaseImage(&image);
            image = tmpImage;
        }

        string imageId = strBaseName(imageSequence[i]);

        // extract positive examples
        svlObject2dFrame objects;
        if (objectLabels.find(imageId) != objectLabels.end()) 
        {
            objects = objectLabels[imageId];
        }
        if (objectName != NULL) 
        {
            removeNonMatchingObjects(objects, objectName);
        }

        if( !skipPositives )
        {

            for (unsigned j = 0; j < objects.size(); j++) 
            {            
                CvRect r = cvRect((int)objects[j].x, (int)objects[j].y,(int)objects[j].w, (int)objects[j].h);

                IplImage *depthClip = NULL;

                cvSetImageROI(image, r);
                IplImage *imageClip = cvCreateImage(cvSize(r.width, r.height), IPL_DEPTH_8U, 1);
                cvCopy(image, imageClip);            
                cvResetImageROI(image);
                
                if ((resizeWidth > 0) && (resizeHeight > 0)) 
                {
                    resizeInPlace(&imageClip, resizeHeight, resizeWidth);
                }

                if( isDepthChannel )
                {
                    cvSetImageROI(depthMap, r);
                    depthClip = cvCreateImage(cvSize(r.width, r.height), IPL_DEPTH_32F, 1);
                    cvCopy(depthMap, depthClip);            
                    cvResetImageROI(depthMap);
                    
                    if ((resizeWidth > 0) && (resizeHeight > 0)) 
                    {
                        resizeInPlace(&depthClip, resizeHeight, resizeWidth);
                    }

                }

                if (positiveCounts.find(objects[j].name) == positiveCounts.end()) 
                {
                    positiveCounts[objects[j].name] = 0;

                    if (createDirs) {
                        string dirName = string(baseDirectory) + string("/") +
                            objects[j].name;
                        svlCreateDirectory(dirName.c_str());
                    }                    
                }

                int n = positiveCounts[objects[j].name];
                
                string baseFilename = string(baseDirectory) + string("/") +
                    objects[j].name + string("/") ;

                if( posPrefix == NULL )
                   baseFilename += objects[j].name + padString(toString(n), 5);
                else
                   baseFilename += string(posPrefix) + objects[j].name + padString(toString(n), 5);
            
                cvSaveImage((baseFilename + imageExt).c_str(), imageClip);
                if (bIncludeFlipped) {
                    cvFlip(imageClip, NULL, 1);
                    cvSaveImage((baseFilename + string("f") + imageExt).c_str(), imageClip);
                }
                cvReleaseImage(&imageClip);

                if( isDepthChannel )
                {
                     //*********************** Normalize depth based on median*****************************
					
                     vector<double> depths;
                    for( int y=0 ; y<depthClip->height ; y++)
                        {
                            for( int x=0 ; x<depthClip->width ; x++)
                                {
                                    depths.push_back( CV_IMAGE_ELEM(depthClip,float,y,x) );
                                }
                        }
                    
                    sort(depths.begin() , depths.end());
                    
                    double median = depths[ depths.size() / 2];
                    
                    for( int y=0 ; y<depthClip->height ; y++)
                        {
                            for( int x=0 ; x<depthClip->width ; x++)
                                {
                                    CV_IMAGE_ELEM(depthClip,float,y,x) = CV_IMAGE_ELEM(depthClip,float,y,x) - median;
                                }
                        }
                    
                    
                    //*******************************************************************************
                    
                    writeMatrixAsIplImage(depthClip,
                        (baseFilename + depthMapExt).c_str());
                    if (bIncludeFlipped) {
                        cvFlip(depthClip, NULL, 1);
                        writeMatrixAsIplImage(depthClip, 
                            (baseFilename + string("f") + depthMapExt).c_str());
                    }
                    cvReleaseImage(&depthClip);
                }

                positiveCounts[objects[j].name] = n + 1;
            }//j
        }//if not skip

        // extract negative examples
        svlObject2dFrame detections;
        
        if (objectDetections.find(imageId) != objectDetections.end()) 
        {
            detections = objectDetections[imageId];
            removeGroundTruthObjects(detections, objects, 0.5);
            detections = sortObjects(detections);
        }

        // add random rectangles if no detections
        if (detections.empty() && !fponly)
        {

            // TO DO: check for overlap with a positive
            for (unsigned j = 0; j < (unsigned)numNegatives; j++) 
            {
                CvRect r;
                
                if (negAspectRatio <= 0.0) 
                {
                    r.width = (cvRandInt(&rngState) % (image->width - minWidth)) + minWidth;
                    r.height = (cvRandInt(&rngState) % (image->height - minHeight)) + minHeight;
                } else if (negAspectRatio > 1.0) 
                {
                    if( negHeight == -1 )
                    {
                      r.width = (cvRandInt(&rngState) % (image->width - minWidth)) + minWidth;
                      r.height = (int)(r.width / negAspectRatio);
                    }
                    else
                    {
                        r.height = negHeight;
                        r.width = (int)(negAspectRatio * r.height);
                    }
                } 
                else 
                {
                    if( negHeight == -1 )
                    {
                        r.height = (cvRandInt(&rngState) % (image->height - minHeight)) + minHeight;
                        r.width = (int)(negAspectRatio * r.height);
                    }
                    else
                    {
                        r.height = negHeight;
                        r.width = (int)(negAspectRatio * r.height);
                    }
                }

                //************************** Check for overlap with all ground truth objects in image ************
                
                int flag = 1;

                do
                {

                    flag = 1;

                    r.x = cvRandInt(&rngState) % (image->width - r.width);
                    r.y = cvRandInt(&rngState) % (image->height - r.height);

                    svlObject2d temp(r.x,r.y,r.width,r.height);

                    for(unsigned j=0 ; j<objects.size() ; j++)
                        if( objects[j].overlap ( temp ) != 0 )
						{
							flag = 0;
							break;
                        }
						
                }while(flag==0);
                

                cvSetImageROI(image, r);
                
                IplImage *imageClip = cvCreateImage(cvSize(r.width, r.height), IPL_DEPTH_8U, 1);
                cvCopy(image, imageClip);            
                cvResetImageROI(image);
                
                if ((resizeWidth > 0) && (resizeHeight > 0)) 
                {
                    resizeInPlace(&imageClip, resizeHeight, resizeWidth);
                }
                
                string filename = string(baseDirectory) + string("/") + string(negName) + string("/");

                if( negPrefix == NULL)
                    filename += string("image") + padString(toString(negativeCounts), 5) + imageExt;
                else
                    filename += string(negPrefix) + string("image") + padString(toString(negativeCounts), 5) + imageExt;
                
                cvSaveImage(filename.c_str(), imageClip);
                cvReleaseImage(&imageClip);


                if( isDepthChannel )
                {
                     cvSetImageROI(depthMap, r);
                    
                    IplImage *depthClip = cvCreateImage(cvSize(r.width, r.height), IPL_DEPTH_32F, 1);
                    cvCopy(depthMap, depthClip);            
                    cvResetImageROI(depthMap);
                    
                    if ((resizeWidth > 0) && (resizeHeight > 0)) 
                    {
                        resizeInPlace(&depthClip, resizeHeight, resizeWidth);
                    }
                    
                    string depthFilename = string(baseDirectory) + string("/") + string(negName) + string("/");

                    if( negPrefix == NULL )
                        depthFilename += string("image") + padString(toString(negativeCounts), 5) + depthMapExt;
                    else
                        depthFilename += string(negPrefix) + string("image") + padString(toString(negativeCounts), 5) + depthMapExt;
                        
                    

                    //*********************** Normalize depth based on median*****************************
					
					vector<double> depths;

                    for( int y=0 ; y<depthClip->height ; y++)
					{
						for( int x=0 ; x<depthClip->width ; x++)
						{
							depths.push_back( CV_IMAGE_ELEM(depthClip,float,y,x) );
						}
					}

					sort(depths.begin() , depths.end());

					double median = depths[ depths.size() / 2];

					for( int y=0 ; y<depthClip->height ; y++)
					{
						for( int x=0 ; x<depthClip->width ; x++)
						{
							CV_IMAGE_ELEM(depthClip,float,y,x) = CV_IMAGE_ELEM(depthClip,float,y,x) - median;
						}
					}


					//*******************************************************************************

                    writeMatrixAsIplImage(depthClip,depthFilename.c_str());
                    cvReleaseImage(&depthClip);
                }

                negativeCounts += 1;
            }

        }
        else //if detections are not empty
        {
            // add worst false-positives
            for (unsigned j = 0; j < (unsigned)numNegatives; j++) 
            {
                if (j >= detections.size()) 
                    break;
                
                CvRect r = cvRect((int)detections[j].x, (int)detections[j].y,
                    (int)detections[j].w, (int)detections[j].h);
                
                cvSetImageROI(image, r);

                IplImage *imageClip = cvCreateImage(cvSize(r.width, r.height), IPL_DEPTH_8U, 1);
                cvCopy(image, imageClip);            
                cvResetImageROI(image);
                
                if ((resizeWidth > 0) && (resizeHeight > 0)) 
                {
                    resizeInPlace(&imageClip, resizeHeight, resizeWidth);
                }
                
                string filename = string(baseDirectory) + string("/") + string(negName) + string("/");

                if( negPrefix == NULL )
                    filename += string("image") + padString(toString(negativeCounts), 5) + imageExt;
                else
                    filename += string(negPrefix) + string("image") + padString(toString(negativeCounts), 5) + imageExt;
                
                cvSaveImage(filename.c_str(), imageClip);
                cvReleaseImage(&imageClip);
                
                if( isDepthChannel )
                {
                    
                    cvSetImageROI(depthMap, r);

                    IplImage *depthClip = cvCreateImage(cvSize(r.width, r.height), IPL_DEPTH_32F, 1);
                    cvCopy(depthMap, depthClip);            
                    cvResetImageROI(depthMap);
                    
                    if ((resizeWidth > 0) && (resizeHeight > 0)) 
                    {
                        resizeInPlace(&depthClip, resizeHeight, resizeWidth);
                    }
                    
                    string depthFilename = string(baseDirectory) + string("/") + string(negName)+ string("/");

                    if( negPrefix == NULL )
                        depthFilename += string("image") + padString(toString(negativeCounts), 5) + depthMapExt;
                    else
                        depthFilename += string(negPrefix) + string("image") + padString(toString(negativeCounts), 5) + depthMapExt;

                        

                    //*********************** Normalize depth based on median*****************************
					
					vector<double> depths;

                    for( int y=0 ; y<depthClip->height ; y++)
					{
						for( int x=0 ; x<depthClip->width ; x++)
						{
							depths.push_back( CV_IMAGE_ELEM(depthClip,float,y,x) );
						}
					}

					sort(depths.begin() , depths.end());

					double median = depths[ depths.size() / 2];

					for( int y=0 ; y<depthClip->height ; y++)
					{
						for( int x=0 ; x<depthClip->width ; x++)
						{
							CV_IMAGE_ELEM(depthClip,float,y,x) = CV_IMAGE_ELEM(depthClip,float,y,x) - median;
						}
					}


					//*******************************************************************************
                    
                   writeMatrixAsIplImage(depthClip,depthFilename.c_str());
                   cvReleaseImage(&depthClip);

                }

                negativeCounts += 1;
            }
        }

        cvReleaseImage(&image);

        if( isDepthChannel )
            cvReleaseImage(&depthMap);
    }
    
    SVL_LOG(SVL_LOG_MESSAGE, "Generated dataset with:");
    
    for (map<string, int>::const_iterator it = positiveCounts.begin();it != positiveCounts.end(); ++it) 
    {
        SVL_LOG(SVL_LOG_MESSAGE, it->second << " instances of " << it->first);
    }
    
    SVL_LOG(SVL_LOG_MESSAGE, negativeCounts << " negative examples");

    return 0;
}


