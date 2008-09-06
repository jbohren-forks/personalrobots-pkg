/*****************************************************************************
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
** FILENAME:    classifyImages.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
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

#include "svlBase.h"
#include "svlML.h"
#include "svlVision.h"

using namespace std;

#define WINDOW_NAME "classifyImages"

// main -----------------------------------------------------------------------

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./classifyImages [OPTIONS] [<dictionary>] <model> (<image>* | <imageSequence>)" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -b                    :: only search at base scale" << endl
	 << "  -c <type>             :: classifier type: GABOR, PATCH (default)" << endl
	 << "  -n <object name>      :: name of object" << endl
	 << "  -o <filename>         :: output to file" << endl
	 << "  -profile              :: turn on SVL code profiling" << endl
	 << "  -r <x> <y> <w> <h>    :: scan subregion of image" << endl
	 << "  -R <filename>         :: only scan rectangles in file (<x> <y> <w> <h>)" <<endl
	 << "  -s <w> <h>            :: resize image to <w>-by-<h> before scanning" << endl
	 << "  -t <threshold>        :: classifier threshold (default: 0.5)" << endl
         << "  -v                    :: verbose" << endl
	 << "  -x                    :: display image and matches" << endl
	 << "  -ch n <string1>..<stringn> :: n input channels, along with types for each of them"<< endl
	 << "  -dmsize <width> <height> :: dimensions of the depth map"<< endl
	 << "  -imageExt <string> :: extension of the image data"<< endl
	 << "  -depthMapExt <string> :: extension of the depth map data if any"<< endl
	 << endl;
}

int main(int argc, char *argv[])
{		
    svlLogger::setLogLevel(SVL_LOG_MESSAGE);
    const unsigned MAX_RESULTS_TO_SHOW = 16;
    const unsigned MAX_WIDTH = 1024;

	//Default values for the channels (transforms of the raw input data)
	vector<string> channelTypes;
	channelTypes.push_back("INTENSITY");
	channelTypes.push_back("EDGE");
	
	//Maps command line channel symbols to patch type enum
	map<string,svlPatchDefinitionType> patchTypeToEnum;	
	patchTypeToEnum["INTENSITY"] = SVL_INTENSITY_PATCH;
	patchTypeToEnum["EDGE"] = SVL_INTENSITY_PATCH;
	patchTypeToEnum["DEPTH"] = SVL_DEPTH_PATCH;

    // read commandline parameters
    const char *dictionaryFilename = NULL;
    const char *classifierFilename = NULL;
    const char *outputFilename = NULL;
    const char *objectName = "[unknown]";
    const char *classifierType = "patch";
    double threshold = 0.5;
    bool bDisplayImage = false;
    int width = -1;
    int height = -1;
    bool bBaseScaleOnly = false;
    CvRect subRegion = cvRect(0, 0, 0, 0);
    char *regionsFile = NULL;
	int nChannels = 2;
	int depthMapWidth = 640;
	int depthMapHeight = 480;
	string imageExt = string(".jpg");
	string depthMapExt = string(".reconstructed.z.txt");

    char **args = argv + 1;
    while (--argc) {
	if (!strcmp(*args, "-n")) {
	    objectName = *(++args);              argc--;
	} else if (!strcmp(*args, "-b")) {
	    bBaseScaleOnly = true;
	} 
	else if (!strcmp(*args, "-ch")) 
	{
	   channelTypes.clear();
	   nChannels = atoi(*(++args)); argc--;

		//Read all the channel types
		for( int i=0 ; i<nChannels ; i++)
		{
			channelTypes.push_back( string(*(++args)));
			argc--;
		}

	} 
	else if (!strcmp(*args, "-dmsize")) 
	{
		depthMapWidth = atoi(*(++args)); argc--;
		depthMapHeight = atoi(*(++args)); argc--;
    }
	else if (!strcmp(*args, "-imageExt")) {
	    imageExt = string(*(++args)); argc--;
	}
	else if (!strcmp(*args, "-depthMapExt")) {
		depthMapExt = string(*(++args)); argc--;
	}
	else if (!strcmp(*args, "-c")) {	    
	    classifierType = *(++args);          argc--;
	} else if (!strcmp(*args, "-o")) {
	    outputFilename = *(++args);          argc--;
	} else if (!strcmp(*args, "-profile")) {
	    svlCodeProfiler::enabled = true;
            svlCodeProfiler::tic(svlCodeProfiler::getHandle("main"));
	} else if (!strcmp(*args, "-r")) {
	    subRegion.x = atoi(*(++args));       argc--;
	    subRegion.y = atoi(*(++args));       argc--;
	    subRegion.width = atoi(*(++args));   argc--;
	    subRegion.height = atoi(*(++args));  argc--;
	} else if (!strcmp(*args, "-s")) {
	    width = atoi(*(++args));             argc--;
	    height = atoi(*(++args));            argc--;
	} else if (!strcmp(*args, "-t")) {
	    threshold = atof(*(++args));         argc--;
	} else if (!strcmp(*args, "-R")) {
            regionsFile = *(++args);             argc--;
        } else if (!strcmp(*args, "-v")) {
            svlLogger::setLogLevel(SVL_LOG_VERBOSE);
	} else if (!strcmp(*args, "-x")) {
	    bDisplayImage = true;
	} else if ((*args)[0] == '-') {
	    SVL_LOG(SVL_LOG_FATAL, "unrecognized option " << *args);
	} else {
	    break;
	}
	args++;
    }

    if (argc < (!strcasecmp(classifierType, "patch") ? 3 : 2)) {
	usage();
	return -1;
    }

	//Maps patch type enums to respetive input data extensions
	map<svlPatchDefinitionType,string> enumToExtension;	
	enumToExtension[SVL_INTENSITY_PATCH] = imageExt;
	enumToExtension[SVL_DEPTH_PATCH] = depthMapExt;


    if (!strcasecmp(classifierType, "patch")) {
	dictionaryFilename = *args++; argc--;
    }
    classifierFilename = *args++; argc--;

    // process image names
    vector<string> imageNames;
    for (int index = 0; index < argc; index++)
	{
        string imageFilename = string(args[index]);
        if (strExtension(imageFilename).compare("xml") == 0) {
            svlImageSequence sequence;
            sequence.load(imageFilename.c_str());
            for (unsigned i = 0; i < sequence.size(); i++) {
                imageNames.push_back(sequence.directory() + string("/") + sequence[i]);
            }
        } else {
            imageNames.push_back(imageFilename);
        }
	}

    // read regions for processing
    vector<CvRect> regions;        
    if (regionsFile) {
        SVL_LOG(SVL_LOG_VERBOSE, "Reading regions from " << regionsFile << "...");
        ifstream ifs(regionsFile);
        assert(!ifs.fail());
        CvRect r;
        while (!ifs.fail()) {
            ifs >> r.x >> r.y >> r.width >> r.height;
            if (ifs.fail()) break;
            regions.push_back(r);
        }
        SVL_LOG(SVL_LOG_VERBOSE, "..." << regions.size() << " regions read");
    }

    // initialize object detector
    svlSlidingWindowDetector *classifier = NULL;
    if (!strcasecmp(classifierType, "patch")) {
	classifier = new svlPatchBasedObjectDetector(objectName);
	SVL_LOG(SVL_LOG_VERBOSE, "Loading dictionary " << dictionaryFilename << "...");
	((svlPatchBasedObjectDetector *)classifier)->readDictionary(dictionaryFilename);
    }
    assert(classifier != NULL);
    SVL_LOG(SVL_LOG_VERBOSE, "Loading classifier " << classifierFilename << "...");
    classifier->readModel(classifierFilename);
    classifier->setThreshold(threshold);

    // run classifier on all supplied images
    vector<IplImage *> annotatedImages;
    annotatedImages.resize(((imageNames.size() < MAX_RESULTS_TO_SHOW) ?
            imageNames.size() : MAX_RESULTS_TO_SHOW), NULL);
    if (sqrt((double)annotatedImages.size()) * width > MAX_WIDTH) {
	annotatedImages.resize(1);
    }
    IplImage *resultsImage = NULL;

    if (bDisplayImage) {
	cvNamedWindow(WINDOW_NAME, 0);
    }

    ofstream ofs;
    if (outputFilename) {
        ofs.open(outputFilename);
        assert(!ofs.fail());
        ofs << "<Object2dSequence>" << endl;
    }

    svlSoftEdgeMap softEdgeMap;

	//Image objects for holding raw input data, which are transformed into channels
	IplImage *imageData = NULL;
	IplImage *depthData = NULL;

	string firstChannelExtension = ".jpg";//(*(enumToExtension.find( (*(patchTypeToEnum.find(channelTypes[0]))).second ))).second;
    SVL_LOG(SVL_LOG_MESSAGE, "Running classifier on " << imageNames.size() << " images/sequences...");

    for (unsigned index = 0; index < imageNames.size(); index++) 
	{
        SVL_LOG(SVL_LOG_VERBOSE, "Processing " << imageNames[index] << "...");

        vector<IplImage *> images;

		//Load images from each channel
		for( int i=0 ; i<nChannels ; i++)
		{
			//Whether the input data has to be loaded from image or has been peviously loaded
			int loadFromDisk = 1;

			IplImage *img = NULL;
			
			//Map to enumeration
			svlPatchDefinitionType patchEnumType = (*(patchTypeToEnum.find(channelTypes[i]))).second;

			if( patchEnumType == SVL_INTENSITY_PATCH )
			{
				if( imageData != NULL )
				{
					img = cvCloneImage(imageData);
					loadFromDisk = 0;
				}
									
			}
			else if(patchEnumType == SVL_DEPTH_PATCH )
			{
				if( depthData != NULL)
				{
					img = cvCloneImage(depthData);
					loadFromDisk = 0;
				}
			}
			else
				SVL_LOG(SVL_LOG_FATAL, "Unknown channel type provided" );

			//If the raw input data for this channel needs to be loaded from disk
			if( loadFromDisk )
			{
				string imageFilename = imageNames[index];

				//Form filename for the rest of the channels by modifying extensions
				//if( i !=0 )
				//{
					imageFilename.replace(imageFilename.length() - firstChannelExtension.size() , firstChannelExtension.size() 
						,  (*(enumToExtension.find(patchEnumType))).second);
				//}

				if( patchEnumType == SVL_INTENSITY_PATCH )
				{
					img = cvLoadImage(imageFilename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
					if( imageData == NULL )
						imageData = cvCloneImage(img);

				}
				else if( patchEnumType == SVL_DEPTH_PATCH )
				{
					cout<<"\n "<<firstChannelExtension.c_str()<<" "<<channelTypes[i];
					cout<<"\n "<<imageFilename.c_str();
					img = readMatrixAsIplImage(imageFilename.c_str(), depthMapWidth,depthMapHeight);

					if(depthData == NULL )
						depthData = cvCloneImage(img);
				}

			}

			if (img == NULL) {
                            SVL_LOG(SVL_LOG_ERROR, "could not read image \"" << imageNames[index] << "\"");
                            continue;
			} else {
                            // resize image if -s commandline argument given
                            if ((width > 0) && (height > 0)) {
                                SVL_LOG(SVL_LOG_VERBOSE, "Resizing image from " << img->width << " x " 
                                    << img->height << " to " << width << " x " << height << "...");
                                resizeInPlace(&img, height, width);
                            }
                            
                            //Handle transform of the raw input data
                            if (channelTypes[i] == string("EDGE")) {
                                img = cvCloneImage(softEdgeMap.processImage(img));
                                images.push_back(img);
                            } else {
                                images.push_back(img);
                            }
			}

		}//channels loop
        


	// classify image
	svlObject2dFrame objects;
        if (!regions.empty()) {
            SVL_LOG(SVL_LOG_FATAL, "region scanning not implemented yet");
        } else if (bBaseScaleOnly)
		{
            vector<CvPoint> locations;
            if ((subRegion.width > 0) && (subRegion.height > 0)) {
                classifier->createWindowLocations(subRegion.width, subRegion.height, locations);
                for (vector<CvPoint>::iterator it = locations.begin(); it != locations.end(); ++it) {
                    it->x += subRegion.x;
                    it->y += subRegion.y;
                }
            } 
	else 
	{
                classifier->createWindowLocations(images[0]->width, images[0]->height, locations);
        }

            objects = classifier->classifyImage(images, locations);
        } 
		else if ((subRegion.width > 0) && (subRegion.height > 0)) {
            objects = classifier->classifySubImage(images, subRegion);
	} 
		else 
		{
            objects = classifier->classifyImage(images);
			
		}

	// output objects
	if (outputFilename) {
            writeObject2dFrame(ofs, objects, strBaseName(imageNames[index]).c_str());
	}

	if (bDisplayImage) {
	    IplImage *colourImage = cvCreateImage(cvSize(images[0]->width, images[0]->height),
		IPL_DEPTH_8U, 3);
	    cvCvtColor(images[0], colourImage, CV_GRAY2RGB);
	    if (resultsImage != NULL) {		
		cvReleaseImage(&resultsImage);
	    }
            
	    for (unsigned i = 0; i < objects.size(); i++) {
		cvRectangle(colourImage, cvPoint((int)objects[i].x, (int)objects[i].y),
		    cvPoint((int)(objects[i].x + objects[i].w - 1), 
			(int)(objects[i].y + objects[i].h - 1)),
		    CV_RGB(0, 255, 0), 1);
	    }
	    
	    cvRectangle(colourImage, cvPoint(subRegion.x, subRegion.y),
		cvPoint(subRegion.x + subRegion.width - 1, subRegion.y + subRegion.height - 1), 
		CV_RGB(0, 0, 255), 1);
	 
	    if (annotatedImages[index % annotatedImages.size()] != NULL) {
		cvReleaseImage(&annotatedImages[index % annotatedImages.size()]);
	    }
	    annotatedImages[index % annotatedImages.size()] = colourImage;
	    resultsImage = combineImages(annotatedImages);
	    cvShowImage(WINDOW_NAME, resultsImage);
	    int c = cvWaitKey(index != imageNames.size() - 1 ? 100 : 0);
	    //int c = cvWaitKey(0);

	    if (c == (int)'s') {
                stringstream filename;
                filename << "classifyImage" << index << ".jpg";
                cvSaveImage(filename.str().c_str(), resultsImage);
	    }
	}
    
	// free image
        for (unsigned i = 0; i < images.size(); i++) {
            cvReleaseImage(&images[i]);
        }

		if( imageData != NULL )
			cvReleaseImage(&imageData);
		if( depthData != NULL )
			cvReleaseImage(&depthData);

	}//image names in image sequence loop

    if (outputFilename) {
        ofs << "</Object2dSequence>" << endl;
        ofs.close();
    }

    // free classifier and results
    delete classifier;
    if (resultsImage != NULL) {
	cvReleaseImage(&resultsImage);
    }
    for (unsigned i = 0; i < annotatedImages.size(); i++) {
	if (annotatedImages[i] != NULL) {
	    cvReleaseImage(&annotatedImages[i]);
	}
    }
    
    cvDestroyAllWindows();
    svlCodeProfiler::toc(svlCodeProfiler::getHandle("main"));
    svlCodeProfiler::print(cerr);

    return 0;
}
