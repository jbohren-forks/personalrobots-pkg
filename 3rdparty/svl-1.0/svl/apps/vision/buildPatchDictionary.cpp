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
** FILENAME:    buildPatchDictionary.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
**  Build patch dictionary for object recognition by taking random snips from
**  all images in a given directory.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

#include <sys/types.h>
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "win32/dirent.h"
#else
#include <dirent.h>
#endif

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

#define WINDOW_NAME "buildPatchDictionary"

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./buildPatchDictionary [OPTIONS] <dir> <width> <height>" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -debug             :: debug mode (visualize dictionary)" << endl
	 << "  -i <filename>      :: input dictionary file" << endl
	 << "  -o <filename>      :: output dictionary file" << endl
	 << "  -n <num>           :: number of patches per image (default: 10)" << endl
         << "  -max_size <num>    :: maximum size of dictionary" << endl
	 << "  -ch <n> <c1>..<cn> :: n input channels, along with types for each of them"<< endl
	 << "  -dmsize <w> <h>    :: dimensions of the depth map"<< endl
	 << "  -imageExt <ext>    :: extension of the image data"<< endl
	 << "  -depthMapExt <ext> :: extension of the depth map data if any"<< endl
	 << endl;
}

int main(int argc, char *argv[])
{
    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 3;

    //Default values for the channels (transforms of the raw input data)
    vector<string> channelTypes;
    channelTypes.push_back("INTENSITY");
    channelTypes.push_back("EDGE");
    
    //Maps command line channel symbols to patch type enum
    map<string, svlPatchDefinitionType> patchTypeToEnum;	
    patchTypeToEnum["INTENSITY"] = SVL_INTENSITY_PATCH;
    patchTypeToEnum["EDGE"] = SVL_INTENSITY_PATCH;
    patchTypeToEnum["DEPTH"] = SVL_DEPTH_PATCH;
    	
    const char *imageDir = NULL;
    unsigned width;
    unsigned height;

    bool bDebug = false;
    const char *inputFilename = NULL;
    const char *outputFilename = NULL;
    int numPatchesPerImage = 10;
    unsigned maxSize = numeric_limits<unsigned>::max();
    int nChannels = 2;
    int depthMapWidth = -1;
    int depthMapHeight = -1;
    string imageExt = string(".jpg");
    string depthMapExt = string(".reconstructed.z.txt");
    
    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
	if (!strcmp(*args, "-debug")) {
	    bDebug = true;
	}else if (!strcmp(*args, "-ch")) {
            channelTypes.clear();
            nChannels = atoi(*(++args)); argc--;

            //Read all the channel types
            for( int i=0 ; i<nChannels ; i++) {
                channelTypes.push_back( string(*(++args)));
                argc--;
            }
	} else if (!strcmp(*args, "-i")) {
	    inputFilename = *(++args); argc--;
	} else if (!strcmp(*args, "-o")) {
	    outputFilename = *(++args); argc--;
	} else if (!strcmp(*args, "-n")) {
	    numPatchesPerImage = atoi(*(++args)); argc--;
        } else if (!strcmp(*args, "-dmsize")) {
            depthMapWidth = atoi(*(++args)); argc--;
            depthMapHeight = atoi(*(++args)); argc--;
        } else if (!strcmp(*args, "-imageExt")) {
	    imageExt = string(*(++args)); argc--;
	} else if (!strcmp(*args, "-depthMapExt")) {
            depthMapExt = string(*(++args)); argc--;
	} else if (!strcmp(*args, "-max_size")) {
	    maxSize = atoi(*(++args)); argc--;
	} else {
            SVL_LOG(SVL_LOG_FATAL, "unrecognized option " << *args);
	}
	args++;
    }

    if (argc != NUM_REQUIRED_PARAMETERS) {
	usage();
	return -1;
    }

    imageDir = args[0];
    width = atoi(args[1]);
    height = atoi(args[2]);

    if( depthMapWidth == -1 )
        depthMapWidth = width;
    if( depthMapHeight == -1 )
        depthMapHeight = height;
    
    
    //Maps patch type enums to respetive input data extensions
    map<svlPatchDefinitionType,string> enumToExtension;	
    enumToExtension[SVL_INTENSITY_PATCH] = imageExt;
    enumToExtension[SVL_DEPTH_PATCH] = depthMapExt;
    
    svlSoftEdgeMap softEdgeCalculator;

    // read and resize images
    vector<vector<IplImage *> > images;

    // get sorted list of files
    string firstChannelExtension = (*(enumToExtension.find( 
        (*(patchTypeToEnum.find(channelTypes[0]))).second ))).second;
    vector<string> baseNames = svlDirectoryListing(imageDir, 
        firstChannelExtension.c_str(), false, false);

    // image objects for holding raw input data, which are transformed
    // into channels
    IplImage *imageData = NULL;
    IplImage *depthData = NULL;
    
    for (vector<string>::const_iterator it = baseNames.begin(); it != baseNames.end(); ++it) 
    {
        SVL_LOG(SVL_LOG_MESSAGE, "Adding " << it->c_str() << " to image database...");
        images.push_back(vector<IplImage *>());
            
        // load images for each of the channels
        for (int i = 0; i < nChannels; i++) {
            // Whether the input data has to be loaded from image
            // or has been peviously loaded
            int loadFromDisk = 1;
                
            IplImage *img = NULL;
                
            //Map to enumeration
            svlPatchDefinitionType patchEnumType = (*(patchTypeToEnum.find(channelTypes[i]))).second;
                
            if( patchEnumType == SVL_INTENSITY_PATCH ) {
                if( imageData != NULL ) {
                    img = cvCloneImage(imageData);
                    loadFromDisk = 0;
                }                    
            } else if (patchEnumType == SVL_DEPTH_PATCH ) {
                if( depthData != NULL) {
                    img = cvCloneImage(depthData);
                    loadFromDisk = 0;
                }
            } else SVL_LOG(SVL_LOG_FATAL, "Unknown channel type provided" );
                
            //If the raw input data for this channel needs to be loaded from disk
            if ( loadFromDisk ) 
            {
                string imageFilename = string(imageDir) + string("/") + (*it) +
                    (*(enumToExtension.find(patchEnumType))).second;
                        
                if( patchEnumType == SVL_INTENSITY_PATCH ) {
                    img = cvLoadImage(imageFilename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
                    
                    if( imageData == NULL )
                        imageData = cvCloneImage(img);
                } else if( patchEnumType == SVL_DEPTH_PATCH ) {						
                    img = readMatrixAsIplImage(imageFilename.c_str(), depthMapWidth,depthMapHeight);
                    
                    if(depthData == NULL )
                        depthData = cvCloneImage(img);
                }
                
            }
                
            if (img == NULL) 
            {
                SVL_LOG(SVL_LOG_MESSAGE, "...failed");
            } 
            else 
            {
                if ((img->width != (int)width) || (img->height != (int)height)) 
                {
                    resizeInPlace(&img, height, width);
                }
                
                //Handle transform of the raw input data
                if (channelTypes[i] == string("EDGE")) 
                {
                    images.back().push_back(cvCloneImage(softEdgeCalculator.processImage(img)));
                    cvReleaseImage(&img);
                } else {
                    images.back().push_back(img);
                }
            }
        } //channels loop
            
            
        if( imageData != NULL )
            cvReleaseImage(&imageData);
        if( depthData != NULL )
            cvReleaseImage(&depthData);
    }

    SVL_LOG(SVL_LOG_MESSAGE, "...read " << images.size() << " images from " << imageDir);

    // build dictionary
    svlPatchDictionary dictionary(width, height);

    if (inputFilename != NULL) {
        SVL_LOG(SVL_LOG_MESSAGE, "Reading dictionary from " << inputFilename);
        dictionary.read(inputFilename);
        SVL_LOG(SVL_LOG_MESSAGE, "..." << dictionary.numEntries() << " entries read");
        assert((dictionary.windowWidth() == width) && (dictionary.windowHeight() == height));
        
        if (dictionary.numEntries() > maxSize) {
            dictionary.truncate(maxSize);
        }
    }
    
    SVL_LOG(SVL_LOG_MESSAGE, "Building dictionary...");

    vector<svlPatchDefinitionType> patchTypes;

    //Store svl enumerations for each of the channels
    for( int i=0 ; i<nChannels ; i++) {
        //Map to enumeration
        svlPatchDefinitionType patchEnumType = (*(patchTypeToEnum.find(channelTypes[i]))).second;
        
        patchTypes.push_back(patchEnumType);
    }
    

    dictionary.buildDictionary(images, patchTypes, numPatchesPerImage);
    SVL_LOG(SVL_LOG_MESSAGE, "..." << dictionary.numEntries() << " entries");
    
    if (dictionary.numEntries() > maxSize) {
        dictionary.truncate(maxSize);
        SVL_LOG(SVL_LOG_MESSAGE, "... truncated to " << dictionary.numEntries() << " entries");
    }

    SVL_LOG(SVL_LOG_MESSAGE, "...done");

    if (outputFilename != NULL) {
        SVL_LOG(SVL_LOG_MESSAGE, "Writing dictionary to " << outputFilename);
        dictionary.write(outputFilename);
    }

    if (bDebug) {
        cvNamedWindow(WINDOW_NAME, 1);
        IplImage *dicImages = dictionary.visualizeDictionary();
        cvShowImage(WINDOW_NAME, dicImages);
        cvWaitKey(0);
        cvReleaseImage(&dicImages);
        cvDestroyWindow(WINDOW_NAME);
    }

    // free memory
    for (unsigned i = 0; i < images.size(); i++) {
        for (unsigned j = 0; j < images[i].size(); j++) {
            cvReleaseImage(&images[i][j]);
        }
    }

    return 0;
}
