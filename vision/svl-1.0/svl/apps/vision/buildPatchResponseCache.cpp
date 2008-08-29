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
** FILENAME:    buildPatchResponseCache.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Build a cache of patch responses.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
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
#include "svlDevel.h"

// TODO: remove and only rely on svlThreadPool once threading is stable
#define USE_THREADS

using namespace std;

// Thread args
struct ProcessFileArgs {
  ProcessFileArgs(const char *imageD, string fname, const char *outD,
		  const int nC,
		  const map<string, svlPatchDefinitionType> &ptToEnum,
		  const vector<string> &cTs, const string &firstCExt,
		  const map<svlPatchDefinitionType, string> &enumToExt,
		  const int w, const int h, const int dmW, const int dmH,
		  svlPatchDictionary &dict) :
    imageDir(imageD), filename(fname), outputDir(outD), nChannels(nC),
    patchTypeToEnum(ptToEnum), channelTypes(cTs),
    firstChannelExtension(firstCExt), enumToExtension(enumToExt),
    width(w), height(h), depthMapWidth(dmW), depthMapHeight(dmH),
    dictionary(dict) {}

  const char *imageDir;
  string filename;
  const char *outputDir;
  const int nChannels;
  const map<string, svlPatchDefinitionType> &patchTypeToEnum;
  const vector<string> &channelTypes;
  const string &firstChannelExtension;
  const map<svlPatchDefinitionType, string> &enumToExtension;
  const int width, height, depthMapWidth, depthMapHeight;
  svlPatchDictionary &dictionary;
};

// Thread function
void *processFile(void *voidArgs) {
  ProcessFileArgs *args = (ProcessFileArgs*) voidArgs;
  const char *imageDir = args->imageDir;
  string filename = args->filename;
  const char *outputDir = args->outputDir;
  const int nChannels = args->nChannels;
  const map<string, svlPatchDefinitionType> &patchTypeToEnum =
    args->patchTypeToEnum;
  const vector<string> channelTypes = args->channelTypes;
  const string &firstChannelExtension = args->firstChannelExtension;
  const map<svlPatchDefinitionType, string> enumToExtension =
    args->enumToExtension;
  const int width = args->width;
  const int height = args->height;
  const int depthMapWidth = args->depthMapWidth;
  const int depthMapHeight = args->depthMapHeight;
  svlPatchDictionary &dictionary = args->dictionary;

  IplImage *imageData = NULL;
  IplImage *depthData = NULL;

  svlSoftEdgeMap edgeMapCalculator;

  SVL_LOG(SVL_LOG_MESSAGE, "Processing " << filename << "...");
  vector<IplImage *> images;
		
  // Load images from each channel
  for (int i = 0; i < nChannels; i++) {
    //Whether the input data has to be loaded from image or has been peviously loaded
    int loadFromDisk = 1;

    IplImage *img = NULL;
                
    // Map to enumeration
    svlPatchDefinitionType patchEnumType =
      (*(patchTypeToEnum.find(channelTypes[i]))).second;
                
    if (patchEnumType == SVL_INTENSITY_PATCH) {
      if (imageData != NULL) {
	img = cvCloneImage(imageData);
	loadFromDisk = 0;
      }                    
    } else if(patchEnumType == SVL_DEPTH_PATCH ) {
      if (depthData != NULL) {
	img = cvCloneImage(depthData);
	loadFromDisk = 0;
      }
    } else SVL_LOG(SVL_LOG_FATAL, "Unknown channel type provided" );
                
    //If the raw input data for this channel needs to be loaded from disk
    if (loadFromDisk) {
      string imageFilename = string(imageDir) + string("/") + filename;
                        
      //Form filename for the rest of the channels by modifying extensions
      if (i != 0) {
	imageFilename.replace(imageFilename.length() -
			      firstChannelExtension.size(),
			      firstChannelExtension.size(),
			      (*(enumToExtension.find(patchEnumType))).second);
      }
                        
      if (patchEnumType == SVL_INTENSITY_PATCH) {
	img = cvLoadImage(imageFilename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
        
	if (img == NULL) {
	  SVL_LOG(SVL_LOG_ERROR, "Problem opening " << imageFilename);
	  return NULL;
	}
	if (imageData == NULL) imageData = cvCloneImage(img);
      } else if( patchEnumType == SVL_DEPTH_PATCH ) {
	img = readMatrixAsIplImage(imageFilename.c_str(),
				   depthMapWidth, depthMapHeight);
	
	if (depthData == NULL) depthData = cvCloneImage(img);
      }                        
    }
                
    if (img == NULL) {
      SVL_LOG(SVL_LOG_MESSAGE, "...failed");
    } else {
      if ((img->width != (int)width) || (img->height != (int)height)) {
	resizeInPlace(&img, height, width);
      }
                        
      // Handle transform of the raw input data
      if (channelTypes[i] == string("EDGE")) {
	img = cvCloneImage(edgeMapCalculator.processImage(img));
	images.push_back(img);
      } else 
	images.push_back(img);
    }
                
  }//channels loop
            
  // process image
  vector<double> v = dictionary.patchResponse(images);
            
  // write feature vector
  string outputFilename = string(outputDir) + string("/") + filename;
  outputFilename.replace(outputFilename.size() - 3, 3, "txt");

  SVL_LOG(SVL_LOG_VERBOSE, "...writing output to " << outputFilename.c_str());
            
  ofstream ofs(outputFilename.c_str());
  if (ofs.fail()) {
    SVL_LOG(SVL_LOG_WARNING, "...failed");
  } else {
    for (unsigned i = 0; i < v.size(); i++) {
      ofs << v[i] << endl;
    }
  }
  ofs.close();
            
  // Free memory
  for (unsigned i = 0; i < images.size(); i++) cvReleaseImage(&(images[i]));

  if (imageData != NULL) cvReleaseImage(&imageData);
  if (depthData != NULL) cvReleaseImage(&depthData);

  delete args;

  return NULL;
}

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./buildPatchResponseCache [OPTIONS] <image dir> <output dir> <dictionary>" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -maxImages <num>   :: maximum training images to cache (default: 10000)" << endl
         << "  -profile           :: turn on SVL code profiling" << endl
         << "  -v                 :: verbose" << endl
	 << "  -ch <n> <c1> ... <c_n> :: n input channels, along with types for each of them"<< endl
	 << "  -dmsize <w> <h>    :: dimensions of the depth map"<< endl
	 << "  -imageExt <str>    :: extension of the image data"<< endl
	 << "  -depthMapExt <str> :: extension of the depth map data if any"<< endl
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
    map<string,svlPatchDefinitionType> patchTypeToEnum;	
    patchTypeToEnum["INTENSITY"] = SVL_INTENSITY_PATCH;
    patchTypeToEnum["EDGE"] = SVL_INTENSITY_PATCH;
    patchTypeToEnum["DEPTH"] = SVL_DEPTH_PATCH;
    
    const char *imageDir = NULL;
    const char *outputDir = NULL;
    const char *dictionaryFilename = NULL;
    int maxImages = 10000;
    int nChannels = 2;
    int depthMapWidth = -1;
    int depthMapHeight = -1;
    string imageExt = string(".jpg");
    string depthMapExt = string(".reconstructed.z.txt");
	
    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        if (!strcmp(*args, "-maxImages")) {
            maxImages = atoi(*(++args)); argc--;
	} else if (!strcmp(*args, "-profile")) {
	  svlCodeProfiler::enabled = true;
	  svlCodeProfiler::tic(svlCodeProfiler::getHandle("buildPatchResponseCache"));
        } else if (!strcmp(*args, "-ch")) {
	   channelTypes.clear();
	   nChannels = atoi(*(++args)); argc--;
           
           //Read all the channel types
           for( int i=0 ; i<nChannels ; i++) {
               channelTypes.push_back( string(*(++args)));
               argc--;
           }
        } else if (!strcmp(*args, "-v")) {
            svlLogger::setLogLevel(SVL_LOG_VERBOSE);
        } else if (!strcmp(*args, "-dmsize")) {
            depthMapWidth = atoi(*(++args)); argc--;
            depthMapHeight = atoi(*(++args)); argc--;
        } else if (!strcmp(*args, "-imageExt")) {
            imageExt = string(*(++args)); argc--;
        } else if (!strcmp(*args, "-depthMapExt")) {
            depthMapExt = string(*(++args)); argc--;
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
    outputDir = args[1];
    dictionaryFilename = args[2];

    // read dictionary
    svlPatchDictionary dictionary;
    dictionary.read(dictionaryFilename);
	
    SVL_LOG(SVL_LOG_MESSAGE, dictionary.numEntries() << " patches in dictionary");

    int width = dictionary.windowWidth();
    int height = dictionary.windowHeight();

	
    if( depthMapWidth == -1 )
        depthMapWidth = width;
    if( depthMapHeight == -1 )
        depthMapHeight = height;

    //Maps patch type enums to respetive input data extensions
    map<svlPatchDefinitionType,string> enumToExtension;	
    enumToExtension[SVL_INTENSITY_PATCH] = imageExt;
    enumToExtension[SVL_DEPTH_PATCH] = depthMapExt;
    
    // read, resize and process images
    DIR *dir = opendir(imageDir);
    if (dir == NULL) {
        SVL_LOG(SVL_LOG_FATAL, "could not open image directory " << imageDir);
    }
    
    //Image objects for holding raw input data, which are transformed into channels
    IplImage *imageData = NULL;
    IplImage *depthData = NULL;

    svlSoftEdgeMap edgeMapCalculator;

    int numImages = 0;
    struct dirent *e = readdir(dir);

    string firstChannelExtension = (*(enumToExtension.find( (*(patchTypeToEnum.find(channelTypes[0]))).second ))).second;

    // TODO: how many threads?
    svlThreadPool threadPool(8);
    threadPool.start();

    while ((e != NULL) && (numImages < maxImages)) {

#ifdef USE_THREADS
      string filename(e->d_name);
      if (strstr(filename.c_str(), firstChannelExtension.c_str()) != NULL) {
	threadPool.addJob(processFile,
			  new ProcessFileArgs(imageDir, filename, outputDir,
					      nChannels,
					      patchTypeToEnum, channelTypes,
					      firstChannelExtension,
					      enumToExtension, width, height,
					      depthMapWidth, depthMapHeight,
					      dictionary));

	numImages++; // TODO: make this contingent on success?
      }
#else
      if (strstr(e->d_name,firstChannelExtension.c_str()) != NULL) {
            SVL_LOG(SVL_LOG_MESSAGE, "Processing " << e->d_name << "...");
            vector<IplImage *> images;
			
            //Load images from each channel
            for (int i = 0; i < nChannels; i++) {
                //Whether the input data has to be loaded from image or has been peviously loaded
                int loadFromDisk = 1;

                IplImage *img = NULL;
                
                //Map to enumeration
                svlPatchDefinitionType patchEnumType = (*(patchTypeToEnum.find(channelTypes[i]))).second;
                
                if( patchEnumType == SVL_INTENSITY_PATCH ) {
                    if( imageData != NULL ) {
                        img = cvCloneImage(imageData);
                        loadFromDisk = 0;
                    }                    
                } else if(patchEnumType == SVL_DEPTH_PATCH ) {
                    if( depthData != NULL) {
                        img = cvCloneImage(depthData);
                        loadFromDisk = 0;
                    }
                } else
                    SVL_LOG(SVL_LOG_FATAL, "Unknown channel type provided" );
                
                //If the raw input data for this channel needs to be loaded from disk
                if( loadFromDisk ) {
                    string imageFilename = string(imageDir) + string("/") + string(e->d_name);
                        
                    //Form filename for the rest of the channels by modifying extensions
                        if( i !=0 ) {
                            imageFilename.replace(imageFilename.length() - firstChannelExtension.size() , firstChannelExtension.size() 
                                ,  (*(enumToExtension.find(patchEnumType))).second);
                        }
                        
                        if( patchEnumType == SVL_INTENSITY_PATCH ) {
                            img = cvLoadImage(imageFilename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
                            
                            if( imageData == NULL )
                                imageData = cvCloneImage(img);
                        } else if( patchEnumType == SVL_DEPTH_PATCH ) {
			  img = readMatrixAsIplImage(imageFilename.c_str(), depthMapWidth, depthMapHeight);
                                
                            if(depthData == NULL )
                                depthData = cvCloneImage(img);
                        }                        
                    }
                
                if (img == NULL) {
                    SVL_LOG(SVL_LOG_MESSAGE, "...failed");
                } else {
                    if ((img->width != (int)width) || (img->height != (int)height)) {
                        resizeInPlace(&img, height, width);
                    }
                        
                    //Handle transform of the raw input data
                    if (channelTypes[i] == string("EDGE")) {
                        img = cvCloneImage(edgeMapCalculator.processImage(img));
                        images.push_back(img);
                    } else 
                        images.push_back(img);
                }
                
            }//channels loop
            
            // process image
            vector<double> v = dictionary.patchResponse(images);
            
            // write feature vector
            string outputFilename = string(outputDir) + string("/") + string(e->d_name);
            outputFilename.replace(outputFilename.size() - 3, 3, "txt");
                        
            SVL_LOG(SVL_LOG_VERBOSE, "...writing output to " << outputFilename.c_str());
            
            ofstream ofs(outputFilename.c_str());
            if (ofs.fail()) {
                SVL_LOG(SVL_LOG_WARNING, "...failed");
            } else {
                for (unsigned i = 0; i < v.size(); i++) {
                    ofs << v[i] << endl;
                }
                numImages += 1;
            }
            ofs.close();
            
            //Free memory
            for( int i=0 ; i<images.size() ; i++)
                cvReleaseImage(&(images[i]));
            
            
            if( imageData != NULL )
                cvReleaseImage(&imageData);
            if( depthData != NULL )
                cvReleaseImage(&depthData);
            
        }//Extension match if
#endif
        e = readdir(dir);
        
    }//dir while
    closedir(dir);

    threadPool.finish();
    svlCodeProfiler::toc(svlCodeProfiler::getHandle("buildPatchResponseCache"));
    svlCodeProfiler::print(cerr);
    
    return 0;
}

