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
** FILENAME:    mrfDepthSmooth.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Smooths the projected depth map using an MRF with potentials defined over
**  image and point cloud features. See Diebel and Thrun 2005 for details.
**
**  Assumes the point cloud has been translated and rotated to the standard
**  world coordinate system (laser centric). Default camera extrinsics and
**  intrinsics are used unless -camera option is specified.
**
*****************************************************************************/

#include <cstdlib>
#include <string>
#include <limits>
#include <iomanip>

// OpenCV library
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

// stair vision library
#include "svlBase.h"
#include "svlVision.h"

#define DEBUG_WND_NAME "mrfDepthSmooth"

using namespace std;

// Main ----------------------------------------------------------------------

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./mrfDepthSmooth [OPTIONS] <image> <pointCloud>" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -alpha <num>           :: measurement potential" << endl
	 << "  -beta <num>            :: 0-th derivative smoothness potential" << endl
	 << "  -gamma <num>           :: 1-st derivative smoothness potential" << endl
	 << "  -camera <file>         :: intrinsics and extrinsics calibration file" << endl
	 << "  -debug                 :: toggle debugging (default: false)" << endl
    //	 << "  -discrete              :: run discrete optimization (min-cut)" << endl
	 << "  -holdout <frac>        :: hold out <frac> of the points and compare with MRF" << endl
	 << "  -maxiter <num>         :: maximum number of iterations (default: 1000)" << endl
	 << "  -o <filename>          :: output filestem (adds .txt extensions)" << endl
	 << "  -rescale <scale>       :: scale image before smoothing" << endl
	 << "  -v                     :: toggle verbose (default: false)" << endl
	 << "  -weight <num>          :: image edge weight factor" << endl
	 << "  -undistort             :: use intrinsics to undistort image" << endl
	 << "  -pointsOnly            :: point cloud contains points only" << endl
	 << "  -dataWidth <n>         :: number of fields in point cloud file (default: 3)" << endl
	 << endl;
}

int main(int argc, char *argv[])
{
    // read command line parameters
    const int NUM_REQUIRED_PARAMETERS = 2;
    
    const char *imageFilename = NULL;
    const char *pointCloudFilename = NULL;

    bool bDebug = false;
    bool bVerbose = false;
    bool bDiscrete = false;
    int nMaxIterations = 1000;
    const char *outputFilename = NULL;
    double scaleFactor = 1.0;
    const char *calibrationFilename = NULL;
    bool bPointsOnly = false;
    int nDataWidth = 3;
    bool bHoldout = false;
    bool bUndistort = false;

    double weightFactor = 0.05;
    double alpha = 1.0;
    double beta = 0.0;
    double gamma = 0.5;
    double holdoutFrac = 0.3;

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
	if (!strcmp(*args, "-alpha")) {
	    alpha = atof(*(++args));
	    argc--;
	} else if (!strcmp(*args, "-beta")) {
	    beta = atof(*(++args));
	    argc--;
	} else if (!strcmp(*args, "-gamma")) {
	    gamma = atof(*(++args));
	    argc--;
	} else if (!strcmp(*args, "-camera")) {
	    calibrationFilename = *(++args); argc--;
	} else if (!strcmp(*args, "-debug")) {
	    bDebug = !bDebug;
	} else if (!strcmp(*args, "-holdout")) {
	    bHoldout = !bHoldout;
	    holdoutFrac = atof(*(++args));
	    argc--;
	} else if (!strcmp(*args, "-discrete")) {
	    bDiscrete = true;
	} else if (!strcmp(*args, "-o") || !strcmp(*args, "-output")) {
	    outputFilename = *(++args);
	    argc--;
	} else if (!strcmp(*args, "-maxiter")) {
	    nMaxIterations = atoi(*(++args));
	    argc--;
	} else if (!strcmp(*args, "-rescale")) {
	    scaleFactor = atof(*(++args));
	    argc--;
	    assert(scaleFactor > 0.0);
	} else if (!strcmp(*args, "-v")) {
	    bVerbose = !bVerbose;
	} else if (!strcmp(*args, "-weight")) {
	    weightFactor = atof(*(++args));
	    argc--;
	} else if (!strcmp(*args, "-undistort")) {
	    bUndistort = true;
	} else if (!strcmp(*args, "-pointsOnly")) {
	    bPointsOnly = true;
	} else if (!strcmp(*args, "-dataWidth")) {
	    nDataWidth = atoi(*(++args));
	    argc--;
	    assert(nDataWidth >= 3);
	} else {
	    cerr << "ERROR: unrecognized option " << *args << endl;
	    return -1;
	}
	args++;
    }
    
    if (argc != NUM_REQUIRED_PARAMETERS) {
	usage();
	return -1;
    }

    imageFilename = args[0];
    pointCloudFilename = args[1];

    // read image and point cloud
    if (bVerbose) {
	cerr << "Reading image from " << imageFilename << "..." << endl;
	cerr << "Reading point cloud from " << pointCloudFilename << "..." << endl;
	if (calibrationFilename != NULL)
	    cerr << "Reading camera calibration from " << calibrationFilename << "..." << endl;
    }

    IplImage *image = NULL;
    image = cvLoadImage(imageFilename, CV_LOAD_IMAGE_GRAYSCALE);
    assert(image != NULL);

    svlPointCloudData cloud(numeric_limits<unsigned int>::max()), test_cloud(numeric_limits<unsigned int>::max());
    if (bPointsOnly) {
	cloud.import(pointCloudFilename, true, nDataWidth);
    } else {
	cloud.read(pointCloudFilename);
    }

    if (bHoldout){
	int num_to_drop = (int)(cloud.pointCloud.size() * holdoutFrac);
	cout << "holding out " << num_to_drop << " points to compare measurements with MRF inferences..." << endl;
	for (int i = 0; i < num_to_drop; i++) {
	    int idx = rand() % cloud.pointCloud.size();
	    test_cloud.pointCloud.push_back(cloud.pointCloud[idx]);
	    cloud.pointCloud.erase(cloud.pointCloud.begin() + idx);
	}
	cout << "cloud has " << cloud.pointCloud.size() << " points" << endl;
	cout << "test cloud has " << test_cloud.pointCloud.size() << " points" << endl;
    }
    
    if (bVerbose) {
	cerr << "  " << image->width << " x " << image->height << " image resolution ("
	     << (image->width * image->height) << " pixels)" << endl;
	cerr << "  " << cloud.pointCloud.size() << " points read" << endl;
	cerr << "...done" << endl;
    }

    svlCameraIntrinsics intrinsics;
    svlCameraExtrinsics extrinsics;
    if (calibrationFilename == NULL) {
	cerr << "WARNING: using default intrinsics" << endl;
	intrinsics.initialize(557.267, 568.585, 328.355, 238.273, 0.0,
	    -0.326683, 0.244823, 0.000453, -0.001617);	
	cerr << "WARNING: using default extrinsics" << endl;
	// from gates_108 dataset (23 August 2007)
	extrinsics.initialize(0.9968,   -0.0233,    0.0770,
	    0.0222,    0.9996,    0.0157,
	    -0.0773,   -0.0140,    0.9969,
	    -0.1456,   -0.7145,   -0.0552);
    } else {
	intrinsics.initialize(calibrationFilename);
	extrinsics.initialize(calibrationFilename);
    }

    if (bUndistort) {
	const IplImage *undistortedImage = intrinsics.undistort(image);
	cvCopyImage(undistortedImage, image);
    }

    if (scaleFactor != 1.0) {
        intrinsics.rescale(scaleFactor);

        IplImage *rescaledImage = cvCreateImage(cvSize((int)(scaleFactor * image->width),
            (int)(scaleFactor * image->height)), image->depth, image->nChannels);
        cvResize(image, rescaledImage);
        cvReleaseImage(&image);
        image = rescaledImage;
    }

    if (bDebug) {
	cvNamedWindow(DEBUG_WND_NAME, CV_WINDOW_AUTOSIZE);
	cvShowImage(DEBUG_WND_NAME, image);
	cvWaitKey(-1);
    }

    // define mrf
    if (bVerbose) {
	cerr << "Defining MRF..." << endl;
    }

    // data for optimization
    svlSecondOrderSuperResolutionMRF mrf(image->width, image->height);

    //cvSmooth(image, image, CV_GAUSSIAN, 3);

    mrf.weightFactor = weightFactor;
    mrf.alpha = alpha;
    mrf.beta = beta;
    mrf.gamma = gamma;
    mrf.initializeWeights(image);

    if (bVerbose) {
	cerr << "...done" << endl;
    }

    svlImageProjector projector(intrinsics, extrinsics);
    projector.clearImage(mrf.getWidth(), mrf.getHeight());
    projector.project(cloud.pointCloud);
    mrf.initializeMeasurements(projector.getData(2));

    if (bDebug) {
        IplImage *debugImage = cvCreateImage(cvSize(image->width, image->height),
            IPL_DEPTH_8U, 3);
        cvCvtColor(image, debugImage, CV_GRAY2RGB);
        const CvMat *depth = projector.getData(2);
        for (int y = 0; y < image->height; y++) {
            for (int x = 0; x < image->width; x++) {
                if (cvmGet(depth, y, x) > 0) {
                    debugImage->imageData[y * debugImage->widthStep + 3 * x + 2] = 255;
                    debugImage->imageData[y * debugImage->widthStep + 3 * x + 1] = 0;
                    debugImage->imageData[y * debugImage->widthStep + 3 * x + 0] = 0;
                }
            }
        }
	
    	cvShowImage(DEBUG_WND_NAME, debugImage);
	    cvWaitKey(-1);
        cvReleaseImage(&debugImage);
    }

    // starting point    
    projector.clearImage(mrf.getWidth(), mrf.getHeight());
    //projector.setSmoothing(0.2);
    projector.project(cloud.pointCloud);

    CvMat *initialDepth = cvCloneMat(projector.getData(2));
    svlNearestNeighbourFill(initialDepth);

#if 0
    if (bDebug)
	writeMatrix(initialDepth, "depth.debug.txt");
#endif

    mrf.initializeDepthEstimate(initialDepth);
    cvReleaseMat(&initialDepth);

    // optimize
    if (bVerbose) cerr << "Optimizing..." << endl;
    mrf.solve(nMaxIterations, 1.0e-2, bDebug);
    if (bVerbose) cerr << "...done" << endl;

    if (bDebug) {
	cvWaitKey(-1);
    }

    if (bHoldout) {
        cout << "Comparing projections with real measurements at " << test_cloud.pointCloud.size() << " test points" << endl;
        FILE *f = fopen("holdout_errors.txt", "w");
        double sse = 0.0;
        for (size_t i = 0; i < test_cloud.pointCloud.size(); i++) {
            // find image point corresponding to 3-d point
            CvPoint pt = intrinsics.point(extrinsics.world2camera(test_cloud.pointCloud[i]));
            double depth_est = mrf.getDepth(pt.y * mrf.getWidth() + pt.x);
            // compare depth estimate with actual depth
            fprintf(f, "%f\n", depth_est - extrinsics.lastZ());
            sse += (depth_est - extrinsics.lastZ()) * (depth_est - extrinsics.lastZ());
        }
        fclose(f);
        cout << "...average error is " << (sse / (double)test_cloud.pointCloud.size()) << endl;
    }

    // write output
    if (outputFilename != NULL) {
	if (bVerbose) cerr << "Writing output to " << outputFilename << ".txt..." << endl;

	cvReleaseImage(&image);
	image = cvLoadImage(imageFilename, CV_LOAD_IMAGE_COLOR);

	// TO DO: should undistort this image

	if (scaleFactor != 1.0) {
	    IplImage *rescaledImage = cvCreateImage(cvSize((int)(scaleFactor * image->width),
		    (int)(scaleFactor * image->height)), image->depth, image->nChannels);
	    cvResize(image, rescaledImage);
	    cvReleaseImage(&image);
	    image = rescaledImage;
	}
	
	// reconstruct point cloud
	svlPointCloudData reconstructed;
	mrf.reconstructPointCloud(reconstructed, image, intrinsics);
	string filename = string(outputFilename) + string(".txt");
	reconstructed.write(filename.c_str());

	// reconstruct dense point cloud
	CvMat *X, *Y, *Z;
	X = cvCreateMat(image->height, image->width, CV_32FC1);
	Y = cvCreateMat(image->height, image->width, CV_32FC1);
	Z = cvCreateMat(image->height, image->width, CV_32FC1);
	mrf.reconstructPointCloud(X, Y, Z, intrinsics);
	
	if (bVerbose) cerr << "Writing dense point clouds to " << outputFilename << ".{x,y,z}.txt..." << endl;
	filename = string(outputFilename) + string(".x.txt");
	ofstream ofs(filename.c_str());
	writeMatrix(X, ofs);
	ofs.close();

	filename = string(outputFilename) + string(".y.txt");
	ofs.open(filename.c_str());
	writeMatrix(Y, ofs);
	ofs.close();

	filename = string(outputFilename) + string(".z.txt");
	ofs.open(filename.c_str());
	writeMatrix(Z, ofs);
	ofs.close();

	cvReleaseMat(&Z);
	cvReleaseMat(&Y);
	cvReleaseMat(&X);
	if (bVerbose) cerr << "...done" << endl;	
    }

    // free memory
    cvDestroyAllWindows();
    cvReleaseImage(&image);

    return 0;
}

