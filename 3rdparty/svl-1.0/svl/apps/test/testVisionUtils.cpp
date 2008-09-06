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
** FILENAME:    testVisionUtils.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Regression tests for vision utilities.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

// function prototypes ------------------------------------------------------

void planeFitTest(const char *filename);

// main ---------------------------------------------------------------------

void usage()
{
    cerr << SVL_USAGE_HEADER << "\n";
    cerr << "USAGE: ./testVisionUtils [OPTIONS]\n";
    cerr << "OPTIONS:\n"
	 << "  -plane <file> :: fit a plane to a set of points\n"
         << "  -profile      :: toggle on profiling\n"
         << "  -verbose      :: toggle verbose\n"
	 << endl;
}

int main(int argc, char *argv[])
{
    svlLogger::setLogLevel(SVL_LOG_MESSAGE);

    if (argc == 1) {
        usage();
        return 0;
    }

    char **args = argv + 1;
    while (--argc > 0) {
	if (!strcmp(*args, "-profile")) {
            svlCodeProfiler::enabled = !svlCodeProfiler::enabled;
	} else if (!strcmp(*args, "-plane")) {
            int handle = svlCodeProfiler::getHandle("plane");
            svlCodeProfiler::tic(handle);
            planeFitTest(*(++args));
            argc -= 1;
            svlCodeProfiler::toc(handle);
        } else if (!strcmp(*args, "-verbose")) {
            if (svlLogger::getLogLevel() == SVL_LOG_VERBOSE) {
                svlLogger::setLogLevel(SVL_LOG_MESSAGE);
            } else {
                svlLogger::setLogLevel(SVL_LOG_VERBOSE);
            }
	} else {
	    SVL_LOG(SVL_LOG_ERROR, "unrecognized option " << *args);
	    usage();
	    return -1;
	}
	args++;
    }
    
    svlCodeProfiler::print(cerr);
    return 0;
}

// test functions -----------------------------------------------------------

void planeFitTest(const char *filename)
{
    // read points
    SVL_LOG(SVL_LOG_VERBOSE, "Reading points from " << filename << "...");

    ifstream ifs(filename);
    if (ifs.fail()) {
        SVL_LOG(SVL_LOG_ERROR, "could not load points from " << filename);
        return;
    }
    
    vector<svlPoint3d> points;
    while (!ifs.fail()) {
        double x, y, z;
        ifs >> x >> y >> z;
        if (ifs.fail()) break;
        points.push_back(svlPoint3d(x, y, z));
    }
    ifs.close();

    SVL_LOG(SVL_LOG_VERBOSE, "..." << points.size() << " points read");

    // construct matrices for points
    CvMat *X = cvCreateMat(points.size(), 1, CV_32FC1);
    CvMat *Y = cvCreateMat(points.size(), 1, CV_32FC1);
    CvMat *Z = cvCreateMat(points.size(), 1, CV_32FC1);
    assert((X != NULL) && (Y != NULL) && (Z != NULL));

    vector<CvPoint> pointIndices(points.size());
    for (unsigned i = 0; i < points.size(); i++) {
        pointIndices[i] = cvPoint(0, i);
        CV_MAT_ELEM(*X, float, i, 0) = (float)points[i].x;
        CV_MAT_ELEM(*Y, float, i, 0) = (float)points[i].y;
        CV_MAT_ELEM(*Z, float, i, 0) = (float)points[i].z;
    }

    // compute planar fit
    SVL_LOG(SVL_LOG_VERBOSE, "Estimating planar fit...");
    svlPoint3d a, b;
    estimatePlane(X, Y, Z, pointIndices, a, b);
    SVL_LOG(SVL_LOG_VERBOSE, "...done");
    
    SVL_LOG(SVL_LOG_MESSAGE, "Plane normal is " << toString(a));
    SVL_LOG(SVL_LOG_MESSAGE, "Plane offset is " << toString(b));

    // release memory
    cvReleaseMat(&Z);
    cvReleaseMat(&Y);
    cvReleaseMat(&X);    
}
