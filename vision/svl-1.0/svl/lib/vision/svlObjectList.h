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
** FILENAME:    svlObjectList.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Olga Russakovsky (olga@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**  Classes for defining object lists in 2d.
**
*****************************************************************************/

#pragma once

#include <vector>
#include <iostream>
#include <map>

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

// svlObject2d Class --------------------------------------------------------
// This class holds the definition of a single object in the image plane.
//
class svlObject2d {
public:
    svlObject2d();
    svlObject2d(const svlObject2d& o);
    svlObject2d(double nx, double ny, double s = 1.0);
    svlObject2d(double nx, double ny, double nw, double nh, double p = 1.0);
    svlObject2d(const svlPoint3d& u, const svlPoint3d& v, double p = 1.0);
    ~svlObject2d();

    bool hit(double qx, double qy) const;
    inline double area() const { return fabs(w * h); }
    double overlap(const svlObject2d& o) const;
    inline double areaOverlap(const svlObject2d& o) const { 
        double ov = overlap(o);
        return (ov / (area() + o.area() - ov));
    }
    inline bool inside(const svlObject2d& o) const { 
        return (fabs(overlap(o) - area()) < SVL_EPSILON);
    }
    inline void scale(double s) {
	x *= s; y *= s; w *= s; h *= s;
    }
    inline void scale(double x_scale, double y_scale) {
        x *= x_scale; y *= y_scale; w *= x_scale; h *= y_scale;
    }

    svlObject2d& operator=(const svlObject2d& o);
    friend std::ostream& operator<<(std::ostream& os, const svlObject2d& o);

public:
    std::string name;   // name of object
    double x, y;        // top-left
    double w, h;        // width/height
    double pr;          // probability

    int index;          // index for external reference
};

// svlObject2dFrame Class ---------------------------------------------------
// This class holds a list of 2d objects (for example, all objects appearing
// in a single video frame).
//
typedef std::vector<svlObject2d> svlObject2dFrame;

bool writeObject2dFrame(std::ostream &os, const svlObject2dFrame& v,
    const char *id = NULL);

int removeOverlappingObjects(svlObject2dFrame& frame, double threshold = 0.9);
int removeGroundTruthObjects(svlObject2dFrame& frame, svlObject2dFrame& truth, 
    double threshold = 0.9);
int removeNonGroundTruthObjects(svlObject2dFrame& frame, svlObject2dFrame& truth, 
    double threshold = 0.9);
int removeMatchingObjects(svlObject2dFrame& frame, const char *name);
int removeNonMatchingObjects(svlObject2dFrame& frame, const char *name);

// remove non-maximal objects from frame within neighborhood defined by (dx, dy, ds)
int nonMaximalSuppression(svlObject2dFrame& frame, double dx = 0.125, double dy = 0.125,
    double ds = 0.8, double dsArea = 0.75);

// sort detections by probability
svlObject2dFrame sortObjects(const svlObject2dFrame& frame);

// svlObject2dSequence Class -----------------------------------------------
// This class holds a map of object lists. This can be used to hold one
// list of objects per image frame for the entire video or image sequence.
// The map is indexed by filename (or ID) or in the case of videos by
// (0-based) frame index.
//
typedef std::map<string, svlObject2dFrame> svlObject2dSequence;

bool writeObject2dSequence(const char *filename, const svlObject2dSequence& v);
bool readObject2dSequence(const char *filename, svlObject2dSequence& v);

void scaleObject2dSequence(svlObject2dSequence &v, double x_scale, double y_scale);

int countObjects(const svlObject2dSequence& v);
int removeOverlappingObjects(svlObject2dSequence& v, double threshold = 0.9);
int nonMaximalSuppression(svlObject2dSequence& v, double dx = 0.125, double dy = 0.125,
    double ds = 0.8, double dsArea = 0.75);

bool removeFramesFromSequence(svlObject2dSequence &v, const char *filename);


