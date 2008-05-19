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
** FILENAME:    objectList.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <cassert>
#include <string>

#include "svlPoint3d.h"
#include "svlObjectList.h"

// svlObject2d Class --------------------------------------------------------

svlObject2d::svlObject2d() :
    name("[unknown]"), x(0.0), y(0.0), w(1.0), h(1.0), pr(1.0), index(-1)
{
    // do nothing
}

svlObject2d::svlObject2d(const svlObject2d& o) :
    name(o.name), x(o.x), y(o.y), w(o.w), h(o.h), pr(o.pr), index(o.index)
{
    // do nothing
}

svlObject2d::svlObject2d(double nx, double ny, double s) :
    name("[unknown]"), x(nx), y(ny), w(s), h(s), pr(1.0), index(-1)
{
    // do nothing
}

svlObject2d::svlObject2d(double nx, double ny, double nw, double nh, double p) :
    name("[unknown]"), x(nx), y(ny), w(nw), h(nh), pr(p), index(-1)
{
    // do nothing
}

svlObject2d::svlObject2d(const svlPoint3d& u, const svlPoint3d& v, double p) :
    name("[unknown]"), x(u.x), y(u.y), w(v.x - u.x), h(v.y - u.y), pr(p), index(-1)
{
    if (w < 0.0) { x = v.x; w = -w; }
    if (h < 0.0) { y = v.y; h = -h; }
}

svlObject2d::~svlObject2d()
{
    // do nothing
}
 
bool svlObject2d::hit(double qx, double qy) const
{
    return ((qx >= x) && (qx <= x + w) && (qy >= y) && (qy <= y + h));
}

double svlObject2d::overlap(const svlObject2d& o) const
{
    double ix, iy, iw, ih;  // intersection
    
    // find region of overlap
    ix = (x < o.x) ? o.x : x;
    iy = (y < o.y) ? o.y : y;
    iw = ((x + w < o.x + o.w) ? x + w : o.x + o.w) - ix;
    ih = ((y + h < o.y + o.h) ? y + h : o.y + o.h) - iy;
    
    if ((iw < 0) || (ih < 0.0)) {
        return 0.0;
    }

    // return area of intersection
    return (iw * ih);
}

svlObject2d& svlObject2d::operator=(const svlObject2d& o)
{
  x = o.x; y = o.y; w = o.w; h = o.h; pr = o.pr;
  name = o.name;
  index = o.index;

  return *this;
}

ostream& operator<<(ostream& os, const svlObject2d& o)
{
    os << o.name.c_str() << " <" << o.x << ", " << o.y << ", " << o.w << ", " << o.h << ">";
    return os;
}

// svlObject2dFrame Class --------------------------------------------------------

bool writeObject2dFrame(ostream &os, const svlObject2dFrame& v, int index)
{
    if (os.fail()) return false;

    os << "    <Object2dFrame index=\"" << index << "\">" << endl;
    for (unsigned j = 0; j < v.size(); j++) {
	os << "        <Object name=\"" << v[j].name.c_str() << "\""
	   << " x=\"" << v[j].x << "\""
	   << " y=\"" << v[j].y << "\""
	   << " w=\"" << v[j].w << "\""
	   << " h=\"" << v[j].h << "\""
	   << " pr=\"" << v[j].pr << "\" />" << endl;
    }
    os << "    </Object2dFrame>" << endl;
    
    return true;
}

int removeOverlappingObjects(svlObject2dFrame& frame, double threshold)
{
    int count = 0;
    for (unsigned i = 0; i < frame.size(); i++) {
	double areaA = frame[i].area();
	for (unsigned j = (unsigned)frame.size() - 1; j > i; j--) {
	    if (frame[i].name != frame[j].name) {
		continue;
	    }
	    double areaOverlap = frame[i].overlap(frame[j]);
	    double areaB = frame[j].area();
	    if ((areaOverlap > threshold * areaA) &&
		(areaOverlap > threshold * areaB)) {
		if (areaB > areaA) {
		    frame[i] = frame[j];		   
		}
		frame.erase(frame.begin() + j);
		count += 1;
	    }
	}
    }

    return count;
}

int removeGroundTruthObjects(svlObject2dFrame& frame, svlObject2dFrame& truth, double threshold)
{
    int count = 0;
    for (int i = (int)frame.size() - 1; i >= 0; i--) {
	double areaA = frame[i].area();
	for (int j = 0; j < (int)truth.size(); j++) {
	    if (frame[i].name != truth[j].name) {
		continue;
	    }
	    double areaOverlap = frame[i].overlap(truth[j]);
	    double areaB = truth[j].area();
	    if ((areaOverlap > threshold * areaA) &&
		(areaOverlap > threshold * areaB)) {
		frame.erase(frame.begin() + i);
		count += 1;
		break;
	    }
	}
    }

    return count;
}

int removeNonGroundTruthObjects(svlObject2dFrame& frame, svlObject2dFrame& truth, double threshold)
{
    svlObject2dFrame keepList;
    
    keepList.reserve(frame.size());
    for (int i = (int)frame.size() - 1; i >= 0; i--) {
	double areaA = frame[i].area();
	for (int j = 0; j < (int)truth.size(); j++) {
	    if (frame[i].name != truth[j].name) {
		continue;
	    }
	    double areaOverlap = frame[i].overlap(truth[j]);
	    double areaB = truth[j].area();
	    if ((areaOverlap > threshold * areaA) &&
		(areaOverlap > threshold * areaB)) {
		keepList.push_back(frame[i]);
		break;
	    }
	}
    }

    int count = (int)(frame.size() - keepList.size());
    frame = keepList;

    return count;
}

int removeMatchingObjects(svlObject2dFrame& frame, const char *name)
{
    int count = 0;
    for (int i = (int)frame.size() - 1; i >= 0; i--) {
	if (frame[i].name == string(name)) {
	    frame.erase(frame.begin() + i);
	    count += 1;
	}
    }

    return count;
}

int removeNonMatchingObjects(svlObject2dFrame& frame, const char *name)
{
    int count = 0;
    for (int i = (int)frame.size() - 1; i >= 0; i--) {
	if (frame[i].name != string(name)) {
	    frame.erase(frame.begin() + i);
	    count += 1;
	}
    }

    return count;
}

int nonMaximalSuppression(svlObject2dFrame& frame, double dx, double dy, double ds, double dsArea)
{
    double threshold = (1.0 - dx) * (1.0 - dy);

    // decide which objects to include in the output
    vector<bool> includeInOutput(frame.size(), true);
    for (unsigned i = 0; i < frame.size(); i++) {
	double areaA = frame[i].area();
	for (unsigned j = (unsigned)frame.size() - 1; j > i; j--) {
	    if (frame[i].name != frame[j].name) {
		continue;
	    }
	    double areaOverlap = frame[i].overlap(frame[j]);
	    double areaB = frame[j].area();

#if 1
	    // first check same scale, otherwise check neighbouring scale
	    if ((areaA == areaB) && (areaOverlap > threshold * areaA)) {
		if (frame[i].pr < frame[j].pr) {
		    includeInOutput[i] = false;
		} else {
		    includeInOutput[j] = false;
		}
	    } else if ((areaA > areaB * ds) && (areaB > areaA * ds) &&
		((areaOverlap >= areaA * dsArea) || (areaOverlap >= areaB * dsArea))) {
		if (frame[i].pr < frame[j].pr) {
		    includeInOutput[i] = false;
		} else {
		    includeInOutput[j] = false;
		}		    
	    }
#else
	    // use area overlap measure = intersection / union
	    areaOverlap = areaOverlap / (areaA + areaB - areaOverlap);
	    if (areaOverlap > dsArea) {
		if (frame[i].pr < frame[j].pr) {
		    includeInOutput[i] = false;
		} else {
		    includeInOutput[j] = false;
		}	    		
	    }
#endif
	}
    }
    
    // remove suppressed frames
    for (int i = (int)includeInOutput.size() - 1; i >= 0; i--) {
	if (!includeInOutput[i]) {
	    frame.erase(frame.begin() + i);
	}
    }

    return (int)(includeInOutput.size() - frame.size());
}

// svlObject2dVideo Class --------------------------------------------------------

bool writeObject2dVideo(const char *filename, const svlObject2dVideo& v)
{
    ofstream ofs(filename);
    if (ofs.fail()) return false;

    ofs << "<Object2dVideo version=\"1.0\">" << endl;
    for (unsigned i = 0; i < v.size(); i++) {
        if (!v[i].empty()) {
	    writeObject2dFrame(ofs, v[i], i);
        }
    }
    ofs << "</Object2dVideo>" << endl;

    ofs.close();
    return true;
}

bool readObject2dVideo(const char *filename, svlObject2dVideo& v)
{
    cerr << "NOT IMPLEMENTED" << endl;
    assert(false);
    return false;
}

int countObjects(const svlObject2dVideo& v)
{
    int count = 0;
    for (unsigned i = 0; i < v.size(); i++) {
	    count += (int)v[i].size();
    }

    return count;
}

int removeOverlappingObjects(svlObject2dVideo& v, double threshold)
{
    int count = 0;
    for (unsigned i = 0; i < v.size(); i++) {
	count += removeOverlappingObjects(v[i], threshold);
    }

    return count;
}

int nonMaximalSuppression(svlObject2dVideo& v, double dx, double dy, double ds, double dsArea)
{
    int count = 0;   
    for (unsigned i = 0; i < v.size(); i++) {
	count += nonMaximalSuppression(v[i], dx, dy, ds, dsArea);
    }

    return count;
}

// removed objects that don't appear in either the previous or next frame
int temporalObjectFiltering(svlObject2dVideo& v, double da)
{
    int count = 0;
    for (unsigned i = 1; i < v.size() - 1; i++) {
	for (int j = (int)v[i].size() - 1; j >= 0; j--) {
	    double areaA = v[i][j].area();
	    bool bMatched = false;
	    for (int k = 0; k < (int)v[i-1].size(); k++) {
		if (v[i][j].name != v[i-1][k].name) {
		    continue;
		}
		double areaOverlap = v[i][j].overlap(v[i-1][k]);
		double areaB = v[i-1][k].area();
		if ((areaOverlap > da * areaA) &&
		    (areaOverlap > da * areaB)) {
		    bMatched = true;
		    break;
		}
	    }

	    if (!bMatched) {
		for (int k = 0; k < (int)v[i+1].size(); k++) {
		    if (v[i][j].name != v[i+1][k].name) {
			continue;
		    }
		    double areaOverlap = v[i][j].overlap(v[i+1][k]);
		    double areaB = v[i+1][k].area();
		    if ((areaOverlap > da * areaA) &&
			(areaOverlap > da * areaB)) {
			bMatched = true;
			break;
		    }
		}
	    }
	    
	    if (!bMatched) {
		v[i].erase(v[i].begin() + j);
		count += 1;
	    }
	}
    }

    return count;
}

void scaleObject2dVideo(svlObject2dVideo &v, double x_scale, double y_scale) {
  for (unsigned i = 0; i < v.size(); i++) {
    for (unsigned j = 0; j < v[i].size(); j++) {
      v[i][j].scale(x_scale, y_scale);
    }
  }
}

