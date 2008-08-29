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
** FILENAME:    objectList.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#define _CRT_SECURE_NO_DEPRECATE
#include <windows.h>
#undef max
#undef far
#undef near
#endif

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <cassert>
#include <string>

#include "xmlParser/xmlParser.h"

#include "svlBase.h"
#include "svlVision.h"

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

bool writeObject2dFrame(ostream &os, const svlObject2dFrame& v, const char *id)
{
    if (os.fail()) return false;

    os << "    <Object2dFrame id=\"" << (id == NULL ? "NULL" : id) << "\">" << endl;
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

svlObject2dFrame sortObjects(const svlObject2dFrame& frame)
{
    // sort detections by score (highest first)
    vector<pair<double, int> > sortIndex;
    sortIndex.reserve(frame.size());
    for (unsigned i = 0; i < frame.size(); i++) {
        sortIndex.push_back(make_pair(-frame[i].pr, i));
    }
    stable_sort(sortIndex.begin(), sortIndex.end());

    svlObject2dFrame sortedFrame;
    sortedFrame.reserve(frame.size());
    for (unsigned i = 0; i < sortIndex.size(); i++) {
        sortedFrame.push_back(frame[sortIndex[i].second]);
    }
    
    return sortedFrame;
}

// svlObject2dSequence Class --------------------------------------------------------

bool writeObject2dSequence(const char *filename, const svlObject2dSequence& v)
{
    ofstream ofs(filename);
    if (ofs.fail()) return false;

    ofs << "<Object2dSequence version=\"1.0\">" << endl;
    
    for (svlObject2dSequence::const_iterator it = v.begin(); it != v.end(); ++it) {
        writeObject2dFrame(ofs, it->second, it->first.c_str());
    }
    ofs << "</Object2dSequence>" << endl;

    ofs.close();
    return true;
}

bool readObject2dSequence(const char *filename, svlObject2dSequence& v)
{
    v.clear();

    XMLNode root = XMLNode::parseFile(filename, "Object2dSequence");
    if (root.isEmpty()) {
		return false;
    }

    for (int i = 0; i < root.nChildNode("Object2dFrame"); i++) {
	    XMLNode node = root.getChildNode("Object2dFrame", i);
	    if (node.isEmpty() || !(node.getAttribute("id"))) continue;
	    string id = string(node.getAttribute("id"));
            if (v.find(id) != v.end()) {
                SVL_LOG(SVL_LOG_WARNING, "duplicate id \"" << id << "\" in " << filename);
	            continue;
	        }
	
	    v[id].resize(node.nChildNode("Object"));
	    for (int j = 0; j < node.nChildNode("Object"); j++) {
	        XMLNode objNode = node.getChildNode("Object", j);
    	    v[id][j].name = objNode.getAttribute("name");
	        v[id][j].x = atof(objNode.getAttribute("x"));
	        v[id][j].y = atof(objNode.getAttribute("y"));
    	    v[id][j].w = atof(objNode.getAttribute("w"));
	        v[id][j].h = atof(objNode.getAttribute("h"));
            if (v[id][j].w < 0.0) { 
                v[id][j].x += v[id][j].w;
                v[id][j].w = -v[id][j].w;
            }
            if (v[id][j].h < 0.0) { 
                v[id][j].y += v[id][j].h;
                v[id][j].h = -v[id][j].h;
            }
	        if (objNode.getAttribute("pr")) {
		        v[id][j].pr = atof(objNode.getAttribute("pr"));
    	    } else {
	    	    v[id][j].pr = 1.0;
	        }
	    }
    }

    return true;
}

int countObjects(const svlObject2dSequence& v)
{
    int count = 0;
    for (svlObject2dSequence::const_iterator it = v.begin(); it != v.end(); ++it) {
        count += (int)it->second.size();
    }

    return count;
}

int removeOverlappingObjects(svlObject2dSequence& v, double threshold)
{
    int count = 0;
    for (svlObject2dSequence::iterator it = v.begin(); it != v.end(); ++it) {
	count += removeOverlappingObjects(it->second, threshold);
    }

    return count;
}

int nonMaximalSuppression(svlObject2dSequence& v, double dx, double dy, double ds, double dsArea)
{
    int count = 0;   
    for (svlObject2dSequence::iterator it = v.begin(); it != v.end(); ++it) {
	count += nonMaximalSuppression(it->second, dx, dy, ds, dsArea);
    }

    return count;
}

void scaleObject2dSequence(svlObject2dSequence &v, double x_scale, double y_scale) {
    for (svlObject2dSequence::iterator it = v.begin(); it != v.end(); ++it) {
        for (unsigned j = 0; j < it->second.size(); j++) {
            it->second[j].scale(x_scale, y_scale);
        }
    }
}

bool removeFramesFromSequence(svlObject2dSequence &v, const char *filename) {
    // decide which of the current frames to keep
    map<string, bool> keep;
    for (svlObject2dSequence::const_iterator it = v.begin(); it != v.end(); ++it) {
        keep[it->first] = false;
    }

    // parse an xml results file that contains only a subset of the frames
    XMLNode root = XMLNode::parseFile(filename, "Object2dSequence");
    if (root.isEmpty()) {
        return false;
    }

    for (int i = 0; i < root.nChildNode("Object2dFrame"); i++) {
        XMLNode node = root.getChildNode("Object2dFrame", i);
        if (node.isEmpty() || !(node.getAttribute("id"))) continue;
        
        string id = string(node.getAttribute("id"));
        if (v.find(id) != v.end())
            keep[id] = true;
    }  
    
    // delete all frames from video that weren't present in the given file
    for (svlObject2dSequence::iterator it = v.begin(); it != v.end(); ++it) {
        if (keep[it->first]) continue;
        v.erase(it);
    }
    
    return true;
}


