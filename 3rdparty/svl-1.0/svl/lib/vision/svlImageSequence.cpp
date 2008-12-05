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
** FILENAME:    svlImageSequence.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <cassert>
#include <string>

#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "opencv/highgui.h"

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "win32/dirent.h"
#else
#include <dirent.h>
#endif

#include "xmlParser/xmlParser.h"

#include "svlBase.h"
#include "svlImageSequence.h"

svlImageSequence::svlImageSequence() : 
    _directoryName("."), _imageBuffer(NULL)
{
    // do nothing
}

svlImageSequence::svlImageSequence(const svlImageSequence& s) :
    _directoryName(s._directoryName), _imageNames(s._imageNames),
    _imageBuffer(NULL)
{
    // do nothing
}

svlImageSequence::~svlImageSequence()
{
    if (_imageBuffer != NULL)
	cvReleaseImage(&_imageBuffer);
}

void svlImageSequence::load(const char *filename)
{
    assert(filename != NULL);

    XMLNode root = XMLNode::parseFile(filename, "ImageSequence");
    if (root.isEmpty()) {
        SVL_LOG(SVL_LOG_WARNING, "image sequence " << filename << " is empty");
        return;
    }

    if (root.getAttribute("dir") != NULL) {
        _directoryName = string(root.getAttribute("dir")) + string("/");
    } else {
        _directoryName = "";
    }

    for (int i = 0; i < root.nChildNode("Image"); i++) {
	XMLNode node = root.getChildNode("Image", i);
	string name = node.getAttribute("name");
        _imageNames.push_back(name);
    }
}

void svlImageSequence::save(const char *filename) const
{
    assert(filename != NULL);
    ofstream ofs(filename);
    ofs << "<ImageSequence\n"
        << "  dir=\"" << _directoryName << "\"\n"
        << "  version=\"1.0\">\n";
    for (unsigned i = 0; i < _imageNames.size(); i++) {
	ofs << "  <Image name=\"" << _imageNames[i] << "\"/>\n";
    }
    ofs << "</ImageSequence>\n";
    ofs.close();
}

void svlImageSequence::dir(const char *directory, const char *extension)
{
    assert(directory != NULL);

    DIR *dir = opendir(directory);
    assert(dir != NULL);

    _directoryName = string(directory) + string("/");
    struct dirent *e = readdir(dir);
    while (e != NULL) {	
        if (strstr(e->d_name, extension) != NULL) {
            _imageNames.push_back(string(e->d_name));
        }
        e = readdir(dir);
    }

    // sort image names, so that sequence corresponds to frame index
    sort(_imageNames.begin(), _imageNames.end());
    
    closedir(dir);
}

const IplImage *svlImageSequence::image(unsigned index)
{
    assert(index < _imageNames.size());
    if (_imageBuffer != NULL)
	cvReleaseImage(&_imageBuffer);
    string filename = _directoryName + _imageNames[index];
    _imageBuffer = cvLoadImage(filename.c_str());
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
    cvFlip(_imageBuffer, NULL, 0);
#endif

#if 0
    cvNamedWindow("debug", 1);
    cvShowImage("debug", _imageBuffer);
    cvWaitKey(-1);
    cvDestroyWindow("debug");
#endif

    return _imageBuffer;
}

