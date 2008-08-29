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
** FILENAME:    svlImageSequence.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Defines an image sequence. The idea is that this can be used as a
**  surrogate video.
**
*****************************************************************************/

#pragma once

#include <vector>
#include <iostream>
#include <string>

#include "svlBase.h"

using namespace std;

class svlImageSequence {
 public:
    svlImageSequence();
    svlImageSequence(const svlImageSequence& s);
    virtual ~svlImageSequence();

    void load(const char *filename);
    void save(const char *filename) const;
    void dir(const char *directory, const char *extension = ".jpg");

    void clear() { _imageNames.clear(); }
    bool empty() const { return _imageNames.empty(); }
    unsigned size() const { return (unsigned)_imageNames.size(); }
    void push_back(const string& s) { _imageNames.push_back(s); }
    void pop_back() { _imageNames.pop_back(); }

    // Returns the i-th image. Calling function should not free the memory.
    const IplImage *image(unsigned index);

    // Returns the directory name.
    const string& directory() const { return _directoryName; }
    void setDirectory(string directoryName) { _directoryName = directoryName; }

    // Returns the i-th's image name.
    const string operator[](unsigned i) const { return _imageNames[i]; }
    string& operator[](unsigned i) { return _imageNames[i]; }

 protected:
    string _directoryName;
    vector<string> _imageNames;
    IplImage *_imageBuffer;
};

