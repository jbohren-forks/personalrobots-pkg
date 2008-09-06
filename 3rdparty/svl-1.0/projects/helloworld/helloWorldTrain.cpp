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
** FILENAME:    helloWorldTrain.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

// C++ Standard Headers
#include <cstdlib>
#include <cassert>
#include <string>

// Open CV Headers
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

// SVL Headers
#include "svlBase.h"
#include "svlML.h"
#include "svlVision.h"

// Project Headers
#include "helloWorldModel.h"

using namespace std;

// usage ---------------------------------------------------------------------

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./helloWorldTrain <filestem> <imageSequence> <imageLabels>\n"
         << "  Assumes model files are named <filestem>.dictionary\n"
         << "  and <filestem>.model. Writes learned parameters to\n"
         << "  <filestem>.logistic.\n\n";
}

// main ----------------------------------------------------------------------

int main(int argc, char *argv[])
{
    if (argc != 4) {
        usage();
        return -1;
    }

    const char *modelFilestem = argv[1];
    const char *imageSeqFilename = argv[2];
    const char *imageLabelsFilename = argv[3];

    // instantiate model
    HelloWorldModel model;
    model.loadDetector(modelFilestem);

    // load image sequence and groundtruth labels
    svlImageSequence imageSequence;
    imageSequence.load(imageSeqFilename);
    svlObject2dSequence groundtruthLabels;
    readObject2dSequence(imageLabelsFilename, groundtruthLabels);

    // train the detector
    model.learnModel(imageSequence, groundtruthLabels);

    // save parameters
    model.saveLogistic((string(modelFilestem) + string(".logistic")).c_str());

    return 0;
}
