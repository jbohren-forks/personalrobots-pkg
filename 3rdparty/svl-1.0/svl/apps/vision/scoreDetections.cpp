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
** FILENAME:    scoreDetections.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**   Given two XML object files, computes the precision and recall.
*****************************************************************************/

#include <cstdlib>
#include <string>
#include <limits>
#include <iomanip>
#include <fstream>
#include <map>
#include <vector>

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

// type definitions and function prototypes ----------------------------------

typedef map<string, vector<pair<double, int> > > TObjectScores;

void accumulateScores(const svlObject2dSequence& gtObjectList, 
    const svlObject2dSequence& objectList, 
    bool ignoreMissingFrames, double areaRatio,
    TObjectScores& objectScores);

// main ----------------------------------------------------------------------

void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./scoreDetections [OPTIONS] (<groundtruth xml> <results xml>)+" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -area <num>       :: area overlap for positive hit (default 0.5)" << endl
	 << "  -threshold <num>  :: threshold for positive classification (default 0.5)" << endl
	 << "  -pr <filestem>    :: generate precision-recall curves" << endl
	 << "  -prPoints <num>   :: number of points on p-r curves (default: 100)" << endl
	 << "  -o <filestem>     :: output detections (true and false)" << endl
	 << "  -match            :: ignores frames that occur in one file but not the other" << endl
         << SVL_STANDARD_OPTIONS_USAGE
	 << endl;
}

int main(int argc, char *argv[])
{
    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 2;

    const char *gtFilename = NULL;
    const char *xmlFilename = NULL;

    const char *prFilestem = NULL;
    const char *outputFilestem = NULL;
    double areaRatio = 0.5;
    double threshold = 0.5;
    double prPoints = 100;
    bool onlyMatchingFrames = false;

    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) {
        SVL_PROCESS_STANDARD_OPTIONS(args, argc)
	else if (!strcmp(*args, "-area")) {
	    areaRatio = atof(*(++args)); argc--;
	} else if (!strcmp(*args, "-threshold")) {
	    threshold = atof(*(++args)); argc--;
 	} else if (!strcmp(*args, "-pr")) {
	    prFilestem = *(++args); argc--;
 	} else if (!strcmp(*args, "-prPoints")) {
	    prPoints = atoi(*(++args)); argc--;
	    assert(prPoints > 1);
	} else if (!strcmp(*args, "-o")) {
	    outputFilestem = *(++args); argc--;
	} else if (!strcmp(*args, "-match")) {
            onlyMatchingFrames = true;
	} else if (**args == '-') {
            SVL_LOG(SVL_LOG_FATAL, "unrecognized option " << *args);
	} else {
	    break;
	}
	args++;
    }

    if (argc < NUM_REQUIRED_PARAMETERS) {
	usage();
	return -1;
    }

    // Accumulate counts per object class, e.g. objectScores["mug"]
    // holds the score of every mug detection and whether it is a
    // true positive (1) or not (0).
    TObjectScores objectScores;

    for (unsigned i = 0; i < (unsigned)(argc / 2); i++) 
	{
        svlObject2dSequence gtObjectList;
        svlObject2dSequence objectList;

	gtFilename = args[2 * i];
	xmlFilename = args[2 * i + 1];

	SVL_LOG(SVL_LOG_VERBOSE, "Reading from " << gtFilename << " and " << xmlFilename);
	
	if (!readObject2dSequence(gtFilename, gtObjectList)) 
	{
            SVL_LOG(SVL_LOG_ERROR, "...failed to read " << gtFilename);
            continue;
        }
	if (!readObject2dSequence(xmlFilename, objectList)) {
            SVL_LOG(SVL_LOG_ERROR, "...failed to read " << xmlFilename);
            continue;
        }

	SVL_LOG(SVL_LOG_VERBOSE, gtObjectList.size() << " frames in ground truth");
	SVL_LOG(SVL_LOG_VERBOSE, objectList.size() << " frames in results");

	int total = countObjects(objectList);
	int removed = nonMaximalSuppression(objectList, 0.25, 0.25, 0.64);
	SVL_LOG(SVL_LOG_VERBOSE, removed << " out of " << total << " non-maximal objects suppressed");

 	// output detections
	if (outputFilestem) {

        svlObject2dSequence outputObjects;

        for (svlObject2dSequence::const_iterator it = objectList.begin();
                 it != objectList.end(); ++it) {
		for (unsigned k = 0; k < it->second.size(); k++) {
		    if (it->second[k].pr >= threshold) {
			outputObjects[it->first].push_back(it->second[k]);
		    }
		}
	    }
	    
	    string filename = string(outputFilestem) + string(".") + toString(i) + string(".xml");
	    writeObject2dSequence(filename.c_str(), outputObjects);
	}

        // accumulate counts
        accumulateScores(gtObjectList, objectList,
            onlyMatchingFrames, areaRatio, objectScores);
    }

    
    // show results
    cout << endl << "OBJECT\tCOUNT\tHIT\tERROR\tRECALL\tPREC.\tF1(" << threshold << ")" << endl;
    
    for (TObjectScores::iterator it = objectScores.begin(); 
         it != objectScores.end(); ++it) 
    {
        
        int totalPositives = 0;
        int totalDetections = 0;
        int trueDetections = 0;
        for (unsigned i = 0; i < it->second.size(); i++) 
        {
            if (it->second[i].first > threshold) 
            {
                totalDetections += 1;                
                trueDetections += it->second[i].second;
            }
            totalPositives += it->second[i].second;
        }

        double recall = (double)trueDetections/(double)totalPositives;
        double precision = (double)trueDetections/(double)totalDetections;

        cout << it->first << "\t" << totalPositives << "\t"
             << trueDetections << "\t" << (totalDetections - trueDetections) << "\t"
             << setprecision(3) << recall << "\t"
             << setprecision(3) << precision << "\t"
             << setprecision(3) << (2.0 * precision * recall / (precision + recall)) << endl;
    }

    // construct PR curve
    if (prFilestem != NULL) {
	SVL_LOG(SVL_LOG_VERBOSE, "Generating precision-recall curves...");

        for (TObjectScores::iterator it = objectScores.begin(); 
             it != objectScores.end(); ++it) {
            
            // sort scores
            sort(it->second.begin(), it->second.end());
            vector<int> cumsumDetections(it->second.size(), it->second.front().second);
            for (unsigned i = 1; i < cumsumDetections.size(); i++) {
                cumsumDetections[i] = cumsumDetections[i - 1] + 
                    it->second[i].second;
            }

#if 0
            SVL_LOG(SVL_LOG_DEBUG, it->first << " scores:");
            for (unsigned i = 0; i < it->second.size(); i++) {
                SVL_LOG(SVL_LOG_DEBUG, it->second[i].first << " "
                    << it->second[i].second << " " << cumsumDetections[i]);
            }
#endif

            SVL_LOG(SVL_LOG_DEBUG, "total " << it->first << ": " << toString(cumsumDetections.back()));

            string filename = string(prFilestem) + string(".") +
                it->first + string(".txt");

            ofstream ofs(filename.c_str());
            assert(!ofs.fail());

            ofs << "THRESHOLD, RECALL, PRECISION\n";
            double totalPositives = (double)cumsumDetections.back();
            double delta = (double)it->second.size() / (double)(prPoints + 1);
            for (int i = 0; i < prPoints; i++) {
                int indx = (int)((i + 1) * delta);
                double t = it->second[indx].first;
                while ((indx > 0) && (it->second[indx - 1].first == t)) {
                    indx -= 1;
                }
                double missedPositives = (double)cumsumDetections[indx];
                double totalDetections = (double)(cumsumDetections.size() - indx);
                ofs << t << ", " 
                    << (totalPositives - missedPositives) / totalPositives << ", "
                    << (double)(totalPositives - missedPositives) / totalDetections << "\n"; 
            }

            ofs.close();
        }

	SVL_LOG(SVL_LOG_VERBOSE, "...done");
    }

    return 0;
}

// private functions ---------------------------------------------------------

void accumulateScores(const svlObject2dSequence& gtObjectList, 
    const svlObject2dSequence& objectList, 
    bool ignoreMissingFrames, double areaRatio,
    TObjectScores& objectScores)
{
    for (svlObject2dSequence::const_iterator itg = gtObjectList.begin();
         itg != gtObjectList.end(); ++itg) {
        
        const svlObject2dFrame& groundTruthObjects = itg->second;

        svlObject2dSequence::const_iterator itd = objectList.find(itg->first);
        if (itd == objectList.end()) {
            if (!ignoreMissingFrames) {
                // notch up missing for all objects in the frame
                for (unsigned i = 0; i < groundTruthObjects.size(); i++) {
                    objectScores[groundTruthObjects[i].name].push_back(make_pair(0.0, 1));
                }
            }
            continue;
        }

        const svlObject2dFrame detectedObjects = itd->second;

        // list of objects already matched
        vector<bool> alreadyMatched(groundTruthObjects.size(), false);

        // sort detections by score
        vector<pair<double, int> > sortedDetections;
        for (unsigned i = 0; i < detectedObjects.size(); i++) {
            sortedDetections.push_back(make_pair(-detectedObjects[i].pr, i));
        }
        stable_sort(sortedDetections.begin(), sortedDetections.end());
        
        // find overlapping detections
        for (unsigned i = 0; i < sortedDetections.size(); i++) {
            bool bMatched = false;
            svlObject2d detection = detectedObjects[sortedDetections[i].second];
            for (unsigned j = 0; j < alreadyMatched.size(); j++) {
                if (alreadyMatched[j])
                    continue;
                if (groundTruthObjects[j].name != detection.name)
                    continue;
                if (groundTruthObjects[j].areaOverlap(detection) < areaRatio)
                    continue;
                
                // we have a match
                bMatched = true;
                alreadyMatched[j] = true;
                break;
            }
            
            objectScores[detection.name].push_back(make_pair(detection.pr, bMatched ? 1 : 0));
        }

        // all detections not matched by now are misses
        for (unsigned i = 0; i < alreadyMatched.size(); i++) {
            if (!alreadyMatched[i]) {
                objectScores[itg->second[i].name].push_back(make_pair(0.0, 1));
            }
        }
    }
}


