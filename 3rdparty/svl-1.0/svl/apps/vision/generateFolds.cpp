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
** FILENAME:    generateFolds.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
**
** DESCRIPTION:
**  Generates different folds for a data set based on images grouped into scenes

*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

#include <sys/stat.h>
#include <sys/types.h>
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "win32/dirent.h"
#include <windows.h>
#else
#include <dirent.h>
#endif

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlVision.h"

using namespace std;


void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./generateFolds [OPTIONS] <imageSeq> <foldLabels> <outputFolder>" << endl;
    cerr << "OPTIONS:" << endl
     << "  -foldName          :: prefix for the folders for each fold (default :: fold)" << endl
     << "  -createDir         :: controls creation of folders for each fold" << endl
     << "  -globalPath         :: global path for the data directory" << endl
     << "  -prefix         :: prefix for the names of the fold image sequences" << endl
	 << endl;
}

//Populates data about scene and various folds
int populateSceneData(const char *filename,vector<CvPoint> &ranges,vector<vector<int> > &foldLabels);

//Finds the folds to which this image belongs
void findFoldIndices(vector<CvPoint> ranges,vector<vector<int> > foldLabels,int totalFolds,int index , vector<int> &trainIndices,vector<int> &testIndices);

int main(int argc, char *argv[])
{
    // read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 3;
    
    int createDir = 0;
    const char *foldName = "fold";
    const char *imageSequencePath = NULL;
    const char *foldLabelsPath = NULL;
    const char *outputFolder = NULL;
    const char *globalPath = NULL;
    const char *prefix = NULL;
      
    char **args = argv + 1;
  
    while (--argc > NUM_REQUIRED_PARAMETERS) 
    {
	    if (!strcmp(*args, "-createDir")) 
        {
	        createDir = 1; 
	    } 
        else if (!strcmp(*args, "-foldName")) 
        {
	        foldName = *(++args); argc--;
	    } 
        else if (!strcmp(*args, "-globalPath")) 
        {
	        globalPath = *(++args); argc--;
	    } 
        else if (!strcmp(*args, "-prefix")) 
        {
	        prefix = *(++args); argc--;
	    } 

	    args++;
    }

    if (argc != NUM_REQUIRED_PARAMETERS) 
    {
	    usage();
	    return -1;
    }

    imageSequencePath = args[0];
    foldLabelsPath = args[1];
    outputFolder = args[2];
    
    

    vector<CvPoint> ranges;
    vector<vector<int> > foldLabels;

    //Populate scene data
    int totalFolds = populateSceneData(foldLabelsPath,ranges,foldLabels);
    
    cout<<"\n\nTotal folds\n"<<totalFolds;
    
    //Load dataset
    svlImageSequence imageSequence;
    imageSequence.load(imageSequencePath);    

    //image sequences for test and training data for all the folds
    vector<svlImageSequence> testSets(totalFolds);
    vector<svlImageSequence> trainSets(totalFolds);
    

    for( unsigned i=0 ; i<imageSequence.size() ; i++)
    {
        cout<<"\n Processing targets for image "<<i;
        vector<int> trainIndices;
        vector<int> testIndices;

        findFoldIndices(ranges,foldLabels,totalFolds,i,trainIndices,testIndices);

        for( unsigned j=0 ; j<trainIndices.size() ; j++)
        {
            printf(" %d" , trainIndices[j]);
            trainSets[trainIndices[j]].push_back(imageSequence[i]);
        }

        printf(" * ");
        for( unsigned j=0 ; j<testIndices.size() ; j++)
        {
            printf(" %d" , testIndices[j]);
            testSets[testIndices[j]].push_back(imageSequence[i]);
        }
        
    }

    //Save all folds to disk
    for( int i=0 ; i<totalFolds ; i++)
    {
        string dirName = string(outputFolder) + string("/") + string(foldName) + toString(i+1);

        if( createDir )
        {           
            
            #if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)

            CreateDirectory(dirName.c_str(),NULL);
            #else

	        mkdir(dirName.c_str(),0777);
            
            #endif
        }

        string trainFoldName = dirName + string("/") ;

        if( prefix == NULL )
            trainFoldName += string("trainInput.xml");
        else
            trainFoldName += string(prefix) + string("trainInput.xml");

        string testFoldName = dirName + string("/") ;

        if( prefix == NULL )
            testFoldName += string("testInput.xml");
        else
            testFoldName += string(prefix) + string("testInput.xml");

        if( globalPath == NULL )
        {
            trainSets[i].setDirectory( imageSequence.directory() );
            testSets[i].setDirectory( imageSequence.directory() );
        }
        else
        {
            trainSets[i].setDirectory( string(globalPath) );
            testSets[i].setDirectory(  string(globalPath) );
        }

        trainSets[i].save(trainFoldName.c_str());
        testSets[i].save(testFoldName.c_str());

    }//folds loop

    return 0;
}

//Finds the folds to which this image belongss
void findFoldIndices(vector<CvPoint> ranges,vector<vector<int> > foldLabels,int totalFolds,int index , vector<int> &trainIndices,vector<int> &testIndices)
{
    //Find corresponding scene for the current index
    for( unsigned i=0 ; i<ranges.size() ; i++)
    {
        if( index >= ranges[i].x && index <= ranges[i].y)
        {
            for( int j=0 ; j<totalFolds ; j++)
            {
                if( foldLabels[i][j] == -1 )
                {
                    trainIndices.push_back(j);
                }
                else if(foldLabels[i][j] == 1 )
                {
                    testIndices.push_back(j);
                }
            }
        }//if found
    }//ranges
}

//Populates scene and fold labels
int populateSceneData(const char *filename,vector<CvPoint> &ranges,vector<vector<int> > &foldLabels)
{
    //Open file stream
    ifstream inputStream(filename,ios::in);

    int totalScenes = -1 , totalFolds = -1;

    inputStream>>totalScenes>>totalFolds;

    
    //Iterate over each scene labels
    for( int i=0 ; i<totalScenes ; i++)
    {
        cout<<"\n Processing scene label "<<i+1;

        int stub, start , finish , foldMember ;

        inputStream>>stub>>start>>finish>>foldMember;

        ranges.push_back(cvPoint(start,finish));
       
         //Store indices of each fold this scene belongs to
        foldLabels.push_back(vector<int>(totalFolds));

        for( int j=0 ; j<totalFolds ; j++)
        {
            foldLabels[i][j] = -1;
        }

        for( int j=0 ; j<foldMember ; j++)
        {
            int index;
            inputStream>>index;
            foldLabels[i][index-1]=1;
        }

    }//total scene i

    //Cleanup
    inputStream.close();

    return totalFolds;
}
