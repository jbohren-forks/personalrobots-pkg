/*********************************************************************
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author: Min Sun

#include "highgui.h"
#include "cv.h"
#include "cvaux.h"
#include "cxcore.h"
#include "write.h"
#include "RandomForest.h"

using namespace cv;
using namespace std;

template<class T> struct index_cmp_v2{
    index_cmp_v2(const T arr) : arr(arr) {}
    bool operator()(const size_t a, const size_t b) const
    { return arr[a] > arr[b]; }
    const T arr;
};

struct SimilarRects
{
    SimilarRects(double _eps) : eps(_eps) {}
    inline bool operator()(const Rect& r1, const Rect& r2) const
    {
        double delta = eps*(std::min(r1.width, r2.width) + std::min(r1.height, r2.height))*0.5;
        return std::abs(r1.x - r2.x) <= delta &&
            std::abs(r1.y - r2.y) <= delta &&
            std::abs(r1.x + r1.width - r2.x - r2.width) <= delta &&
            std::abs(r1.y + r1.height - r2.y - r2.height) <= delta;
    }
    double eps;
};

inline void cast_vots_pcd( Mat& Img, Vector<Rect>& part_rect, Vector<Point>& treeInfo, vector<float>& conf,
    vector< int>& scale_count, vector< vector<float> >& NormVotes, vector<int>& ViewIds, int MaxViewId, 
    int nCandidatePerDim, Size winStride, float winHeight2sinStrideRatio, vector<Size> winSize, int MeanShiftMaxIter,
    Vector<Rect> & ObjCenterWindowsAll, vector<float> & ObjCenterConfAll,  vector<int>& ObjCenterClassIdsAll, bool VisualizationFlag,
    CRandomForest& rf, vector<int>& unique_viewIds, int ObjHeightStep, double ObjASStep, 
    int MaxObjHeightBin, int MinObjHeightBin, int MaxObjASBin, int MinObjASBin, double ConfThre,
    vector< CvMat* >& VotesMap,
    vector< IplImage* >& MatObjHeightBin, vector< IplImage* >& MatObjASBin, float RectOverlapRatio)
{
    cout << "in cast_vots" << endl;
    // magic number zone
    int LowResMapSizePerDim = 2*nCandidatePerDim;// in each dim twice the dimesion, so in total 4 times of the num of candidates
    int LowResMapSizeArea = LowResMapSizePerDim*LowResMapSizePerDim;
    int MeanShiftwindow2winStrideRatio = 5;// window.width/winStride.width = 5;
//    float ObjHeight2MeanShiftwindowRatio = winHeight2sinStrideRatio/(float)(MeanShiftwindow2winStrideRatio);
    CvTermCriteria TermCriteria_ = cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, MeanShiftMaxIter, 0.1 );// MeanShift stopping criteria
//    float RectOverlapRatio = 0.3;// non-maximum suppression

    //parameter setting
    int nPatches = ViewIds.size() ;
    int nSamples = part_rect.size();
    //Size ImgSize = cvGetSize(&CvMat(Img));
    Size ImgSize = Img.size();

    int nScale = scale_count.size();
    vector<int> acc_scale_count;
    acc_scale_count.push_back(0);
    for (int i = 0; i<nScale; i++){
        acc_scale_count.push_back( acc_scale_count.at(i)+scale_count.at(i));
    }
    cout<< "NScale " << nScale << " MaxViewId" <<MaxViewId<<endl;

    // sample level variables
    int treeId;
    int nodeId;
    Rect sinelge_rect;
    float single_conf;
    int scaleId = 0;
    // patch level variables
    vector<int> * patchIdptr;
    int patchId;
    int viewId;
    CvPoint SubVote;
    //float AspectRatio;
    vector<float> VoteWeights;
    VoteWeights.resize(MaxViewId);
    float* ptr;
    Point LowResInd;
    vector<float> LowResStep;
    LowResStep.push_back((float)ImgSize.width/LowResMapSizePerDim);
    LowResStep.push_back((float)ImgSize.height/LowResMapSizePerDim);

    // variables for prunning
    Vector<int> labels;
    int nclasses;
    int nlabels;
    Vector<Rect> rrects;
    vector<float> rweights;
    vector<float> AspectRatioCache;
    vector<float> ObjHeightCache;
    vector<float>* vecLowResVotesPtr; // redundent for debug only

    vector< vector< float> > vecLowResVotes;
    vector< int> SampleCount;
    vecLowResVotes.resize(MaxViewId);
    SampleCount.resize(MaxViewId,0);
    for (unsigned int k=0; k< unique_viewIds.size(); k++){
        int valid_viewId = unique_viewIds[k];
        cvZero( VotesMap.at(valid_viewId) );
        vecLowResVotes.at(valid_viewId).resize(LowResMapSizeArea,0);
        cvZero( MatObjHeightBin[valid_viewId] );
        cvZero( MatObjASBin[valid_viewId] );
    }
    cout << "finish building vote space" <<endl;

    int ObjHeightBin;
    int ObjASBin;

    scaleId = 0;
    cout << "constructing vote map" << endl;
    for (int SampleId = 0; SampleId < nSamples; SampleId++)// loop through samples
    {
        treeId = treeInfo[SampleId].x;
        nodeId = treeInfo[SampleId].y;
        sinelge_rect = part_rect[SampleId];
        single_conf = conf[SampleId];
        for (int k=0; k < MaxViewId; k++)
            VoteWeights[k] = 0;
        patchIdptr = rf.get_vecpatchIds(treeId, nodeId);

        // increment scaleId
        if (SampleId >= acc_scale_count[scaleId+1])
            scaleId++;

        //cout << "scaleId" << scaleId << endl;

        // count viewId
        int SumviewIdCount = 0;
        for (unsigned int PatchCount = 1; PatchCount < patchIdptr->size(); PatchCount++)// loop through patches
        {
            patchId = patchIdptr->at(PatchCount);
            //cout << "patchId" << patchId <<endl;
            if (patchId >= (int)(ViewIds.size())){ // backgroud patch so skip
                continue;
            }
            viewId = ViewIds[ patchId]-1;
            if (viewId < 0) // backgroud patch so skip
                continue;
            SumviewIdCount++;
        }
        //cout << "finish SumviewIdCount" <<endl;

        for (int k=0; k < MaxViewId; k++){
            VoteWeights[k] = conf[SampleId]/(float)(SumviewIdCount);
        }
        //cout << "finish set VoteWeights" <<endl;

        for (unsigned int PatchCount = 1; PatchCount < patchIdptr->size(); PatchCount++)// loop through patches
        {
            patchId = patchIdptr->at(PatchCount);
            if (patchId >= nPatches)
            continue;
            viewId = ViewIds[ patchId]-1;

            if (viewId < 0) // backgroud patch so skip
                continue;

            SubVote.x = NormVotes.at(0).at(patchId)*sinelge_rect.width+sinelge_rect.width/2+sinelge_rect.x;
            SubVote.y = NormVotes.at(1).at(patchId)*sinelge_rect.height+sinelge_rect.height/2+sinelge_rect.y;

            if ( SubVote.x< 0 || SubVote.y < 0 || SubVote.x >= ImgSize.width || SubVote.y >= ImgSize.height){
                continue;
            }

            LowResInd.x = (int) SubVote.x/LowResStep[0];
            LowResInd.y = (int) SubVote.y/LowResStep[1];
            int LosResPtr = LowResInd.y*LowResMapSizePerDim + LowResInd.x;
            if (LosResPtr >=LowResMapSizeArea || viewId >= MaxViewId)
                cout << LosResPtr << endl;
            vecLowResVotes[viewId][LosResPtr] += VoteWeights[viewId];

            SampleCount[viewId]++;
            ptr =  (float*)( VotesMap[viewId]->data.ptr + SubVote.y*VotesMap[viewId]->step);
            ptr[SubVote.x] += VoteWeights[viewId];


            ObjASBin =  (int)(NormVotes.at(2).at(patchId)/ObjASStep);
            ptr = (float*) (MatObjASBin[viewId]->imageData + SubVote.y*MatObjASBin[viewId]->widthStep);
            ptr[MatObjASBin[viewId]->nChannels*SubVote.x+ObjASBin - MinObjASBin] += VoteWeights[viewId];
            //AspectRatio =  NormVotes.at(2).at(patchId);
            //CalMedian( AspectRatio, vecASMedianCandSmaller[viewId][SubVote.y * ImgSize.width + SubVote.x],
            //    vecASMedianCandBigger[viewId][SubVote.y * ImgSize.width + SubVote.x],
            //    vecASMedianCandSmallerFlag[viewId][SubVote.y * ImgSize.width + SubVote.x]);

            //cout << "start votes for object height" <<endl;
            // handle object height
            ObjHeightBin = (int)(NormVotes.at(3).at(patchId)*winSize[scaleId].height/ObjHeightStep);
            ptr = (float*) (MatObjHeightBin[viewId]->imageData + SubVote.y*MatObjHeightBin[viewId]->widthStep);
            ptr[MatObjHeightBin[viewId]->nChannels*SubVote.x+ObjHeightBin - MinObjHeightBin] += VoteWeights[viewId];
//            cout << ptr[MatObjHeightBin[viewId]->nChannels*SubVote.x+ObjHeightBin - MinObjHeightBin]<<endl;
            //CalMedian( ObjHeight, vecObjHeightMedianCandSmaller[viewId][SubVote.y * ImgSize.width + SubVote.x],
            //    vecObjHeightMedianCandBigger[viewId][SubVote.y * ImgSize.width + SubVote.x],
            //    vecObjHeightMedianCandSmallerFlag[viewId][SubVote.y * ImgSize.width + SubVote.x]);
            //cout << "finish votes for object height" <<endl;
        }

    }

    cout << "do mean shift" << endl;
    vector<int> LowResInds;
    LowResInds.resize(LowResMapSizeArea);
    CvConnectedComp comp;
    int count;
    Point BottomRightPt;
    int MeanShiftIter;

    CvRect window;
    window.width = winStride.width*MeanShiftwindow2winStrideRatio;//*winSize[scaleId].width/winSize.back().width;
    window.height = winStride.height*MeanShiftwindow2winStrideRatio;//*winSize[scaleId].width/winSize.back().width;

    for ( viewId = 0; viewId < MaxViewId; viewId++){
        // skip the one with zero sample
        count = SampleCount[viewId];
        if (count != 0){
            Vector<Rect> ObjCenterWindows;
            vector<float> ObjCenterConf;
            for (int i=0; i<LowResMapSizeArea; i++)
                LowResInds.at(i) = i;

            //LOOP THROUGH DIFFERENT initialize window
            vecLowResVotesPtr =& vecLowResVotes[viewId];
            sort( LowResInds.begin(), LowResInds.end(), index_cmp_v2<vector<float>&>(vecLowResVotes[viewId]) );
            for (int i = 0; i < (nCandidatePerDim*nCandidatePerDim); i++){
                LowResInd.x = (int) LowResInds[i]/LowResMapSizePerDim;
                LowResInd.y = (int) LowResInds[i] % LowResMapSizePerDim;
                window.x = LowResInd.x*LowResStep[0];
                window.y = LowResInd.y*LowResStep[1];
                // check ImgBoundary
                BottomRightPt.x = window.x+window.width;
                BottomRightPt.y = window.y+window.height;
                if ( BottomRightPt.x >=ImgSize.width || BottomRightPt.y >=ImgSize.height){
                    window.x -= (BottomRightPt.x-ImgSize.width +1);
                    window.y -= (BottomRightPt.y-ImgSize.height+1);
                }
                if (window.x< 0 || window.y < 0 || (window.x+window.width) >= ImgSize.width || (window.y+window.height) >= ImgSize.height)
                    cout << window.x <<" "<< window.y <<" "<< (window.x+window.width) << " " << (window.y+window.height) << endl;
                MeanShiftIter = cvMeanShift( VotesMap[viewId], window, TermCriteria_, &comp);

                ObjCenterWindows.push_back( Rect(comp.rect));
                ObjCenterConf.push_back( comp.area);
            }

            // do connected component within each class using a fix window size
            nclasses = partition(ObjCenterWindows, labels, SimilarRects( RectOverlapRatio));
            nlabels = (int)labels.size();
            rrects.clear();
            rrects.resize(nclasses);
            rweights.clear();
            rweights.resize(nclasses,0);
            vector<int> cache(nclasses,0);
            for( int i = 0; i < nlabels; i++ )
            {
                int cls = labels[i];
                if (rweights[cls]< ObjCenterConf[i]){
                    rrects[cls].x = ObjCenterWindows[i].x;
                    rrects[cls].y = ObjCenterWindows[i].y;
                    rrects[cls].width = ObjCenterWindows[i].width;
                    rrects[cls].height = ObjCenterWindows[i].height;
                    rweights[cls] = ObjCenterConf[i];
                }
            }
            for( int i = 0; i < nclasses; i++ )
            {
		if ( rweights[i] <= ConfThre)
			continue;

                ObjCenterConfAll.push_back( rweights[i]);// this is sum over all scale and AS
                vector<float> ObjHeightBin_(MaxObjHeightBin-MinObjHeightBin+1, 0);
                vector<float> ObjASBin_(MaxObjASBin-MinObjASBin+1, 0);
                for (int y = int(rrects[i].y); y< int(rrects[i].y+rrects[i].height); y++)
                    for (int x = int(rrects[i].x); x< int(rrects[i].x+rrects[i].width); x++){
                        for (unsigned int b = 0; b< ObjHeightBin_.size(); b++){
                            if (y >= MatObjHeightBin[viewId]->height ||
                                    x >= MatObjHeightBin[viewId]->width){
                            }
                            ptr = (float*)(MatObjHeightBin[viewId]->imageData + y*MatObjHeightBin[viewId]->widthStep);
                            ObjHeightBin_[b] += ptr[MatObjHeightBin[viewId]->nChannels*x+b];
                        }
                        for (unsigned int b = 0; b< ObjASBin_.size(); b++){
                            if (y >= MatObjASBin[viewId]->height ||
                                    x >= MatObjASBin[viewId]->width){
                            }
                            ptr = (float*)(MatObjASBin[viewId]->imageData + y*MatObjASBin[viewId]->widthStep);
                            ObjASBin_[b] += ptr[MatObjASBin[viewId]->nChannels*x+b];
                        }
                    }
                float BestObjASBin=0;
                float BestObjASScore=0;
                for ( unsigned int b=0; b< ObjASBin_.size(); b++){
                    if (BestObjASScore < ObjASBin_[b])
                    {
                        BestObjASBin = b;
                        BestObjASScore = ObjASBin_[b];
                    }
                }

                float BestObjHeightBin=0;
                float BestObjHeightScore=0;
                for ( unsigned int b=0; b< ObjHeightBin_.size(); b++){
                    if (BestObjHeightScore < ObjHeightBin_[b])
                    {
                        BestObjHeightBin = b;
                        BestObjHeightScore = ObjHeightBin_[b];
                    }
                }
                //ObjCenterConfAll.push_back( BestObjHeightScore*BestObjASScore);
                
		// set the actuall object windowsSize
            	Rect Objbbx;
            	Point ObjCenter( rrects[i].x+rrects[i].width/2, rrects[i].y+rrects[i].height/2);
            	Objbbx.height = (BestObjHeightBin+MinObjHeightBin)*ObjHeightStep;
            	Objbbx.width = Objbbx.height*(BestObjASBin+MinObjASBin)*ObjASStep;
            	Objbbx.x = ObjCenter.x-Objbbx.width/2;
            	Objbbx.y = ObjCenter.y-Objbbx.height/2;
                ObjCenterWindowsAll.push_back( Objbbx);
		ObjCenterClassIdsAll.push_back( viewId);
            }
        }
    }


        // do connected component to prune using object window across all class
        labels.clear();
        nclasses = partition(ObjCenterWindowsAll, labels, SimilarRects( RectOverlapRatio));
        nlabels = (int)labels.size();
        rrects.clear();
        rrects.resize(nclasses);
        rweights.clear();
        rweights.resize(nclasses,0);
	vector<int> rclassIds(nclasses,0);
        vector<int> cache(nclasses,0);
        for( int i = 0; i < nlabels; i++ )
        {
            int cls = labels[i];
            if (rweights[cls]< ObjCenterConfAll[i] ){
                rrects[cls].x = ObjCenterWindowsAll[i].x;
                rrects[cls].y = ObjCenterWindowsAll[i].y;
                rrects[cls].width = ObjCenterWindowsAll[i].width;
                rrects[cls].height = ObjCenterWindowsAll[i].height;
                rweights[cls] = ObjCenterConfAll[i];
		rclassIds[cls] = ObjCenterClassIdsAll[i];
            }
        }

        ObjCenterWindowsAll.clear();
        ObjCenterConfAll.clear();
	ObjCenterClassIdsAll.clear();
        for( int i = 0; i < nclasses; i++ )
        {
            ObjCenterWindowsAll.push_back( rrects[i]);
            ObjCenterConfAll.push_back( rweights[i]);
	    ObjCenterClassIdsAll.push_back( rclassIds[i]);
        }
}
