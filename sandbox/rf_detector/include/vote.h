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

inline void CalMedian(float NewValue, float& SmallerValue, float& BiggerValue, bool SmallFlag){
    // we assume if Value = -Inf, it needs to initialized
    if (SmallerValue == numeric_limits<float>::max()){
        SmallerValue =NewValue;
        SmallFlag = true;
        return;
    }

    if (BiggerValue == numeric_limits<float>::max()){
        if (SmallerValue > NewValue){
            BiggerValue = SmallerValue;
            SmallerValue = NewValue;
        }else{
            BiggerValue = NewValue;
        }
        SmallFlag = false;
        return;
    }

    if (NewValue < SmallerValue){
        if (SmallFlag){
            BiggerValue = SmallerValue;
            SmallerValue = NewValue;
            SmallFlag = false;
        }else{
            SmallFlag = true;
        }
    }else if( NewValue < BiggerValue){
        if (SmallFlag){
            BiggerValue = NewValue;
            SmallFlag = false;
        }else{
            SmallerValue = NewValue;
            SmallFlag = true;
        }
    }else{
        if (SmallFlag){
            SmallFlag = false;
        }else{
            SmallerValue = BiggerValue;
            BiggerValue = NewValue;
            SmallFlag = true;
        }
    }

    return;
}

inline float getMedian( float SmallerValue, float BiggerValue, bool SmallFlag){
    if (SmallFlag){
        return SmallerValue;
    }else{
        return (SmallerValue+BiggerValue)/2;
    }
}

inline void AddWeighted( float* ptr1, float weight1, float* ptr2, float weight2, IplImage* des){
    float* ptrdes = (float*) des->imageData;
    for (int i = 0; i< des->width*des->height; i++)
        ptrdes[i] = weight1*ptr1[i]+weight2*ptr2[i];
}

inline void cast_vots( Mat& Img, Vector<Rect>& part_rect, Vector<Point>& treeInfo, vector<float>& conf,
    vector< int>& scale_count, vector< vector<float> >& NormVotes, vector<int>& ViewIds, int MaxViewId, vector<double>& AvergeKernel,
    int nCandidatePerDim, Size winStride, float winHeight2sinStrideRatio, float Scale, int MeanShiftMaxIter,
    vector< Vector<Rect> >& ObjCenterWindowsAll, vector< vector<float> > & ObjCenterConfAll, bool VisualizationFlag, 
    CRandomForest& rf)
{
    cout << "in cast_vots" << endl;
    // magic number zone
    int LowResMapSizePerDim = 2*nCandidatePerDim;// in each dim twice the dimesion, so in total 4 times of the num of candidates
    int LowResMapSizeArea = LowResMapSizePerDim*LowResMapSizePerDim;
    int MeanShiftwindow2winStrideRatio = 5;// window.width/winStride.width = 5;
    float ObjHeight2MeanShiftwindowRatio = winHeight2sinStrideRatio/(float)(MeanShiftwindow2winStrideRatio);
    CvTermCriteria TermCriteria_ = cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, MeanShiftMaxIter, 0.1 );// MeanShift stopping criteria
    float RectOverlapRatio = 0.5;// non-maximum suppression

    //parameter setting
    int nPatches = ViewIds.size() ;
    int nSamples = part_rect.size();
    Size ImgSize = cvGetSize(&CvMat(Img));
    int ImgArea = ImgSize.width*ImgSize.height;

    int nScale = scale_count.size();
    vector<int> acc_scale_count;
    acc_scale_count.push_back(0);
    for (int i = 0; i<nScale; i++){
        acc_scale_count.push_back( acc_scale_count.at(i)+scale_count.at(i));
    }
    cout<< "NScale " << nScale << " MaxViewId" <<MaxViewId<<endl;

    // find unique ViewIds
    vector<int> unique_viewIds;
    for (unsigned int i=0; i< ViewIds.size(); i++){
        bool unique_flag = true;
        for ( unsigned int  j = 0; j<unique_viewIds.size(); j++){
            if ((ViewIds[i]-1) == unique_viewIds[j]){
                unique_flag = false;
                break;
            }
        }
        if (unique_flag){
            unique_viewIds.push_back(ViewIds[i]-1);
            cout << "unique_viewId=" << ViewIds[i] << endl;
        }
    }

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
    float AspectRatio;
    vector<float> VoteWeights;
    VoteWeights.resize(MaxViewId);
    float* ptr;
    Point LowResInd;
    vector<float> LowResStep;
    LowResStep.push_back((float)ImgSize.width/LowResMapSizePerDim);
    LowResStep.push_back((float)ImgSize.height/LowResMapSizePerDim);

    CvRect window;
    window.width = winStride.width*MeanShiftwindow2winStrideRatio;
    window.height = winStride.height*MeanShiftwindow2winStrideRatio;

    vector< vector<int> > ScaleIdCacheAll;
    ScaleIdCacheAll.resize( MaxViewId);
    vector< vector<float> > AspectRatioCacheAll;
    AspectRatioCacheAll.resize( MaxViewId);
    ObjCenterWindowsAll.resize(MaxViewId);
    ObjCenterConfAll.resize(MaxViewId);

    // variables for prunning
    Vector<int> labels;
    int nclasses;
    int nlabels;
    Vector<Rect> rrects;
    vector<float> rweights;
    vector<float> ravg_weights;
    vector<int> ScaleIdCache;
    vector<float> AspectRatioCache;
    vector<float>* vecLowResVotesPtr; // redundent for debug only

    for (int scaleId = 0; scaleId < nScale; scaleId++){
//        cout << "start each scale setup" <<endl;
        vector< CvMat* > VotesMap;
        vector< CvMat* > VotesCount;
        vector< vector< float> > vecASMedianCandSmaller;
        vector< vector< float> > vecASMedianCandBigger;
        vector< vector< bool> > vecASMedianCandSmallerFlag;
        vector< vector< float> > vecLowResVotes;
        vector< int> SampleCount;
        VotesMap.resize(MaxViewId);
        VotesCount.resize(MaxViewId);
        vecASMedianCandSmaller.resize(MaxViewId);
        vecASMedianCandBigger.resize(MaxViewId);
        vecASMedianCandSmallerFlag.resize(MaxViewId);
        vecLowResVotes.resize(MaxViewId);
        SampleCount.resize(MaxViewId,0);
        for (int k=0; k<unique_viewIds.size(); k++){
            int valid_viewId = unique_viewIds[k];
            VotesMap.at(valid_viewId) = cvCreateMat( ImgSize.height, ImgSize.width, CV_32FC1);
            cvZero( VotesMap.at(valid_viewId) );
            VotesCount.at(valid_viewId) = cvCreateMat( ImgSize.height, ImgSize.width, CV_32FC1);
            cvZero( VotesCount.at(valid_viewId) );
            vecASMedianCandSmaller.at(valid_viewId).resize(ImgArea,numeric_limits<float>::max());
            vecASMedianCandBigger.at(valid_viewId).resize(ImgArea,numeric_limits<float>::max());
            vecASMedianCandSmallerFlag.at(valid_viewId).resize(ImgArea,true);
            vecLowResVotes.at(valid_viewId).resize(LowResMapSizeArea,0);
        }
//        cout << "end each scale setup" <<endl;

        cout << "constructing vote map" << endl;
        for (int SampleId = acc_scale_count[scaleId]; SampleId < acc_scale_count[scaleId+1]; SampleId++)// loop through samples
        {
            treeId = treeInfo[SampleId].x;
            nodeId = treeInfo[SampleId].y;
            sinelge_rect = part_rect[SampleId];
            single_conf = conf[SampleId];
            for (int k=0; k < MaxViewId; k++)
                VoteWeights[k] = 0;
	    patchIdptr = rf.get_vecpatchIds(treeId, nodeId);

            // count viewId
            int SumviewIdCount = 0;
            for (int PatchCount = 1; PatchCount < patchIdptr->size(); PatchCount++)// loop through patches
            {
                patchId = patchIdptr->at(PatchCount);
                if (patchId >= ViewIds.size()){ // backgroud patch so skip
                    continue;
                }
                viewId = ViewIds[ patchId]-1;
                if (viewId < 0) // backgroud patch so skip
                    continue;
                SumviewIdCount++;
            }

            for (int k=0; k < MaxViewId; k++){
                VoteWeights[k] = conf[SampleId]/(float)(SumviewIdCount);
            }

            for (int PatchCount = 1; PatchCount < patchIdptr->size(); PatchCount++)// loop through patches
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

                AspectRatio =  NormVotes.at(2).at(patchId);
                LowResInd.x = (int) SubVote.x/LowResStep[0];
                LowResInd.y = (int) SubVote.y/LowResStep[1];
                int LosResPtr = LowResInd.y*LowResMapSizePerDim + LowResInd.x;
                if (LosResPtr >=LowResMapSizeArea || viewId >= MaxViewId)
                    cout << LosResPtr << endl;
                vecLowResVotes[viewId][LosResPtr] += VoteWeights[viewId];

                SampleCount[viewId]++;
                ptr =  (float*)( VotesMap[viewId]->data.ptr + SubVote.y*VotesMap[viewId]->step);
                ptr[SubVote.x] += VoteWeights[viewId];
                ptr =  (float*)( VotesCount[viewId]->data.ptr + SubVote.y*VotesCount[viewId]->step);
                ptr[SubVote.x] ++;

                CalMedian( AspectRatio, vecASMedianCandSmaller[viewId][SubVote.y * ImgSize.width + SubVote.x],
                    vecASMedianCandBigger[viewId][SubVote.y * ImgSize.width + SubVote.x],
                    vecASMedianCandSmallerFlag[viewId][SubVote.y * ImgSize.width + SubVote.x]);
            }
        }

        cout << "do mean shift" << endl;
        vector<int> LowResInds;
        LowResInds.resize(LowResMapSizeArea);
        CvConnectedComp comp;
        int count;
        Point BottomRightPt;
        int MeanShiftIter;

        float Smaller;
        float Bigger;
        bool SmallerFlag;

        for ( viewId = 0; viewId < MaxViewId; viewId++){
            // skip the one with zero sample
            count = SampleCount[viewId];
            if (count != 0){
//                cout << "view="<< viewId<< endl;
                Vector<Rect> ObjCenterWindows;
                vector<float> ObjCenterConf;
                vector<float> ObjCenterAvgConf;
                for (int i=0; i<LowResMapSizeArea; i++)
                    LowResInds.at(i) = i;

                //LOOP THROUGH DIFFERENT initialize window
                vecLowResVotesPtr =& vecLowResVotes[viewId];
                sort( LowResInds.begin(), LowResInds.end(), index_cmp_v2<vector<float>&>(vecLowResVotes[viewId]) );
                for (int i = 0; i <  (nCandidatePerDim*nCandidatePerDim); i++){
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
                        //if (window.x<0){
                        //    window.width=+window.x;
                        //    window.x = 0;
                        //}
                        //if (window.y<0){
                        //    window.height=+window.y;
                        //    window.y = 0;
                        //}
                    }
                    if (window.x< 0 || window.y < 0 || (window.x+window.width) >= ImgSize.width || (window.y+window.height) >= ImgSize.height)
                        cout << window.x <<" "<< window.y <<" "<< (window.x+window.width) << " " << (window.y+window.height) << endl;
                    MeanShiftIter = cvMeanShift( VotesMap[viewId], window, TermCriteria_, &comp);
                    // verify comp
                    float division_count =0;
                    float check_area =0;
                    for (int y=comp.rect.y; y<(comp.rect.y+comp.rect.height);y++)
                        for (int x=comp.rect.x; x<(comp.rect.x+comp.rect.width);x++)
                        {
                            ptr =  (float*)( VotesCount[viewId]->data.ptr + y*VotesCount[viewId]->step);
                            division_count+=ptr[x];
                            ptr =  (float*)( VotesMap[viewId]->data.ptr + y*VotesMap[viewId]->step);
                            check_area+=ptr[x];
                        }
                    if (abs(check_area-comp.area)>1)
                        cout << "check_area=" << check_area << " comp.area=" << comp.area << endl;
                    //cout << "division_count" << division_count << endl;

                    ObjCenterWindows.push_back( Rect(comp.rect));
                    ObjCenterConf.push_back( comp.area);
                    //ObjCenterAvgConf.push_back( comp.area/division_count);
                    ObjCenterAvgConf.push_back( comp.area);
                }

//                cout << " start remove redundent rect" <<endl;
                // do connected component within each scale
                nclasses = partition(ObjCenterWindows, labels, SimilarRects( RectOverlapRatio));
                nlabels = (int)labels.size();
                rrects.clear();
                rrects.resize(nclasses);
                rweights.clear();
                rweights.resize(nclasses,0);
                ravg_weights.clear();
                ravg_weights.resize(nclasses,0);
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
                        ravg_weights[cls] = ObjCenterAvgConf[i];
                    }
                }
                for( int i = 0; i < nclasses; i++ )
                {
                    ObjCenterWindowsAll[viewId].push_back(rrects[i]);
                    ObjCenterConfAll[viewId].push_back( ravg_weights[i]);
                    ScaleIdCacheAll[viewId].push_back( scaleId);
                    // get median Aspect Ratio
                    Smaller = numeric_limits<float>::max();
                    Bigger = numeric_limits<float>::max();
                    SmallerFlag = true;
                    for (int y = int(rrects[i].y); y< int(rrects[i].y+rrects[i].height); y++)
                        for (int x = int(rrects[i].x); x< int(rrects[i].x+rrects[i].width); x++){
                            int index = y*ImgSize.width + x;
                            float aspect_ = getMedian( vecASMedianCandSmaller[viewId][index],
                                vecASMedianCandBigger[viewId][index],
                                vecASMedianCandSmallerFlag[viewId][index]);
                            if (aspect_ != numeric_limits<float>::max()){
                                CalMedian( aspect_, Smaller, Bigger, SmallerFlag);
                            }
                        }
                    AspectRatioCacheAll[viewId].push_back( getMedian( Smaller, Bigger, SmallerFlag));
//                    cout << "as" << getMedian( Smaller, Bigger, SmallerFlag) << endl;
                }
//                cout << " finish remove redundent rect" << endl;
            }
        }
        window.width = window.width*Scale;
        window.height = window.height*Scale;
//        cout << "change different scale" <<endl;
    }

//    cout << "finish all scale" << endl;
    for ( viewId = 0; viewId < MaxViewId; viewId++){
        // do connected component to prune across scale
        labels.clear();
        //nclasses = partition(ObjCenterWindowsAll[viewId], labels, SimilarRects( RectOverlapRatio));
        nclasses = partition(ObjCenterWindowsAll[viewId], labels, SimilarRects( 0.));
//        cout << "nCand" << ObjCenterWindowsAll[viewId].size() << " CorssCalenCand" << nclasses;
        nlabels = (int)labels.size();
        rrects.clear();
        rrects.resize(nclasses);
        rweights.clear();
        rweights.resize(nclasses,0);
        ScaleIdCache.clear();
        ScaleIdCache.resize(nclasses);
        AspectRatioCache.clear();
        AspectRatioCache.resize(nclasses);
        vector<int> cache(nclasses,0);
        for( int i = 0; i < nlabels; i++ )
        {
            int cls = labels[i];
            if (rweights[cls]< ObjCenterConfAll[viewId][i] ){
                rrects[cls].x = ObjCenterWindowsAll[viewId][i].x;
                rrects[cls].y = ObjCenterWindowsAll[viewId][i].y;
                rrects[cls].width = ObjCenterWindowsAll[viewId][i].width;
                rrects[cls].height = ObjCenterWindowsAll[viewId][i].height;
                rweights[cls] = ObjCenterConfAll[viewId][i];
                ScaleIdCache[cls] = ScaleIdCacheAll[viewId][i];
                AspectRatioCache[cls] = AspectRatioCacheAll[viewId][i];
            }
        }

        // now on change the Object Center Windows to object bounding boxes
        ObjCenterWindowsAll[viewId].clear();
        ObjCenterConfAll[viewId].clear();
        for( int i = 0; i < nclasses; i++ )
        {
//            cout << "AspectRatioCache" << AspectRatioCache[i] << endl;
            Rect Objbbx;
            Point ObjCenter( rrects[i].x+rrects[i].width/2, rrects[i].y+rrects[i].height/2);
            Objbbx.height = rrects[i].height*ObjHeight2MeanShiftwindowRatio;
            Objbbx.width = Objbbx.height*AspectRatioCache[i];
            Objbbx.x = ObjCenter.x-Objbbx.width/2;
            Objbbx.y = ObjCenter.y-Objbbx.height/2;
            ObjCenterWindowsAll[viewId].push_back(Objbbx);
            ObjCenterConfAll[viewId].push_back( rweights[i]);
        }
    }

}

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
    Size ImgSize = cvGetSize(&CvMat(Img));
    int ImgArea = ImgSize.width*ImgSize.height;

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
    unsigned short* uint16ptr;
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
    for (int k=0; k<unique_viewIds.size(); k++){
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
        for (int PatchCount = 1; PatchCount < patchIdptr->size(); PatchCount++)// loop through patches
        {
            patchId = patchIdptr->at(PatchCount);
            //cout << "patchId" << patchId <<endl;
            if (patchId >= ViewIds.size()){ // backgroud patch so skip
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

        for (int PatchCount = 1; PatchCount < patchIdptr->size(); PatchCount++)// loop through patches
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
                        int index = y*ImgSize.width + x;
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

