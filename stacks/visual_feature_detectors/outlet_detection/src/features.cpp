/*
 *  features.cpp
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 4/23/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include "outlet_detection/features.h"
#include "outlet_detection/outlet_model.h"

void GetSURFFeatures(IplImage* src, vector<feature_t>& features)
{
    CvMemStorage* storage = cvCreateMemStorage();
    CvSeq* surf_points = 0;
    cvExtractSURF(src, 0, &surf_points, 0, storage, cvSURFParams(512));
    
    features.clear();
    for(int i = 0; i < surf_points->total; i++)
    {
        CvSURFPoint* point = (CvSURFPoint*)cvGetSeqElem(surf_points, i);
        CvPoint center = cvPoint(point->pt.x, point->pt.y);
        features.push_back(feature_t(center, (float)point->size));
    }
    
    cvReleaseMemStorage(&storage);
}

void GetStarFeatures(IplImage* src, vector<feature_t>& features)
{
    CvMemStorage* storage = cvCreateMemStorage();
    CvSeq* star_points = cvGetStarKeypoints(src, storage, cvStarDetectorParams(45));
    
    features.clear();
    for(int i = 0; i < star_points->total; i++)
    {
        CvStarKeypoint* keypoint = (CvStarKeypoint*)cvGetSeqElem(star_points, i);
        features.push_back(feature_t(keypoint->pt, (float)keypoint->size));
    }
    
    cvReleaseMemStorage(&storage);
}

void GetHarrisFeatures(IplImage* src, vector<feature_t>& features)
{
    IplImage* grey = src;
    if(src->nChannels > 1)
    {
        grey = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
        cvCvtColor(src, grey, CV_RGB2GRAY);
    }
    
    
    IplImage* eig_img = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_32F, 1);
    IplImage* temp_img = cvCloneImage(eig_img);
    
    int corner_count = 1024;
    CvPoint2D32f* corners = new CvPoint2D32f[corner_count];
    cvGoodFeaturesToTrack(grey, eig_img, temp_img, corners, &corner_count, .5, 0);//, 0, 3, 1);
    
    for(int i = 0; i < corner_count; i++)
    {
        features.push_back(feature_t(cvPoint(corners[i].x, corners[i].y), 1.0f));
    }
    
    if(src->nChannels > 1)
    {
        cvReleaseImage(&grey);
    }
    cvReleaseImage(&eig_img);
    cvReleaseImage(&temp_img);
}

void GetHoleFeatures(IplImage* src, vector<feature_t>& features, float hole_contrast)
{
    vector<outlet_feature_t> outlet_features;
    find_outlet_features_fast(src, outlet_features, hole_contrast, 0, 0);
    for(int i = 0; i < outlet_features.size(); i++)
    {
        features.push_back(feature_t(feature_center(outlet_features[i]), outlet_features[i].bbox.width));
    }
}

void DrawFeatures(IplImage* img, const vector<feature_t>& features)
{
    for(int i = 0; i < features.size(); i++)
    {
        cvCircle(img, features[i].center, features[i].scale, CV_RGB(255, 0, 0), 2);
    }
}

void FilterFeatures(vector<feature_t>& features, float min_scale, float max_scale)
{
    vector<feature_t> selected;
    for(int i = 0; i < features.size(); i++)
    {
        if(features[i].scale >= min_scale && features[i].scale <= max_scale)
        {
            selected.push_back(features[i]);
        }
    }
    
    features = selected;
}

void SelectNeighborFeatures(vector<feature_t>& features, const vector<feature_t>& voc)
{
    const int max_dist = 10;
    vector<feature_t> filtered;
    for(int i = 0; i < (int)features.size(); i++)
    {
        for(int j = 0; j < (int)voc.size(); j++)
        {
            if(length(features[i].center - voc[j].center) < max_dist)
            {
                filtered.push_back(features[i]);
            }
        }
    }
    
    features = filtered;
}
