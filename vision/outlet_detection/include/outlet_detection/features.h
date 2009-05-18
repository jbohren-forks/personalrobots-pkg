/*
 *  features.h
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 4/23/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#ifndef _OUTLET_DETECTION_FEATURES_H
#define _OUTLET_DETECTION_FEATURES_H

#include <vector>
using namespace std;

#include <cv.h>

inline CvPoint operator -(CvPoint p1, CvPoint p2)
{
    return cvPoint(p1.x - p2.x, p1.y - p2.y);
};

inline float length(CvPoint p)
{
    return sqrt(float(p.x*p.x) + p.y*p.y);
};


struct feature_t
{
    CvPoint center;
    float scale;
    int part_id;
    
    ~feature_t() {};
    feature_t(CvPoint _center = cvPoint(-1, -1), float _scale = 1, int _part_id = -1) 
    {
        center = _center; 
        scale = _scale;
        part_id = _part_id;
    };
};

void GetSURFFeatures(IplImage* src, vector<feature_t>& features);
void GetStarFeatures(IplImage* src, vector<feature_t>& features);
void GetHarrisFeatures(IplImage* src, vector<feature_t>& features);
void GetHoleFeatures(IplImage* src, vector<feature_t>& features);

void DrawFeatures(IplImage* img, const vector<feature_t>& features);
void FilterFeatures(vector<feature_t>& features, float min_scale, float max_scale);

void SelectNeighborFeatures(vector<feature_t>& features, const vector<feature_t>& voc);

#endif // _FEATURES_H
