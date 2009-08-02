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
using namespace cv;

inline CvPoint operator -(CvPoint p1, CvPoint p2)
{
    return cvPoint(p1.x - p2.x, p1.y - p2.y);
};

inline float length(CvPoint p)
{
    return sqrt(float(p.x*p.x) + p.y*p.y);
};

inline Point2f operator -(Point2f p1, Point2f p2)
{
    return Point2f(p1.x - p2.x, p1.y - p2.y);
};

inline Point2f operator -(Point2f p1, CvPoint p2)
{
    return Point2f(p1.x - p2.x, p1.y - p2.y);
};

inline Point2f operator -(CvPoint p1, Point2f p2)
{
    return Point2f(p1.x - p2.x, p1.y - p2.y);
};

inline float length(Point2f p)
{
    return sqrt(float(p.x*p.x) + p.y*p.y);
};

#if 0
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
#else
class CV_EXPORTS KeyPointEx : public KeyPoint
{
public:
    KeyPointEx(CvPoint _center = cvPoint(-1, -1), float _scale = 1, int _class_id = -1) : 
        KeyPoint(_center.x, _center.y, _scale, 0.0f, 0.0f, 0)
    {
        class_id = _class_id;
    };
    
    ~KeyPointEx() {};
    
    int class_id;
};

typedef KeyPointEx feature_t;

/*void operator =(const Point2f& src, CvPoint& dst)
{
    dst = cvPoint((int)src.x, (int)src.y);
};*/
#endif

void GetSURFFeatures(IplImage* src, vector<feature_t>& features);
void GetStarFeatures(IplImage* src, vector<feature_t>& features);
void GetHarrisFeatures(IplImage* src, vector<feature_t>& features);
void GetHoleFeatures(IplImage* src, vector<feature_t>& features, float hole_contrast = 1.1f);

void DrawFeatures(IplImage* img, const vector<feature_t>& features);
void FilterFeatures(vector<feature_t>& features, float min_scale, float max_scale);

void SelectNeighborFeatures(vector<feature_t>& features, const vector<feature_t>& voc);

#endif // _FEATURES_H
