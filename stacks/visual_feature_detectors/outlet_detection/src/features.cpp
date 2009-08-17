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
#include <highgui.h>

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
    for(size_t i = 0; i < outlet_features.size(); i++)
    {
        features.push_back(feature_t(feature_center(outlet_features[i]), outlet_features[i].bbox.width));
    }
}

void GetHoleFeatures(IplImage* src, Vector<feature_t>& features, float hole_contrast)
{
    vector<outlet_feature_t> outlet_features;
    find_outlet_features_fast(src, outlet_features, hole_contrast, 0, 0);
    for(size_t i = 0; i < outlet_features.size(); i++)
    {
        features.push_back(feature_t(feature_center(outlet_features[i]), outlet_features[i].bbox.width));
    }
}

void DrawFeatures(IplImage* img, const vector<feature_t>& features)
{
    for(size_t i = 0; i < features.size(); i++)
    {
        cvCircle(img, features[i].pt, features[i].size, CV_RGB(255, 0, 0), 2);
    }
}

void FilterFeatures(vector<feature_t>& features, float min_scale, float max_scale)
{
    vector<feature_t> selected;
    for(size_t i = 0; i < features.size(); i++)
    {
        if(features[i].size >= min_scale && features[i].size <= max_scale)
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
            if(length(features[i].pt - voc[j].pt) < max_dist)
            {
                filtered.push_back(features[i]);
            }
        }
    }
    
    features = filtered;
}

int CalcFeatures(IplImage* image, Vector<Vector<feature_t> >& features, Vector<IplImage*>& images)
{    
    size_t pyr_levels = features.size();
    images.resize(pyr_levels);
    IplImage* image_features = cvCloneImage(image);
    for(size_t i = 0; i < features.size(); i++)
    {
        images[i] = image_features;
        GetHoleFeatures(image_features, features[i]);
        IplImage* temp_image = cvCreateImage(cvSize(image_features->width/2, image_features->height/2), IPL_DEPTH_8U, 1);
        cvPyrDown(image_features, temp_image);
        image_features = temp_image;
    }
    cvReleaseImage(&image_features);
    
    int feature_count = 0;
    
    for(size_t i = 0; i < pyr_levels; i++)
    {
        feature_count += features[i].size();
    }
    
    cvReleaseImage(&image);
    
    return feature_count;
}

int LoadFeatures(const char* filename, Vector<Vector<feature_t> >& features, Vector<IplImage*>& images)
{
    IplImage* image = loadImageRed(filename);

    int feature_count = CalcFeatures(image, features, images);
    
    return feature_count;
}

IplImage* loadImageRed(const char* filename)
{
    IplImage* temp = cvLoadImage(filename);
    IplImage* red = cvCreateImage(cvSize(temp->width, temp->height), IPL_DEPTH_8U, 1);
    cvSetImageCOI(temp, 3);
    cvCopy(temp, red);
    cvReleaseImage(&temp);
    
#if defined(_SCALE_IMAGE_2)
    IplImage* red2 = cvCreateImage(cvSize(red->width/2, red->height/2), IPL_DEPTH_8U, 1);
    cvResize(red, red2);
    cvReleaseImage(&red);
    red = red2;
#endif //_SCALE_IMAGE_2
    
    return red;
}

void ReleaseImageVector(Vector<IplImage*>& images)
{
    for(size_t i = 0; i < images.size(); i++)
    {
        cvReleaseImage(&images[i]);
    }
}

void LoadTrainingFeatures(CvOneWayDescriptorObject& descriptors, const char* train_image_filename_object, const char* train_image_filename_background)
{
    
    IplImage* train_image_object = loadImageRed(train_image_filename_object);
    IplImage* train_image_background = loadImageRed(train_image_filename_background);
    
    Vector<Vector<feature_t> > object_features;
    object_features.resize(descriptors.GetPyrLevels());
    Vector<IplImage*> images;
    int object_feature_count = LoadFeatures(train_image_filename_object, object_features, images);
    
    Vector<Vector<feature_t> > background_features;
    Vector<IplImage*> background_images;
    background_features.resize(1);
    int background_feature_count = LoadFeatures(train_image_filename_background, background_features, background_images);
    
    int train_feature_count = object_feature_count + background_feature_count;
    printf("Found %d train points...\n", train_feature_count);
    
    descriptors.Allocate(train_feature_count, object_feature_count);
    
    int descriptor_count = 0;
    for(int i = 0; i < descriptors.GetPyrLevels(); i++)
    {
        char feature_label[1024];
        sprintf(feature_label, "%s_%d", train_image_filename_object, i);
        IplImage* img = images[i];
        descriptors.InitializeObjectDescriptors(img, object_features[i], feature_label, descriptor_count, (float)(1<<i));
        descriptor_count += object_features[i].size();
    }
    
    descriptors.InitializeObjectDescriptors(background_images[0], background_features[0], train_image_filename_background, object_feature_count);
    
    cvReleaseImage(&train_image_object);
    cvReleaseImage(&train_image_background);
    
    ReleaseImageVector(images);
    ReleaseImageVector(background_images);
}

void ScaleFeatures(const vector<feature_t>& src, vector<feature_t>& dst, float scale)
{
    dst.resize(src.size());
    for(size_t i = 0; i < src.size(); i++)
    {
        dst[i] = feature_t(cvPoint(src[i].pt.x*scale, src[i].pt.y*scale), src[i].size, src[i].class_id);
    }
}
