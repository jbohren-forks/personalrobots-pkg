/*
 *  chamfer_match.cpp
 *  outlet_detection
 *
 *  Created by Victor  Eruhimov on 3/29/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include "chamfer_match.h"

ChamferMatch::ChamferMatch()
{
}

ChamferMatch::~ChamferMatch()
{
    for(int i = 0; i < distances.size(); i++)
    {
        cvReleaseImage(&distances[i]);
    }
}

void ChamferMatch::AddTemplate(CvSeq* templ_seq)
{
    CvRect rect = cvBoundingRect(templ_seq);
    IplImage* grey = cvCreateImage(cvSize(2*rect.width, 2*rect.height), IPL_DEPTH_8U, 1);
    IplImage* dist = cvCreateImage(cvSize(2*rect.width, 2*rect.height), IPL_DEPTH_32F, 1);

    cvSetZero(grey);
    cvDrawContours(grey, templ_seq, cvScalar(255), cvScalar(255), 0, 1, 8, 
                   cvPoint(rect.width/4 - rect.x, rect.height/4 - rect.y));
    cvNot(grey, grey);
    cvDistTransform(grey, dist);
    
    AddTemplate(dist);
    
    cvReleaseImage(&grey);
}

void ChamferMatch::AddTemplate(IplImage* dist)
{
    distances.push_back(dist);
}

void ChamferMatch::LoadTrainingBase(const char* path, const char* config)
{
    FILE* fp = fopen(config, "rt");
    char filename[1024];
    
    CvMemStorage* storage = cvCreateMemStorage();
    
    while(fscanf(fp, "%s\n", filename) > 0)
    {
        char buf[1024];
        sprintf(buf, "%s/%s", path, filename);
        IplImage* src = cvLoadImage(buf, CV_LOAD_IMAGE_GRAYSCALE);
    
        CvSeq* first = 0;
        cvFindContours(src, storage, &first, sizeof(CvContour), CV_RETR_LIST);
        AddTemplate(first);
        
        cvReleaseImage(&src);
    }

    cvReleaseMemStorage(&storage);
}

void ChamferMatch::Match(CvSeq* seq, float* dist_min, int* idx_min)
{
    float _dist_min = 1e10;
    int _idx_min = -1;
    CvPoint offset_min;
    for(int i = 0; i < distances.size(); i++)
    {
        float dist;
        const float out_of_bound_penalty = 1e10;
        CvPoint offset;
        ChamferDistance(seq, distances[i], &dist, &offset, out_of_bound_penalty);
        
        if(dist < _dist_min)
        {
            _dist_min = dist;
            _idx_min = i;
            offset_min = offset;
        }
    }
    
    if(dist_min) *dist_min = _dist_min;
    if(idx_min) *idx_min = _idx_min;
    
    
#if 0
    if(_dist_min > 3.0f)
    {
        return;
    }

    IplImage* grey = cvCreateImage(cvSize(distances[_idx_min]->width, distances[_idx_min]->height), 
                                   IPL_DEPTH_8U, 1);
    double maxval;
    cvMinMaxLoc(distances[_idx_min], 0, &maxval);
    cvConvertScale(distances[_idx_min], grey, 255/maxval);
    
    CvRect bbox = cvBoundingRect(seq);
    cvDrawContours(grey, seq, cvScalar(255), cvScalar(255), 0, 2, 8, 
                   cvPoint(offset_min.x-bbox.x, offset_min.y-bbox.y));
    printf("Distance = %f\n", _dist_min);

    cvNamedWindow("1", 1);
    cvShowImage("1", grey);
    cvWaitKey(0);
#endif
}

IplImage* ChamferMatch::GetTemplateDistMap(int idx)
{
    return distances[idx];
}

void ChamferDistance(CvSeq* seq, IplImage* dist_img, float* dist_min, CvPoint* offset_min, float out_of_bound_penalty)
{
    *dist_min = 1e10;
    CvRect bbox = cvBoundingRect(seq);
    for(int y = 0; y < dist_img->height; y += 2)
    {
        for(int x = 0; x < dist_img->width; x += 2)
        {
            float dist = CalcContourDistTemplate(seq, dist_img, x - bbox.x, y - bbox.y, 
                                                 out_of_bound_penalty);

            if(dist < *dist_min)
            {
                *dist_min = dist;
                if(offset_min)
                {
                    offset_min->x = x;
                    offset_min->y = y;
                }
            }
        }
    }
}

void ChamferDistanceScale(IplImage* templ_img, IplImage* dist_img, 
                    float* dist_min, CvPoint* offset_min, float* scale_min, float min_scale, float max_scale, int count_scale, 
                    IplImage* match_map, float dist_thresh)
{
    *dist_min = 1e10;
    for(int i = 0; i < count_scale; i++)
    {
        float dist;
        CvPoint offset;
        
        float scale = min_scale + (max_scale - min_scale)*i/count_scale;
        int width = int(templ_img->width*scale);
        int height = int(templ_img->height*scale);
        
        IplImage* templ_scale = cvCreateImage(cvSize(width, height), IPL_DEPTH_32S, 1);
        cvResize(templ_img, templ_scale, CV_INTER_NN);
        
        ChamferDistance(templ_scale, dist_img, &dist, &offset, match_map, dist_thresh);
        if(dist < *dist_min)
        {
            *dist_min = dist;
            *offset_min = offset;
            *scale_min = scale;
        }
        
        cvReleaseImage(&templ_scale);
    }
}

// here templ_img should contain 0xffffffff in all edge pixels and 0 in all non-edge pixels 
void ChamferDistance(IplImage* templ_img, IplImage* dist_img, 
                    float* dist_min, CvPoint* offset_min, IplImage* match_map, double dist_thresh)
{
    *dist_min = 1e10;
    
    CvRect dist_bbox = cvGetImageROI(dist_img);
    CvRect templ_bbox = cvGetImageROI(templ_img);
    
    IplImage* mask_img = cvCreateImage(cvSize(templ_bbox.width, templ_bbox.height), IPL_DEPTH_32F, 1);
    
    int count = cvCountNonZero(templ_img);
    
    for(int y = 0; y < dist_bbox.height - templ_bbox.height; y += 2)
    {
        for(int x = 0; x < dist_bbox.width - templ_bbox.width; x += 2)
        {
            CvRect roi = cvRect(x, y, templ_bbox.width, templ_bbox.height);
            cvSetImageROI(dist_img, roi);
            
            templ_img->depth = IPL_DEPTH_32F;
            cvAnd(templ_img, dist_img, mask_img);
            templ_img->depth = IPL_DEPTH_32S;
            
            float dist = cvSum(mask_img).val[0]/count;
            
            if(dist < *dist_min)
            {
                *dist_min = dist;
                if(offset_min)
                {
                    offset_min->x = x;
                    offset_min->y = y;
                }
            }
            
            if(match_map && dist < dist_thresh)
            {
                match_map->imageData[y*match_map->widthStep + x] = 255;
            }
        }
    }
    
    cvResetImageROI(dist_img);
    
    cvReleaseImage(&mask_img);
}

float CalcContourDistTemplate(CvSeq* seq, IplImage* dist_img, int x, int y, float out_of_bound_penalty)
{
    float sum = 0;
    for(int i = 0; i < seq->total; i++)
    {
        CvPoint* p = (CvPoint*)cvGetSeqElem(seq, i);
        CvPoint poff = cvPoint(p->x + x, p->y + y);
        if(poff.x < 0 || poff.y < 0 || poff.x >= dist_img->width || poff.y >= dist_img->height)
        {
            sum += out_of_bound_penalty;
            continue;
        }
        
        float* val = (float*)(dist_img->imageData + poff.y*dist_img->widthStep) + poff.x;
        //sum += *val*(*val);
        sum = MAX(sum, *val);
    }
    
//    sum = sqrt(sum/seq->total);
    return(sum);
}

ChamferMatchEx::ChamferMatchEx()
{
    SetScaleLimits(0.7f, 1.2f);
}

ChamferMatchEx::~ChamferMatchEx()
{
    for(int i = 0; i < (int)template_images.size(); i++)
    {
        cvReleaseImage(&template_images[i]);
    }
}

void ChamferMatchEx::AddTemplate(IplImage* edges)
{
    IplImage* mask_img = cvCreateImage(cvSize(edges->width, edges->height), IPL_DEPTH_32S, 1);

    const unsigned int val = 0xffffffff;
    for(int y = 0; y < edges->height; y++)
    {
        for(int x = 0; x < edges->width; x++)
        {
            float* pdata = (float*)(mask_img->imageData + y*mask_img->widthStep) + x;
            if(edges->imageData[y*edges->widthStep + x])
            {
                *(int*)pdata = val;
            }
            else
            {
                *(int*)pdata = 0;
            }
        }
    }
    
    template_images.push_back(mask_img);
}

void ChamferMatchEx::Match(IplImage* edges, float* dist_min, int* idx_min, CvPoint* offset_min, 
                           float* scale_min, IplImage* match_map, float dist_thresh)
{
    float _dist_min = 1e10;
    int _idx_min = -1;
    CvPoint _offset_min;
    double _scale_min;
    
    if(match_map)
    {
        cvSetZero(match_map);
    }
    
    CvRect edges_bbox = cvGetImageROI(edges);
    IplImage* dist_img = cvCreateImage(cvSize(edges_bbox.width, edges_bbox.height), IPL_DEPTH_32F, 1);
    cvNot(edges, edges);
    cvDistTransform(edges, dist_img, CV_DIST_L2);
    cvNot(edges, edges);
    
    for(int i = 0; i < (int)template_images.size(); i++)
    {
        float dist;
        CvPoint offset;
        float scale;
        ChamferDistanceScale(template_images[i], dist_img, &dist, &offset, &scale, 
                             min_scale, max_scale, 1, match_map, dist_thresh);
        
        if(dist < _dist_min)
        {
            _dist_min = dist;
            _idx_min = i;
            _offset_min = offset;
            _scale_min = scale;
        }
        
        printf(".");
    }
    
    if(dist_min) *dist_min = _dist_min;
    if(idx_min) *idx_min = _idx_min;
    if(offset_min) *offset_min = _offset_min;
    if(scale_min) *scale_min = _scale_min;
    
    cvReleaseImage(&dist_img);
}

void ChamferMatchEx::LoadTrainingBase(const char* path, const char* config)
{
    FILE* fp = fopen(config, "rt");
    char filename[1024];
    
    CvMemStorage* storage = cvCreateMemStorage();
    
    while(fscanf(fp, "%s\n", filename) > 0)
    {
        char buf[1024];
        sprintf(buf, "%s/%s", path, filename);
        IplImage* src = cvLoadImage(buf, CV_LOAD_IMAGE_GRAYSCALE);
        if(!src)
        {
            continue;
        }
        
        
#if 0
        cvNamedWindow("edges", 1);
        cvShowImage("edges", src);
        cvWaitKey(0);
#endif   
        AddTemplate(src);
        
        cvReleaseImage(&src);
    }
    
    cvReleaseMemStorage(&storage);
}

IplImage* ChamferMatchEx::GetTemplate(int idx)
{
    if(idx < 0 || idx >= (int)template_images.size())
    {
        return 0;
    }
    else
    {
        return template_images[idx];
    }
}

void ChamferMatchEx::SetScaleLimits(float _min_scale, float _max_scale)
{
    min_scale = _min_scale;
    max_scale = _max_scale;
}