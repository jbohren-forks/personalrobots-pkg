/*
 *  chamfer_match.h
 *  outlet_detection
 *
 *  Created by Victor  Eruhimov on 3/29/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_CHAMFER_MATCH_H)
#define _CHAMFER_MATCH_H

#include <vector>
using namespace std;

#include <cxcore.h>
#include <cv.h>
#include <highgui.h>

class ChamferMatch
{
public:
	ChamferMatch();
	~ChamferMatch();
	
	void AddTemplate(CvSeq* templ_seq);
	void AddTemplate(IplImage* dist);
    void LoadTrainingBase(const char* path, const char* config);
    void Match(CvSeq* seq, float* dist_min, int* idx_min);
    IplImage* GetTemplateDistMap(int idx);

protected:
	
	vector<IplImage*> distances;
};

void ChamferDistance(CvSeq* seq, IplImage* dist_img, float* dist_min, 
                     CvPoint* offset_min, float out_of_bound_penalty);
float CalcContourDistTemplate(CvSeq* seq, IplImage* dist_img, int x, int y, 
                              float out_of_bound_penalty);

class ChamferMatchEx
{
public:
    ChamferMatchEx();
    ~ChamferMatchEx();
    
    void AddTemplate(IplImage* edges);
    
    // Loads image templates. Each template should be a binary 0/255 image.
    // Input parameters: 
    //  path: path to images
    //  config: path to a file with image names
    void LoadTrainingBase(const char* path, const char* config);
    
    // match templates to a binary image using chamfer matching
    // Input parameters:
    //  edges: input binary image
    //  match_map: a greyscale image of the same size as edges, filled by ChamferMatchEx::Match. 
    //      A pixel in match_map is set to 255 if at least one template had distance less than dist_thresh
    //      Otherwise a pixel is set to 0. match_map is ignored if zero.
    //  dist_thresh: threshold for distances of interest (see match_map)
    // Output parameters:
    //  dist_min: the distance for the closest match
    //  idx_min: the index of the closest template
    //  offset_min: offset of the closest match in edges image
    //  scale_min: scale of the closest match
    //  match_map: a binary image of the 
    void Match(IplImage* edges, float* dist_min, int* idx_min, CvPoint* offset_min, 
               float* scale_min, IplImage* match_map = 0, float dist_thresh = 0.0);
    
    // return a template image
    IplImage* GetTemplate(int idx);
    
    // set scale limits for templates
    void SetScaleLimits(float _min_scale, float _max_scale);
    
protected:
    
    vector<IplImage*> template_images;
    float min_scale;
    float max_scale;
};

void ChamferDistance(IplImage* templ_img, IplImage* dist_img, 
                     float* dist_min, CvPoint* offset_min = 0, IplImage* match_map = 0, double dist_thresh = 0.0);
void ChamferDistanceScale(IplImage* templ_img, IplImage* dist_img, 
                          float* dist_min, CvPoint* offset_min = 0, float* scale_min = 0, 
                          float min_scale = 0.5, float max_scale = 1.1, 
                          int count_scale = 10, IplImage* match_map = 0, float dist_thresh = 0.0);


#endif //_CHAMFER_MATCH_H