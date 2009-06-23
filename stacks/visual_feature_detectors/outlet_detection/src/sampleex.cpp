/*
 *  main.cpp
 *  outlet_model
 *
 *  Created by Victor  Eruhimov on 1/16/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

//*****************************************************************************************
// Warning: this is research code with poor architecture, performance and no documentation!
//*****************************************************************************************

#include <stdio.h>
#include <sys/stat.h>

#include <vector>
using namespace std;

#include <cv.h>
#include <highgui.h>
#include <ml.h>

#include "outlet_detection/learning.h"
#include "star_detector/detector.h"
#include "outlet_detection/outlet_tuple.h"
#include "outlet_detection/outlet_model.h"

#include "outlet_detection/chamfer_match.h"

void ExtractEdges(IplImage* src, IplImage* edges);
void FilterLongEdges(IplImage* edges);
int LoadApplicationParams(char* filename);
CvMat* getHomographyMatrix(IplImage* src, IplImage* dst=NULL);

int LoadCameraParams(char* filename, CvMat** intrinsic_matrix, CvMat** distortion_coeffs);
/*{
	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
	if (fs==NULL) return 0;
	
    *intrinsic_matrix = (CvMat*)cvReadByName( fs,0,"camera_matrix");
	*distortion_coeffs = (CvMat*)cvReadByName( fs,0,"distortion_coefficients");
	
	return 1;
}*/

int main(int argc,char** argv)
{
    if(argc < 2)
    {
        printf("Usage: outlet_sampleex mode <mode specific params>\n");
        return(0);
    }
    
    int mode = atoi(argv[1]);
    if(mode == 0)
    {
        char path[1024], config_filename[1024], camera_filename[1024], output_path[1024];
        if(argc < 6)
        {
            printf("Usage: outlet_sampleex 0 <path_to_images> <config_filename> <camera_config> <output_path>\n");
            return(0);
        }
        
        strcpy(path, argv[2]);
        strcpy(config_filename, argv[3]);
        strcpy(camera_filename, argv[4]);
        strcpy(output_path, argv[5]);
        
        char cmd_buf[1024];
        sprintf(cmd_buf, "mkdir %s", output_path);
        system(cmd_buf);
        
        printf("Reading config file...\n");
        
        FILE* fp = fopen(config_filename, "rt");
        if(fp == 0)
        {
            printf("Config file not found! Exiting...\n");
            exit(1);
        }
        
        char buf[1024];
        int ret = 0;
        
        // reading camera params
        CvMat* intrinsic_matrix = 0;
        CvMat* distortion_params = 0; 
        LoadCameraParams(camera_filename, &intrinsic_matrix, &distortion_params);
        
        CvMemStorage* storage = cvCreateMemStorage();
        
        while((ret=fscanf(fp, "%s\n", buf)) > 0)
        {
            char filename[1024];
            sprintf(filename, "%s/%s", path, buf);
            
            IplImage* src = cvLoadImage(filename);
            if(src == 0)
            {
                printf("File %s not found, skipping...\n", filename);
                continue;
            }
            
            printf("\nImage %s:", buf);
            
            vector<outlet_t> outlets;
            outlet_tuple_t outlet_tuple;
            outlet_tuple.tuple_mask = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
            int ret = find_outlet_centroids(src, outlet_tuple, 0, 0);
            if(!ret)
            {
                // tuple not found...
                printf("Tuple not found...\n");
                cvReleaseImage(&src);
                continue;
            }
            
            // select each outlet mask
            for(int i = 0; i < 4; i++)
            {
                IplImage* outlet_mask = cvCloneImage(outlet_tuple.tuple_mask);
                cvCmpS(outlet_tuple.tuple_mask, i + 1, outlet_mask, CV_CMP_EQ);
                CvRect roi = cvBoundingRect(outlet_mask);
                if(roi.width > 0 && roi.height > 0)
                {
                    cvSetImageROI(outlet_mask, roi);
                    
                    char filename[1024];
                    sprintf(filename, "%s/%s_%d.png", output_path, 
                            string(buf).substr(0, strlen(buf) - 4).c_str(), i);
                    
                    IplImage* outlet_mask_small = cvCreateImage(cvSize(roi.width/4, 
                                roi.height/4), IPL_DEPTH_8U, 1);
                    cvResize(outlet_mask, outlet_mask_small);
                    
                    // extract the boundary
                    CvSeq* first = 0;
                    cvFindContours(outlet_mask_small, storage, &first, sizeof(CvContour), CV_RETR_LIST);
                    cvSetZero(outlet_mask_small);
                    cvDrawContours(outlet_mask_small, first, cvScalar(255), cvScalar(255), 10, 1);
                    
                    cvSaveImage(filename, outlet_mask_small);
                    cvReleaseImage(&outlet_mask_small);
                }
                cvReleaseImage(&outlet_mask);
            }           
            
            cvReleaseImage(&src);
        }
        
        cvReleaseMemStorage(&storage);
    }
    else if(mode == 1)
    {
        if(argc < 8)
        {
            printf("Usage: sampleex <template_path> <template_config> <images_path> <images_config> <calib_filename> <output_path>\n");
            return(0);
        }
        
        char template_path[1024], template_config[1024], images_path[1024], config_filename[1024], 
            calib_filename[1024], output_path[1024];
        
        strcpy(template_path, argv[2]);
        strcpy(template_config, argv[3]);
        strcpy(images_path, argv[4]);
        strcpy(config_filename, argv[5]);
        strcpy(calib_filename, argv[6]);
        strcpy(output_path, argv[7]);
        
        char sys_buf[1024];
        sprintf(sys_buf, "mkdir %s", output_path);
        system(sys_buf);
        
//        ChamferMatch match;
        ChamferMatchEx match;
        match.LoadTrainingBase(template_path, template_config);
        
        CvMat* intrinsic_matrix = 0;
        CvMat* distortion_coeffs = 0;
        LoadCameraParams(calib_filename, &intrinsic_matrix, &distortion_coeffs);
        
        FILE* fp = fopen(config_filename, "rt");
        if(!fp)
        {
            printf("Cannot load file %s\n", config_filename);
            return(0);
        }
        
        char filename[1024];
        while(fscanf(fp, "%s\n", filename) > 0)
        {
            char buf[1024];
            sprintf(buf, "%s/%s", images_path, filename);
            IplImage* src = cvLoadImage(buf, CV_LOAD_IMAGE_GRAYSCALE);
            
            IplImage* srcu = cvCloneImage(src);
            cvUndistort2(srcu, src, intrinsic_matrix, distortion_coeffs);
            cvReleaseImage(&srcu);
            
            IplImage* srcroi = cvCreateImage(cvSize(1740, src->height), IPL_DEPTH_8U, 1);
            cvSetImageROI(src, cvRect(0, 0, 1740, src->height));
            cvCopy(src, srcroi);
            IplImage* srcold = src;
            src = srcroi;
            cvReleaseImage(&srcold);
            
            IplImage* edges = cvCreateImage(cvSize(src->width/4, src->height/4), IPL_DEPTH_8U, 1);
            ExtractEdges(src, edges);

            float dist_min;
            int idx_min;
            CvPoint offset_min;
            float scale_min;
            IplImage* match_map = cvCloneImage(edges);
            match.Match(edges, &dist_min, &idx_min, &offset_min, &scale_min, match_map, 1.5);
            
            IplImage* templ_img = match.GetTemplate(idx_min);

            
            IplImage* templ_img_8u = cvCreateImage(cvSize(templ_img->width, templ_img->height), IPL_DEPTH_8U, 1);
            cvCmpS(templ_img, 0, templ_img_8u, CV_CMP_LT);
            
            IplImage* templ_img_8ur = cvCreateImage(cvSize(int(templ_img->width*scale_min), int(templ_img->height*scale_min)), 
                                                    IPL_DEPTH_8U, 1);
            cvResize(templ_img_8u, templ_img_8ur);
            cvReleaseImage(&templ_img_8u);
            
            double minval, maxval;
            cvMinMaxLoc(templ_img_8ur, &minval, &maxval);

            cvConvertScale(edges, edges, 0.2, 1);
            CvRect roi = cvRect(offset_min.x, offset_min.y, templ_img_8ur->width, templ_img_8ur->height);
            cvSetImageROI(edges, roi);
            cvOr(edges, templ_img_8ur, edges);
            cvResetImageROI(edges);

            sprintf(buf, "%s/%s", output_path, filename);
            cvSaveImage(buf, edges);

            printf("Minimum distance: %f, index: %d\n", dist_min, idx_min);
            
#if 1
            cvNamedWindow("edges", 1);
            cvShowImage("edges", edges);
            
            cvNamedWindow("template", 1);
            cvShowImage("template", templ_img_8ur);
            
            cvNamedWindow("match map", 1);
            cvShowImage("match map", match_map);
            
            cvWaitKey(0);
#endif
            
            cvReleaseImage(&templ_img_8ur);
            cvReleaseImage(&src);
            
            printf("File %s: done\n", filename);
        }
        fclose(fp);
    }
    else
    {
        printf("Unknown mode\n");
    }
		
    return 0;
}


void ExtractEdges(IplImage* src, IplImage* edges)
{
    IplImage* src_small = cvCloneImage(edges);
    cvResize(src, src_small);
    cvCanny(src_small, edges, 70, 50);

#if 0
    IplImage* laplace = cvCreateImage(cvSize(edges->width, edges->height), IPL_DEPTH_32F, 1);
    cvLaplace(src_small, laplace);
    double valmax;
    cvMinMaxLoc(laplace, 0, &valmax);
    cvConvertScale(laplace, edges, 255.0f/valmax);
    cvThreshold(edges, edges, 16, 255, CV_THRESH_BINARY);
#endif
    
    FilterLongEdges(edges);

#if 1
    cvNamedWindow("edges", 1);
    cvShowImage("edges", edges);
    cvWaitKey(0);
#endif
}

void FilterLongEdges(IplImage* edges)
{
    const int max_size = 50;
    const int min_size = 5;
    
    CvMemStorage* storage = cvCreateMemStorage();
    CvSeq* first = 0;
    cvFindContours(edges, storage, &first, sizeof(CvContour), CV_RETR_LIST);
    
    cvSetZero(edges);
    for(CvSeq* seq = first; seq != 0; seq = seq->h_next)
    {
        CvRect bbox = cvBoundingRect(seq);
        if(bbox.width > max_size || bbox.height > max_size)
        {
            continue;
        }
        
        if(bbox.width < min_size && bbox.height < min_size)
        {
            continue;
        }
        
        cvDrawContours(edges, seq, cvScalar(255), cvScalar(255), 0, 1);
    }
    
    cvReleaseMemStorage(&storage);
}