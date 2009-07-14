/*
 *  pca_features.cpp
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 5/15/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include "outlet_detection/pca_features.h"
#include <highgui.h>

void savePCAFeatures(const char* filename, CvMat* avg, CvMat* eigenvectors)
{
    CvMemStorage* storage = cvCreateMemStorage();
    
    CvFileStorage* fs = cvOpenFileStorage(filename, storage, CV_STORAGE_WRITE);
    cvWrite(fs, "avg", avg);
    cvWrite(fs, "eigenvectors", eigenvectors);
    cvReleaseFileStorage(&fs);   
    
    cvReleaseMemStorage(&storage);
}

void calcPCAFeatures(vector<IplImage*>& patches, const char* filename)
{
    int width = patches[0]->width;
    int height = patches[0]->height;
    int length = width*height;
    int patch_count = (int)patches.size();
    
    CvMat* data = cvCreateMat(patch_count, length, CV_32FC1);
    CvMat* avg = cvCreateMat(1, length, CV_32FC1);
    CvMat* eigenvalues = cvCreateMat(length, 1, CV_32FC1);
    CvMat* eigenvectors = cvCreateMat(length, length, CV_32FC1);
    
    for(int i = 0; i < patch_count; i++)
    {
        float sum = cvSum(patches[i]).val[0];
        for(int y = 0; y < height; y++)
        {
            for(int x = 0; x < width; x++)
            {
                *((float*)(data->data.ptr + data->step*i) + y*width + x) = (float)(unsigned char)patches[i]->imageData[y*patches[i]->widthStep + x]/sum;
            }
        }
    }
    
    cvCalcPCA(data, avg, eigenvalues, eigenvectors, CV_PCA_DATA_AS_ROW);
    
    // save pca data
    savePCAFeatures(filename, avg, eigenvectors);
    
    cvReleaseMat(&data);
    cvReleaseMat(&eigenvalues);
    cvReleaseMat(&eigenvectors);
}

void eigenvector2image(CvMat* eigenvector, IplImage* img)
{
    CvRect roi = cvGetImageROI(img);
    if(img->depth == 32)
    {
        for(int y = 0; y < roi.height; y++)
        {
            for(int x = 0; x < roi.width; x++)
            {
                float val = cvmGet(eigenvector, 0, roi.width*y + x);
                *((float*)(img->imageData + (roi.y + y)*img->widthStep) + roi.x + x) = val;
            }
        }
    }
    else
    {
        for(int y = 0; y < roi.height; y++)
        {
            for(int x = 0; x < roi.width; x++)
            {
                float val = cvmGet(eigenvector, 0, roi.width*y + x);
                img->imageData[(roi.y + y)*img->widthStep + roi.x + x] = (unsigned char)val;
            }
        }
    }
}

void loadPCAFeatures(const char* path, vector<IplImage*>& patches)
{
    const int file_count = 20;
    for(int i = 0; i < file_count; i++)
    {
        char buf[1024];
        sprintf(buf, "%s/frame%04d.jpg", path, i);
        IplImage* img = loadImageRed(buf);
        
        vector<feature_t> features;
        GetHoleFeatures(img, features);
        
        for(int j = 0; j < (int)features.size(); j++)
        {
            const int patch_width = 24;
            const int patch_height = 24;
            
            CvPoint center = features[j].center;
            
            CvRect roi = cvRect(center.x - patch_width/2, center.y - patch_height/2, patch_width, patch_height);
            cvSetImageROI(img, roi);
            roi = cvGetImageROI(img);
            if(roi.width != patch_width || roi.height != patch_height)
            {
                continue;
            }
            
            IplImage* patch = cvCreateImage(cvSize(patch_width, patch_height), IPL_DEPTH_8U, 1);
            cvCopy(img, patch);
            patches.push_back(patch);
            cvResetImageROI(img);
            
        }
        
        printf("Completed file %d, extracted %d features\n", i, (int)features.size());
        
        cvReleaseImage(&img);
    }
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

void readPCAFeatures(const char* filename, CvMat** avg, CvMat** eigenvectors)
{
    CvMemStorage* storage = cvCreateMemStorage();
    CvFileStorage* fs = cvOpenFileStorage(filename, storage, CV_STORAGE_READ);
    if(!fs)
    {
        printf("Cannot open file %s! Exiting!", filename);
        cvReleaseMemStorage(&storage);
    }
    
    CvFileNode* node = cvGetFileNodeByName(fs, 0, "avg");
    CvMat* _avg = (CvMat*)cvRead(fs, node);
    node = cvGetFileNodeByName(fs, 0, "eigenvectors");
    CvMat* _eigenvectors = (CvMat*)cvRead(fs, node);
    
    *avg = cvCloneMat(_avg);
    *eigenvectors = cvCloneMat(_eigenvectors);
    
    cvReleaseFileStorage(&fs);
    cvReleaseMemStorage(&storage);
}