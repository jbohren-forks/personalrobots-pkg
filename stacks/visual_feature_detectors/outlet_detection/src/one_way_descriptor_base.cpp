/*
 *  one_way_descriptor_base.cpp
 *  outlet_detection
 *
 *  Created by Victor  Eruhimov on 7/9/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include "outlet_detection/features.h"
#include "outlet_detection/pca_features.h"
#include "outlet_detection/one_way_descriptor_base.h"

#include <cv.h>

CvMat* ConvertImageToMatrix(IplImage* patch)
{
    CvRect roi = cvGetImageROI(patch);
    CvMat* mat = cvCreateMat(1, roi.width*roi.height, CV_32FC1);
    
    if(patch->depth == 32)
    {
        for(int y = 0; y < roi.height; y++)
        {
            for(int x = 0; x < roi.width; x++)
            {
                mat->data.fl[y*roi.width + x] = *((float*)(patch->imageData + (y + roi.y)*patch->widthStep) + x + roi.x);
            }
        }
    }
    else if(patch->depth == 8)
    {
        for(int y = 0; y < roi.height; y++)
        {
            for(int x = 0; x < roi.width; x++)
            {
                mat->data.fl[y*roi.width + x] = (float)(unsigned char)patch->imageData[(y + roi.y)*patch->widthStep + x + roi.x];
            }
        }
    }
    else
    {
        printf("Image depth %d is not supported\n", patch->depth);
        return 0;
    }
    
    return mat;
}

CvOneWayDescriptorBase::CvOneWayDescriptorBase(CvSize patch_size, int pose_count, const char* train_path, 
                                               const char* pca_config, const char* pca_hr_config, 
                                               const char* pca_desc_config, int pyr_levels, 
                                               int pca_dim_high, int pca_dim_low) : m_pca_dim_high(pca_dim_high), m_pca_dim_low(pca_dim_low)
{
    m_patch_size = patch_size;
    m_pose_count = pose_count;
    m_pyr_levels = pyr_levels;
    m_poses = 0;
    m_transforms = 0;
    
    char pca_config_filename[1024];
    sprintf(pca_config_filename, "%s/%s", train_path, pca_config);
    readPCAFeatures(pca_config_filename, &m_pca_avg, &m_pca_eigenvectors);
    if(pca_hr_config && strlen(pca_hr_config) > 0)
    {
        char pca_hr_config_filename[1024];
        sprintf(pca_hr_config_filename, "%s/%s", train_path, pca_hr_config);
        readPCAFeatures(pca_hr_config_filename, &m_pca_hr_avg, &m_pca_hr_eigenvectors);
    }
    
    m_pca_descriptors = new CvOneWayDescriptor[m_pca_dim_high + 1];
    if(pca_desc_config && strlen(pca_desc_config) > 0)
//    if(0)
    {
        printf("Loading the descriptors...\n");
        char pca_desc_config_filename[1024];
        sprintf(pca_desc_config_filename, "%s/%s", train_path, pca_desc_config);
        LoadPCADescriptors(pca_desc_config_filename);
    }
    else
    {
        printf("Initializing the descriptors...\n");
        InitializePoseTransforms();
        CreatePCADescriptors();
        SavePCADescriptors("pca_descriptors.yml");
    }
//    SavePCADescriptors("./pca_descriptors.yml");
    
}

CvOneWayDescriptorBase::~CvOneWayDescriptorBase()
{
    cvReleaseMat(&m_pca_avg);
    cvReleaseMat(&m_pca_eigenvectors);
    
    if(m_pca_hr_eigenvectors)
    {
        delete[] m_pca_descriptors;
        cvReleaseMat(&m_pca_hr_avg);
        cvReleaseMat(&m_pca_hr_eigenvectors);
    }
    
    
    delete []m_descriptors;
    
    if(!m_transforms)
    {
        delete []m_poses;
    }
    
    for(int i = 0; i < m_pose_count; i++)
    {
        cvReleaseMat(&m_transforms[i]);
    }
    delete []m_transforms;
}

void CvOneWayDescriptorBase::InitializePoses()
{
    m_poses = new CvAffinePose[m_pose_count];
    for(int i = 0; i < m_pose_count; i++)
    {
        m_poses[i] = GenRandomAffinePose();
    }
}

void CvOneWayDescriptorBase::InitializeTransformsFromPoses()
{
    m_transforms = new CvMat*[m_pose_count];
    for(int i = 0; i < m_pose_count; i++)
    {
        m_transforms[i] = cvCreateMat(2, 3, CV_32FC1);
        GenerateAffineTransformFromPose(cvSize(m_patch_size.width*2, m_patch_size.height*2), m_poses[i], m_transforms[i]);
    }        
}

void CvOneWayDescriptorBase::InitializePoseTransforms()
{
    InitializePoses();
    InitializeTransformsFromPoses();
}

void CvOneWayDescriptorBase::InitializeDescriptor(int desc_idx, IplImage* train_image, const char* feature_label)
{
    m_descriptors[desc_idx].SetPCADimHigh(m_pca_dim_high);
    m_descriptors[desc_idx].SetPCADimLow(m_pca_dim_low);
    
    if(!m_pca_hr_eigenvectors)
    {
        m_descriptors[desc_idx].Initialize(m_pose_count, train_image, feature_label);
    }
    else
    {
        m_descriptors[desc_idx].InitializeFast(m_pose_count, train_image, feature_label, 
                                      m_pca_hr_avg, m_pca_hr_eigenvectors, m_pca_descriptors);
    }
    
    m_descriptors[desc_idx].InitializePCACoeffs(m_pca_avg, m_pca_eigenvectors);    
}


void CvOneWayDescriptorBase::FindDescriptor(IplImage* patch, int& desc_idx, int& pose_idx, float& distance) const
{
#if 1
    ::FindOneWayDescriptor(m_train_feature_count, m_descriptors, patch, desc_idx, pose_idx, distance, m_pca_avg, m_pca_eigenvectors);
#else
    const float scale_min = 0.8f;
    const float scale_max = 1.2f;
    const float scale_step = 1.1f;
    float scale = 1.0f;
    ::FindOneWayDescriptorEx(m_train_feature_count, m_descriptors, patch, 
                             scale_min, scale_max, scale_step, desc_idx, pose_idx, distance, scale, 
                             m_pca_avg, m_pca_eigenvectors);
#endif
}


void CvOneWayDescriptorBase::CreatePCADescriptors()
{
    IplImage* frontal = cvCreateImage(m_patch_size, IPL_DEPTH_32F, 1);
    
    eigenvector2image(m_pca_hr_avg, frontal);
    m_pca_descriptors[0].SetTransforms(m_poses, m_transforms);
    m_pca_descriptors[0].Initialize(m_pose_count, frontal, "", 0);
    
    for(int j = 0; j < m_pca_dim_high; j++)
    {
        CvMat eigenvector;
        cvGetSubRect(m_pca_hr_eigenvectors, &eigenvector, cvRect(0, j, m_pca_hr_eigenvectors->cols, 1));
        eigenvector2image(&eigenvector, frontal);
        
        m_pca_descriptors[j + 1].SetTransforms(m_poses, m_transforms);
        m_pca_descriptors[j + 1].Initialize(m_pose_count, frontal, "", 0);
        
        printf("Created descriptor for PCA component %d\n", j);
    }
    
    cvReleaseImage(&frontal);
}


int CvOneWayDescriptorBase::LoadPCADescriptors(const char* filename)
{
    CvMemStorage* storage = cvCreateMemStorage();
    CvFileStorage* fs = cvOpenFileStorage(filename, storage, CV_STORAGE_READ);
    
    // read affine poses
    CvFileNode* node = cvGetFileNodeByName(fs, 0, "affine poses");
    if(node != 0)
    {
        CvMat* poses = (CvMat*)cvRead(fs, node);
        if(poses->rows != m_pose_count)
        {
            printf("Inconsistency in the number of poses between the class instance and the file! Exiting...\n");
            cvReleaseMat(&poses);
            cvReleaseFileStorage(&fs);
            cvReleaseMemStorage(&storage);
        }
        
        if(m_poses)
        {
            delete m_poses;
        }
        m_poses = new CvAffinePose[m_pose_count];
        for(int i = 0; i < m_pose_count; i++)
        {
            m_poses[i].phi = cvmGet(poses, i, 0);
            m_poses[i].theta = cvmGet(poses, i, 1);
            m_poses[i].lambda1 = cvmGet(poses, i, 2);
            m_poses[i].lambda2 = cvmGet(poses, i, 3);
        }
        cvReleaseMat(&poses);
        
        // now initialize pose transforms
        InitializeTransformsFromPoses();
    }
    
    node = cvGetFileNodeByName(fs, 0, "pca components number");
    if(node != 0)
    {
        
        m_pca_dim_high = cvReadInt(node);
        if(m_pca_descriptors)
        {
            delete []m_pca_descriptors;
        }
        m_pca_descriptors = new CvOneWayDescriptor[m_pca_dim_high + 1];
        for(int i = 0; i < m_pca_dim_high + 1; i++)
        {
            m_pca_descriptors[i].Allocate(m_pose_count, m_patch_size, 1);
            m_pca_descriptors[i].SetTransforms(m_poses, m_transforms);
            char buf[1024];
            sprintf(buf, "descriptor for pca component %d", i);
            m_pca_descriptors[i].ReadByName(fs, 0, buf);
        }
    }
    cvReleaseFileStorage(&fs);
    cvReleaseMemStorage(&storage);
    
    return 1;
}

void CvOneWayDescriptorBase::SavePCADescriptors(const char* filename)
{
    CvMemStorage* storage = cvCreateMemStorage();
    CvFileStorage* fs = cvOpenFileStorage(filename, storage, CV_STORAGE_WRITE);
    
    cvWriteInt(fs, "pca components number", m_pca_dim_high);
    cvWriteComment(fs, "The first component is the average vector, so the total number of components is <pca components number> + 1", 0);
    cvWriteInt(fs, "patch width", m_patch_size.width);
    cvWriteInt(fs, "patch height", m_patch_size.height);
    
    // pack the affine transforms into a single CvMat and write them
    CvMat* poses = cvCreateMat(m_pose_count, 4, CV_32FC1);
    for(int i = 0; i < m_pose_count; i++)
    {
        cvmSet(poses, i, 0, m_poses[i].phi);
        cvmSet(poses, i, 1, m_poses[i].theta);
        cvmSet(poses, i, 2, m_poses[i].lambda1);
        cvmSet(poses, i, 3, m_poses[i].lambda2);
    }
    cvWrite(fs, "affine poses", poses);
    cvReleaseMat(&poses);
    
    for(int i = 0; i < m_pca_dim_high + 1; i++)
    {
        char buf[1024];
        sprintf(buf, "descriptor for pca component %d", i);
        m_pca_descriptors[i].Write(fs, buf);
    }
    
    cvReleaseMemStorage(&storage);
    cvReleaseFileStorage(&fs);
}

void CvOneWayDescriptorBase::CreateDescriptorsFromImage(const char* image_filename)
{
    
    vector<vector<feature_t> > object_features;
    object_features.resize(m_pyr_levels);
    vector<Ptr<IplImage> > images;
    m_train_feature_count = LoadFeatures(image_filename, object_features, images);
        
    m_descriptors = new CvOneWayDescriptor[m_train_feature_count];
    
    int descriptor_count = 0;
    for(int i = 0; i < m_pyr_levels; i++)
    {
        char feature_label[1024];
        sprintf(feature_label, "%s_%d", image_filename, i);
        InitializeDescriptors(images[i], object_features[i], feature_label, descriptor_count);
        descriptor_count += object_features[i].size();
    }
    
}

void CvOneWayDescriptorBase::InitializeDescriptors(IplImage* train_image, const vector<feature_t>& features, 
                                                           const char* feature_label, int desc_start_idx)
{
    for(int i = 0; i < (int)features.size(); i++)
    {
        CvPoint center = features[i].pt;
        
        CvRect roi = cvRect(center.x - m_patch_size.width/2, center.y - m_patch_size.height/2, m_patch_size.width, m_patch_size.height);
        cvSetImageROI(train_image, roi);
        roi = cvGetImageROI(train_image);
        if(roi.width != m_patch_size.width || roi.height != m_patch_size.height)
        {
            continue;
        }
        
        InitializeDescriptor(desc_start_idx + i, train_image, feature_label);
        
    }
    cvResetImageROI(train_image);
}

void CvOneWayDescriptorObject::LoadTrainingFeatures(const char* train_image_filename_object, const char* train_image_filename_background)
{
    
    IplImage* train_image_object = loadImageRed(train_image_filename_object);
    IplImage* train_image_background = loadImageRed(train_image_filename_background);
    
    vector<vector<feature_t> > object_features;
    object_features.resize(m_pyr_levels);
    vector<Ptr<IplImage> > images;
    m_object_feature_count = LoadFeatures(train_image_filename_object, object_features, images);
    
    vector<vector<feature_t> > background_features;
    vector<Ptr<IplImage> > background_images;
    background_features.resize(1);
    int background_feature_count = LoadFeatures(train_image_filename_background, background_features, background_images);
    
    m_train_feature_count = m_object_feature_count + background_feature_count;
    printf("Found %d train points...\n", m_train_feature_count);
        
    m_descriptors = new CvOneWayDescriptor[m_train_feature_count];
    m_part_id = new int[m_object_feature_count];
    
    int descriptor_count = 0;
    for(int i = 0; i < m_pyr_levels; i++)
    {
        char feature_label[1024];
        sprintf(feature_label, "%s_%d", train_image_filename_object, i);
        InitializeObjectDescriptors(images[i], object_features[i], feature_label, descriptor_count, 
                         m_part_id + descriptor_count, 1<<i);
        descriptor_count += object_features[i].size();
    }
    
    InitializeObjectDescriptors(background_images[0], background_features[0], train_image_filename_background, m_object_feature_count);
    
    cvReleaseImage(&train_image_object);
    cvReleaseImage(&train_image_background);
}

void CvOneWayDescriptorObject::InitializeObjectDescriptors(IplImage* train_image, const vector<feature_t>& features, 
                                              const char* feature_label, int desc_start_idx, int* part_id, float scale)
{
    InitializeDescriptors(train_image, features, feature_label, desc_start_idx);
    
    for(int i = 0; i < (int)features.size(); i++)
    {
        CvPoint center = features[i].pt;
                
        if(part_id)
        {
            // remember descriptor part id
            CvPoint center_scaled = cvPoint(round(center.x*scale), round(center.y*scale));
            part_id[i] = MatchPointToPart(center_scaled);
        }
    }
    cvResetImageROI(train_image);
}

int CvOneWayDescriptorObject::IsDescriptorObject(int desc_idx) const
{
    return desc_idx < m_object_feature_count ? 1 : 0;
}

int CvOneWayDescriptorObject::MatchPointToPart(CvPoint pt) const
{
    int idx = -1;
    const int max_dist = 10;
    for(int i = 0; i < (int)m_train_features.size(); i++)
    {
        if(length(pt - m_train_features[i].pt) < max_dist)
        {
            idx = i;
            break;
        }
    }
    
    return idx;
}

int CvOneWayDescriptorObject::GetDescriptorPart(int desc_idx) const
{
    //    return MatchPointToPart(GetDescriptor(desc_idx)->GetCenter());
    return m_part_id[desc_idx];
}

CvOneWayDescriptorObject::CvOneWayDescriptorObject(CvSize patch_size, int pose_count, const char* train_path, 
                                               const char* pca_config, const char* pca_hr_config, const char* pca_desc_config, int pyr_levels) :
        CvOneWayDescriptorBase(patch_size, pose_count, train_path, pca_config, pca_hr_config, pca_desc_config, pyr_levels)
{
    m_part_id = 0;
}

CvOneWayDescriptorObject::~CvOneWayDescriptorObject()
{
    delete m_part_id;
}