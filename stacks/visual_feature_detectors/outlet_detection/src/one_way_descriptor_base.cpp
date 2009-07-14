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
                                               const char* train_config, const char* pca_config, 
                                               const char* pca_hr_config, const char* pca_desc_config)
{
    m_patch_size = patch_size;
    m_pose_count = pose_count;
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
    
    m_pca_descriptors = new CvOneWayDescriptor[pca_dim + 1];
    if(pca_desc_config && strlen(pca_desc_config) > 0)
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
    }
    
    char outlet_filename[1024];
    char nonoutlet_filename[1024];
    char train_config_filename[1024];
    sprintf(train_config_filename, "%s/%s", train_path, train_config);
    readTrainingBase(train_config_filename, outlet_filename, nonoutlet_filename, m_train_features);
    
    char train_image_filename[1024];
    char train_image_filename1[1024];
    sprintf(train_image_filename, "%s/%s", train_path, outlet_filename);
    sprintf(train_image_filename1, "%s/%s", train_path, nonoutlet_filename);   
    
    LoadTrainingFeatures(train_image_filename, train_image_filename1);
    
    SavePCADescriptors("./pca_descriptors.yml");
    
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

void CvOneWayDescriptorBase::LoadTrainingFeatures(const char* train_image_filename, const char* train_image_filename1)
{
    //#if !defined(_ORANGE_OUTLET)
    //    IplImage* train_image = cvLoadImage(train_image_filename, CV_LOAD_IMAGE_GRAYSCALE);
    //    IplImage* train_image1 = cvLoadImage(train_image_filename1, CV_LOAD_IMAGE_GRAYSCALE);
    //#else
    IplImage* train_image = loadImageRed(train_image_filename);
    IplImage* train_image1 = loadImageRed(train_image_filename1);
    //#endif // _ORANGE_OUTLET
    
    vector<feature_t> outlet_features;
    GetHoleFeatures(train_image, outlet_features);
    //    FilterFeatures(features, min_scale, max_scale);
    
    //    SelectNeighborFeatures(outlet_features, train_features);
    
    vector<feature_t> non_outlet_features;
    GetHoleFeatures(train_image1, non_outlet_features);
    const int max_background_feature_count = 100;
    if((int)non_outlet_features.size() > max_background_feature_count)
    {
        non_outlet_features.resize(max_background_feature_count);
    }
    
    int train_feature_count = outlet_features.size() + non_outlet_features.size();
    printf("Found %d train points...\n", train_feature_count);
    
    m_train_feature_count = train_feature_count;
    m_descriptors = new CvOneWayDescriptor[m_train_feature_count];
    
    //    IplImage* train_image_features = cvCloneImage(train_image);
    //    DrawFeatures(train_image_features, outlet_features);
    
    for(int i = 0; i < (int)outlet_features.size(); i++)
    {
        CvPoint center = outlet_features[i].center;
        
        CvRect roi = cvRect(center.x - m_patch_size.width/2, center.y - m_patch_size.height/2, m_patch_size.width, m_patch_size.height);
        cvSetImageROI(train_image, roi);
        roi = cvGetImageROI(train_image);
        if(roi.width != m_patch_size.width || roi.height != m_patch_size.height)
        {
            continue;
        }
        
        //        m_descriptors[i].SetTransforms(m_poses, m_transforms);
        if(!m_pca_hr_eigenvectors)
        {
            m_descriptors[i].Initialize(m_pose_count, train_image, train_image_filename);
        }
        else
        {
            m_descriptors[i].InitializeFast(m_pose_count, train_image, train_image_filename, 
                                            m_pca_hr_avg, m_pca_hr_eigenvectors, m_pca_descriptors);
        }
        
        m_descriptors[i].InitializePCACoeffs(m_pca_avg, m_pca_eigenvectors);
    }
    cvResetImageROI(train_image);
    m_object_feature_count = outlet_features.size();
    
    for(int i = 0; i < (int)non_outlet_features.size(); i++)
    {
        CvPoint center = non_outlet_features[i].center;
        
        CvRect roi = cvRect(center.x - m_patch_size.width/2, center.y - m_patch_size.height/2, m_patch_size.width, m_patch_size.height);
        cvSetImageROI(train_image1, roi);
        roi = cvGetImageROI(train_image1);
        if(roi.width != m_patch_size.width || roi.height != m_patch_size.height)
        {
            continue;
        }
        
        if(!m_pca_hr_eigenvectors)
        {
            m_descriptors[outlet_features.size() + i].Initialize(m_pose_count, train_image1, train_image_filename1);
        }
        else
        {
            m_descriptors[outlet_features.size() + i].InitializeFast(m_pose_count, train_image1, train_image_filename1,
                                                                     m_pca_hr_avg, m_pca_hr_eigenvectors, m_pca_descriptors);
        }
        
        m_descriptors[outlet_features.size() + i].InitializePCACoeffs(m_pca_avg, m_pca_eigenvectors);
        
        printf("Completed %d out of %d\n", i, (int)non_outlet_features.size());
    }   
    
    cvReleaseImage(&train_image);
    cvReleaseImage(&train_image1);
}

void CvOneWayDescriptorBase::FindDescriptor(IplImage* patch, int& desc_idx, int& pose_idx, float& distance) const
{
    ::FindOneWayDescriptor(m_train_feature_count, m_descriptors, patch, desc_idx, pose_idx, distance, m_pca_avg, m_pca_eigenvectors);
}

int CvOneWayDescriptorBase::IsDescriptorObject(int desc_idx) const
{
    return desc_idx < m_object_feature_count ? 1 : 0;
}

int CvOneWayDescriptorBase::MatchPointToPart(CvPoint pt) const
{
    int idx = -1;
    const int max_dist = 10;
    for(int i = 0; i < (int)m_train_features.size(); i++)
    {
        if(length(pt - m_train_features[i].center) < max_dist)
        {
            idx = i;
            break;
        }
    }
    
    return idx;
}

int CvOneWayDescriptorBase::GetDescriptorPart(int desc_idx) const
{
    return MatchPointToPart(GetDescriptor(desc_idx)->GetCenter());
}

void CvOneWayDescriptorBase::CreatePCADescriptors()
{
    IplImage* frontal = cvCreateImage(m_patch_size, IPL_DEPTH_32F, 1);
    
    eigenvector2image(m_pca_hr_avg, frontal);
    m_pca_descriptors[0].SetTransforms(m_poses, m_transforms);
    m_pca_descriptors[0].Initialize(m_pose_count, frontal, "", 0);
    
    for(int j = 0; j < pca_dim; j++)
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
        
        pca_dim = cvReadInt(node);
        if(m_pca_descriptors)
        {
            delete []m_pca_descriptors;
        }
        m_pca_descriptors = new CvOneWayDescriptor[pca_dim + 1];
        for(int i = 0; i < pca_dim + 1; i++)
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
    
    cvWriteInt(fs, "pca components number", pca_dim);
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
    
    for(int i = 0; i < pca_dim + 1; i++)
    {
        char buf[1024];
        sprintf(buf, "descriptor for pca component %d", i);
        m_pca_descriptors[i].Write(fs, buf);
    }
    
    cvReleaseMemStorage(&storage);
    cvReleaseFileStorage(&fs);
}



void readTrainingBase(const char* config_filename, char* outlet_filename, 
                      char* nonoutlet_filename, vector<feature_t>& train_features)
{
    CvMemStorage* storage = cvCreateMemStorage();
    
    CvFileStorage* fs = cvOpenFileStorage(config_filename, storage, CV_STORAGE_READ);
    
    CvFileNode* outlet_node = cvGetFileNodeByName(fs, 0, "outlet");
    const char* str = cvReadStringByName(fs, outlet_node, "outlet filename");
    strcpy(outlet_filename, str);
    
    CvFileNode* nonoutlet_node = cvGetFileNodeByName(fs, 0, "nonoutlet");
    str = cvReadStringByName(fs, nonoutlet_node, "nonoutlet filename");
    strcpy(nonoutlet_filename, str);
    
    CvPoint pt;
    readCvPointByName(fs, outlet_node, "power1", pt);
    train_features.push_back(feature_t(pt, 1, 0));
    
    readCvPointByName(fs, outlet_node, "power2", pt);
    train_features.push_back(feature_t(pt, 1, 0));
    
    readCvPointByName(fs, outlet_node, "power3", pt);
    train_features.push_back(feature_t(pt, 1, 0));
    
    readCvPointByName(fs, outlet_node, "power4", pt);
    train_features.push_back(feature_t(pt, 1, 0));
    
    readCvPointByName(fs, outlet_node, "ground1", pt);
    train_features.push_back(feature_t(pt, 1, 1));
    
    readCvPointByName(fs, outlet_node, "ground2", pt);
    train_features.push_back(feature_t(pt, 1, 1));
    
    cvReleaseFileStorage(&fs);
    
    cvReleaseMemStorage(&storage);
}

void writeCvPoint(CvFileStorage* fs, const char* name, CvPoint pt)
{
    cvStartWriteStruct(fs, name, CV_NODE_SEQ);
    cvWriteRawData(fs, &pt, 1, "ii");
    cvEndWriteStruct(fs);
}

void readCvPointByName(CvFileStorage* fs, CvFileNode* parent, const char* name, CvPoint& pt)
{
    CvFileNode* node = cvGetFileNodeByName(fs, parent, name);
    cvReadRawData(fs, node, &pt, "ii");
    pt.x /= 2;
    pt.y /= 2;
    
}
