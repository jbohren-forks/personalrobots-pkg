/*
 *  homography_transform.h
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 4/19/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */


static const float pi = 3.1415926;

#include <stdio.h>
#include <vector>
using namespace std;

#include "outlet_detection/one_way_descriptor.h"

#include <highgui.h>

static inline CvPoint rect_center(CvRect rect)
{
	return cvPoint(rect.x + rect.width/2, rect.y + rect.height/2);
}

void homography_transform(IplImage* frontal, IplImage* result, CvMat* homography)
{
    cvWarpPerspective(frontal, result, homography);
}

CvAffinePose perturbate_pose(CvAffinePose pose, float noise)
{
    // perturbate the matrix
    float noise_mult_factor = 1 + (0.5f - float(rand())/RAND_MAX)*noise;
    float noise_add_factor = noise_mult_factor - 1;
    
    CvAffinePose pose_pert = pose;
    pose_pert.phi += noise_add_factor;
    pose_pert.theta += noise_mult_factor;
    pose_pert.lambda1 *= noise_mult_factor;
    pose_pert.lambda2 *= noise_mult_factor;

    return pose_pert;
}

void generate_mean_patch(IplImage* frontal, IplImage* result, CvAffinePose pose, int pose_count, float noise)
{
    IplImage* sum = cvCreateImage(cvSize(result->width, result->height), IPL_DEPTH_32F, 1);
    IplImage* workspace = cvCloneImage(result);
    IplImage* workspace_float = cvCloneImage(sum);
    
    cvSetZero(sum);
    for(int i = 0; i < pose_count; i++)
    {
        CvAffinePose pose_pert = perturbate_pose(pose, noise);
        AffineTransformPatch(frontal, workspace, pose_pert);
        cvConvertScale(workspace, workspace_float);
        cvAdd(sum, workspace_float, sum);
    }
    
    cvConvertScale(sum, result, 1.0f/pose_count);
            
    cvReleaseImage(&workspace);
    cvReleaseImage(&sum);
    cvReleaseImage(&workspace_float);
}

void generate_mean_patch_fast(IplImage* frontal, IplImage* result, CvAffinePose pose, 
                    CvMat* pca_hr_avg, CvMat* pca_hr_eigenvectors, const CvOneWayDescriptor* pca_descriptors)
{
    for(int i = 0; i < pca_hr_eigenvectors->cols; i++)
    {
        
    }
}


CvOneWayDescriptor::CvOneWayDescriptor()
{
    m_pose_count = 0;
    m_samples = 0;
    m_pca_coeffs = 0;
    m_affine_poses = 0;
    m_transforms = 0;
}

CvOneWayDescriptor::~CvOneWayDescriptor()
{
    if(m_pose_count)
    {
        for(int i = 0; i < m_pose_count; i++)
        {
            cvReleaseImage(&m_samples[i]);
            cvReleaseMat(&m_pca_coeffs[i]);
        }
        delete []m_samples;
        delete []m_pca_coeffs;
        
        if(!m_transforms)
        {
            delete []m_affine_poses;
        }
    }
}

void CvOneWayDescriptor::Allocate(int pose_count, CvSize size, int nChannels)
{
    m_pose_count = pose_count;
    m_samples = new IplImage* [m_pose_count];
    m_pca_coeffs = new CvMat* [m_pose_count];
    
    if(!m_transforms)
    {
        m_affine_poses = new CvAffinePose[m_pose_count];
    }
    
    int length = pca_dim_small;//roi.width*roi.height;
    for(int i = 0; i < m_pose_count; i++)
    {
        m_samples[i] = cvCreateImage(cvSize(size.width/2, size.height/2), IPL_DEPTH_32F, nChannels);
        m_pca_coeffs[i] = cvCreateMat(1, length, CV_32FC1);
    }
}

void cvmSet2DPoint(CvMat* matrix, int row, int col, CvPoint2D32f point)
{
    cvmSet(matrix, row, col, point.x);
    cvmSet(matrix, row, col + 1, point.y);
}

void cvmSet3DPoint(CvMat* matrix, int row, int col, CvPoint3D32f point)
{
    cvmSet(matrix, row, col, point.x);
    cvmSet(matrix, row, col + 1, point.y);
    cvmSet(matrix, row, col + 2, point.z);
}

CvAffinePose GenRandomAffinePose()
{
    const float scale_min = 0.8f;
    const float scale_max = 1.2f;
    CvAffinePose pose;
    pose.theta = float(rand())/RAND_MAX*120 - 60;
    pose.phi = float(rand())/RAND_MAX*360;
    pose.lambda1 = scale_min + float(rand())/RAND_MAX*(scale_max - scale_min);
    pose.lambda2 = scale_min + float(rand())/RAND_MAX*(scale_max - scale_min);
    
    return pose;
}

void GenerateAffineTransformFromPose(CvSize size, CvAffinePose pose, CvMat* transform)
{
    CvMat* temp = cvCreateMat(3, 3, CV_32FC1);
    CvMat* final = cvCreateMat(3, 3, CV_32FC1);
    cvmSet(temp, 2, 0, 0.0f);
    cvmSet(temp, 2, 1, 0.0f);
    cvmSet(temp, 2, 2, 1.0f);

    CvMat rotation;
    cvGetSubRect(temp, &rotation, cvRect(0, 0, 3, 2));
    
    cv2DRotationMatrix(cvPoint2D32f(size.width/2, size.height/2), pose.phi, 1.0, &rotation);
    cvCopy(temp, final);

    cvmSet(temp, 0, 0, pose.lambda1);
    cvmSet(temp, 0, 1, 0.0f);
    cvmSet(temp, 1, 0, 0.0f);
    cvmSet(temp, 1, 1, pose.lambda2);
    cvmSet(temp, 0, 2, size.width/2*(1 - pose.lambda1));
    cvmSet(temp, 1, 2, size.height/2*(1 - pose.lambda2));
    cvMatMul(temp, final, final);
    
    cv2DRotationMatrix(cvPoint2D32f(size.width/2, size.height/2), pose.theta - pose.phi, 1.0, &rotation);
    cvMatMul(temp, final, final);

    cvGetSubRect(final, &rotation, cvRect(0, 0, 3, 2));
    cvCopy(&rotation, transform);   
    
    cvReleaseMat(&temp);
    cvReleaseMat(&final);
}

void AffineTransformPatch(IplImage* src, IplImage* dst, CvAffinePose pose)
{
    CvRect src_large_roi = cvGetImageROI(src);
    
    IplImage* temp = cvCreateImage(cvSize(src_large_roi.width, src_large_roi.height), IPL_DEPTH_32F, src->nChannels);
    cvSetZero(temp);
    IplImage* temp2 = cvCloneImage(temp);
    
    cvConvertScale(src, temp);
    cvResetImageROI(temp);
    
    CvMat* rotation_phi = cvCreateMat(2, 3, CV_32FC1);
    
    cv2DRotationMatrix(cvPoint2D32f(temp->width/2, temp->height/2), pose.phi, 1.0, rotation_phi);
    cvWarpAffine(temp, temp2, rotation_phi);
    
    cvSetZero(temp);
    CvSize new_size = cvSize(temp->width*pose.lambda1, temp->height*pose.lambda2);
    IplImage* temp3 = cvCreateImage(new_size, IPL_DEPTH_32F, src->nChannels);
    cvResize(temp2, temp3);
    
    cv2DRotationMatrix(cvPoint2D32f(temp3->width/2, temp3->height/2), pose.theta - pose.phi, 1.0, rotation_phi);
    cvWarpAffine(temp3, temp, rotation_phi);
    
    cvSetImageROI(temp, cvRect(temp->width/2 - src_large_roi.width/4, temp->height/2 - src_large_roi.height/4, 
                               src_large_roi.width/2, src_large_roi.height/2));
    cvConvertScale(temp, dst);   
    
    cvReleaseMat(&rotation_phi);
    
    cvReleaseImage(&temp);
    cvReleaseImage(&temp2);
    cvReleaseImage(&temp3);
}

void CvOneWayDescriptor::GenerateSamples(int pose_count, IplImage* frontal, int norm)
{
/*    if(m_transforms)
    {
        GenerateSamplesWithTransforms(pose_count, frontal);
        return;
    }
*/    
    CvRect roi = cvGetImageROI(frontal);
    IplImage* patch_8u = cvCreateImage(cvSize(roi.width/2, roi.height/2), frontal->depth, frontal->nChannels);
    for(int i = 0; i < pose_count; i++)
    {
        if(!m_transforms)
        {
            m_affine_poses[i] = GenRandomAffinePose();
        }
        //AffineTransformPatch(frontal, patch_8u, m_affine_poses[i]);
        generate_mean_patch(frontal, patch_8u, m_affine_poses[i], num_mean_components, noise_intensity);

        float scale = 1.0f;
        if(norm)
        {
            float sum = cvSum(patch_8u).val[0];
            scale = 1/sum;
        }
        cvConvertScale(patch_8u, m_samples[i], scale);
        
#if 0
        double maxval;
        cvMinMaxLoc(m_samples[i], 0, &maxval);
        IplImage* test = cvCreateImage(cvSize(roi.width/2, roi.height/2), IPL_DEPTH_8U, 1);
        cvConvertScale(m_samples[i], test, 255.0/maxval);
        cvNamedWindow("1", 1);
        cvShowImage("1", test);
        cvWaitKey(0);
#endif
    }
    cvReleaseImage(&patch_8u);
}

void CvOneWayDescriptor::GenerateSamplesFast(IplImage* frontal, CvMat* pca_hr_avg, 
                                             CvMat* pca_hr_eigenvectors, CvOneWayDescriptor* pca_descriptors)
{
    CvMat* pca_coeffs = cvCreateMat(1, pca_hr_eigenvectors->cols, CV_32FC1);
    double maxval;
    cvMinMaxLoc(frontal, 0, &maxval);
    CvMat* frontal_data = ConvertImageToMatrix(frontal);
    
    float sum = cvSum(frontal_data).val[0];
    cvConvertScale(frontal_data, frontal_data, 1.0f/sum);
    cvProjectPCA(frontal_data, pca_hr_avg, pca_hr_eigenvectors, pca_coeffs);
    for(int i = 0; i < m_pose_count; i++)
    {
        cvSetZero(m_samples[i]);
        for(int j = 0; j < pca_dim; j++)
        {
            float coeff = cvmGet(pca_coeffs, 0, j);
            IplImage* patch = pca_descriptors[j + 1].GetPatch(i);
            cvAddWeighted(m_samples[i], 1.0, patch, coeff, 0, m_samples[i]);
            
#if 0
            printf("coeff%d = %f\n", j, coeff);
            IplImage* test = cvCreateImage(cvSize(12, 12), IPL_DEPTH_8U, 1);
            double maxval;
            cvMinMaxLoc(patch, 0, &maxval);
            cvConvertScale(patch, test, 255.0/maxval);
            cvNamedWindow("1", 1);
            cvShowImage("1", test);
            cvWaitKey(0);
#endif
        }
        
        cvAdd(pca_descriptors[0].GetPatch(i), m_samples[i], m_samples[i]);
        float sum = cvSum(m_samples[i]).val[0];
        cvConvertScale(m_samples[i], m_samples[i], 1.0/sum);
        
#if 0
        IplImage* test = cvCreateImage(cvSize(12, 12), IPL_DEPTH_8U, 1);
/*        IplImage* temp1 = cvCreateImage(cvSize(12, 12), IPL_DEPTH_32F, 1);
        eigenvector2image(pca_hr_avg, temp1);
        IplImage* test = cvCreateImage(cvSize(12, 12), IPL_DEPTH_8U, 1);
        cvAdd(m_samples[i], temp1, temp1);
        cvMinMaxLoc(temp1, 0, &maxval);
        cvConvertScale(temp1, test, 255.0/maxval);*/
        cvMinMaxLoc(m_samples[i], 0, &maxval);
        cvConvertScale(m_samples[i], test, 255.0/maxval);

        cvNamedWindow("1", 1);
        cvShowImage("1", frontal);
        cvNamedWindow("2", 1);
        cvShowImage("2", test);
        cvWaitKey(0);
#endif
    }
    
    cvReleaseMat(&pca_coeffs);
    cvReleaseMat(&frontal_data);
}

void CvOneWayDescriptor::SetTransforms(CvAffinePose* poses, CvMat** transforms)
{
    if(m_affine_poses)
    {
        delete []m_affine_poses;
    }
    
    m_affine_poses = poses;
    m_transforms = transforms;
}

void CvOneWayDescriptor::Initialize(int pose_count, IplImage* frontal, const char* feature_name, int norm)
{
    m_feature_name = string(feature_name);
    CvRect roi = cvGetImageROI(frontal);
    m_center = rect_center(roi);
    
    Allocate(pose_count, cvSize(roi.width, roi.height), frontal->nChannels);
    
    GenerateSamples(pose_count, frontal, norm);
}

void CvOneWayDescriptor::InitializeFast(int pose_count, IplImage* frontal, const char* feature_name, 
                    CvMat* pca_hr_avg, CvMat* pca_hr_eigenvectors, CvOneWayDescriptor* pca_descriptors)
{
    m_feature_name = string(feature_name);
    CvRect roi = cvGetImageROI(frontal);
    m_center = rect_center(roi);
    
    Allocate(pose_count, cvSize(roi.width, roi.height), frontal->nChannels);
    
    GenerateSamplesFast(frontal, pca_hr_avg, pca_hr_eigenvectors, pca_descriptors);
}

void CvOneWayDescriptor::InitializePCACoeffs(CvMat* avg, CvMat* eigenvectors)
{
    for(int i = 0; i < m_pose_count; i++)
    {
        ProjectPCASample(m_samples[i], avg, eigenvectors, m_pca_coeffs[i]);
    }
}

void CvOneWayDescriptor::ProjectPCASample(IplImage* patch, CvMat* avg, CvMat* eigenvectors, CvMat* pca_coeffs)
{
    CvMat* patch_mat = ConvertImageToMatrix(patch);
//    CvMat eigenvectorsr;
//    cvGetSubRect(eigenvectors, &eigenvectorsr, cvRect(0, 0, eigenvectors->cols, pca_coeffs->cols));
    CvMat* temp = cvCreateMat(1, eigenvectors->cols, CV_32FC1);
    cvProjectPCA(patch_mat, avg, eigenvectors, temp);
    CvMat temp1;
    cvGetSubRect(temp, &temp1, cvRect(0, 0, pca_coeffs->cols, 1));
    cvCopy(&temp1, pca_coeffs);
    
    cvReleaseMat(&temp);
    cvReleaseMat(&patch_mat);
}

void CvOneWayDescriptor::EstimatePosePCA(IplImage* patch, int& pose_idx, float& distance, CvMat* avg, CvMat* eigenvectors)
{
    CvRect roi = cvGetImageROI(patch);
    CvMat* pca_coeffs = cvCreateMat(1, pca_dim_small, CV_32FC1);
    
    IplImage* patch_32f = cvCreateImage(cvSize(roi.width, roi.height), IPL_DEPTH_32F, 1);
    float sum = cvSum(patch).val[0];
    cvConvertScale(patch, patch_32f, 1.0f/sum);
 
    ProjectPCASample(patch_32f, avg, eigenvectors, pca_coeffs);

    distance = 1e10;
    pose_idx = -1;
    
    for(int i = 0; i < m_pose_count; i++)
    {
        float dist = cvNorm(m_pca_coeffs[i], pca_coeffs);
        if(dist < distance)
        {
            distance = dist;
            pose_idx = i;
        }
    }
    
    cvReleaseMat(&pca_coeffs);
    cvReleaseImage(&patch_32f);
}

void CvOneWayDescriptor::EstimatePose(IplImage* patch, int& pose_idx, float& distance)
{
    distance = 1e10;
    pose_idx = -1;
    
    CvRect roi = cvGetImageROI(patch);
    IplImage* patch_32f = cvCreateImage(cvSize(roi.width, roi.height), IPL_DEPTH_32F, patch->nChannels);
    float sum = cvSum(patch).val[0];
    cvConvertScale(patch, patch_32f, 1/sum);
    
    for(int i = 0; i < m_pose_count; i++)
    {
        if(m_samples[i]->width != patch_32f->width || m_samples[i]->height != patch_32f->height)
        {
            continue;
        }
        float dist = cvNorm(m_samples[i], patch_32f);
        if(dist < distance)
        {
            distance = dist;
            pose_idx = i;
        }
        
#if 0
        IplImage* img1 = cvCreateImage(cvSize(roi.width, roi.height), IPL_DEPTH_8U, 1);
        IplImage* img2 = cvCreateImage(cvSize(roi.width, roi.height), IPL_DEPTH_8U, 1);
        double maxval;
        cvMinMaxLoc(m_samples[i], 0, &maxval);
        cvConvertScale(m_samples[i], img1, 255.0/maxval);
        cvMinMaxLoc(patch_32f, 0, &maxval);
        cvConvertScale(patch_32f, img2, 255.0/maxval);
        
        cvNamedWindow("1", 1);
        cvShowImage("1", img1);
        cvNamedWindow("2", 1);
        cvShowImage("2", img2);
        printf("Distance = %f\n", dist);
        cvWaitKey(0);
#endif
    }
    
    cvReleaseImage(&patch_32f);
}

void CvOneWayDescriptor::Save(const char* path)
{
    for(int i = 0; i < m_pose_count; i++)
    {
        char buf[1024];
        sprintf(buf, "%s/patch_%04d.jpg", path, i);
        IplImage* patch = cvCreateImage(cvSize(m_samples[i]->width, m_samples[i]->height), IPL_DEPTH_8U, m_samples[i]->nChannels);
        
        double maxval;
        cvMinMaxLoc(m_samples[i], 0, &maxval);
        cvConvertScale(m_samples[i], patch, 255/maxval);
        
        cvSaveImage(buf, patch);
        
        cvReleaseImage(&patch);
    }
}

void CvOneWayDescriptor::Write(CvFileStorage* fs, const char* name)
{
    CvMat* mat = cvCreateMat(m_pose_count, m_samples[0]->width*m_samples[0]->height, CV_32FC1);
    
    // prepare data to write as a single matrix
    for(int i = 0; i < m_pose_count; i++)
    {
        for(int y = 0; y < m_samples[i]->height; y++)
        {
            for(int x = 0; x < m_samples[i]->width; x++)
            {
                float val = *((float*)(m_samples[i]->imageData + m_samples[i]->widthStep*y) + x);
                cvmSet(mat, i, y*m_samples[i]->width + x, val);
            }
        }
    }
    
    cvWrite(fs, name, mat);
    
    cvReleaseMat(&mat);
}

int CvOneWayDescriptor::ReadByName(CvFileStorage* fs, CvFileNode* parent, const char* name)
{
    CvMat* mat = (CvMat*)cvReadByName(fs, parent, name);
    if(!mat)
    {
        return 0;
    }
    

    for(int i = 0; i < m_pose_count; i++)
    {
        for(int y = 0; y < m_samples[i]->height; y++)
        {
            for(int x = 0; x < m_samples[i]->width; x++)
            {
                float val = cvmGet(mat, i, y*m_samples[i]->width + x);
                *((float*)(m_samples[i]->imageData + y*m_samples[i]->widthStep) + x) = val;
            }
        }
    }
            
    cvReleaseMat(&mat);
    return 1;
}

IplImage* CvOneWayDescriptor::GetPatch(int index)
{
    return m_samples[index];
}

CvAffinePose CvOneWayDescriptor::GetPose(int index) const
{
    return m_affine_poses[index];
}

void FindOneWayDescriptor(int desc_count, CvOneWayDescriptor* descriptors, IplImage* patch, int& desc_idx, int& pose_idx, float& distance, 
    CvMat* avg, CvMat* eigenvectors)
{
    desc_idx = -1;
    pose_idx = -1;
    distance = 1e10;
    for(int i = 0; i < desc_count; i++)
    {
        int _pose_idx = -1;
        float _distance = 0;

#if 0
        descriptors[i].EstimatePose(patch, _pose_idx, _distance);
#else
        descriptors[i].EstimatePosePCA(patch, _pose_idx, _distance, avg, eigenvectors);
#endif
        
        if(_distance < distance)
        {
            desc_idx = i;
            pose_idx = _pose_idx;
            distance = _distance;
        }
    }
}

const char* CvOneWayDescriptor::GetFeatureName() const
{
    return m_feature_name.c_str();
}

CvPoint CvOneWayDescriptor::GetCenter() const
{
    return m_center;
}


