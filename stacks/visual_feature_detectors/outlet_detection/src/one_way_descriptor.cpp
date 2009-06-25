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

#include "outlet_detection/features.h"
#include "outlet_detection/pca_features.h"
#include "outlet_detection/one_way_descriptor.h"

#include "outlet_detection/outlet_model.h"

#include <highgui.h>

static int pca_dim = 20;
static int pca_dim_small = 20;

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

void generate_mean_transform(CvSize size, CvAffinePose pose, int num_samples, float noise, CvMat* transform)
{
    cvSetZero(transform);
    CvMat* temp = cvCloneMat(transform);
    
    for(int i = 0; i < num_samples; i++)
    {
        CvAffinePose pose_pert = perturbate_pose(pose, noise);
        GenerateAffineTransformFromPose(size, pose_pert, temp);
        cvAdd(temp, transform, transform);
    }
    
    cvConvertScale(transform, transform, 1.0f/num_samples, 0);
    
    cvReleaseMat(&temp);
}

void generate_mean_patch(IplImage* frontal, IplImage* result, CvAffinePose pose, int num_samples, float noise)
{
    IplImage* sum = cvCreateImage(cvSize(result->width, result->height), IPL_DEPTH_32F, 1);
    IplImage* workspace = cvCloneImage(result);
    IplImage* workspace_float = cvCloneImage(sum);
    
    cvSetZero(sum);
    for(int i = 0; i < num_samples; i++)
    {
        CvAffinePose pose_pert = perturbate_pose(pose, noise);
        AffineTransformPatch(frontal, workspace, pose_pert);
        cvConvertScale(workspace, workspace_float);
        cvAdd(sum, workspace_float, sum);
    }
    
    cvConvertScale(sum, result, 1.0f/num_samples);
            
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
    m_num_samples = 0;
    m_samples = 0;
    m_pca_coeffs = 0;
    m_affine_poses = 0;
    m_transforms = 0;
}

CvOneWayDescriptor::~CvOneWayDescriptor()
{
    if(m_num_samples)
    {
        for(int i = 0; i < m_num_samples; i++)
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

void CvOneWayDescriptor::Allocate(int num_samples, CvSize size, int nChannels)
{
    m_num_samples = num_samples;
    m_samples = new IplImage* [m_num_samples];
    m_pca_coeffs = new CvMat* [m_num_samples];
    
    if(!m_transforms)
    {
        m_affine_poses = new CvAffinePose[m_num_samples];
    }
    
    int length = pca_dim_small;//roi.width*roi.height;
    for(int i = 0; i < m_num_samples; i++)
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

/*void CvOneWayDescriptor::CalcInitialPose(CvRect roi)
{
    CvMat* object_points = cvCreateMat(4, 3, CV_32FC1);
    CvMat* image_points = cvCreateMat(4, 3, CV_32FC1);
    
    cvmSet2DPoint(image_points, 0, 0, cvPoint2D32f(roi.x, roi.y));
    cvmSet2DPoint(image_points, 1, 0, cvPoint2D32f(roi.x + roi.width - 1, roi.y));
    cvmSet2DPoint(image_points, 2, 0, cvPoint2D32f(roi.x + roi.width - 1, roi.y + roi.height - 1));
    cvmSet2DPoint(image_points, 3, 0, cvPoint2D32f(roi.x, roi.y + roi.height - 1));
    
    cvmSet3DPoint(object_points, 0, 0, cvPoint3D32f(0, 0, 0));
    cvmSet3DPoint(object_points, 1, 0, cvPoint3D32f(1, 0, 0));
    cvmSet3DPoint(object_points, 2, 0, cvPoint3D32f(1, 1, 0));
    cvmSet3DPoint(object_points, 3, 0, cvPoint3D32f(0, 1, 0));
    
    CvMat* rotation = cvCreateMat(1, 3, CV_32FC1);
    CvMat* translation = cvCreateMat(1, 3, CV_32FC1);
    
    cvFindExtrinsicCameraParams2(object_points, image_points, 
            m_intrinsic_matrix, m_distortion_params, rotation, translation);
    m_initial_pose.SetPose(rotation, translation);
    
    cvReleaseMat(&object_points);
    cvReleaseMat(&image_points);
    cvReleaseMat(&rotation);
    cvReleaseMat(&translation);
}*/

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

void CvOneWayDescriptor::GenerateSamples(int num_samples, IplImage* frontal, int norm)
{
/*    if(m_transforms)
    {
        GenerateSamplesWithTransforms(num_samples, frontal);
        return;
    }
*/    
    CvRect roi = cvGetImageROI(frontal);
    IplImage* patch_8u = cvCreateImage(cvSize(roi.width/2, roi.height/2), frontal->depth, frontal->nChannels);
    for(int i = 0; i < num_samples; i++)
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
    for(int i = 0; i < m_num_samples; i++)
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

void CvOneWayDescriptor::GenerateSamplesWithTransforms(int num_samples, IplImage* frontal)
{
    CvRect roi = cvGetImageROI(frontal);
    CvRect large_roi = double_rect(roi);
    cvSetImageROI(frontal, large_roi);
    IplImage* patch_8u = cvCreateImage(cvSize(large_roi.width, large_roi.height), IPL_DEPTH_8U, frontal->nChannels);
    
    for(int i = 0; i < num_samples; i++)
    {
        cvWarpAffine(frontal, patch_8u, m_transforms[i]);
        cvSetImageROI(patch_8u, cvRect(roi.width/2, roi.height/2, roi.width, roi.height));
        float sum = cvSum(patch_8u).val[0];
        cvConvertScale(patch_8u, m_samples[i], 1.0f/sum);
        cvResetImageROI(patch_8u);
        
#if 1
        cvNamedWindow("1", 1);
        cvShowImage("1", frontal);
        cvNamedWindow("2", 1);
        cvShowImage("2", patch_8u);
        cvWaitKey();
#endif
}
    
    cvSetImageROI(frontal, roi);
    cvReleaseImage(&patch_8u);
}

void CvOneWayDescriptor::Initialize(int num_samples, IplImage* frontal, const char* image_name, CvPoint center, int norm)
{
    m_image_name = string(image_name);
    m_center = center;
    
    CvRect roi = cvGetImageROI(frontal);
    Allocate(num_samples, cvSize(roi.width, roi.height), frontal->nChannels);
    
    GenerateSamples(num_samples, frontal, norm);
}

void CvOneWayDescriptor::InitializeFast(int num_samples, IplImage* frontal, const char* image_name, CvPoint center,
                    CvMat* pca_hr_avg, CvMat* pca_hr_eigenvectors, CvOneWayDescriptor* pca_descriptors)
{
    m_image_name = string(image_name);
    m_center = center;
    
    CvRect roi = cvGetImageROI(frontal);
    Allocate(num_samples, cvSize(roi.width, roi.height), frontal->nChannels);
    
    GenerateSamplesFast(frontal, pca_hr_avg, pca_hr_eigenvectors, pca_descriptors);
}

void CvOneWayDescriptor::InitializePCACoeffs(CvMat* avg, CvMat* eigenvectors)
{
    for(int i = 0; i < m_num_samples; i++)
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
    
    for(int i = 0; i < m_num_samples; i++)
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
    
    for(int i = 0; i < m_num_samples; i++)
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
    for(int i = 0; i < m_num_samples; i++)
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
    CvMat* mat = cvCreateMat(m_num_samples, m_samples[0]->width*m_samples[0]->height, CV_32FC1);
    
    // prepare data to write as a single matrix
    for(int i = 0; i < m_num_samples; i++)
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
    

    for(int i = 0; i < m_num_samples; i++)
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

const char* CvOneWayDescriptor::GetImageName() const
{
    return m_image_name.c_str();
}

CvPoint CvOneWayDescriptor::GetCenter() const
{
    return m_center;
}

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
        generate_mean_transform(cvSize(m_patch_size.width*2, m_patch_size.height*2), m_poses[i], 
                                num_mean_components, noise_intensity, m_transforms[i]);
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
            m_descriptors[i].Initialize(m_pose_count, train_image, train_image_filename, center);
        }
        else
        {
            m_descriptors[i].InitializeFast(m_pose_count, train_image, train_image_filename, center,
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
            m_descriptors[outlet_features.size() + i].Initialize(m_pose_count, train_image1, train_image_filename1, center);
        }
        else
        {
            m_descriptors[outlet_features.size() + i].InitializeFast(m_pose_count, train_image1, train_image_filename1, center,
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
    m_pca_descriptors[0].Initialize(m_pose_count, frontal, "", cvPoint(m_patch_size.width/2, m_patch_size.height/2), 0);
    
    for(int j = 0; j < pca_dim; j++)
    {
        CvMat eigenvector;
        cvGetSubRect(m_pca_hr_eigenvectors, &eigenvector, cvRect(0, j, m_pca_hr_eigenvectors->cols, 1));
        eigenvector2image(&eigenvector, frontal);
        
        m_pca_descriptors[j + 1].SetTransforms(m_poses, m_transforms);
        m_pca_descriptors[j + 1].Initialize(m_pose_count, frontal, "", cvPoint(m_patch_size.width/2, m_patch_size.height/2), 0);
        
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

