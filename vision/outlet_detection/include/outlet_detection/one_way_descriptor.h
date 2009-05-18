/*
 *  homography_transform.cpp
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 4/19/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_ONE_WAY_DESCRIPTOR)
#define _ONE_WAY_DESCRIPTOR

#include <cv.h>

#include <vector>
#include <string>
using namespace std;

#include "outlet_detection/features.h"

/*
inline CvRect fit_rect_roi(CvRect rect, CvRect roi)
{
	CvRect fit = rect;
	fit.x = MAX(fit.x, roi.x);
	fit.y = MAX(fit.y, roi.y);
	fit.width = MIN(fit.width, roi.x + roi.width - fit.x - 1);
	fit.height = MIN(fit.height, roi.y + roi.height - fit.y - 1);
	assert(fit.width > 0);
	assert(fit.height > 0);
	return(fit);
}

inline CvRect fit_rect(CvRect rect, IplImage* img)
{
	CvRect roi = cvGetImageROI(img);
	return fit_rect_roi(rect, roi);
}

inline CvRect double_rect(CvRect small_rect)
{
	return cvRect(small_rect.x - small_rect.width/2, small_rect.y - small_rect.height/2,
				  small_rect.width*2, small_rect.height*2);
}*/


class CvCameraPose
{
public:
    CvCameraPose()
    {
        m_rotation = cvCreateMat(1, 3, CV_32FC1);
        m_translation = cvCreateMat(1, 3, CV_32FC1);
    };
    
    ~CvCameraPose()
    {
        cvReleaseMat(&m_rotation);
        cvReleaseMat(&m_translation);
    };
    
    void SetPose(CvMat* rotation, CvMat* translation)
    {
        cvCopy(rotation, m_rotation);
        cvCopy(translation, m_translation);
    };
    
    CvMat* GetRotation() {return m_rotation;};
    CvMat* GetTranslation() {return m_translation;};    

protected:
    CvMat* m_rotation;
    CvMat* m_translation;
};

class CvAffinePose
{
public:
    float phi;
    float theta;
    float lambda1;
    float lambda2;
};

void AffineTransformPatch(IplImage* src, IplImage* dst, CvAffinePose pose);


class CvOneWayDescriptor
{
public:
    CvOneWayDescriptor();
    ~CvOneWayDescriptor();
    
    void Allocate(int num_samples, IplImage* frontal);
    void GenerateSamples(int num_samples, IplImage* frontal);
    void CalcInitialPose(CvRect roi);
    void Initialize(int num_samples, IplImage* frontal, const char* image_name = 0, CvPoint center = cvPoint(0, 0));
    void ProjectPCASample(IplImage* patch, CvMat* avg, CvMat* eigenvectors, CvMat* pca_coeffs);
    void InitializePCACoeffs(CvMat* avg, CvMat* eigenvectors);
    void EstimatePosePCA(IplImage* patch, int& pose_idx, float& distance, CvMat* avg = 0, CvMat* eigenvalues = 0);

    
    IplImage* GetPatch(int index);
    CvAffinePose GetPose(int index) const;

    void CalcInitialPose();
    
    void Save(const char* path);
    
    // returns the index of the closest patch
    void EstimatePose(IplImage* patch, int& pose_idx, float& distance);
    
    const char* GetImageName() const;
    CvPoint GetCenter() const;
    
protected:
    int m_num_samples;
    IplImage** m_samples;
    CvMat** m_pca_coeffs;
    CvAffinePose* m_affine_poses;
    
    string m_image_name;
    CvPoint m_center;
    
};

class CvOneWayDescriptorBase
{
public:
    CvOneWayDescriptorBase(CvSize patch_size, int pose_count, const char* train_path, const char* train_config, const char* pca_config);
    ~CvOneWayDescriptorBase();
    
    CvSize GetPatchSize() const {return m_patch_size;};
    int GetPoseCount() const {return m_pose_count;};
    
    void LoadTrainingFeatures(const char* train_image_filename, const char* train_image_filename1);
    
    const CvOneWayDescriptor* GetDescriptor(int desc_idx) const {return &m_descriptors[desc_idx];};
    void FindDescriptor(IplImage* patch, int& desc_idx, int& pose_idx, float& distance) const;
    
    int IsDescriptorObject(int desc_idx) const;
    int MatchPointToPart(CvPoint pt) const;
    int GetDescriptorPart(int desc_idx) const;
    const vector<feature_t>& GetTrainFeatures() const {return m_train_features;};
    
    
protected:
    CvSize m_patch_size;
    int m_pose_count;
    int m_train_feature_count;
    int m_object_feature_count;
    CvOneWayDescriptor* m_descriptors;
    CvMat* m_pca_avg;
    CvMat* m_pca_eigenvectors;
    vector<feature_t> m_train_features;
};

void readTrainingBase(const char* config_filename, char* outlet_filename, 
                      char* nonoutlet_filename, vector<feature_t>& train_features);
void readCvPointByName(CvFileStorage* fs, CvFileNode* parent, const char* name, CvPoint& pt);

void FindOneWayDescriptor(int desc_count, CvOneWayDescriptor* descriptors, IplImage* patch, int& desc_idx, int& pose_idx, float& distance, 
                          CvMat* avg = 0, CvMat* eigenvalues = 0);
CvMat* ConvertImageToMatrix(IplImage* patch);

#endif //_ONE_WAY_DESCRIPTOR
