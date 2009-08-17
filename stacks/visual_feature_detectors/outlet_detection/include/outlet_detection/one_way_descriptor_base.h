/*
 *  one_way_descriptor_base.h
 *  outlet_detection
 *
 *  Created by Victor  Eruhimov on 7/9/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_ONE_WAY_DESCRIPTOR_BASE)
#define _ONE_WAY_DESCRIPTOR_BASE

#include <vector>
using namespace std;

#include <cv.h>

#include "outlet_detection/one_way_descriptor.h"

inline CvRect fit_rect_roi_fixedsize(CvRect rect, CvRect roi)
{
	CvRect fit = rect;
	fit.x = MAX(fit.x, roi.x);
	fit.y = MAX(fit.y, roi.y);
    fit.x = MIN(fit.x, roi.x + roi.width - fit.width - 1);
    fit.y = MIN(fit.y, roi.y + roi.height - fit.height - 1);
	return(fit);
}

inline CvRect fit_rect_fixedsize(CvRect rect, IplImage* img)
{
	CvRect roi = cvGetImageROI(img);
	return fit_rect_roi_fixedsize(rect, roi);
}

class CV_EXPORTS KeyPointEx : public KeyPoint
{
public:
    KeyPointEx(CvPoint _center = cvPoint(-1, -1), float _scale = 1, int _class_id = -1) : 
    KeyPoint(_center.x, _center.y, _scale, 0.0f, 0.0f, 0)
    {
        class_id = _class_id;
    };
    
    KeyPointEx(const KeyPoint& keypoint) : KeyPoint(keypoint)
    {
    };
    
    ~KeyPointEx() {};
    
    int class_id;
};

typedef KeyPointEx feature_t;

// CvOneWayDescriptorBase: encapsulates functionality for training/loading a set of one way descriptors
// and finding the nearest closest descriptor to an input feature
class CvOneWayDescriptorBase
    {
    public:
        
        // creates an instance of CvOneWayDescriptor from a set of training files
        // - patch_size: size of the input (large) patch
        // - pose_count: the number of poses to generate for each descriptor
        // - train_path: path to training files
        // - pca_config: the name of the file that contains PCA for small patches (2 times smaller
        // than patch_size each dimension
        // - pca_hr_config: the name of the file that contains PCA for large patches (of patch_size size)
        // - pca_desc_config: the name of the file that contains descriptors of PCA components
        CvOneWayDescriptorBase(CvSize patch_size, int pose_count, const char* train_path = 0, const char* pca_config = 0, 
                               const char* pca_hr_config = 0, const char* pca_desc_config = 0, int pyr_levels = 2, 
                                int pca_dim_high = 100, int pca_dim_low = 100);
        
        ~CvOneWayDescriptorBase();
        
        // Allocate: allocates memory for a given number of descriptors
        void Allocate(int train_feature_count);
        
        // AllocatePCADescriptors: allocates memory for pca descriptors
        void AllocatePCADescriptors();
        
        // returns patch size
        CvSize GetPatchSize() const {return m_patch_size;};
        // returns the number of poses for each descriptor
        int GetPoseCount() const {return m_pose_count;};
        
        // returns the number of pyramid levels
        int GetPyrLevels() const {return m_pyr_levels;};
        
        // CreateDescriptorsFromImage: creates descriptors for each of the input features
        // - src: input image
        // - features: input features
        // - pyr_levels: the number of pyramid levels
        void CreateDescriptorsFromImage(IplImage* src, const Vector<KeyPointEx>& features);
        
        // CreatePCADescriptors: generates descriptors for PCA components, needed for fast generation of feature descriptors
        void CreatePCADescriptors();
        
        // returns a feature descriptor by feature index
        const CvOneWayDescriptor* GetDescriptor(int desc_idx) const {return &m_descriptors[desc_idx];};
        
        // FindDescriptor: finds the closest descriptor
        // - patch: input image patch
        // - desc_idx: output index of the closest descriptor to the input patch
        // - pose_idx: output index of the closest pose of the closest descriptor to the input patch
        // - distance: distance from the input patch to the closest feature pose
        void FindDescriptor(IplImage* patch, int& desc_idx, int& pose_idx, float& distance) const;
        
        // FindDescriptor: finds the closest descriptor
        // - src: input image 
        // - pt: center of the feature
        // - desc_idx: output index of the closest descriptor to the input patch
        // - pose_idx: output index of the closest pose of the closest descriptor to the input patch
        // - distance: distance from the input patch to the closest feature pose
        void FindDescriptor(IplImage* src, Point2f pt, int& desc_idx, int& pose_idx, float& distance) const;
        
        // InitializePoses: generates random poses
        void InitializePoses();
        
        // InitializeTransformsFromPoses: generates 2x3 affine matrices from poses (initializes m_transforms)
        void InitializeTransformsFromPoses();
        
        // InitializePoseTransforms: subsequently calls InitializePoses and InitializeTransformsFromPoses
        void InitializePoseTransforms();
        
        // InitializeDescriptor: initializes a descriptor
        // - desc_idx: descriptor index
        // - train_image: image patch (ROI is supported)
        // - feature_label: feature textual label
        void InitializeDescriptor(int desc_idx, IplImage* train_image, const char* feature_label);
        
        // InitializeDescriptors: load features from an image and create descriptors for each of them 
        void InitializeDescriptors(IplImage* train_image, const Vector<KeyPointEx>& features, 
                              const char* feature_label = "", int desc_start_idx = 0);
        
        // LoadPCADescriptors: loads PCA descriptors from a file
        // - filename: input filename
        int LoadPCADescriptors(const char* filename);
        
        // SavePCADescriptors: saves PCA descriptors to a file
        // - filename: output filename
        void SavePCADescriptors(const char* filename);
        
        // SetPCAHigh: sets the high resolution pca matrices (copied to internal structures)
        void SetPCAHigh(CvMat* avg, CvMat* eigenvectors);
        
        // SetPCALow: sets the low resolution pca matrices (copied to internal structures)
        void SetPCALow(CvMat* avg, CvMat* eigenvectors);
        
        
    protected:
        CvSize m_patch_size; // patch size
        int m_pose_count; // the number of poses for each descriptor
        int m_train_feature_count; // the number of the training features
        CvOneWayDescriptor* m_descriptors; // array of train feature descriptors
        CvMat* m_pca_avg; // PCA average Vector for small patches
        CvMat* m_pca_eigenvectors; // PCA eigenvectors for small patches
        CvMat* m_pca_hr_avg; // PCA average Vector for large patches
        CvMat* m_pca_hr_eigenvectors; // PCA eigenvectors for large patches
        CvOneWayDescriptor* m_pca_descriptors; // an array of PCA descriptors
        
        CvAffinePose* m_poses; // array of poses
        CvMat** m_transforms; // array of affine transformations corresponding to poses
        
        int m_pca_dim_high;
        int m_pca_dim_low;
        
        int m_pyr_levels;
};

class CvOneWayDescriptorObject : public CvOneWayDescriptorBase
{
    public:
        // creates an instance of CvOneWayDescriptorObject from a set of training files
        // - patch_size: size of the input (large) patch
        // - pose_count: the number of poses to generate for each descriptor
        // - train_path: path to training files
        // - pca_config: the name of the file that contains PCA for small patches (2 times smaller
        // than patch_size each dimension
        // - pca_hr_config: the name of the file that contains PCA for large patches (of patch_size size)
        // - pca_desc_config: the name of the file that contains descriptors of PCA components
        CvOneWayDescriptorObject(CvSize patch_size, int pose_count, const char* train_path, const char* pca_config, 
                                 const char* pca_hr_config = 0, const char* pca_desc_config = 0, int pyr_levels = 2);
        
        ~CvOneWayDescriptorObject();
        
        // Allocate: allocates memory for a given number of features
        // - train_feature_count: the total number of features
        // - object_feature_count: the number of features extracted from the object 
        void Allocate(int train_feature_count, int object_feature_count);
        
        
        void SetLabeledFeatures(const vector<KeyPointEx>& features) {m_train_features = features;};
        vector<KeyPointEx>& GetLabeledFeatures() {return m_train_features;};
        const vector<KeyPointEx>& GetLabeledFeatures() const {return m_train_features;};
        vector<feature_t> _GetLabeledFeatures() const;
        
        // IsDescriptorObject: returns 1 if descriptor with specified index is positive, otherwise 0
        int IsDescriptorObject(int desc_idx) const;
        
        // MatchPointToPart: returns the part number of a feature if it matches one of the object parts, otherwise -1
        int MatchPointToPart(CvPoint pt) const;
        
        // GetDescriptorPart: returns the part number of the feature corresponding to a specified descriptor  
        // - desc_idx: descriptor index
        int GetDescriptorPart(int desc_idx) const;
        
        
        void InitializeObjectDescriptors(IplImage* train_image, const Vector<KeyPointEx>& features, 
                                         const char* feature_label, int desc_start_idx = 0, float scale = 1.0f);
        
    protected:
        int* m_part_id; // contains part id for each of object descriptors
        vector<KeyPointEx> m_train_features; // train features
        int m_object_feature_count; // the number of the positive features
        
};

void readPCAFeatures(const char* filename, CvMat** avg, CvMat** eigenvectors);
void eigenvector2image(CvMat* eigenvector, IplImage* img);

#endif // _ONE_WAY_DESCRIPTOR_BASE
