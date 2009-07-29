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

#include <cv.h>

#include "outlet_detection/one_way_descriptor.h"

// CvOneWayDescriptorBase: encapsulates functionality for training/loading a set of one way descriptors
// and finding the nearest closest descriptor to an input feature
class CvOneWayDescriptorBase
    {
    public:
        
        // creates an instance of CvOneWayDescriptor from a set of training files
        // - patch_size: size of the input (large) patch
        // - pose_count: the number of poses to generate for each descriptor
        // - train_path: path to training files
        // - train_config: configuration filename
        // - pca_config: the name of the file that contains PCA for small patches (2 times smaller
        // than patch_size each dimension
        // - pca_hr_config: the name of the file that contains PCA for large patches (of patch_size size)
        // - pca_desc_config: the name of the file that contains descriptors of PCA components
        CvOneWayDescriptorBase(CvSize patch_size, int pose_count, const char* train_path, const char* train_config, 
                               const char* pca_config, const char* pca_hr_config = 0, const char* pca_desc_config = 0, 
                                int pyr_levels = 2);
        
        ~CvOneWayDescriptorBase();
        
        // returns patch size
        CvSize GetPatchSize() const {return m_patch_size;};
        // returns the number of poses for each descriptor
        int GetPoseCount() const {return m_pose_count;};
        
        // returns the number of pyramid levels
        int GetPyrLevels() const {return m_pyr_levels;};
        
        // LoadTrainingFeatures: loads positive and negative features from images
        // - train_image_filename: positive image
        // - train_image_filename1: negative image
        void LoadTrainingFeatures(const char* train_image_filename, const char* train_image_filename1);
        
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
        
        // InitializePoses: generates random poses
        void InitializePoses();
        
        // InitializeTransformsFromPoses: generates 2x3 affine matrices from poses (initializes m_transforms)
        void InitializeTransformsFromPoses();
        
        // InitializePoseTransforms: subsequently calls InitializePoses and InitializeTransformsFromPoses
        void InitializePoseTransforms();
        
        // IsDescriptorObject: returns 1 if descriptor with specified index is positive, otherwise 0
        int IsDescriptorObject(int desc_idx) const;
        
        // MatchPointToPart: returns the part number of a feature if it matches one of the object parts, otherwise -1
        int MatchPointToPart(CvPoint pt) const;
        
        // GetDescriptorPart: returns the part number of the feature corresponding to a specified descriptor  
        // - desc_idx: descriptor index
        int GetDescriptorPart(int desc_idx) const;
        
        // GetTrainFeatures: returns a set of training features
        const vector<feature_t>& GetTrainFeatures() const {return m_train_features;};
        
        // LoadPCADescriptors: loads PCA descriptors from a file
        // - filename: input filename
        int LoadPCADescriptors(const char* filename);
        
        // SavePCADescriptors: saves PCA descriptors to a file
        // - filename: output filename
        void SavePCADescriptors(const char* filename);
        
    protected:
        void BuildDescriptors(IplImage* train_image, const vector<feature_t>& features, 
                         const char* feature_label, CvOneWayDescriptor* descriptors, int* part_id = 0, float scale = 1.0f);
        
        
        
    protected:
        CvSize m_patch_size; // patch size
        int m_pose_count; // the number of poses for each descriptor
        int m_train_feature_count; // the number of the training features
        int m_object_feature_count; // the number of the positive features
        CvOneWayDescriptor* m_descriptors; // array of train feature descriptors
        CvMat* m_pca_avg; // PCA average vector for small patches
        CvMat* m_pca_eigenvectors; // PCA eigenvectors for small patches
        CvMat* m_pca_hr_avg; // PCA average vector for large patches
        CvMat* m_pca_hr_eigenvectors; // PCA eigenvectors for large patches
        CvOneWayDescriptor* m_pca_descriptors; // an array of PCA descriptors
        vector<feature_t> m_train_features; // train features
        
        CvAffinePose* m_poses; // array of poses
        CvMat** m_transforms; // array of affine transformations corresponding to poses
        
        int* m_part_id; // contains part id for each of object descriptors
        int m_pyr_levels;
    };

void readTrainingBase(const char* config_filename, char* outlet_filename, 
                      char* nonoutlet_filename, vector<feature_t>& train_features);
void readCvPointByName(CvFileStorage* fs, CvFileNode* parent, const char* name, CvPoint& pt);



#endif // _ONE_WAY_DESCRIPTOR_BASE
