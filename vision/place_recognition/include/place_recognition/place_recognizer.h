#ifndef _PLACE_RECOGNIZER_H_
#define _PLACE_RECOGNIZER_H_

#include "place_recognition/vocabulary_tree.h"
#include <star_detector/detector.h>
#include <calonder_descriptor/rtree_classifier.h>
#include <calonder_descriptor/matcher.h>
#include <Cv3DPoseEstimateStereo.h>

namespace vision {

class PlaceRecognizer : boost::noncopyable
{
public:
  PlaceRecognizer(const std::string& vocabulary_file,
                  const std::string& classifier_file,
                  unsigned int W, unsigned int H);

  //! Add a place represented by a stereo pair to the database
  unsigned int addPlace(const cv::WImage1_b& left, const cv::WImage1_b& right);

  // TODO: return multiple matches
  //! Returns an image ID and transformation if a good match is found,
  //! -1 otherwise.
  int findPlace(const cv::WImage1_b& left, const cv::WImage1_b& right,
                CvMat* rot, CvMat* shift) /*const*/;

  //findOrAddPlace
  
  //setCameraParams

  //! Number of place matches to feed to the geometric check
  unsigned int matchesToCheck() const { return matches_to_check_; }
  void setMatchesToCheck(unsigned int mc) { matches_to_check_ = mc; }
  
  unsigned int minInliers() const { return min_inliers_; }
  void setMinInliers(unsigned int min_inliers) { min_inliers_ = min_inliers; }
  
private:
  typedef cv::willow::Keypoint voKeypoint;
  typedef cv::willow::Keypoints voKeypoints;

  void findFeatures(const cv::WImage1_b& left, const cv::WImage1_b& right,
                    FeatureMatrix& all_features, voKeypoints& keypoints,
                    std::vector<float*>& descriptors) /*const*/;
  
  VocabularyTree tree_;
  cv::willow::PoseEstimateStereo estimator_;
  StarDetector detector_;
  features::RTreeClassifier classifier_;
  // TODO: this is pretty clunky, need C-descriptor support in visual_odometry
  std::vector< FeatureMatrix > image_all_features_;
  std::vector<voKeypoints> image_pts_;
  std::vector< std::vector<float*> > image_descriptors_;
  unsigned int matches_to_check_;
  unsigned int min_inliers_;
};

} // namespace vision

#endif
