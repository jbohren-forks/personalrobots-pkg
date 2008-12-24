#include "place_recognition/place_recognizer.h"
#include "place_recognition/sparse_stereo.h"

namespace vision {

// TODO: use uint8_t* signatures instead of float*

// TODO: clean up hard-coded numbers
PlaceRecognizer::PlaceRecognizer(const std::string& vocabulary_file,
                                 const std::string& classifier_file,
                                 unsigned int W, unsigned int H)
  : detector_(cvSize(W, H), 7.0, 10.0),
    classifier_(true),
    min_inliers_(40)
{
  tree_.load(vocabulary_file);
  classifier_.read(classifier_file.c_str());
  estimator_.setInlierErrorThreshold(3.0);
  estimator_.setNumRansacIterations(15);
  // Camera parameters for james4
  estimator_.setCameraParams(432.0, 432.0, .088981, 313.7821, 313.7821, 220.407);
}

unsigned int PlaceRecognizer::addPlace(const cv::WImage1_b& left,
                                       const cv::WImage1_b& right)
{
  unsigned int id = image_pts_.size();
  image_all_features_.resize(id + 1);
  image_pts_.resize(id + 1);
  image_descriptors_.resize(id + 1);

  FeatureMatrix& all_features = *image_all_features_.rbegin();
  voKeypoints& keypoints = *image_pts_.rbegin();
  std::vector<float*>& descriptors = *image_descriptors_.rbegin();

  findFeatures(left, right, all_features, keypoints, descriptors);
  tree_.insert(all_features);

  return id;
}

int PlaceRecognizer::findPlace(const cv::WImage1_b& left,
                               const cv::WImage1_b& right,
                               CvMat* rot, CvMat* shift) /*const*/
{
  FeatureMatrix query;
  voKeypoints keypoints;
  std::vector<float*> descriptors;

  findFeatures(left, right, query, keypoints, descriptors);

  // Find top N matches
  std::vector<VocabularyTree::Match> matches;
  matches.reserve(matches_to_check_);
  tree_.find(query, matches_to_check_, std::back_inserter(matches));

  // Set up matcher for signatures in query image
  // TODO: could add signatures directly through findFeatures
  features::BruteForceMatcher<float, int> matcher(classifier_.classes());
  BOOST_FOREACH( float* sig, descriptors )
    matcher.addSignature(sig, 0);

  // Geometric check (find number of inliers)
  float distance; // dummy
  double rot_tmp_[9];
  double shift_tmp_[3];
  CvMat rot_tmp = cvMat(3, 3, CV_64FC1, rot_tmp_);
  CvMat shift_tmp = cvMat(3, 1, CV_64FC1, shift_tmp_);
  int inliers_best = 0;
  int index_best = -1;
  for (unsigned int i = 0; i < matches_to_check_; ++i) {
    // Find matching keypoint pairs
    std::vector< std::pair<int, int> > match_index_pairs;
    unsigned int match_id = matches[i].id;
    for (int index2 = 0; index2 < (int)image_descriptors_[match_id].size(); ++index2) {
      float* sig = image_descriptors_[match_id][index2];
      int index1 = matcher.findMatch(sig, &distance);
      // TODO: some threshold on distance?
      if (index1 >= 0)
        match_index_pairs.push_back( std::make_pair(index1, index2) );
    }
    
    // Run RANSAC, no smoothing (?)
    // TODO: want to do cheap RANSAC on all matches then refine best
    int inliers = estimator_.estimate(keypoints, image_pts_[match_id],
                                      match_index_pairs, rot_tmp, shift_tmp, false);
    if (inliers > (int)min_inliers_ && inliers > inliers_best) {
      inliers_best = inliers;
      index_best = i;
      cvCopy(&rot_tmp, rot);
      cvCopy(&shift_tmp, shift);
    }
  }
  
  return index_best;
}

void PlaceRecognizer::findFeatures(const cv::WImage1_b& left,
                                   const cv::WImage1_b& right,
                                   FeatureMatrix& all_features,
                                   voKeypoints& keypoints,
                                   std::vector<float*>& descriptors) /*const*/
{
  // Detect keypoints in left image
  std::vector<Keypoint> star_pts;
  detector_.DetectPoints(const_cast<IplImage*>(left.Ipl()), std::back_inserter(star_pts));
  
  // Compute descriptor and disparity for each keypoint
  SparseStereoFrame frame(left, right); // TODO: reuse memory here
  all_features.resize((int)star_pts.size(), (int)classifier_.classes());
  float* sig = all_features.data();
  BOOST_FOREACH( const Keypoint& pt, star_pts ) {
    // Signature
    cv::WImageView1_b view = features::extractPatch(const_cast<IplImage*>(left.Ipl()), pt);
    classifier_.getSignature(view.Ipl(), sig);

    // Disparity
    double d = frame.lookupDisparity(pt.x, pt.y);
    if (d > 0) {
      keypoints.push_back( voKeypoint(pt.x, pt.y, d, pt.response, pt.scale, NULL) );
      descriptors.push_back(sig);
    }
    
    sig += classifier_.classes();
  }
}

} // namespace vision
