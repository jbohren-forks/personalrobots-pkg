#ifndef FEATURES_MATCHER_H
#define FEATURES_MATCHER_H

#include "calonder_descriptor/signature.h"
#include <cv.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_sparse.hpp>
#include <vector>
#include <utility>

namespace ublas = boost::numeric::ublas;

namespace features {

// TODO: when writing more sophisticated matcher(s), figure out what
//       common interface should really look like
template < typename Signature, typename Data >
class BruteForceMatcher
{
public:
  // TODO: change to some reasonable value, or should we have this at all?
  static const float DEFAULT_THRESHOLD = 100.0f;
  
  explicit BruteForceMatcher(float threshold = DEFAULT_THRESHOLD);
  /*
  explicit BruteForceMatcher(std::vector< Signature > const& signatures,
                             std::vector< Data > const& data,
                             float threshold = DEFAULT_THRESHOLD);
  */

  //void assignSignatures(std::vector< Signature > &signatures);
  
  void addSignature(Signature const& signature, Data const& data);
  //void addSignatures(std::vector< Signature > const& signatures);

  Signature& getSignature(int index);
  const Signature& getSignature(int index) const;
  Data& getData(int index);
  const Data& getData(int index) const;

  int findMatch(Signature const& signature, float *distance) const;

  int findMatchInWindow(Signature const& signature, CvRect window,
                        float *distance) const;

  // Returns top two matches, useful for ratio test
  int findMatches(Signature const& signature, float *d1, int *second,
                  float *d2) const;

  // FIXME: for testing/debugging only
  const std::vector<Signature>& signatures() { return signatures_; }

private:
  std::vector< Signature > signatures_;
  std::vector< Data > data_;
  float threshold_;
};

template < typename Signature, typename Data >
inline
BruteForceMatcher<Signature, Data>::BruteForceMatcher(float threshold)
  : threshold_(threshold)
{}

/*
template < typename Data >
inline
BruteForceMatcher::BruteForceMatcher(std::vector< Signature > const& signatures,
                                     float threshold)
  : signatures_(signatures), threshold_(threshold)
{}

inline
void BruteForceMatcher::assignSignatures(std::vector< Signature > &signatures)
{
  signatures_.swap(signatures);
}
*/

template < typename Signature, typename Data >
inline
void BruteForceMatcher<Signature, Data>::addSignature(Signature const& signature,
                                                      Data const& data)
{
  signatures_.push_back(signature);
  data_.push_back(data);
}

/*
inline
void BruteForceMatcher::addSignatures(std::vector< Signature > const& new_sigs)
{
  signatures_.insert(signatures_.end(), new_sigs.begin(), new_sigs.end());
}
*/

template < typename Signature, typename Data >
inline
Signature& BruteForceMatcher<Signature, Data>::getSignature(int index)
{
  return signatures_[index];
}

template < typename Signature, typename Data >
inline
const Signature& BruteForceMatcher<Signature, Data>::getSignature(int index) const
{
  return signatures_[index];
}

template < typename Signature, typename Data >
inline
Data& BruteForceMatcher<Signature, Data>::getData(int index)
{
  return data_[index];
}

template < typename Signature, typename Data >
inline
const Data& BruteForceMatcher<Signature, Data>::getData(int index) const
{
  return data_[index];
}

template < typename Signature, typename Data >
inline
int BruteForceMatcher<Signature, Data>::findMatch(Signature const& signature,
                                                  float *distance) const
{
  return bestMatch(signature, signatures_, distance, threshold_);
}

template < typename Signature, typename Data >
inline
int BruteForceMatcher<Signature, Data>::findMatchInWindow(Signature const& signature,
                                                          CvRect window,
                                                          float *distance) const
{
  int match = -1;
  float best_distance = threshold_;

  for (int i = 0; i < (int)signatures_.size(); ++i) {
    Data const& data = data_[i];
    if (data.x < window.x || data.y < window.y ||
        data.x >= window.x + window.width ||
        data.y >= window.y + window.height)
      continue;

    float next_distance = squaredDistance(signature, signatures_[i]);
    if (next_distance < best_distance) {
      best_distance = next_distance;
      match = i;
    }
  }

  *distance = best_distance;
  return match;
}

template < typename Signature, typename Data >
inline
int BruteForceMatcher<Signature, Data>::findMatches(Signature const& signature,
                                                    float *d1, int *second,
                                                    float *d2) const
{
  return bestTwoMatches(signature, signatures_, d1, second, d2, threshold_);
}

} // namespace features

#endif
