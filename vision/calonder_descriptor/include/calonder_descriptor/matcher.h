#ifndef FEATURES_MATCHER_H
#define FEATURES_MATCHER_H

#include "calonder_descriptor/basic_math.h"
#include <cv.h>
#include <limits>
#include <vector>
#include <utility>
#include <boost/foreach.hpp>

namespace features {

template < typename Data >
class BruteForceMatcher
{
public:
  BruteForceMatcher(size_t signature_size);

  // TODO: mostly to get Python bindings working, probably don't want size to be changeable
  BruteForceMatcher() : threshold_(std::numeric_limits<float>::max()), size_(0) {}
  inline void setSize(size_t size) { size_ = size; }
  
  // BruteForceMatcher does NOT take ownership of signature's memory
  void addSignature(float* signature, Data const& data);

  float* getSignature(int index);
  const float* getSignature(int index) const;
  Data& getData(int index);
  const Data& getData(int index) const;

  int findMatch(const float* signature, float *distance) const;

  int findMatchInWindow(const float* signature, CvRect window,
                        float *distance) const;
  int findMatchPredicated(const float* signature, char *predicates, float *distance) const;

  // Returns top two matches, useful for ratio test
  int findMatches(const float* signature, float *d1, int *second,
                  float *d2) const;

  // TODO: restore threshold functionality if useful. May depend on #trees.
  
  // FIXME: for testing/debugging only
  const std::vector<float*>& signatures() { return signatures_; }

private:
  std::vector< float* > signatures_;
  std::vector< Data > data_;
  float threshold_;
  size_t size_;
};

template < typename Data >
inline
BruteForceMatcher<Data>::BruteForceMatcher(size_t signature_size)
  : threshold_(std::numeric_limits<float>::max()),
    size_(signature_size)
{}

template < typename Data >
inline
void BruteForceMatcher<Data>::addSignature(float* signature, Data const& data)
{
  signatures_.push_back(signature);
  data_.push_back(data);
}

template < typename Data >
inline
float* BruteForceMatcher<Data>::getSignature(int index)
{
  return signatures_[index];
}

template < typename Data >
inline
const float* BruteForceMatcher<Data>::getSignature(int index) const
{
  return signatures_[index];
}

template < typename Data >
inline
Data& BruteForceMatcher<Data>::getData(int index)
{
  return data_[index];
}

template < typename Data >
inline
const Data& BruteForceMatcher<Data>::getData(int index) const
{
  return data_[index];
}

template < typename Data >
inline
int BruteForceMatcher<Data>::findMatch(const float* query_sig,
                                       float *distance) const
{
  int match = -1;
  float best_distance = threshold_;
  int index = 0;

//printf("sted = "); for (int i=0; i<10; i++) printf(" %.2e ", stored_sig[i]); printf("\n");
  BOOST_FOREACH( const float* stored_sig, signatures_ ) {
//printf("call %i: best = %.6e, next = %.6e, size = %i\n", index, best_distance, next_distance, size_);    
// printf("arg  = "); for (int i=0; i<10; i++) printf(" %.2e ", query_sig[i]); printf("\n");
    float next_distance = squaredDistance(size_, query_sig, stored_sig);
    if (next_distance < best_distance) {
      best_distance = next_distance;
      match = index;
    }
    ++index;
  }

  *distance = best_distance;
  return match;
}

template < typename Data >
inline
int BruteForceMatcher<Data>::findMatchInWindow(const float* signature,
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

    float next_distance = squaredDistance(size_, signature, signatures_[i]);
    if (next_distance < best_distance) {
      best_distance = next_distance;
      match = i;
    }
  }

  *distance = best_distance;
  return match;
}

template < typename Data >
inline
int BruteForceMatcher<Data>::findMatchPredicated(const float* signature,
                                                 char *predicates,
                                                 float *distance) const
{
  int match = -1;
  float best_distance = threshold_;

  for (int i = 0; i < (int)signatures_.size(); ++i) {
    if (predicates[i]) {
      float next_distance = squaredDistance(size_, signature, signatures_[i]);
      if (next_distance < best_distance) {
        best_distance = next_distance;
        match = i;
      }
    }
  }

  *distance = best_distance;
  return match;
}
template < typename Data >
inline
int BruteForceMatcher<Data>::findMatches(const float* query_sig,
                                         float *d1, int *second,
                                         float *d2) const
{
  int best_match = -1;
  float best_distance = threshold_;
  int second_match = -1;
  float second_distance = threshold_;
  int index = 0;

  BOOST_FOREACH( const float* stored_sig, signatures_ ) {
    float next_distance = squaredDistance(size_, query_sig, stored_sig);
    if (next_distance < best_distance) {
      second_distance = best_distance;
      second_match = best_match;
      best_distance = next_distance;
      best_match = index;
    } else if (next_distance < second_distance) {
      second_distance = next_distance;
      second_match = index;
    }
    ++index;
  }

  *d1 = best_distance;
  *second = second_match;
  *d2 = second_distance;

  return best_match;
}

} // namespace features

#endif
