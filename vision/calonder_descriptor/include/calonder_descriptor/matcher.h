#ifndef FEATURES_MATCHER_H
#define FEATURES_MATCHER_H

#include "calonder_descriptor/basic_math.h"
#include <cv.h>
#include <limits>
#include <vector>
#include <utility>
#include <boost/foreach.hpp>

namespace features {

template < typename SigElem, typename Data >
class BruteForceMatcher
{
public:
  typedef typename Promote<SigElem>::type distance_type;
  
  BruteForceMatcher(size_t signature_dimension);
  
  // BruteForceMatcher does NOT take ownership of signature's memory
  void addSignature(SigElem* signature, Data const& data);

  size_t numSignatures();
  
  SigElem* getSignature(int index);
  const SigElem* getSignature(int index) const;
  Data& getData(int index);
  const Data& getData(int index) const;

  int findMatch(const SigElem* signature, distance_type *distance) const;

  int findMatchInWindow(const SigElem* signature, CvRect window,
                        distance_type *distance) const;
  int findMatchPredicated(const SigElem* signature, char *predicates,
                          distance_type *distance) const;

  // Returns top two matches, useful for ratio test
  int findMatches(const SigElem* signature, distance_type *d1, int *second,
                  distance_type *d2) const;

  // TODO: for testing/debugging only, remove
  const std::vector<SigElem*>& signatures() { return signatures_; }

private:
  std::vector< SigElem* > signatures_;
  std::vector< Data > data_;
  distance_type threshold_;
  size_t dimension_;
  L1DistanceFunc<SigElem> distance_func;
};

template < typename SigElem, typename Data >
inline
BruteForceMatcher<SigElem, Data>::BruteForceMatcher(size_t signature_dimension)
  : threshold_(std::numeric_limits<float>::max()),
    dimension_(signature_dimension),
    distance_func(signature_dimension)
{}

template < typename SigElem, typename Data >
inline
size_t BruteForceMatcher<SigElem, Data>::numSignatures()
{
  return signatures_.size();
}

template < typename SigElem, typename Data >
inline
void BruteForceMatcher<SigElem, Data>::addSignature(SigElem* signature,
                                                    Data const& data)
{
  signatures_.push_back(signature);
  data_.push_back(data);
}

template < typename SigElem, typename Data >
inline
SigElem* BruteForceMatcher<SigElem, Data>::getSignature(int index)
{
  return signatures_[index];
}

template < typename SigElem, typename Data >
inline
const SigElem* BruteForceMatcher<SigElem, Data>::getSignature(int index) const
{
  return signatures_[index];
}

template < typename SigElem, typename Data >
inline
Data& BruteForceMatcher<SigElem, Data>::getData(int index)
{
  return data_[index];
}

template < typename SigElem, typename Data >
inline
const Data& BruteForceMatcher<SigElem, Data>::getData(int index) const
{
  return data_[index];
}

template < typename SigElem, typename Data >
inline
int BruteForceMatcher<SigElem, Data>::findMatch(const SigElem* query_sig,
                                                distance_type *distance) const
{
  int match = -1;
  float best_distance = threshold_;
  int index = 0;

  BOOST_FOREACH( const SigElem* stored_sig, signatures_ ) {
    float next_distance = distance_func(query_sig, stored_sig);
    if (next_distance < best_distance) {
      best_distance = next_distance;
      match = index;
    }
    ++index;
  }

  *distance = best_distance;
  return match;
}

template < typename SigElem, typename Data >
inline
int BruteForceMatcher<SigElem, Data>::findMatchInWindow(const SigElem* signature,
                                                        CvRect window,
                                                        distance_type *distance) const
{
  int match = -1;
  float best_distance = threshold_;

  for (int i = 0; i < (int)signatures_.size(); ++i) {
    Data const& data = data_[i];
    if (data.x < window.x || data.y < window.y ||
        data.x >= window.x + window.width ||
        data.y >= window.y + window.height)
      continue;

    float next_distance = distance_func(signature, signatures_[i]);
    if (next_distance < best_distance) {
      best_distance = next_distance;
      match = i;
    }
  }

  *distance = best_distance;
  return match;
}

template < typename SigElem, typename Data >
inline
int BruteForceMatcher<SigElem, Data>::findMatchPredicated(const SigElem* signature,
                                                          char *predicates,
                                                          distance_type *distance) const
{
  int match = -1;
  float best_distance = threshold_;

  for (int i = 0; i < (int)signatures_.size(); ++i) {
    if (predicates[i]) {
      float next_distance = distance_func(signature, signatures_[i]);
      if (next_distance < best_distance) {
        best_distance = next_distance;
        match = i;
      }
    }
  }

  *distance = best_distance;
  return match;
}
template < typename SigElem, typename Data >
inline
int BruteForceMatcher<SigElem, Data>::findMatches(const SigElem* query_sig,
                                                  distance_type *d1, int *second,
                                                  distance_type *d2) const
{
  int best_match = -1;
  float best_distance = threshold_;
  int second_match = -1;
  float second_distance = threshold_;
  int index = 0;

  BOOST_FOREACH( const SigElem* stored_sig, signatures_ ) {
    float next_distance = distance_func(query_sig, stored_sig);
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
