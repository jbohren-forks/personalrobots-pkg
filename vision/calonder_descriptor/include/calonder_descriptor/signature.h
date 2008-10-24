#ifndef FEATURES_SIGNATURE_STATISTICS_H
#define FEATURES_SIGNATURE_STATISTICS_H

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_sparse.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/foreach.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <cassert>

namespace ublas = boost::numeric::ublas;

namespace features {

// Convenience typedefs for signature storage types
typedef ublas::vector<float> DenseSignature;
typedef ublas::compressed_vector<float> SparseSignature;

// Efficient squared distance functor that integrates with uBLAS' dispatch machinery
template<class V1, class V2, class TV>
struct SquaredDistanceFunctor: public ublas::vector_scalar_binary_functor<V1, V2, TV>
{
  typedef typename ublas::vector_scalar_binary_functor<V1, V2, TV>::value_type value_type;
  typedef typename ublas::vector_scalar_binary_functor<V1, V2, TV>::result_type result_type;
  
  // General case (access by index)
  template<class E1, class E2>
  static BOOST_UBLAS_INLINE
  result_type apply (const ublas::vector_expression<E1> &e1,
                     const ublas::vector_expression<E2> &e2) {
    typedef typename E1::size_type vector_size_type;
    assert(e1().size() == e2().size());
    vector_size_type size ( e1().size() );
    result_type result = result_type (0);
    for (vector_size_type i = 0; i < size; ++i) {
      result_type diff = e1()(i) - e2()(i);
      result += diff*diff;
    }
    return result;
  }

  // Dense case - apparently never used by uBLAS?
  template<class D, class I1, class I2>
  static BOOST_UBLAS_INLINE
  result_type apply (D size, I1 it1, I2 it2) {
    result_type result = result_type (0);
    while (--size >= 0) {
      result_type diff = *it1 - *it2;
      ++it1; ++it2;
      result += diff*diff;
    }
    return result;
  }

  // Sparse case
  template<class I1, class I2>
  static BOOST_UBLAS_INLINE
  result_type apply (I1 it1, const I1 &it1_end, I2 it2, const I2 &it2_end,
                     ublas::sparse_bidirectional_iterator_tag) {
    result_type result = result_type (0), diff;

    // Check if either vector is empty so we don't segfault
    if (it1 == it1_end)
      return ublas::vector_inner_prod<V1, V2, TV>::apply(it2, it2_end, it2, it2_end,
                                                         ublas::sparse_bidirectional_iterator_tag());
    if (it2 == it2_end)
      return ublas::vector_inner_prod<V1, V2, TV>::apply(it1, it1_end, it1, it1_end,
                                                         ublas::sparse_bidirectional_iterator_tag());

    while (true) {
      if (it1.index() == it2.index()) {
        diff = *it1 - *it2;
        ++it1;
        ++it2;
      } else if (it1.index() < it2.index()) {
        diff = *it1;
        ++it1;
      } else {
        diff = *it2;
        ++it2;
      }
      result += diff*diff;
      
      if (it1 == it1_end) {
        while( it2 != it2_end) {
          diff = *it2;
          result += diff*diff;
          ++it2;
        }
        break;
      }
      if (it2 == it2_end) {
        while( it1 != it1_end) {
          diff = *it1;
          result += diff*diff;
          ++it1;
        }
        break;
      }
    }
    
    return result;
  }
};

// Meta-program to get type traits for squared distance expressions
template < typename E1, typename E2 >
struct SquaredDistanceTraits
  : ublas::vector_scalar_binary_traits<E1, E2,
                                       SquaredDistanceFunctor<E1, E2,
                                                              typename ublas::promote_traits<typename E1::value_type,
                                                                                             typename E2::value_type>::promote_type
                                                              >
                                       >
{};

// squaredDistance(v1, v2) = dot(v1 - v2, v1 - v2)
template < typename E1, typename E2 >
BOOST_UBLAS_INLINE
typename SquaredDistanceTraits<E1, E2>::result_type squaredDistance(ublas::vector_expression<E1> const& e1,
                                                                    ublas::vector_expression<E2> const& e2)
{
  typedef typename SquaredDistanceTraits<E1, E2>::expression_type expression_type;

  return expression_type (e1 (), e2 ());
}

// Simple Euclidean distance, not particularly optimized
template < typename Left, typename Right >
inline
float euclideanDistance(ublas::vector_expression<Left> const& left,
                        ublas::vector_expression<Right> const& right)
{
  return ublas::norm_2( left() - right() );
}

template < typename Expr >
float entropy(ublas::vector_expression<Expr> const& posterior)
{
  float H = 0;
  BOOST_FOREACH( float elem, posterior() ) {
    if (elem > 0)
      H -= elem * log2(elem);
  }
  
  return H;
}

template < typename C1, typename SigT >
int bestMatch(ublas::vector_container<C1> const& query_sig,
              std::vector< SigT > const& signatures,
              float *distance,
              float threshold = std::numeric_limits<float>::max())
{
  int match = -1;
  float best_distance = threshold;
  int index = 0;

  BOOST_FOREACH( SigT const& stored_sig, signatures ) {
    float next_distance = squaredDistance(query_sig(), stored_sig);
    if (next_distance < best_distance) {
      best_distance = next_distance;
      match = index;
    }
  }

  *distance = best_distance;
  return match;
}

template < typename C1, typename SigT >
int bestTwoMatches(ublas::vector_container<C1> const& query_sig,
                   std::vector< SigT > const& signatures,
                   float *d1, int *second, float *d2,
                   float threshold = std::numeric_limits<float>::max())
{
  int best_match = -1;
  float best_distance = threshold;
  int second_match = -1;
  float second_distance = threshold;
  int index = 0;

  BOOST_FOREACH( SigT const& stored_sig, signatures ) {
    float next_distance = squaredDistance(query_sig(), stored_sig);
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
