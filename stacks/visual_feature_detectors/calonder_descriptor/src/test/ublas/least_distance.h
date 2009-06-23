#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_sparse.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/foreach.hpp>

namespace ublas = boost::numeric::ublas;

template < typename V1, typename V2, typename TV >
struct LeastSquaredDistanceFunctor
{
  typedef TV value_type;
  typedef bool result_type;
  
  // General case (access by index)
  template<class E1, class E2>
  static BOOST_UBLAS_INLINE
  result_type apply (const ublas::vector_expression<E1> &e1,
                     const ublas::vector_expression<E2> &e2,
                     value_type &best_distance) {
    typedef typename E1::size_type vector_size_type;
    assert(e1().size() == e2().size());
    
    vector_size_type size ( e1().size() );
    value_type distance = value_type (0);
    for (vector_size_type i = 0; i < size; ++i) {
      value_type diff = e1()(i) - e2()(i);
      distance += diff*diff;
      if (distance >= best_distance)
        return false;
    }

    best_distance = distance;
    return true;
  }

  // Dense case - apparently never used by uBLAS?
  template<class D, class I1, class I2>
  static BOOST_UBLAS_INLINE
  result_type apply (D size, I1 it1, I2 it2, value_type &best_distance) {
    value_type distance = value_type (0);
    while (--size >= 0) {
      value_type diff = *it1 - *it2;
      distance += diff*diff;
      if (distance >= best_distance)
        return false;
      ++it1; ++it2;
    }

    best_distance = distance;
    return true;
  }

  // Sparse case
  template<class I1, class I2>
  static BOOST_UBLAS_INLINE
  result_type apply (I1 it1, const I1 &it1_end, I2 it2, const I2 &it2_end,
                     value_type &best_distance,
                     ublas::sparse_bidirectional_iterator_tag) {
    value_type distance = value_type (0), diff;

    while (true) {
      if (it1 == it1_end) {
        while( it2 != it2_end) {
          diff = *it2;
          distance += diff*diff;
          if (distance >= best_distance)
            return false;
          ++it2;
        }
        break;
      }
      if (it2 == it2_end) {
        while( it1 != it1_end) {
          diff = *it1;
          distance += diff*diff;
          if (distance >= best_distance)
            return false;
          ++it1;
        }
        break;
      }
      
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
      distance += diff*diff;
      if (distance >= best_distance)
        return false;
    }
    
    best_distance = distance;;
    return true;
  }
};

// How much of this boilerplate is really necessary?
template < typename E1, typename E2, typename F >
class LeastDistance
  : public ublas::scalar_expression< LeastDistance<E1, E2, F> >
{
  typedef E1 expression1_type;
  typedef E2 expression2_type;
  typedef F functor_type;
  typedef typename E1::const_closure_type expression1_closure_type;
  typedef typename E2::const_closure_type expression2_closure_type;
  typedef typename ublas::iterator_restrict_traits<typename E1::const_iterator::iterator_category,
                                                   typename E2::const_iterator::iterator_category>::iterator_category iterator_category;
  typedef LeastDistance<E1, E2, F> self_type;

public:
  typedef typename F::result_type value_type;
  typedef typename F::value_type distance_type;
  typedef const self_type const_closure_type;
  typedef const_closure_type closure_type;
  typedef ublas::unknown_storage_tag storage_category;

  BOOST_UBLAS_INLINE
  LeastDistance( const expression1_type &e1, const expression2_type &e2,
                 distance_type &best_distance)
    : e1_(e1), e2_(e2), d_(best_distance)
  {}

  // Conversion operator implicitly evaluates the expression
  operator value_type () const {
    return evaluate( iterator_category() );
  }

private:
  // Dense random access specialization
  BOOST_UBLAS_INLINE
  value_type evaluate (ublas::dense_random_access_iterator_tag) const {
    // Indexing version
    return functor_type::apply(e1_, e2_, d_);
    // Iterating version
    //return functor_type::apply(e1_.size(), e1_.begin(), e2_.begin(), d_);
  }

  // Skip packed specialization
  
  // Sparse bidirectional specialization
  BOOST_UBLAS_INLINE
  value_type evaluate (ublas::sparse_bidirectional_iterator_tag) const {
    return functor_type::apply( e1_.begin(), e1_.end(), e2_.begin(), e2_.end(), d_,
                                ublas::sparse_bidirectional_iterator_tag() );
  }

  expression1_closure_type e1_;
  expression2_closure_type e2_;
  distance_type &d_;
};

// Meta-program to get type traits for squared distance expressions
template < typename E1, typename E2 >
struct LeastDistanceTraits
{
  typedef typename ublas::promote_traits<typename E1::value_type,
                                         typename E2::value_type>::promote_type value_type;
  typedef LeastSquaredDistanceFunctor<E1, E2, value_type> functor_type;
  typedef LeastDistance<E1, E2, functor_type> expression_type;
  typedef typename functor_type::result_type result_type;
};

template < typename E1, typename E2 >
BOOST_UBLAS_INLINE
bool
leastSquaredDistance(ublas::vector_expression<E1> const& e1,
                     ublas::vector_expression<E2> const& e2,
                     typename LeastDistanceTraits<E1, E2>::value_type &best_distance)
{
  typedef typename LeastDistanceTraits<E1, E2>::expression_type expression_type;

  return expression_type (e1 (), e2 (), best_distance);
}
