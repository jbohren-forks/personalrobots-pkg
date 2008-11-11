#ifndef FEATURES_RNG_H
#define FEATURES_RNG_H

#ifdef HAVE_GSL
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#else
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random.hpp>
#endif

namespace features {

/*!
  A pseudo-random number generator usable with std::random_shuffle.
  If HAVE_GSL is defined, uses GSL as a backend, otherwise Boost.Random.
 */
class Rng
{
public:
#ifdef HAVE_GSL
  typedef unsigned long int_type;
#else
  typedef uint32_t int_type;
#endif
  
  Rng(int_type seed = 0);

  ~Rng();

  void seed(int_type seed);

  //! Returns a random integer sampled uniformly over the range of int_type.
  // TODO: does this hold for Boost.Random?
  int_type operator()();

  //! Returns a random integer sampled uniformly from [0, N).
  int_type operator()(int_type N);

  double uniform(double a = 0, double b = 1);
  
  //! Returns Gaussian random variate with mean zero.
  double gaussian(double sigma);

private:
#ifdef HAVE_GSL
  gsl_rng* state_;
#else
  typedef boost::mt19937 engine_type;
  engine_type engine_;
#endif
};

#ifdef HAVE_GSL
/* Implementation using GSL backend */

inline Rng::Rng(int_type seed)
{
  state_ = gsl_rng_alloc(gsl_rng_taus);
  Rng::seed(seed);
}

inline Rng::~Rng()
{
  gsl_rng_free(state_);
}

inline void Rng::seed(int_type seed)
{
  gsl_rng_set(state_, seed);
}

inline Rng::int_type Rng::operator()()
{
  return gsl_rng_get(state_);
}

inline Rng::int_type Rng::operator()(int_type N)
{
  return gsl_rng_uniform_int(state_, N);
}

inline double Rng::uniform(double a, double b)
{
  return gsl_ran_flat(state_, a, b);
}

inline double Rng::gaussian(double sigma)
{
  return gsl_ran_gaussian_ziggurat(state_, sigma);
}

#else
/* Implementation using Boost.Random backend */

inline Rng::Rng(int_type seed)
  : engine_(seed)
{}

inline Rng::~Rng() {}

inline void Rng::seed(int_type seed)
{
  engine_.seed(seed);
}

inline Rng::int_type Rng::operator()()
{
  return engine_();
}

inline Rng::int_type Rng::operator()(int_type N)
{
  return boost::uniform_int<int_type>(0, N-1)(engine_);
}

inline double Rng::uniform(double a, double b)
{
  return boost::uniform_real<double>(a, b)(engine_);
}

inline double Rng::gaussian(double sigma)
{
  // this did not work (got NANs): 
  // return boost::normal_distribution<double>(0, sigma)(engine_);
  boost::normal_distribution<double> norm_dist(0., sigma);
  boost::variate_generator<engine_type&, boost::normal_distribution<double> >  normal_sampler(engine_, norm_dist);
  return normal_sampler();  
}

#endif
  
} // namespace features

#endif
