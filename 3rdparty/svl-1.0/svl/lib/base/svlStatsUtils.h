/*****************************************************************************
** STAIR VISION LIBRARY
** Copyright (c) 2007-2008, Stephen Gould
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the Stanford University nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
******************************************************************************
** FILENAME:    svlStatUtils.h
** AUTHOR(S):   Ian Goodfellow <ia3n@stanford.edu>
**              Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Generic statistical utilities.
**
*****************************************************************************/

#pragma once

#include <cassert>
#include <vector>
#include <algorithm>

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#undef min
#undef max
#endif

using namespace std;

template <typename T>
T minElem(const vector<T>& v);

template <typename T>
T maxElem(const vector<T>& v);

template <typename T>
T mean(const vector<T>& v);

template <typename T>
T median(const vector<T>& v);

template <typename T>
T variance(const vector<T>& v);

template <typename T>
T stdev(const vector<T>& v);

template <typename T>
int argmin(const vector<T>& v);

template <typename T>
vector<int> argmins(const vector<vector<T> >& v);

template <typename T>
int argmax(const vector<T>& v);

template <typename T>
vector<int> argmaxs(const vector<vector<T> >& v);

template <typename T>
T excessKurtosis(const vector<T> & v);

template <typename T>
vector<float> percentiles(const vector<T> & v);

template <typename T>
pair<T, T> range(const vector<T>& v);

// select an ordered subvector from a vector
template <typename T>
vector<T> extractSubVector(const vector<T>& v, const vector<int> indx);

// removes (v.size() - keepSize)/2 minimum and maximum entries
template <typename T>
vector<T> removeOutliers(const vector<T>& v,
    const vector<double>& scores, int keepSize);

// logistic function
double logistic(const vector<double>& theta, const vector<double>& data); 
double logistic(const double *theta, const double *data, int n);

// computes the entropy of a possibly unnormalized distribution
double entropy(const std::vector<double>& p);

// exponentiates and normalizes a vector in-place
void expAndNormalize(std::vector<double>& v);

// compute a random permutation of the numbers [0..n-1]
std::vector<int> randomPermutation(int n);

// computes successor and predecessor discrete vectors, for example
// successor([1 0 0], 2) produces [0 1 0]
void predecessor(std::vector<int>& array, int limit);
void successor(std::vector<int>& array, int limit);
void predecessor(std::vector<int>& array, const std::vector<int>& limits);
void successor(std::vector<int>& array, const std::vector<int>& limits);

// huber penalty function
inline double huberFunction(double x, double m = 1.0);
inline double huberDerivative(double x, double m = 1.0);
inline double huberFunctionAndDerivative(double x, double *df, double m = 1.0);

// Distance metrics. P and Q are probability distributions but do not need 
// to be normalized unless otherwise stated.
double bhattacharyyaDistance(std::vector<double>& p, std::vector<double>& q);

// Implementation -----------------------------------------------------------

template <typename T>
T minElem(const vector<T> & v)
{
  assert (v.size() > 0);

  T minObj(v.front());

  for (typename vector<T>::const_iterator i = v.begin() + 1, last = v.end(); i != last; ++i) {
      const T & curObj = *i;

      if (curObj < minObj)
          minObj = curObj;
  }   

  return minObj;
}

template <typename T>
T maxElem(const vector<T> & v)
{
  assert (v.size() > 0);

  T maxObj(v.front());

  for (typename vector<T>::const_iterator i = v.begin() + 1, last = v.end(); i != last; ++i) {
      const T & curObj = *i;
      
      if (curObj > maxObj)
          maxObj = curObj;
  }   

  return maxObj;
}

template <typename T>
T mean(const vector<T>& v)
{
  assert (v.size() > 0);

  T sum(0);

  for (typename vector<T>::const_iterator i = v.begin(), last = v.end();  i != last; ++i) {
      sum += *i;
  }   

  return sum / T(v.size());
}

template <typename T>
T median(const vector<T>& v)
{
    assert (v.size() > 0);
    
    vector<T> w(v);
    sort(w.begin(), w.end());
    
    if (w.size() % 2 == 0) {
        return w[w.size() / 2];
    } else {
        return 2.0 * (w[(w.size() - 1) / 2] + w[(w.size() + 1) / 2]);
    }
}

template <typename T>
T variance(const vector<T> & v)
{
  assert (v.size() > 0);

  T mu = mean(v);

  //assert (! (isnan(mu) || isinf(mu)));
  //printf("variance mu: %f\n",mu);

  T sum(0);

  for (typename vector<T>::const_iterator i = v.begin(), last = v.end(); i != last; ++i) {
      double dev =  *i - mu;
      sum += dev * dev;
  }   

  return sum / T(v.size());
}

template <typename T>
T stdev(const vector<T> &v)
{
    T std2 = variance(v);
    return (std2 > 0.0 ? sqrt(std2) : 0.0);
}

template <typename T>
int argmin(const vector<T> & v)
{
    if (v.empty()) {
        return -1;
    }

    int minIndx = 0;
    for (int i = 1; i < (int)v.size(); i++) {
        if (v[i] < v[minIndx]) {
            minIndx = i;
        }          
    }   

    return minIndx;
}

template <typename T>
vector<int> argmins(const vector<vector<T> >& v)
{
    vector<int> minIndx(v.size(), -1);
    for (int i = 0; i < (int)v.size(); i++) {
        minIndx[i] = argmin(v[i]);
    }

    return minIndx;
}

template <typename T>
int argmax(const vector<T> & v)
{
    if (v.empty()) {
        return -1;
    }

    int maxIndx = 0;
    for (int i = 1; i < (int)v.size(); i++) {
        if (v[i] > v[maxIndx]) {
            maxIndx = i;
        }          
    }   

    return maxIndx;
}

template <typename T>
vector<int> argmaxs(const vector<vector<T> >& v)
{
    vector<int> maxIndx(v.size(), -1);
    for (int i = 0; i < (int)v.size(); i++) {
        maxIndx[i] = argmax(v[i]);
    }

    return maxIndx;
}

template <typename T>
T excessKurtosis(const vector<T> & v)
{
  assert (v.size() > 0);

  T mu = mean(v);
  T sigma_squared = variance(v);

  //assert (! (isnan(mu) || isinf(mu)));
  //printf("variance mu: %f\n",mu);

  T sum(0);

  for (typename vector<T>::const_iterator i = v.begin(), last = v.end(); i != last; ++i)
    {
      float dev = *i - mu;
      float sqDev = dev * dev;
      sum += sqDev * sqDev;
    }   

  return sum / ( T(v.size() * sigma_squared * sigma_squared)) -3.0;
}

template <typename T>
vector<float> percentiles(const vector<T> &v)
{
  vector<float> rval;
  for (int i = 0; i < v.size(); i++)
    {
      int sum = 0;
      for (int j = 0; j < v.size(); j++)
	{
	  if (v[j] < v[i])
	    sum++;
	}
      rval.push_back(float(sum)/float(v.size()));
    }
  return rval;
}

template <typename T>
pair<T, T> range(const vector<T>& v)
{
    assert (v.size() > 0);

    typename vector<T>::const_iterator minObj(v.begin());
    typename vector<T>::const_iterator maxObj(v.begin());
    for (typename vector<T>::const_iterator i = v.begin() + 1;
         i != v.end(); ++i) {
        if (*i < *minObj) minObj = i;
        if (*i > *maxObj) maxObj = i;
    }   

    return make_pair(*minObj, *maxObj);    
}

template <typename T>
vector<T> extractSubVector(const vector<T>& v, const vector<int> indx)
{
    vector<T> w;

    w.reserve(indx.size());    
    for (vector<int>::const_iterator it = indx.begin(); it != indx.end(); ++it) {
        w.push_back(v[*it]);
    }

    return w;
}

template <typename T>
vector<T> removeOutliers(const vector<T>& v, 
    const vector<double>& scores, int keepSize)
{
    assert(scores.size() == v.size());
    if (keepSize >= (int)v.size()) {
        return v;
    }

    // sort scores
    vector<pair<double, int> > indx(v.size());
    for (unsigned i = 0; i < v.size(); i++) {
        indx[i] = make_pair(scores[i], i);
    }
    sort(indx.begin(), indx.end());
    
    vector<T> w(keepSize);
    unsigned startIndx = (v.size() - keepSize) / 2;
    unsigned endIndx = startIndx + keepSize;
    for (unsigned i = startIndx; i < endIndx; i++) {
        w[i - startIndx] = v[indx[i].second];
    }

    return w;
}

inline double huberFunction(double x, double m)
{
    if (x < -m) return (m * (-2.0 * x - m));
    if (x > m) return (m * (2.0 * x - m));
    
    return x * x;
}

inline double huberDerivative(double x, double m)
{
    if (x < -m) return -2.0 * m;
    if (x > m) return 2.0 * m;

    return 2.0 * x;
}

inline double huberFunctionAndDerivative(double x, double *df, double m)
{
    if (x < -m) {
	*df = -2.0 * m;
	return (m * (-2.0 * x - m));
    } else if (x > m) {
	*df = 2.0 * m;
	return (m * (2.0 * x - m));
    } else {
	*df = 2.0 * x;
	return x * x;
    }
}

