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
** FILENAME:    svlLogistic.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  The svlLogistic class implements a logistic regression classifier,
**      P(Y = 0 | X = x) = 1 / (1 + exp(-w^T x))
**  The classifier is trained using Newton's method. Currently the class
**  uses the OpenCV CvMat data structures, but can also be called with stl
**  vectors.
**
**  The svlMultiClassLogistic class supercedes the svlLogistic class
**  and implements a multiclass logistic regression classifier,
**      P(Y = 0 | X = x) = 1 / (1 + sum exp(w_i^T x))
**      P(Y = k | X = x) = exp(w_k^T x) / (1 + sum exp(w_i^T x))
**  The classifier is trained using the svlOptimizer class and does not
**  rely on OpenCV data structures.
**
** TO DO (svlLogistic):
**  2. implement line search in Netown's method
**  3. implement score function (same as evaluate but without the 1/1+exp)
**  4. implement cholesky and remove dependency on openCV
**
*****************************************************************************/

#pragma once

#include <vector>
#include <limits>

#include <opencv/cxcore.h>

#include "svlBase.h"

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#undef max
#undef min
#endif

using namespace std;

// svlLogistic class ---------------------------------------------------------

class svlLogistic {
private:
    CvMat *_theta;            // parameters
    CvMat *_gradient;         // gradient (during training)
    CvMat *_hessian;          // hessian (during training)
    float _posClassWeight;    // relative weight of each positive sample
    float _lambda;            // regularization parameter (x 2)
    CvMat *_featureSum;       // feature mean for computing feature variance
    CvMat *_featureSum2;      // feature variance for scaling regularization
    int _numSamples;          // number of samples accumulated
    float _stepSize;          // truncated newton step size

public:
    svlLogistic();
    svlLogistic(int n, float w = 1.0, float r = 1.0e-6);
    svlLogistic(const CvMat *t);
    svlLogistic(const vector<double>& t);
    svlLogistic(const svlLogistic& model);
    ~svlLogistic();

    // initialize the parameters
    void initialize(int n, float w = 1.0, float r = 1.0e-6);
    void initialize(const CvMat *t);
    void initialize(const vector<double>& t);
    void zero();
    void randomize();

    // i/o
    bool save(const char *filename) const;
    bool load(const char *filename);

    // evaluate model (caller is responsible for providing correct size matrix)
    // each row in x is a feature vector
    void evaluate(const CvMat *x, CvMat * y) const;
    void evaluate(const vector<vector<double> >& x, vector<double>& y) const;
    double evaluateSingle(const vector<double>& x) const;
    double mse(const CvMat *y, const CvMat *p) const;
    double mse(const vector<double>& y, const vector<double>& p) const;

    // accumulate evidence with data x, target y and prediction p
    void accumulate(const CvMat *x, const CvMat *y, const CvMat *p);
    void accumulate(const vector<vector<double> >& x, const vector<double>& y,
		    const vector<double> &p);

    // perform a single training update with regularization using accumulated data
    void update();

    // perform full training until mse no longer decreases
    double train(const CvMat *x, const CvMat *y, double eps = 1.0e-6,
	unsigned maxIterations = numeric_limits<unsigned>::max());
    double train(const vector<vector<double> >& x, const vector<double>& y,
	double eps = 1.0e-6, unsigned maxIterations = numeric_limits<unsigned>::max());
    double train(const char *filename, double eps = 1.0e-6,
	unsigned maxIterations = numeric_limits<unsigned>::max());

    // access functions
    CvMat *parameters() const { return _theta; }
    int size() const { return _theta->rows; }

    // standard operators
    double operator[](unsigned index) const { return cvmGet(_theta, index, 0); }
    svlLogistic& operator=(const svlLogistic& model);
};

// svlMultiClassLogistic class -----------------------------------------------

class svlMultiClassLogistic {
private:
    int _nFeatures;                    // number of features
    int _nClasses;                     // number of classes
    vector<vector<double> > _theta;    // parameters (_nClasses - 1) * _nFeatures
    vector<double> _weights;           // relative weight for each class
    double _lambda;                    // regularization parameter

public:
    svlMultiClassLogistic(unsigned n = 1, unsigned k = 2, double r = 1.0e-9);
    svlMultiClassLogistic(const svlMultiClassLogistic& model);
    ~svlMultiClassLogistic();

    // initialize the parameters
    void initialize(unsigned n, unsigned k = 2, double r = 1.0e-9);
    void zero();

    // i/o
    bool save(const char *filename) const;
    bool load(const char *filename);

    // evaluate model (each row in x is a feature vector)
    void evaluate(const vector<double>& x, int& y) const;
    void evaluate(const vector<vector<double> >& x, vector<int>& y) const;
    void evaluateMarginal(const vector<double>& x, vector<double>& y) const;
    void evaluateMarginal(const vector<vector<double> >& x,
	vector<vector<double> >& y) const;

    // perform full training until log-likelihood no longer decreases
    double train(const vector<vector<double> >& x, const vector<int>& y,
	double eps = 1.0e-6, int maxIterations = numeric_limits<int>::max());
    double train(const vector<vector<double> >& x, const vector<int>& y,
        const vector<double>& w, double eps = 1.0e-6,
        int maxIterations = numeric_limits<int>::max());
    double train(const char *filename, double eps = 1.0e-6,
        int maxIterations = numeric_limits<int>::max());

    // access functions
    int numFeatures() const { return _nFeatures; }
    int numClasses() const { return _nClasses; }
    vector<vector<double> > getTheta() const { return _theta; }
    void setTheta(const vector<vector<double> > &param);

    // standard operators
    svlMultiClassLogistic& operator=(const svlMultiClassLogistic& model);
};

// svlMultiClassLogisticOptimizer ------------------------------------------
// class to do all the grunt work for optimizing the multi-class logistic
class svlMultiClassLogisticOptimizer : public svlOptimizer
{
 public:
    int nClasses;
    int nFeatures;
    const vector<vector<double> > *pTrainingData;
    const vector<int> *pTrainingLabels;
    const vector<double> *pTrainingWeights;
    double lambda;

    svlMultiClassLogisticOptimizer() : svlOptimizer(), 
	pTrainingData(NULL), pTrainingLabels(NULL), lambda(0.0) { }
    svlMultiClassLogisticOptimizer(unsigned n) : svlOptimizer(n), 
	pTrainingData(NULL), pTrainingLabels(NULL), lambda(0.0) { }

    double objective(const double *x);
    void gradient(const double *x, double *df);
    double objectiveAndGradient(const double *x, double *df);
};

