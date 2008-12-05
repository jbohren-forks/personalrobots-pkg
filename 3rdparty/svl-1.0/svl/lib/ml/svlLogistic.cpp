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
** FILENAME:    svlLogistic.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  See svlLogistic.h
**
*****************************************************************************/

#include <cassert>
#include <iostream>
#include <fstream>
#include <limits>

// OpenCV libraries
#include "opencv/cv.h"
#include "opencv/cxcore.h"

// SVL libraries
#include "svlBase.h"
#include "svlLogistic.h"

using namespace std;

// svlLogistic class ---------------------------------------------------------

// default constructor
svlLogistic::svlLogistic() :
    _gradient(NULL), _hessian(NULL), _posClassWeight(1.0f), _lambda(0.0f),
    _featureSum(NULL), _featureSum2(NULL), _numSamples(0), _stepSize(0.9f)
{
    _theta = cvCreateMat(1, 1, CV_32FC1);
    cvZero(_theta);
}

svlLogistic::svlLogistic(int n, float w, float r) :
    _gradient(NULL), _hessian(NULL), _posClassWeight(w), _lambda(r),
    _featureSum(NULL), _featureSum2(NULL), _numSamples(0), _stepSize(0.9f)
{
    assert(n > 0);
    _theta = cvCreateMat(n, 1, CV_32FC1);
    cvZero(_theta);
}

svlLogistic::svlLogistic(const CvMat *t) :
    _gradient(NULL), _hessian(NULL), _posClassWeight(1.0f), _lambda(1.0e-6f),
    _featureSum(NULL), _featureSum2(NULL), _numSamples(0), _stepSize(0.9f)
{
    assert((t != NULL) && (t->cols == 1));
    _theta = cvCloneMat(t);
}

svlLogistic::svlLogistic(const vector<double>& t) :
    _gradient(NULL), _hessian(NULL), _posClassWeight(1.0f), _lambda(1.0e-6f),
    _featureSum(NULL), _featureSum2(NULL), _numSamples(0), _stepSize(0.9f)
{
    _theta = cvCreateMat((unsigned)t.size(), 1, CV_32FC1);
    for (unsigned i = 0; i < (unsigned)t.size(); i++) {
	cvmSet(_theta, i, 0, t[i]);
    }
}

// copy constructor
svlLogistic::svlLogistic(const svlLogistic& model)
{
    _theta = cvCloneMat(model._theta);
    if (model._gradient != NULL) {
        _gradient = cvCloneMat(model._gradient);
        _hessian = cvCloneMat(model._hessian);
    } else {
        _gradient = NULL;
        _hessian = NULL;
    }
    _posClassWeight = model._posClassWeight;
    _lambda = model._lambda;
    if (model._featureSum != NULL) {
	_featureSum = cvCloneMat(model._featureSum);
	_featureSum2 = cvCloneMat(model._featureSum2);
    } else {
	_featureSum = NULL;
	_featureSum2 = NULL;
    }
   _numSamples = model._numSamples;
    _stepSize = model._stepSize;
}

// destructor
svlLogistic::~svlLogistic()
{
    cvReleaseMat(&_theta);
    if (_gradient != NULL) {
        cvReleaseMat(&_hessian);
        cvReleaseMat(&_gradient);
    }
    if (_featureSum != NULL) {
	cvReleaseMat(&_featureSum);
	cvReleaseMat(&_featureSum2);
    }
}

// initialize the parameters
void svlLogistic::initialize(int n, float w, float r)
{
    assert(n > 0);
    cvReleaseMat(&_theta);
    _theta = cvCreateMat(n, 1, CV_32FC1);
    cvZero(_theta);
    if (_gradient != NULL) {
        cvReleaseMat(&_hessian);
        cvReleaseMat(&_gradient);
    }
    _gradient = NULL;
    _hessian = NULL;
    _posClassWeight = w;
    _lambda = r;
    if (_featureSum != NULL) {
	cvReleaseMat(&_featureSum);
	cvReleaseMat(&_featureSum2);
    }
    _featureSum = NULL;
    _featureSum2 = NULL;
    _numSamples = 0;
    _stepSize = 0.9f;
}

void svlLogistic::initialize(const CvMat *t)
{
    assert((t != NULL) && (t->cols == 1));
    cvReleaseMat(&_theta);
    _theta = cvCloneMat(t);
    if (_gradient != NULL) {
        cvReleaseMat(&_hessian);
        cvReleaseMat(&_gradient);
    }
    _gradient = NULL;
    _hessian = NULL;
    if (_featureSum != NULL) {
	cvReleaseMat(&_featureSum);
	cvReleaseMat(&_featureSum2);
    }
    _featureSum = NULL;
    _featureSum2 = NULL;
    _numSamples = 0;
    _stepSize = 0.9f;
}

void svlLogistic::initialize(const vector<double>& t)
{
    cvReleaseMat(&_theta);
    _theta = cvCreateMat((unsigned)t.size(), 1, CV_32FC1);
    for (unsigned i = 0; i < (unsigned)t.size(); i++) {
	cvmSet(_theta, i, 0, t[i]);
    }
    if (_gradient != NULL) {
        cvReleaseMat(&_hessian);
        cvReleaseMat(&_gradient);
    }
    _gradient = NULL;
    _hessian = NULL;
    if (_featureSum != NULL) {
	cvReleaseMat(&_featureSum);
	cvReleaseMat(&_featureSum2);
    }
    _featureSum = NULL;
    _featureSum2 = NULL;
    _numSamples = 0;
    _stepSize = 0.9f;
}

void svlLogistic::zero()
{
    cvZero(_theta);
}

void svlLogistic::randomize()
{
    CvRNG rng = cvRNG();
    cvRandArr(&rng, _theta, CV_RAND_UNI, cvScalar(-1.0), cvScalar(1.0));
    if (_gradient != NULL) {
        cvReleaseMat(&_hessian);
        cvReleaseMat(&_gradient);
    }
    _gradient = NULL;
    _hessian = NULL;
    if (_featureSum != NULL) {
	cvReleaseMat(&_featureSum);
	cvReleaseMat(&_featureSum2);
    }
    _featureSum = NULL;
    _featureSum2 = NULL;
}

bool svlLogistic::save(const char *filename) const
{
    ofstream ofs(filename);
    if (ofs.fail()) {
	return false;
    }

    ofs << _theta->rows << endl;
    for (int i = 0; i < _theta->rows; i++) {
	ofs << cvmGet(_theta, i, 0) << endl;
    }

    ofs.close();
    return true;
}

bool svlLogistic::load(const char *filename)
{
    ifstream ifs(filename);
    if (ifs.fail()) {
	return false;
    }

    unsigned n;
    ifs >> n;

    vector<double> t(n);
    for (unsigned i = 0; i < n; i++) {
	ifs >> t[i];
    }
    ifs.close();

    initialize(t);
    return true;
}

// evaluate model (caller is responsible for providing correct size matrix)
void svlLogistic::evaluate(const CvMat *x, CvMat * y) const
{
    assert((x != NULL) && (y != NULL));
    assert((x->cols == _theta->rows) && (x->rows == y->rows) && (y->cols == 1));

    //cvZero(y);
    for (int i = 0; i < y->rows; i++) {
        double v = 0.0;
        for (int j = 0; j < x->cols; j++) {
            v += cvmGet(x, i, j) * cvmGet(_theta, j, 0);
        }
        cvmSet(y, i, 0, 1.0 / (1.0 + exp(-1.0 * v)));
    }
}

void svlLogistic::evaluate(const vector<vector<double> >& x, vector<double>& y) const
{
    y.resize(x.size());
    for (int i = 0; i < (int)y.size(); i++) {
	assert(x[i].size() == (unsigned)_theta->rows);
        double v = 0.0;
        for (int j = 0; j < _theta->rows; j++) {
            v += x[i][j] * cvmGet(_theta, j, 0);
        }
        y[i] = 1.0 / (1.0 + exp(-1.0 * v));
    }
}

double svlLogistic::evaluateSingle(const vector<double>& x) const
{
    assert(x.size() == (unsigned)_theta->rows);

    double v = 0.0;
    for (int j = 0; j < _theta->rows; j++) {
	v += x[j] * cvmGet(_theta, j, 0);
    }

    return (1.0 / (1.0 + exp(-1.0 * v)));
}

// compute mean-square-error between two vectors
double svlLogistic::mse(const CvMat *y, const CvMat *p) const
{
    assert((y != NULL) && (p != NULL));
    assert((y->rows == p->rows) && (y->cols == p->cols));

    CvMat *m = cvCreateMat(y->rows, y->cols, CV_32FC1);
    cvSub(y, p, m);
    cvMul(m, m, m);
    CvScalar err = cvAvg(m);
    cvReleaseMat(&m);

    return err.val[0];
}

double svlLogistic::mse(const vector<double>& y, const vector<double>& p) const
{
    assert(y.size() == p.size());
    if (y.empty()) {
	return 0.0;
    }

    double m = 0.0;
    for (unsigned i = 0; i < y.size(); i++) {
	m += (y[i] - p[i]) * (y[i] - p[i]);
    }

    return m / (double)y.size();
}

// accumulate evidence with data x, target y and prediction p
void svlLogistic::accumulate(const CvMat *x, const CvMat *y, const CvMat *p)
{
    assert((x != NULL) && (y != NULL) && (p != NULL));
    assert((x->cols == _theta->rows) && (x->rows == y->rows) && (y->cols == 1));
    assert((p->cols == y->cols) && (p->rows == y->rows));

    int n = _theta->rows;
    if (_gradient == NULL) {
        _gradient = cvCreateMat(n, 1, CV_32FC1);
        _hessian = cvCreateMat(n, n, CV_32FC1);
        cvZero(_gradient);
        cvZero(_hessian);
    }

    if (_featureSum == NULL) {
        _featureSum = cvCreateMat(n, 1, CV_32FC1);
        _featureSum2 = cvCreateMat(n, n, CV_32FC1);
        cvZero(_featureSum);
        cvZero(_featureSum2);
    }

    // compute (y-p)
    CvMat * y_minus_p;
    y_minus_p = cvCloneMat(y);
    cvSub(y, p, y_minus_p);

    // iterate over all training samples
    for (int i = 0; i < y->rows; i++) {
        // compute p * (1 - p)
        double pp = cvmGet(p, i, 0) * (1.0 - cvmGet(p, i, 0));
	// weight positive class
	if (cvmGet(y, i, 0) > 0.5) {
	    pp *= _posClassWeight;
	    cvmSet(y_minus_p, i, 0, _posClassWeight * cvmGet(y_minus_p, i, 0));
	}
	// compute graident and _hessian
        for (int n = 0; n < _theta->rows; n++) {
            double xyp = cvmGet(x, i, n) * cvmGet(y_minus_p, i, 0);
            cvmSet(_gradient, n, 0, cvmGet(_gradient, n, 0) + xyp);
            for (int m = 0; m < _theta->rows; m++) {
                cvmSet(_hessian, n, m, cvmGet(_hessian, n, m) +
                    cvmGet(x, i, n) * cvmGet(x, i, m) * pp);
            }
        }
	// accumulate statistics for feature mean and variance
	for (int n = 0; n < _theta->rows; n++) {
	    cvmSet(_featureSum, n, 0, cvmGet(_featureSum, n, 0) + cvmGet(x, i, n));
	    cvmSet(_featureSum2, n, 0, cvmGet(_featureSum2, n, 0) + cvmGet(x, i, n) * cvmGet(x, i, n));
	}
    }

    _numSamples += y->rows;
    cvReleaseMat(&y_minus_p);
}

void svlLogistic::accumulate(const vector<vector<double> >& x,
			       const vector<double>& y,
			       const vector<double> &p)
{
    assert((x.size() == y.size()) && (y.size() == p.size()));

    int n = _theta->rows;
    if (_gradient == NULL) {
        _gradient = cvCreateMat(n, 1, CV_32FC1);
        _hessian = cvCreateMat(n, n, CV_32FC1);
        cvZero(_gradient);
        cvZero(_hessian);
    }

    if (_featureSum == NULL) {
        _featureSum = cvCreateMat(n, 1, CV_32FC1);
        _featureSum2 = cvCreateMat(n, n, CV_32FC1);
        cvZero(_featureSum);
        cvZero(_featureSum2);
    }

    // compute (y-p)
    vector<double> y_minus_p(y.size());
    for (unsigned i = 0; i < y.size(); i++) {
	y_minus_p[i] = y[i] - p[i];
    }

    // iterate over all training samples
    for (int i = 0; i < (int)y.size(); i++) {
	assert(x[i].size() == (unsigned)_theta->rows);
        // compute p * (1 - p)
        double pp = p[i] * (1.0 - p[i]);
	// weight positive class
	if (y[i] > 0.5) {
	    pp *= _posClassWeight;
	    y_minus_p[i] *= _posClassWeight;
	}
	// compute _gradient and _hessian
        for (int n = 0; n < _theta->rows; n++) {
            double xyp = x[i][n] * y_minus_p[i];
            cvmSet(_gradient, n, 0, cvmGet(_gradient, n, 0) + xyp);
            for (int m = 0; m < _theta->rows; m++) {
                cvmSet(_hessian, n, m, cvmGet(_hessian, n, m) +
                    x[i][n] * x[i][m] * pp);
            }
        }
	// accumulate statistics for feature mean and variance
	for (int n = 0; n < _theta->rows; n++) {
	    cvmSet(_featureSum, n, 0, cvmGet(_featureSum, n, 0) + x[i][n]);
	    cvmSet(_featureSum2, n, 0, cvmGet(_featureSum2, n, 0) + x[i][n] * x[i][n]);
	}
    }

    _numSamples += (int)y.size();
}

// perform a single training update
void svlLogistic::update()
{
    if (_gradient == NULL) {
        return;
    }
    assert(_numSamples > 0);

    CvMat *update = cvCreateMat(_theta->rows, 1, CV_32FC1);

    for (int n = 0; n < _hessian->rows; n++) {
#if 1
	// TO DO: should implement svlWhitenedLogistic class
	double mu =  cvmGet(_featureSum, n, 0) / (double)_numSamples;
	double sigma = cvmGet(_featureSum2, n, 0) / (double)_numSamples - mu * mu;
#else
	double sigma = 1.0;
#endif
	cvmSet(_hessian, n, n, cvmGet(_hessian, n, n) - _lambda * sigma * _numSamples);
	cvmSet(_gradient, n, 0, cvmGet(_gradient, n, 0) - _lambda * sigma * _numSamples * 
	    cvmGet(_theta, n, 0));
    }

    if (cvSolve(_hessian, _gradient, update, CV_SVD_SYM) == 0) {
        cerr << "WARNING: _hessian is singular in update" << endl;
	if (_theta->rows < 16) {
	    for (int i = 0; i < _theta->rows; i++) {
		for (int j = 0; j < _theta->rows; j++) {
		    cerr << "\t" << cvmGet(_hessian, i, j);
		}
		cerr << "\t\t" << cvmGet(_gradient, i, 0) << endl;
	    }
	}
        cvReleaseMat(&update);
        return;
    }
#if 1
    // TO DO: line search
    cvScale(update, update, _stepSize);
#endif
    cvAdd(_theta, update, _theta);

    cvReleaseMat(&update);

    cvReleaseMat(&_gradient);
    cvReleaseMat(&_hessian);
    _gradient = NULL;
    _hessian = NULL;

    cvReleaseMat(&_featureSum);
    cvReleaseMat(&_featureSum2);
    _featureSum = NULL;
    _featureSum2 = NULL;

    _numSamples = 0;
}

double svlLogistic::train(const CvMat *x, const CvMat *y, double eps,
			       unsigned maxIterations)
{
    assert((x != NULL) && (y != NULL));

    _stepSize = 0.9f;
    CvMat *p = cvCreateMat(y->rows, 1, CV_32FC1);
    double currentMse, lastMse;
    lastMse = numeric_limits<double>::max();
    CvMat *lastTheta = cvCloneMat(_theta);
    unsigned iter = 0;
    while (iter++ < maxIterations) {
	evaluate(x, p);
	currentMse = mse(y, p);
	if ((currentMse > lastMse) || isnan(currentMse)) {
	    cvCopy(lastTheta, _theta);
	    evaluate(x, p);
	    _stepSize *= 0.95f;
	} else {
	    if (lastMse - currentMse < eps) break;
	    lastMse = currentMse;
	    cvCopy(_theta, lastTheta);
	}
	accumulate(x, y, p);
	update();
    }
    cvReleaseMat(&p);
    cvReleaseMat(&lastTheta);

    return lastMse;
}

double svlLogistic::train(const vector<vector<double> >& x, const vector<double>& y,
    double eps, unsigned maxIterations)
{
    _stepSize = 0.9f;
    vector<double> p;
    double currentMse, lastMse;
    lastMse = numeric_limits<double>::max();
    CvMat *lastTheta = cvCloneMat(_theta);
    unsigned iter = 0;
    while (iter++ < maxIterations) {
	evaluate(x, p);
	currentMse = mse(y, p);
	cerr << iter << " " << currentMse << " (" << _stepSize << ")" << endl;
	if ((currentMse > lastMse) || isnan(currentMse)) {
	    cvCopy(lastTheta, _theta);
	    evaluate(x, p);
	    _stepSize *= 0.95f;
	} else {
	    if (lastMse - currentMse < eps) break;
	    lastMse = currentMse;
	    cvCopy(_theta, lastTheta);
	}
	accumulate(x, y, p);
	update();
    }
    cvReleaseMat(&lastTheta);

    return lastMse;
}

double svlLogistic::train(const char *filename, double eps, unsigned maxIterations)
{
    assert(filename != NULL);

    ifstream ifs(filename);
    if (ifs.fail()) {
        cerr << "ERROR (svlLogistic::train): could not open training file " << filename << endl;
        return -1.0;
    }

    vector<vector<double> > x;
    vector<double> y;

    vector<double> v(_theta->rows);
    double d;
    while (!ifs.fail()) {
        for (unsigned i = 0; i < v.size(); i++) {
            ifs >> v[i];
        }
        ifs >> d;
        if (ifs.fail()) break;
        x.push_back(v);
        y.push_back(d);
    }
    ifs.close();

    return train(x, y, eps, maxIterations);
}

svlLogistic& svlLogistic::operator=(const svlLogistic& model)
{
    cvReleaseMat(&_theta);
    _theta = cvCloneMat(model._theta);
    if (_gradient != NULL) {
	cvReleaseMat(&_gradient);
	cvReleaseMat(&_hessian);
        _gradient = NULL;
        _hessian = NULL;
    }

    if (model._gradient != NULL) {
        _gradient = cvCloneMat(model._gradient);
        _hessian = cvCloneMat(model._hessian);
    }

    _posClassWeight = model._posClassWeight;
    _lambda = model._lambda;
    if (_featureSum != NULL) {
	cvReleaseMat(&_featureSum);
	cvReleaseMat(&_featureSum2);
	_featureSum = NULL;
	_featureSum2 = NULL;
    }

    if (model._featureSum != NULL) {
	_featureSum = cvCloneMat(model._featureSum);
	_featureSum2 = cvCloneMat(model._featureSum2);
    }

    _numSamples = model._numSamples;
    _stepSize = model._stepSize;

    return *this;
}


// svlMultiClassLogistic class -----------------------------------------------

svlMultiClassLogistic::svlMultiClassLogistic(unsigned n, unsigned k, double r) :
    _nFeatures(n), _nClasses(k), _lambda(r)
{
    assert((_nClasses > 1) && (_nFeatures > 0));
    _theta.resize(_nClasses - 1);
    for (unsigned i = 0; i < _theta.size(); i++) {
	_theta[i].resize(_nFeatures, 0.0);
    }
    _weights.resize(_nFeatures, 1.0);
}

svlMultiClassLogistic::svlMultiClassLogistic(const svlMultiClassLogistic& model) :
    _nFeatures(model._nFeatures), _nClasses(model._nClasses),
    _theta(model._theta), _weights(model._weights), _lambda(model._lambda)
{
    // do nothing
}

svlMultiClassLogistic::~svlMultiClassLogistic()
{
    // do nothing
}

// initialize the parameters
void svlMultiClassLogistic::initialize(unsigned n, unsigned k, double r)
{
    _nFeatures = n; _nClasses = k; _lambda = r;

    assert((_nClasses > 1) && (_nFeatures > 0));
    _theta.resize(_nClasses - 1);
    for (unsigned i = 0; i < _theta.size(); i++) {
	_theta[i].resize(_nFeatures);
	fill(_theta[i].begin(), _theta[i].end(), 0.0);
    }
    _weights.resize(_nFeatures);
    fill(_weights.begin(), _weights.end(), 1.0);
}

void svlMultiClassLogistic::zero()
{
    for (unsigned i = 0; i < _theta.size(); i++) {
	fill(_theta[i].begin(), _theta[i].end(), 0.0);
    }    
}

// i/o
bool svlMultiClassLogistic::save(const char *filename) const
{
    ofstream ofs(filename);
    if (ofs.fail()) {
	return false;
    }

    ofs << _nFeatures << " " << _nClasses << "\n";
    for (unsigned i = 0; i < _theta.size(); i++) {
	for (unsigned j = 0; j < _theta[i].size(); j++) {
	    ofs << " " << _theta[i][j];
	}
	ofs << "\n";
    }

    ofs.close();
    return true;
}

bool svlMultiClassLogistic::load(const char *filename)
{
    ifstream ifs(filename);
    if (ifs.fail()) {
	return false;
    }

    ifs >> _nFeatures;
    ifs >> _nClasses;

    initialize(_nFeatures, _nClasses, _lambda);
    for (unsigned i = 0; i < _theta.size(); i++) {
	for (unsigned j = 0; j < _theta[i].size(); j++) {
	    ifs >> _theta[i][j];
	}
    }

    ifs.close();
    return true;    
}

// evaluate model (each row in x is a feature vector)
void svlMultiClassLogistic::evaluate(const vector<double>& x, int& y) const
{
    assert(x.size() == (unsigned)_nFeatures);

    double maxScore = 0.0;
    y = (int)_nClasses - 1;
    
    for (int k = 0; k < _nClasses - 1; k++) {
	double score = 0.0;
	for (int i = 0; i < _nFeatures; i++) {
	    score += _theta[k][i] * x[i];
	}
	if (score > maxScore) {
	    y = k;
	    maxScore = score;
	}
    }
}

void svlMultiClassLogistic::evaluate(const vector<vector<double> >& x, vector<int>& y) const
{
    y.resize(x.size());
    vector<vector<double> >::const_iterator itx = x.begin();
    vector<int>::iterator ity = y.begin();
    
    while (itx != x.end()) {
	evaluate(*itx, *ity);
	++itx;
	++ity;
    }
}

void svlMultiClassLogistic::evaluateMarginal(const vector<double>& x, vector<double>& y) const
{
    assert(x.size() == (unsigned)_nFeatures);
    y.resize(_nClasses);
    fill(y.begin(), y.end(), 0.0);
    
    double maxValue = 0.0;
    for (int k = 0; k < _nClasses - 1; k++) {
	for (int i = 0; i < _nFeatures; i++) {
	    y[k] += _theta[k][i] * x[i];
	}
	if (y[k] > maxValue)
	    maxValue = y[k];
    }

    // exponentiate and normalize
    double Z = 0.0;
    for (unsigned i = 0; i < y.size(); i++) {
        y[i] = exp(y[i] - maxValue);
        Z += y[i];
    }

    for (unsigned i = 0; i < y.size(); i++) {
        y[i] /= Z;
    }
}

void svlMultiClassLogistic::evaluateMarginal(const vector<vector<double> >& x,
    vector<vector<double> >& y) const
{
    y.resize(x.size());
    vector<vector<double> >::const_iterator itx = x.begin();
    vector<vector<double> >::iterator ity = y.begin();
    
    while (itx != x.end()) {
	evaluateMarginal(*itx, *ity);
	++itx;
	++ity;
    }
}

// perform full training until log-likelihood no longer decreases
double svlMultiClassLogistic::train(const vector<vector<double> >& x, 
    const vector<int>& y, double eps, int maxIterations)
{
    assert(x.size() == y.size());
    if (x.empty()) return 0.0;

    // check input
    for (unsigned i = 0; i < x.size(); i++) {
	assert(x[i].size() == (unsigned)_nFeatures);
	assert(y[i] < _nClasses);
    }

    // instantiate worker class
    svlMultiClassLogisticOptimizer optimizer((_nClasses - 1) * _nFeatures);

    // copy current parameters into optimizer
    for (int k = 0; k < _nClasses - 1; k++) {
	for (int i = 0; i < _nFeatures; i++) {
	    optimizer[k * _nFeatures + i] = _theta[k][i];
	}
    }
    
    optimizer.nClasses = _nClasses;
    optimizer.nFeatures = _nFeatures;
    optimizer.lambda = _lambda;
    optimizer.pTrainingData = &x;
    optimizer.pTrainingLabels = &y;
    optimizer.pTrainingWeights = NULL;
    
    // solve
    double negLogL = optimizer.solve(maxIterations, eps, true);

    // copy parameters out of optimizer
    for (int k = 0; k < _nClasses - 1; k++) {
	for (int i = 0; i < _nFeatures; i++) {
	    _theta[k][i] = optimizer[k * _nFeatures + i];
	}
    }
    
    return -negLogL;
}

// perform weighted full training until log-likelihood no longer decreases
double svlMultiClassLogistic::train(const vector<vector<double> >& x, 
    const vector<int>& y, const vector<double>& w, double eps, int maxIterations)
{
    assert(x.size() == y.size());
    assert(x.size() == w.size());
    if (x.empty()) return 0.0;

    // check input
    for (unsigned i = 0; i < x.size(); i++) {
	assert(x[i].size() == (unsigned)_nFeatures);
	assert(y[i] < _nClasses);
    }

    // instantiate worker class
    svlMultiClassLogisticOptimizer optimizer((_nClasses - 1) * _nFeatures);

    // copy current parameters into optimizer
    for (int k = 0; k < _nClasses - 1; k++) {
	for (int i = 0; i < _nFeatures; i++) {
	    optimizer[k * _nFeatures + i] = _theta[k][i];
	}
    }
    
    optimizer.nClasses = _nClasses;
    optimizer.nFeatures = _nFeatures;
    optimizer.lambda = _lambda;
    optimizer.pTrainingData = &x;
    optimizer.pTrainingLabels = &y;
    optimizer.pTrainingWeights = &w;
    
    // solve
    double negLogL = optimizer.solve(maxIterations, eps, true);

    // copy parameters out of optimizer
    for (int k = 0; k < _nClasses - 1; k++) {
	for (int i = 0; i < _nFeatures; i++) {
	    _theta[k][i] = optimizer[k * _nFeatures + i];
	}
    }
    
    return -negLogL;
}

double svlMultiClassLogistic::train(const char *filename, double eps,
    int maxIterations)
{
    assert(false);

    vector<vector<double> > x;
    vector<int> y;

    return train(x, y, eps, maxIterations);
}

void svlMultiClassLogistic::setTheta(const vector<vector<double> > &param)
{
    SVL_ASSERT(!param.empty());

    _theta = param;
    _nClasses = param.size() + 1;
    _nFeatures = param[0].size();
    for (int i = 0; i < _nClasses - 1; i++) {
        SVL_ASSERT_MSG(param[i].size() == (unsigned)_nFeatures, 
            "svlMultiClassLogistic::setTheta() non-rectangular input");
    }
}

// standard operators
svlMultiClassLogistic& svlMultiClassLogistic::operator=(const svlMultiClassLogistic& model)
{
    _nFeatures = model._nFeatures;
    _nClasses = model._nClasses;
    _theta = model._theta;
    _weights = model._weights;
    _lambda = model._lambda;

    return *this;
}

// class to do all the grunt work for optimizing the multi-class logistic
double svlMultiClassLogisticOptimizer::objective(const double *x)
{
    double negLogL = 0.0;
    int numTerms = 0;

    vector<double> p(nClasses);
    for (unsigned n = 0; n < pTrainingData->size(); n++) {
	if ((*pTrainingLabels)[n] < 0) continue;	
	fill(p.begin(), p.end(), 0.0);

	// compute marginal for training sample
	double maxValue = 0.0;
	for (int k = 0; k < nClasses - 1; k++) {
	    for (int i = 0; i < nFeatures; i++) {
		p[k] += x[k * nFeatures + i] * (*pTrainingData)[n][i];
	    }
	    if (p[k] > maxValue)
		maxValue = p[k];
	}

	// exponentiate and normalize
	double Z = 0.0;
	for (unsigned i = 0; i < p.size(); i++) {
	    p[i] = exp(p[i] - maxValue);
	    Z += p[i];
	}

        if (pTrainingWeights == NULL) {
            negLogL -= log(p[(*pTrainingLabels)[n]] / Z);
        } else {
            negLogL -= (*pTrainingWeights)[n] * log(p[(*pTrainingLabels)[n]] / Z);
        }
	numTerms += 1;
    }

    // regularization
    double weightNorm = 0.0;
    for (unsigned i = 0; i < _n; i++) {
	weightNorm += x[i] * x[i];
    }

    negLogL += 0.5 * (double)numTerms * lambda * weightNorm;

#if 0
    cerr << "svlMultiClassLogisticOptimizer::objective() is " << negLogL << endl;
#endif
    return negLogL;
}

void svlMultiClassLogisticOptimizer::gradient(const double *x, double *df)
{
    int numTerms = 0;
    memset(df, 0, size() * sizeof(double));
    
    vector<double> p(nClasses);
    for (unsigned n = 0; n < pTrainingData->size(); n++) {
	if ((*pTrainingLabels)[n] < 0) continue;
        double alpha = (pTrainingWeights == NULL) ? 1.0 : (*pTrainingWeights)[n];
	fill(p.begin(), p.end(), 0.0);

	// compute marginal for training sample
	double maxValue = 0.0;
	for (int k = 0; k < nClasses - 1; k++) {
	    for (int i = 0; i < nFeatures; i++) {
		p[k] += x[k * nFeatures + i] * (*pTrainingData)[n][i];
	    }
	    if (p[k] > maxValue)
		maxValue = p[k];
	}

	// exponentiate and normalize
	double Z = 0.0;
	for (unsigned i = 0; i < p.size(); i++) {
	    p[i] = exp(p[i] - maxValue);
	    Z += p[i];
	}
	for (unsigned i = 0; i < p.size(); i++) {
	    p[i] /= Z;
	}

	numTerms += 1;

	// increment derivative
	for (int k = 0; k < nClasses - 1; k++) {
	    for (int i = 0; i < nFeatures; i++) {
		df[k * nFeatures + i] += alpha * p[k] * (*pTrainingData)[n][i];
	    }
	    if ((*pTrainingLabels)[n] == k) {
		for (int i = 0; i < nFeatures; i++) {
		    df[k * nFeatures + i] -= alpha * (*pTrainingData)[n][i];
		}
	    }
	}
    }

    // regularization
    for (unsigned i = 0; i < _n; i++) {
	df[i] += (double)numTerms * lambda * x[i];
    }
}


double svlMultiClassLogisticOptimizer::objectiveAndGradient(const double *x, double *df)
{
    double negLogL = 0.0;
    int numTerms = 0;
    memset(df, 0, size() * sizeof(double));

    vector<double> p(nClasses);
    for (unsigned n = 0; n < pTrainingData->size(); n++) {
	if ((*pTrainingLabels)[n] < 0) continue;
        double alpha = (pTrainingWeights == NULL) ? 1.0 : (*pTrainingWeights)[n];
	fill(p.begin(), p.end(), 0.0);

	// compute marginal for training sample
	double maxValue = 0.0;
	for (int k = 0; k < nClasses - 1; k++) {
	    for (int i = 0; i < nFeatures; i++) {
		p[k] += x[k * nFeatures + i] * (*pTrainingData)[n][i];
	    }
	    if (p[k] > maxValue)
		maxValue = p[k];
	}

	// exponentiate and normalize
	double Z = 0.0;
	for (unsigned i = 0; i < p.size(); i++) {
	    p[i] = exp(p[i] - maxValue);
	    Z += p[i];
	}
	for (unsigned i = 0; i < p.size(); i++) {
	    p[i] /= Z;
	}

	// increment log-likelihood
	negLogL -= alpha * log(p[(*pTrainingLabels)[n]]);
	numTerms += 1;

	// increment derivative
	for (int k = 0; k < nClasses - 1; k++) {
	    for (int i = 0; i < nFeatures; i++) {
		df[k * nFeatures + i] += alpha * p[k] * (*pTrainingData)[n][i];
	    }
	    if ((*pTrainingLabels)[n] == k) {
		for (int i = 0; i < nFeatures; i++) {
		    df[k * nFeatures + i] -= alpha * (*pTrainingData)[n][i];
		}
	    }
	}       	
    }

    // regularization
    double weightNorm = 0.0;
    for (unsigned i = 0; i < _n; i++) {
	weightNorm += x[i] * x[i];
	df[i] += (double)numTerms * lambda * x[i];
    }

    negLogL += 0.5 * (double)numTerms * lambda * weightNorm;

#if 0
    cerr << "svlMultiClassLogisticOptimizer::objectiveAndGradient() is " << negLogL << endl;
#endif
    return negLogL;
}



