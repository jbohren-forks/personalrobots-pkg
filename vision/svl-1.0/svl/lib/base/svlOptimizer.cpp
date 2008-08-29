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
** FILENAME:    svlOptimizer.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

// C++ standard library
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <cmath>
#include <string.h>
#include <assert.h>
#include <limits>
#include <iomanip>

// LBFGS library
#include "lbfgs/ap.h"
#include "lbfgs/lbfgs.h"

// SVL headers
#include "svlCompatibility.h"
#include "svlLogger.h"
#include "svlOptimizer.h"

using namespace std;

// Private Functions --------------------------------------------------------

static void fdf(ap::real_1d_array x, double& f, ap::real_1d_array& g, void *params)
{
    for (int i = 0; i < (int)((svlOptimizer *)params)->_n; i++) {
        ((svlOptimizer *)params)->_x[i] = x(i + 1);
    }
    f = ((svlOptimizer *)params)->objectiveAndGradient(((svlOptimizer *)params)->_x,
        ((svlOptimizer *)params)->_df);
    for (int i = 0; i < (int)((svlOptimizer *)params)->_n; i++) {
        g(i + 1) = ((svlOptimizer *)params)->_df[i];
    }
}

static void newiter(int iter, const ap::real_1d_array& x, double f,
    const ap::real_1d_array& g, void *params)
{
    for (int i = 0; i < (int)((svlOptimizer *)params)->_n; i++) {
	((svlOptimizer *)params)->_x[i] = x(i + 1);
    }
    ((svlOptimizer *)params)->monitor(iter, f);
}

// Constructors/Destructors -------------------------------------------------

svlOptimizer::svlOptimizer() :
    _n(0), _x(NULL), _df(NULL)
{
    // do nothing
}

svlOptimizer::svlOptimizer(unsigned n) :
    _n(n)
{
    _x = new double[_n];
    memset(_x, 0, _n * sizeof(double));
    _df = new double[_n];
}

svlOptimizer::svlOptimizer(const svlOptimizer& o) :
    _n(o._n)
{
    if (_n == 0) {
	_x = NULL;
	_df = NULL;
	return;
    }

    _x = new double[_n];
    memcpy(_x, o._x, _n * sizeof(double));
    _df = new double[_n];
    memcpy(_df, o._df, _n * sizeof(double));
}

svlOptimizer::~svlOptimizer()
{
    if (_x != NULL) {
	delete[] _x;
	delete[] _df;
    }
}

// Public Member Functions ---------------------------------------------------

void svlOptimizer::initialize(unsigned n, const double *x)
{
    assert(n != 0);
    if (_x != NULL) {
	delete[] _x;
	delete[] _df;
    }
    
    _x = new double[_n = n];
    _df = new double[_n];

    initialize(x);
}

void svlOptimizer::initialize(const double *x)
{
    assert(_n != 0);
    if (x == NULL) {
	memset(_x, 0, _n * sizeof(double));
    } else {
	memcpy(_x, x, _n * sizeof(double));
    }
}

double svlOptimizer::solve(unsigned maxiter, double tol, bool bMonitor)
{
    assert(_x != NULL);

    // constants
    const int m = (_n < 7 ? _n : 7);
    const double eps = 1.0e-6;

    // starting point
    ap::real_1d_array x;
    x.setbounds(1, (int)_n);
    for (int i = 0; i < (int)_n; i++) {
        x(i + 1) = _x[i];
    }

    // optimize
    int info;
    lbfgsminimize(_n, m, x, fdf, (bMonitor ? newiter : NULL), 
	(void *)this, tol, eps, eps, maxiter, info);

    switch (info) {
    case 1:
        SVL_LOG(SVL_LOG_MESSAGE, "Optimization converged (relative function decrease <= tol).");
        break;
    case 2:
        SVL_LOG(SVL_LOG_MESSAGE, "Optimization converged (step size <= eps)");
        break;
    case 4:
        SVL_LOG(SVL_LOG_MESSAGE, "Optimization converged (gradient norm <= eps)");
        break;
    case 5:
        SVL_LOG(SVL_LOG_MESSAGE, "Maximum number of iterations reached.");
        break;
    default:
        SVL_LOG(SVL_LOG_ERROR, "could not complete optimization.");
	SVL_LOG(SVL_LOG_ERROR, "Parameters were: _n = " << _n << ", m = " << m
            << ", bMonitor = " << (bMonitor ? 1 : 0)
            << ", tol = " << tol << ", eps = "  << eps
            << ", maxiter = " << maxiter << ", info = " << info);
        //assert(false);
    }

    // copy out solution
    for (unsigned i = 0; i < _n; i++) {
	_x[i] = x(i + 1);
    }

    return objective(_x);
}

void svlOptimizer::monitor(unsigned iter, double objValue)
{ 
#if 1
    char buffer[32];
    sprintf(buffer, "%5d %10.5f", iter, objValue);
    SVL_LOG(SVL_LOG_MESSAGE, buffer);
    cout.flush();
#else
    fprintf(stdout, "%5d %10.5f\n", iter, objValue);
#endif
}
    

