/******************************************************************************
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
** FILENAME:    svlConfusionMatrix.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** 
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

#include "svlConfusionMatrix.h"

using namespace std;

svlConfusionMatrix::svlConfusionMatrix(int n)
{
    _matrix.resize(n);
    for (int i = 0; i < n; i++) {
	_matrix[i].resize(n, 0);
    }
}
   
svlConfusionMatrix::svlConfusionMatrix(int n, int m)
{
    _matrix.resize(n);
    for (int i = 0; i < n; i++) {
	_matrix[i].resize(m, 0);
    }
}

svlConfusionMatrix::~svlConfusionMatrix()
{
    // do nothing
}

int svlConfusionMatrix::numRows() const
{
    return _matrix.size();
}

int svlConfusionMatrix::numCols() const
{
    if (_matrix.empty())
        return 0;
    return _matrix[0].size();
}

void svlConfusionMatrix::clear()
{
    for (int i = 0; i < (int)_matrix.size(); i++) {
	fill(_matrix[i].begin(), _matrix[i].end(), 0);
    }
}

void svlConfusionMatrix::accumulate(int actual, int predicted)
{
    if ((actual < 0) || (predicted < 0))
        return;
    _matrix[actual][predicted] += 1;
}

void svlConfusionMatrix::accumulate(const vector<int>& actual,
    const vector<int>& predicted)
{
    assert(actual.size() == predicted.size());
    for (unsigned i = 0; i < actual.size(); i++) {
	// treat < 0 as unknown
	if ((actual[i] < 0) || (predicted[i] < 0))
	    continue;
	assert(actual[i] < (int)_matrix.size());
	assert(predicted[i] < (int)_matrix[actual[i]].size());
	
	_matrix[actual[i]][predicted[i]] += 1;
    }
}

void svlConfusionMatrix::printCounts(ostream &os) const
{
    os << "--- Confusion matrix: (actual, predicted) ---" << endl;
    for (int i = 0; i < (int)_matrix.size(); i++) {
	for (int j = 0; j < (int)_matrix[i].size(); j++) {
	    os << "\t" << _matrix[i][j];
	}
	os << "\n";
    }
    os << "\n";
}

void svlConfusionMatrix::printRowNormalized(ostream &os) const
{
    os << "--- Confusion matrix: (actual, predicted) ---" << endl;
    for (int i = 0; i < (int)_matrix.size(); i++) {
	double total = rowSum(i);
	for (int j = 0; j < (int)_matrix[i].size(); j++) {
	    cout << "\t" << fixed << setprecision(3) << setw(4) 
		 << ((double)_matrix[i][j] / total);
	}
	os << "\n";
    }
    os << "\n";    
}

void svlConfusionMatrix::printColNormalized(ostream &os) const
{
    vector<double> totals;
    for (int i = 0; i < (int)_matrix[0].size(); i++) {
	totals.push_back(colSum(i));
    }

    os << "--- Confusion matrix: (actual, predicted) ---" << endl;
    for (int i = 0; i < (int)_matrix.size(); i++) {
	for (int j = 0; j < (int)_matrix[i].size(); j++) {
	    cout << "\t" << fixed << setprecision(3) << setw(4) 
		 << ((double)_matrix[i][j] / totals[j]);
	}
	os << "\n";
    }
    os << "\n";
}

void svlConfusionMatrix::printNormalized(ostream &os) const
{
    double total = totalSum();

    os << "--- Confusion matrix: (actual, predicted) ---" << endl;
    for (int i = 0; i < (int)_matrix.size(); i++) {
	for (int j = 0; j < (int)_matrix[i].size(); j++) {
	    cout << "\t" << fixed << setprecision(3) << setw(4) 
		 << ((double)_matrix[i][j] / total);
	}
	os << "\n";
    }
    os << "\n";
}

void svlConfusionMatrix::write(ostream &os) const
{
    printCounts(os);
}

void svlConfusionMatrix::read(istream &is)
{
    for (int i = 0; i < (int)_matrix.size(); i++) {
	for (int j = 0; j < (int)_matrix[i].size(); j++) {
	    is >> _matrix[i][j];
	}
    }    
}

double svlConfusionMatrix::rowSum(int n) const
{
    double v = 0.0;
    for (int i = 0; i < (int)_matrix[n].size(); i++) {
	v += (double)_matrix[n][i];
    }
    return v;
}

double svlConfusionMatrix::colSum(int m) const
{
    double v = 0;
    for (int i = 0; i < (int)_matrix.size(); i++) {
	v += (double)_matrix[i][m];
    }
    return v;
}

double svlConfusionMatrix::diagSum() const
{
    double v = 0;
    for (int i = 0; i < (int)_matrix.size(); i++) {
	if (i >= (int)_matrix[i].size())
	    break;
	v += (double)_matrix[i][i];
    }
    return v;
}

double svlConfusionMatrix::totalSum() const
{
    double v = 0;
    for (int i = 0; i < (int)_matrix.size(); i++) {
	for (int j = 0; j < (int)_matrix[i].size(); j++) {
	    v += (double)_matrix[i][j];
	}
    }
    return v;
}

double svlConfusionMatrix::accuracy() const
{
    return (diagSum() / totalSum());
}

const unsigned svlConfusionMatrix::operator()(int i, int j) const
{
    return _matrix[i][j];
}

unsigned& svlConfusionMatrix::operator()(int i, int j)
{
    return _matrix[i][j];
}


