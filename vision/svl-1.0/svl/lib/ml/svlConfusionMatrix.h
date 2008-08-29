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
** FILENAME:    svlConfusionMatrix.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**   Utility class for computing and printing confusion matrices. A negative
**   actual/predicted class is considered unknown and not counted.
** 
*****************************************************************************/

#pragma once

#include <vector>
#include <iostream>

class svlConfusionMatrix {
 protected:
    std::vector<std::vector<unsigned> > _matrix;

 public:
    svlConfusionMatrix(int n);
    svlConfusionMatrix(int n, int m);
    virtual ~svlConfusionMatrix();

    int numRows() const;
    int numCols() const;

    void clear();
    void accumulate(int actual, int predicted);
    void accumulate(const std::vector<int>& actual,
	const std::vector<int>& predicted);

    void printCounts(std::ostream &os = std::cout) const;
    void printRowNormalized(std::ostream &os = std::cout) const;
    void printColNormalized(std::ostream &os = std::cout) const;
    void printNormalized(std::ostream &os = std::cout) const;

    void write(std::ostream &os) const;
    void read(std::istream &is);

    double rowSum(int n) const;
    double colSum(int m) const;
    double diagSum() const;
    double totalSum() const;
    double accuracy() const;

    const unsigned operator()(int x, int y) const;
    unsigned& operator()(int x, int y);
};


