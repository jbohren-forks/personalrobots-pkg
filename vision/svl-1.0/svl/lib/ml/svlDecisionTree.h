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
** FILENAME:    svlDecisionTree.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Implements a decision tree classifier on continuous or discrete data.
**
*****************************************************************************/

#pragma once

#include <vector>
#include <limits>

#include "xmlParser/xmlParser.h"

#include "svlBase.h"

using namespace std;

// svlDecisionTreeDataset class ----------------------------------------------

typedef struct _svlDecisionTreeDataset {
    vector<vector<double> > x;  // feature vectors
    vector<int> y;              // target assignments (< 0 for unobserved)
    vector<double> w;           // instance weights
    
    vector<vector<int> > indx;  // sorted feature vector indices
} svlDecisionTreeDataset;

// svlDecisionTreeNode class -------------------------------------------------

class svlDecisionTreeNode : public svlOptions {
 protected:
    int _nodeIndex;     // back index into decision tree array
    int _featureIndex;  // index into feature vector
    double _threshold;  // tree split
    
    svlDecisionTreeNode * _leftChild;   // feature < _threshold
    svlDecisionTreeNode * _rightChild;  // feature >= _threshold

    double _value;      // value of this node
                        // SVL_DT_DISCRETE : _value = argmax_k E_w[y_i == k]
                        // SVL_DT_REGRESSION : _value = E_w[y_i]

 public:
    svlDecisionTreeNode();
    svlDecisionTreeNode(XMLNode& node);
    virtual ~svlDecisionTreeNode();

    void learn(const svlDecisionTreeDataset& dataset,
        const vector<int>& indices);
    void learn(const svlDecisionTreeDataset& dataset);
    

};

// svlDecisionTree class -----------------------------------------------------


// SVL_BOOST_DISCRETE, SVL_BOOST_REAL, SVL_BOOST_GENTLE



