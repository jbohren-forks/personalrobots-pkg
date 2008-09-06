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
** FILENAME:    svlPairwiseCRF.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**   Pairwise conditional Markov random field classes. The classes work
**   together to allow inference and parameter learning of log-linear
**   pairiwse CRF models. These classes assume that all random variables
**   have the same cardinality. See svlGeneralCRF.h for a more general CRF
**   model class.
**
**   The svlPairwiseCRFInstance encapsulates each (training or test)
**   instance and has a number of public data members:
**     Xn     :: feature vectors on each node (in R^p)
**     Yn     :: target assignment for each node (in [0, ..., ])
**               a value < 0 means unobserved
**     Xnm    :: feature vectors on each edge (in R^q)
**     edges  :: defines graph connectivity (same size as Xnm)
**     weight :: weight of instance for training (default: 1.0)
**
**   The svlPairwiseCRFWeights class stores the CRF parameters:
**     Wn     :: singleton feature weights (in R^p)
**     WnMask :: mask for singleton feature weights (in {0,1}^p)
**     Wnm    :: pairwise feature weights (in R^q)
**   Since this class holds the same data structures as required by
**   learning algorithms to keep sufficient statistics, it contains
**   some utility functions for that purpose.
**
**   The svlPairwiseCRFModel class encapsulates parameters learning and
**   inference.
**
*****************************************************************************/

#pragma once

#include <vector>
#include <iostream>

#include "svlBase.h"
#include "svlClusterGraph.h"

using namespace std;

// svlPairwiseCRFInstance Class ---------------------------------------------

class svlPairwiseCRFInstance {
 public:
    vector<vector<double> > Xn;       // singleton features (N x p)
    vector<int> Yn;                   // target assignments (N x 1)
    vector<vector<double> > Xnm;      // pairwise features (E x q)

    vector<pair<int, int> > edges;    // edges

    double weight;                    // training weight
    
 public:
    svlPairwiseCRFInstance();
    svlPairwiseCRFInstance(const svlPairwiseCRFInstance& crf);
    virtual ~svlPairwiseCRFInstance();

    int numVariables() const { return (int)Xn.size(); }
    int dimSingleton() const { return Xn.empty() ? 0 : (int)Xn[0].size(); }
    int dimPairwise() const { return Xnm.empty() ? 0 : (int)Xnm[0].size(); }
    int numEdges() const { return (int)edges.size(); }

    vector<int> getNeighbors(int nodeIndx) const;
    vector<int> getAdjacentEdges(int nodeIndx) const;

    // xml i/o functions
    bool write(ostream& os) const;
    bool write(const char *filename) const;
    bool read(const char *filename);

    // data integrity functions
    bool checkInstanceData() const;
    bool isConnected() const;
};

// svlPairwiseCRFWeights Class ----------------------------------------------

class svlPairwiseCRFWeights
{
 protected:
    int _nClasses;                     // number of classes (K)
    int _nSingletonFeatures;           // dimension of node features
    int _nPairwiseFeatures;            // dimension of edge features

    vector<vector<double> > _Wn;       // singleton feature weights (K x p)
    vector<vector<int> > _WnMask;      // singleton feature mask (K x p)
    vector<vector<double> > _Wnm;      // pairwise feature weights (q x K*(K+1)/2)

 public:
    svlPairwiseCRFWeights();
    svlPairwiseCRFWeights(int K, int p, int q);
    svlPairwiseCRFWeights(const svlPairwiseCRFWeights& w);
    virtual ~svlPairwiseCRFWeights();

    inline int numClasses() const { return _nClasses; }
    inline int dimSingleton() const { return _nSingletonFeatures; }
    inline int dimPairwise() const { return _nPairwiseFeatures; }
    inline int numParameters() const { 
	return (numSingletonParameters() + numPairwiseParameters());
    }
    int numSingletonParameters() const;
    int numPairwiseParameters() const;

    void initialize(int K, int p, int q);
    void initialize(const svlPairwiseCRFWeights& w);
    void reinitialize();

    // i/o functions
    bool write(ostream& os) const;
    bool write(const char *filename) const;
    bool read(const char *filename);

    void toVector(double *w) const;
    void fromVector(const double *w);

    // factor construction functions
    double singletonDotProduct(int Yn, const vector<double>& Xn) const;
    double pairwiseDotProduct(int Yn, int Ym, const vector<double>& Xnm) const;

    // for sufficient statistics
    void singletonAddWeighted(int Yn, const vector<double>& Xn, double alpha = 1.0);
    void pairwiseAddWeighted(int Yn, int Ym, const vector<double>& Xnm, double alpha = 1.0);
 
    // parameter access
    void setFeatureMask(int Yn, const vector<int>& mask);
    double& operator[](unsigned indx);
    double operator[](unsigned indx) const;

 protected:
    int pairwiseIndex(int Yn, int Ym) const;
};

// svlPairwiseCRFModel Class -------------------------------------------------

class svlPairwiseCRFModel
{
 protected:
    svlPairwiseCRFWeights _weights;

 public:
    svlPairwiseCRFModel();
    svlPairwiseCRFModel(int nClasses, int dimSingleton, int dimPairwise);
    svlPairwiseCRFModel(const svlPairwiseCRFWeights& w);
    virtual ~svlPairwiseCRFModel();
    
    inline int numClasses() const { return _weights.numClasses(); }
    inline int dimSingleton() const { return _weights.dimSingleton(); }
    inline int dimPairwise() const { return _weights.dimPairwise(); }

    // xml i/o functions
    bool write(ostream& os) const {
	return _weights.write(os);
    }
    bool write(const char *filename) const {
	return _weights.write(filename);
    }

    bool read(const char *filename) {
	return _weights.read(filename);
    }

    bool dumpClusterGraph(ostream& os,
	const svlPairwiseCRFInstance& instance) const;

    void setFeatureMask(int Yn, const vector<int>& mask) {
        _weights.setFeatureMask(Yn, mask);
    }

    // learning and inference (vectorized versions are faster
    // if adjacent instances have the same structure)
    void learn(const vector<svlPairwiseCRFInstance>& instances,
	int maxIterations = 1000, double lambdaNode = 1.0e-9,
	double lambdaEdge = 1.0e-3);
    void inference(const svlPairwiseCRFInstance& instance,
	vector<vector<double> >& marginals,
	int maxIterations = 1000);
    void inference(const svlPairwiseCRFInstance& instance,
	vector<int>& marginals,
	int maxIterations = 1000);
    void inference(const vector<svlPairwiseCRFInstance>& instances,
        vector<vector<vector<double> > >& marginals,
        int maxIterations = 1000);
    void inference(const vector<svlPairwiseCRFInstance>& instances,
        vector<vector<int> >& marginals,
        int maxIterations = 1000);

 protected:
    svlClusterGraph *buildGraph(const svlPairwiseCRFInstance& instance) const;
    void updatePotentials(svlClusterGraph *graph,
	const svlPairwiseCRFInstance& instance) const;
};

// svlPseudoLikelihoodObjective -------------------------------------------

class svlPseudoLikelihoodObjective : public svlOptimizer
{
 protected:
    svlPairwiseCRFWeights *_weights;
    const vector<svlPairwiseCRFInstance> *_instances;

 public:
    double lambdaNode;
    double lambdaEdge;

 public:
    svlPseudoLikelihoodObjective(svlPairwiseCRFWeights *pWeights,
	const vector<svlPairwiseCRFInstance> *pInstances);
    ~svlPseudoLikelihoodObjective();

    double objective(const double *x);
    void gradient(const double *x, double *df);
    double objectiveAndGradient(const double *x, double *df);

 protected:
    void monitor(unsigned iter, double objValue);    
};



