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
** FILENAME:    svlGeneralCRF.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**   This set of classes provides functionality for implementing (running
**   inference and learning parameters) for general log-linear conditional
**   Markov random fields. Uses the svlFactorTemplate class for mapping from
**   weights and features to factor (clique potential) entries.
**
**   The svlGeneralCRFInstance encapsulates each (training or test)
**   instance (features and structure) and has a number of public data
**   members:
**     Yn    :: target assignment for each variable (in [0, ..., K(n)-1])
**              a value < 0 means unobserved
**     Kn    :: cardinality for the variable n (in Z+)
**     Cm    :: vector of variables (Y_i) in clique m
**     Xm    :: feature vectors for each clique (in R^p(m))
**     Tm    :: template ID for creating factors (the templates are
**              stored in the actual model, svlGeneralModel)
**
**   The svlGeneralCRFModel class encapsulates parameters learning and
**   inference.
**
** TODO:
**   - weighted training instances
**   - move objective functions into cpp file
**
*****************************************************************************/

#pragma once

#include <vector>
#include <iostream>

#include "xmlParser/xmlParser.h"

#include "svlBase.h"
#include "svlClusterGraph.h"
#include "svlFactorTemplate.h"

using namespace std;

// svlGeneralCRFInstance Class ----------------------------------------------

class svlGeneralCRFInstance {
 public:
    vector<int> Yn;                   // target assignments (N x 1)
    vector<int> Kn;                   // variable cardinality (N x 1)
    
    vector<vector<int> > Cm;          // cliques (factors) (M x s(m))
    vector<vector<double> > Xm;       // clique features (M x p(m))
    vector<int> Tm;                   // template identifier for clique

 public:
    svlGeneralCRFInstance();
    svlGeneralCRFInstance(XMLNode& root);
    virtual ~svlGeneralCRFInstance();

    int numVariables() const { return (int)Yn.size(); }
    int dimVariable(int n) const { return Kn[n]; }
    int numCliques() const { return (int)Cm.size(); }

    void clear();
    vector<int> getAssignments(int cliqueId) const;
    vector<int> getAssignments(const vector<int>& c) const;

    // xml i/o functions
    bool write(ostream& os) const;
    bool write(const char *filename) const;
    bool read(XMLNode& root);
    bool read(const char *filename);

    // data integrity functions
    bool checkInstanceData() const;
};

// svlGeneralCRFModel Class --------------------------------------------------

class svlGeneralCRFModel
{
 public:
    static bool BLOCK_COORDINATE_LEARNING;

 protected:
    vector<double> _weights;
    vector<svlFactorTemplate> _factorTemplates;

 public:
    svlGeneralCRFModel(int nw = 0);
    svlGeneralCRFModel(const vector<double> &w,
	const vector<svlFactorTemplate> &t);
    virtual ~svlGeneralCRFModel();
    
    inline int numWeights() const { return (int)_weights.size(); }
    inline int numTemplates() const { return (int)_factorTemplates.size(); }

    void setNumWeights(int nw);

    void addTemplate(const svlFactorTemplate& t);
    void setTemplates(const vector<svlFactorTemplate>& t);

    // xml i/o functions
    bool write(ostream& os) const;
    bool write(const char *filename) const;
    bool read(const char *filename);

    bool dumpClusterGraph(ostream& os,
	const svlGeneralCRFInstance& instance) const;

    // learning and inference
    void learn(const vector<svlGeneralCRFInstance>& instances,
	int maxIterations = 1000, double lambda = 1.0e-3);
    void learn(const vector<svlGeneralCRFInstance>& instances,
	int maxIterations, const vector<double>& lambda);
    void inference(const svlGeneralCRFInstance& instance,
	vector<vector<double> >& marginals,
	int maxIterations = 1000);
    void inference(const svlGeneralCRFInstance& instance,
	vector<int>& marginals,
	int maxIterations = 1000);
    // compute the energy of a given assignment
    double energy(const svlGeneralCRFInstance& instance);

    // access to weights
    inline double& operator[](unsigned index) { return _weights[index]; }
    inline double operator[](unsigned index) const { return _weights[index]; }

 protected:
    svlClusterGraph *buildGraph(const svlGeneralCRFInstance& instance) const;
    void updatePotentials(svlClusterGraph *graph,
	const svlGeneralCRFInstance& instance) const;
};

// svlGeneralCRFPseudoLikelihoodObjective ---------------------------------

class svlGeneralCRFPseudoLikelihoodObjective : public svlOptimizer
{
 protected:
    const vector<svlFactorTemplate> *_factorTemplates;
    const vector<svlGeneralCRFInstance> *_instances;

 public:
    vector<double> lambda; // regularization (one per weight)

 public:
    svlGeneralCRFPseudoLikelihoodObjective(int n,
        const vector<svlFactorTemplate> *pTemplates,
	const vector<svlGeneralCRFInstance> *pInstances);
    ~svlGeneralCRFPseudoLikelihoodObjective();

    double objective(const double *x);
    void gradient(const double *x, double *df);
    double objectiveAndGradient(const double *x, double *df);

 protected:
    //void monitor(unsigned iter, double objValue);    
};

// svlGeneralCRFBlockCoordPseudoObjective ---------------------------------

class svlGeneralCRFBlockCoordPseudoObjective : public svlOptimizer
{
 protected:
    const vector<svlFactorTemplate> *_factorTemplates;
    const vector<svlGeneralCRFInstance> *_instances;

    int _templateId;
    vector<int> _weightMapping;    
    vector<vector<svlFactor> > _cliquePotentials;
    vector<vector<svlFactor> > _varMarginals;

    vector<double> _fullWeights;
    vector<double> _lambda;

 public:
    svlGeneralCRFBlockCoordPseudoObjective(
        const vector<svlFactorTemplate> *pTemplates,
	const vector<svlGeneralCRFInstance> *pInstances);
    ~svlGeneralCRFBlockCoordPseudoObjective();

    // set the clique template to be optimized; returns size of problem
    int setCoordinate(int templateId, const vector<double>& weights,
        const vector<double>& lambda);
    // retreive the updated weights
    void updateWeights(vector<double>& weights);

    double objective(const double *x);
    void gradient(const double *x, double *df);
    double objectiveAndGradient(const double *x, double *df);

 protected:
    //void monitor(unsigned iter, double objValue);    
};
