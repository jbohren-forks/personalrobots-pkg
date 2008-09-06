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
** FILENAME:    svlGeneralCRF.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <stdlib.h>
#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <deque>
#include <algorithm>
#include <limits>
#include <cmath>

#include "xmlParser/xmlParser.h"

#include "svlBase.h"
#include "svlFactor.h"
#include "svlClusterGraph.h"
#include "svlMessagePassing.h"
#include "svlGeneralCRF.h"

using namespace std;

// svlGeneralCRFInstance Class ----------------------------------------------

svlGeneralCRFInstance::svlGeneralCRFInstance()
{
    // do nothing
}

svlGeneralCRFInstance::svlGeneralCRFInstance(XMLNode& root)
{
    read(root);
}

svlGeneralCRFInstance::~svlGeneralCRFInstance()
{
    // do nothing
}

void svlGeneralCRFInstance::clear()
{
    Yn.clear();
    Kn.clear();
    Cm.clear();
    Xm.clear();
    Tm.clear();
}

vector<int> svlGeneralCRFInstance::getAssignments(int cliqueId) const
{
    assert((cliqueId >= 0) && (cliqueId < (int)Cm.size()));
    return getAssignments(Cm[cliqueId]);
}

vector<int> svlGeneralCRFInstance::getAssignments(const vector<int>& c) const
{
    vector<int> assignments(c.size());
    for (unsigned i = 0; i < c.size(); i++) {
        assignments[i] = Yn[c[i]];
    }
    return assignments;
}

// xml i/o functions
bool svlGeneralCRFInstance::write(ostream& os) const
{
    os << "<crfInstance version=\"1\"\n"
       << "             numVars=\"" << Yn.size() << "\"\n"
       << "             numCliques=\"" << Cm.size() << "\"\n"
       << ">\n";

    // write out variable cardinalities
    os << "  <Cards>\n"
       << "   " << toString(Kn) << "\n"
       << "  </Cards>\n";
    
    // write out target assignments
    os << "  <targetAssignments>\n"
       << "   " << toString(Yn) << "\n"
       << "  </targetAssignments>\n";

    // write out cliques
    for (int m = 0; m < (int)Cm.size(); m++) {
	os << "  <Clique templateIndex=\"" << Tm[m] << "\">\n";
	os << "    <Vars>\n"
	   << "     " << toString(Cm[m]) << "\n"
	   << "    </Vars>\n";
	if (!Xm[m].empty()) {
	    os << "    <Data>\n"
	       << "     " << toString(Xm[m]) << "\n"
	       << "    </Data>\n";
	}
	os << "  </Clique>\n";
    }

    os << "</crfInstance>" << endl;

    return true;
}

bool svlGeneralCRFInstance::write(const char *filename) const
{
    assert(filename != NULL);
    ofstream ofs(filename);
    assert(!ofs.fail());

    return write(ofs);    
}

bool svlGeneralCRFInstance::read(XMLNode& root)
{
    Yn.clear(); Kn.clear();
    Cm.clear(); Xm.clear(); Tm.clear();

    int nVars = atoi(root.getAttribute("numVars"));
    int nCliques = atoi(root.getAttribute("numCliques"));

    XMLNode node = root.getChildNode("Cards");
    assert(node.getText() != NULL);
    parseString<int>(string(node.getText()), Kn);
    assert((int)Kn.size() == nVars);

    node = root.getChildNode("targetAssignments");
    if (node.isEmpty()) {
	Yn.resize(nVars, -1);
    } else {
	assert(node.getText() != NULL);
	parseString<int>(string(node.getText()), Yn);
	assert((int)Yn.size() == nVars);
    }
    
    Cm.resize(nCliques);
    Xm.resize(nCliques);
    Tm.resize(nCliques, -1);
    assert(root.nChildNode("Clique") == nCliques);
    for (int m = 0; m < (int)root.nChildNode("Clique"); m++) {
	node = root.getChildNode("Clique", m);
	Tm[m] = atoi(node.getAttribute("templateIndex"));
	XMLNode subNode = node.getChildNode("Vars");
	assert(subNode.getText() != NULL);
	parseString<int>(string(subNode.getText()), Cm[m]);
	subNode = node.getChildNode("Data");
	if (!subNode.isEmpty() && (subNode.getText() != NULL)) {
	    parseString<double>(string(subNode.getText()), Xm[m]);
	}
    }

    return true;
}

bool svlGeneralCRFInstance::read(const char *filename)
{
    assert(filename != NULL);    
    XMLNode root = XMLNode::parseFile(filename, "crfInstance");
    if (root.isEmpty()) {
        return false;
    }
     
    return read(root);
}

// data integrity functions
bool svlGeneralCRFInstance::checkInstanceData() const
{
    if (Yn.size() != Kn.size()) return false;
    if (Xm.size() != Cm.size()) return false;
    if (Tm.size() != Cm.size()) return false;

    // check cliques contain valid variables
    for (unsigned m = 0; m < Cm.size(); m++) {
	if (Cm[m].empty()) return false;
	for (unsigned i = 0; i < Cm[m].size(); i++) {
	    if ((Cm[m][i] < 0) || (Cm[m][i] >= (int)Yn.size()))
		return false;
	}
    }

    return true;
}

// svlGeneralCRFModel Class --------------------------------------------------

bool svlGeneralCRFModel::BLOCK_COORDINATE_LEARNING = true;

svlGeneralCRFModel::svlGeneralCRFModel(int nw)
{
    assert(nw >= 0);
    _weights.resize(nw, 0.0);
}

svlGeneralCRFModel::svlGeneralCRFModel(const vector<double> &w,
    const vector<svlFactorTemplate> &t) :
    _weights(w), _factorTemplates(t)
{
    // do nothing
}

svlGeneralCRFModel::~svlGeneralCRFModel()
{
    // do nothing
}
    
void svlGeneralCRFModel::setNumWeights(int nw)
{
    assert(nw >= 0);
    _weights.resize(nw, 0.0);
}

void svlGeneralCRFModel::addTemplate(const svlFactorTemplate& t)
{
    _factorTemplates.push_back(t);

    // update number of weights
    if ((int)_weights.size() < t.maxWeightIndex()) {
	_weights.resize(t.maxWeightIndex() + 1);
    }
}

void svlGeneralCRFModel::setTemplates(const vector<svlFactorTemplate>& t)
{
    _factorTemplates = t;

    // update number of weights
    for (vector<svlFactorTemplate>::const_iterator it = t.begin(); it != t.end(); ++it) {
	if ((int)_weights.size() < it->maxWeightIndex()) {
	    _weights.resize(it->maxWeightIndex() + 1);
	}	
    }
}

// xml i/o functions
bool svlGeneralCRFModel::write(ostream& os) const
{
    // write out xml header
    os << "<crfModel version=\"1\"\n"
       << "          numWeights=\"" << _weights.size() << "\"\n"
       << "          numTemplates=\"" << _factorTemplates.size() << "\"\n"
       << ">\n";

    // write out templates
    os << "  <Weights>\n"
       << "   " << toString(_weights) << "\n"
       << "  </Weights>\n";
    
    // write out weights
    for (int i = 0; i < (int)_factorTemplates.size(); i++) {
	_factorTemplates[i].write(os, 2);
    }

    os << "</crfModel>" << endl;

    return true;
}

bool svlGeneralCRFModel::write(const char *filename) const
{
    assert(filename != NULL);
    ofstream ofs(filename);
    assert(!ofs.fail());

    return write(ofs);
}

bool svlGeneralCRFModel::read(const char *filename)
{
    assert(filename != NULL);    
    XMLNode root = XMLNode::parseFile(filename, "crfModel");
    if (root.isEmpty()) {
        return false;
    }

    int nWeights = atoi(root.getAttribute("numWeights"));
    int nTemplates = atoi(root.getAttribute("numTemplates"));

    _weights.clear();
    _factorTemplates.clear();

    XMLNode node = root.getChildNode("Weights");
    parseString<double>(node.getText(), _weights);
    assert((int)_weights.size() == nWeights);

    assert(root.nChildNode("FactorTemplate") == nTemplates);
    for (int i = 0; i < root.nChildNode("FactorTemplate"); i++) {
	node = root.getChildNode("FactorTemplate", i);
	_factorTemplates.push_back(svlFactorTemplate(node));
    }

    return true;
}

bool svlGeneralCRFModel::dumpClusterGraph(ostream& os,
    const svlGeneralCRFInstance& instance) const
{
    svlClusterGraph *graph = buildGraph(instance);
    assert(graph != NULL);
    updatePotentials(graph, instance);
    graph->write(os);
    delete graph;

    return true;
}

// learning and inference
void svlGeneralCRFModel::learn(const vector<svlGeneralCRFInstance>& instances,
    int maxIterations, double lambda)
{
    learn(instances, maxIterations, vector<double>(_weights.size(), lambda));
}

void svlGeneralCRFModel::learn(const vector<svlGeneralCRFInstance>& instances,
    int maxIterations, const vector<double>& lambda)
{
    assert(lambda.size() == _weights.size());
    static int handle = svlCodeProfiler::getHandle("svlGeneralCRFModel::learn");
    svlCodeProfiler::tic(handle);

    if (BLOCK_COORDINATE_LEARNING) {
        fill(_weights.begin(), _weights.end(), 0.0);

        svlGeneralCRFBlockCoordPseudoObjective optimizer(&_factorTemplates, &instances);

        for (unsigned t = 0; t < _factorTemplates.size(); t++) {
            SVL_LOG(SVL_LOG_MESSAGE, "optimizing factor template " << t 
                << " of size " << _factorTemplates[t].size() << "...");
            if (optimizer.setCoordinate(t, _weights, lambda) == 0)
                continue;
            optimizer.solve(maxIterations, 1.0e-3, true);
            optimizer.updateWeights(_weights);
        }
    } else {
        svlGeneralCRFPseudoLikelihoodObjective optimizer((int)_weights.size(),
            &_factorTemplates, &instances);
        optimizer.lambda = lambda;
        optimizer.solve(maxIterations, 1.0e-3, true);
        
        for (int i = 0; i < (int)_weights.size(); i++) {
            _weights[i] = optimizer[i];
        }
    }

    svlCodeProfiler::toc(handle);
}

void svlGeneralCRFModel::inference(const svlGeneralCRFInstance& instance,
    vector<vector<double> >& marginals, int maxIterations)
{
    // build graph and set potentials
    svlClusterGraph *graph = buildGraph(instance);
    updatePotentials(graph, instance);

    // run inference
    svlMessagePassingInference infObj(*graph);
    infObj.inference(SVL_MP_ASYNCSUMPRODDIV, maxIterations);

    // extract marginals
    marginals.resize(instance.numVariables());
    for (int i = 0; i < instance.numVariables(); i++) {
	marginals[i].resize(infObj[i].size(), 0.0);
	for (int j = 0; j < (int)infObj[i].size(); j++) {
	    marginals[i][j] = infObj[i][j];
	}
    }

    // free memory
    delete graph;
}

void svlGeneralCRFModel::inference(const svlGeneralCRFInstance& instance,
    vector<int>& marginals, int maxIterations)
{
    // build graph and set potentials
    svlClusterGraph *graph = buildGraph(instance);
    updatePotentials(graph, instance);

    // run inference
    svlMessagePassingInference infObj(*graph);
    infObj.inference(SVL_MP_ASYNCMAXPRODDIV, maxIterations);
    
    // extract max-marginals
    marginals.resize(instance.numVariables(), 0);
    for (int i = 0; i < instance.numVariables(); i++) {
	for (int j = 0; j < (int)infObj[i].size(); j++) {
	    if (infObj[i][j] > infObj[i][marginals[i]]) {
		marginals[i] = j;
	    }
	}
    }

    // free memory
    delete graph;
}

// compute the energy of a given assignment
double svlGeneralCRFModel::energy(const svlGeneralCRFInstance& instance)
{
    double e = 0.0;

    for (unsigned m = 0; m < instance.Cm.size(); m++) {
        vector<int> assignment = instance.getAssignments(m);
        int indx = _factorTemplates[instance.Tm[m]].indexOf(assignment);
        e += _factorTemplates[instance.Tm[m]].entryLogValue(indx, _weights,
            instance.Xm[m]);
    }

    return e;
}

svlClusterGraph *svlGeneralCRFModel::buildGraph(const svlGeneralCRFInstance& instance) const
{    
    static int handle = svlCodeProfiler::getHandle("svlGeneralCRFModel::buildGraph");
    svlCodeProfiler::tic(handle);

    // build cluster graph
    svlClusterGraph *graph = new svlClusterGraph(instance.numVariables(),
	instance.Kn);
    assert(graph != NULL);

    // add singleton connecting nodes first
    svlClique clique;
    for (int i = 0; i < instance.numVariables(); i++) {
	clique.clear();
	clique.insert(i);
	graph->addClique(clique);
    }

    // add all cliques (which will have assigned factors)
    for (int i = 0; i < instance.numCliques(); i++) {
	clique.clear();
	clique.insert(instance.Cm[i].begin(), instance.Cm[i].end());
	graph->addClique(clique);	
    }

    // connect graph
    //graph->betheApprox();
    graph->connectGraph();

    svlCodeProfiler::toc(handle);
    return graph;
}

void svlGeneralCRFModel::updatePotentials(svlClusterGraph *graph,
    const svlGeneralCRFInstance& instance) const
{
    static int handle = svlCodeProfiler::getHandle("svlGeneralCRFModel::updatePotentials");
    svlCodeProfiler::tic(handle);

    assert(graph != NULL);
    assert(graph->numVariables() == instance.numVariables());

    for (int i = 0; i < instance.numCliques(); i++) {
	assert((instance.Tm[i] >= 0) &&
	    (instance.Tm[i] < (int)_factorTemplates.size()));
	// create potential
	svlFactor phi = _factorTemplates[instance.Tm[i]].createFactor(instance.Cm[i],
	    _weights, instance.Xm[i]);
	// set potential
	graph->setCliquePotential(i + instance.numVariables(), phi);
    }

    svlCodeProfiler::toc(handle);
}

// svlGeneralCRFPseudoLikelihoodObjective -----------------------------------

svlGeneralCRFPseudoLikelihoodObjective::
svlGeneralCRFPseudoLikelihoodObjective(int n,
    const vector<svlFactorTemplate> *pTemplates,
    const vector<svlGeneralCRFInstance> *pInstances) : 
    svlOptimizer(n), _factorTemplates(pTemplates), _instances(pInstances)
{
    assert((_factorTemplates != NULL) && (_instances != NULL));
    lambda.resize(n, 0.0);
}
    
svlGeneralCRFPseudoLikelihoodObjective::~svlGeneralCRFPseudoLikelihoodObjective()
{
    // do nothing
}

double svlGeneralCRFPseudoLikelihoodObjective::objective(const double *x)
{
    double negLogLikelihood = 0.0;

    // TO DO: avoid this copy somehow
    vector<double> weights(_n);
    for (unsigned i = 0; i < _n; i++) {
	weights[i] = x[i];
    }

    // debugging output
    SVL_LOG(SVL_LOG_DEBUG, "weights (objective): " << toString(weights));

    // for each training example
    for (vector<svlGeneralCRFInstance>::const_iterator it = _instances->begin();
	 it != _instances->end(); ++it) {

        // deterimine clique-to-variable mapping for this instance and cache
        // clique potentials
        vector<vector<int> > cliqueSets(it->numVariables());
        vector<svlFactor> cliquePotentials(it->numCliques());
        for (int m = 0; m < it->numCliques(); m++) {
            bool bMissingValues = false;
            for (vector<int>::const_iterator ni = it->Cm[m].begin(); 
                 ni != it->Cm[m].end(); ++ni) {
                if (it->Yn[*ni] < 0) {
                    bMissingValues = true;
                    break;
                }
            }
            if (!bMissingValues) {
                for (vector<int>::const_iterator ni = it->Cm[m].begin(); 
                     ni != it->Cm[m].end(); ++ni) {
                    cliqueSets[*ni].push_back(m);
                }

                // cache clique potential
		cliquePotentials[m] = (*_factorTemplates)[it->Tm[m]].createFactor(it->Cm[m],
		    weights, it->Xm[m]);
            }
        }

	// for each variable
	for (int n = 0; n < it->numVariables(); n++) {
	    // check for missing value
	    if (it->Yn[n] < 0)
		continue;

	    // compute P(Y_n | Y_i - Y_n \in {cliques containing Yn})
	    svlFactor marginal(n, it->Kn[n]);
	    for (vector<int>::const_iterator mi = cliqueSets[n].begin();
		 mi != cliqueSets[n].end(); ++mi) {

#if 0		
		svlFactor phi = (*_factorTemplates)[it->Tm[*mi]].createFactor(it->Cm[*mi],
		    weights, it->Xm[*mi]);
#else
                svlFactor phi = cliquePotentials[*mi];
#endif
		for (int i = 0; i < (int)it->Cm[*mi].size(); i++) {
		    if (it->Cm[*mi][i] == n)
			continue;
		    if (it->Yn[it->Cm[*mi][i]] < 0) {
			// unobserved
			phi.marginalize(it->Cm[*mi][i]);
		    } else {
			// observed
			phi.reduce(it->Cm[*mi][i], it->Yn[it->Cm[*mi][i]]);
		    }
		}
		marginal.product(phi);
	    }
            marginal.normalize();
	    
	    // add to objective
	    negLogLikelihood -= log(marginal[it->Yn[n]]);
	}
    }

    // regularization
    double weightReg = 0.0;
    for (unsigned i = 0; i < _n; i++) {
	weightReg += lambda[i] * x[i] * x[i];
    }
    negLogLikelihood += 0.5 * weightReg;
    
    return negLogLikelihood;
}

void svlGeneralCRFPseudoLikelihoodObjective::gradient(const double *x, double *df)
{
    // objective is cheap to compute compared to the gradient
    objectiveAndGradient(x, df);
}

#define USE_LOG_FACTORS

double svlGeneralCRFPseudoLikelihoodObjective::objectiveAndGradient(const double *x, double *df)
{
    double negLogLikelihood = 0.0;

    // TO DO: avoid this copy somehow
    vector<double> weights(_n);
    for (unsigned i = 0; i < _n; i++) {
	weights[i] = x[i];
    }

    // debugging output
    SVL_LOG(SVL_LOG_DEBUG, "weights (objectiveAndGradient): " << toString(weights));
    SVL_LOG(SVL_LOG_DEBUG, " lambda (objectiveAndGradient): " << toString(lambda));

    vector<double> gradient(_n, 0.0);

    // for each training example
    for (vector<svlGeneralCRFInstance>::const_iterator it = _instances->begin();
	 it != _instances->end(); ++it) {

        // deterimine clique-to-variable mapping for this instance and cache
        // clique potentials
        vector<vector<int> > cliqueSets(it->numVariables());
        vector<svlFactor> cliquePotentials(it->numCliques());
        for (int m = 0; m < it->numCliques(); m++) {
            bool bMissingValues = false;
            for (vector<int>::const_iterator ni = it->Cm[m].begin(); 
                 ni != it->Cm[m].end(); ++ni) {
                if (it->Yn[*ni] < 0) {
                    bMissingValues = true;
                    break;
                }
            }
            if (!bMissingValues) {
                for (vector<int>::const_iterator ni = it->Cm[m].begin(); 
                     ni != it->Cm[m].end(); ++ni) {
                    cliqueSets[*ni].push_back(m);
                }

                // cache clique potential
#ifdef USE_LOG_FACTORS
		cliquePotentials[m] = 
                    (*_factorTemplates)[it->Tm[m]].createLogFactor(it->Cm[m],
                        weights, it->Xm[m]);
#else
		cliquePotentials[m] = (*_factorTemplates)[it->Tm[m]].createFactor(it->Cm[m],
		    weights, it->Xm[m]);
#endif
            }
        }

	// for each variable
	for (int n = 0; n < it->numVariables(); n++) {
	    // check for missing value
	    if (it->Yn[n] < 0)
		continue;

	    // compute P(Y_n | Y_i - Y_n \in {cliques contaning Yn})
	    svlFactor marginal(n, it->Kn[n]);
	    for (vector<int>::const_iterator mi = cliqueSets[n].begin();
		 mi != cliqueSets[n].end(); ++mi) {
		
                svlFactor phi = cliquePotentials[*mi];
		for (int i = 0; i < (int)it->Cm[*mi].size(); i++) {
		    if (it->Cm[*mi][i] == n)
			continue;
		    if (it->Yn[it->Cm[*mi][i]] < 0) {
			// unobserved
#ifdef USE_LOG_FACTORS
                        assert(false);
#else
			phi.marginalize(it->Cm[*mi][i]);
#endif
		    } else {
			// observed
			phi.reduce(it->Cm[*mi][i], it->Yn[it->Cm[*mi][i]]);
		    }
		}
#ifdef USE_LOG_FACTORS
                marginal.add(phi);
#else
		marginal.product(phi);
#endif
	    }

#ifdef USE_LOG_FACTORS
            // exponentiate and normalize
            double maxPhi = marginal[marginal.indexOfMax()];
            for (int i = 0; i < marginal.size(); i++) {
                marginal[i] = exp(marginal[i] - maxPhi);
            }
#endif
            marginal.normalize();
	    
	    // add to objective
	    negLogLikelihood -= log(marginal[it->Yn[n]]);

	    // accumulate sufficient statistics (gradient)
	    for (vector<int>::const_iterator mi = cliqueSets[n].begin();
		 mi != cliqueSets[n].end(); ++mi) {

		int indx = -1;
		vector<int> assignment(it->Cm[*mi].size());
		for (int i = 0; i < (int)assignment.size(); i++) {
		    assignment[i] = it->Yn[it->Cm[*mi][i]];
		    if (it->Cm[*mi][i] == n)
			indx = i;
		}
		assert(indx != -1);

		(*_factorTemplates)[it->Tm[*mi]].accumulateStatistics(gradient,
		    assignment, it->Xm[*mi], -1.0);

		for (int y = 0; y < it->Kn[n]; y++) {
		    assignment[indx] = y;
		    (*_factorTemplates)[it->Tm[*mi]].accumulateStatistics(gradient,
			assignment, it->Xm[*mi], marginal[y]);
		}
	    }
	}
    }

    // compute gradient
    for (unsigned i = 0; i < _n; i++) {
	df[i] = gradient[i];
    }

    // regularization
    double weightReg = 0.0;
    for (unsigned i = 0; i < _n; i++) {
	weightReg += lambda[i] * x[i] * x[i];
        df[i] += lambda[i] * x[i];
    }
    negLogLikelihood += 0.5 * weightReg;
    
    return negLogLikelihood;
}

// svlGeneralCRFBlockCoordPseudoObjective -------------------------------------

svlGeneralCRFBlockCoordPseudoObjective::
svlGeneralCRFBlockCoordPseudoObjective(const vector<svlFactorTemplate> *pTemplates,
    const vector<svlGeneralCRFInstance> *pInstances) : 
    svlOptimizer(), _factorTemplates(pTemplates), _instances(pInstances), 
    _templateId(-1)
{
    assert((_factorTemplates != NULL) && (_instances != NULL));
}

svlGeneralCRFBlockCoordPseudoObjective::~svlGeneralCRFBlockCoordPseudoObjective()
{
    // do nothing
}

int svlGeneralCRFBlockCoordPseudoObjective::setCoordinate(int templateId,
    const vector<double>& weights, const vector<double> &lambda)
{
    assert(_templateId < (int)_factorTemplates->size());
    assert(weights.size() == lambda.size());

    _templateId = templateId;
    
    // determine which weights are associated with _templateId
    _weightMapping.clear();
    set<int> wi = (*_factorTemplates)[_templateId].getWeightIndices();
    _weightMapping.reserve(wi.size());
    for (set<int>::const_iterator it = wi.begin(); it != wi.end(); ++it) {
        _weightMapping.push_back(*it);
    }
    SVL_LOG(SVL_LOG_DEBUG, "template " << templateId << " uses " << _weightMapping.size() 
        << " weights: " << toString(_weightMapping));

    if (_weightMapping.empty())
        return 0;

    // cache clique potentials for all training instances:
    // _cliquePotentials holds cliques with _templateId
    // _varMarginals holds pre-multiplied terms for each variable 
    // not involving cliques with _templateId
    int nTotalCliques = 0;
    int nLearnableCliques = 0;
    _cliquePotentials.resize(_instances->size());
    _varMarginals.resize(_instances->size());
    for (int i = 0; i < (int)_instances->size(); i++) {
        const svlGeneralCRFInstance *pi = &((*_instances)[i]);
        _cliquePotentials[i].resize(pi->numCliques());
        _varMarginals[i].clear();
        _varMarginals[i].reserve(pi->numVariables());

        // add initial margin
        for (int n = 0; n < pi->numVariables(); n++) {
            _varMarginals[i].push_back(svlFactor(n, pi->Kn[n]));
        }

        // iterate over cliques
        for (int m = 0; m < pi->numCliques(); m++) {
#ifdef USE_LOG_FACTORS
            svlFactor phi = (*_factorTemplates)[pi->Tm[m]].createLogFactor(pi->Cm[m],
                weights, pi->Xm[m]);
#else
            svlFactor phi = (*_factorTemplates)[pi->Tm[m]].createFactor(pi->Cm[m],
                weights, pi->Xm[m]);
#endif  
            if (pi->Tm[m] == _templateId) {
                _cliquePotentials[i][m] = phi;            
                nLearnableCliques += 1;
            }

            // iterate over variables in this clique
            if (pi->Tm[m] != _templateId) {
                for (vector<int>::const_iterator ni = pi->Cm[m].begin(); 
                     ni != pi->Cm[m].end(); ++ni) {
                
                    svlFactor reducedPhi = phi;
                    for (vector<int>::const_iterator nj = pi->Cm[m].begin(); 
                         nj != pi->Cm[m].end(); ++nj) {
                        if (*ni == *nj) continue;
                        if (pi->Yn[*nj] < 0) {
                            // unobserved
#ifdef USE_LOG_FACTORS
                            //assert(false);
                            reducedPhi.fill(0.0);
                            reducedPhi.marginalize(*nj);
#else
                            reducedPhi.marginalize(*nj);
#endif
                        } else {
                            // observed
                            reducedPhi.reduce(*nj, pi->Yn[*nj]);
                        }
                    }

#if 0
                    // debugging output
                    cerr << reducedPhi.numVars() << " :";
                    for (int v = 0; v < reducedPhi.numVars(); v++) {
                        cerr << " " << reducedPhi.variableId(v);
                    }
                    cerr << endl;
                    cerr << _varMarginals[i][*ni].numVars() << " : " << _varMarginals[i][*ni].variableId(0) << " (" << *ni << ")" << endl;
#endif
                    assert(reducedPhi.numVars() == 1);
                    assert(reducedPhi.variableId(0) == *ni);

#ifdef USE_LOG_FACTORS
                    _varMarginals[i][*ni].add(reducedPhi);
#else
                    _varMarginals[i][*ni].product(reducedPhi);
#endif                    
                }
            }
        }

        nTotalCliques += pi->numCliques();
    }
    SVL_LOG(SVL_LOG_VERBOSE, "training with " << nTotalCliques << "/" << 
        nLearnableCliques << " total/learnable cliques");

    // initialize optimizer
    initialize((int)_weightMapping.size(), NULL);
    _lambda.resize(_weightMapping.size());
    for (unsigned i = 0; i < _n; i++) {
        _x[i] = weights[_weightMapping[i]];
        _lambda[i] = lambda[_weightMapping[i]];
    }

    // save all weights
    _fullWeights = weights;

    return _n;
}

void svlGeneralCRFBlockCoordPseudoObjective::updateWeights(vector<double> &weights)
{
    assert(_n == _weightMapping.size());
    for (unsigned i = 0; i < _n; i++) {
        weights[_weightMapping[i]] = _x[i];
    }    
}

double svlGeneralCRFBlockCoordPseudoObjective::objective(const double *x)
{
    // not actually called by svlOptimizer
    double *df = new double[_n];
    double f = objectiveAndGradient(x, df);
    delete[] df;

    return f;
}

void svlGeneralCRFBlockCoordPseudoObjective::gradient(const double *x, double *df)
{
    // objective is cheap to compute compared to the gradient
    objectiveAndGradient(x, df);
}

double svlGeneralCRFBlockCoordPseudoObjective::objectiveAndGradient(const double *x, double *df)
{
    assert(_templateId >= 0);
    double negLogLikelihood = 0.0;

    // update full weights
    for (unsigned i = 0; i < _n; i++) {
        _fullWeights[_weightMapping[i]] = x[i];
    }    

    // debugging output
    SVL_LOG(SVL_LOG_DEBUG, "weights (objectiveAndGradient): " << toString(_fullWeights));

    vector<double> gradient(_fullWeights.size(), 0.0);

    // for each training example
    for (int t = 0; t < (int)_instances->size(); t++) {
        const svlGeneralCRFInstance *pi = &((*_instances)[t]);

        // deterimine clique-to-variable mapping for this instance and update
        // cached clique potentials
        vector<vector<int> > cliqueSets(pi->numVariables());
        for (int m = 0; m < pi->numCliques(); m++) {
            bool bMissingValues = false;
            for (vector<int>::const_iterator ni = pi->Cm[m].begin(); 
                 ni != pi->Cm[m].end(); ++ni) {
                if (pi->Yn[*ni] < 0) {
                    bMissingValues = true;
                    break;
                }
            }
            if (!bMissingValues) {
                for (vector<int>::const_iterator ni = pi->Cm[m].begin(); 
                     ni != pi->Cm[m].end(); ++ni) {
                    cliqueSets[*ni].push_back(m);
                }
            }
            
            // update cached clique potential
            if (pi->Tm[m] == _templateId) {
#ifdef USE_LOG_FACTORS
                _cliquePotentials[t][m] = 
                    (*_factorTemplates)[pi->Tm[m]].createLogFactor(pi->Cm[m], 
                        _fullWeights, pi->Xm[m]);
#else
                _cliquePotentials[t][m] = (*_factorTemplates)[pi->Tm[m]].createFactor(pi->Cm[m],
                    _fullWeights, pi->Xm[m]);
#endif  
            }
        }

	// for each variable
	for (int n = 0; n < pi->numVariables(); n++) {
	    // check for missing value
	    if (pi->Yn[n] < 0)
		continue;

	    // compute P(Y_n | Y_i - Y_n \in {cliques contaning Yn})
	    svlFactor marginal(_varMarginals[t][n]);
	    for (vector<int>::const_iterator mi = cliqueSets[n].begin();
		 mi != cliqueSets[n].end(); ++mi) {

                // FIXME: clean up above
                if (pi->Tm[*mi] != _templateId) continue;
		
                svlFactor phi = _cliquePotentials[t][*mi];
		for (int i = 0; i < (int)pi->Cm[*mi].size(); i++) {
		    if (pi->Cm[*mi][i] == n)
			continue;
		    if (pi->Yn[pi->Cm[*mi][i]] < 0) {
			// unobserved
#ifdef USE_LOG_FACTORS
                        assert(false);
#else
			phi.marginalize(pi->Cm[*mi][i]);
#endif
		    } else {
			// observed
			phi.reduce(pi->Cm[*mi][i], pi->Yn[pi->Cm[*mi][i]]);
		    }
		}
#ifdef USE_LOG_FACTORS
                marginal.add(phi);
#else
		marginal.product(phi);
#endif
	    }

#ifdef USE_LOG_FACTORS
            // exponentiate and normalize
            double maxPhi = marginal[marginal.indexOfMax()];
            for (int i = 0; i < marginal.size(); i++) {
                marginal[i] = exp(marginal[i] - maxPhi);
            }
#endif
            marginal.normalize();
	    
	    // add to objective
	    negLogLikelihood -= log(marginal[pi->Yn[n]]);

	    // accumulate sufficient statistics (gradient)
	    for (vector<int>::const_iterator mi = cliqueSets[n].begin();
		 mi != cliqueSets[n].end(); ++mi) {

                if (pi->Tm[*mi] != _templateId) continue;

		int indx = -1;
		vector<int> assignment(pi->Cm[*mi].size());
		for (int i = 0; i < (int)assignment.size(); i++) {
		    assignment[i] = pi->Yn[pi->Cm[*mi][i]];
		    if (pi->Cm[*mi][i] == n)
			indx = i;
		}
		assert(indx != -1);

		(*_factorTemplates)[pi->Tm[*mi]].accumulateStatistics(gradient,
		    assignment, pi->Xm[*mi], -1.0);

		for (int y = 0; y < pi->Kn[n]; y++) {
		    assignment[indx] = y;
		    (*_factorTemplates)[pi->Tm[*mi]].accumulateStatistics(gradient,
			assignment, pi->Xm[*mi], marginal[y]);
		}
	    }
	}
    }

    // compute gradient
    for (unsigned i = 0; i < _n; i++) {
	df[i] = gradient[_weightMapping[i]];
    }

    // regularization
    double weightReg = 0.0;
    for (unsigned i = 0; i < _n; i++) {
	weightReg += _lambda[i] * x[i] * x[i];
        df[i] += _lambda[i] * x[i];
    }
    negLogLikelihood += 0.5 * weightReg;
    
    return negLogLikelihood;
}


