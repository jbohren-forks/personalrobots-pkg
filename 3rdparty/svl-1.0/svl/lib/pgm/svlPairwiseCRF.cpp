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
** FILENAME:    svlPairwiseCRF.cpp
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
#include "svlPairwiseCRF.h"

using namespace std;

// svlPairwiseCRFInstance Class ---------------------------------------------

svlPairwiseCRFInstance::svlPairwiseCRFInstance() : weight(1.0)
{
    // do nothing
}

svlPairwiseCRFInstance::svlPairwiseCRFInstance(const svlPairwiseCRFInstance& crf)
{
    Xn = crf.Xn;
    Yn = crf.Yn;
    Xnm = crf.Xnm;    
    edges = crf.edges;
    weight = crf.weight;
}

svlPairwiseCRFInstance::~svlPairwiseCRFInstance()
{
    // do nothing
}

vector<int> svlPairwiseCRFInstance::getNeighbors(int nodeIndx) const
{
    vector<int> nbrs;
    for (vector<pair<int, int> >::const_iterator it = edges.begin();
	 it != edges.end(); ++it) {
	if (it->first == nodeIndx)
	    nbrs.push_back(it->second);
	if (it->second == nodeIndx)
	    nbrs.push_back(it->first);
    }

    return nbrs;
}

vector<int> svlPairwiseCRFInstance::getAdjacentEdges(int nodeIndx) const
{
    vector<int> adjs;
    for (int i = 0; i < (int)edges.size(); i++) {
	if ((edges[i].first == nodeIndx) || (edges[i].second == nodeIndx))
	    adjs.push_back(i);
    }

    return adjs;
}

// xml i/o functions
bool svlPairwiseCRFInstance::write(ostream &os) const
{
    os << "<crfInstance nodes=\"" << Xn.size() << "\""
       << " edges=\"" << Xnm.size() << "\""
       << " weight=\"" << weight << "\""
       << ">\n";

    os << "  <edges>\n";
    for (unsigned i = 0; i < Xnm.size(); i++) {
        os << "    " << edges[i].first << " " << edges[i].second << "\n";
    }
    os << "  </edges>\n";
    os << "  <singletonFeatures>\n";
    for (unsigned i = 0; i < Xn.size(); i++) {
        os << "    " << toString(Xn[i]) << "\n";
    }
    os << "  </singletonFeatures>\n";

    os << "  <pairwiseFeatures>\n";
    for (unsigned i = 0; i < Xnm.size(); i++) {
	os << "    " << toString(Xnm[i]) << "\n";
    }
    os << "  </pairwiseFeatures>\n";
    
    os << "  <targetAssignment>\n";
    for (unsigned i = 0; i < Yn.size(); i++) {
        os << "    " << Yn[i] << "\n";
    }
    os << "  </targetAssignment>\n";

    os << "</crfInstance>\n";

    return (!os.fail());
}

bool svlPairwiseCRFInstance::write(const char *filename) const
{
    ofstream os(filename);
    assert(!os.fail());

    this->write(os);
    os.close();

    return true;
}

bool svlPairwiseCRFInstance::read(const char *filename)
{
    XMLNode root = XMLNode::parseFile(filename, "crfInstance");
    if (root.isEmpty()) {
        return false;
    }

    unsigned i, j, k;
    unsigned nXn, nXnm, dXn, dXnm;
    nXn = atoi(root.getAttribute("nodes"));
    assert(nXn > 0);
    nXnm = atoi(root.getAttribute("edges"));
    if (root.getAttribute("weight") == NULL) {
        weight = 1.0;
    } else {
        weight = atof(root.getAttribute("weight"));
    }

    edges.resize(nXnm);
    vector<int> vi;
    parseString(root.getChildNode("edges").getText(), vi);
    assert(vi.size() == 2 * edges.size());
    for (i = 0; i < edges.size(); i++) {
        edges[i].first = vi[2 * i];
        edges[i].second = vi[2 * i + 1];
    }
    
    vector<double> vf;
    XMLNode node;

    Xn.resize(nXn);
    vf.clear();
    node = root.getChildNode("singletonFeatures");
    if (!node.isEmpty()) {
	parseString(node.getText(), vf);
    }
    dXn = vf.size() / nXn;
    for (i = 0, k = 0; i < Xn.size(); i++) {
        Xn[i].resize(dXn);
        for (j = 0; j < dXn; j++) {
            Xn[i][j] = vf[k++];
        }
    }

    Xnm.resize(nXnm);
    vf.clear();
    node = root.getChildNode("pairwiseFeatures");
    if (!node.isEmpty()) {
	parseString(node.getText(), vf);
    }
    dXnm = vf.size() / nXnm;
    for (i = 0, k = 0; i < Xnm.size(); i++) {
	Xnm[i].resize(dXnm);
	for (j = 0; j < dXnm; j++) {
	    Xnm[i][j] = vf[k++];
	}
    }
    
    node = root.getChildNode("targetAssignment");
    if (!node.isEmpty()) {
        parseString(node.getText(), Yn);
        assert(Yn.size() == Xn.size());
    } else {
        Yn.resize(nXn);
        fill(Yn.begin(), Yn.end(), -1);
    }

    return true;
}

// data integrity functions
bool svlPairwiseCRFInstance::checkInstanceData() const
{
    if (!Yn.empty() && (Yn.size() != Xn.size())) return false;
    if (Xnm.size() != edges.size()) return false;

    // check feature vector sizes
    for (vector<vector<double> >::const_iterator it = Xn.begin() + 1;
	 it != Xn.end(); ++it) {
	if (it->size() != Xn[0].size())
	    return false;
    }

    for (vector<vector<double> >::const_iterator it = Xnm.begin() + 1;
	 it != Xnm.end(); ++it) {
	if (it->size() != Xnm[0].size())
	    return false;
    }

    // check edges
    for (vector<pair<int, int> >::const_iterator it = edges.begin();
	 it != edges.end(); ++it) {
	if ((it->first < 0) || (it->first >= (int)Xn.size()))
	    return false;
	if ((it->second < 0) || (it->second >= (int)Xn.size()))
	    return false;
    }

    return true;
}

bool svlPairwiseCRFInstance::isConnected() const
{
    if (Xn.size() <= 1) {
        return true;
    }

    // perform depth-first search from 0 to every other node
    vector<bool> bNodesVisited(Xn.size(), false);

    deque<int> frontier;
    frontier.push_back(0);

    while (!frontier.empty()) {
        int currentNode = frontier.front();
        frontier.pop_front();
        if (bNodesVisited[currentNode]) {
            continue;
        }
        bNodesVisited[currentNode] = true;
        for (unsigned i = 0; i < edges.size(); i++) {
            if ((edges[i].first == currentNode) &&
		(!bNodesVisited[edges[i].second])) {
                frontier.push_back(edges[i].second);
	    } else if ((edges[i].second == currentNode) &&
		(!bNodesVisited[edges[i].first])) {
                frontier.push_back(edges[i].first);
            }
        }
    }

    for (unsigned i = 0; i < bNodesVisited.size(); i++) {
        if (bNodesVisited[i] == false)
            return false;
    }

    return true;
}

// svlPairwiseCRFWeights Class ----------------------------------------------

svlPairwiseCRFWeights::svlPairwiseCRFWeights() :
    _nClasses(0), _nSingletonFeatures(0), _nPairwiseFeatures(0)
{
    // do nothing
}

svlPairwiseCRFWeights::svlPairwiseCRFWeights(int K, int p, int q) :
    _nClasses(0), _nSingletonFeatures(0), _nPairwiseFeatures(0)
{
    initialize(K, p, q);
}

svlPairwiseCRFWeights::svlPairwiseCRFWeights(const svlPairwiseCRFWeights& w) :
    _nClasses(0), _nSingletonFeatures(0), _nPairwiseFeatures(0)
{
    initialize(w);
}

svlPairwiseCRFWeights::~svlPairwiseCRFWeights()
{
    // do nothing
}

int svlPairwiseCRFWeights::numSingletonParameters() const
{ 
    return (_nClasses * _nSingletonFeatures);
}

int svlPairwiseCRFWeights::numPairwiseParameters() const
{ 
    return (_nClasses * (_nClasses + 1) / 2 * _nPairwiseFeatures);
}				    

void svlPairwiseCRFWeights::initialize(int K, int p, int q)
{
    assert((K > 1) && (p > 0) && (q >= 0));

    _nClasses = K;
    _nSingletonFeatures = p;
    _nPairwiseFeatures = q;

    reinitialize();
}

void svlPairwiseCRFWeights::initialize(const svlPairwiseCRFWeights& w)
{
    _nClasses = w._nClasses;
    _nSingletonFeatures = w._nSingletonFeatures;
    _nPairwiseFeatures = w._nPairwiseFeatures;

    reinitialize();
    _WnMask = w._WnMask;
}

void svlPairwiseCRFWeights::reinitialize()
{
    // weights
    _Wn.resize(_nClasses);
    for (int i = 0; i < _nClasses; i++) {
	_Wn[i].resize(_nSingletonFeatures);
	fill(_Wn[i].begin(), _Wn[i].end(), 0.0);
    }

    _Wnm.resize(_nClasses * (_nClasses + 1) / 2);
    for (int i = 0; i < (int)_Wnm.size(); i++) {
	_Wnm[i].resize(_nPairwiseFeatures);
	fill(_Wnm[i].begin(), _Wnm[i].end(), 0.0);
    }

    // masks
    _WnMask.resize(_nClasses);
    for (int i = 0; i < _nClasses; i++) {
        _WnMask[i].resize(_nSingletonFeatures);
        fill(_WnMask[i].begin(), _WnMask[i].end(), 1);
    }
}

// xml i/o functions
bool svlPairwiseCRFWeights::write(ostream& os) const
{
    // write out xml header
    os << "<crfWeights version=\"2\"\n"
       << "            classes=\"" << _nClasses << "\"\n"
       << "            singletonFeatures=\"" << _nSingletonFeatures << "\"\n"
       << "            pairwiseFeatures=\"" << _nPairwiseFeatures << "\"\n"
       << ">\n";

    os << "  <singletonWeights>\n";
    for (unsigned i = 0; i < _Wn.size(); i++) {
	os << "    " << toString(_Wn[i]) << "\n";
    }
    os << "  </singletonWeights>\n";

    if (_nPairwiseFeatures > 0) {
	os << "  <pairwiseWeights>\n";
	for (unsigned i = 0; i < _Wnm.size(); i++) {
	    os << "    " << toString(_Wnm[i]) << "\n";
	}
	os << "  </pairwiseWeights>\n";
    }

    os << "  <singletonMasks>\n";
    for (unsigned i = 0; i < _WnMask.size(); i++) {
	os << "    " << toString(_WnMask[i]) << "\n";
    }
    os << "  </singletonMasks>\n";

    os << "</crfWeights>\n";

    return (!os.fail());
}

bool svlPairwiseCRFWeights::write(const char *filename) const
{
    ofstream os(filename);
    assert(!os.fail());

    this->write(os);
    os.close();

    return true;
}

bool svlPairwiseCRFWeights::read(const char *filename)
{
    XMLNode root = XMLNode::parseFile(filename, "crfWeights");
    if (root.isEmpty()) {
        return false;
    }

    _nClasses = atoi(root.getAttribute("classes"));
    _nSingletonFeatures = atoi(root.getAttribute("singletonFeatures"));
    _nPairwiseFeatures = atoi(root.getAttribute("pairwiseFeatures"));

    unsigned i, j, k;
    vector<double> vf;
    vector<int> vi;
    XMLNode node;

    _Wn.resize(_nClasses);
    vf.clear();
    node = root.getChildNode("singletonWeights");
    assert(!node.isEmpty());
    parseString(node.getText(), vf);
    assert(vf.size() == (unsigned)(_Wn.size() * _nSingletonFeatures));
    for (i = 0, k = 0; i < _Wn.size(); i++) {
        _Wn[i].resize(_nSingletonFeatures);
        for (j = 0; j < _Wn[i].size(); j++) {
            _Wn[i][j] = vf[k++];
        }
    }

    if (_nPairwiseFeatures > 0) {
	_Wnm.resize(_nClasses * (_nClasses + 1) / 2);
	vf.clear();
	node = root.getChildNode("pairwiseWeights");
	assert(!node.isEmpty());
	parseString(node.getText(), vf);
	assert(vf.size() == (unsigned)(_Wnm.size() * _nPairwiseFeatures));

	for (i = 0, k = 0; i < _Wnm.size(); i++) {
	    _Wnm[i].resize(_nPairwiseFeatures);
	    for (j = 0; j < _Wnm[i].size(); j++) {
		_Wnm[i][j] = vf[k++];
	    }
	}
    } else {
	_Wnm.clear();
    }

    _WnMask.resize(_nClasses);
    node = root.getChildNode("singletonMasks");
    if (node.isEmpty()) {
        for (i = 0; i < _WnMask.size(); i++) {
            _WnMask[i].resize(_nSingletonFeatures);
            fill(_WnMask[i].begin(), _WnMask[i].end(), 1);
        }
    } else {
        vi.clear();
        parseString(node.getText(), vi);
        assert(vi.size() == (unsigned)(_WnMask.size() * _nSingletonFeatures));
        for (i = 0, k = 0; i < _WnMask.size(); i++) {
            _WnMask[i].resize(_nSingletonFeatures);
            for (j = 0; j < _WnMask[i].size(); j++) {
                _WnMask[i][j] = vi[k++];
            }
        }
    }

    return true;
}

void svlPairwiseCRFWeights::toVector(double *w) const
{
    // singleton weights
    for (unsigned i = 0; i < _Wn.size(); i++) {
	for (unsigned j = 0; j < _Wn[i].size(); j++) {
	    *w++ = _Wn[i][j];
	}
    }

    // pairwise weights
    for (unsigned i = 0; i < _Wnm.size(); i++) {
	for (unsigned j = 0; j < _Wnm[i].size(); j++) {
	    *w++ = _Wnm[i][j];
	}
    }
}

void svlPairwiseCRFWeights::fromVector(const double *w)
{
    // singleton weights
    for (unsigned i = 0; i < _Wn.size(); i++) {
	for (unsigned j = 0; j < _Wn[i].size(); j++) {
	    _Wn[i][j] = *w++;
	}
    }

    // pairwise weights
    for (unsigned i = 0; i < _Wnm.size(); i++) {
	for (unsigned j = 0; j < _Wnm[i].size(); j++) {
	    _Wnm[i][j] = *w++;
	}
    }
}

// factor construction functions
double svlPairwiseCRFWeights::singletonDotProduct(int Yn, const vector<double>& Xn) const
{
    assert((Yn >= 0) && (Yn < _nClasses));
    assert(Xn.size() == _Wn[Yn].size());

    double v = 0.0;
    for (unsigned i = 0; i < Xn.size(); i++) {
        if (_WnMask[Yn][i]) {
            v += _Wn[Yn][i] * Xn[i];
        }
    }

    return v;
}

double svlPairwiseCRFWeights::pairwiseDotProduct(int Yn, int Ym, const vector<double>& Xnm) const
{
    int indx = pairwiseIndex(Yn, Ym);

    double v = 0.0;
    assert(Xnm.size() == _Wnm[indx].size());
    for (unsigned i = 0; i < Xnm.size(); i++) {
	v += _Wnm[indx][i] * Xnm[i];
    }

    return v;
}

// for sufficient statistics
void svlPairwiseCRFWeights::singletonAddWeighted(int Yn, const vector<double>& Xn, double alpha)
{
    assert((Yn >= 0) && (Yn < _nClasses));
    assert(Xn.size() == _Wn[Yn].size());

    for (unsigned i = 0; i < Xn.size(); i++) {
        if (_WnMask[Yn][i]) {
            _Wn[Yn][i] += alpha * Xn[i];
        }
    }
}

void svlPairwiseCRFWeights::pairwiseAddWeighted(int Yn, int Ym, const vector<double>& Xnm, double alpha)
{
    int indx = pairwiseIndex(Yn, Ym);

    assert(Xnm.size() == _Wnm[indx].size());
    for (unsigned i = 0; i < Xnm.size(); i++) {
	_Wnm[indx][i] += alpha * Xnm[i];
    }
}

// parameter access
void svlPairwiseCRFWeights::setFeatureMask(int Yn, const vector<int>& mask)
{
    assert((Yn >= 0) && (Yn < _nClasses));
    assert(mask.size() == (unsigned)_nSingletonFeatures);

    _WnMask[Yn] = mask;
}

double& svlPairwiseCRFWeights::operator[](unsigned indx)
{
    assert(indx < (unsigned)numParameters());
    if (indx < (unsigned)numSingletonParameters()) {
	int i = (int)(indx / _nSingletonFeatures);
	int j = indx % _nSingletonFeatures;
	return _Wn[i][j];
    } else {
	indx -= numSingletonParameters();
	int i = indx % _nPairwiseFeatures;
	int j = (int)(indx / _nPairwiseFeatures);
	return _Wnm[i][j];	
    }
}

double svlPairwiseCRFWeights::operator[](unsigned indx) const
{
    assert(indx < (unsigned)numParameters());
    if (indx < (unsigned)numSingletonParameters()) {
	int i = (int)(indx / _nSingletonFeatures);
	int j = indx % _nSingletonFeatures;
	return _Wn[i][j];
    } else {
	indx -= numSingletonParameters();
	int i = indx % _nPairwiseFeatures;
	int j = (int)(indx / _nPairwiseFeatures);
	return _Wnm[i][j];	
    }
}

int svlPairwiseCRFWeights::pairwiseIndex(int Yn, int Ym) const
{
    assert((Yn >= 0) && (Yn < _nClasses));
    assert((Ym >= 0) && (Ym < _nClasses));

    int indx;
    if (Yn < Ym) {
	indx = ((2 * _nClasses - Yn + 1) * Yn) / 2 + (Ym - Yn);
    } else {
	indx = ((2 * _nClasses - Ym + 1) * Ym) / 2 + (Yn - Ym);
    }
    
    return indx;
}

// svlPairwiseCRFModel Class -------------------------------------------------

svlPairwiseCRFModel::svlPairwiseCRFModel()
{
    // do nothing
}

svlPairwiseCRFModel::svlPairwiseCRFModel(int nClasses, int dimSingleton, int dimPairwise) :
    _weights(nClasses, dimSingleton, dimPairwise)
{
    // do nothing
}

svlPairwiseCRFModel::svlPairwiseCRFModel(const svlPairwiseCRFWeights& w) :
    _weights(w)
{
    // do nothing
}

svlPairwiseCRFModel::~svlPairwiseCRFModel()
{
    // do nothing
}


bool svlPairwiseCRFModel::dumpClusterGraph(ostream& os,
    const svlPairwiseCRFInstance& instance) const
{
    svlClusterGraph *graph = buildGraph(instance);
    assert(graph != NULL);
    updatePotentials(graph, instance);
    graph->write(os);
    delete graph;

    return true;
}

// learn weights using pseudo-likdlihood objective    
void svlPairwiseCRFModel::learn(const vector<svlPairwiseCRFInstance>& instances,
    int maxIterations, double lambdaNode, double lambdaEdge)
{
    svlPseudoLikelihoodObjective optimizer(&_weights, &instances);
    optimizer.lambdaNode = lambdaNode;
    optimizer.lambdaEdge = lambdaEdge;
    optimizer.solve(maxIterations, 1.0e-3, true);
}

void svlPairwiseCRFModel::inference(const svlPairwiseCRFInstance& instance,
    vector<vector<double> >& marginals, int maxIterations)
{
    // build graph and set potentials
    svlClusterGraph *graph = buildGraph(instance);
    updatePotentials(graph, instance);

    // run inference
    svlMessagePassingInference infObj(*graph);
    infObj.inference(SVL_MP_SUMPROD, maxIterations);
    
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

void svlPairwiseCRFModel::inference(const svlPairwiseCRFInstance& instance,
    vector<int>& marginals, int maxIterations)
{
    // build graph and set potentials
    svlClusterGraph *graph = buildGraph(instance);
    updatePotentials(graph, instance);

    // run inference
    svlMessagePassingInference infObj(*graph);
    infObj.inference(SVL_MP_MAXPROD, maxIterations);
    
    // extract max-marginals
    marginals.resize(instance.numVariables(), 0);
    for (int i = 0; i < instance.numVariables(); i++) {
	for (int j = 1; j < (int)infObj[i].size(); j++) {
	    if (infObj[i][j] > infObj[i][marginals[i]]) {
		marginals[i] = j;
	    }
	}
    }

    // free memory
    delete graph;
}

void svlPairwiseCRFModel::inference(const vector<svlPairwiseCRFInstance>& instances,
    vector<vector<vector<double> > >& marginals, int maxIterations)
{
    // reserve memory for marginals
    marginals.resize(instances.size());

    svlMessagePassingInference *infObj = NULL;

    // iterate through instances
    for (unsigned n = 0; n < instances.size(); n++) {
        // clear inference objects if last instance had different graph
        if ((n > 0) && (instances[n].edges != instances[n - 1].edges)) {
            delete infObj;
            infObj = NULL;
        }
        
        // build graph and set potentials
        if (infObj == NULL) {
            svlClusterGraph *graph = buildGraph(instances[n]);
            infObj = new svlMessagePassingInference(*graph);
            delete graph;
        }
        assert(infObj != NULL);
        updatePotentials(&infObj->graph(), instances[n]);

        // run inference
        infObj->inference(SVL_MP_SUMPROD, maxIterations);
    
        // extract marginals
        marginals[n].resize(instances[n].numVariables());
        for (int i = 0; i < instances[n].numVariables(); i++) {
            marginals[n][i].resize((*infObj)[i].size(), 0.0);
            for (int j = 0; j < (int)(*infObj)[i].size(); j++) {
                marginals[n][i][j] = (*infObj)[i][j];
            }
        }
    }

    // free memory
    if (infObj != NULL) {
        delete infObj;
    }
}

void svlPairwiseCRFModel::inference(const vector<svlPairwiseCRFInstance>& instances,
    vector<vector<int> >& marginals, int maxIterations)
{
    // reserve memory for marginals
    marginals.resize(instances.size());

    svlMessagePassingInference *infObj = NULL;

    // iterate through instances
    for (unsigned n = 0; n < instances.size(); n++) {
        // clear inference objects if last instance had different graph
        if ((n > 0) && (instances[n].edges != instances[n - 1].edges)) {
            delete infObj;
            infObj = NULL;
        }
        
        // build graph and set potentials
        if (infObj == NULL) {
            svlClusterGraph *graph = buildGraph(instances[n]);
            infObj = new svlMessagePassingInference(*graph);
            delete graph;
        }
        assert(infObj != NULL);
        updatePotentials(&infObj->graph(), instances[n]);

        // run inference
        infObj->inference(SVL_MP_MAXPROD, maxIterations);
    
        // extract marginals
        marginals[n].resize(instances[n].numVariables(), 0);
        for (int i = 0; i < instances[n].numVariables(); i++) {
            for (int j = 1; j < (int)(*infObj)[i].size(); j++) {
                if ((*infObj)[i][j] > (*infObj)[i][marginals[n][i]]) {
                    marginals[n][i] = j;
                }
            }
        }
    }

    // free memory
    if (infObj != NULL) {
        delete infObj;
    }
}

svlClusterGraph *svlPairwiseCRFModel::buildGraph(const svlPairwiseCRFInstance& instance) const
{
    // build cluster graph
    svlClusterGraph *graph = new svlClusterGraph(instance.numVariables(),
	_weights.numClasses());
    assert(graph != NULL);

    // add singleton nodes
    svlClique clique;
    for (int i = 0; i < instance.numVariables(); i++) {
	clique.clear();
	clique.insert(i);
	graph->addClique(clique);
    }

    // add pairwise nodes
    for (int i = 0; i < instance.numEdges(); i++) {
	clique.clear();
	clique.insert(instance.edges[i].first);
	clique.insert(instance.edges[i].second);
	graph->addClique(clique);	
    }

    // connect graph
    graph->betheApprox();

    return graph;
}

void svlPairwiseCRFModel::updatePotentials(svlClusterGraph *graph,
    const svlPairwiseCRFInstance& instance) const
{
    assert(graph != NULL);
    assert(graph->numVariables() == instance.numVariables());

    // set node potentials
    for (int i = 0; i < instance.numVariables(); i++) {
	svlFactor phi(i, _weights.numClasses());
	double maxPhi = -numeric_limits<double>::max();
	// theta^transpose x
	for (int j = 0; j < _weights.numClasses(); j++) {
	    phi[j] = _weights.singletonDotProduct(j, instance.Xn[i]);
	    if (phi[j] > maxPhi)
		maxPhi = phi[j];
	}
	// exponentiate
	for (int j = 0; j < phi.size(); j++) {
	    phi[j] = exp(phi[j] - maxPhi);
	}
	// set potential
	graph->setCliquePotential(i, phi);
    }

    // set edge potentials
    for (int i = 0; i < instance.numEdges(); i++) {
	svlFactor phi;
	phi.addVariable(instance.edges[i].first, _weights.numClasses());
	phi.addVariable(instance.edges[i].second, _weights.numClasses());
	int indx = 0;
	double maxPhi = -numeric_limits<double>::max();
	// theta^transpose x
	for (int v1 = 0; v1 < _weights.numClasses(); v1++) {
	    indx = phi.indexOf(instance.edges[i].first, v1);
	    for (int v2 = 0; v2 < _weights.numClasses(); v2++) {
		indx = phi.indexOf(instance.edges[i].second, v2, indx);
		phi[indx] = _weights.pairwiseDotProduct(v1, v2, instance.Xnm[i]);
		if (phi[indx] > maxPhi)
		    maxPhi = phi[indx];
	    }
	}
	// exponentiate
	for (int j = 0; j < phi.size(); j++) {
	    phi[j] = exp(phi[j] - maxPhi);
	}
	// set potential
	graph->setCliquePotential(i + instance.numVariables(), phi);
    }
}

// svlPseudoLikelihoodObjective ---------------------------------------------

svlPseudoLikelihoodObjective::svlPseudoLikelihoodObjective(svlPairwiseCRFWeights *pWeights,
    const vector<svlPairwiseCRFInstance> *pInstances) : 
    svlOptimizer(), _weights(pWeights), _instances(pInstances),
    lambdaNode(0.0), lambdaEdge(0.0)
{
    assert((_weights != NULL) && (_instances != NULL));
    initialize(_weights->numParameters());
}
    
svlPseudoLikelihoodObjective::~svlPseudoLikelihoodObjective()
{
    // do nothing
}

double svlPseudoLikelihoodObjective::objective(const double *x)
{
    double negLogLikelihood = 0.0;

    _weights->fromVector(x);    
    vector<double> pr(_weights->numClasses());
    double numTerms = 0.0;

    // for each training example
    for (vector<svlPairwiseCRFInstance>::const_iterator it = _instances->begin();
	 it != _instances->end(); ++it) {
	// adjacent edges to each variable
	vector<vector<int> > adjEdges(it->numVariables());
	for (int e = 0; e < (int)it->edges.size(); e++) {
	    adjEdges[it->edges[e].first].push_back(e);
	    adjEdges[it->edges[e].second].push_back(e);
	}
	// for each variable
	for (int n = 0; n < it->numVariables(); n++) {
	    // check for missing value
	    if (it->Yn[n] < 0)
		continue;

	    double maxPr = -numeric_limits<double>::max();
	    for (int y = 0; y < _weights->numClasses(); y++) {
		// singleton term
		pr[y] = _weights->singletonDotProduct(y, it->Xn[n]);
		// pairwise terms
		for (vector<int>::const_iterator e = adjEdges[n].begin();
		     e != adjEdges[n].end(); e++) {
		    int m = it->edges[*e].first;
		    if (m == n) m = it->edges[*e].second;
#if 0
		    if (m < n) continue;
#endif
		    // check for missing values
		    if (it->Yn[m] < 0)
			continue;
		    pr[y] += _weights->pairwiseDotProduct(y, it->Yn[m], it->Xnm[*e]);
		}
		
		// check maximum potential
		if (pr[y] > maxPr)
		    maxPr = pr[y];
	    }
	    
	    // compute partition function
	    double Z = 0.0;
	    for (int y = 0; y < _weights->numClasses(); y++) {
		Z += exp(pr[y] - maxPr);
	    }
	    assert(Z > 0.0);
	    
	    // add to (weighted) objective
	    negLogLikelihood -= it->weight * (pr[it->Yn[n]] - maxPr - log(Z));
	    numTerms += it->weight;
	}
    }

    // regularization
    double nodeWeight = 0.0;
    for (int i = 0; i < _weights->numSingletonParameters(); i++) {
	nodeWeight += x[i] * x[i];
    }
    double edgeWeight = 0.0;
    for (int i = _weights->numSingletonParameters(); i < (int)_n; i++) {
	edgeWeight += x[i] * x[i];
    }
    negLogLikelihood += 0.5 * numTerms * 
	(lambdaNode * nodeWeight + lambdaEdge * edgeWeight);
    
    return negLogLikelihood;
}

void svlPseudoLikelihoodObjective::gradient(const double *x, double *df)
{
    _weights->fromVector(x);    
    vector<double> pr(_weights->numClasses());
    double numTerms = 0.0;

    svlPairwiseCRFWeights g(*_weights);

    // for each training example
    for (vector<svlPairwiseCRFInstance>::const_iterator it = _instances->begin();
	 it != _instances->end(); ++it) {
	// adjacent edges to each variable
	vector<vector<int> > adjEdges(it->numVariables());
	for (int e = 0; e < (int)it->edges.size(); e++) {
	    adjEdges[it->edges[e].first].push_back(e);
	    adjEdges[it->edges[e].second].push_back(e);
	}
	// for each variable
	for (int n = 0; n < it->numVariables(); n++) {
	    // check for missing value
	    if (it->Yn[n] < 0)
		continue;

	    double maxPr = -numeric_limits<double>::max();
	    for (int y = 0; y < _weights->numClasses(); y++) {
		// singleton term
		pr[y] = _weights->singletonDotProduct(y, it->Xn[n]);
		// pairwise terms
		for (vector<int>::const_iterator e = adjEdges[n].begin();
		     e != adjEdges[n].end(); e++) {
		    int m = it->edges[*e].first;
		    if (m == n) m = it->edges[*e].second;
#if 0
		    if (m < n) continue;
#endif
		    // check for missing values
		    if (it->Yn[m] < 0)
			continue;
		    pr[y] += _weights->pairwiseDotProduct(y, it->Yn[m], it->Xnm[*e]);
		}
		
		// check maximum potential
		if (pr[y] > maxPr)
		    maxPr = pr[y];
	    }
	    
	    // compute partition function
	    double Z = 0.0;
	    for (int y = 0; y < _weights->numClasses(); y++) {
		Z += exp(pr[y] - maxPr);
	    }
	    assert(Z > 0.0);

	    // exponentiate and normalize
	    for (int y = 0; y < _weights->numClasses(); y++) {
		pr[y] = exp(pr[y] - maxPr) / Z;
	    }
	    
	    // accumulate sufficient statistics
	    g.singletonAddWeighted(it->Yn[n], it->Xn[n], -1.0 * it->weight);
	    for (int y = 0; y < _weights->numClasses(); y++) {
		g.singletonAddWeighted(y, it->Xn[n], pr[y] * it->weight);
	    }

	    for (vector<int>::const_iterator e = adjEdges[n].begin(); 
		 e != adjEdges[n].end(); e++) {
		int m = it->edges[*e].first;
		if (m == n) m = it->edges[*e].second;
#if 0
		if (m < n) continue;
#endif
		// check for missing values
		if (it->Yn[m] < 0)
		    continue;
		g.pairwiseAddWeighted(it->Yn[n], it->Yn[m], it->Xnm[*e], -1.0 * it->weight);
		for (int y = 0; y < _weights->numClasses(); y++) {
		    g.pairwiseAddWeighted(y, it->Yn[m], it->Xnm[*e], pr[y] * it->weight);
		}	
	    }

	    numTerms += it->weight;
	}
    }

    // compute gradient
    g.toVector(df);

    // regularization
    for (int i = 0; i < _weights->numSingletonParameters(); i++) {
	df[i] += numTerms * lambdaNode * x[i];
    }
    for (int i = _weights->numSingletonParameters(); i < (int)_n; i++) {
	df[i] += numTerms * lambdaEdge * x[i];
    }
}

double svlPseudoLikelihoodObjective::objectiveAndGradient(const double *x, double *df)
{
    double negLogLikelihood = 0.0;

    _weights->fromVector(x);    
    vector<double> pr(_weights->numClasses());
    double numTerms = 0.0;

    svlPairwiseCRFWeights g(*_weights);

    // for each training example
    for (vector<svlPairwiseCRFInstance>::const_iterator it = _instances->begin();
	 it != _instances->end(); ++it) {
	// adjacent edges to each variable
	vector<vector<int> > adjEdges(it->numVariables());
	for (int e = 0; e < (int)it->edges.size(); e++) {
	    adjEdges[it->edges[e].first].push_back(e);
	    adjEdges[it->edges[e].second].push_back(e);
	}
	// for each variable
	for (int n = 0; n < it->numVariables(); n++) {
	    // check for missing value
	    if (it->Yn[n] < 0)
		continue;

	    double maxPr = -numeric_limits<double>::max();
	    for (int y = 0; y < _weights->numClasses(); y++) {
		// singleton term
		pr[y] = _weights->singletonDotProduct(y, it->Xn[n]);
		// pairwise terms
		for (vector<int>::const_iterator e = adjEdges[n].begin();
		     e != adjEdges[n].end(); e++) {
		    int m = it->edges[*e].first;
		    if (m == n) m = it->edges[*e].second;
#if 0
		    if (m < n) continue;
#endif
		    // check for missing values
		    if (it->Yn[m] < 0)
			continue;
		    pr[y] += _weights->pairwiseDotProduct(y, it->Yn[m], it->Xnm[*e]);
		}
		
		// check maximum potential
		if (pr[y] > maxPr)
		    maxPr = pr[y];
	    }
	    
	    // compute partition function
	    double Z = 0.0;
	    for (int y = 0; y < _weights->numClasses(); y++) {
		Z += exp(pr[y] - maxPr);
	    }
            assert(Z > 0.0);
	    
	    // add to objective
	    negLogLikelihood -= it->weight * (pr[it->Yn[n]] - maxPr - log(Z));
	    numTerms += it->weight;

	    // exponentiate and normalize
	    for (int y = 0; y < _weights->numClasses(); y++) {
		pr[y] = exp(pr[y] - maxPr) / Z;
	    }

	    // accumulate sufficient statistics
	    g.singletonAddWeighted(it->Yn[n], it->Xn[n], -1.0 * it->weight);
	    for (int y = 0; y < _weights->numClasses(); y++) {
		g.singletonAddWeighted(y, it->Xn[n], pr[y] * it->weight);
	    }

	    for (vector<int>::const_iterator e = adjEdges[n].begin();
		 e != adjEdges[n].end(); e++) {
		int m = it->edges[*e].first;
		if (m == n) m = it->edges[*e].second;
#if 0
		if (m < n) continue;
#endif
		// check for missing values
		if (it->Yn[m] < 0)
		    continue;
		g.pairwiseAddWeighted(it->Yn[n], it->Yn[m], it->Xnm[*e], -1.0 * it->weight);
		for (int y = 0; y < _weights->numClasses(); y++) {
		    g.pairwiseAddWeighted(y, it->Yn[m], it->Xnm[*e], pr[y] * it->weight);
		}		
	    }		
	}
    }

    // compute gradient
    g.toVector(df);

    // regularization
    double nodeWeight = 0.0;
    for (int i = 0; i < _weights->numSingletonParameters(); i++) {
	nodeWeight += x[i] * x[i];
	df[i] += numTerms * lambdaNode * x[i];
    }
    double edgeWeight = 0.0;
    for (int i = _weights->numSingletonParameters(); i < (int)_n; i++) {
	edgeWeight += x[i] * x[i];
	df[i] += numTerms * lambdaEdge * x[i];
    }
    negLogLikelihood += 0.5 * numTerms *
	(lambdaNode * nodeWeight + lambdaEdge * edgeWeight);

    return negLogLikelihood;
}

void svlPseudoLikelihoodObjective::monitor(unsigned iter, double objValue)
{
    svlOptimizer::monitor(iter, objValue);
    //_weights->write(cout);
}

