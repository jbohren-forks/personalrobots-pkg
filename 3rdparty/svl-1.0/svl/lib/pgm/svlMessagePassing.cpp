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
** FILENAME:    svlMessagePassing.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <stdlib.h>
#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <deque>
#include <queue>
#include <algorithm>

#include "svlBase.h"
#include "svlFactor.h"
#include "svlClusterGraph.h"
#include "svlMessagePassing.h"

using namespace std;

// svlMessagePassingInference Class -----------------------------------------

svlMessagePassingInference::svlMessagePassingInference(svlClusterGraph& graph) :
    _graph(graph), _algorithm(SVL_MP_NONE)
{
    // check running intersection property
    assert(_graph.checkRunIntProp());
}

svlMessagePassingInference::~svlMessagePassingInference()
{
    // free memory used by intermediate factors and computations
    reset();
}

void svlMessagePassingInference::reset()
{
    for (unsigned i = 0; i < _computations.size(); i++) {
	delete _computations[i];
    }
    _computations.clear();

    for (unsigned i = 0; i < _intermediateFactors.size(); i++) {
	delete _intermediateFactors[i];
    }
    _intermediateFactors.clear();

    for (unsigned i = 0; i < _forwardMessages.size(); i++) {
        delete _forwardMessages[i];
        delete _backwardMessages[i];
        delete _oldForwardMessages[i];
        delete _oldBackwardMessages[i];
    }
    _forwardMessages.clear();
    _backwardMessages.clear();
    _oldForwardMessages.clear();
    _oldBackwardMessages.clear();

    for (unsigned i = 0; i < _cliquePotentials.size(); i++) {
        delete _cliquePotentials[i];
    }
    _cliquePotentials.clear();

    _algorithm = SVL_MP_NONE;

    for (unsigned i = 0; i < _sharedStorage.size(); i++) {
        delete _sharedStorage[i];
    }
    _sharedStorage.clear();

    // convergent message passing data
    _lpCliqueEdges.clear();
    _lpSeparatorEdges.clear();
    _lpSeparators.clear();
    _lpEdges.clear();    
}

bool svlMessagePassingInference::inference(svlMessagePassingAlgorithms mpAlgorithm,
    int maxIterations)
{
    // make sure we're running the same algorithm as the computation graph
    if (mpAlgorithm != _algorithm) {
	reset();
    }
    _algorithm = mpAlgorithm;

    // assign initial clique potentials and messages
    switch (_algorithm) {
    case SVL_MP_SUMPROD:
    case SVL_MP_MAXPROD:
    case SVL_MP_SUMPRODDIV:
    case SVL_MP_MAXPRODDIV:
    case SVL_MP_ASYNCSUMPROD:
    case SVL_MP_ASYNCMAXPROD:
    case SVL_MP_ASYNCSUMPRODDIV:
    case SVL_MP_ASYNCMAXPRODDIV:
    case SVL_MP_RBP:
        initializeMessagePassing();
        break;
    case SVL_MP_LOGMAXPROD:
    case SVL_MP_LOGMAXPRODDIV:
    case SVL_MP_ASYNCLOGMAXPROD:
    case SVL_MP_ASYNCLOGMAXPRODDIV:
        initializeLogMessagePassing();
        break;
    case SVL_MP_GEMPLP:
        initializeGeneralizedMPLP();
        break;
    default:
        assert(_algorithm == SVL_MP_NONE);
        break;
    }

    // update old message cache
    if (_oldForwardMessages.empty()) {
	_oldForwardMessages.reserve(_forwardMessages.size());
	_oldBackwardMessages.reserve(_backwardMessages.size());
        for (unsigned i = 0; i < _forwardMessages.size(); i++) {
            _oldForwardMessages.push_back(new svlFactor());
            _oldBackwardMessages.push_back(new svlFactor());
        }
    }

    for (unsigned i = 0; i < _forwardMessages.size(); i++) {
        *_oldForwardMessages[i] = *_forwardMessages[i];
        *_oldBackwardMessages[i] = *_backwardMessages[i];
    }

    // set up computation graph
    if (_computations.empty()) {
        int handle = svlCodeProfiler::getHandle("svlMessagePassingInference::buildComputationGraph");
        svlCodeProfiler::tic(handle);
	switch (_algorithm) {
	case SVL_MP_SUMPROD:
	    buildSumProdComputationGraph();
	    break;
	case SVL_MP_MAXPROD:
	    buildMaxProdComputationGraph();
	    break;
	case SVL_MP_LOGMAXPROD:
	    buildLogMaxProdComputationGraph();
	    break;
	case SVL_MP_SUMPRODDIV:
	    buildSumProdDivComputationGraph();
	    break;
	case SVL_MP_MAXPRODDIV:
	    buildMaxProdDivComputationGraph();
	    break;
	case SVL_MP_LOGMAXPRODDIV:
	    buildLogMaxProdDivComputationGraph();
	    break;
	case SVL_MP_ASYNCSUMPROD:
	    buildAsyncSumProdComputationGraph();
	    break;
	case SVL_MP_ASYNCMAXPROD:
	    buildAsyncSumProdComputationGraph();
	    break;
	case SVL_MP_ASYNCLOGMAXPROD:
	    buildAsyncLogMaxProdComputationGraph();
	    break;
	case SVL_MP_ASYNCSUMPRODDIV:
	    buildAsyncSumProdDivComputationGraph();
	    break;
	case SVL_MP_ASYNCMAXPRODDIV:
	    buildAsyncMaxProdDivComputationGraph();
	    break;
	case SVL_MP_ASYNCLOGMAXPRODDIV:
	    buildAsyncLogMaxProdDivComputationGraph();
	    break;
	case SVL_MP_RBP:
	    buildResidualBPComputationGraph();
	    break;
        case SVL_MP_GEMPLP:
            buildGeneralizedMPLPGraph();
            break;
	default:
	    assert(_algorithm == SVL_MP_NONE);
	    break;
	}
        svlCodeProfiler::toc(handle);
    }

    // assert that all messages and intermediate factors are non-empty
    for (int i = 0; i < (int)_forwardMessages.size(); i++) {
        SVL_ASSERT(!_forwardMessages[i]->empty());
        SVL_ASSERT(!_backwardMessages[i]->empty());
    }
#if 0
    for (int i = 0; i < (int)_intermediateFactors.size(); i++) {
        SVL_ASSERT(!_intermediateFactors[i]->empty());
    }
#endif

    // run inference
    bool bConverged = false;
    switch (_algorithm) {
    case SVL_MP_RBP:
	bConverged = residualBPMessagePassingLoop(maxIterations);
	break;
    default:
	bConverged = messagePassingLoop(maxIterations);
	break;
    }
    
    // compute final clique potentials
    switch (_algorithm) {
    case SVL_MP_SUMPROD:
    case SVL_MP_MAXPROD:
    case SVL_MP_SUMPRODDIV:
    case SVL_MP_MAXPRODDIV:
    case SVL_MP_ASYNCSUMPROD:
    case SVL_MP_ASYNCMAXPROD:
    case SVL_MP_ASYNCSUMPRODDIV:
    case SVL_MP_ASYNCMAXPRODDIV:
    case SVL_MP_RBP:
        finalizeMessagePassing();
        break;
    case SVL_MP_LOGMAXPROD:
    case SVL_MP_LOGMAXPRODDIV:
    case SVL_MP_ASYNCLOGMAXPROD:
    case SVL_MP_ASYNCLOGMAXPRODDIV:
        finalizeLogMessagePassing();
        break;
    case SVL_MP_GEMPLP:
        finalizeGeneralizedMPLP();
        break;
    default:
        assert(_algorithm == SVL_MP_NONE);
        break;
    }

    return bConverged;
}

// message passing initialization and finalization ---------------------------

void svlMessagePassingInference::initializeMessagePassing()
{
    // initialize clique potentials
    if (_cliquePotentials.empty()) {
        _cliquePotentials.resize(_graph.numCliques());
        for (int n = 0; n < _graph.numCliques(); n++) {
            _cliquePotentials[n] = new svlFactor(_graph.getCliquePotential(n));
        }
    } else {
        assert(_cliquePotentials.size() == (unsigned)_graph.numCliques());
        for (int n = 0; n < _graph.numCliques(); n++) {
            *_cliquePotentials[n] = _graph.getCliquePotential(n);
        }
    }

    // set up forward and backward messages
    if (_forwardMessages.empty()) {
	_forwardMessages.reserve(_graph.numEdges());
	_backwardMessages.reserve(_graph.numEdges());
	for (int i = 0; i < _graph.numEdges(); i++) {
	    _forwardMessages.push_back(new svlFactor());
	    const svlClique s = _graph.getSepSet(i);
#if 0
            cerr << toString(s) << " separates " 
                 << toString(_graph.getClique(_graph.getEdge(i).first)) << " and "
                 << toString(_graph.getClique(_graph.getEdge(i).second)) << endl;
#endif
	    for (svlClique::const_iterator j = s.begin(); j != s.end(); ++j) {
		_forwardMessages.back()->addVariable(*j, _graph.getCardinality(*j));
	    }
	    _forwardMessages.back()->fill(1.0);
            _backwardMessages.push_back(new svlFactor(*_forwardMessages.back()));
	}
    } else {
        // reset messages to all ones
	for (unsigned i = 0; i < _forwardMessages.size(); i++) {
            _forwardMessages[i]->fill(1.0);
            _backwardMessages[i]->fill(1.0);
        }
    }
}

void svlMessagePassingInference::initializeLogMessagePassing()
{
    initializeMessagePassing();

    // log potentials
    for (int n = 0; n < _graph.numCliques(); n++) {
        for (int i = 0; i < _cliquePotentials[n]->size(); i++) {
            (*_cliquePotentials[n])[i] = log((*_cliquePotentials[n])[i]);
        }
    }

    // reset messages to all ones
    for (unsigned i = 0; i < _forwardMessages.size(); i++) {
        _forwardMessages[i]->fill(0.0);
        _backwardMessages[i]->fill(0.0);
    }
}

void svlMessagePassingInference::initializeGeneralizedMPLP()
{
    // Get mapping from cliques and separators to edges where an edge is
    // between a clique and a spearator set.
    _lpCliqueEdges.resize(_graph.numCliques());
    _lpSeparatorEdges.clear();
    _lpSeparators.clear();
    _lpEdges.clear();

    for (int i = 0; i < _graph.numCliques(); i++) {
        svlClique c_i = _graph.getClique(i);
        for (int j = i + 1; j < _graph.numCliques(); j++) {
            svlClique c_j = _graph.getClique(j);
            svlClique s_ij;
            set_intersection(c_i.begin(), c_i.end(),
                c_j.begin(), c_j.end(), 
                insert_iterator<svlClique>(s_ij, s_ij.begin()));

            if (s_ij.empty())
                continue;
            
            int k = 0;
            for ( ; k < (int)_lpSeparators.size(); k++) {
                if (_lpSeparators[k] == s_ij)
                    break;
            }

            if (k == (int)_lpSeparators.size()) {
                _lpSeparators.push_back(s_ij);
                _lpSeparatorEdges.push_back(vector<int>());
            }

            if (find(_lpEdges.begin(), _lpEdges.end(), make_pair(i, k)) == _lpEdges.end()) {
                _lpCliqueEdges[i].push_back(_lpEdges.size());
                _lpSeparatorEdges[k].push_back(_lpEdges.size());
                _lpEdges.push_back(make_pair(i, k));
            }

            if (find(_lpEdges.begin(), _lpEdges.end(), make_pair(j, k)) == _lpEdges.end()) {
                _lpCliqueEdges[j].push_back(_lpEdges.size());
                _lpSeparatorEdges[k].push_back(_lpEdges.size());
                _lpEdges.push_back(make_pair(j, k));
            }
        }
    }

#if 0
    cerr << "Clusters:";
    for (int c = 0; c < _graph.numCliques(); c++) {
        cerr << " " << toString(_graph.getClique(c));
    }
    cerr << "\nSeparators:";
    for (int s = 0; s < (int)_lpSeparators.size(); s++) {
        cerr << " " << toString(_lpSeparators[s]);
    }
    cerr << endl;
#endif

    // assign initial clique potentials
    _cliquePotentials.resize(_graph.numCliques());
    for (int n = 0; n < _graph.numCliques(); n++) {
        _cliquePotentials[n] = new svlFactor(_graph.getCliquePotential(n));
        for (int i = 0; i < _cliquePotentials[n]->size(); i++) {
            (*_cliquePotentials[n])[i] = log((*_cliquePotentials[n])[i]);
        }
    }

    // set up forward and backward messages
    _forwardMessages.reserve(_lpEdges.size());
    _backwardMessages.reserve(_forwardMessages.size());
    for (unsigned i = 0; i < _lpEdges.size(); i++) {
        _forwardMessages.push_back(new svlFactor());
        const svlClique s = _lpSeparators[_lpEdges[i].second];
        for (svlClique::const_iterator j = s.begin(); j != s.end(); ++j) {
            _forwardMessages.back()->addVariable(*j, _graph.getCardinality(*j));
        }
        if (_cliquePotentials[_lpEdges[i].first]->empty()) {
            _forwardMessages.back()->fill(0.0);
        } else {
            svlFactorMaximizeOp msgInitOp(_forwardMessages.back(), 
                _cliquePotentials[_lpEdges[i].first]);
            msgInitOp.execute();
            _forwardMessages.back()->scale(1.0 / 
                (double)_lpCliqueEdges[_lpEdges[i].first].size());
        }
    }

    for (unsigned i = 0; i < _lpEdges.size(); i++) {
        int cliqueId = _lpEdges[i].first;
        int separatorId = _lpEdges[i].second;
        _backwardMessages.push_back(new svlFactor());
        _backwardMessages.back()->fill(0.0);
        for (unsigned c = 0; c < _lpSeparatorEdges[separatorId].size(); c++) {
            if (_lpEdges[_lpSeparatorEdges[separatorId][c]].first == cliqueId)
                continue;
            _backwardMessages.back()->add(*_forwardMessages[_lpSeparatorEdges[separatorId][c]]);
        }
    }
    _oldForwardMessages.reserve(_forwardMessages.size());
    _oldBackwardMessages.reserve(_backwardMessages.size());
    for (unsigned i = 0; i < _forwardMessages.size(); i++) {
        _oldForwardMessages.push_back(new svlFactor());
        _oldBackwardMessages.push_back(new svlFactor());
    }

    for (unsigned i = 0; i < _forwardMessages.size(); i++) {
        *_oldForwardMessages[i] = *_forwardMessages[i];
        *_oldBackwardMessages[i] = *_backwardMessages[i];
    }
}


// TODO: don't compute marginals on empty cliques
void svlMessagePassingInference::finalizeMessagePassing()
{
    for (int m = 0; m < _graph.numEdges(); m++) {
	pair<int, int> e = _graph.getEdge(m);
        _cliquePotentials[e.first]->product(*_backwardMessages[m]);
        _cliquePotentials[e.second]->product(*_forwardMessages[m]);
    }
    for (int n = 0; n < _graph.numCliques(); n++) {
        _cliquePotentials[n]->normalize();
    }
}

void svlMessagePassingInference::finalizeLogMessagePassing()
{
    for (int m = 0; m < _graph.numEdges(); m++) {
	pair<int, int> e = _graph.getEdge(m);
        _cliquePotentials[e.first]->add(*_backwardMessages[m]); 
        _cliquePotentials[e.second]->add(*_forwardMessages[m]);        
    }
    for (int n = 0; n < _graph.numCliques(); n++) {
        if (_cliquePotentials[n]->empty())
            continue;
        int maxIndex = _cliquePotentials[n]->indexOfMax();
        double maxValue = (*_cliquePotentials[n])[maxIndex];        
        for (int i = 0; i < _cliquePotentials[n]->size(); i++) {
            (*_cliquePotentials[n])[i] = exp((*_cliquePotentials[n])[i] - maxValue);
        }
        _cliquePotentials[n]->normalize();
    }
}

void svlMessagePassingInference::finalizeGeneralizedMPLP()
{
#if 0
    for (int n = 0; n < _graph.numCliques(); n++) {
        _cliquePotentials[n]->fill(0.0);
    }
#endif
    for (unsigned m = 0; m < _lpEdges.size(); m++) {
        int c = _lpEdges[m].first;
        //_cliquePotentials[c]->add(*_forwardMessages[m]);
        _cliquePotentials[c]->add(*_backwardMessages[m]);
    }
    for (int n = 0; n < _graph.numCliques(); n++) {
        if (_cliquePotentials[n]->empty())
            continue;
        int maxIndex = _cliquePotentials[n]->indexOfMax();
        double maxValue = (*_cliquePotentials[n])[maxIndex];
        for (int i = 0; i < _cliquePotentials[n]->size(); i++) {
            (*_cliquePotentials[n])[i] = exp((*_cliquePotentials[n])[i] - maxValue);
        }
        _cliquePotentials[n]->normalize();
    }
}


// computation graphs --------------------------------------------------------

void svlMessagePassingInference::buildSumProdComputationGraph()
{
    _intermediateFactors.reserve(2 * _graph.numEdges());  
    _sharedStorage.push_back(new svlFactorStorage(0, true));

    for (int m = 0; m < _graph.numEdges(); m++) {
	int fwdIndx = _graph.getEdge(m).first;
	int bckIndx = _graph.getEdge(m).second;
	vector<const svlFactor *> incomingFwdMsgs;
	vector<const svlFactor *> incomingBckMsgs;

	incomingFwdMsgs.push_back(_cliquePotentials[fwdIndx]);
	incomingBckMsgs.push_back(_cliquePotentials[bckIndx]);

	for (int k = 0; k < _graph.numEdges(); k++) {
	    if (k == m) continue;
	    if (_graph.getEdge(k).first == fwdIndx) {
		incomingFwdMsgs.push_back(_oldBackwardMessages[k]);
	    } else if (_graph.getEdge(k).second == fwdIndx) {
		incomingFwdMsgs.push_back(_oldForwardMessages[k]);
	    }
	    if (_graph.getEdge(k).first == bckIndx) {
		incomingBckMsgs.push_back(_oldBackwardMessages[k]);
	    } else if (_graph.getEdge(k).second == bckIndx) {
		incomingBckMsgs.push_back(_oldForwardMessages[k]);
	    }
	}
	
	// forwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorProductOp(_intermediateFactors.back(),
				    incomingFwdMsgs));
	_computations.push_back(new svlFactorMarginalizeOp(_forwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorNormalizeOp(_forwardMessages[m]));

	// backwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorProductOp(_intermediateFactors.back(),
				    incomingBckMsgs));
	_computations.push_back(new svlFactorMarginalizeOp(_backwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorNormalizeOp(_backwardMessages[m]));
    }
}

void svlMessagePassingInference::buildMaxProdComputationGraph()
{
    _intermediateFactors.reserve(2 * _graph.numEdges());
    _sharedStorage.push_back(new svlFactorStorage(0, true));

    for (int m = 0; m < _graph.numEdges(); m++) {
	int fwdIndx = _graph.getEdge(m).first;
	int bckIndx = _graph.getEdge(m).second;
	vector<const svlFactor *> incomingFwdMsgs;
	vector<const svlFactor *> incomingBckMsgs;

	incomingFwdMsgs.push_back(_cliquePotentials[fwdIndx]);
	incomingBckMsgs.push_back(_cliquePotentials[bckIndx]);

	for (int k = 0; k < _graph.numEdges(); k++) {
	    if (k == m) continue;
	    if (_graph.getEdge(k).first == fwdIndx) {
		incomingFwdMsgs.push_back(_oldBackwardMessages[k]);
	    } else if (_graph.getEdge(k).second == fwdIndx) {
		incomingFwdMsgs.push_back(_oldForwardMessages[k]);
	    }
	    if (_graph.getEdge(k).first == bckIndx) {
		incomingBckMsgs.push_back(_oldBackwardMessages[k]);
	    } else if (_graph.getEdge(k).second == bckIndx) {
		incomingBckMsgs.push_back(_oldForwardMessages[k]);
	    }
	}
	
	// forwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorProductOp(_intermediateFactors.back(),
				    incomingFwdMsgs));
	_computations.push_back(new svlFactorMaximizeOp(_forwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorNormalizeOp(_forwardMessages[m]));

	// backwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorProductOp(_intermediateFactors.back(),
				    incomingBckMsgs));
	_computations.push_back(new svlFactorMaximizeOp(_backwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorNormalizeOp(_backwardMessages[m]));
    }
}

void svlMessagePassingInference::buildLogMaxProdComputationGraph()
{
    _intermediateFactors.reserve(2 * _graph.numEdges());
    _sharedStorage.push_back(new svlFactorStorage(0, true));

    for (int m = 0; m < _graph.numEdges(); m++) {
	int fwdIndx = _graph.getEdge(m).first;
	int bckIndx = _graph.getEdge(m).second;
	vector<const svlFactor *> incomingFwdMsgs;
	vector<const svlFactor *> incomingBckMsgs;

	incomingFwdMsgs.push_back(_cliquePotentials[fwdIndx]);
	incomingBckMsgs.push_back(_cliquePotentials[bckIndx]);

	for (int k = 0; k < _graph.numEdges(); k++) {
	    if (k == m) continue;
	    if (_graph.getEdge(k).first == fwdIndx) {
		incomingFwdMsgs.push_back(_oldBackwardMessages[k]);
	    } else if (_graph.getEdge(k).second == fwdIndx) {
		incomingFwdMsgs.push_back(_oldForwardMessages[k]);
	    }
	    if (_graph.getEdge(k).first == bckIndx) {
		incomingBckMsgs.push_back(_oldBackwardMessages[k]);
	    } else if (_graph.getEdge(k).second == bckIndx) {
		incomingBckMsgs.push_back(_oldForwardMessages[k]);
	    }
	}
	
	// forwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorAdditionOp(_intermediateFactors.back(),
				    incomingFwdMsgs));
	_computations.push_back(new svlFactorMaximizeOp(_forwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorLogNormalizeOp(_forwardMessages[m]));

	// backwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorAdditionOp(_intermediateFactors.back(),
				    incomingBckMsgs));
	_computations.push_back(new svlFactorMaximizeOp(_backwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorLogNormalizeOp(_backwardMessages[m]));
    }
}

void svlMessagePassingInference::buildSumProdDivComputationGraph()
{
    _intermediateFactors.reserve(_graph.numCliques() + 2 * _graph.numEdges());

    // incoming message products
    for (int n = 0; n < _graph.numCliques(); n++) {
	vector<const svlFactor *> incomingMessages;
	incomingMessages.push_back(_cliquePotentials[n]);
	for (int m = 0; m < _graph.numEdges(); m++) {
	    if (_graph.getEdge(m).first == n) {
		incomingMessages.push_back(_backwardMessages[m]);
	    }
	    if (_graph.getEdge(m).second == n) {
		incomingMessages.push_back(_forwardMessages[m]);
	    }
	}
	_intermediateFactors.push_back(new svlFactor());
	_computations.push_back(new svlFactorProductOp(_intermediateFactors.back(),
				    incomingMessages));
    }

    // all following intermediate factors share storage
    _sharedStorage.push_back(new svlFactorStorage(0, true));

    // outgoing marginalized messages
    for (int m = 0; m < _graph.numEdges(); m++) {
	// forwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorDivideOp(_intermediateFactors.back(),
				    _intermediateFactors[_graph.getEdge(m).first], 
				    _oldBackwardMessages[m]));
	_computations.push_back(new svlFactorMarginalizeOp(_forwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorNormalizeOp(_forwardMessages[m]));

	// backwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorDivideOp(_intermediateFactors.back(),
				    _intermediateFactors[_graph.getEdge(m).second], 
				    _oldForwardMessages[m]));
	_computations.push_back(new svlFactorMarginalizeOp(_backwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorNormalizeOp(_backwardMessages[m]));
    }
}

void svlMessagePassingInference::buildMaxProdDivComputationGraph()
{
    _intermediateFactors.reserve(_graph.numCliques() + 2 * _graph.numEdges());

    // incoming message products
    for (int n = 0; n < _graph.numCliques(); n++) {
	vector<const svlFactor *> incomingMessages;
	incomingMessages.push_back(_cliquePotentials[n]);
	for (int m = 0; m < _graph.numEdges(); m++) {
	    if (_graph.getEdge(m).first == n) {
		incomingMessages.push_back(_backwardMessages[m]);
	    }
	    if (_graph.getEdge(m).second == n) {
		incomingMessages.push_back(_forwardMessages[m]);
	    }
	}
	_intermediateFactors.push_back(new svlFactor());
	_computations.push_back(new svlFactorProductOp(_intermediateFactors.back(),
				    incomingMessages));
    }

    // all following intermediate factors share storage
    _sharedStorage.push_back(new svlFactorStorage(0, true));

    // outgoing marginalized messages
    for (int m = 0; m < _graph.numEdges(); m++) {
	// forwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorDivideOp(_intermediateFactors.back(),
				    _intermediateFactors[_graph.getEdge(m).first], 
				    _oldBackwardMessages[m]));
	_computations.push_back(new svlFactorMaximizeOp(_forwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorNormalizeOp(_forwardMessages[m]));

	// backwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorDivideOp(_intermediateFactors.back(),
				    _intermediateFactors[_graph.getEdge(m).second], 
				    _oldForwardMessages[m]));
	_computations.push_back(new svlFactorMaximizeOp(_backwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorNormalizeOp(_backwardMessages[m]));
    }
}

void svlMessagePassingInference::buildLogMaxProdDivComputationGraph()
{
    _intermediateFactors.reserve(_graph.numCliques() + 2 * _graph.numEdges());

    // incoming message products
    for (int n = 0; n < _graph.numCliques(); n++) {
	vector<const svlFactor *> incomingMessages;
	incomingMessages.push_back(_cliquePotentials[n]);
	for (int m = 0; m < _graph.numEdges(); m++) {
	    if (_graph.getEdge(m).first == n) {
		incomingMessages.push_back(_backwardMessages[m]);
	    }
	    if (_graph.getEdge(m).second == n) {
		incomingMessages.push_back(_forwardMessages[m]);
	    }
	}
	_intermediateFactors.push_back(new svlFactor());
	_computations.push_back(new svlFactorAdditionOp(_intermediateFactors.back(),
				    incomingMessages));
    }

    // all following intermediate factors share storage
    _sharedStorage.push_back(new svlFactorStorage(0, true));

    // outgoing marginalized messages
    for (int m = 0; m < _graph.numEdges(); m++) {
	// forwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorSubtractOp(_intermediateFactors.back(),
				    _intermediateFactors[_graph.getEdge(m).first], 
				    _oldBackwardMessages[m]));
	_computations.push_back(new svlFactorMaximizeOp(_forwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorLogNormalizeOp(_forwardMessages[m]));

	// backwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorSubtractOp(_intermediateFactors.back(),
				    _intermediateFactors[_graph.getEdge(m).second], 
				    _oldForwardMessages[m]));
	_computations.push_back(new svlFactorMaximizeOp(_backwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorLogNormalizeOp(_backwardMessages[m]));
    }
}

void svlMessagePassingInference::buildAsyncSumProdComputationGraph()
{
    _intermediateFactors.reserve(2 * _graph.numEdges());

    // all intermediate factors share storage
    _sharedStorage.push_back(new svlFactorStorage(0, true));

    for (int m = 0; m < _graph.numEdges(); m++) {
	int fwdIndx = _graph.getEdge(m).first;
	int bckIndx = _graph.getEdge(m).second;
	vector<const svlFactor *> incomingFwdMsgs;
	vector<const svlFactor *> incomingBckMsgs;

	incomingFwdMsgs.push_back(_cliquePotentials[fwdIndx]);
	incomingBckMsgs.push_back(_cliquePotentials[bckIndx]);

	for (int k = 0; k < _graph.numEdges(); k++) {
	    if (k == m) continue;
	    if (_graph.getEdge(k).first == fwdIndx) {
		incomingFwdMsgs.push_back(_backwardMessages[k]);
	    } else if (_graph.getEdge(k).second == fwdIndx) {
		incomingFwdMsgs.push_back(_forwardMessages[k]);
	    }
	    if (_graph.getEdge(k).first == bckIndx) {
		incomingBckMsgs.push_back(_backwardMessages[k]);
	    } else if (_graph.getEdge(k).second == bckIndx) {
		incomingBckMsgs.push_back(_forwardMessages[k]);
	    }
	}
	
	// forwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorProductOp(_intermediateFactors.back(),
				    incomingFwdMsgs));
	_computations.push_back(new svlFactorMarginalizeOp(_forwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorNormalizeOp(_forwardMessages[m]));

	// backwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorProductOp(_intermediateFactors.back(),
				    incomingBckMsgs));
	_computations.push_back(new svlFactorMarginalizeOp(_backwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorNormalizeOp(_backwardMessages[m]));
    }
}

void svlMessagePassingInference::buildAsyncMaxProdComputationGraph()
{
    _intermediateFactors.reserve(2 * _graph.numEdges());

    // all intermediate factors share storage
    _sharedStorage.push_back(new svlFactorStorage(0, true));

    for (int m = 0; m < _graph.numEdges(); m++) {
	int fwdIndx = _graph.getEdge(m).first;
	int bckIndx = _graph.getEdge(m).second;
	vector<const svlFactor *> incomingFwdMsgs;
	vector<const svlFactor *> incomingBckMsgs;

	incomingFwdMsgs.push_back(_cliquePotentials[fwdIndx]);
	incomingBckMsgs.push_back(_cliquePotentials[bckIndx]);

	for (int k = 0; k < _graph.numEdges(); k++) {
	    if (k == m) continue;
	    if (_graph.getEdge(k).first == fwdIndx) {
		incomingFwdMsgs.push_back(_backwardMessages[k]);
	    } else if (_graph.getEdge(k).second == fwdIndx) {
		incomingFwdMsgs.push_back(_forwardMessages[k]);
	    }
	    if (_graph.getEdge(k).first == bckIndx) {
		incomingBckMsgs.push_back(_backwardMessages[k]);
	    } else if (_graph.getEdge(k).second == bckIndx) {
		incomingBckMsgs.push_back(_forwardMessages[k]);
	    }
	}
	
	// forwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorProductOp(_intermediateFactors.back(),
				    incomingFwdMsgs));
	_computations.push_back(new svlFactorMaximizeOp(_forwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorNormalizeOp(_forwardMessages[m]));

	// backwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorProductOp(_intermediateFactors.back(),
				    incomingBckMsgs));
	_computations.push_back(new svlFactorMaximizeOp(_backwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorNormalizeOp(_backwardMessages[m]));
    }
}

void svlMessagePassingInference::buildAsyncLogMaxProdComputationGraph()
{
    _intermediateFactors.reserve(2 * _graph.numEdges());

    // all intermediate factors share storage
    _sharedStorage.push_back(new svlFactorStorage(0, true));

    for (int m = 0; m < _graph.numEdges(); m++) {
	int fwdIndx = _graph.getEdge(m).first;
	int bckIndx = _graph.getEdge(m).second;
	vector<const svlFactor *> incomingFwdMsgs;
	vector<const svlFactor *> incomingBckMsgs;

	incomingFwdMsgs.push_back(_cliquePotentials[fwdIndx]);
	incomingBckMsgs.push_back(_cliquePotentials[bckIndx]);

	for (int k = 0; k < _graph.numEdges(); k++) {
	    if (k == m) continue;
	    if (_graph.getEdge(k).first == fwdIndx) {
		incomingFwdMsgs.push_back(_backwardMessages[k]);
	    } else if (_graph.getEdge(k).second == fwdIndx) {
		incomingFwdMsgs.push_back(_forwardMessages[k]);
	    }
	    if (_graph.getEdge(k).first == bckIndx) {
		incomingBckMsgs.push_back(_backwardMessages[k]);
	    } else if (_graph.getEdge(k).second == bckIndx) {
		incomingBckMsgs.push_back(_forwardMessages[k]);
	    }
	}
	
	// forwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorAdditionOp(_intermediateFactors.back(),
				    incomingFwdMsgs));
	_computations.push_back(new svlFactorMaximizeOp(_forwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorLogNormalizeOp(_forwardMessages[m]));

	// backwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	_computations.push_back(new svlFactorAdditionOp(_intermediateFactors.back(),
				    incomingBckMsgs));
	_computations.push_back(new svlFactorMaximizeOp(_backwardMessages[m],
				    _intermediateFactors.back()));
	_computations.push_back(new svlFactorLogNormalizeOp(_backwardMessages[m]));
    }
}

void svlMessagePassingInference::buildAsyncSumProdDivComputationGraph()
{
    _intermediateFactors.reserve(_graph.numCliques() + 2 * _graph.numEdges());

    // node factors and intermediate factors share storage
    _sharedStorage.push_back(new svlFactorStorage(0, true));
    _sharedStorage.push_back(new svlFactorStorage(0, true));

    // since we're implementing sum-product-divide, we need to
    // send all messages orginating from the same node at once
    for (int n = 0; n < _graph.numCliques(); n++) {
	vector<const svlFactor *> incomingMessages;
	incomingMessages.push_back(_cliquePotentials[n]);
	for (int m = 0; m < _graph.numEdges(); m++) {
	    if (_graph.getEdge(m).first == n) {
		incomingMessages.push_back(_backwardMessages[m]);
	    }
	    if (_graph.getEdge(m).second == n) {
		incomingMessages.push_back(_forwardMessages[m]);
	    }
	}

	// intermediate node factor
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	svlFactor *nodeFactor = _intermediateFactors.back();
	_computations.push_back(new svlFactorProductOp(nodeFactor,
				    incomingMessages));
	// outgoing messages
	for (int m = 0; m < _graph.numEdges(); m++) {
	    if (_graph.getEdge(m).first == n) {
		// forwards
		_intermediateFactors.push_back(new svlFactor(_sharedStorage[1]));
		_computations.push_back(new svlFactorDivideOp(_intermediateFactors.back(),
					    nodeFactor,
					    _backwardMessages[m]));
		_computations.push_back(new svlFactorMarginalizeOp(_forwardMessages[m],
					    _intermediateFactors.back()));
		_computations.push_back(new svlFactorNormalizeOp(_forwardMessages[m]));
	    } else if (_graph.getEdge(m).second == n) {
		// backwards
		_intermediateFactors.push_back(new svlFactor(_sharedStorage[1]));
		_computations.push_back(new svlFactorDivideOp(_intermediateFactors.back(),
					    nodeFactor,
					    _forwardMessages[m]));
		_computations.push_back(new svlFactorMarginalizeOp(_backwardMessages[m],
					    _intermediateFactors.back()));
		_computations.push_back(new svlFactorNormalizeOp(_backwardMessages[m]));
	    }
	}
    }
}

void svlMessagePassingInference::buildAsyncMaxProdDivComputationGraph()
{
    _intermediateFactors.reserve(_graph.numCliques() + 2 * _graph.numEdges());

    // node factors and intermediate factors share storage
    _sharedStorage.push_back(new svlFactorStorage(0, true));
    _sharedStorage.push_back(new svlFactorStorage(0, true));

    // since we're implementing max-product-divide, we need to
    // send all messages orginating from the same node at once
    for (int n = 0; n < _graph.numCliques(); n++) {
	vector<const svlFactor *> incomingMessages;
	incomingMessages.push_back(_cliquePotentials[n]);
	for (int m = 0; m < _graph.numEdges(); m++) {
	    if (_graph.getEdge(m).first == n) {
		incomingMessages.push_back(_backwardMessages[m]);
	    }
	    if (_graph.getEdge(m).second == n) {
		incomingMessages.push_back(_forwardMessages[m]);
	    }
	}

	// intermediate node factor
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	svlFactor *nodeFactor = _intermediateFactors.back();
	_computations.push_back(new svlFactorProductOp(nodeFactor,
				    incomingMessages));
	// outgoing messages
	for (int m = 0; m < _graph.numEdges(); m++) {
	    if (_graph.getEdge(m).first == n) {
		// forwards
		_intermediateFactors.push_back(new svlFactor(_sharedStorage[1]));
		_computations.push_back(new svlFactorDivideOp(_intermediateFactors.back(),
					    nodeFactor,
					    _backwardMessages[m]));
		_computations.push_back(new svlFactorMaximizeOp(_forwardMessages[m],
					    _intermediateFactors.back()));
		_computations.push_back(new svlFactorNormalizeOp(_forwardMessages[m]));
	    } else if (_graph.getEdge(m).second == n) {
		// backwards
		_intermediateFactors.push_back(new svlFactor(_sharedStorage[1]));
		_computations.push_back(new svlFactorDivideOp(_intermediateFactors.back(),
					    nodeFactor,
					    _forwardMessages[m]));
		_computations.push_back(new svlFactorMaximizeOp(_backwardMessages[m],
					    _intermediateFactors.back()));
		_computations.push_back(new svlFactorNormalizeOp(_backwardMessages[m]));
	    }
	}
    }
}

void svlMessagePassingInference::buildAsyncLogMaxProdDivComputationGraph()
{
    _intermediateFactors.reserve(_graph.numCliques() + 2 * _graph.numEdges());

    // node factors and intermediate factors share storage
    _sharedStorage.push_back(new svlFactorStorage(0, true));
    _sharedStorage.push_back(new svlFactorStorage(0, true));

    // since we're implementing max-product-divide, we need to
    // send all messages orginating from the same node at once
    for (int n = 0; n < _graph.numCliques(); n++) {
	vector<const svlFactor *> incomingMessages;
	incomingMessages.push_back(_cliquePotentials[n]);
	for (int m = 0; m < _graph.numEdges(); m++) {
	    if (_graph.getEdge(m).first == n) {
		incomingMessages.push_back(_backwardMessages[m]);
	    }
	    if (_graph.getEdge(m).second == n) {
		incomingMessages.push_back(_forwardMessages[m]);
	    }
	}

	// intermediate node factor
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	svlFactor *nodeFactor = _intermediateFactors.back();
	_computations.push_back(new svlFactorAdditionOp(nodeFactor,
				    incomingMessages));
	// outgoing messages
	for (int m = 0; m < _graph.numEdges(); m++) {
	    if (_graph.getEdge(m).first == n) {
		// forwards
		_intermediateFactors.push_back(new svlFactor(_sharedStorage[1]));
		_computations.push_back(new svlFactorSubtractOp(_intermediateFactors.back(),
					    nodeFactor,
					    _backwardMessages[m]));
		_computations.push_back(new svlFactorMaximizeOp(_forwardMessages[m],
					    _intermediateFactors.back()));
		_computations.push_back(new svlFactorLogNormalizeOp(_forwardMessages[m]));
	    } else if (_graph.getEdge(m).second == n) {
		// backwards
		_intermediateFactors.push_back(new svlFactor(_sharedStorage[1]));
		_computations.push_back(new svlFactorSubtractOp(_intermediateFactors.back(),
					    nodeFactor,
					    _forwardMessages[m]));
		_computations.push_back(new svlFactorMaximizeOp(_backwardMessages[m],
					    _intermediateFactors.back()));
		_computations.push_back(new svlFactorLogNormalizeOp(_backwardMessages[m]));
	    }
	}
    }
}

void svlMessagePassingInference::buildResidualBPComputationGraph()
{
    _intermediateFactors.reserve(2 * _graph.numEdges());

    // all intermediate factors share storage
    _sharedStorage.push_back(new svlFactorStorage(0, true));

    for (int m = 0; m < _graph.numEdges(); m++) {
	int fwdIndx = _graph.getEdge(m).first;
	int bckIndx = _graph.getEdge(m).second;
	vector<const svlFactor *> incomingFwdMsgs;
	vector<const svlFactor *> incomingBckMsgs;

	incomingFwdMsgs.push_back(_cliquePotentials[fwdIndx]);
	incomingBckMsgs.push_back(_cliquePotentials[bckIndx]);

	for (int k = 0; k < _graph.numEdges(); k++) {
	    if (k == m) continue;
	    if (_graph.getEdge(k).first == fwdIndx) {
		incomingFwdMsgs.push_back(_backwardMessages[k]);
	    } else if (_graph.getEdge(k).second == fwdIndx) {
		incomingFwdMsgs.push_back(_forwardMessages[k]);
	    }
	    if (_graph.getEdge(k).first == bckIndx) {
		incomingBckMsgs.push_back(_backwardMessages[k]);
	    } else if (_graph.getEdge(k).second == bckIndx) {
		incomingBckMsgs.push_back(_forwardMessages[k]);
	    }
	}
	
	// forwards
	vector<svlFactorOperation *> atom;
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	atom.push_back(new svlFactorProductOp(_intermediateFactors.back(),
			   incomingFwdMsgs));
	atom.push_back(new svlFactorMarginalizeOp(_forwardMessages[m],
			   _intermediateFactors.back()));
	atom.push_back(new svlFactorNormalizeOp(_forwardMessages[m]));

	_computations.push_back(new svlFactorAtomicOp(atom));
	atom.clear();

	// backwards
	_intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
	atom.push_back(new svlFactorProductOp(_intermediateFactors.back(),
			   incomingBckMsgs));
	atom.push_back(new svlFactorMarginalizeOp(_backwardMessages[m],
			   _intermediateFactors.back()));
	atom.push_back(new svlFactorNormalizeOp(_backwardMessages[m]));
	
	_computations.push_back(new svlFactorAtomicOp(atom));
	atom.clear();
    }
}

void svlMessagePassingInference::buildGeneralizedMPLPGraph()
{
    // convergent message passing algorithm
    // Globerson and Jaakkola, NIPS 2007

    // intermediate factors for \lambda_{s \to c} and \lambda_{c \to s}
    _intermediateFactors.reserve(2 * _lpEdges.size());
    _sharedStorage.push_back(new svlFactorStorage(0, true));
    _sharedStorage.push_back(new svlFactorStorage(0, true));
        
    // since we're implementing coordinate ascent, we need to
    // send all messages orginating from the same node at once
    for (int c = 0; c < _graph.numCliques(); c++) {

        // send all \lambda_{c \to s} messages
        for (int s = 0; s < (int)_lpCliqueEdges[c].size(); s++) {
            int messageId = _lpCliqueEdges[c][s];
            int separatorId = _lpEdges[messageId].second;
            assert(_lpEdges[messageId].first == c);

#if 0
            SVL_LOG(SVL_LOG_MESSAGE, "*** generating messages from c_" << c << " " <<
                toString(_graph.getClique(c)) << " to s_" << separatorId << " " << 
                toString(_lpSeparators[separatorId]));
#endif

            vector<const svlFactor *> incomingMessages;
            incomingMessages.push_back(_cliquePotentials[c]);
            for (int sHat = 0; sHat < (int)_lpCliqueEdges[c].size(); sHat++) {
                if (sHat == s) continue;                
#if 1
                incomingMessages.push_back(_backwardMessages[_lpCliqueEdges[c][sHat]]);
#else
                pair<int, int> csPair = edges[_lpEliqueEdges[c][sHat]]; 
                for (int cHat = 0; cHat < (int)_lpSeparatorEdges[_lpEdges[_lpCliqueEdges[c][sHat]].second].size(); cHat++) {
                    pair<int, int> e = edges[_lpSeparatorEdges[csPair.second][cHat]];
                    if (e.first == c) continue;
                    incomingMessages.push_back(_forwardMessages[_lpSeparatorEdges[csPair.second][cHat]]);
                }
#endif
            }
            
            // sum_{hat{s}} \lambda_{\hat{s} \to c} + \theta_c
            _intermediateFactors.push_back(new svlFactor(_sharedStorage[0]));
            svlFactor *sumFactor = _intermediateFactors.back();
            _computations.push_back(new svlFactorAdditionOp(sumFactor,
                                        incomingMessages));
            // max (sum_{hat{s}} \lambda_{\hat{s} \to c} + \theta_c)
            _intermediateFactors.push_back(new svlFactor(_sharedStorage[1]));
            svlFactor *maxFactor = _intermediateFactors.back();            
            svlClique ns;
            set_difference(_graph.getClique(c).begin(), _graph.getClique(c).end(),
                _lpSeparators[separatorId].begin(), _lpSeparators[separatorId].end(),
                insert_iterator<svlClique>(ns, ns.begin()));                
            _computations.push_back(new svlFactorMaximizeOp(maxFactor,
                sumFactor, ns));
            // weighted sum
            double w = 1.0 / (double)_lpCliqueEdges[c].size();
            _computations.push_back(new svlFactorWeightedSumOp(_forwardMessages[messageId],
                                        _backwardMessages[messageId], maxFactor,
                                        (w - 1.0), w));
        }

        // update all \lambda_{s \to \hat{c}} messages
        for (int s = 0; s < (int)_lpCliqueEdges[c].size(); s++) {
            int messageId = _lpCliqueEdges[c][s];
            int separatorId = _lpEdges[messageId].second;
            for (int cHat = 0; cHat < (int)_lpSeparatorEdges[separatorId].size(); cHat++) {
                int backwardMessageId = _lpSeparatorEdges[separatorId][cHat];
                int cHatCliqueId = _lpEdges[backwardMessageId].first;
                if (cHatCliqueId == c)
                    continue;

                assert((messageId < (int)_forwardMessages.size()) &&
                    (backwardMessageId < (int)_backwardMessages.size()));

#if 0
                SVL_LOG(SVL_LOG_MESSAGE, "*** generating messages from s_" << separatorId << " " <<
                    toString(_lpSeparators[separatorId]) << " to c_" << cHatCliqueId << " " <<
                    toString(_graph.getClique(cHatCliqueId)));
#endif
                
                _computations.push_back(new svlFactorSubtractOp(_backwardMessages[backwardMessageId],
                                            _backwardMessages[backwardMessageId],
                                            _oldForwardMessages[messageId]));
                _computations.push_back(new svlFactorAdditionOp(_backwardMessages[backwardMessageId],
                                            _backwardMessages[backwardMessageId],
                                            _forwardMessages[messageId]));
            }
        }    
    }
}

// message passing loops -----------------------------------------------------

bool svlMessagePassingInference::messagePassingLoop(int maxIterations)
{
    SVL_LOG(SVL_LOG_VERBOSE, "Starting message passing loop...");
    SVL_LOG(SVL_LOG_VERBOSE, "..." << _graph.numVariables() << " variables; "
        << _graph.numCliques() << " cliques; "
        << _graph.numEdges() << " edges");
    bool bConverged = false;
    int nIteration = 0;

    while (!bConverged) {
	int nConverged = 0;
        bConverged = true;
        nIteration += 1;

	for (int i = 0; i < (int)_forwardMessages.size(); i++) {
	    *_oldForwardMessages[i] = *_forwardMessages[i];
	    *_oldBackwardMessages[i] = *_backwardMessages[i];
	}

	for (int i = 0; i < (int)_computations.size(); i++) {
	    _computations[i]->execute();
	}	

	for (int i = 0; i < (int)_forwardMessages.size(); i++) {
            if (_oldForwardMessages[i]->dataCompare(*_forwardMessages[i])) {
                nConverged += 1;
            } else {               
                bConverged = false;   
            }

            if (_oldBackwardMessages[i]->dataCompare(*_backwardMessages[i])) {
                nConverged += 1;   
            } else {
                bConverged = false;   
            }	    
	}

#if 1
        if (_algorithm != SVL_MP_GEMPLP) {
            SVL_LOG(SVL_LOG_VERBOSE, "...iteration " << nIteration << " ("
                << nConverged << " of " << (2 * _forwardMessages.size()) 
                << " messages converged)");
        }else {
            double dualObjective = 0;
            for (int s = 0; s < (int)_lpSeparatorEdges.size(); s++) {           
                svlFactor phi(*_forwardMessages[_lpSeparatorEdges[s][0]]);
                for (int c = 1; c < (int)_lpSeparatorEdges[s].size(); c++) {
                    phi.add(*_forwardMessages[_lpSeparatorEdges[s][c]]);
                }
                assert(phi.numVars() == (int)_lpSeparators[s].size());
                dualObjective += phi[phi.indexOfMax()];
            }
            SVL_LOG(SVL_LOG_VERBOSE, "...iteration " << nIteration 
                << "; dual objective " << dualObjective << " (" 
                << nConverged << " of " << (2 * _forwardMessages.size()) 
                << " messages converged)");
        }
#endif

        if ((nIteration >= maxIterations) && !bConverged) {
            SVL_LOG(SVL_LOG_WARNING, "message passing failed to converge after "
                << nIteration << " iterations (" << nConverged << " of "
                << (2 * _forwardMessages.size()) << " messages converged)"); 
            break;
        }
    }

    if (bConverged) {
	SVL_LOG(SVL_LOG_VERBOSE, "...converged in " << nIteration << " iterations");
    }

    return bConverged;
}

bool svlMessagePassingInference::residualBPMessagePassingLoop(int maxIterations)
{
    // run initial iteration
    for (int i = 0; i < (int)_computations.size(); i++) {
	_computations[i]->execute();
    }	

    // compute residuals
    vector<pair<double, int> > Q(2 * _graph.numEdges());
    for (int i = 0; i < _graph.numEdges(); i++) {
	Q[2 * i].first = 0.0;
	for (int k = 0; k < _forwardMessages[i]->size(); k++) {
	    Q[2 * i].first += fabs((*_forwardMessages[i])[k] - 
		(*_oldForwardMessages[i])[k]);
	}
	Q[2 * i].second = 2 * i;

	Q[2 * i + 1].first = 0.0;
	for (int k = 0; k < _backwardMessages[i]->size(); k++) {
	    Q[2 * i + 1].first += fabs((*_backwardMessages[i])[k] - 
		(*_oldBackwardMessages[i])[k]);
	}
	Q[2 * i + 1].second = 2 * i + 1;
    }
    sort(Q.begin(), Q.end());

    // loop until convergence
    SVL_LOG(SVL_LOG_VERBOSE, "Starting message passing loop...");
    int nIteration = 0;
    bool bConverged;

    while (!(bConverged = Q.back().first < 1.0e-9)) {
        nIteration += 1;
	if (nIteration > maxIterations) {
	    break;
	}

        //cerr << "..." << nIteration << "\r";

	// send message at front of queue
	int edgeIndx = Q.back().second;
	int nodeIndx = -1;
	if (edgeIndx % 2 == 0) {
	    edgeIndx /= 2;
	    *_oldForwardMessages[edgeIndx] = *_forwardMessages[edgeIndx];
	    Q.back().first = 0.0;
	    nodeIndx = _graph.getEdge(edgeIndx).second;
	} else {
	    edgeIndx = (edgeIndx - 1) / 2;
	    *_oldBackwardMessages[edgeIndx] = *_backwardMessages[edgeIndx];
	    Q.back().first = 0.0;
	    nodeIndx = _graph.getEdge(edgeIndx).first;
	}
	
	assert(nodeIndx >= 0);
	
	// update neighbours
	for (int i = 0; i < (int)Q.size(); i++) {
	    edgeIndx = Q[i].second;
	    if (edgeIndx % 2 == 0) {
		edgeIndx /= 2;
		if (_graph.getEdge(edgeIndx).first == nodeIndx) {
		    _computations[Q[i].second]->execute();

		    Q[i].first = 0.0;
		    for (int k = 0; k < _forwardMessages[edgeIndx]->size(); k++) {
			Q[i].first += fabs((*_forwardMessages[edgeIndx])[k] - 
			    (*_oldForwardMessages[edgeIndx])[k]);
		    }
		}
	    } else {
		edgeIndx = (edgeIndx - 1) / 2;
		if (_graph.getEdge(edgeIndx).second == nodeIndx) {
		    _computations[Q[i].second]->execute();

		    Q[i].first = 0.0;
		    for (int k = 0; k < _backwardMessages[edgeIndx]->size(); k++) {
			Q[i].first += fabs((*_backwardMessages[edgeIndx])[k] - 
			    (*_oldBackwardMessages[edgeIndx])[k]);
		    }
		}
	    }
	}

	// resort Q
	sort(Q.begin(), Q.end());
    }

    if (bConverged) {
	SVL_LOG(SVL_LOG_VERBOSE, "...converged in " << nIteration << " iterations");
    } else {
	int nConverged = 0;
	for (int i = 0; i < _graph.numEdges(); i++) {
            if (_oldForwardMessages[i]->dataCompare(*_forwardMessages[i])) {
                nConverged += 1;   
            }
            if (_oldBackwardMessages[i]->dataCompare(*_backwardMessages[i])) {
                nConverged += 1;   
            }	    
	}

	SVL_LOG(SVL_LOG_WARNING, "message passing failed to converge after "
	     << nIteration << " iterations (" << nConverged << " of "
            << (int)(2 * _graph.numEdges()) << " messages converged)");
    }

    return bConverged;
}

