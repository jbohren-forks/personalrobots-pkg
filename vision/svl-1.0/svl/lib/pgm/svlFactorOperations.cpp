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
** FILENAME:    svlFactorOperations.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <stdlib.h>
#include <cassert>
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

#include "svlBase.h"
#include "svlFactor.h"
#include "svlFactorOperations.h"

using namespace std;

// svlFactorOperation class ------------------------------------------------

bool svlFactorOperation::CACHE_INDEX_MAPPING = true;

svlFactorOperation::svlFactorOperation(svlFactor *target) :
    _target(target)
{
    assert(target != NULL);
    // do nothing
}

svlFactorOperation::svlFactorOperation(const svlFactorOperation& op) : 
    _target(op._target)
{
    // do nothing
}

svlFactorOperation::~svlFactorOperation()
{
    // do nothing
}

// svlFactorBinaryOp -------------------------------------------------------

svlFactorBinaryOp::svlFactorBinaryOp(svlFactor *target,
    const svlFactor *A, const svlFactor *B) :
    svlFactorOperation(target), _A(A), _B(B)
{
    assert((A != NULL) && (B != NULL));
    initialize();
}

svlFactorBinaryOp::svlFactorBinaryOp(const svlFactorBinaryOp &phi) :
    svlFactorOperation(phi), _A(phi._A), _B(phi._B)
{
    _mappingA = phi._mappingA;
    _mappingB = phi._mappingB;
}

svlFactorBinaryOp::~svlFactorBinaryOp()
{
    // do nothing
}

void svlFactorBinaryOp::initialize()
{
    // add variables and check domains match
    if (_target->empty()) {
	_target->addVariables(*_A);
	_target->addVariables(*_B);
    } else {
	assert(checkTarget());
    }
    
    // create mappings
    if (CACHE_INDEX_MAPPING) {
        _mappingA = _target->mapFrom(*_A);
        _mappingB = _target->mapFrom(*_B);
    } else {
        _mappingA = _A->strideMapping(_target->vars());
        _mappingB = _B->strideMapping(_target->vars());
    }
}

bool svlFactorBinaryOp::checkTarget()
{
    for (int i = 0; i < _A->numVars(); i++) {
	if (!_target->hasVariable(_A->variableId(i))) {
            SVL_LOG(SVL_LOG_ERROR, "target vars:" <<
                toString(_target->vars()) << "; A vars:" <<
                toString(_A->vars()));
	    return false;
	}
    }

    for (int i = 0; i < _B->numVars(); i++) {
	if (!_target->hasVariable(_B->variableId(i))) {
            SVL_LOG(SVL_LOG_ERROR, "target vars:" <<
                toString(_target->vars()) << "; B vars:" <<
                toString(_B->vars()));
	    return false;
	}
    }

    // TO DO: check reverse direction and domain sizes

    return true;
}

// svlFactorNAryOp --------------------------------------------------------

svlFactorNAryOp::svlFactorNAryOp(svlFactor *target,
    const svlFactor *A, const svlFactor *B) :
    svlFactorOperation(target)
{
    assert((A != NULL) && (B != NULL));

    // add factors to list
    _factors.push_back(A);
    _factors.push_back(B);
    
    initialize();
}

svlFactorNAryOp::svlFactorNAryOp(svlFactor *target,
    const std::vector<const svlFactor *>& A) :
    svlFactorOperation(target)
{
    assert(!A.empty());

    // add factors to list
    for (unsigned i = 0; i < _factors.size(); i++) {
	assert(_factors[i] != NULL);
    }
    _factors.insert(_factors.end(), A.begin(), A.end());

    initialize();
}

svlFactorNAryOp::svlFactorNAryOp(const svlFactorNAryOp &phi) :
    svlFactorOperation(phi)
{
    _factors = phi._factors;
    _mappings = phi._mappings;
}

svlFactorNAryOp::~svlFactorNAryOp()
{
    // do nothing
}

void svlFactorNAryOp::initialize()
{
    // add variables and check domains match
    if (_target->empty()) {
	for (unsigned i = 0; i < _factors.size(); i++) {
	    _target->addVariables(*_factors[i]);
	}
    } else {
	assert(checkTarget());
    }

    // create mappings
    if (CACHE_INDEX_MAPPING) {
        _mappings.resize(_factors.size());
        for (unsigned i = 0; i < _factors.size(); i++) {
            _mappings[i] = _target->mapFrom(*_factors[i]);
        }
    } else {
        _mappings.resize(_factors.size());
        for (unsigned i = 0; i < _factors.size(); i++) {
            _mappings[i] = _factors[i]->strideMapping(_target->vars());
        }
    }
}

bool svlFactorNAryOp::checkTarget()
{
    for (unsigned i = 0; i < _factors.size(); i++) {
	for (int j = 0; j < _factors[i]->numVars(); j++) {
	    if (!_target->hasVariable(_factors[i]->variableId(j))) {
		return false;
	    }
	}
    }

    // TO DO: check reverse direction and domain sizes

    return true;
}

// svlFactorAtomicOp class -------------------------------------------------

svlFactorAtomicOp::svlFactorAtomicOp(svlFactorOperation *op) :
    svlFactorOperation(op->target())
{
    _computations.push_back(op);
}

svlFactorAtomicOp::svlFactorAtomicOp(const vector<svlFactorOperation *>& ops) :
    svlFactorOperation(ops.back()->target()), _computations(ops)
{    
    assert(!_computations.empty());
    for (unsigned i = 0; i < _computations.size(); i++) {
	assert(_computations[i] != NULL);
    }
    _target = _computations.back()->target();
}

svlFactorAtomicOp::~svlFactorAtomicOp()
{
    // delete computation tree
    for (unsigned i = 0; i < _computations.size(); i++) {
	delete _computations[i];
    }
    _computations.clear();
}

void svlFactorAtomicOp::execute()
{
    for (unsigned i = 0; i < _computations.size(); i++) {
	_computations[i]->execute();
    }	    
}

void svlFactorAtomicOp::addOperation(svlFactorOperation *op)
{
    assert(op != NULL);
    _computations.push_back(op);
    _target = _computations.back()->target();
}
    
// svlFactorProductOp ------------------------------------------------------

svlFactorProductOp::svlFactorProductOp(svlFactor *target,
    const svlFactor *A, const svlFactor *B) :
    svlFactorNAryOp(target, A, B)
{
    // do nothing
}

svlFactorProductOp::svlFactorProductOp(svlFactor *target,
    const std::vector<const svlFactor *>& A) :
    svlFactorNAryOp(target, A)
{
    // do nothing
}

svlFactorProductOp::svlFactorProductOp(const svlFactorProductOp &phi) :
    svlFactorNAryOp(phi)
{
    // do nothing
}

svlFactorProductOp::~svlFactorProductOp()
{
    // do nothing
}

void svlFactorProductOp::execute()
{
#ifdef SVL_FACTOR_DEBUG_STATISTICS
    svlFactor::_dbStatsProductCount += _factors.size() - 1;
#endif

    if (CACHE_INDEX_MAPPING) {
        if (_factors[0]->empty()) {
            _target->fill(1.0);
        } else {
            for (int i = 0; i < _target->size(); i++) {
                (*_target)[i] = (*_factors[0])[_mappings[0][i]];
            }
        }
        
        for (int k = 1; k < (int)_factors.size(); k++) {
            if (_factors[k]->empty())
                continue;
            for (int i = 0; i < _target->size(); i++) {
                (*_target)[i] *= (*_factors[k])[_mappings[k][i]];
            }
        }
    } else {
        vector<int> assignment(_target->numVars(), 0);

        if (_factors[0]->empty()) {
            _target->fill(1.0);
        } else {
            int kPhi = 0;
            for (int k = 0; k < _target->size(); k++) {
                assert((kPhi >= 0) && (kPhi < _factors[0]->size()));
                (*_target)[k] = (*_factors[0])[kPhi];
                for (int i = 0; i < _target->numVars(); i++) {
                    assignment[i] += 1;
                    kPhi += _mappings[0][i];
                    if (assignment[i] == _target->cards()[i]) {
                        assignment[i] = 0;
                    } else {
                        break;
                    }
                }
            }
        }

        for (int f = 1; f < (int)_factors.size(); f++) {
            if (_factors[f]->empty())
                continue;

            int kPhi = 0;
            for (int k = 0; k < _target->size(); k++) {
                assert((kPhi >= 0) && (kPhi < _factors[f]->size()));
                (*_target)[k] *= (*_factors[f])[kPhi];
                for (int i = 0; i < _target->numVars(); i++) {
                    assignment[i] += 1;
                    kPhi += _mappings[f][i];
                    if (assignment[i] == _target->cards()[i]) {
                        assignment[i] = 0;
                    } else {
                        break;
                    }
                }
            }
        }
    }
}

// svlFactorDivideOp -------------------------------------------------------

svlFactorDivideOp::svlFactorDivideOp(svlFactor *target,
    const svlFactor *A, const svlFactor *B) :
    svlFactorBinaryOp(target, A, B)
{
    // do nothing
}

svlFactorDivideOp::svlFactorDivideOp(const svlFactorDivideOp &phi) :
    svlFactorBinaryOp(phi)
{
    // do nothing
}

svlFactorDivideOp::~svlFactorDivideOp()
{
    // do nothing
}

void svlFactorDivideOp::execute()
{
#ifdef SVL_FACTOR_DEBUG_STATISTICS
    svlFactor::_dbStatsDivideCount += 1;
#endif
    
    if (CACHE_INDEX_MAPPING) {
        for (int i = 0; i < _target->size(); i++) {
            // check for zero (and pervent zero/zero)
            if ((*_A)[_mappingA[i]] == 0.0) {
                (*_target)[i] = 0.0;
                continue;
            }
            (*_target)[i] = (*_A)[_mappingA[i]] / (*_B)[_mappingB[i]];
        }
    } else {
        vector<int> assignment(_target->numVars(), 0);

        int kA = 0;
        int kB = 0;
        for (int k = 0; k < _target->size(); k++) {
            assert((kA >= 0) && (kA < _A->size()));
            assert((kB >= 0) && (kB < _B->size()));

            // check for zero (and pervent zero/zero)
            if ((*_A)[kA] == 0.0) {
                (*_target)[k] = 0.0;
            } else {
                (*_target)[k] = (*_A)[kA] / (*_B)[kB];
            }

            for (int i = 0; i < _target->numVars(); i++) {
                assignment[i] += 1;
                kA += _mappingA[i];
                kB += _mappingB[i];
                if (assignment[i] == _target->cards()[i]) {
                    assignment[i] = 0;
                } else {
                    break;
                }
            }
        }
    }
}

// svlFactorAdditionOp -----------------------------------------------------

svlFactorAdditionOp::svlFactorAdditionOp(svlFactor *target,
    const svlFactor *A, const svlFactor *B) :
    svlFactorNAryOp(target, A, B)
{
    // do nothing
}

svlFactorAdditionOp::svlFactorAdditionOp(svlFactor *target,
    const std::vector<const svlFactor *>& A) :
    svlFactorNAryOp(target, A)
{
    // do nothing
}

svlFactorAdditionOp::svlFactorAdditionOp(const svlFactorAdditionOp &phi) :
    svlFactorNAryOp(phi)
{
    // do nothing
}

svlFactorAdditionOp::~svlFactorAdditionOp()
{
    // do nothing
}

void svlFactorAdditionOp::execute()
{
#ifdef SVL_FACTOR_DEBUG_STATISTICS
    svlFactor::_dbStatsAdditionCount += _factors.size() - 1;
#endif

    if (CACHE_INDEX_MAPPING) {
        if (_factors[0]->empty()) {
            _target->fill(0.0);
        } else if (_target != _factors[0]) {
            for (int i = 0; i < _target->size(); i++) {
                (*_target)[i] = (*_factors[0])[_mappings[0][i]];
            }
        }
        
        for (int k = 1; k < (int)_factors.size(); k++) {
            if (_factors[k]->empty())
                continue;
            for (int i = 0; i < _target->size(); i++) {
                (*_target)[i] += (*_factors[k])[_mappings[k][i]];
            }
        }
    } else {
        vector<int> assignment(_target->numVars(), 0);

        if (_factors[0]->empty()) {
            _target->fill(0.0);
        } else if (_target != _factors[0]) {
            int kPhi = 0;
            for (int k = 0; k < _target->size(); k++) {
                assert((kPhi >= 0) && (kPhi < _factors[0]->size()));
                // operation
                (*_target)[k] = (*_factors[0])[kPhi];
                // update assignment
                for (int i = 0; i < _target->numVars(); i++) {
                    assignment[i] += 1;
                    kPhi += _mappings[0][i];
                    if (assignment[i] == _target->cards()[i]) {
                        assignment[i] = 0;
                    } else {
                        break;
                    }
                }
            }
        }

        for (int f = 1; f < (int)_factors.size(); f++) {
            if (_factors[f]->empty())
                continue;

            int kPhi = 0;
            for (int k = 0; k < _target->size(); k++) {
                assert((kPhi >= 0) && (kPhi < _factors[f]->size()));
                // operation
                (*_target)[k] += (*_factors[f])[kPhi];
                // update assignment
                for (int i = 0; i < _target->numVars(); i++) {
                    assignment[i] += 1;
                    kPhi += _mappings[f][i];
                    if (assignment[i] == _target->cards()[i]) {
                        assignment[i] = 0;
                    } else {
                        break;
                    }
                }
            }
        }
    }
}

// svlFactorSubtractOp -------------------------------------------------------

svlFactorSubtractOp::svlFactorSubtractOp(svlFactor *target,
    const svlFactor *A, const svlFactor *B) :
    svlFactorBinaryOp(target, A, B)
{
    // do nothing
}

svlFactorSubtractOp::svlFactorSubtractOp(const svlFactorSubtractOp &phi) :
    svlFactorBinaryOp(phi)
{
    // do nothing
}

svlFactorSubtractOp::~svlFactorSubtractOp()
{
    // do nothing
}

void svlFactorSubtractOp::execute()
{
#ifdef SVL_FACTOR_DEBUG_STATISTICS
    svlFactor::_dbStatsSubtractionCount += 1;
#endif
    
    if (CACHE_INDEX_MAPPING) {
        for (int i = 0; i < _target->size(); i++) {
            (*_target)[i] = (*_A)[_mappingA[i]] - (*_B)[_mappingB[i]];
        }
    } else {
        vector<int> assignment(_target->numVars(), 0);

        int kA = 0;
        int kB = 0;
        for (int k = 0; k < _target->size(); k++) {
            assert((kA >= 0) && (kA < _A->size()));
            assert((kB >= 0) && (kB < _B->size()));
            // operation
            (*_target)[k] = (*_A)[kA] - (*_B)[kB];
            // update assignment
            for (int i = 0; i < _target->numVars(); i++) {
                assignment[i] += 1;
                kA += _mappingA[i];
                kB += _mappingB[i];
                if (assignment[i] == _target->cards()[i]) {
                    assignment[i] = 0;
                } else {
                    break;
                }
            }
        }
    }
}

// svlFactorWeightedSumOp ----------------------------------------------------

svlFactorWeightedSumOp::svlFactorWeightedSumOp(svlFactor *target,
    const svlFactor *A, const svlFactor *B, double wA, double wB) :
    svlFactorBinaryOp(target, A, B), _weightA(wA), _weightB(wB)
{
    // do nothing
}

svlFactorWeightedSumOp::svlFactorWeightedSumOp(const svlFactorWeightedSumOp &phi) :
    svlFactorBinaryOp(phi), _weightA(phi._weightA), _weightB(phi._weightB)
{
    // do nothing
}

svlFactorWeightedSumOp::~svlFactorWeightedSumOp()
{
    // do nothing
}

void svlFactorWeightedSumOp::execute()
{
#ifdef SVL_FACTOR_DEBUG_STATISTICS
    svlFactor::_dbStatsAdditionCount += 1;
#endif
    
    if (CACHE_INDEX_MAPPING) {
        if (_A->empty()) {
            for (int i = 0; i < _target->size(); i++) {
                (*_target)[i] = _weightB * (*_B)[_mappingB[i]];
            }
        } else if (_B->empty()) {
            for (int i = 0; i < _target->size(); i++) {
                (*_target)[i] = _weightA * (*_A)[_mappingA[i]];
            }
        } else {
            for (int i = 0; i < _target->size(); i++) {
                (*_target)[i] = _weightA * (*_A)[_mappingA[i]] +
                    _weightB * (*_B)[_mappingB[i]];
            }
        }
    } else {
        vector<int> assignment(_target->numVars(), 0);

        int kA = 0;
        int kB = 0;

        if (_A->empty()) {
            for (int k = 0; k < _target->size(); k++) {
                assert((kB >= 0) && (kB < _B->size()));
                // operation
                (*_target)[k] = _weightB * (*_B)[kB];
                // update assignment
                for (int i = 0; i < _target->numVars(); i++) {
                    assignment[i] += 1;
                    kB += _mappingB[i];
                    if (assignment[i] == _target->cards()[i]) {
                        assignment[i] = 0;
                    } else {
                        break;
                    }
                }
            }
        } else if (_B->empty()) {
            for (int k = 0; k < _target->size(); k++) {
                assert((kA >= 0) && (kA < _A->size()));
                // operation
                (*_target)[k] = _weightA * (*_A)[kA];
                // update assignment
                for (int i = 0; i < _target->numVars(); i++) {
                    assignment[i] += 1;
                    kB += _mappingB[i];
                    if (assignment[i] == _target->cards()[i]) {
                        assignment[i] = 0;
                    } else {
                        break;
                    }
                }
            }
        } else {
            for (int k = 0; k < _target->size(); k++) {
                assert((kA >= 0) && (kA < _A->size()));
                assert((kB >= 0) && (kB < _B->size()));
                // operation
                (*_target)[k] = _weightA * (*_A)[kA] + _weightB * (*_B)[kB];
                // update assignment
                for (int i = 0; i < _target->numVars(); i++) {
                    assignment[i] += 1;
                    kA += _mappingA[i];
                    kB += _mappingB[i];
                    if (assignment[i] == _target->cards()[i]) {
                        assignment[i] = 0;
                    } else {
                        break;
                    }
                }
            }
        }
    }
}

// svlFactorMarginalizeOp class --------------------------------------------

svlFactorMarginalizeOp::svlFactorMarginalizeOp(svlFactor *target,
    const svlFactor *A) :
    svlFactorOperation(target), _A(A)
{
    assert(A != NULL);
    assert(checkTarget());

    // create mapping
    if (CACHE_INDEX_MAPPING) {
        _mappingA = _target->mapOnto(*_A);
    } else {
        _mappingA = _target->strideMapping(_A->vars());
    }
}


svlFactorMarginalizeOp::svlFactorMarginalizeOp(svlFactor *target,
    const svlFactor *A, int v) :
    svlFactorOperation(target), _A(A)
{
    assert(A != NULL);
    assert(_A->hasVariable(v));

    // initialize target
    if (_target->empty()) {
	for (int i = 0; i < _A->numVars(); i++) {
	    if (_A->variableId(i) == v)
		continue;
	    _target->addVariable(_A->variableId(i),
		_A->varCardinality(_A->variableId(i)));
	}
    } else {
	assert(checkTarget());
    }

    // create mapping
    if (CACHE_INDEX_MAPPING) {
        _mappingA = _target->mapOnto(*_A);
    } else {
        _mappingA = _target->strideMapping(_A->vars());
    }
}

svlFactorMarginalizeOp::svlFactorMarginalizeOp(svlFactor *target,
    const svlFactor *A, const std::set<int>& v) :
    svlFactorOperation(target), _A(A)
{
    assert(A != NULL);

    // add variables
    if (_target->empty()) {
	for (int i = 0; i < _A->numVars(); i++) {
	    if (v.find(_A->variableId(i)) != v.end())
		continue;
	    _target->addVariable(_A->variableId(i),
		_A->varCardinality(_A->variableId(i)));
	}
    } else {
	assert(checkTarget());
    }

    // create mapping
    if (CACHE_INDEX_MAPPING) {
        _mappingA = _target->mapOnto(*_A);
    } else {
        _mappingA = _target->strideMapping(_A->vars());
    }
}

svlFactorMarginalizeOp::svlFactorMarginalizeOp(const svlFactorMarginalizeOp &phi) :
    svlFactorOperation(phi), _A(phi._A)
{
    _mappingA = phi._mappingA;
}

svlFactorMarginalizeOp::~svlFactorMarginalizeOp()
{
    // do nothing
}

void svlFactorMarginalizeOp::execute()
{
#ifdef SVL_FACTOR_DEBUG_STATISTICS
    svlFactor::_dbStatsMarginalCount += 1;
#endif
    _target->fill(0.0);

    if (CACHE_INDEX_MAPPING) {
        for (int i = 0; i < (int)_mappingA.size(); i++) {
            (*_target)[_mappingA[i]] += (*_A)[i];
        }
    } else {
        vector<int> assignment(_A->numVars(), 0);

        int kTarget = 0;
        for (int k = 0; k < _A->size(); k++) {
            assert((kTarget >= 0) && (kTarget < _target->size()));
            // perform operation
            (*_target)[kTarget] += (*_A)[k];
            // increment assignment
            for (int i = 0; i < _A->numVars(); i++) {
                assignment[i] += 1;
                kTarget += _mappingA[i];
                if (assignment[i] == _A->cards()[i]) {
                    assignment[i] = 0;
                } else {
                    break;
                }
            }
        }
    }
}

bool svlFactorMarginalizeOp::checkTarget()
{
    // TO DO
    return true;
}

// svlFactorMaximizeOp class ------------------------------------------------

svlFactorMaximizeOp::svlFactorMaximizeOp(svlFactor *target,
    const svlFactor *A) :
    svlFactorOperation(target), _A(A)
{
    assert(A != NULL);
    assert(checkTarget());

    // create mapping
    if (CACHE_INDEX_MAPPING) {
        _mappingA = _target->mapOnto(*_A);
    } else {
        _mappingA = _target->strideMapping(_A->vars());
    }
}

svlFactorMaximizeOp::svlFactorMaximizeOp(svlFactor *target,
    const svlFactor *A, int v) :
    svlFactorOperation(target), _A(A)
{
    assert(A != NULL);
    assert(_A->hasVariable(v));

    // initialize target
    if (_target->empty()) {
	for (int i = 0; i < _A->numVars(); i++) {
	    if (_A->variableId(i) == v)
		continue;
	    _target->addVariable(_A->variableId(i),
		_A->varCardinality(_A->variableId(i)));
	}
    } else {
	assert(checkTarget());
    }

    // create mapping
    if (CACHE_INDEX_MAPPING) {
        _mappingA = _target->mapOnto(*_A);
    } else {
        _mappingA = _target->strideMapping(_A->vars());
    }
}

svlFactorMaximizeOp::svlFactorMaximizeOp(svlFactor *target,
    const svlFactor *A, const std::set<int>& v) :
    svlFactorOperation(target), _A(A)
{
    assert(A != NULL);

    // add variables
    if (_target->empty()) {
	for (int i = 0; i < _A->numVars(); i++) {
	    if (v.find(_A->variableId(i)) != v.end())
		continue;
	    _target->addVariable(_A->variableId(i),
		_A->varCardinality(_A->variableId(i)));
	}
    } else {
	assert(checkTarget());
    }

    // create mapping
    if (CACHE_INDEX_MAPPING) {
        _mappingA = _target->mapOnto(*_A);
    } else {
        _mappingA = _target->strideMapping(_A->vars());
    }
}

svlFactorMaximizeOp::svlFactorMaximizeOp(const svlFactorMaximizeOp &phi) :
    svlFactorOperation(phi), _A(phi._A)
{
    _mappingA = phi._mappingA;
}

svlFactorMaximizeOp::~svlFactorMaximizeOp()
{
    // do nothing
}

void svlFactorMaximizeOp::execute()
{
#ifdef SVL_FACTOR_DEBUG_STATISTICS
    svlFactor::_dbStatsMaximizeCount += 1;
#endif
    if (_target->empty())
        return;

    _target->fill(-numeric_limits<double>::max());
    if (CACHE_INDEX_MAPPING) {
        for (int i = 0; i < (int)_mappingA.size(); i++) {
            if ((*_target)[_mappingA[i]] < (*_A)[i]) {
                (*_target)[_mappingA[i]] = (*_A)[i];
            }
        }
    } else {
        vector<int> assignment(_A->numVars(), 0);

        int kTarget = 0;
        for (int k = 0; k < _A->size(); k++) {
            assert((kTarget >= 0) && (kTarget < _target->size()));
            // perform operation
            if ((*_target)[kTarget] < (*_A)[k])
                (*_target)[kTarget] = (*_A)[k];
            // increment assignment
            for (int i = 0; i < _A->numVars(); i++) {
                assignment[i] += 1;
                kTarget += _mappingA[i];
                if (assignment[i] == _A->cards()[i]) {
                    assignment[i] = 0;
                } else {
                    break;
                }
            }
        }
    }
}

bool svlFactorMaximizeOp::checkTarget()
{
    // TO DO
    return true;
}

// svlFactorNormalizeOp class ----------------------------------------------

svlFactorNormalizeOp::svlFactorNormalizeOp(svlFactor *target) :
    svlFactorOperation(target)
{
    // do nothing
}

svlFactorNormalizeOp::~svlFactorNormalizeOp()
{
    // do nothing
}

void svlFactorNormalizeOp::execute()
{
#ifdef SVL_FACTOR_DEBUG_STATISTICS
    svlFactor::_dbStatsNormalizeCount += 1;
#endif

    if (_target->empty())
	return;

    double total = 0.0;
    for (int i = 0; i < _target->size(); i++) {
        total += (*_target)[i];
    }
    if (total > 0.0) {
        for (int i = 0; i < _target->size(); i++) {
            (*_target)[i] /= total;
        }
    } else {
#ifdef SVL_FACTOR_DEBUG_STATISTICS
	svlFactor::_dbStatsNormalizeErrors += 1;
#endif
        _target->fill(1.0 / (double)_target->size());
    }
}

bool svlFactorNormalizeOp::checkTarget()
{
    // normalizing into yourself, so always true
    return true;
}

// svlLogFactorNormalizeOp class -------------------------------------------

svlFactorLogNormalizeOp::svlFactorLogNormalizeOp(svlFactor *target) :
    svlFactorOperation(target)
{
    // do nothing
}

svlFactorLogNormalizeOp::~svlFactorLogNormalizeOp()
{
    // do nothing
}

void svlFactorLogNormalizeOp::execute()
{
#ifdef SVL_FACTOR_DEBUG_STATISTICS
    svlFactor::_dbStatsNormalizeCount += 1;
#endif

    if (_target->empty())
	return;

    double maxVal = -numeric_limits<double>::max();
    for (int i = 0; i < _target->size(); i++) {
        if ((*_target)[i] > maxVal)
            maxVal = (*_target)[i];
    }
    for (int i = 0; i < _target->size(); i++) {
        (*_target)[i] -= maxVal;
    }
}

bool svlFactorLogNormalizeOp::checkTarget()
{
    // normalizing into yourself, so always true
    return true;
}
