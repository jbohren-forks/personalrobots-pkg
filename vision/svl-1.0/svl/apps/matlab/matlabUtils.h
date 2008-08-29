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
** FILENAME:    matlabUtils.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Matlab utilities.
**
*****************************************************************************/

#pragma once

#include <cstdlib>
#include <cassert>
#include <string>
#include <map>

// Matlab
#include "mex.h"
#include "matrix.h"

// STAIR Vision Library
#include "svlBase.h"
#include "svlPGM.h"

using namespace std;

void mexMsgTxt(const char *message) { mexPrintf("%s\n", message); }
svlGeneralCRFInstance parseCRFInstance(const mxArray *instance,
    int instanceIndx = 0);svlGeneralCRFModel parseCRFModel(const mxArray *model);
svlFactorTemplate parseCRFFactorTemplate(const mxArray *factorTemplate, 
    int templateIndx = 0);
bool parseOptions(const mxArray *optionStruct, map<string, string>& options);
void mxArrayToVector(const mxArray *m, vector<vector<double> > &features);

// Implementation -----------------------------------------------------------

// TODO: work out how to compile these not inline

// Parses Matlab structure into an svlGeneralCRFInstance object. The structure
// should have fields:
//   s.cards, s.values (optional)
//   s.cliques(m).Cm, s.cliques(m).Xm, s.cliques(m).Tm
svlGeneralCRFInstance parseCRFInstance(const mxArray *instance, int instanceIndx)
{
    svlGeneralCRFInstance crfInstance;
    mxAssert(mxIsStruct(instance), 
        "instance must be a structure with fields 'cards', 'values' and 'cliques'");

    // read nodes (cardinality and values)
    mxArray *cards = mxGetField(instance, instanceIndx, "cards");
    mxAssert(cards != NULL, "field 'cards' missing");
    
    if (mxIsEmpty(cards)) {
        return crfInstance;
    }

    int N = mxGetNumberOfElements(cards);
    crfInstance.Yn.resize(N, -1);
    crfInstance.Kn.resize(N, 0);

    mxArray *values = mxGetField(instance, instanceIndx, "values");
    if ((values != NULL) && (!mxIsEmpty(values))) {
        mxAssert(mxGetNumberOfElements(values) == N, 
            "'values' must be the same size as 'cards'");
    }
    for (int n = 0; n < N; n++) {
        crfInstance.Kn[n] = (int)mxGetPr(cards)[n];
        mxAssert(crfInstance.Kn[n] > 1, "'cards' must all be greater than 1");
        if ((values != NULL) && (!mxIsEmpty(values))) {
            crfInstance.Yn[n] = (int)mxGetPr(values)[n];
        }
    }

    // read cliques (vars, features and templates)
    mxArray *cliques = mxGetField(instance, instanceIndx, "cliques");
    mxAssert(cliques != NULL, "field 'cliques' missing");

    int M = mxGetNumberOfElements(cliques);
    crfInstance.Cm.resize(M);
    crfInstance.Xm.resize(M);
    crfInstance.Tm.resize(M, -1);

    for (int m = 0; m < M; m++) {
        mxArray *Cm = mxGetField(cliques, m, "Cm");
        mxAssert(Cm != NULL, "field 'cliques(m).Cm' missing");
        mxArray *Xm = mxGetField(cliques, m, "Xm");
        mxAssert(Xm != NULL, "field 'cliques(m).Xm' missing");
        mxArray *Tm = mxGetField(cliques, m, "Tm");
        mxAssert(Tm != NULL, "field 'cliques(m).Tm' missing");
    
        int P = mxGetNumberOfElements(Cm);
        crfInstance.Cm[m].resize(P, -1);
        for (int i = 0; i < P; i++) {
            crfInstance.Cm[m][i] = (int)mxGetPr(Cm)[i];
        }

        int Q = mxGetNumberOfElements(Xm);
        crfInstance.Xm[m].resize(Q);
        for (int i = 0; i < Q; i++) {
            crfInstance.Xm[m][i] = mxGetPr(Xm)[i];
        }

        mxAssert(mxGetNumberOfElements(Tm) == 1, "field 'cliques(m).Tm' must be a scalar");           
        crfInstance.Tm[m] = (int)mxGetScalar(Tm);
    }   

    SVL_ASSERT_MSG(crfInstance.checkInstanceData(), "instance(" << instanceIndx + 1 
        << ") is invalid");
    return crfInstance;
}

// Parses Matlab structure into an svlGeneralCRFModel object. The structure
// should have fields:
//   s.weights, s.templates(n).cards, 
//   s.templates(n).entries(m).xi, s.templates(n).entries(m).wi

svlGeneralCRFModel parseCRFModel(const mxArray *model)
{
    svlGeneralCRFModel crfModel;
    mxAssert(mxIsStruct(model), "invalid crf model");
    
    // extract weights
    mxArray *weights = mxGetField(model, 0, "weights");
    mxAssert(weights != NULL, "field 'weights' missing");

    crfModel.setNumWeights(mxGetNumberOfElements(weights));
    for (int i = 0; i < crfModel.numWeights(); i++) {
        crfModel[i] = mxGetPr(weights)[i];
    }

    // extract factor templates
    mxArray *factorTemplates = mxGetField(model, 0, "templates");
    mxAssert(factorTemplates != NULL, "field 'templates' missing");
    
    for (int i = 0; i < mxGetNumberOfElements(factorTemplates); i++) {        
        svlFactorTemplate phi = parseCRFFactorTemplate(factorTemplates, i);
        crfModel.addTemplate(phi); 
    }    

    return crfModel;
}

// Parses a Matlab structure into an svlFactorTemplate object. The structure
// should have fields:
//   s.cards, s.entries(n).xi and s.entries(n).wi
// where n = 1, ..., prod(s.cards).
svlFactorTemplate parseCRFFactorTemplate(const mxArray *factorTemplate,
    int templateIndx)
{
    svlFactorTemplate phi;
    mxAssert(mxIsStruct(factorTemplate), "invalid factor template");

    // add cardinalities
    mxArray *cards = mxGetField(factorTemplate, templateIndx, "cards");
    mxAssert(cards != NULL, "field 'cards' missing");
    
    int N = mxGetNumberOfElements(cards);
    for (int n = 0; n < N; n++) {
        phi.addVariable((int)mxGetPr(cards)[n]);
    }

    // add feature mapping
    mxArray *entries = mxGetField(factorTemplate, templateIndx, "entries");
    mxAssert(entries != NULL, "field 'entries' missing");
    
    int M = mxGetNumberOfElements(entries);
    mxAssert(M == phi.size(), "incorrect number of factor entries");
    for (int m = 0; m < M; m++) {
        mxArray *xi = mxGetField(entries, m, "xi");
        mxAssert(xi != NULL, "field 'entries(m).xi' missing");
        mxArray *wi = mxGetField(entries, m, "wi");
        mxAssert(wi != NULL, "field 'entries(m).wi' missing");
        mxAssert(mxGetNumberOfElements(xi) == mxGetNumberOfElements(wi),
            "mismatch between 'entries(m).xi' and 'entries(m).wi'");

        phi[m].reserve(mxGetNumberOfElements(xi));
        for (int i = 0; i < mxGetNumberOfElements(xi); i++) {
            phi[m].push_back(make_pair((int)mxGetPr(wi)[i], (int)mxGetPr(xi)[i]));
        }        
    }
    
    return phi;
}

// Parses Matlab structure of options. Assumes that each field is
// either a scalar or a string. Returns true if any options were set.
bool parseOptions(const mxArray *optionStruct, map<string, string>& options)
{
    if ((optionStruct == NULL) || (mxIsEmpty(optionStruct))) {
        return false;
    }

    mxAssert(mxIsStruct(optionStruct), "invalid options");
        
    int N = mxGetNumberOfFields(optionStruct);
    for (int i = 0; i < N; i++) {
        const char *name = mxGetFieldNameByNumber(optionStruct, i);
        mxArray *value = mxGetFieldByNumber(optionStruct, 0, i);
        if ((value == NULL) || (mxIsEmpty(value))) {
            continue;
        }
        if (mxIsChar(value)) {
            char *v = mxArrayToString(value);
            options[string(name)] = string(v);
            mxFree(v);
        } else if (mxIsNumeric(value)) {
            if (mxGetNumberOfElements(value) == 1) {
                options[string(name)] = toString(mxGetScalar(value));
            } else {
                string v;
                for (int i = 0; i < mxGetNumberOfElements(value); i++) {
                    if (i > 0) v = v + string(" ");
                    v = v + toString(mxGetPr(value)[i]);
                }
                options[string(name)] = v;
            }
        } else {
            mexErrMsgTxt("invalid option value");
        }
    }

    return true;
}

// Converts a Matlab matrix to an stl vector
void mxArrayToVector(const mxArray *m, vector<vector<double> > &features) 
{     
    int nRows = mxGetM(m);  // i.e., number of rows
    int nCols = mxGetN(m);
    features.resize(nRows);
    for (int i=0; i < nRows; i++) {
        //mexPrintf("Cell %d = %f\n",i,mxGetPr(prhs[0])[i]);
        vector<double> v(nCols);
        for (int j=0; j < nCols; j++) {
            v[j] = mxGetPr(m)[j*nRows + i];
        }
        features[i] = v;
    }     
}
