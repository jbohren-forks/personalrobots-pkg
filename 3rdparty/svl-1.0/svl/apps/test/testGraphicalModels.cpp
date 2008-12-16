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
** FILENAME:    testGraphicalModels.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Regression tests for graphical models classes.
**
*****************************************************************************/

#include <cstdlib>
#include <cstring>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

#include "svlBase.h"
#include "svlPGM.h"

using namespace std;

// globals ------------------------------------------------------------------

bool gbQuiet = false;
int gMaxIterations = 1000;

// prototypes ---------------------------------------------------------------

void testFactors();
void testSpeed(int speedTestLoops, int speedTestCard);
void testClusterGraph(const char *clusterGraphFile, const char *inferenceType);
void testFactorTemplates();
void testGeneralCRF(const char *modelName, const char *instanceName);
void testSudokuCRF(const char *instanceName);

// main ---------------------------------------------------------------------

void usage()
{
    cerr << SVL_USAGE_HEADER << "\n"; 
    cerr << "USAGE: ./testGraphicalModels [OPTIONS]\n";
    cerr << "OPTIONS:\n"
	 << "  -basic        :: do basic factor tests\n"
	 << "  -s <n> <m>    :: speed test loops (n) and factor size (m)\n"
	 << "  -c <filename> :: calibrate cluster graph <filename>\n"
	 << "  -inf <type>   :: inference type (SUMPROD (default), SUMPRODDIV,\n"
	 << "                   MAXPROD, MAXPRODDIV, LOGMAXPROD, LOGMAXPRODDIV,\n"
         << "                   ASYNCSUMPROD, ASYNCSUMPRODDIV, ASYNCMAXPROD, ASYNCMAXPRODDIV,\n"
         << "                   ASYNCLOGMAXPROD, ASYNCLOGMAXPRODDIV, RBP, GEMPLP)\n"
         << "  -maxIters <n> :: maximum number of inference iterations\n"
         << "  -noIndexCache :: turn off index caching for inference\n"
	 << "  -templates    :: test factor templates\n"
	 << "  -crf <m> <i>  :: test CRF model on model <m> and instance <i>\n"
	 << "  -sudoku <i>   :: test CRF model on sudoku puzzle\n"
         << "  -quiet        :: don't print verbose messages\n"
	 << endl;
}

int main(int argc, char *argv[])
{
    svlCodeProfiler::enabled = true;
    svlLogger::setLogLevel(SVL_LOG_VERBOSE);

    bool bBasicTests = false;

    int speedTestLoops = 0;
    int speedTestCard = 2;

    const char *clusterGraphFile = NULL;
    const char *inferenceType = "SUMPROD";

    bool bTestFactorTemplates = false;

    const char *crfModelName = NULL;
    const char *crfInstanceName = NULL;

    const char *sudokuName = NULL;

    char **args = argv + 1;
    while (--argc > 0) {
	if (!strcmp(*args, "-basic")) {
	    bBasicTests = true;
        } else if (!strcmp(*args, "-s")) {
	    speedTestLoops = atoi(*(++args)); argc--;
	    speedTestCard = atoi(*(++args)); argc--;
	} else if (!strcmp(*args, "-c")) {
	    clusterGraphFile = *(++args); argc--;
	} else if (!strcmp(*args, "-inf")) {
	    inferenceType = *(++args); argc--;
        } else if (!strcmp(*args, "-noIndexCache")) {
            svlFactorOperation::CACHE_INDEX_MAPPING = false;
	} else if (!strcmp(*args, "-templates")) {
	    bTestFactorTemplates = true;
	} else if (!strcmp(*args, "-crf")) {
	    crfModelName = *(++args);
	    crfInstanceName = *(++args);
	    argc -= 2;
        } else if (!strcmp(*args, "-maxIters")) {
            gMaxIterations = atoi(*(++args));
            argc -= 1;
	} else if (!strcmp(*args, "-sudoku")) {
	    sudokuName = *(++args); argc--;
        } else if (!strcmp(*args, "-quiet")) {
            gbQuiet = true;
	} else {
	    cerr << "ERROR: unrecognized option " << *args << endl;
	    usage();
	    return -1;
	}
	args++;
    }

    if (bBasicTests) {
	cerr << "---- BASIC FACTOR OPERATIONS TEST ----" << endl;
	testFactors();
    }

    if (speedTestLoops > 0) {
	cerr << "---- SPEED TEST ----" << endl;
	testSpeed(speedTestLoops, speedTestCard);
    }

    if (clusterGraphFile != NULL) {
	cerr << "---- CLUSTER GRAPH TEST ----" << endl;
	testClusterGraph(clusterGraphFile, inferenceType);
    }

    if (bTestFactorTemplates) {
	cerr << "---- FACTOR TEMPLATE TEST ----" << endl;
	testFactorTemplates();
    }

    if (crfModelName != NULL) {
	cerr << "---- CRF MODEL TEST ----" << endl;
	testGeneralCRF(crfModelName, crfInstanceName);
    }

    if (sudokuName != NULL) {
	cerr << "---- SUDOKU CRF TEST ----" << endl;
	cerr << "(not the best way to solve sudoku puzzles)" << endl;
	testSudokuCRF(sudokuName);
    }

    svlCodeProfiler::print(cerr);
    return 0;
}

// tests ------------------------------------------------------------------

// These tests are from Daphne Koller and Nir Friedman's text book,
// "Structured Probabilistic Models". Yet to be published. From the
// section on variable elimination.    
void testFactors()
{
    svlFactor phi1;
    svlFactor phi2;
    
    // define factors
    phi1.addVariable(1, 2);  // 'b'
    phi1.addVariable(0, 3);  // 'a'
    phi2.addVariable(2, 2);  // 'c'
    phi2.addVariable(1, 2);  // 'b'

    // populate directly, but could also use use:
    //  phi1[phi1.indexOf(0, <a>, phi1.indexOf(1, <b>))] = <x>;
    phi1[0] = 0.5; phi1[1] = 0.8; phi1[2] = 0.1;
    phi1[3] = 0.0; phi1[4] = 0.3; phi1[5] = 0.9;
    phi1.write(cout);
    
    phi2[0] = 0.5; phi2[1] = 0.7;
    phi2[2] = 0.1; phi2[3] = 0.2;
    phi2.write(cout);
    
    // multiply factors together
    svlFactor phi3;
    svlFactorProductOp op1(&phi3, &phi1, &phi2);
    op1.execute();
    phi3.write(cout);

    // marginalize b
    svlFactor phi4;
    svlFactorMarginalizeOp op2(&phi4, &phi3, 1);
    op2.execute();
    phi4.write(cout);
    
#if 0
    // check results
    const double EXPECTED_RESULTS[3][2] = {
        {0.33, 0.51}, {0.05, 0.07}, {0.24, 0.39}
    };
    for (int a = 0; a < 3; a++) {
        for (int c = 0; c < 2; c++) {
            if (fabs(phi2[phi2.indexOf(0, a, phi2.indexOf(2, c))] - EXPECTED_RESULTS[a][c]) > 1.0e-16) {
                cerr << "ERROR: term mismatch " << phi2[phi2.indexOf(0, a, phi2.indexOf(2, c))]
                    << " != " << EXPECTED_RESULTS[a][c] << endl;
		assert(false);
            }
        }
    }
#endif    
}

void testSpeed(int speedTestLoops, int speedTestCard)
{    
    int h = svlCodeProfiler::getHandle("svlFactor speed test");
    svlFactor tmpA, tmpB, tmpC, tmpD;
    tmpA.addVariable(0, speedTestCard);
    tmpA.addVariable(1, speedTestCard);
    tmpB.addVariable(1, speedTestCard);
    tmpB.addVariable(2, speedTestCard);
    svlFactorProductOp tmpOp1(&tmpC, &tmpA, &tmpB);
    svlFactorMarginalizeOp tmpOp2(&tmpD, &tmpC, 1);
    for (int i = 0; i < speedTestLoops; i++) {
	svlCodeProfiler::tic(h);
	tmpOp1.execute();
	tmpOp2.execute();
	svlCodeProfiler::toc(h);
    }
}

void testClusterGraph(const char *clusterGraphFile, const char *inferenceType)
{
    svlClusterGraph network;

    svlCodeProfiler::tic(svlCodeProfiler::getHandle("read cluster graph"));
    if (!network.read(clusterGraphFile)) {
	cerr << "ERROR: could not open " << clusterGraphFile << endl;
	return;
    }
    svlCodeProfiler::toc(svlCodeProfiler::getHandle("read cluster graph"));

    if (!gbQuiet) network.write(cout);
	
    string inferenceStr = string("inference ") + string(inferenceType);
    svlCodeProfiler::tic(svlCodeProfiler::getHandle(inferenceStr.c_str()));
    svlMessagePassingInference infObject(network);
    if (!strcasecmp(inferenceType, "SUMPROD")) {
	infObject.inference(SVL_MP_SUMPROD, gMaxIterations);
    } else if (!strcasecmp(inferenceType, "SUMPRODDIV")) {
	infObject.inference(SVL_MP_SUMPRODDIV, gMaxIterations);
    } else if (!strcasecmp(inferenceType, "MAXPROD")) {
 	infObject.inference(SVL_MP_MAXPROD, gMaxIterations);
    } else if (!strcasecmp(inferenceType, "MAXPRODDIV")) {
 	infObject.inference(SVL_MP_MAXPRODDIV, gMaxIterations);
    } else if (!strcasecmp(inferenceType, "LOGMAXPROD")) {
 	infObject.inference(SVL_MP_LOGMAXPROD, gMaxIterations);
    } else if (!strcasecmp(inferenceType, "LOGMAXPRODDIV")) {
 	infObject.inference(SVL_MP_LOGMAXPRODDIV, gMaxIterations);
    } else if (!strcasecmp(inferenceType, "ASYNCSUMPROD")) {
	infObject.inference(SVL_MP_ASYNCSUMPROD, gMaxIterations);
    } else if (!strcasecmp(inferenceType, "ASYNCSUMPRODDIV")) {
	infObject.inference(SVL_MP_ASYNCSUMPRODDIV, gMaxIterations);
    } else if (!strcasecmp(inferenceType, "ASYNCMAXPROD")) {
	infObject.inference(SVL_MP_ASYNCMAXPROD, gMaxIterations);
    } else if (!strcasecmp(inferenceType, "ASYNCMAXPRODDIV")) {
	infObject.inference(SVL_MP_ASYNCMAXPRODDIV, gMaxIterations);
    } else if (!strcasecmp(inferenceType, "ASYNCLOGMAXPROD")) {
	infObject.inference(SVL_MP_ASYNCLOGMAXPROD, gMaxIterations);
    } else if (!strcasecmp(inferenceType, "ASYNCLOGMAXPRODDIV")) {
	infObject.inference(SVL_MP_ASYNCLOGMAXPRODDIV, gMaxIterations);
    } else if (!strcasecmp(inferenceType, "RBP")) {
	infObject.inference(SVL_MP_RBP, gMaxIterations);
    } else if (!strcasecmp(inferenceType, "GEMPLP")) {
	infObject.inference(SVL_MP_GEMPLP, gMaxIterations);
    } else {
	assert(false);
	return;
    }
    svlCodeProfiler::toc(svlCodeProfiler::getHandle(inferenceStr.c_str()));

    if (!gbQuiet) {
        for (int i = 0; i < network.numCliques(); i++) {
            infObject[i].write(cout);
        }
    } else {
        vector<int> mapAssignment(network.numVariables(), -1);
        for (int i = 0; i < network.numCliques(); i++) {
            svlFactor phi = infObject[i]; 
            if (phi.numVars() == 1) {
                mapAssignment[phi.variableId(0)] = phi.indexOfMax();
            }
        }

        double energy = 0.0;
        for (int i = 0; i < network.numCliques(); i++) {
            if (network[i].empty())
                continue;
            vector<int> subAssignment = extractSubVector(mapAssignment,
                network[i].vars());            
            energy += log(network[i][network[i].indexOf(subAssignment)]);
        }

        cout << "MAP:    " << toString(mapAssignment) << endl;
        cout << "Energy: " << energy << endl;
    }
}

void testFactorTemplates()
{
    // pairwise potential over ternary variable
    svlFactorTemplate t(3, 2);
    
    // diagonal terms are w_c, off diagonal terms are shared
    for (int Yi = 0; Yi < 3; Yi++) {
	for (int Yj = 0; Yj < 3; Yj++) {
	    int indx = t.indexOf(1, Yj, t.indexOf(0, Yi));
	    t[indx].push_back(make_pair(Yi == Yj ? Yi : 3, -1));
	}
    }

    t.write(cout) << endl;

    // create factor between Y3 and Y5
    vector<int> variables;
    vector<double> weights(4);
    vector<double> features;

    variables.push_back(3);
    variables.push_back(5);
    weights[0] = log(1.0);
    weights[1] = log(0.3);
    weights[2] = log(0.8);
    weights[3] = log(0.2);
    svlFactor phi = t.createFactor(variables, weights, features);

    phi.write(cout) << endl;
}

void testGeneralCRF(const char *modelName, const char *instanceName)
{
    svlGeneralCRFModel model;
    cout << "Reading CRF model..." << endl;
    model.read(modelName);
    cout << "...done" << endl;

    cout << "Reading CRF instance..." << endl;
    svlGeneralCRFInstance instance;
    instance.read(instanceName);
    cout << "...done" << endl;
    
    model.write(cout);
    instance.write(cout);

    model.dumpClusterGraph(cout, instance);

    vector<vector<double> > marginals;
    model.inference(instance, marginals);

    for (unsigned i = 0; i < marginals.size(); i++) {
	cout << "Y_" << i << " :" << toString(marginals[i]) << endl;
    }
}

void testSudokuCRF(const char *instanceName)
{
    int puzzle[9][9];

    ifstream ifs(instanceName);
    assert(!ifs.fail());

    // read puzzle
    for (int i = 0; i < 9; i++) {
	for (int j = 0; j < 9; j++) {
	    ifs >> puzzle[i][j];
	}
    }

    ifs.close();

    // show puzzle
    for (int i = 0; i < 9; i++) {
	if (i % 3 == 0) cout << "\n";
	for (int j = 0; j < 9; j++) {
	    if (j % 3 == 0) cout << " ";
	    cout << " ";
	    if (puzzle[i][j] > 0)
		cout << puzzle[i][j];
	    else cout << "-";
	}
	cout << "\n";
    }
    cout << endl;

    // construct general crf for solving sudoku
    svlGeneralCRFModel sudokuModel(3);
    sudokuModel[0] = 0.0;
    sudokuModel[1] = -1.0;
    sudokuModel[2] = 6.0;

    svlFactorTemplate cliqueConstraints(9, 3);
    vector<int> assignment(3, 0);
    for (int i = 0; i < cliqueConstraints.size(); i++) {
	if ((assignment[0] == assignment[1]) ||
	    (assignment[1] == assignment[2]) ||
	    (assignment[2] == assignment[0])) {
	    cliqueConstraints[i].push_back(make_pair(1, -1));
	} else {
	    cliqueConstraints[i].push_back(make_pair(0, -1));
	}
	successor(assignment, 9);
    }
    
    svlFactorTemplate observationConstraints(9, 1);
    for (int i = 0; i < observationConstraints.size(); i++) {
	for (int j = 0; j < 9; j++) {
	    observationConstraints[i].push_back(make_pair(i == j ? 2 : 0, j));
	}
    }

    sudokuModel.addTemplate(cliqueConstraints);
    sudokuModel.addTemplate(observationConstraints);

    // construct specific instance
    svlGeneralCRFInstance instance;
    instance.Yn.resize(81, -1);
    instance.Kn.resize(81, 9);
    
    // observation cliques
    for (int i = 0; i < 9; i++) {
	for (int j = 0; j < 9; j++) {
	    if (puzzle[i][j] > 0) {
		int varId = 9 * i + j;
		instance.Cm.push_back(vector<int>(1, varId));
		instance.Xm.push_back(vector<double>(9, 0.0));
		instance.Xm.back()[puzzle[i][j] - 1] = 1.0;
		instance.Tm.push_back(1);
	    }
	}
    }

    // constraint cliques
    for (int i = 0; i < 81; i++) {
	int iRow = i / 9;
	int iCol = i % 9;
	int iBlock = 3 * (int)(iRow / 3) + (int)(iCol / 3);

	for (int j = i + 1; j < 81; j++) {
	    int jRow = j / 9;
	    int jCol = j % 9;
	    int jBlock = 3 * (int)(jRow / 3) + (int)(jCol / 3);

	    for (int k = j + 1; k < 81; k++) {
		int kRow = k / 9;
		int kCol = k % 9;
		int kBlock = 3 * (int)(kRow / 3) + (int)(kCol / 3);

		if (((iRow == jRow) && (iRow == kRow)) ||
		    ((iCol == jCol) && (iCol == kCol)) ||
		    ((iBlock == jBlock) && (iBlock == kBlock))) {

		    instance.Cm.push_back(vector<int>(3));
		    instance.Cm.back()[0] = i;
		    instance.Cm.back()[1] = j;
		    instance.Cm.back()[2] = k;
		    instance.Xm.push_back(vector<double>());
		    instance.Tm.push_back(0);
		}
		
	    }
	}
    }

    // run inference
    vector<vector<double> > marginals;
    svlCodeProfiler::tic(svlCodeProfiler::getHandle("sudoku.inference"));
    sudokuModel.inference(instance, marginals, 100);
    svlCodeProfiler::toc(svlCodeProfiler::getHandle("sudoku.inference"));
	
    // display result
    for (int i = 0; i < 9; i++) {
	if (i % 3 == 0) cout << "\n";
	for (int j = 0; j < 9; j++) {
	    if (j % 3 == 0) cout << " ";
	    cout << " " << (argmax<double>(marginals[9 * i + j]) + 1);	    
	}
	cout << "\n";
    }
    cout << endl;
}

