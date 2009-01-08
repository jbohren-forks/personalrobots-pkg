#ifndef __PRECOMPUTED_ADJACENCY_LIST_H_
#define __PRECOMPUTED_ADJACENCY_LIST_H_

#include <iostream>
#include <vector>
#include <map>
#include <utility>

using namespace std; // Necessary because some of the below includes assume it


#include "../../sbpl/headers.h"




typedef int StateId;
struct AdjacentState
{
  StateId adjacentStateId;
  int cost;
};
typedef vector<AdjacentState> AdjacentStateVector;







class AdjacencyListSBPLEnv : public DiscreteSpaceInformation
{

public:

  AdjacencyListSBPLEnv (vector<StateId> stateIds, vector<AdjacentStateVector> adjacencyVector, StateId startState, StateId goalState);
    
  void writeToStream (ostream& str = cout);

  bool InitializeEnv (const char* sEnvFile);
  bool InitializeMDPCfg (MDPConfig *MDPCfg);
  int  GetFromToHeuristic(int FromStateID, int ToStateID);

  int  GetGoalHeuristic(int stateID);
  int  GetStartHeuristic(int stateID);
  void SetAllActionsandAllOutcomes(CMDPSTATE* state);
  void SetAllPreds(CMDPSTATE* state);
  void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
  void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
  int  SizeofCreatedEnv();
  void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
  void PrintEnv_Config(FILE* fOut);
  


private:

  // Disallow copy and assignment
  AdjacencyListSBPLEnv (const AdjacencyListSBPLEnv&);
  AdjacencyListSBPLEnv& operator= (const AdjacencyListSBPLEnv&);


  // Members
  vector<StateId> stateIds_;
  vector<AdjacentStateVector> adjacencyVector_;
  StateId startStateId_, goalStateId_;
  map<StateId,int> stateIndex_;
  
};


#endif
