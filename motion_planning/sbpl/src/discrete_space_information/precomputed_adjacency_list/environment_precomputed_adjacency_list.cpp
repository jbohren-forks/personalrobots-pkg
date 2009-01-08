#include <stdlib.h>
#include "environment_precomputed_adjacency_list.h"


AdjacencyListSBPLEnv::AdjacencyListSBPLEnv (vector<StateId> stateIds, vector<AdjacentStateVector> adjacencyVector, StateId startState, StateId goalState)
  : stateIds_(stateIds), adjacencyVector_(adjacencyVector), startStateId_(startState), goalStateId_(goalState)
{
  for (unsigned int i=0; i<stateIds_.size(); i++) {
    stateIndex_[stateIds_[i]] = i;

    // Not sure what this does but planners seem to expect that it is initialized
    while ((int)StateID2IndexMapping.size() <= stateIds_[i]) {
      int* entry = new int[NUMOFINDICES_STATEID2IND];
      for (unsigned int j=0; j<NUMOFINDICES_STATEID2IND; j++) {
        entry[j] = -1;
      }
      StateID2IndexMapping.push_back(entry);
    }
  }

}

void AdjacencyListSBPLEnv::writeToStream (ostream& str)
{
  str << "Adjacency list env with " << stateIds_.size() << " states." << endl;
  str << "Start state: " << startStateId_ << "  Goal state: " << goalStateId_ << endl;
  for (unsigned int i=0; i<stateIds_.size(); i++) {
    str << stateIndex_[stateIds_[i]] << ". State " << stateIds_[i] << ".  Neighbors: ";
    for (unsigned int j=0; j<adjacencyVector_[i].size(); j++) {
      str << "(" << adjacencyVector_[i][j].adjacentStateId << ", " << adjacencyVector_[i][j].cost << ") ";
    }
    str << endl;
  }
}


bool AdjacencyListSBPLEnv::InitializeMDPCfg (MDPConfig *MDPCfg)
{
  MDPCfg->goalstateid = goalStateId_;
  MDPCfg->startstateid = startStateId_;
  return true;
}


// For now, we're only interested in bfs, so just assume a constantly-zero heuristic
int AdjacencyListSBPLEnv::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  return 0;
}


int AdjacencyListSBPLEnv::GetGoalHeuristic(int stateID)
{
  return 0;
}

int AdjacencyListSBPLEnv::GetStartHeuristic(int stateID)
{
  return 0;
}

void AdjacencyListSBPLEnv::PrintState(int stateID, bool bVerbose, FILE* fOut)
{
  // Note we're ignoring the fOut argument
  cout << "State " << stateID << endl;
}

void AdjacencyListSBPLEnv::PrintEnv_Config(FILE* fOut)
{
  // Note we're ignoring the fOut argument
  cout << "Adjacency list env" << endl;
}


int AdjacencyListSBPLEnv::SizeofCreatedEnv()
{
  return stateIds_.size();
}


bool AdjacencyListSBPLEnv::InitializeEnv (const char* sEnvFile)
{
  cout << "AdjacencyList initialization currently does nothing.";
  return true;
}



// To implement
void AdjacencyListSBPLEnv::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  // goal state is absorbing
  if (state->StateID == goalStateId_) {
    return;
  }

  int ind=stateIndex_[state->StateID];
  cout << "Getting transitions from state " << state->StateID << " with index " << ind << endl;
  
  for (unsigned int actionIndex=0; actionIndex<adjacencyVector_[ind].size(); actionIndex++) {
    CMDPACTION* action = state->AddAction(actionIndex);
    AdjacentState adj = adjacencyVector_[ind][actionIndex];
    action->AddOutcome(adj.adjacentStateId, adj.cost, 1.0);
    cout << "Added transition to " << adj.adjacentStateId << " with cost " << adj.cost << endl;
  }
}

void AdjacencyListSBPLEnv::SetAllPreds(CMDPSTATE* state)
{
  // Apparently this is not always necessary
  cout << "Error: SetAllPreds not implemented for adjacency list";
  exit(1);
}

void AdjacencyListSBPLEnv::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
  SuccIDV->clear();
  CostV->clear();

  if (SourceStateID == goalStateId_) {
    return;
  }

  int ind=stateIndex_[SourceStateID];
  cout << "Getting transitions from state " << SourceStateID << " with index " << ind << endl;
  
  for (unsigned int actionIndex=0; actionIndex<adjacencyVector_[ind].size(); actionIndex++) {
    AdjacentState adj = adjacencyVector_[ind][actionIndex];
    SuccIDV->push_back(adj.adjacentStateId);
    CostV->push_back(adj.cost);
    cout << "Added transition to " << adj.adjacentStateId << " with cost " << adj.cost << endl;
  }
  

}

void AdjacencyListSBPLEnv::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
  cout << "Error: GetPreds not currently implemented for adjacency list";
  exit(1);
}

