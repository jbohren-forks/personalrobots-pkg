#ifndef __PRECOMPUTED_ADJACENCY_LIST_H_
#define __PRECOMPUTED_ADJACENCY_LIST_H_

#include <iostream>
#include <vector>
#include <map>
#include <utility>
#include <assert.h>
#include <stdlib.h>

using namespace std; // Necessary because some of the below includes assume it
#include "../../sbpl/headers.h"

struct Adjacency
{
  int neighbor;
  int cost;
};
typedef vector<Adjacency> Adjacencies;


/** \brief SBPL Environment represented as an adjacency list graph.
 *
 * \tparam Coords Coords is a type that has operator<< and heuristicDistanceTo (const Coords&) defined on it, and can be used as a key of an STL map.
 */
template <class Coords>
class AdjacencyListSBPLEnv : public DiscreteSpaceInformation
{

public:

  AdjacencyListSBPLEnv ();
  void writeToStream (ostream& str = cout);
  void addPoint (const Coords& c);
  void setCost (const Coords& c1, const Coords& c2, int cost);
  void setStartState (const Coords& c);
  void setGoalState (const Coords& c);
  vector<Coords> findOptimalPath (void);

  // Inherited DiscreteSpaceInformation ops
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

  void resetStateId2IndexMapping(void);

  // Members
  vector<Coords> points_;
  map<Coords,int> pointIds_;
  vector<Adjacencies> adjacencyVector_;
  int startStateId_;
  int goalStateId_;
};






template <class Coords>
AdjacencyListSBPLEnv<Coords>::AdjacencyListSBPLEnv () : startStateId_(-1), goalStateId_(-1)
{}


template <class Coords>
void AdjacencyListSBPLEnv<Coords>::writeToStream (ostream& str)
{
  str << "Adjacency list SBPL Env " << endl;
  for (unsigned int i=0; i<points_.size(); i++) {
    str << i << ". " << points_[i] << ".  Neighbors: ";
    for (unsigned int j=0; j<adjacencyVector_[i].size(); j++) {
      Adjacency adj = adjacencyVector_[i][j];
      str << "[" << points_[adj.neighbor] << " " << adj.cost << "] ";
    }
    str << endl;
  }
}

template <class Coords>
void AdjacencyListSBPLEnv<Coords>::addPoint (const Coords& c)
{
  pointIds_[c] = points_.size();
  points_.push_back(c);
  Adjacencies a;
  adjacencyVector_.push_back(a);

  int* entry = new int[NUMOFINDICES_STATEID2IND];
  for (unsigned int i=0; i<NUMOFINDICES_STATEID2IND; i++) {
    entry[i]=-1;
  }
  StateID2IndexMapping.push_back(entry);
}

template <class Coords>
void AdjacencyListSBPLEnv<Coords>::setCost (const Coords& c1, const Coords& c2, int cost)
{
  typename map<Coords,int>::iterator i1 = pointIds_.find(c1);
  typename map<Coords,int>::iterator i2 = pointIds_.find(c2);

  assert (i1);
  assert (i2);

  int index1=i1->second;
  int index2=i2->second;

  Adjacencies& adj = adjacencyVector_[index1];
  unsigned int i, l=adj.size();
  for (i=0; (i<l) && (adj[i].neighbor!=index2); i++);
  if (i==l) {
    Adjacency a;
    a.neighbor=index2;
    a.cost=cost;
    adj.push_back(a);
  }
  else {
    adj[i].cost = cost;
  }
}

template <class Coords>
void AdjacencyListSBPLEnv<Coords>::setStartState (const Coords& c)
{
  typename map<Coords,int>::iterator i = pointIds_.find(c);
  assert(i);
  startStateId_ = i->second;
}

template <class Coords>
void AdjacencyListSBPLEnv<Coords>::setGoalState (const Coords& c)
{
  typename map<Coords,int>::iterator i = pointIds_.find(c);
  assert(i);
  goalStateId_ = i->second;
}

template <class Coords>
bool AdjacencyListSBPLEnv<Coords>::InitializeMDPCfg (MDPConfig *MDPCfg)
{
  MDPCfg->goalstateid = goalStateId_;
  MDPCfg->startstateid = startStateId_;
  return true;
}


template <class Coords>
int AdjacencyListSBPLEnv<Coords>::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  return points_[FromStateID].heuristicDistanceTo(points_[ToStateID]);
}

template <class Coords>
int AdjacencyListSBPLEnv<Coords>::GetGoalHeuristic(int stateID)
{
  return GetFromToHeuristic (stateID, goalStateId_);
}

template <class Coords>
int AdjacencyListSBPLEnv<Coords>::GetStartHeuristic(int stateID)
{
  return GetFromToHeuristic (startStateId_, stateID);
}

template <class Coords>
void AdjacencyListSBPLEnv<Coords>::PrintState(int stateID, bool bVerbose, FILE* fOut)
{
  // Note we're ignoring the fOut argument
  cout << points_[stateID] << endl;
}

template <class Coords>
void AdjacencyListSBPLEnv<Coords>::PrintEnv_Config(FILE* fOut)
{
  // Note we're ignoring the fOut argument
  cout << "Adjacency list env" << endl;
}


template <class Coords>
int AdjacencyListSBPLEnv<Coords>::SizeofCreatedEnv()
{
  return points_.size();
}


template <class Coords>
bool AdjacencyListSBPLEnv<Coords>::InitializeEnv (const char* sEnvFile)
{
  cout << "AdjacencyList initialization currently does nothing.";
  return true;
}



template <class Coords>
void AdjacencyListSBPLEnv<Coords>::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  // goal state is absorbing
  if (state->StateID == goalStateId_) {
    return;
  }

  Adjacencies& v = adjacencyVector_[state->StateID];
  for (unsigned int actionIndex=0; actionIndex<v.size(); actionIndex++) {
    CMDPACTION* action = state->AddAction(actionIndex);
    Adjacency adj = v[actionIndex];
    action->AddOutcome(adj.neighbor, adj.cost, 1.0);
    cout << "Added transition to " << adj.neighbor << " with cost " << adj.cost << endl;
  }
}

template <class Coords>
void AdjacencyListSBPLEnv<Coords>::SetAllPreds(CMDPSTATE* state)
{
  // Apparently this is not always necessary
  cout << "Error: SetAllPreds not implemented for adjacency list";
  exit(1);
}

template <class Coords>
void AdjacencyListSBPLEnv<Coords>::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
  SuccIDV->clear();
  CostV->clear();

  if (SourceStateID == goalStateId_) {
    return;
  }

  Adjacencies& v = adjacencyVector_[SourceStateID];
  
  for (unsigned int actionIndex=0; actionIndex<v.size(); actionIndex++) {
    Adjacency adj = v[actionIndex];
    SuccIDV->push_back(adj.neighbor);
    CostV->push_back(adj.cost);
    cout << "Added transition to " << adj.neighbor << " with cost " << adj.cost << endl;
  }
}

template <class Coords>
void AdjacencyListSBPLEnv<Coords>::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
  cout << "Error: GetPreds not currently implemented for adjacency list";
  exit(1);
}

template <class Coords>
vector<Coords> AdjacencyListSBPLEnv<Coords>::findOptimalPath ()
{
  // Initialize ARA planner
  ARAPlanner p(this, true);
  p.set_start(startStateId_);
  cout << "Hello" << endl;
  p.set_goal(goalStateId_);
  cout << "Hello" << endl;
  vector<int> solution;
  p.replan(1.0, &solution);
  cout << "Hello" << endl;

  vector<Coords> solutionPoints;
  for (unsigned int i=0; i<solution.size(); i++) {
    solutionPoints.push_back(points_[solution[i]]);
  }

  resetStateId2IndexMapping();

  return solutionPoints;
}

// There's some side effect where you have to reset this every time you call the ARA planner
template <class Coords>
void AdjacencyListSBPLEnv<Coords>::resetStateId2IndexMapping (void)
{
  for (unsigned int i=0; i<StateID2IndexMapping.size(); i++) {
    for (unsigned int j=0; j<NUMOFINDICES_STATEID2IND; j++) {
      StateID2IndexMapping[i][j]=-1;
    }
  }
}



#endif

// Local variables:
// mode:c++
// End:
