#include <iostream>
#include "../discrete_space_information/precomputed_adjacency_list/environment_precomputed_adjacency_list.h"


int main (int, char**)
{
  // Creating the adjacency list
  AdjacentState s12 = {2, 4};
  AdjacentState s23 = {3, 5};
  AdjacentState s14 = {4, 6};
  AdjacentState s43 = {3, 2};

  AdjacentStateVector v1;
  AdjacentStateVector v2;
  AdjacentStateVector v3;
  AdjacentStateVector v4;
  
  v1.push_back(s12);
  v1.push_back(s14);
  v2.push_back(s23);
  v4.push_back(s43);

  vector<AdjacentStateVector> adjacencies;
  adjacencies.push_back(v1);
  adjacencies.push_back(v2);
  adjacencies.push_back(v3);
  adjacencies.push_back(v4);

  vector<int> ids;
  for (int i=0; i<4; i++) {
    ids.push_back(i+1);
  }
  AdjacencyListSBPLEnv e(ids, adjacencies, 1, 3);
  e.writeToStream();


  // Initialize the MDPConfig (what does this do exactly?)
  MDPConfig c;
  e.InitializeMDPCfg(&c);

  // Initialize ARA planner
  ARAPlanner p(&e, true);
  p.set_start(c.startstateid);
  p.set_goal(c.goalstateid);


  // Do planning
  vector<int> solution;
  p.replan(1.0, &solution);
  cout << "Returned plan is ";
  for (unsigned int i=0; i<solution.size(); i++) {
    cout << solution[i] << " ";
  }
  cout << endl;

  return 0;
}
  
