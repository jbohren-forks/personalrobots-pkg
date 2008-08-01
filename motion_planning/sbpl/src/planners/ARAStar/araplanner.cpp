/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "../../headers.h"




//-----------------------------------------------------------------------------------------------------
void ARAPlanner::Initialize_searchinfo(CMDPSTATE* state, ARASearchStateSpace_t* pSearchStateSpace)
{

	ARAState* searchstateinfo = (ARAState*)state->PlannerSpecificData;

	searchstateinfo->MDPstate = state;
	InitializeSearchStateInfo(searchstateinfo, pSearchStateSpace); 
}


CMDPSTATE* ARAPlanner::CreateState(int stateID, ARASearchStateSpace_t* pSearchStateSpace)
{	
	CMDPSTATE* state = NULL;

#if DEBUG
	if(environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND] != -1)
	{
		printf("ERROR in CreateState: state already created\n");
		exit(1);
	}
#endif

	//adds to the tail a state
	state = pSearchStateSpace->searchMDP.AddState(stateID);

	//remember the index of the state
	environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND] = pSearchStateSpace->searchMDP.StateArray.size()-1;

#if DEBUG
	if(state != pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND]])
	{
		printf("ERROR in CreateState: invalid state index\n");
		exit(1);
	}
#endif


	//create search specific info
	state->PlannerSpecificData = (ARAState*)malloc(sizeof(ARAState));	
	Initialize_searchinfo(state, pSearchStateSpace);
	MaxMemoryCounter += sizeof(ARAState);

	return state;

}

CMDPSTATE* ARAPlanner::GetState(int stateID, ARASearchStateSpace_t* pSearchStateSpace)
{	

	if(stateID >= environment_->StateID2IndexMapping.size())
	{
		printf("ERROR int GetState: stateID is invalid\n");
		exit(1);
	}

	if(environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND] == -1)
		return CreateState(stateID, pSearchStateSpace);
	else
		return pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND]];

}



//-----------------------------------------------------------------------------------------------------




int ARAPlanner::ComputeHeuristic(CMDPSTATE* MDPstate, ARASearchStateSpace_t* pSearchStateSpace)
{
	//compute heuristic for search
#if ARA_SEARCH_FORWARD

#if MEM_CHECK == 1
	//int WasEn = DisableMemCheck();
#endif

    //forward search: heur = distance from state to searchgoal which is Goal ARAState
	int retv =  environment_->GetFromToHeuristic(MDPstate->StateID, pSearchStateSpace->searchgoalstate->StateID);

#if MEM_CHECK == 1
	//if (WasEn)
	//	EnableMemCheck();
#endif

	return retv;

#else
	//backward search: heur = distance from searchgoal to state
	return Env_GetFromToHeuristic(pSearchStateSpace->searchgoalstate->StateID, MDPstate->StateID);
#endif

}


//initialization of a state
void ARAPlanner::InitializeSearchStateInfo(ARAState* state, ARASearchStateSpace_t* pSearchStateSpace)
{
	state->g = INFINITECOST;
	state->v = INFINITECOST;
	state->iterationclosed = 0;
	state->callnumberaccessed = pSearchStateSpace->callnumber;
	state->bestnextstate = NULL;
	state->costtobestnextstate = INFINITECOST;
	state->heapindex = 0;
	state->listelem[INCONS_LIST_ID] = 0;
	state->numofexpands = 0;

#if ARA_SEARCH_FORWARD == 1
	state->bestpredstate = NULL;
#endif

	//compute heuristics
#if USE_HEUR
	if(pSearchStateSpace->searchgoalstate != NULL)
		state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace); 
	else 
		state->h = 0;
#else
	state->h = 0;
#endif


}



//re-initialization of a state
void ARAPlanner::ReInitializeSearchStateInfo(ARAState* state, ARASearchStateSpace_t* pSearchStateSpace)
{
	state->g = INFINITECOST;
	state->v = INFINITECOST;
	state->iterationclosed = 0;
	state->callnumberaccessed = pSearchStateSpace->callnumber;
	state->bestnextstate = NULL;
	state->costtobestnextstate = INFINITECOST;
	state->heapindex = 0;
	state->listelem[INCONS_LIST_ID] = 0;
	state->numofexpands = 0;

#if ARA_SEARCH_FORWARD == 1
	state->bestpredstate = NULL;
#endif

	//compute heuristics
#if USE_HEUR

	if(pSearchStateSpace->searchgoalstate != NULL)
	{

#if ARA_SEARCH_FORWARD == 0
		state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace); 
#endif

	}
	else 
		state->h = 0;

#else

	state->h = 0;

#endif


}



void ARAPlanner::DeleteSearchStateData(ARAState* state)
{
	//no memory was allocated
	MaxMemoryCounter = 0;
	return;
}



#if !ARA_SEARCH_FORWARD
//used for backward search
void ARAPlanner::UpdatePreds(ARAState* state, ARASearchStateSpace_t* pSearchStateSpace)
{
    vector<int> PredIDV;
    vector<int> CostV;
	CKey key;
	int bestc;
	double bestprob;
	CMDPACTION* bestaction;
	ARAState *predstate;

    Env_GetPreds(state->MDPstate->StateID, &PredIDV, &CostV);

	//iterate through predecessors of s
	for(int pind = 0; pind < PredIDV.size(); pind++)
	{
		CMDPSTATE* PredMDPState = GetState(PredIDV[pind]);
		predstate = (ARAState*)(PredMDPState->PlannerSpecificData);
		if(predstate->callnumberaccessed != pSearchStateSpace->callnumber)
			ReInitializeSearchStateInfo(predstate, pSearchStateSpace);

		//see if we can improve the value of predstate
		if(predstate->g > state->v + CostV[pind])
		{
			predstate->g = state->v + CostV[pind];
			predstate->bestnextstate = state->MDPstate;
			predstate->costtobestnextstate = CostV[pind];
			predstate->bestnextaction = bestaction;

			//re-insert into heap if not closed yet
			if(predstate->iterationclosed != pSearchStateSpace->iteration)
			{
				key.key[0] = predstate->g + (int)(pSearchStateSpace->eps*predstate->h);
				//key.key[1] = predstate->h;
				if(predstate->heapindex != 0)
					pSearchStateSpace->heap->updateheap(predstate,key);
				else
					pSearchStateSpace->heap->insertheap(predstate,key);
			}
			//take care of incons list
			else if(predstate->listelem[INCONS_LIST_ID] == NULL)
			{
				pSearchStateSpace->inconslist->insert(predstate, INCONS_LIST_ID);
			}
		}
	} //for predecessors

}
#endif

#if ARA_SEARCH_FORWARD
//used for forward search
void ARAPlanner::UpdateSuccs(ARAState* state, ARASearchStateSpace_t* pSearchStateSpace)
{
    vector<int> SuccIDV;
    vector<int> CostV;
	CKey key;
	ARAState *succstate;

    environment_->GetSuccs(state->MDPstate->StateID, &SuccIDV, &CostV);

	//iterate through predecessors of s
	for(int sind = 0; sind < SuccIDV.size(); sind++)
	{
		CMDPSTATE* SuccMDPState = GetState(SuccIDV[sind], pSearchStateSpace);
		int cost = CostV[sind];

		succstate = (ARAState*)(SuccMDPState->PlannerSpecificData);
		if(succstate->callnumberaccessed != pSearchStateSpace->callnumber)
			ReInitializeSearchStateInfo(succstate, pSearchStateSpace);

		//see if we can improve the value of succstate
		//taking into account the cost of action
		if(succstate->g > state->v + cost)
		{
			succstate->g = state->v + cost;
			succstate->bestpredstate = state->MDPstate; 

			//re-insert into heap if not closed yet
			if(succstate->iterationclosed != pSearchStateSpace->iteration)
			{
				
				key.key[0] = succstate->g + (int)(pSearchStateSpace->eps*succstate->h);

				//key.key[1] = succstate->h;

				if(succstate->heapindex != 0)
					pSearchStateSpace->heap->updateheap(succstate,key);
				else
					pSearchStateSpace->heap->insertheap(succstate,key);
			}
			//take care of incons list
			else if(succstate->listelem[INCONS_LIST_ID] == NULL)
			{
				pSearchStateSpace->inconslist->insert(succstate, INCONS_LIST_ID);
			}
		} //check for cost improvement 

	} //for actions
}
#endif


int ARAPlanner::GetGVal(int StateID, ARASearchStateSpace_t* pSearchStateSpace)
{
	 CMDPSTATE* cmdp_state = GetState(StateID, pSearchStateSpace);
	 ARAState* state = (ARAState*)cmdp_state->PlannerSpecificData;
	 return state->g;
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
int ARAPlanner::ImprovePath(ARASearchStateSpace_t* pSearchStateSpace, int MaxNumofSecs)
{
	int expands;
	ARAState *state, *searchgoalstate;
	CKey key, minkey;
	CKey goalkey;

	expands = 0;

	clock_t currenttime = clock();

	if(pSearchStateSpace->searchgoalstate == NULL)
	{
		printf("ERROR searching: no goal state is set\n");
		exit(1);
	}

	//goal state
	searchgoalstate = (ARAState*)(pSearchStateSpace->searchgoalstate->PlannerSpecificData);
	if(searchgoalstate->callnumberaccessed != pSearchStateSpace->callnumber)
		ReInitializeSearchStateInfo(searchgoalstate, pSearchStateSpace);

	//set goal key
	goalkey.key[0] = searchgoalstate->g;
	//goalkey.key[1] = searchgoalstate->h;

	//expand states until done
	minkey = pSearchStateSpace->heap->getminkeyheap();
	CKey oldkey = minkey;
	while(!pSearchStateSpace->heap->emptyheap() && goalkey > minkey &&
		(clock()-TimeStarted) < MaxNumofSecs*(double)CLOCKS_PER_SEC) 
    {

		//get the state		
		state = (ARAState*)pSearchStateSpace->heap->deleteminheap();


#if DEBUG
		fprintf(fDeb, "expanding state(%d): h=%d g=%u key=%u v=%u iterc=%d callnuma=%d expands=%d (g(goal)=%u)\n",
			state->MDPstate->StateID, state->h, state->g, state->g+(int)(pSearchStateSpace->eps*state->h), state->v, 
			state->iterationclosed, state->callnumberaccessed, state->numofexpands, searchgoalstate->g);
		Env_PrintState(state->MDPstate->StateID, true, false, fDeb);
		fflush(fDeb);
#endif

#if DEBUG
		if(minkey.key[0] < oldkey.key[0] && fabs(this->finitial_eps - 1.0) < ERR_EPS)
		{
			//printf("WARN in search: the sequence of keys decreases\n");
			//exit(1);
		}
		oldkey = minkey;
#endif

		if(state->v == state->g)
		{
			printf("ERROR: consistent state is being expanded\n");
			exit(1);
		}

		//recompute state value      
		state->v = state->g;
		state->iterationclosed = pSearchStateSpace->iteration;

		//new expand      
		expands++;
		state->numofexpands++;


#if ARA_SEARCH_FORWARD == 0
		UpdatePreds(state, pSearchStateSpace);
#else
		UpdateSuccs(state, pSearchStateSpace);
#endif
		
		//recompute minkey
		minkey = pSearchStateSpace->heap->getminkeyheap();

		//recompute goalkey if necessary
		if(goalkey.key[0] != searchgoalstate->g)
		{
			//printf("re-computing goal key\n");
			//recompute the goal key (heuristics should be zero)
			goalkey.key[0] = searchgoalstate->g;
			//goalkey.key[1] = searchgoalstate->h;
		}

		if(expands%100000 == 0 && expands > 0)
		{
			printf("expands so far=%u\n", expands);
		}

	}

	int retv = 1;
	if(searchgoalstate->g == INFINITECOST && pSearchStateSpace->heap->emptyheap())
	{
		printf("solution does not exist: search exited because heap is empty\n");
		retv = 0;
	}
	else if(!pSearchStateSpace->heap->emptyheap() && goalkey > minkey)
	{
		printf("search exited because it ran out of time\n");
		retv = 2;
	}
	else if(searchgoalstate->g == INFINITECOST && !pSearchStateSpace->heap->emptyheap())
	{
		printf("solution does not exist: search exited because all candidates for expansion have infinite heuristics\n");
		retv = 0;
	}
	else
	{
		printf("search exited with a solution for eps=%.3f\n", pSearchStateSpace->eps);
		retv = 1;
	}

	//fprintf(fDeb, "expanded=%d\n", expands);

	searchexpands += expands;

	return retv;		
}


void ARAPlanner::BuildNewOPENList(ARASearchStateSpace_t* pSearchStateSpace)
{
	ARAState *state;
	CKey key;
	CHeap* pheap = pSearchStateSpace->heap;
	CList* pinconslist = pSearchStateSpace->inconslist; 
		
	//move incons into open
	while(pinconslist->firstelement != NULL)
	  {
	    state = (ARAState*)pinconslist->firstelement->liststate;
	    
	    //compute f-value
	    key.key[0] = state->g + (int)(pSearchStateSpace->eps*state->h);
	    //key.key[1] = state->h;
	    
	    //insert into OPEN
	    pheap->insertheap(state, key);
	    //remove from INCONS
	    pinconslist->remove(state, INCONS_LIST_ID);
	  }
}


void ARAPlanner::Reevaluatefvals(ARASearchStateSpace_t* pSearchStateSpace)
{
	CKey key;
	int i;
	CHeap* pheap = pSearchStateSpace->heap;
	CList* pinconslist = pSearchStateSpace->inconslist; 
	
	//recompute priorities for states in OPEN and reorder it
	for (i = 1; i <= pheap->currentsize; ++i)
	  {
		ARAState* state = (ARAState*)pheap->heap[i].heapstate;
	    pheap->heap[i].key.key[0] = state->g + 
	      (int)(pSearchStateSpace->eps*state->h); 
	    //pheap->heap[i].key.key[1] = state->h; 
	  }
	pheap->makeheap();

	pSearchStateSpace->bReevaluatefvals = false;
}




//creates (allocates memory) search state space
//does not initialize search statespace
int ARAPlanner::CreateSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace)
{

	//create a heap
	pSearchStateSpace->heap = new CHeap;
	pSearchStateSpace->inconslist = new CList;
	MaxMemoryCounter += sizeof(CHeap);
	MaxMemoryCounter += sizeof(CList);

	pSearchStateSpace->searchgoalstate = NULL;
	pSearchStateSpace->searchstartstate = NULL;

	searchexpands = 0;

	
	return 1;
}

//deallocates memory used by SearchStateSpace
void ARAPlanner::DeleteSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace)
{
	if(pSearchStateSpace->heap != NULL)
	{
		pSearchStateSpace->heap->makeemptyheap();
		delete pSearchStateSpace->heap;
		pSearchStateSpace->heap = NULL;
	}

	if(pSearchStateSpace->inconslist != NULL)
	{
		pSearchStateSpace->inconslist->makeemptylist(INCONS_LIST_ID);
		delete pSearchStateSpace->inconslist;
		pSearchStateSpace->inconslist = NULL;
	}

	//delete the states themselves
	int iend = pSearchStateSpace->searchMDP.StateArray.size();
	for(int i=0; i < iend; i++)
	{
		CMDPSTATE* state = pSearchStateSpace->searchMDP.StateArray[i];
		DeleteSearchStateData((ARAState*)state->PlannerSpecificData);
		delete (ARAState*)state->PlannerSpecificData;
		state->PlannerSpecificData = NULL;
	}
	pSearchStateSpace->searchMDP.Delete();
	environment_->StateID2IndexMapping.clear();
}



//reset properly search state space
//needs to be done before deleting states
int ARAPlanner::ResetSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace)
{
	pSearchStateSpace->heap->makeemptyheap();
	pSearchStateSpace->inconslist->makeemptylist(INCONS_LIST_ID);

	return 1;
}

//initialization before each search
void ARAPlanner::ReInitializeSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace)
{
	CKey key;

	//increase callnumber
	pSearchStateSpace->callnumber++;

	//reset iteration
	pSearchStateSpace->iteration = 0;

	pSearchStateSpace->heap->makeemptyheap();
	pSearchStateSpace->inconslist->makeemptylist(INCONS_LIST_ID);

	//initialize start state
	ARAState* startstateinfo = (ARAState*)(pSearchStateSpace->searchstartstate->PlannerSpecificData);
	if(startstateinfo->callnumberaccessed != pSearchStateSpace->callnumber)
		ReInitializeSearchStateInfo(startstateinfo, pSearchStateSpace);

	startstateinfo->g = 0;

	//insert start state into the heap
	key.key[0] = (long int)(pSearchStateSpace->eps*startstateinfo->h);
	//key.key[1] = startstateinfo->h;
	pSearchStateSpace->heap->insertheap(startstateinfo, key);

}

//very first initialization
int ARAPlanner::InitializeSearchStateSpace(int SearchStartStateID, 
						  ARASearchStateSpace_t* pSearchStateSpace)
{

	if(pSearchStateSpace->heap->currentsize != 0 || 
		pSearchStateSpace->inconslist->currentsize != 0)
	{
		printf("ERROR in InitializeSearchStateSpace: heap or list is not empty\n");
		exit(1);
	}

	pSearchStateSpace->eps = this->finitial_eps;
	pSearchStateSpace->iteration = 0;
	pSearchStateSpace->callnumber = 0;
	pSearchStateSpace->bReevaluatefvals = false;
	pSearchStateSpace->bEncounteredLinkState = false;


	//create and set the search start state
	pSearchStateSpace->searchgoalstate = NULL;
	pSearchStateSpace->searchstartstate = GetState(SearchStartStateID, pSearchStateSpace);

	
	return 1;

}


int ARAPlanner::SetSearchGoalState(int SearchGoalStateID, ARASearchStateSpace_t* pSearchStateSpace)
{
	if(pSearchStateSpace->searchgoalstate == NULL || 
		pSearchStateSpace->searchgoalstate->StateID != SearchGoalStateID)
	{
		pSearchStateSpace->searchgoalstate = GetState(SearchGoalStateID, pSearchStateSpace);

		//recompute heuristic for the heap if heuristics is used
#if USE_HEUR
		for(int i = 0; i < pSearchStateSpace->searchMDP.StateArray.size(); i++)
		{
			CMDPSTATE* MDPstate = pSearchStateSpace->searchMDP.StateArray[i];
			ARAState* state = (ARAState*)MDPstate->PlannerSpecificData;
			state->h = ComputeHeuristic(MDPstate, pSearchStateSpace);
		}
		
		pSearchStateSpace->bReevaluatefvals = true;
#endif
	}


	return 1;

}


int ARAPlanner::SetSearchStartState(int SearchStartStateID, ARASearchStateSpace_t* pSearchStateSpace)
{

	pSearchStateSpace->searchstartstate = GetState(SearchStartStateID, pSearchStateSpace);

	return 1;

}



int ARAPlanner::ReconstructPath(ARASearchStateSpace_t* pSearchStateSpace, CMDPSTATE* MDPstateGoal)
{	
	CMDPSTATE* MDPstate = MDPstateGoal;
	CMDPSTATE* PredMDPstate;
	ARAState *predstateinfo, *stateinfo;

	while(MDPstate != pSearchStateSpace->searchstartstate)
	{
		stateinfo = (ARAState*)MDPstate->PlannerSpecificData;

		if(stateinfo->g == INFINITECOST)
		{	
			//printf("ERROR in ReconstructPath: g of the state on the path is INFINITE\n");
			//exit(1);
			return -1;
		}

		if(stateinfo->bestpredstate == NULL)
		{
			printf("ERROR in ReconstructPath: bestpred is NULL\n");
			exit(1);
		}

		//get the parent state
		PredMDPstate = stateinfo->bestpredstate;
		predstateinfo = (ARAState*)PredMDPstate->PlannerSpecificData;

		//set its best next info
		predstateinfo->bestnextstate = MDPstate;

		//check the decrease of g-values along the path
		if(predstateinfo->v >= stateinfo->g)
		{
			printf("ERROR in ReconstructPath: g-values are non-decreasing\n");
			exit(1);
		}

		//transition back
		MDPstate = PredMDPstate;
	}


	return 1;
}

int ARAPlanner::ReconstructPath(ARASearchStateSpace_t* pSearchStateSpace, int StateIDGoal)
{
	CMDPSTATE* MDPstateGoal = GetState(StateIDGoal, pSearchStateSpace);
	return(ReconstructPath(pSearchStateSpace, MDPstateGoal));
}


void ARAPlanner::PrintSearchPath(ARASearchStateSpace_t* pSearchStateSpace, FILE* fOut)
{
	ARAState* searchstateinfo;
	CMDPSTATE* state = pSearchStateSpace->searchstartstate;

	if(fOut == NULL)
		fOut = stdout;

	int PathCost = ((ARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;

	fprintf(fOut, "Printing a path from state %d to the goal state %d\n", 
			state->StateID, pSearchStateSpace->searchgoalstate->StateID);
	fprintf(fOut, "Path cost = %d:\n", PathCost);
			
	
	environment_->PrintState(state->StateID, false, fOut);

	int costFromStart = 0;
	while(state->StateID != pSearchStateSpace->searchgoalstate->StateID)
	{
		fprintf(fOut, "state %d ", state->StateID);

		if(state->PlannerSpecificData == NULL)
		{
			fprintf(fOut, "path does not exist since search data does not exist\n");
			break;
		}

		searchstateinfo = (ARAState*)state->PlannerSpecificData;

		if(searchstateinfo->bestnextstate == NULL)
		{
			fprintf(fOut, "path does not exist since bestnextstate == NULL\n");
			break;
		}
		if(searchstateinfo->g == INFINITECOST)
		{
			fprintf(fOut, "path does not exist since bestnextstate == NULL\n");
			break;
		}

		int costToGoal = PathCost - costFromStart;
		costFromStart += searchstateinfo->costtobestnextstate;

		fprintf(fOut, "g=%d-->state %d, h = %d ctg = %d  ", searchstateinfo->g, 			
			searchstateinfo->bestnextstate->StateID, searchstateinfo->h, costToGoal);

		state = searchstateinfo->bestnextstate;

		environment_->PrintState(state->StateID, false, fOut);



	}
}

int ARAPlanner::getHeurValue(ARASearchStateSpace_t* pSearchStateSpace, int StateID)
{
	CMDPSTATE* MDPstate = GetState(StateID, pSearchStateSpace);
	ARAState* searchstateinfo = (ARAState*)MDPstate->PlannerSpecificData;
	return searchstateinfo->h;
}


vector<int> ARAPlanner::GetSearchPath(ARASearchStateSpace_t* pSearchStateSpace, CMDPSTATE* stateGoal, int& solcost)
{
    vector<int> SuccIDV;
    vector<int> CostV;
	vector<int> wholePathIds;
	ARAState* searchstateinfo;
	CMDPSTATE* state = pSearchStateSpace->searchstartstate;

	wholePathIds.push_back(state->StateID);
    solcost = 0;

	FILE* fOut = stdout;
	while(state->StateID != stateGoal->StateID)
	{
		if(state->PlannerSpecificData == NULL)
		{
			fprintf(fOut, "path does not exist since search data does not exist\n");
			break;
		}

		searchstateinfo = (ARAState*)state->PlannerSpecificData;

		if(searchstateinfo->bestnextstate == NULL)
		{
			fprintf(fOut, "path does not exist since bestnextstate == NULL\n");
			break;
		}
		if(searchstateinfo->g == INFINITECOST)
		{
			fprintf(fOut, "path does not exist since bestnextstate == NULL\n");
			break;
		}

        environment_->GetSuccs(state->StateID, &SuccIDV, &CostV);
        int actioncost = INFINITECOST;
        for(int i = 0; i < SuccIDV.size(); i++)
        {   
            if(SuccIDV.at(i) == searchstateinfo->bestnextstate->StateID)
                actioncost = CostV.at(i);

        }
        solcost += actioncost;

        fprintf(fDeb, "actioncost=%d between states %d and %d\n", state->StateID, searchstateinfo->bestnextstate->StateID);
        environment_->PrintState(state->StateID, false, fDeb);
        environment_->PrintState(searchstateinfo->bestnextstate->StateID, false, fDeb);


		state = searchstateinfo->bestnextstate;

		wholePathIds.push_back(state->StateID);
	}
	return wholePathIds;
}

vector<int> ARAPlanner::GetPath(ARASearchStateSpace_t* pSearchStateSpace, int StateIDGoal, int& solcost)
{

	CMDPSTATE* MDPstateGoal = GetState(StateIDGoal, pSearchStateSpace);
	return(GetSearchPath(pSearchStateSpace, MDPstateGoal, solcost));
}


bool ARAPlanner::Search(ARASearchStateSpace_t* pSearchStateSpace, vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs)
{
	CKey key;
	CHeap* pheap = pSearchStateSpace->heap;
	CList* pinconslist = pSearchStateSpace->inconslist;
	TimeStarted = clock();


#if DEBUG
	fprintf(fDeb, "new search call (call number=%d)\n", pSearchStateSpace->callnumber);
#endif

	//re-initialize state space 
	ReInitializeSearchStateSpace(pSearchStateSpace);

	if(bOptimalSolution)
	{
		pSearchStateSpace->eps = 1;
		MaxNumofSecs = INFINITECOST;
	}
	else if(bFirstSolution)
	{
		MaxNumofSecs = INFINITECOST;
	}

	//the main loop of ARA*
	int prevexpands = 0;
	while((pSearchStateSpace->iteration == 0 || pSearchStateSpace->eps > FINAL_EPS + 0.000001) && 
		(clock()- TimeStarted) < MaxNumofSecs*(double)CLOCKS_PER_SEC)
	{
		//it will be a new search iteration
		pSearchStateSpace->iteration++;

		//decrease eps for all subsequent iterations
		if(pSearchStateSpace->iteration > 1)
		{
			pSearchStateSpace->eps = pSearchStateSpace->eps - DECREASE_EPS;
			if(pSearchStateSpace->eps < FINAL_EPS)
				pSearchStateSpace->eps = FINAL_EPS;


			pSearchStateSpace->bReevaluatefvals = true;

			//build a new open list by merging it with incons one
			BuildNewOPENList(pSearchStateSpace);

			//re-compute f-values if necessary and reorder the heap
			if(pSearchStateSpace->bReevaluatefvals)
				Reevaluatefvals(pSearchStateSpace);
		}

		//improve or compute path
		ImprovePath(pSearchStateSpace, MaxNumofSecs);

		//print the solution cost and eps bound
		printf("eps=%f expands=%d g(sstart)=%d\n", pSearchStateSpace->eps, searchexpands - prevexpands,
							((ARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g);
		prevexpands = searchexpands;


		//if just the first solution then we are done
		if(bFirstSolution)
			break;

		//no solution exists
		if(((ARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g == INFINITECOST)
			break;

	}

#if ARA_SEARCH_FORWARD == 1
	ReconstructPath(pSearchStateSpace, pSearchStateSpace->searchgoalstate);
#endif

#if DEBUG
	fflush(fDeb);
#endif

	PathCost = ((ARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;
	printf("\n!!! searchexpands = %u\n\n", searchexpands);
	MaxMemoryCounter += environment_->StateID2IndexMapping.size()*sizeof(int);
	
	printf("MaxMemoryCounter = %d\n", MaxMemoryCounter);

	int solcost = INFINITECOST;
    bool ret = false;
	if(((ARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g == INFINITECOST)
	{
		printf("could not find a solution\n");
		ret = false;
	}
	else
	{
		printf("solution is found\n");      
    	pathIds = GetSearchPath(pSearchStateSpace, pSearchStateSpace->searchgoalstate, solcost);
        ret = true;
	}

	
    //fprintf(fStat, "%d %d\n", searchexpands, solcost);

	return true;

}


//-----------------------------Interface function-----------------------------------------------------
//returns 1 if found a solution, and 0 otherwise
int ARAPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V)
{
    vector<int> pathIds; 
    int PathCost = 0;
    bool bFound = false;
	int ret = 0;
	FILE* fPolicy = fopen("sol.txt","w");
    ARASearchStateSpace_t* pSearchStateSpace = new ARASearchStateSpace_t;
    int SearchStartStateID = 0;
    int SearchGoalStateID = 0;


#if ARA_SEARCH_FORWARD == 1
    SearchStartStateID = MDPCfg_->startstateid;
    SearchGoalStateID = MDPCfg_->goalstateid;
#else
    SearchStartStateID = MDPCfg_->goalstateid;
    SearchGoalStateID = MDPCfg_->startstateid;
#endif

	//create the ARA planner
    if(CreateSearchStateSpace(pSearchStateSpace) != 1)
    {
        printf("ERROR: failed to create statespace\n");
        return 0;
    }
    
    //set the start and goal states
    if(InitializeSearchStateSpace(SearchStartStateID, pSearchStateSpace) != 1)
    {
        printf("ERROR: failed to create statespace\n");
        return 0;
    }

    if(SetSearchStartState(SearchStartStateID, pSearchStateSpace) != 1)
    {
        printf("ERROR: failed to set search start state\n");
        return 0;
    }

    if(SetSearchGoalState(SearchGoalStateID, pSearchStateSpace) != 1)
    {
        printf("ERROR: failed to set search goal state\n");
        return 0;
    }

    //plan for the first solution only
    if((bFound = Search(pSearchStateSpace, *solution_stateIDs_V, PathCost, false, false, allocated_time_secs)) == false) 
    {
        printf("failed to find a solution\n");
    }

    //delete the statespace
    DeleteSearchStateSpace(pSearchStateSpace);


	return 1;

}

//---------------------------------------------------------------------------------------------------------

