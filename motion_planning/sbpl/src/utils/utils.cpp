#include "../headers.h"

#if MEM_CHECK == 1
void DisableMemCheck()
{
// Get the current state of the flag
// and store it in a temporary variable
int tmpFlag = _CrtSetDbgFlag( _CRTDBG_REPORT_FLAG );

// Turn On (OR) - All freed memory is re-initialized with xDD
tmpFlag |= _CRTDBG_DELAY_FREE_MEM_DF;

// Turn Off (AND) - memory checking is disabled for future allocations
tmpFlag &= ~_CRTDBG_ALLOC_MEM_DF;

// Set the new state for the flag
_CrtSetDbgFlag( tmpFlag );

}

void EnableMemCheck()
{
// Get the current state of the flag
// and store it in a temporary variable
int tmpFlag = _CrtSetDbgFlag( _CRTDBG_REPORT_FLAG );

// Turn On (OR) - All freed memory is re-initialized with xDD
tmpFlag |= _CRTDBG_DELAY_FREE_MEM_DF;

// Turn On (OR) - memory checking is enabled for future allocations
tmpFlag |= _CRTDBG_ALLOC_MEM_DF;

// Set the new state for the flag
_CrtSetDbgFlag( tmpFlag );

}
#endif

void checkmdpstate(CMDPSTATE* state)
{
#if DEBUG == 0
	printf("ERROR: checkMDPstate is too expensive for not in DEBUG mode\n");
	exit(1);
#endif

	for(int aind = 0; aind < state->Actions.size(); aind++)
	{
		for(int aind1 = 0; aind1 < state->Actions.size(); aind1++)
		{
			if(state->Actions[aind1]->ActionID == state->Actions[aind]->ActionID &&
				aind1 != aind)
			{
				printf("ERROR in CheckMDP: multiple actions with the same ID exist\n");
				exit(1);
			}
		}
		for(int sind = 0; sind < state->Actions[aind]->SuccsID.size(); sind++)
		{	
			for(int sind1 = 0; sind1 < state->Actions[aind]->SuccsID.size(); sind1++)
			{
				if(state->Actions[aind]->SuccsID[sind] == state->Actions[aind]->SuccsID[sind1] &&
					sind != sind1)
				{
					printf("ERROR in CheckMDP: multiple outcomes with the same ID exist\n");
					exit(1);
				}
			}
		}		
	}
}


void CheckMDP(CMDP* mdp)
{
	for(int i = 0; i < mdp->StateArray.size(); i++)
	{
		checkmdpstate(mdp->StateArray[i]);
	}
}




void PrintMatrix(int** matrix, int rows, int cols, FILE* fOut)
{
	for(int r = 0; r < rows; r++)
	{
		for(int c = 0; c < cols; c++)
		{
			fprintf(fOut, "%d ", matrix[r][c]);
		}
		fprintf(fOut, "\n");
	}
}


//return true if there exists a path from sourcestate to targetstate and false otherwise
bool PathExists(CMDP* pMarkovChain, CMDPSTATE* sourcestate, CMDPSTATE* targetstate)
{
	CMDPSTATE* state;
	vector<CMDPSTATE*> WorkList;
	int i;
	bool *bProcessed = new bool [pMarkovChain->StateArray.size()];
	bool bFound = false;

	//insert the source state
	WorkList.push_back(sourcestate);
	while(WorkList.size() > 0)
	{
		//get the state and its info
		state = WorkList[WorkList.size()-1];
		WorkList.pop_back();

		//Markov Chain should just contain a single policy
		if(state->Actions.size() > 1)
		{
			printf("ERROR in PathExists: Markov Chain is a general MDP\n");
			exit(1);
		}

		if(state == targetstate)
		{
			//path found
			bFound = true;
			break;
		}

		//otherwise just insert policy successors into the worklist unless it is a goal state
		for(int sind = 0; state->Actions.size() != 0 && sind < state->Actions[0]->SuccsID.size(); sind++)
		{
			//get a successor
			for(i = 0; i < pMarkovChain->StateArray.size(); i++)
			{
				if(pMarkovChain->StateArray[i]->StateID == state->Actions[0]->SuccsID[sind])
					break;
			}
			if(i == pMarkovChain->StateArray.size())
			{	
				printf("ERROR in PathExists: successor is not found\n");
				exit(1);
			}
			CMDPSTATE* SuccState = pMarkovChain->StateArray[i];
					
			//insert at the end of list if not there or processed already
			if(!bProcessed[i])
			{
				bProcessed[i] = true;
				WorkList.push_back(SuccState);
			}
		} //for successors
	}//while WorkList is non empty

	delete [] bProcessed;

	return bFound;
}	

int ComputeNumofStochasticActions(CMDP* pMDP)
{
	int i;
	int nNumofStochActions = 0;
	printf("ComputeNumofStochasticActions...\n");

	for(i = 0; i < pMDP->StateArray.size(); i++)
	{
		for(int aind = 0; aind < pMDP->StateArray[i]->Actions.size(); aind++)
		{
			if(pMDP->StateArray[i]->Actions[aind]->SuccsID.size() > 1)
				nNumofStochActions++;
		}
	}
	printf("done\n");

	return nNumofStochActions;
}


void EvaluatePolicy(CMDP* PolicyMDP, int StartStateID, int GoalStateID,
					double* PolValue, bool *bFullPolicy, double* Pcgoal, int *nMerges,
					bool* bCycles)
{
	int i, j, startind=-1;
	double delta = INFINITECOST;
	double mindelta = 0.1;

	*Pcgoal = 0;
	*nMerges = 0;

	printf("Evaluating policy...\n");

	//create and initialize values
	double* vals = new double [PolicyMDP->StateArray.size()];
	double* Pcvals = new double [PolicyMDP->StateArray.size()];
	for(i = 0; i < PolicyMDP->StateArray.size(); i++)
	{
		vals[i] = 0;
		Pcvals[i] = 0;

		//remember the start index
		if(PolicyMDP->StateArray[i]->StateID == StartStateID)
		{
			startind = i;
			Pcvals[i] = 1;
		}
	}

	//initially assume full policy
	*bFullPolicy = true;
	bool bFirstIter = true;
	while(delta > mindelta)
	{
		delta = 0;
		for(i = 0; i < PolicyMDP->StateArray.size(); i++)
		{
			//get the state
			CMDPSTATE* state = PolicyMDP->StateArray[i];

			//do the backup for values
			if(state->StateID == GoalStateID)
			{
				vals[i] = 0;
			}
			else if(state->Actions.size() == 0)
			{
				*bFullPolicy = false;
				vals[i] = UNKNOWN_COST;
				*PolValue = vals[startind];
				return;
			}
			else
			{
				//normal backup
				CMDPACTION* action = state->Actions[0];

				//do backup
				double Q = 0;
				for(int oind = 0; oind < action->SuccsID.size(); oind++)
				{
					//get the state
					for(j = 0; j < PolicyMDP->StateArray.size(); j++)
					{	
						int t = PolicyMDP->StateArray[j]->StateID;
						if(PolicyMDP->StateArray[j]->StateID == action->SuccsID[oind])
							break;
					}
					if(j == PolicyMDP->StateArray.size())
					{
						printf("ERROR in EvaluatePolicy: incorrect successor %d\n", 
							action->SuccsID[oind]);
						exit(1);
					}
					Q += action->SuccsProb[oind]*(vals[j] + action->Costs[oind]);
				}

				if(vals[i] > Q)
				{
					printf("ERROR in EvaluatePolicy: val is decreasing\n"); 
					exit(1);
				}

				//update delta
				if(delta < Q - vals[i])
					delta = Q-vals[i];

				//set the value
				vals[i] = Q;
			}

			//iterate through all the predecessors and compute Pc
			double Pc = 0;
			//go over all predecessor states
			int nMerge = 0;
			for(j = 0; j < PolicyMDP->StateArray.size(); j++)
			{
				for(int oind = 0; PolicyMDP->StateArray[j]->Actions.size() > 0 && 
					oind <  PolicyMDP->StateArray[j]->Actions[0]->SuccsID.size(); oind++)
				{
					if(PolicyMDP->StateArray[j]->Actions[0]->SuccsID[oind] == state->StateID)
					{
						//process the predecessor
						double PredPc = Pcvals[j];
						double OutProb = PolicyMDP->StateArray[j]->Actions[0]->SuccsProb[oind];
				
						//accumulate into Pc
						Pc = Pc + OutProb*PredPc;
						nMerge++;

						//check for cycles
						if(bFirstIter && !(*bCycles))
						{
							if(PathExists(PolicyMDP, state, PolicyMDP->StateArray[j]))
								*bCycles = true;
						}
					}
				}
			}
			if(bFirstIter && state->StateID != GoalStateID && nMerge > 0)
				*nMerges += (nMerge-1);

			//assign Pc
			if(state->StateID != StartStateID)
				Pcvals[i] = Pc;

			if(state->StateID == GoalStateID)
				*Pcgoal = Pcvals[i];
		} //over  states
		bFirstIter = false;
	} //until delta small

	*PolValue = vals[startind];
	
	printf("done\n");
}

