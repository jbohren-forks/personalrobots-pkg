#ifndef __UTILS_H_
#define __UTILS_H_

#ifndef WIN32
#define __max(x,y) (x>y?x:y)
#define __min(x,y) (x>y?y:x)
#endif


//function prototypes
#if MEM_CHECK == 1
void DisableMemCheck();
void EnableMemCheck();
#endif
void CheckMDP(CMDP* mdp);
void PrintMatrix(int** matrix, int rows, int cols, FILE* fOut);
void EvaluatePolicy(CMDP* PolicyMDP, int StartStateID, int GoalStateID,
					double* PolValue, bool *bFullPolicy, double *Pcgoal, 
					int* nMerges, bool *bCycles);
int ComputeNumofStochasticActions(CMDP* pMDP);

#if 0
void CheckSearchMDP(CMDP* mdp, int ExcludeSuccStateID = -1);
void CheckSearchPredSucc(CMDPSTATE* state, int ExcludeSuccStateID = -1);
#endif

#endif
