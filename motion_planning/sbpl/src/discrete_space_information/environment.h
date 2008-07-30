#ifndef __ENVIRONMENT_H_
#define __ENVIRONMENT_H_


class DiscreteSpaceInformation
{

public:

	//data
	vector<int*> StateID2IndexMapping;


	virtual bool InitializeEnv(char* sEnvFile) = 0;


	virtual bool InitializeMDPCfg(MDPConfig *MDPCfg) = 0;
	virtual int  GetFromToHeuristic(int FromStateID, int ToStateID) = 0;
	virtual int  GetGoalHeuristic(int stateID) = 0;
	virtual int  GetStartHeuristic(int stateID) = 0;
	virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) = 0;
	virtual void SetAllPreds(CMDPSTATE* state) = 0;
	virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV) = 0;

	virtual int	 SizeofCreatedEnv() = 0;
	virtual void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL) = 0;
	virtual void PrintEnv_Config(FILE* fOut) = 0;

};



#endif

