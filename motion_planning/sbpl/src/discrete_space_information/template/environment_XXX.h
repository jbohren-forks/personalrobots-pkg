#ifndef __ENVIRONMENT_XXX_H_
#define __ENVIRONMENT_XXX_H_


#define XXX_MAXACTIONSWIDTH 9		

typedef struct ENV_XXX_CONFIG
{
	//parameters that are read from the configuration file
	unsigned int StartX1;
	unsigned int StartX2;
	unsigned int StartX3;
	unsigned int StartX4;
	unsigned int GoalX1;
	unsigned int GoalX2;
	unsigned int GoalX3;
	unsigned int GoalX4;


	//derived and initialized elsewhere parameters

} EnvXXXConfig_t;

typedef struct ENVXXXHASHENTRY
{
	int stateID;
	unsigned int X1;
	unsigned int X2;
	unsigned int X3;
	unsigned int X4;

} EnvXXXHashEntry_t;

typedef struct
{

	int startstateid;
	int goalstateid;

	//hash table of size x_size*y_size. Maps from coords to stateId	
	int HashTableSize;
	vector<EnvXXXHashEntry_t*>* Coord2StateIDHashTable;

	//vector that maps from stateID to coords	
	vector<EnvXXXHashEntry_t*> StateID2CoordTable;

	//any additional variables

}EnvironmentXXX_t;



class EnvironmentXXX : public DiscreteSpaceInformation
{

public:

	bool InitializeEnv(char* sEnvFile);


	bool InitializeMDPCfg(MDPConfig *MDPCfg);
	int  GetFromToHeuristic(int FromStateID, int ToStateID);
	int  GetGoalHeuristic(int stateID);
	int  GetStartHeuristic(int stateID);
	void SetAllActionsandAllOutcomes(CMDPSTATE* state);
	void SetAllPreds(CMDPSTATE* state);
	void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);

	int	 SizeofCreatedEnv();
	void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
	void PrintEnv_Config(FILE* fOut);


private:

	//member variables
	EnvXXXConfig_t EnvXXXCfg;
	EnvironmentXXX_t EnvXXX;


	
	void ReadConfiguration(FILE* fCfg);

	void InitializeEnvConfig();

	unsigned int GETHASHBIN(unsigned int X1, unsigned int X2, 
						unsigned int X3, unsigned int X4);

	void PrintHashTableHist();


	EnvXXXHashEntry_t* GetHashEntry(unsigned int X1, unsigned int X2, 
										unsigned int X3, unsigned int X4);


	EnvXXXHashEntry_t* CreateNewHashEntry(unsigned int X1, unsigned int X2, 
													unsigned int X3, unsigned int X4);


	void CreateStartandGoalStates();

	void InitializeEnvironment();


	void AddAllOutcomes(unsigned int SourceX1, unsigned int SourceX2, unsigned int SourceX3,
						unsigned int SourceX4, CMDPACTION* action, int cost);

	void ComputeHeuristicValues();


};


#endif

