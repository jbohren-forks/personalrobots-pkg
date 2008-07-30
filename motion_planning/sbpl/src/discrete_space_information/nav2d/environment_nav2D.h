#ifndef __ENVIRONMENT_NAV2D_H_
#define __ENVIRONMENT_NAV2D_H_


#define COSTMULT 1000


#define NAV2D_MAXACTIONSWIDTH 9		


//-1, 0, 1 per each dX and dY
#define ACTIONSWIDTH 8

typedef struct ENV_NAV2D_CONFIG
{
	int EnvWidth_c;
	int EnvHeight_c;
	int StartX_c;
	int StartY_c;
	int EndX_c;
	int EndY_c;
	char** Grid2D;

	int dXY[ACTIONSWIDTH][2];


} EnvNAV2DConfig_t;

typedef struct ENVHASHENTRY
{
	int stateID;
	int X;
	int Y;
} EnvNAV2DHashEntry_t;



typedef struct
{

	int startstateid;
	int goalstateid;

	//hash table of size x_size*y_size. Maps from coords to stateId	
	int HashTableSize;
	vector<EnvNAV2DHashEntry_t*>* Coord2StateIDHashTable;

	//vector that maps from stateID to coords	
	vector<EnvNAV2DHashEntry_t*> StateID2CoordTable;

	//any additional variables

}EnvironmentNAV2D_t;



class EnvironmentNAV2D : public DiscreteSpaceInformation
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

	//member data
	EnvNAV2DConfig_t EnvNAV2DCfg;
	EnvironmentNAV2D_t EnvNAV2D;



	void ReadConfiguration(FILE* fCfg);

	void InitializeEnvConfig();

	unsigned int GETHASHBIN(unsigned int X, unsigned int Y);

	void PrintHashTableHist();


	EnvNAV2DHashEntry_t* GetHashEntry(int X, int Y);

	EnvNAV2DHashEntry_t* CreateNewHashEntry(int X, int Y);


	void CreateStartandGoalStates();

	void InitializeEnvironment();

	void ComputeHeuristicValues();

	bool IsValidCell(int X, int Y);

	bool IsWithinMapCell(int X, int Y);


};

#endif

