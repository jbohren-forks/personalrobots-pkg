#ifndef __VIPLANNER_H_
#define __VIPLANNER_H_



struct VIPLANNER_T
{
	CMDP MDP;
	CMDPSTATE* StartState;
	CMDPSTATE* GoalState;
	int iteration;
};


typedef class VIPLANNERSTATEDATA : public AbstractSearchState
{
public:
	CMDPSTATE* MDPstate; //the MDP state itself
	//planner relevant data
	float v;
	float Pc;
	unsigned int iteration;

	//best action
	CMDPACTION *bestnextaction; 

	
public:
	VIPLANNERSTATEDATA() {};	
	~VIPLANNERSTATEDATA() {};
} VIState;



class VIPlanner : public SBPLPlanner
{

public:
	int replan(double allocated_time_secs, vector<int>* solution_stateIDs_V);

	//constructors
	VIPlanner(DiscreteSpaceInformation* environment, MDPConfig* MDP_cfg)
	{
		environment_ = environment;
		MDPCfg_ = MDP_cfg;
	};


private:
	
	//member variables
	MDPConfig*  MDPCfg_;
	VIPLANNER_T viPlanner;


	void Initialize_vidata(CMDPSTATE* state);

	CMDPSTATE* CreateState(int stateID);

	CMDPSTATE* GetState(int stateID);


	void PrintVIData();

	void PrintStatHeader(FILE* fOut);

	void PrintStat(FILE* fOut, clock_t starttime);


	void PrintPolicy(FILE* fPolicy);


	void backup(CMDPSTATE* state);

	void perform_iteration_backward();

	void perform_iteration_forward();

	void InitializePlanner();




};




#endif
