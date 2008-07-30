#ifndef __MDP_H_
#define __MDP_H_

#define EPS_ERROR 0.000001

//the maximum size of the heap
#define MAXSTATESPACESIZE 2000000

class CMDPSTATE;
class CMDPACTION
{

//data
public:
	int ActionID;
	int SourceStateID;
	vector<int> SuccsID;
	vector<int> Costs;
	vector<float> SuccsProb;
	void* PlannerSpecificData;

//constructors
public:
	CMDPACTION(int ID, int sourcestateid) 
	  {
		ActionID = ID;
		SourceStateID = sourcestateid;
		PlannerSpecificData = NULL;
	  };
	~CMDPACTION()
	{
		if(PlannerSpecificData != NULL)
		{
			fprintf(stderr, "ERROR: state deletion: planner specific data is not deleted\n");
			exit(1);
		}
	};

//functions
public:
	bool Delete();
	bool IsValid();
	void AddOutcome(int OutcomeStateID, int OutcomeCost, float OutcomeProb);
	int GetIndofMostLikelyOutcome();
	int GetIndofOutcome(int OutcomeID);
	bool DeleteAllOutcomes();

private:

//operators
public:
  void operator = (const CMDPACTION& rhsaction);


};


class CMDPSTATE
{
//data
public:
	int StateID;
	vector<CMDPACTION*> Actions;
	vector<int> PredsID;
	void* PlannerSpecificData;

//constructors
public:
	CMDPSTATE(int ID) 
	  {
		StateID = ID;
		PlannerSpecificData = NULL;
	  };
	~CMDPSTATE()
	{
		if(PlannerSpecificData != NULL)
		{
			fprintf(stderr, "ERROR: state deletion: planner specific data is not deleted\n");
			exit(1);
		}
	};

//functions
public:
	bool Delete();
	CMDPACTION* AddAction(int ID);
	bool ContainsPred(int stateID);
	bool AddPred(int stateID);
	bool RemovePred(int stateID);
	bool RemoveAllActions();
	CMDPACTION* GetAction(int actionID);

private:

//operators
public:

  void operator = (const CMDPSTATE& rhsstate);

};

class CMDP
{

//data
public:
	vector<CMDPSTATE*> StateArray;

//constructors
public:
	CMDP()
	  {
	  };
	~CMDP()
	{
	};

//functions
public:
  bool empty();
  bool full();
  //creates numofstates states. Their ids are their orderings for Original, Thresholded & Search MDPs
  bool Create(int numofstates);
  bool Delete();
  void Print(FILE* fOut);
  CMDPSTATE* AddState(int StateID);

private:





//operators
public:


};




#endif
