#ifndef __PLANNER_H_
#define __PLANNER_H_


#define 	GETSTATEIND(stateid, mapid) StateID2IndexMapping[mapid][stateid]


//indices for the StateID2Index mapping
enum STATEID2IND {	STATEID2IND_SLOT0 = 0, //add more slots if necessary
				 NUMOFINDICES_STATEID2IND
};

//use the slots above for the mutually exclusive algorithms
#define VIMDP_STATEID2IND STATEID2IND_SLOT0
#define ARAMDP_STATEID2IND STATEID2IND_SLOT0

//for example
//#define YYYPLANNER_STATEID2IND STATEID2IND_SLOT0
//#define YYYPLANNER_STATEID2IND STATEID2IND_SLOT1


typedef enum 
	{	//different state types if you have more than one type inside a single planner
		ABSTRACT_STATE = 0,
		ABSTRACT_STATEACTIONPAIR,
		ABSTRACT_GENERALSTATE
} AbstractSearchStateType_t; 

class AbstractSearchState
{

public:
	struct listelement* listelem[2];
	//index of the state in the heap
	int heapindex;
	AbstractSearchStateType_t StateType; 

public:
	AbstractSearchState(){StateType = ABSTRACT_GENERALSTATE;};
	~AbstractSearchState(){};
};

class DiscreteSpaceInformation;

class SBPLPlanner
{

public:

	//returns 1 if solution is found, 0 otherwise
	virtual int replan(double allocated_time_sec, vector<int>* solution_stateIDs_V) = 0;


protected:
	DiscreteSpaceInformation *environment_;

};



#endif

