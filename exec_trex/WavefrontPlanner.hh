#ifndef _WAVEFRONT_PLANNER_H_
#define _WAVEFRONT_PLANNER_H_

#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <list>

#define DTOR(a) ((a)*M_PI/180.0)

// The Player lib we're using
#include <libstandalone_drivers/plan.h>

class WavefrontPlanner {

 public:

  static WavefrontPlanner* Instance(char* mapdata = NULL, int sx = 0, int sy = 0);
  
  ~WavefrontPlanner();
  
  int GenerateGlobalPlan(double curx, double cury,
			 double goalx, double goaly);
  
  int GenerateLocalPlan(double goalx, double goaly,
			double curx, double cury);

  int DetermineDiffDriveCmds(double& vx, double& vy,
			     double curx, double cury, double curTh,
			     double goalx, double goaly, double goalth);
	
  //bool NeedNewGlobalPlan(double gx, double gy);

  void SetObstacles(double* obs, size_t num);

  double WorldCenterX() const;
  double WorldCenterY() const;
		   
  double SizeX() const;
  double SizeY() const;
  
  plan_t* GetMasterPlan() const;
  plan_t* GetActivePlan() const;
  
 protected:

  WavefrontPlanner(char* mapdata, int sx = 0, int sy = 0);


 private:

  typedef struct PlanEntry{
    
    PlanEntry(double, double, const plan_t*);
    ~PlanEntry();

    double goal_x;
    double goal_y;
    plan_t* plan;
  };

  std::list<PlanEntry*> _planEntries;
  plan_t* _masterPlan;
  plan_t* _activePlan;

  plan_t* GetPlan(double x, double y);

  plan_t* CreateNewPlan(double gx, double gy);
  
  static WavefrontPlanner* _minstance;

  double _cenx;
  double _ceny;
  
  //double _lastX;
  //double _lastY;

};

#endif
