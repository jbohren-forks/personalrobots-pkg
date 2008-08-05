#include <iostream>
#include <sys/time.h>

#include "WavefrontPlanner.hh"
#include "map_helpers.h"

static const bool DoTiming = false;
static const double InitLastGoalVal = -10000.0;

static const double VerySmall = .01;

//hardcoding in parameters
static const double PlanHalfwidth = 5.0;
static const double DistEps = .6;
static const double AngEps = MY_DTOR(10.0);
static const double LookaheadMaxdist = 2.0;
static const double LookaheadDistweight = 5.0;
static const double TvMin = 0.2;
static const double TvMax = 0.75;
static const double AvMin = MY_DTOR(10.0);
static const double AvMax = MY_DTOR(80.0);
static const double AMin = MY_DTOR(10.0);
static const double AMax = MY_DTOR(40.0);

static const double RobotRadius = .5;
static const double MaxRadius = 1.0;
static const double SafetyDist = 0.05;
static const double DistPenalty = 2.0;
static const double VelXMin = -10;
static const double VelXMax = 10;
static const double VelThMin = -10;
static const double VelThMax = 10;

WavefrontPlanner* WavefrontPlanner::_minstance = 0;

WavefrontPlanner::WavefrontPlanner(char* ex_mapdata, int ex_sx, int ex_sy)
  //  _lastX(InitLastGoalVal), _lastY(InitLastGoalVal)
{

  struct timeval timebefore;
  struct timeval timeafter;
  struct timezone tzdummy;

  if(DoTiming) {
    gettimeofday(&timebefore,&tzdummy);
  }

  bool exMap = (ex_mapdata != NULL);

  char* mapdata;
  int sx, sy;
  if(!exMap) {
    assert(read_map_from_image(&sx, &sy, &mapdata, "willow-clean.png", 0) == 0); 
  } else {
    sx = ex_sx;
    sy = ex_sy;
    mapdata = ex_mapdata;
  }


  assert(_masterPlan = plan_alloc(RobotRadius+SafetyDist,
			    RobotRadius+SafetyDist,
			    MaxRadius,
			    DistPenalty,
			    0.5));
  
  assert(_masterPlan->cells == NULL);
  assert((_masterPlan->cells = 
	  (plan_cell_t*)realloc(_masterPlan->cells,
                                (sx * sy * sizeof(plan_cell_t)))));
  
  for(int j=0;j<sy;j++) {
    for(int i=0;i<sx;i++) {
      _masterPlan->cells[i+j*sx].occ_state = mapdata[i+j*sx];
    }
  }

  if(!exMap) {
    free(mapdata);
  }

  _masterPlan->scale = .1;
  _masterPlan->size_x = sx;
  _masterPlan->size_y = sy;
  _masterPlan->origin_x = 0.0;
  _masterPlan->origin_y = 0.0;

  _cenx = (sx*1.0*_masterPlan->scale)/2.0;
  _ceny = (sy*1.0*_masterPlan->scale)/2.0;

  std::cout << "Center x " << _cenx << " " << _ceny << std::endl;

  // Do initialization
  plan_init(_masterPlan);

  // Compute cspace over static map
  plan_compute_cspace(_masterPlan);

  if(DoTiming) {
    gettimeofday(&timeafter,&tzdummy);
    int best_seconds = timeafter.tv_sec-timebefore.tv_sec;
    int best_microseconds = timeafter.tv_usec-timebefore.tv_usec;
    double best_tt = best_seconds*1.0+(.000001)*(best_microseconds*1.0);
    std::cout << "Map initialization took " << best_tt << " seconds\n";
  }

  if(DoTiming) {
    gettimeofday(&timebefore,&tzdummy);
  }

  _activePlan = _masterPlan;
  _lastObs = NULL;

  // if(DoTiming) {
//     gettimeofday(&timeafter,&tzdummy);
//     int best_seconds = timeafter.tv_sec-timebefore.tv_sec;
//     int best_microseconds = timeafter.tv_usec-timebefore.tv_usec;
//     double best_tt = best_seconds*1.0+(.000001)*(best_microseconds*1.0);
//     std::cout << "Map copy took " << best_tt << " seconds\n";
//   }   

}

WavefrontPlanner::~WavefrontPlanner() {
  if(_minstance != 0) {
    delete _minstance;
    _minstance = 0;
  }

  plan_free(_masterPlan);

  for(std::list<PlanEntry*>::iterator it = _planEntries.begin();
      it != _planEntries.end();
      it++) {
    delete (*it);
  }
  _planEntries.clear();
}


WavefrontPlanner* WavefrontPlanner::Instance(char* mapdata, int sx, int sy) {

  if(_minstance == 0) {
    _minstance = new WavefrontPlanner(mapdata, sx, sy);
  }
  return _minstance;
}

int WavefrontPlanner::GenerateGlobalPlan(double curx, double cury,
					 double goalx, double goaly) {
  
  struct timeval timebefore;
  struct timeval timeafter;
  struct timezone tzdummy;
  
  if(DoTiming) {
    gettimeofday(&timebefore,&tzdummy);
  }

  //check if we already have a plan for this
  PlanEntry* pt = GetPlan(goalx,goaly);
  if(pt == NULL) {
    pt =  CreateNewPlan(goalx, goaly);
  } else {
    //for now assume we don't need to replan
    //return 1;
  }

  UpdatePlanEntryObsIfNeeded(pt);
  
  int res = 0;
  res = plan_do_global(pt->plan, curx, cury, goalx, goaly);

  if(res < 0) {
    std::cout << "Global plan failing\n";
    //std::cout << " cur x " << curx << " cur y " << cury << std::endl;
    //std::cout << " goal x " << goalx << " goal y " << goaly << std::endl;
    //draw_path(pt, curx, cury, "global_path.png");
    //draw_map(pt, curx, cury, "cur.png");
    //draw_map(pt, goalx, goaly, "goal.png");
    //draw_costmap(pt, "cost.png");
  } 

  if(DoTiming) {
    gettimeofday(&timeafter,&tzdummy);
    int best_seconds = timeafter.tv_sec-timebefore.tv_sec;
    int best_microseconds = timeafter.tv_usec-timebefore.tv_usec;
    double best_tt = best_seconds*1.0+(.000001)*(best_microseconds*1.0);
    std::cout << "Global planning took " << best_tt << " seconds\n";
  }
  //draw_path(pt, curx, cury, "global_path.png");

  //if(res >= 0) {
  //  _lastX = goalx;
  //  _lastY = goaly;
  //}
  return res;
}

int WavefrontPlanner::GenerateLocalPlan(double curx, double cury,
					double goalx, double goaly) {

  struct timeval timebefore;
  struct timeval timeafter;
  struct timezone tzdummy;
  
  if(DoTiming) {
    gettimeofday(&timebefore,&tzdummy);
  }

  PlanEntry* pt = GetPlan(goalx,goaly);
  if(pt == NULL) return -1;

  _activePlan = pt->plan;

  UpdatePlanEntryObsIfNeeded(pt);

  int res = plan_do_local(pt->plan, curx, cury, PlanHalfwidth);

  if(res < 0) {
    std::cout << "Local planning failed.\n";
  }

  if(DoTiming) {
    gettimeofday(&timeafter,&tzdummy);
    int best_seconds = timeafter.tv_sec-timebefore.tv_sec;
    int best_microseconds = timeafter.tv_usec-timebefore.tv_usec;
    //double best_tt = best_seconds*1.0+(.000001)*(best_microseconds*1.0);
    //std::cout << "Local planning took " << best_tt << " seconds\n";
  }
  return res;
}

int WavefrontPlanner::DetermineDiffDriveCmds(double& vx, double& va,
					     double curx, double cury, double curTh,
					     double goalx, double goaly, double goalth) {

  int rotateDir = 0;

  struct timeval timebefore;
  struct timeval timeafter;
  struct timezone tzdummy;
  
  if(DoTiming) {
    gettimeofday(&timebefore,&tzdummy);
  }

  PlanEntry* pt = GetPlan(goalx,goaly);
  if(pt == NULL) return -1;

  //assume we've already inserted obstacles if needed

  int res = plan_compute_diffdrive_cmds(pt->plan,&vx,&va,
					&rotateDir,
					curx, cury, curTh,
					goalx, goaly, goalth,
					DistEps, AngEps,
					LookaheadMaxdist,
					LookaheadDistweight,
					TvMin, TvMax,
					AvMin, AvMax,
					AMin, AMax);
  if(DoTiming) {
    gettimeofday(&timeafter,&tzdummy);
    int best_seconds = timeafter.tv_sec-timebefore.tv_sec;
    int best_microseconds = timeafter.tv_usec-timebefore.tv_usec;
    double best_tt = best_seconds*1.0+(.000001)*(best_microseconds*1.0);
    //std::cout << "DiffDrive planning took " << best_tt << " seconds\n";
  }
  return res;
  
}
			     
double WavefrontPlanner::WorldCenterX() const {
  return _cenx;
}

double WavefrontPlanner::WorldCenterY() const {
  return _ceny;
}

double WavefrontPlanner::SizeX() const {
  return((_masterPlan->size_x*1.0)*_masterPlan->scale);
}

double WavefrontPlanner::SizeY() const {
  return((_masterPlan->size_y*1.0)*_masterPlan->scale);
}

plan_t* WavefrontPlanner::GetMasterPlan() const {
  return _masterPlan;
}

plan_t* WavefrontPlanner::GetActivePlan() const {
  return _activePlan;
}

WavefrontPlanner::PlanEntry* WavefrontPlanner::GetPlan(double gx, double gy) {
  for(std::list<PlanEntry*>::iterator it = _planEntries.begin();
      it != _planEntries.end();
      it++) {
    if(abs(gx-(*it)->goal_x) < VerySmall &&
       abs(gy-(*it)->goal_y) < VerySmall) {
      return ((*it));
    }
  }
  return NULL;
}

// bool WavefrontPlanner::NeedNewGlobalPlan(double gx, double gy) {
//   if(_lastX == gx && _lastY == gy) return false;
//   return true;
// }

WavefrontPlanner::PlanEntry* WavefrontPlanner::CreateNewPlan(double gx, double gy) {
  PlanEntry* pe = new PlanEntry(gx, gy, _masterPlan);
  _planEntries.push_back(pe);
  return pe;
}

WavefrontPlanner::PlanEntry::PlanEntry(double gx, double gy, const plan_t* mp) {
  goal_x = gx;
  goal_y = gy;
  plan = copy_plan_t(mp);
}

WavefrontPlanner::PlanEntry::~PlanEntry() {
  plan_free(plan);
}

void WavefrontPlanner::UpdatePlanEntryObsIfNeeded(PlanEntry* pe) {
  //if we have newer data
  if(_lastTs > pe->obsTime) {
    plan_set_obstacles(pe->plan, _lastObs, _lastNum);
  }
}

void WavefrontPlanner::SetObstacles(unsigned long long obstime, double* obs, size_t num) {
  struct timeval timebefore;
  struct timeval timeafter;
  struct timezone tzdummy;

  if(DoTiming) {
    gettimeofday(&timebefore,&tzdummy);
  }

  if(_lastObs) {
    delete[] _lastObs;
    _lastObs = NULL;
  }
  //two points per num
  _lastObs = new double[num*2];
  _lastNum = num;
  memcpy(_lastObs, obs, num*2*sizeof(double));
  _lastTs = obstime;

  // plan_set_obstacles(_masterPlan, obs, num);

  //  for(std::list<PlanEntry*>::iterator it = _planEntries.begin();
  //    it != _planEntries.end();
  //    it++) {
  //  plan_set_obstacles((*it)->plan, obs, num);
  // }

  if(DoTiming) {
    gettimeofday(&timeafter,&tzdummy);
    int best_seconds = timeafter.tv_sec-timebefore.tv_sec;
    int best_microseconds = timeafter.tv_usec-timebefore.tv_usec;
    double best_tt = best_seconds*1.0+(.000001)*(best_microseconds*1.0);
    std::cout << "Setting obstacles took " << best_tt << " seconds\n";
    draw_map(_masterPlan, 0.0, 0.0, "obs.png");
  }
}
