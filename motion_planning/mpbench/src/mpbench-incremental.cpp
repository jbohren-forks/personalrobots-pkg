/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "setup.hpp"
#include "gfx.hpp"
#include <costmap_2d/costmap_2d.h>
#include <mpglue/sbpl_util.hh>
#include <mpglue/environment.h>
#include <sfl/util/numeric.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <iomanip>

extern "C" {
#include <err.h>
}

using namespace mpbench;
using namespace mpglue;
using namespace boost;
using namespace std;

static void cleanup();
static void usage(ostream & os);
static void parse_options(int argc, char ** argv);
static void create_setup();
static void run_tasks();
static void print_summary();
static void print_gnuplot();

static footprint_t const & getFootprint();
static std::string baseFilename();

static bool enableGfx;
static string plannerType;
static string costmapType;
static string environmentType;
static SBPLBenchmarkOptions opt;
static bool websiteMode;
static double allocTimeMS;

static shared_ptr<SBPLBenchmarkSetup> setup;
static shared_ptr<Environment> environment;
static shared_ptr<SBPLPlannerManager> plannerMgr;
static shared_ptr<ostream> logos;

static shared_ptr<footprint_t> footprint;

static planList_t planList;

typedef map<waypoint_plan_t const *, SBPLPlannerStatsEntry> successStats_t;
typedef map<size_t, SBPLPlannerStatsEntry> failureStats_t;
static successStats_t successStats;
static failureStats_t failureStats;

int main(int argc, char ** argv)
{
  if (0 != atexit(cleanup))
    errx(EXIT_FAILURE, "atexit() failed");
  parse_options(argc, argv);
  string logFilename(baseFilename() + ".txt");
  logos.reset(new ofstream(logFilename.c_str()));
  create_setup();
  run_tasks();
  print_summary();
  print_gnuplot();
  if (enableGfx)
    display(gfx::Configuration(*setup,
			       *environment,
			       opt,
			       websiteMode,
			       baseFilename(),
			       getFootprint(),
			       planList,
			       "3DKIN" != environmentType,
			       *logos),
	    opt.name.c_str(),
	    2, // hack: layoutID
	    &argc, argv);
}


void cleanup()
{
  if (logos)
    *logos << "byebye!\n" << flush;
  setup.reset();
  environment.reset();
  plannerMgr.reset();
  logos.reset();
  planList.clear();
}


void usage(ostream & os)
{
  os << "options:\n"
     << "   -h               help (this message)\n"
     << "   -p  <name>       name of the SBPL planner\n"
     << "   -m  <name>       name of the costmap implementation\n"
     << "   -e  <name>       environment representation type\n"
     << "   -s  <name>       name of the setup\n"
     << "   -r  <cellsize>   set grid resolution\n"
     << "   -i  <in-radius>  set INSCRIBED radius\n"
     << "   -c  <out-radius> set CIRCUMSCRIBED radius\n"
     << "   -I  <inflate-r>  set INFLATION radius\n"
     << "   -a  <time [ms]>  allocated time for (incremental) replan in milliseconds\n"
     << "   -d  <doorwidth>  set width of doors (office setups)\n"
     << "   -H  <hallwidth>  set width of hallways (office setups)\n"
     << "   -n  <filename>   Net PGM file to load (for -s pgm)\n"
     << "   -g  <gray>       cutoff for obstacles in PGM images (for -s pgm)\n"
     << "   -X               dump filename base to stdout (use as last option)\n"
     << "   -W               run in website generation mode\n";
}


static string summarizeOptions()
{
  ostringstream os;
  os << "-p" << canonicalPlannerName(plannerType)
     << "-s" << opt.name
     << "-m" << costmapType
     << "-e" << canonicalEnvironmentName(environmentType)
     << "-r" << (int) rint(1e3 * opt.resolution)
     << "-i" << (int) rint(1e3 * opt.inscribed_radius)
     << "-c" << (int) rint(1e3 * opt.circumscribed_radius)
     << "-I" << (int) rint(1e3 * opt.inflation_radius)
     << "-a" << (int) rint(allocTimeMS);
  if ("pgm" != opt.name)
    os << "-d" << (int) rint(1e3 * opt.door_width)
       << "-H" << (int) rint(1e3 * opt.hall_width);
  return os.str();
}


std::string baseFilename()
{
  return "mpbench-incremental-" + summarizeOptions();
}


static void sanitizeOptions()
{
  if (opt.inscribed_radius > opt.circumscribed_radius)
    opt.circumscribed_radius = opt.inscribed_radius;
  if (opt.inscribed_radius > opt.inflation_radius)
    opt.inflation_radius = opt.inscribed_radius;
  if (opt.circumscribed_radius > opt.inflation_radius)
    opt.inflation_radius = opt.circumscribed_radius;
  plannerType = canonicalPlannerName(plannerType);
  environmentType = canonicalEnvironmentName(environmentType);
}


void parse_options(int argc, char ** argv)
{
  // these should become options
  enableGfx = true;
  
  // default values for options
  plannerType = "ARAPlanner";
  costmapType = "costmap_2d";
  environmentType = "2D";
  websiteMode = false;
  allocTimeMS = 50;
  // most other options handled through SBPLBenchmarkOptions
  
  for (int ii(1); ii < argc; ++ii) {
    if ((strlen(argv[ii]) < 2) || ('-' != argv[ii][0])) {
      cerr << argv[0] << ": problem with option '" << argv[ii] << "'\n";
      usage(cerr);
      exit(EXIT_FAILURE);
    }
    else
      switch (argv[ii][1]) {
	
      case 'h':
	usage(cout);
	exit(EXIT_SUCCESS);
	
      case 'p':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -p requires a name argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	plannerType = argv[ii];
 	break;
	
      case 's':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -s requires a name argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	opt.name = argv[ii];
 	break;
	
      case 'm':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -m requires a name argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	costmapType = argv[ii];
 	break;
	
      case 'e':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -e requires a name argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	environmentType = argv[ii];
 	break;
	
      case 'r':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -r requires a cellsize argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.resolution;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading cellsize argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'i':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -i requires inscribed radius argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.inscribed_radius;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading inscribed radius argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'c':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -c requires circumscribed radius argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.circumscribed_radius;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading circumscribed radius argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'I':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -I requires inflation radius argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.inflation_radius;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading inflation radius argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'd':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -d requires a doorwidth argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.door_width;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading doorwidth argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'a':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -a requires a time [ms] argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> allocTimeMS;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading allocTimeMS argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'H':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -H requires a hallwidth argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.hall_width;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading hallwidth argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'n':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -n requires a filename argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	opt.pgm_filename = argv[ii];
 	break;
	
      case 'g':
	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -g requires a gray argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.obstacle_gray;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading gray argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'X':
	sanitizeOptions();
	cout << baseFilename() << "\n";
	exit(EXIT_SUCCESS);
	
      case 'W':
	websiteMode = true;
	break;
	
      default:
	cerr << argv[0] << ": invalid option '" << argv[ii] << "'\n";
	usage(cerr);
	exit(EXIT_FAILURE);
      }
  }
  
  sanitizeOptions();
}


void create_setup()
{
  if ("costmap_2d" == costmapType)
    opt.use_sfl_cost = false;
  else if ("sfl" == costmapType)
    opt.use_sfl_cost = true;
  else
    errx(EXIT_FAILURE,
	 "create_setup(): unknown costmapType \"%s\", use costmap_2d or sfl",
	 costmapType.c_str());
  
  *logos << "creating setup \"" << opt.name << "\"\n" << flush;
  setup.reset(createBenchmark(opt, logos.get(), 0));
  if ( ! setup)
    errx(EXIT_FAILURE, "could not create setup with name \"%s\"", opt.name.c_str());
  setup->dumpDescription(*logos, "", "  ");
  *logos << flush
	 << "creating environment of type " << environmentType
	 << " for map type " << costmapType << "\n" << flush;
  
  if ("2D" == environmentType) {
    unsigned char const
      obst_cost_thresh(costmap_2d::CostMap2D::INSCRIBED_INFLATED_OBSTACLE);
    environment.reset(new Environment2D(setup->getCostmap().get(), false,
					       setup->getIndexTransform().get(), false,
					       0, 0, // start INDEX (ix, iy)
					       0, 0, // goal INDEX x (ix, iy)
					       obst_cost_thresh));
  }
  else if ("3DKIN" == environmentType) {
    unsigned char const
      obst_cost_thresh(costmap_2d::CostMap2D::LETHAL_OBSTACLE);
    // how about making these configurable?
    double const goaltol_x(0.5 * opt.inscribed_radius);
    double const goaltol_y(0.5 * opt.inscribed_radius);
    double const goaltol_theta(M_PI);
    double const nominalvel_mpersecs(0.6); // human leisurely walking speed
    double const timetoturn45degsinplace_secs(0.6); // guesstimate
    environment.reset(new Environment3DKIN(setup->getCostmap().get(), false,
							setup->getIndexTransform().get(), false,
							obst_cost_thresh,
							0, 0, 0, // start POSE (x, y, th)
							0, 0, 0, // goal POSE (x, y, th)
							goaltol_x, goaltol_y, goaltol_theta,
							getFootprint(), nominalvel_mpersecs,
							timetoturn45degsinplace_secs));
    static bool const do_sanity_check(false);
    if (do_sanity_check) {
      Costmap const & cm(*setup->getCostmap());
      bool sane(true);
      cout << "3DKIN costmap sanity check:\n"
	   << " * correct obstacle\n"
	   << " O missing obstacle\n"
	   << " . correct freespace\n"
	   << " x missing freespace\n"
	   << " @ missing information\n";
      for (ssize_t ix(0); ix < cm.getXEnd(); ++ix) {
	cout << "  ";
	for (ssize_t iy(0); iy < cm.getYEnd(); ++iy) {
	  int cost;
	  if ( !  cm.getCost(ix, iy, &cost)) {
	    cout << "@";
	    sane = false;
	  }
	  else if (cost >= obst_cost_thresh) {
	    if (environment->IsObstacle(ix, iy))
	      cout << "*";
	    else {
	      cout << "O";
	      sane = false;
	    }
	  }
	  else {
	    if ( ! environment->IsObstacle(ix, iy))
	      cout << ".";
	    else {
	      cout << "x";
	      sane = false;
	    }
	  }
	}
	cout << "\n";
      }
      if ( ! sane)
	errx(EXIT_FAILURE, "3DKIN environment is not sane");
    }
  }
  else {
    errx(EXIT_FAILURE, "invalid environmentType \"%s\", use 2D or 3DKIN", environmentType.c_str());
  }
  
  MDPConfig mdpConfig;
  if ( ! environment->InitializeMDPCfg(&mdpConfig))
    errx(EXIT_FAILURE, "environment->InitializeMDPCfg() failed on environment %s",
	 environment->getName().c_str());
  *logos << "  environment name: " << environment->getName() << "\n" << flush;
  
  *logos << "creating planner manager\n" << flush;
  bool const forwardsearch(false);
  plannerMgr.reset(new SBPLPlannerManager(environment->getDSI(), forwardsearch, &mdpConfig));
  if ( ! plannerMgr->select(plannerType, false, logos.get()))
    errx(EXIT_FAILURE, "plannerMgr->select(%s) failed", plannerType.c_str());
  *logos << "  planner name: " << plannerMgr->getName() << "\n" << flush;
}


void run_tasks()
{
  *logos << "running tasks\n" << flush;
  
  IndexTransform const & it(*setup->getIndexTransform());
  SBPLBenchmarkSetup::tasklist_t const & tasklist(setup->getTasks());
  for (size_t ii(0); ii < tasklist.size(); ++ii) {
    planBundle_t bundle;
    SBPLPlannerStatsEntry statsEntry(plannerMgr->getName(), environment->getName());
    SBPLBenchmarkSetup::task const & task(tasklist[ii]);
    
    *logos << "\n  task " << ii << ": " << task.description << "\n" << flush;
      
    // set start
    statsEntry.start.x = task.start_x;
    statsEntry.start.y = task.start_y;
    statsEntry.start.th = task.start_th;
    it.globalToIndex(statsEntry.start.x, statsEntry.start.y,
		     &statsEntry.startIx, &statsEntry.startIy);
    environment->SetStart(statsEntry.start);
    statsEntry.startState = environment->GetStateFromPose(statsEntry.start);
    if (0 > statsEntry.startState)
      errx(EXIT_FAILURE, "invalid start state ID %d from pose (%+8.3f, %+8.3f)",
	   statsEntry.startState, statsEntry.start.x, statsEntry.start.y);
    int status(plannerMgr->set_start(statsEntry.startState));
    if (1 != status)
      errx(EXIT_FAILURE, "failed to set start state ID %d from (%ud, %ud): %d\n",
	   statsEntry.startState, statsEntry.startIx, statsEntry.startIy, status);
      
    // set goal
    statsEntry.goal.x = task.goal_x;
    statsEntry.goal.y = task.goal_y;
    statsEntry.goal.th = task.goal_th;
    it.globalToIndex(statsEntry.goal.x, statsEntry.goal.y,
		     &statsEntry.goalIx, &statsEntry.goalIy);
    environment->SetGoal(statsEntry.goal);
    statsEntry.goalState = environment->GetStateFromPose(statsEntry.goal);
    if (0 > statsEntry.goalState)
      errx(EXIT_FAILURE, "invalid goal state ID %d from pose (%+8.3f, %+8.3f)",
	   statsEntry.goalState, statsEntry.goal.x, statsEntry.goal.y);
    status = plannerMgr->set_goal(statsEntry.goalState);
    if (1 != status)
      errx(EXIT_FAILURE, "failed to set goal state ID %d from (%ud, %ud): %d\n",
	   statsEntry.goalState, statsEntry.goalIx, statsEntry.goalIy, status);
    
    // plan several solutions:
    // - initially just the 1st path we come across
    // - subsequently with some allocated timeslice
    // - until there is no change in the returned path

    statsEntry.plan_length_m = 0; // just in case the first replan() fails
    statsEntry.plan_angle_change_rad = 0;
    vector<int> prevSolution;
    double prevEpsilon(-1);
    double cumul_allocated_time(0);
    double cumul_actual_time_wall(0);
    double cumul_actual_time_user(0);
    double cumul_actual_time_system(0);
    int cumul_expands(0);
    for (size_t jj(0); true; ++jj) {
      
      // Handle the first iteration specially.
      if (0 == jj) {
	statsEntry.stop_at_first_solution = true;
	statsEntry.plan_from_scratch = task.from_scratch;
	statsEntry.allocated_time_sec = numeric_limits<double>::max();
      }
      else {
	statsEntry.stop_at_first_solution = false;
	statsEntry.plan_from_scratch = false;
	statsEntry.allocated_time_sec = 1e-3 * allocTimeMS;
      }
      
      vector<int> solution;
      statsEntry.status = plannerMgr->replan(statsEntry.stop_at_first_solution,
					     statsEntry.plan_from_scratch,
					     statsEntry.allocated_time_sec,
					     &statsEntry.actual_time_wall_sec,
					     &statsEntry.actual_time_user_sec,
					     &statsEntry.actual_time_system_sec,
					     &statsEntry.number_of_expands,
					     &statsEntry.solution_cost,
					     &statsEntry.solution_epsilon,
					     &solution);
      
      // forget about this task if we got a planning failure
      if ((1 != statsEntry.status) // planners should provide status
	  || (1 >= solution.size()) // but sometimes they do not
	  ) {
	statsEntry.logStream(*logos, "  FAILURE", "    ");
	*logos << flush;
	failureStats.insert(make_pair(ii, statsEntry));
	break;
      }
      
      // A little hack to make the initial solution appear as if it
      // had used exactly the allocated time. Actually, it gets
      // allocated a practically infinite amount of time, which is
      // not useful for cumulating in the statistics. Also, it seems
      // that SBPLPlanner uses wall time, not user time, to limit its
      // search.
      
      if (0 == jj)
	statsEntry.allocated_time_sec = statsEntry.actual_time_wall_sec;
      
      if (0 < statsEntry.number_of_expands)
	cumul_expands += statsEntry.number_of_expands;
      
      // detect whether we got the same result as before, in which
      // case we're done
      if (statsEntry.solution_epsilon > 0) {
       	// see if epsilon changed "perceptibly"
       	// XXXX warning: hardcoded threshold
       	if ((prevEpsilon > 0) && (fabs(prevEpsilon - statsEntry.solution_epsilon) < 1e-9)) {
	  statsEntry.allocated_time_sec = cumul_allocated_time;
	  statsEntry.actual_time_wall_sec = cumul_actual_time_wall;
	  statsEntry.actual_time_user_sec = cumul_actual_time_user;
	  statsEntry.actual_time_system_sec = cumul_actual_time_system;
	  statsEntry.number_of_expands = cumul_expands;
	  statsEntry.logStream(*logos, "  FINAL: epsilon did not change (below are cumulated times and expands)", "    ");
	  *logos << flush;
	  //// do NOT add to overall stats because of cumulated times
	  // plannerStats.push_back(statsEntry);
       	  break;
	}
      }
      else {
	// use brute force comparison with previous path
	if (prevSolution.size() == solution.size()) {
	  bool same(true);
	  for (size_t kk(0); kk < prevSolution.size(); ++kk)
	    if (prevSolution[kk] != solution[kk]) {
	      same = false;
	      break;
	    }
	  if (same) {
	    statsEntry.allocated_time_sec = cumul_allocated_time;
	    statsEntry.actual_time_wall_sec = cumul_actual_time_wall;
	    statsEntry.actual_time_user_sec = cumul_actual_time_user;
	    statsEntry.actual_time_system_sec = cumul_actual_time_system;
	    statsEntry.number_of_expands = cumul_expands;
	    statsEntry.logStream(*logos, "  FINAL: path states did not change (below are cumulated times and expands)", "    ");
	    *logos << flush;
	    //// do NOT add to overall stats because of cumulated times
	    // plannerStats.push_back(statsEntry);
	    break;
	  }
	}
      }
      
      // update the cumulated time measurements
      cumul_allocated_time += statsEntry.allocated_time_sec;
      cumul_actual_time_wall += statsEntry.actual_time_wall_sec;
      cumul_actual_time_user += statsEntry.actual_time_user_sec;
      cumul_actual_time_system += statsEntry.actual_time_system_sec;
      
      // save this plan, and prepare for the next round of
      // incremental planning
      shared_ptr<waypoint_plan_t> plan(new waypoint_plan_t());
      convertPlan(*environment, solution, plan.get(),
		  &statsEntry.plan_length_m,
		  &statsEntry.plan_angle_change_rad,
		  0 // XXXX if 3DKIN we actually want something here
		  );
      bundle.push_back(plan);
      
      if (0 == jj)
	statsEntry.logStream(*logos, "  FIRST_SOLUTION", "    ");
      else
	statsEntry.logStream(*logos, "  IMPROVED", "    ");
      *logos << flush;
      successStats.insert(make_pair(plan.get(), statsEntry));
      
      prevSolution.swap(solution);
      prevEpsilon = statsEntry.solution_epsilon;
    } // end loop over incremental solutions
    
    // Well... this ends up copying a std::vector of boost::shared_ptr
    // instances, could probably be smarter about it. Also we will end
    // up storing empty bundles, which is actually what we want
    // because for failed tasks we still want to plot the start and
    // goal poses (see gfx.cpp), but there must be a neater solution
    // to this.
    planList.insert(make_pair(ii, bundle));
    
  } // end loop over tasks
}


// there surely is a std function that does this but I have not searched very thoroughly
static string timeStr(double time_sec)
{
  ostringstream os;
  if (1e-6 > fabs(time_sec))
    os << time_sec << " s";
  else if (500e-6 > fabs(time_sec))
    os << 1e6 * time_sec << " us";
  else if (500e-3 > fabs(time_sec))
    os << 1e3 * time_sec << " ms";
  else if (60 > fabs(time_sec))
    os << time_sec << " s";
  else
    os << floor(time_sec / 60) << " min " << fabs(fmod(time_sec, 60)) << " s";
  return os.str();    
}


void print_summary()
{
  if ( ! websiteMode) {
    *logos << "sorry, summary only implemented for HTML mode\n";
    return;
  }  
  string htmlFilename(baseFilename() + ".html");
  ofstream htmlOs(htmlFilename.c_str());
  if ( ! htmlOs) {
    cout << "sorry, could not open file " << htmlFilename << "\n";
    return;
  }

  //////////////////////////////////////////////////
  // cumulated state expansions
  
  htmlOs << "<table border=\"1\" cellpadding=\"2\">\n"
	 << "<tr><th colspan=\"5\">cumulated state expansions</th></tr>\n"
	 << "<tr><td>task</td><td>init</td><td>final</td><td>delta</td><td>% delta</td></tr>\n";
  for (planList_t::const_iterator ip(planList.begin()); ip != planList.end(); ++ip) {
    htmlOs << "<tr><td>" << ip->first << "</td>";
    planBundle_t const & bundle(ip->second);
    if (bundle.empty()) {
      htmlOs << "<td colspan=\"4\"><em>no solution</em></td></tr>\n";
      continue;
    }
    planBundle_t::const_iterator ib(bundle.begin());
    waypoint_plan_t const * initialPlan(ib->get());
    successStats_t::const_iterator initialStats(successStats.find(initialPlan));
    if (successStats.end() == initialStats) {
      htmlOs << "<td colspan=\"4\"><em>error: no stats for initial solution</em></td></tr>\n";
      continue;
    }
    if (1 == bundle.size()) {
      htmlOs << "<td colspan=\"2\">" << initialStats->second.number_of_expands << "</td>"
	     << "<td colspan=\"2\">N/A</td></tr>\n";
      continue;
    }
    int cumul_expands(initialStats->second.number_of_expands);
    bool error(false);
    for (++ib; ib != bundle.end(); ++ib) {
      successStats_t::const_iterator stats(successStats.find(ib->get()));
      if (successStats.end() == stats) {
	error = true;
	break;
      }
      cumul_expands += stats->second.number_of_expands;
    }
    if (error) {
      htmlOs << "<td colspan=\"3\"><em>error: missing stats</em></td></tr>\n";
      continue;
    }
    int const delta_expands(cumul_expands - initialStats->second.number_of_expands);
    htmlOs << "<td>" << initialStats->second.number_of_expands << "</td>"
	   << "<td>" << cumul_expands << "</td>"
	   << "<td>" << delta_expands << "</td>"
	   << "<td>" << 1.0e2 * delta_expands / initialStats->second.number_of_expands
	   << "</td></tr>\n";
  }
  
  //////////////////////////////////////////////////
  // expansions per second
  
  htmlOs << "<tr><th colspan=\"5\">expansions speed [1/s] (wall clock)</th></tr>\n"
	 << "<tr><td>task</td><td>init</td><td>final</td><td>delta</td><td>% delta</td></tr>\n";
  for (planList_t::const_iterator ip(planList.begin()); ip != planList.end(); ++ip) {
    htmlOs << "<tr><td>" << ip->first << "</td>";
    planBundle_t const & bundle(ip->second);
    if (bundle.empty()) {
      htmlOs << "<td colspan=\"4\"><em>no solution</em></td></tr>\n";
      continue;
    }
    planBundle_t::const_iterator ib(bundle.begin());
    waypoint_plan_t const * initialPlan(ib->get());
    successStats_t::const_iterator initialStats(successStats.find(initialPlan));
    if (successStats.end() == initialStats) {
      htmlOs << "<td colspan=\"4\"><em>error: no stats for initial solution</em></td></tr>\n";
      continue;
    }
    double cumul_time(initialStats->second.actual_time_wall_sec);
    double cumul_expands(initialStats->second.number_of_expands);
    double const init_speed(cumul_expands / cumul_time);
    if (1 == bundle.size()) {
      htmlOs << "<td colspan=\"2\">" << init_speed << "</td>"
	     << "<td colspan=\"2\">N/A</td></tr>\n";
      continue;
    }
    bool error(false);
    for (++ib; ib != bundle.end(); ++ib) {
      successStats_t::const_iterator stats(successStats.find(ib->get()));
      if (successStats.end() == stats) {
	error = true;
	break;
      }
      cumul_time += stats->second.actual_time_wall_sec;
      cumul_expands += stats->second.number_of_expands;
    }
    if (error) {
      htmlOs << "<td colspan=\"3\"><em>error: missing stats</em></td></tr>\n";
      continue;
    }
    double const final_speed(cumul_expands / cumul_time);
    double const delta_speed(final_speed - init_speed);
    htmlOs << "<td>" << init_speed << "</td>"
	   << "<td>" << final_speed << "</td>"
	   << "<td>" << delta_speed << "</td>"
	   << "<td>" << 1.0e2 * delta_speed / init_speed
	   << "</td></tr>\n";
  }
  
  //////////////////////////////////////////////////
  // consumed planning time
  
  htmlOs << "<tr><th colspan=\"5\">consumed planning time (wall clock)</th></tr>\n"
	 << "<tr><td>task</td><td>init</td><td>final</td><td>delta</td><td>% delta</td></tr>\n";
  for (planList_t::const_iterator ip(planList.begin()); ip != planList.end(); ++ip) {
    htmlOs << "<tr><td>" << ip->first << "</td>";
    planBundle_t const & bundle(ip->second);
    if (bundle.empty()) {
      htmlOs << "<td colspan=\"4\"><em>no solution</em></td></tr>\n";
      continue;
    }
    planBundle_t::const_iterator ib(bundle.begin());
    waypoint_plan_t const * initialPlan(ib->get());
    successStats_t::const_iterator initialStats(successStats.find(initialPlan));
    if (successStats.end() == initialStats) {
      htmlOs << "<td colspan=\"4\"><em>error: no stats for initial solution</em></td></tr>\n";
      continue;
    }
    double cumul(initialStats->second.actual_time_wall_sec);
    if (1 == bundle.size()) {
      htmlOs << "<td colspan=\"2\">" << timeStr(cumul) << "</td>"
	     << "<td colspan=\"2\">N/A</td></tr>\n";
      continue;
    }
    bool error(false);
    for (++ib; ib != bundle.end(); ++ib) {
      successStats_t::const_iterator stats(successStats.find(ib->get()));
      if (successStats.end() == stats) {
	error = true;
	break;
      }
      cumul += stats->second.actual_time_wall_sec;
    }
    if (error) {
      htmlOs << "<td colspan=\"3\"><em>error: missing stats</em></td></tr>\n";
      continue;
    }
    double const delta(cumul - initialStats->second.actual_time_wall_sec);
    htmlOs << "<td>" << timeStr(initialStats->second.actual_time_wall_sec) << "</td>"
	   << "<td>" << timeStr(cumul) << "</td>"
	   << "<td>" << timeStr(delta) << "</td>"
	   << "<td>" << 1.0e2 * delta / initialStats->second.actual_time_wall_sec
	   << "</td></tr>\n";
  }
  
  //////////////////////////////////////////////////
  // epsilon
  
  htmlOs << "<tr><th colspan=\"5\">epsilon</th></tr>\n"
	 << "<tr><td>task</td><td>init</td><td>final</td><td>delta</td><td>% delta</td></tr>\n";
  for (planList_t::const_iterator ip(planList.begin()); ip != planList.end(); ++ip) {
    htmlOs << "<tr><td>" << ip->first << "</td>";
    planBundle_t const & bundle(ip->second);
    if (bundle.empty()) {
      htmlOs << "<td colspan=\"4\"><em>no solution</em></td></tr>\n";
      continue;
    }
    waypoint_plan_t const * initialPlan(bundle.begin()->get());
    successStats_t::const_iterator initialStats(successStats.find(initialPlan));
    if (successStats.end() == initialStats) {
      htmlOs << "<td colspan=\"4\"><em>error: no stats for initial solution</em></td></tr>\n";
      continue;
    }
    if (1 == bundle.size()) {
      htmlOs << "<td colspan=\"2\">" << initialStats->second.solution_epsilon << "</td>"
	     << "<td colspan=\"2\">N/A</td></tr>\n";
      continue;
    }
    waypoint_plan_t const * finalPlan(bundle.rbegin()->get());
    successStats_t::const_iterator finalStats(successStats.find(finalPlan));
    if (successStats.end() == finalStats) {
      htmlOs << "<td colspan=\"4\"><em>error: no stats for final solution</em></td></tr>\n";
      continue;
    }
    double const
      delta(finalStats->second.solution_epsilon - initialStats->second.solution_epsilon);
    htmlOs << "<td>" << initialStats->second.solution_epsilon << "</td>"
	   << "<td>" << finalStats->second.solution_epsilon << "</td>"
	   << "<td>" << delta << "</td>"
	   << "<td>" << 1.0e2 * delta / initialStats->second.solution_epsilon << "</td></tr>\n";
  }
  
  //////////////////////////////////////////////////
  // plan length
  
  htmlOs << "<tr><th colspan=\"5\">plan length [m]</th></tr>\n"
	 << "<tr><td>task</td><td>init</td><td>final</td><td>delta</td><td>% delta</td></tr>\n";
  for (planList_t::const_iterator ip(planList.begin()); ip != planList.end(); ++ip) {
    htmlOs << "<tr><td>" << ip->first << "</td>";
    planBundle_t const & bundle(ip->second);
    if (bundle.empty()) {
      htmlOs << "<td colspan=\"4\"><em>no solution</em></td></tr>\n";
      continue;
    }
    waypoint_plan_t const * initialPlan(bundle.begin()->get());
    successStats_t::const_iterator initialStats(successStats.find(initialPlan));
    if (successStats.end() == initialStats) {
      htmlOs << "<td colspan=\"4\"><em>error: no stats for initial solution</em></td></tr>\n";
      continue;
    }
    if (1 == bundle.size()) {
      htmlOs << "<td colspan=\"2\">" << initialStats->second.plan_length_m << "</td>"
	     << "<td colspan=\"2\">N/A</td></tr>\n";
      continue;
    }
    waypoint_plan_t const * finalPlan(bundle.rbegin()->get());
    successStats_t::const_iterator finalStats(successStats.find(finalPlan));
    if (successStats.end() == finalStats) {
      htmlOs << "<td colspan=\"4\"><em>error: no stats for final solution</em></td></tr>\n";
      continue;
    }
    double const delta(  finalStats->second.plan_length_m
		       - initialStats->second.plan_length_m);
    htmlOs << "<td>" << initialStats->second.plan_length_m << "</td>"
	   << "<td>" << finalStats->second.plan_length_m << "</td>"
	   << "<td>" << delta << "</td>"
	   << "<td>" << 1.0e2 * delta / initialStats->second.plan_length_m
	   << "</td></tr>\n";
  }

  //////////////////////////////////////////////////
  // plan tangent change
  
  htmlOs << "<tr><th colspan=\"5\">plan tangent change [deg]</th></tr>\n"
	 << "<tr><td>task</td><td>init</td><td>final</td><td>delta</td><td>% delta</td></tr>\n";
  for (planList_t::const_iterator ip(planList.begin()); ip != planList.end(); ++ip) {
    htmlOs << "<tr><td>" << ip->first << "</td>";
    planBundle_t const & bundle(ip->second);
    if (bundle.empty()) {
      htmlOs << "<td colspan=\"4\"><em>no solution</em></td></tr>\n";
      continue;
    }
    waypoint_plan_t const * initialPlan(bundle.begin()->get());
    successStats_t::const_iterator initialStats(successStats.find(initialPlan));
    if (successStats.end() == initialStats) {
      htmlOs << "<td colspan=\"4\"><em>error: no stats for initial solution</em></td></tr>\n";
      continue;
    }
    if (1 == bundle.size()) {
      htmlOs << "<td colspan=\"2\">"
	     << 180 * initialStats->second.plan_angle_change_rad / M_PI << "</td>"
	     << "<td colspan=\"2\">N/A</td></tr>\n";
      continue;
    }
    waypoint_plan_t const * finalPlan(bundle.rbegin()->get());
    successStats_t::const_iterator finalStats(successStats.find(finalPlan));
    if (successStats.end() == finalStats) {
      htmlOs << "<td colspan=\"4\"><em>error: no stats for final solution</em></td></tr>\n";
      continue;
    }
    double const delta(  finalStats->second.plan_angle_change_rad
		       - initialStats->second.plan_angle_change_rad);
    htmlOs << "<td>" << 180 * initialStats->second.plan_angle_change_rad / M_PI << "</td>"
	   << "<td>" << 180 * finalStats->second.plan_angle_change_rad / M_PI << "</td>"
	   << "<td>" << 180 * delta / M_PI << "</td>"
	   << "<td>" << 1.0e2 * delta / initialStats->second.plan_angle_change_rad
	   << "</td></tr>\n";
  }
  
  htmlOs << "</table>\n";
}


void print_gnuplot()
{
  string dataFilename(baseFilename() + ".data");
  string costhistFilename(baseFilename() + ".costhist");
  string plotFilename(baseFilename() + ".plot");
  ofstream plotOs(plotFilename.c_str());
  if ( ! plotOs) {
    cout << "sorry, could not open file " << plotFilename << "\n";
    return;
  }
  cout << "writing gnuplot script file: " << plotFilename << "\n";
  plotOs
    << "set terminal png large\n"
    // relative plan quality vs relative time
    << "set output \"" << baseFilename() << "--rqual-rtime.png\"\n"
    << "set xlabel \"cumul planning time [% of final] (wallclock)\"\n"
    << "set ylabel \"[% of final]\"\n"
    << "plot \"" << dataFilename << "\" using ($3):($15) title \"solution cost\" with lines, "
    << "\"" << dataFilename << "\" using ($3):($18) title \"plan length\" with lines, "
    << "\"" << dataFilename << "\" using ($3):($20) title \"plan rotation\" with lines\n"
    // absolute expansion speed vs relative time
    << "set output \"" << baseFilename() << "--speed-rtime.png\"\n"
    << "set xlabel \"cumul planning time [% of final] (wallclock)\"\n"
    << "set ylabel \"[1/s] (wallclock)\"\n"
    << "plot \"" << dataFilename << "\" using ($3):($10) title \"expansion speed\" with lines, "
    << "\"" << dataFilename << "\" using ($3):($11) title \"avg expansion speed\" with lines\n"
    // relative plan quality vs absolute time
    << "set output \"" << baseFilename() << "--rqual-atime.png\"\n"
    << "set xlabel \"cumul planning time [s] (wallclock)\"\n"
    << "set ylabel \"[% of final]\"\n"
    << "plot \"" << dataFilename << "\" using ($2):($15) title \"solution cost\" with lines, "
    << "\"" << dataFilename << "\" using ($2):($18) title \"plan length\" with lines, "
    << "\"" << dataFilename << "\" using ($2):($20) title \"plan rotation\" with lines\n"
    // absolute expansion speed vs absolute time
    << "set output \"" << baseFilename() << "--speed-atime.png\"\n"
    << "set xlabel \"cumul planning time [s] (wallclock)\"\n"
    << "set ylabel \"[1/s] (wallclock)\"\n"
    << "plot \"" << dataFilename << "\" using ($2):($10) title \"expansion speed\" with lines, "
    << "\"" << dataFilename << "\" using ($2):($11) title \"avg expansion speed\" with lines lw 2\n"
    // absolute solution cost vs absolute time
    << "set output \"" << baseFilename() << "--cost-atime.png\"\n"
    << "set xlabel \"cumul planning time [s] (wallclock)\"\n"
    << "set ylabel \"[cost]\"\n"
    << "plot \"" << dataFilename << "\" using ($2):($14) title \"solution cost\" with lines\n"
    // absolute plan length vs absolute time
    << "set output \"" << baseFilename() << "--length-atime.png\"\n"
    << "set xlabel \"cumul planning time [s] (wallclock)\"\n"
    << "set ylabel \"[m]\"\n"
    << "plot \"" << dataFilename << "\" using ($2):($17) title \"plan length\" with lines\n"
    // absolute plan rotation vs absolute time
    << "set output \"" << baseFilename() << "--rotation-atime.png\"\n"
    << "set xlabel \"cumul planning time [s] (wallclock)\"\n"
    << "set ylabel \"[rad]\"\n"
    << "plot \"" << dataFilename << "\" using ($2):($19) title \"plan rotation\" with lines\n"
    // costmap histogram
    << "set output \"" << baseFilename() << "--costhist.png\"\n"
    << "set xlabel \"cell cost\"\n"
    << "set ylabel \"log count\"\n"
    << "plot \"" << costhistFilename << "\" using ($1):($3) title \"log cell count\" with impulses lw 10\n";
  
  ofstream dataOs(dataFilename.c_str());
  if ( ! dataOs) {
    cout << "sorry, could not open file " << dataFilename << "\n";
    return;
  }
  cout << "writing gnuplot data file: " << dataFilename << "\n";
  
  dataOs << "# " << dataFilename << "\n"
	 << "# data file for gnuplot\n"
	 << "#\n"
	 << "# multi-data sets correspond to individual tasks\n";
  setup->dumpDescription(dataOs, "", "#   ");
  dataOs << "#\n"
	 << "# columns:\n"
	 << "#  1           time actual (wall) [s]\n"
	 << "#  2 cumulated time actual (wall) [s]\n"
	 << "#  3 cumulated time actual (wall) [% of final]\n"
	 << "#  4           time actual (user) [s]\n"
	 << "#  5 cumulated time actual (user) [s]\n"
	 << "#  6 cumulated time actual (user) [% of final]\n"
	 << "#  7           number of expands\n"
	 << "#  8 cumulated number of expands\n"
	 << "#  9 cumulated number of expands [% of final]\n"
	 << "# 10           expansion speed (wall) [1/s]\n"
	 << "# 11 cumul avg expansion speed (wall) [1/s]\n"
	 << "# 12           expansion speed (user) [1/s]\n"
	 << "# 13 cumul avg expansion speed (user) [1/s]\n"
	 << "# 14 solution cost\n"
	 << "# 15 solution cost [% of final]\n"
	 << "# 16 solution epsilon\n"
	 << "# 17 plan length [m]\n"
	 << "# 18 plan length [% of final]\n"
	 << "# 19 plan rotation [rad]\n"
	 << "# 20 plan rotation [% of final]\n";
  
  for (planList_t::const_iterator ip(planList.begin()); ip != planList.end(); ++ip) {
    dataOs << "\n\n";
    planBundle_t const & bundle(ip->second);
    if (bundle.empty()) {
      dataOs << "# no solution\n";
      continue;
    }
    
    double final_cumul_wall(0);
    double final_cumul_user(0);
    int final_cumul_expands(0);
    int final_cost(0);
    double final_length(0);
    double final_rotation(0);
    for (planBundle_t::const_iterator ib(bundle.begin()); ib != bundle.end(); ++ib) {
      successStats_t::const_iterator iStats(successStats.find(ib->get()));
      if (successStats.end() == iStats) {
	dataOs << "# error: no stats for solution\n";
	continue;
      }
      final_cumul_wall += iStats->second.actual_time_wall_sec;
      final_cumul_user += iStats->second.actual_time_user_sec;
      final_cumul_expands += iStats->second.number_of_expands;
      final_cost = iStats->second.solution_cost;
      final_length = iStats->second.plan_length_m;
      final_rotation = iStats->second.plan_angle_change_rad;
    }
    
    double cumul_wall(0);
    double cumul_user(0);
    int cumul_expands(0);
    for (planBundle_t::const_iterator ib(bundle.begin()); ib != bundle.end(); ++ib) {
      successStats_t::const_iterator iStats(successStats.find(ib->get()));
      if (successStats.end() == iStats) {
	dataOs << "# error: no stats for solution\n";
	continue;
      }
      cumul_wall += iStats->second.actual_time_wall_sec;
      cumul_user += iStats->second.actual_time_user_sec;
      cumul_expands += iStats->second.number_of_expands;
      dataOs
	// wall time
	<< iStats->second.actual_time_wall_sec << "\t"
	<< cumul_wall << "\t"
	<< 100 * cumul_wall / final_cumul_wall << "\t"
	// user time
	<< iStats->second.actual_time_user_sec << "\t"
	<< cumul_user << "\t"
	<< 100 * cumul_user / final_cumul_user << "\t"
	// expands
	<< iStats->second.number_of_expands << "\t"
	<< cumul_expands << "\t"
	<< 100 * cumul_expands / final_cumul_expands << "\t"
	// expansion speed
	<< iStats->second.number_of_expands / iStats->second.actual_time_wall_sec << "\t"
	<< cumul_expands / cumul_wall << "\t"
	<< iStats->second.number_of_expands / iStats->second.actual_time_user_sec << "\t"
	<< cumul_expands / cumul_user << "\t"
	// cost
	<< iStats->second.solution_cost << "\t"
	<< (100.0 * iStats->second.solution_cost) / final_cost << "\t"
	// epsilon
	<< iStats->second.solution_epsilon << "\t"
	// plan quality
	<< iStats->second.plan_length_m << "\t"
	<< 100 * iStats->second.plan_length_m / final_length << "\t"
	<< iStats->second.plan_angle_change_rad << "\t"
	<< 100 * iStats->second.plan_angle_change_rad / final_rotation << "\n";
    }
  }

  ofstream costhistOs(costhistFilename.c_str());
  if ( ! costhistOs) {
    cout << "sorry, could not open file " << costhistFilename << "\n";
    return;
  }
  cout << "writing gnuplot cost histogram file: " << costhistFilename << "\n";
  
  costhistOs << "# " << costhistFilename << "\n"
	     << "# cost histogram file for gnuplot\n"
	     << "#\n"
	     << "# columns:\n"
	     << "#  1 cell cost\n"
	     << "#  2 count\n"
	     << "#  3 log(count)\n";
  boost::shared_ptr<Costmap const> cm(setup->getCostmap());
  typedef map<int, size_t> hist_t;
  hist_t hist;
  for (ssize_t ix(cm->getXBegin()); ix < cm->getXEnd(); ++ix)
    for (ssize_t iy(cm->getYBegin()); iy < cm->getYEnd(); ++iy) {
      int cost;
      if ( ! cm->getCost(ix, iy, &cost))
	continue;
      hist_t::iterator ih(hist.find(cost));
      if (hist.end() == ih)
	hist.insert(make_pair(cost, (ssize_t) 1));
      else
	++ih->second;
    }
  for (hist_t::const_iterator ih(hist.begin()); ih != hist.end(); ++ih)
    costhistOs << ih->first << "\t"
	       << ih->second << "\t"
	       << log10((double) ih->second) << "\n";
}


footprint_t const & getFootprint()
{
  if ( ! footprint) {
    footprint.reset(new footprint_t());
    initSimpleFootprint(*footprint, opt.inscribed_radius, opt.circumscribed_radius);
  }
  return *footprint;
}
