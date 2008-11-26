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

// XXXX should be <sbpl_util/blah> or so
#include <sbpl_util.hh>
#include <environment_wrap.h>

#include <sfl/util/numeric.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <iomanip>

extern "C" {
#include <err.h>
}

using namespace ompl;
using namespace boost;
using namespace std;

static void cleanup();
static void usage(ostream & os);
static void parse_options(int argc, char ** argv);
static void create_setup();
static void run_tasks();
static void print_summary();

static footprint_t const & getFootprint();
static std::string baseFilename();

static bool enableGfx;
static string plannerType;
static string costmapType;
static string environmentType;
static SBPLBenchmarkOptions opt;
static string travmapFilename;
static string costmapFilename;
static bool websiteMode;

static shared_ptr<SBPLBenchmarkSetup> setup;
static shared_ptr<EnvironmentWrapper> environment;
static shared_ptr<SBPLPlannerManager> plannerMgr;
static shared_ptr<ostream> logos;

static shared_ptr<footprint_t> footprint;

static planList_t planList;

typedef vector<SBPLPlannerStatsEntry> plannerStats_t;
static plannerStats_t plannerStats;


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
	    1, // hack: layoutID
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
     << "   -d  <doorwidth>  set width of doors (office setups)\n"
     << "   -H  <hallwidth>  set width of hallways (office setups)\n"
     << "   -n  <filename>   Net PGM file to load (for -s pgm)\n"
     << "   -g  <gray>       cutoff for obstacles in PGM images (for -s pgm)\n"
     << "   -o  <filename>   write sfl::TraversabilityMap to file\n"
     << "   -O  <filename>   write costmap_2d::CostMap2D to file\n"
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
     << "-d" << (int) rint(1e3 * opt.door_width)
     << "-H" << (int) rint(1e3 * opt.hall_width);
  return os.str();
}


std::string baseFilename()
{
  return "mpbench-" + summarizeOptions();
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
  travmapFilename = "";
  costmapFilename = "";
  websiteMode = false;
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
	
      case 'o':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -o requires a filename argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	travmapFilename = argv[ii];
 	break;
	
      case 'O':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -O requires a filename argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	costmapFilename = argv[ii];
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
  {
    shared_ptr<ostream> dump_os;
    if ( ! travmapFilename.empty()) {
      dump_os.reset(new ofstream(travmapFilename.c_str()));
      if ( ! (*dump_os)) {
	*logos << "could not open travmap file " << travmapFilename << "\n" << flush;
	dump_os.reset();
      }
    }
    setup.reset(createBenchmark(opt, logos.get(), dump_os.get()));
    if ( ! setup)
      errx(EXIT_FAILURE, "could not create setup with name \"%s\"", opt.name.c_str());
    if ( ! costmapFilename.empty()) {
      dump_os.reset(new ofstream(costmapFilename.c_str()));
      if ( ! (*dump_os))
	*logos << "could not open costmap file " << costmapFilename << "\n" << flush;
      else {
	*logos << "writing costmap_2d::CostMap2d\n";
	*dump_os << setup->getRaw2DCostmap().toString();
      }
    }
  }
  setup->dumpDescription(*logos, "", "  ");
  *logos << flush
	 << "creating environment of type " << environmentType
	 << " for map type " << costmapType << "\n" << flush;
  
  if ("2D" == environmentType) {
    unsigned char const
      obst_cost_thresh(costmap_2d::CostMap2D::INSCRIBED_INFLATED_OBSTACLE);
    environment.reset(new EnvironmentWrapper2D(setup->getCostmap().get(), false,
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
    environment.reset(new ompl::EnvironmentWrapper3DKIN(setup->getCostmap().get(), false,
							setup->getIndexTransform().get(), false,
							obst_cost_thresh,
							0, 0, 0, // start POSE (x, y, th)
							0, 0, 0, // goal POSE (x, y, th)
							goaltol_x, goaltol_y, goaltol_theta,
							getFootprint(), nominalvel_mpersecs,
							timetoturn45degsinplace_secs));
    static bool const do_sanity_check(false);
    if (do_sanity_check) {
      CostmapWrap const & cm(*setup->getCostmap());
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
  {
    IndexTransformWrap const & it(*setup->getIndexTransform());
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
      
      // plan it
      vector<int> solutionStateIDs;
      statsEntry.stop_at_first_solution = false;
      statsEntry.plan_from_scratch = task.from_scratch;
      statsEntry.allocated_time_sec = numeric_limits<double>::max();
      statsEntry.status = plannerMgr->replan(statsEntry.stop_at_first_solution,
					     statsEntry.plan_from_scratch,
					     statsEntry.allocated_time_sec,
					     &statsEntry.actual_time_wall_sec,
					     &statsEntry.actual_time_user_sec,
					     &statsEntry.actual_time_system_sec,
					     &statsEntry.solution_cost,
					     &statsEntry.solution_epsilon,
					     &solutionStateIDs);
      
      // extract the plan and update statistics
      statsEntry.plan_length_m = 0;
      statsEntry.plan_angle_change_rad = 0;
      if ((1 == statsEntry.status) && (1 < solutionStateIDs.size())) {
	shared_ptr<waypoint_plan_t> plan(new waypoint_plan_t());
	convertPlan(*environment, solutionStateIDs, plan.get(),
		    &statsEntry.plan_length_m,
		    &statsEntry.plan_angle_change_rad,
		    0 // XXXX if 3DKIN we actually want something here
		    );
	bundle.push_back(plan);
      }
      
      char const * title("  SUCCESS");
      if ((1 != statsEntry.status) || (1 >= solutionStateIDs.size()))
	title = "  FAILURE";
      statsEntry.logStream(*logos, title, "    ");
      *logos << flush;
      
      plannerStats.push_back(statsEntry);
      
      // Well... this ends up copying a std::vector of
      // boost::shared_ptr instances, could probably be smarter about
      // it. Also we will end up storing empty bundles, which is
      // actually what we want because for failed tasks we still want
      // to plot the start and goal poses (see gfx.cpp), but there
      // must be a neater solution to this.
      planList.insert(make_pair(ii, bundle));
    }
  }
}


void print_summary()
{
  size_t n_success(0);
  size_t n_fail(0);
  double t_success(0);
  double t_fail(0);
  double lplan(0);
  double rplan(0);
  for (plannerStats_t::const_iterator ie(plannerStats.begin()); ie != plannerStats.end(); ++ie) {
    // cannot always use status, some planners say SUCCESS even they fail
    if ((ie->status != 1) || (ie->plan_length_m < 1e-3)) {
      ++n_fail;
      t_fail += ie->actual_time_user_sec;
    }
    else {
      ++n_success;
      t_success += ie->actual_time_user_sec;
      lplan += ie->plan_length_m;
      rplan += ie->plan_angle_change_rad;
    }
  }
  rplan *= 180.0 / M_PI;
  //  double const ntt(n_success + n_fail);
  double x0, y0, x1, y1;
  setup->getWorkspaceBounds(x0, y0, x1, y1);
  double area((x1-x0)*(y1-y0));
  double ncells(ceil(area / pow(opt.resolution, 2)));
  *logos << "\nsummary:\n"
	 << "  map size:\n"
	 << "    area [m2]:                 " << area << "\n"
	 << "    cells (approx):            " << ncells << "\n"
	 << "  N tasks:\n"
	 << "    success:                   " << n_success << "\n"
	 << "    fail:                      " << n_fail << "\n"
    //	 << "    total:                     " << n_success + n_fail << "\n"
	 << "  total / average:\n"
	 << "    planning time success [s]: " << t_success << " / " << t_success / n_success << "\n"
    //	 << "    planning time failure [s]: " << t_fail << " / " << t_fail / n_success << "\n"
	 << "    plan length [m]:           " << lplan << " / " << lplan / n_success << "\n"
	 << "    plan angle change [deg]:   " << rplan << " / " << rplan / n_success << "\n";
  if (websiteMode) {
    string foo("mpbench-" + summarizeOptions() + ".html");
    ofstream os(foo.c_str());
    if (os)
      os << "<table border=\"1\" cellpadding=\"2\">\n"
	 << "<tr><td><b>N tasks</b></td><td>success: " << n_success << "</td><td>fail: " << n_fail << "</td></tr>\n"
	 << "<tr><td><b>map size</b></td><td>area: " << area << "</td><td>cells " << ncells << "</td></tr>\n"
	 << "</table>\n"
	 << "<table border=\"1\" cellpadding=\"2\">\n"
	 << "<tr><th>&nbsp;</th><th>total</th><th>average</th></tr>\n"
	 << "<tr><td><b>planning time success [s]</b></td><td>" << t_success << "</td><td>" << t_success / n_success << "</td></tr>\n"
	 << "<tr><td><b>plan length [m]</b></td><td>" << lplan << "</td><td>" << lplan / n_success << "</td></tr>\n"
	 << "<tr><td><b>plan angle change [deg]</b></td><td>" << rplan << "</td><td>" << rplan / n_success << "</td></tr>\n"
	 << "</table>\n";
  }
}


footprint_t const & getFootprint()
{
  if ( ! footprint) {
    footprint.reset(new footprint_t());
    initSimpleFootprint(*footprint, opt.inscribed_radius, opt.circumscribed_radius);
  }
  return *footprint;
}
