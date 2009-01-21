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

#include "setup.h"
#include "gfx.h"
#include <costmap_2d/costmap_2d.h>
#include <mpglue/sbpl_planner.h>
#include <mpglue/navfn_planner.h>
#include <sfl/util/numeric.hpp>
#include <sfl/util/strutil.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <iomanip>
#include <set>

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
static std::string canonicalPlannerName(std::string const & name_or_alias);

static footprint_t const & getFootprint();
static string baseFilename();
static string sanitizeSpec(string const & spec);

static bool enableGfx;
static string planner_spec;
static vector<string> planner_name;
static string costmapType;
static SetupOptions opt;
static bool websiteMode;

static shared_ptr<Setup> setup;
static vector<shared_ptr<CostmapPlanner> > planner;
static shared_ptr<ostream> logos;

static shared_ptr<footprint_t> footprint;

static resultlist_t resultlist;


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
			       planner_name,
			       opt,
			       websiteMode,
			       baseFilename(),
			       getFootprint(),
			       resultlist,
			       true, // XXXX to do: depends on 3DKIN
			       *logos),
	    opt.spec.c_str(),
	    3, // hack: layoutID
	    &argc, argv);
}


void cleanup()
{
  if (logos)
    *logos << "byebye!\n" << flush;
  setup.reset();
  planner.clear();
  logos.reset();
  resultlist.clear();
}


void usage(ostream & os)
{
  os << "options:\n"
     << "   -h               help (this message)\n"
     << "   -p  <spec>       colon-separated planner names\n"
     << "   -m  <name>       name of the costmap implementation\n"
    ////     << "   -e  <name>       environment representation type\n"
     << "   -s  <spec>       setup specification, e.g. hc:office1 or pgm:costs.pgm:tasks.xml\n"
     << "   -r  <cellsize>   set grid resolution\n"
     << "   -i  <in-radius>  set INSCRIBED radius\n"
     << "   -c  <out-radius> set CIRCUMSCRIBED radius\n"
     << "   -I  <inflate-r>  set INFLATION radius\n"
     << "   -d  <doorwidth>  set width of doors (office setups)\n"
     << "   -H  <hallwidth>  set width of hallways (office setups)\n"
    ////     << "   -n  <filename>   Net PGM file to load (for -s pgm)\n"
     << "   -g  <gray>       cutoff for obstacles in PGM images\n"
    ////     << "   -o  <filename>   write sfl::TraversabilityMap to file\n"
    ////     << "   -O  <filename>   write costmap_2d::CostMap2D to file\n"
     << "   -X               dump filename base to stdout (use as last option)\n"
     << "   -W               run in website generation mode\n";
}


static string summarizeOptions()
{
  ostringstream os;
  os << "-p" << sanitizeSpec(planner_spec)
     << "-s" << sanitizeSpec(opt.spec)
     << "-m" << costmapType
    ////     << "-e" << canonicalEnvironmentName(environmentType)
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


string sanitizeSpec(string const & spec)
{
  static set<char> forbidden;
  if (forbidden.empty()) {
    for (char cc(0); cc <= '*'; ++cc)
      forbidden.insert(cc);
    forbidden.insert('.');
    forbidden.insert('/');
    forbidden.insert(':');
    forbidden.insert(';');
    forbidden.insert('<');
    forbidden.insert('>');
    forbidden.insert('?');
    forbidden.insert('@');
    forbidden.insert('[');
    forbidden.insert('\\');
    forbidden.insert(']');
    forbidden.insert('`');
    for (char cc(0x7f); cc >= '{'; --cc)
      forbidden.insert(cc);
  }
  string foo(spec);
  for (string::iterator is(foo.begin()); is != foo.end(); ++is)
    if (forbidden.end() != forbidden.find(*is))
      *is = '_';
  return foo;
}


static void sanitizeOptions()
{
  if (opt.inscribed_radius > opt.circumscribed_radius)
    opt.circumscribed_radius = opt.inscribed_radius;
  if (opt.inscribed_radius > opt.inflation_radius)
    opt.inflation_radius = opt.inscribed_radius;
  if (opt.circumscribed_radius > opt.inflation_radius)
    opt.inflation_radius = opt.circumscribed_radius;
}


void parse_options(int argc, char ** argv)
{
  // these should become options
  enableGfx = true;
  
  // default values for options
  planner_spec = "ara:ad:nf";
  costmapType = "costmap_2d";
  ////  environmentType = "2D";
  websiteMode = false;
  // most other options handled through SetupOptions
  
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
 	  cerr << argv[0] << ": -p requires a spec argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	planner_spec = argv[ii];
 	break;
	
      case 's':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -s requires a spec argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	opt.spec = argv[ii];
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
  
  *logos << "creating setup \"" << opt.spec << "\"\n" << flush;
  try {
    setup.reset(createSetup(opt, logos.get()));
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "create_setup(): EXCEPTION %s", ee.what());
  }
  if ( ! setup)
    errx(EXIT_FAILURE, "create_setup(): could not create setup from spec \"%s\"",
	 opt.spec.c_str());
  setup->dumpDescription(*logos, "", "  ");
  
  {
    string spec(planner_spec);
    string head;
    while (sfl::splitstring(spec, ':', head, spec)) {
      string name(canonicalPlannerName(head));
      bool const strict_name_check(true);
      if ( ! name.empty()) {
	planner_name.push_back(name);
	*logos << "added planner name " << name << "\n" << flush;
      }
      else if (strict_name_check)
	errx(EXIT_FAILURE, "create_setup(): strict_name_check failed on \"%s\"", head.c_str());
    }
  }
  if (planner_name.empty())
    errx(EXIT_FAILURE, "create_setup(): no valid planner names in spec \"%s\"",
	 planner_spec.c_str());
  
  bool const forwardsearch(false); // XXXX to do: add an option for this
  int const obstcost_thresh_2d(costmap_2d::CostMap2D::INSCRIBED_INFLATED_OBSTACLE);
  int const obstcost_thresh_3dkin(costmap_2d::CostMap2D::LETHAL_OBSTACLE);
  double const nominalvel_mpersecs(0.6); // XXXX to do: config! human leisurely walking speed
  double const timetoturn45degsinplace_secs(0.6); // XXXX to do: config! guesstimate
  
  for (size_t in(0); in < planner_name.size(); ++in) {
    if ("NavFn" == planner_name[in]) {
      *logos << "creating NavFnPlanner\n" << flush;
      shared_ptr<CostmapPlanner> foo(new NavFnPlanner(setup->getCostmap(),
						      setup->getIndexTransform()));
      planner.push_back(foo);
    }
    else if ("ARAStar2D" == planner_name[in]) {
      shared_ptr<SBPLPlannerWrap> foo(createARAStar2D(setup->getCostmap(),
						      setup->getIndexTransform(),
						      forwardsearch,
						      obstcost_thresh_2d));
      if ( ! foo)
	errx(EXIT_FAILURE, "create_setup(): createARAStar2D() failed");
      foo->stopAtFirstSolution(false); // XXXX to do: use task::start
      foo->setAllocatedTime(numeric_limits<double>::max()); // XXXX to do: use task::start
      planner.push_back(foo);
    }
    else if ("ADStar2D" == planner_name[in]) {
      shared_ptr<SBPLPlannerWrap> foo(createADStar2D(setup->getCostmap(),
						     setup->getIndexTransform(),
						     forwardsearch,
						     obstcost_thresh_2d));
      if ( ! foo)
	errx(EXIT_FAILURE, "create_setup(): createADStar2D() failed");
      foo->stopAtFirstSolution(false); // XXXX to do: use task::start
      foo->setAllocatedTime(numeric_limits<double>::max()); // XXXX to do: use task::start
      planner.push_back(foo);
    }
    else if ("ARAStar3DKIN" == planner_name[in]) {
      shared_ptr<SBPLPlannerWrap> foo(createARAStar3DKIN(setup->getCostmap(),
							 setup->getIndexTransform(),
							 forwardsearch,
							 obstcost_thresh_3dkin,
							 getFootprint(),
							 nominalvel_mpersecs,
							 timetoturn45degsinplace_secs));
      if ( ! foo)
	errx(EXIT_FAILURE, "create_setup(): createARAStar3DKIN() failed");
      foo->stopAtFirstSolution(false); // XXXX to do: use task::start
      foo->setAllocatedTime(numeric_limits<double>::max()); // XXXX to do: use task::start
      planner.push_back(foo);
    }
    else if ("ADStar3DKIN" == planner_name[in]) {
      shared_ptr<SBPLPlannerWrap> foo(createADStar3DKIN(setup->getCostmap(),
							setup->getIndexTransform(),
							forwardsearch,
							obstcost_thresh_3dkin,
							getFootprint(),
							nominalvel_mpersecs,
							timetoturn45degsinplace_secs));
      if ( ! foo)
	errx(EXIT_FAILURE, "create_setup(): createADStar3DKIN() failed");
      foo->stopAtFirstSolution(false); // XXXX to do: use task::start
      foo->setAllocatedTime(numeric_limits<double>::max()); // XXXX to do: use task::start
      planner.push_back(foo);
    }
    else {
      errx(EXIT_FAILURE, "create_setup(): invalid planner name \"%s\"",
	   planner_name[in].c_str());
    }
  }
  
  *logos << "finished creating setup\n" << flush;
}


static void plan_iteratively(size_t planner_id, size_t task_id, size_t episode_id,
			     task::startspec const & start, task::goalspec const & goal,
			     SBPLPlannerWrap & planner_ref)
{
  double prev_epsilon(-1);
  double cumul_allocated_time(0);
  double cumul_actual_time_wall(0);
  double cumul_actual_time_user(0);
  int cumul_expands(0);
  
  for (size_t jj(0); true; ++jj) {
    
    shared_ptr<waypoint_plan_t> plan;
    plan = planner_ref.createPlan();
    shared_ptr<SBPLPlannerStats> stats(planner_ref.copyMyStats());
    
    if ( ! plan) {
      // giving up immediately sort of precludes the possibility that
      // the planner was not given enough time, or that it has not
      // been given a chance of coming up with an initial solution
      // before... ah well, cannot handle every possible case.
      stats->logStream(*logos, "  episode " + sfl::to_string(episode_id) + " iteration "
		       + sfl::to_string(jj) + ": FAILURE", "    ");
      *logos << flush;
      shared_ptr<task::result>
	result(new task::result(planner_id, task_id, episode_id, start, goal, plan, stats));
      resultlist.push_back(result); // XXXX: should a separate failurelist be used instead???
      break;
    }
    
    if (0 < stats->number_of_expands)
      cumul_expands += stats->number_of_expands;
    
    if (stats->solution_epsilon > 0) {
      if ((prev_epsilon > 0) && (fabs(prev_epsilon - stats->solution_epsilon) < 1e-9)) {
	stats->allocated_time = cumul_allocated_time;
	stats->actual_time_wall = cumul_actual_time_wall;
	stats->actual_time_user = cumul_actual_time_user;
	stats->number_of_expands = cumul_expands;
	stats->logStream(*logos,  "  episode " + sfl::to_string(episode_id) + " FINAL: cumul:",
			 "    ");
	*logos << flush;
	//// do NOT add to overall stats because of cumulated times
	break;
      }
    }
    
    cumul_allocated_time += stats->allocated_time;
    cumul_actual_time_wall += stats->actual_time_wall;
    cumul_actual_time_user += stats->actual_time_user;
    
    if (0 == jj)
      stats->logStream(*logos, "  episode " + sfl::to_string(episode_id) + " iteration "
		       + sfl::to_string(jj) + "  FIRST_SOLUTION", "    ");
    else
      stats->logStream(*logos, "  episode " + sfl::to_string(episode_id) + " iteration "
		       + sfl::to_string(jj) + "  IMPROVED", "    ");
    *logos << flush;
    shared_ptr<task::result>
      result(new task::result(planner_id, task_id, episode_id, start, goal, plan, stats));
    resultlist.push_back(result); // XXXX: should a separate failurelist be used instead???
    
    prev_epsilon = stats->solution_epsilon;
  }  
}


static void plan_once(size_t planner_id, size_t task_id, size_t episode_id,
		      task::startspec const & start, task::goalspec const & goal,
		      CostmapPlanner & planner_ref)
{
  shared_ptr<waypoint_plan_t> plan;
  try {
    plan = planner_ref.createPlan();
  }
  catch (std::exception const & ee) {
    *logos << "\n==================================================\n"
	   << "  plan_once(): EXCEPTION from createPlan():\n"
	   << ee.what()
	   << "\n==================================================\n" << flush;
  }
  
  string title;
  if (plan)
    title = "  episode " + sfl::to_string(episode_id) + ": SUCCESS";
  else
    title = "  episode " + sfl::to_string(episode_id) + ": FAILURE";
  shared_ptr<CostmapPlannerStats> stats(planner_ref.copyStats());
  stats->logStream(*logos, title, "    ");
  *logos << flush;
  
  shared_ptr<task::result>
    result(new task::result(planner_id, task_id, episode_id, start, goal, plan, stats));
  resultlist.push_back(result);
}


void run_tasks()
{
  try {
    for (size_t planner_id(0); planner_id < planner.size(); ++planner_id) {
      *logos << "running tasks for planner " << planner_id << "\n" << flush;
      
      tasklist_t const & tasklist(setup->getTasks());
      for (size_t task_id(0); task_id < tasklist.size(); ++task_id) {
	if ( ! tasklist[task_id])
	  errx(EXIT_FAILURE, "run_tasks(): no task with ID %zu", task_id);
	task::setup const task(*tasklist[task_id]);
	if (task.start.empty())
	  errx(EXIT_FAILURE, "run_tasks(): task ID %zu has no episodes", task_id);
      
	*logos << "\n  task " << task_id << ": " << task.description << "\n" << flush;
	planner[planner_id]->setGoal(task.goal.px, task.goal.py, task.goal.pth);
	planner[planner_id]->setGoalTolerance(task.goal.tol_xy, task.goal.tol_th);
      
	for (size_t episode_id(0); episode_id < task.start.size(); ++episode_id) {
	  task::startspec const & start(task.start[episode_id]);
	  CostmapPlanner * costmap_planner(planner[planner_id].get());
	  
	  costmap_planner->setStart(start.px, start.py, start.pth);
	  costmap_planner->forcePlanningFromScratch(start.from_scratch);
	  
	  // not all planners can be run iteratively...
	  SBPLPlannerWrap * sbpl_planner(dynamic_cast<SBPLPlannerWrap *>(costmap_planner));
	  if ( ! sbpl_planner)
	    plan_once(planner_id, task_id, episode_id, start, task.goal, *costmap_planner);
	  else {
	    sbpl_planner->stopAtFirstSolution(start.use_initial_solution);
	    sbpl_planner->setAllocatedTime(start.alloc_time);
	    if (start.allow_iteration)
	      plan_iteratively(planner_id, task_id, episode_id, start, task.goal, *sbpl_planner);
	    else
	      plan_once(planner_id, task_id, episode_id, start, task.goal, *sbpl_planner);
	  }
	} // endfor(episode)
      } // endfor(task)
    } // endfor(planner)
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION in run_tasks():\n%s", ee.what());
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
  for (resultlist_t::const_iterator ie(resultlist.begin()); ie != resultlist.end(); ++ie) {
    shared_ptr<task::result> result(*ie);
    if ( ! result)
      errx(EXIT_FAILURE, "print_summary(): void result");
    if ( ! result->stats)
      errx(EXIT_FAILURE, "print_summary(): void stats");
    if (result->stats->success) {
      ++n_success;
      t_success += result->stats->actual_time_wall;
      lplan += result->stats->plan_length;
      rplan += result->stats->plan_angle_change;
    }
    else {
      ++n_fail;
      t_fail += result->stats->actual_time_wall;
    }
  }
  rplan *= 180.0 / M_PI;
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
	 << "  total / average:\n"
	 << "    planning time success [s]: " << t_success << " / " << t_success / n_success << "\n"
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


std::string canonicalPlannerName(std::string const & name_or_alias)
{
  static map<string, string> planner_alias;
  if (planner_alias.empty()) {
    planner_alias.insert(make_pair("ARAStar2D",    "ARAStar2D"));
    planner_alias.insert(make_pair("ara",          "ARAStar2D"));
    planner_alias.insert(make_pair("ARA",          "ARAStar2D"));
    planner_alias.insert(make_pair("arastar",      "ARAStar2D"));
    planner_alias.insert(make_pair("ARAStar",      "ARAStar2D"));
    planner_alias.insert(make_pair("ara2d",        "ARAStar2D"));
    planner_alias.insert(make_pair("ARA2D",        "ARAStar2D"));
    planner_alias.insert(make_pair("arastar2d",    "ARAStar2D"));
    
    planner_alias.insert(make_pair("ARAStar3DKIN", "ARAStar3DKIN"));
    planner_alias.insert(make_pair("ara3d",        "ARAStar3DKIN"));
    planner_alias.insert(make_pair("ARA3D",        "ARAStar3DKIN"));
    planner_alias.insert(make_pair("arastar3d",    "ARAStar3DKIN"));
    planner_alias.insert(make_pair("ARAStar3D",    "ARAStar3DKIN"));
    
    planner_alias.insert(make_pair("ADStar2D",     "ADStar2D"));
    planner_alias.insert(make_pair("ad",           "ADStar2D"));
    planner_alias.insert(make_pair("AD",           "ADStar2D"));
    planner_alias.insert(make_pair("adstar",       "ADStar2D"));
    planner_alias.insert(make_pair("ADStar",       "ADStar2D"));

    planner_alias.insert(make_pair("ADStar3DKIN",  "ADStar3DKIN"));
    planner_alias.insert(make_pair("ad3d",         "ADStar3DKIN"));
    planner_alias.insert(make_pair("AD3D",         "ADStar3DKIN"));
    planner_alias.insert(make_pair("adstar3d",     "ADStar3DKIN"));
    planner_alias.insert(make_pair("ADStar3D",     "ADStar3DKIN"));

    planner_alias.insert(make_pair("NavFn",        "NavFn"));
    planner_alias.insert(make_pair("navfn",        "NavFn"));
    planner_alias.insert(make_pair("nf",           "NavFn"));
    planner_alias.insert(make_pair("NF",           "NavFn"));
  }
  
  map<string, string>::const_iterator is(planner_alias.find(name_or_alias));
  if (planner_alias.end() == is)
    return "";
  return is->second;
}
