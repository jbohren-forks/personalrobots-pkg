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

static string baseFilename();
static string sanitizeSpec(string const & spec);

static char * const d_world_spec("hc:office1:1.2:3");
static char * const d_planner_spec("ad:2d:bwd");
static char * const d_robot_spec("pr2:325:460:600:600");
static char * const d_costmap_spec("ros:50");
static char * const d_geometry("800x600");

static bool enableGfx;
static string world_spec;
static string planner_spec;
static string costmap_spec;
static string robot_spec;
static string geometry;
static bool websiteMode;

static shared_ptr<Setup> setup;
static shared_ptr<ostream> logos;
static ostream * dbgos(0);

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
  if (enableGfx) {
    int base_width(800);
    int base_height(600);
    string head, tail;
    sfl::splitstring(geometry, 'x', head, tail);
    sfl::string_to(head, base_width);
    sfl::string_to(tail, base_height);
    ////    cout << "w " << base_width << " h " << base_height << "\n";
    display(gfx::Configuration(*setup,
			       base_width,
			       base_height,
			       websiteMode,
			       baseFilename(),
			       resultlist,
			       true, // XXXX to do: depends on 3DKIN
			       *logos),
	    "mpbench",
	    3, // hack: layoutID
	    &argc, argv);
  }
}


void cleanup()
{
  if (logos)
    *logos << "byebye!\n" << flush;
  setup.reset();
  logos.reset();
  resultlist.clear();
}


void usage(ostream & os)
{
  os << "options:\n"
     << "   -h          help (this message)\n"
     << "   -v          verbose: print debug message\n"
     << "   -w  <spec>  world specification string (default " << d_world_spec << ")\n"
     << "   -p  <spec>  planner specification string (default " << d_planner_spec << ")\n"
     << "   -r  <spec>  robot specification string (default " << d_robot_spec << ")\n"
     << "   -c  <spec>  costmap specification string (default " << d_costmap_spec << ")\n"
     << "   -g  <spec>  GLUT window size (default " << d_geometry << ")\n"
     << "   -X          dump filename base to stdout (use as last option)\n"
     << "   -W          run in website generation mode\n";
  SetupOptions::help(os, "help on setup options:", "  ");
}


static string summarizeOptions()
{
  ostringstream os;
  os << "-w" << sanitizeSpec(world_spec)
     << "-p" << sanitizeSpec(planner_spec)
     << "-r" << sanitizeSpec(robot_spec)
     << "-c" << sanitizeSpec(costmap_spec);
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


void parse_options(int argc, char ** argv)
{
  enableGfx = true;
  world_spec = d_world_spec;
  planner_spec = d_planner_spec;
  robot_spec = d_robot_spec;
  costmap_spec = d_costmap_spec;
  geometry = d_geometry;
  websiteMode = false;
  
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
	
      case 'v':
	dbgos = &cout;
	break;
	
      case 'w':
 	++ii;
 	if (ii >= argc) {
 	  warnx("-w requires world_spec argument");
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	world_spec = argv[ii];
 	break;
	
      case 'p':
 	++ii;
 	if (ii >= argc) {
 	  warnx("-p requires planner_spec argument");
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	planner_spec = argv[ii];
 	break;
	
      case 'r':
 	++ii;
 	if (ii >= argc) {
 	  warnx("-r requires robot_spec argument");
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	robot_spec = argv[ii];
 	break;
	
      case 'c':
 	++ii;
 	if (ii >= argc) {
 	  warnx("-c requires costmap_spec argument");
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	costmap_spec = argv[ii];
 	break;
	
      case 'g':
 	++ii;
 	if (ii >= argc) {
 	  warnx("-g requires geometry argument");
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	geometry = argv[ii];
 	break;
	
      case 'X':
	cout << baseFilename() << "\n";
	exit(EXIT_SUCCESS);
	
      case 'W':
	websiteMode = true;
	break;
	
      default:
	warnx("invalid option %s", argv[ii]);
	usage(cerr);
	exit(EXIT_FAILURE);
      }
  }
}


void create_setup()
{
  try {
    setup = Setup::create(world_spec, planner_spec, robot_spec, costmap_spec, logos.get(), dbgos);
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "create_setup(): EXCEPTION %s", ee.what());
  }
  setup->dumpDescription(*logos, "", "  ");
  *logos << "finished creating setup\n" << flush;
}


static void plan_iteratively(size_t task_id, size_t episode_id,
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
	result(new task::result(task_id, episode_id, start, goal, plan, stats));
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
      result(new task::result(task_id, episode_id, start, goal, plan, stats));
    resultlist.push_back(result); // XXXX: should a separate failurelist be used instead???
    
    prev_epsilon = stats->solution_epsilon;
  }  
}


static void plan_once(size_t task_id, size_t episode_id,
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
    result(new task::result(task_id, episode_id, start, goal, plan, stats));
  resultlist.push_back(result);
}


void run_tasks()
{
  try {
    boost::shared_ptr<mpglue::CostmapPlanner> planner(setup->getPlanner());
    *logos << "running tasks\n" << flush;
    
    tasklist_t const & tasklist(setup->getTasks());
    for (size_t task_id(0); task_id < tasklist.size(); ++task_id) {
      if ( ! tasklist[task_id])
	errx(EXIT_FAILURE, "run_tasks(): no task with ID %zu", task_id);
      task::setup const task(*tasklist[task_id]);
      if (task.start.empty())
	errx(EXIT_FAILURE, "run_tasks(): task ID %zu has no episodes", task_id);
      
      *logos << "\n  task " << task_id << ": " << task.description << "\n" << flush;
      planner->setGoal(task.goal.px, task.goal.py, task.goal.pth);
      planner->setGoalTolerance(task.goal.tol_xy, task.goal.tol_th);
      
      for (size_t episode_id(0); episode_id < task.start.size(); ++episode_id) {
	task::startspec const & start(task.start[episode_id]);
	
	planner->setStart(start.px, start.py, start.pth);
	planner->forcePlanningFromScratch(start.from_scratch);
	
	// not all planners can be run iteratively...
	SBPLPlannerWrap * sbpl_planner(dynamic_cast<SBPLPlannerWrap *>(planner.get()));
	if ( ! sbpl_planner)
	  plan_once(task_id, episode_id, start, task.goal, *planner);
	else {
	  sbpl_planner->stopAtFirstSolution(start.use_initial_solution);
	  sbpl_planner->setAllocatedTime(start.alloc_time);
	  if (start.allow_iteration)
	    plan_iteratively(task_id, episode_id, start, task.goal, *sbpl_planner);
	  else
	    plan_once(task_id, episode_id, start, task.goal, *sbpl_planner);
	}
      } // endfor(episode)
    } // endfor(task)
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
  double ncells(ceil(area / pow(setup->getOptions().costmap_resolution, 2)));
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
