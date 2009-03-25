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
static void print_gnuplot();

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
static string customBaseFilename;

static shared_ptr<Setup> setup;
static shared_ptr<ResultCollection> result_collection;
static ostream * logos(0);
static ostream * dbgos(0);

static shared_ptr<footprint_t> footprint;


int main(int argc, char ** argv)
{
  if (0 != atexit(cleanup))
    errx(EXIT_FAILURE, "atexit() failed");
  parse_options(argc, argv);
  string logFilename(baseFilename() + ".txt");
  if (dbgos)
    logos = dbgos;
  else
    logos = new ofstream(logFilename.c_str());
  create_setup();
  run_tasks();
  print_summary();
  ////  print_gnuplot();
  {
    string filename(baseFilename() + ".result.xml");
    ofstream os(filename.c_str());
    if ( ! os) {
      cout << "sorry, could not open file " << filename << "\n";
    }
    else {
      cout << "writing XML result file: " << filename << "\n";
      result_collection->dumpXML(os, "");
    }
  }
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
			       *result_collection,
			       true, // XXXX to do: depends on 3DKIN
			       *logos),
	    "mpbench",
	    3, // hack: layoutID
	    &argc, argv);
  }
}


void cleanup()
{
  if (dbgos)
    *dbgos << "byebye!\n" << flush;
  else
    delete logos;
  setup.reset();
  result_collection.reset();
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
     << "   -x  <fname> set custom base filename\n"
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
  if (customBaseFilename.empty())
    return "mpbench-" + summarizeOptions();
  return customBaseFilename;
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
    forbidden.insert(':');
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
  customBaseFilename = "";
  
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
	
      case 'x':
	++ii;
 	if (ii >= argc) {
 	  warnx("-x requires filename argument");
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	customBaseFilename = argv[ii];
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
    setup = Setup::create(world_spec, planner_spec, robot_spec, costmap_spec, logos, dbgos);
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "create_setup(): EXCEPTION %s", ee.what());
  }
  setup->dumpDescription(*logos, "", "  ");
  result_collection.reset(new ResultCollection(setup->getOptions()));
  *logos << "finished creating setup\n" << flush;
}


static void plan_iteratively(size_t task_id, size_t episode_id,
			     episode::startspec const & start, episode::goalspec const & goal,
			     SBPLPlannerWrap & planner_ref)
{
  double prev_epsilon(-1);
  double cumul_allocated_time(0);
  double cumul_actual_time_wall(0);
  double cumul_actual_time_user(0);
  int cumul_expands(0);
  
  if (start.use_initial_solution) {
    planner_ref.stopAtFirstSolution(true);
    planner_ref.setAllocatedTime(numeric_limits<double>::max()); // override for 1st iteration
  }
  else {
    planner_ref.stopAtFirstSolution(false);
    planner_ref.setAllocatedTime(start.alloc_time);
  }
  
  for (size_t iteration_id(0); true; ++iteration_id) {
    
    shared_ptr<waypoint_plan_t> plan;
    plan = planner_ref.createPlan();
    shared_ptr<SBPLPlannerStats> stats(planner_ref.copyMyStats());
    
    // The first iteration might have to be handled differently from
    // the rest.
    if (0 == iteration_id) {
      if (start.from_scratch)
	planner_ref.forcePlanningFromScratch(false);
      if (start.use_initial_solution) {
	planner_ref.stopAtFirstSolution(false);
	// undo the override for 1st iteration
	planner_ref.setAllocatedTime(start.alloc_time);
	// hack stats: pretend we allocated exactly the time we needed
	stats->allocated_time = stats->actual_time_wall;
      }
    }
    
    if ( ! plan) {
      // giving up immediately sort of precludes the possibility that
      // the planner was not given enough time, or that it has not
      // been given a chance of coming up with an initial solution
      // before... ah well, cannot handle every possible case.
      stats->logStream(*logos, "  episode " + sfl::to_string(episode_id) + " iteration "
		       + sfl::to_string(iteration_id) + ": FAILURE", "    ");
      *logos << flush;
      result_collection->insert(task_id, episode_id, iteration_id, start, goal, plan, stats);
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
    
    if (0 == iteration_id)
      stats->logStream(*logos, "  episode " + sfl::to_string(episode_id) + " iteration "
		       + sfl::to_string(iteration_id) + "  FIRST_SOLUTION", "    ");
    else
      stats->logStream(*logos, "  episode " + sfl::to_string(episode_id) + " iteration "
		       + sfl::to_string(iteration_id) + "  IMPROVED", "    ");
    *logos << flush;
    result_collection->insert(task_id, episode_id, iteration_id, start, goal, plan, stats);
    prev_epsilon = stats->solution_epsilon;
  }  
}


static void plan_once(size_t task_id, size_t episode_id,
		      episode::startspec const & start, episode::goalspec const & goal,
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
  result_collection->insert(task_id, episode_id, 0, start, goal, plan, stats);
}


void run_tasks()
{
  try {
    *logos << "running tasks\n" << flush;
    
    tasklist_t const & tasklist(setup->getTasks());
    for (size_t task_id(0); task_id < tasklist.size(); ++task_id) {
      if ( ! tasklist[task_id])
	errx(EXIT_FAILURE, "run_tasks(): no task with ID %zu", task_id);
      episode::taskspec const task(*tasklist[task_id]);
      if (task.start.empty())
	errx(EXIT_FAILURE, "run_tasks(): task ID %zu has no episodes", task_id);
      
      *logos << "\n  task " << task_id << ": " << task.description << "\n" << flush;
      boost::shared_ptr<mpglue::CostmapPlanner> planner(setup->getPlanner(task_id));
      SBPLPlannerWrap * sbpl_planner(dynamic_cast<SBPLPlannerWrap *>(planner.get()));
      planner->setGoal(task.goal.px, task.goal.py, task.goal.pth);
      planner->setGoalTolerance(task.goal.tol_xy, task.goal.tol_th);
      
      for (size_t episode_id(0); episode_id < task.start.size(); ++episode_id) {
	bool const costs_changed(setup->getWorld()->select(task_id, episode_id));
	episode::startspec const & start(task.start[episode_id]);
	
	planner->setStart(start.px, start.py, start.pth);
	planner->forcePlanningFromScratch(start.from_scratch);
	planner->flushCostChanges(costs_changed);
	
	// not all planners can be run iteratively...
	if ( ! sbpl_planner)
	  plan_once(task_id, episode_id, start, task.goal, *planner);
	else {
	  if (start.allow_iteration)
	    plan_iteratively(task_id, episode_id, start, task.goal, *sbpl_planner);
	  else {
	    sbpl_planner->stopAtFirstSolution(start.use_initial_solution);
	    sbpl_planner->setAllocatedTime(start.alloc_time);
	    plan_once(task_id, episode_id, start, task.goal, *sbpl_planner);
	  }
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
  result::list_t resultlist(result_collection->getAll());
  for (result::list_t::const_iterator ie(resultlist.begin()); ie != resultlist.end(); ++ie) {
    shared_ptr<result::entry> result(*ie);
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
  setup->getWorld()->getWorkspaceBounds(x0, y0, x1, y1);
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
    string foo(baseFilename() + ".html");
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
	 << "#  7 TEMPORARILY DISABLED (always 0)           number of expands\n"
	 << "#  8 TEMPORARILY DISABLED (always 0) cumulated number of expands\n"
	 << "#  9 TEMPORARILY DISABLED (always 0) cumulated number of expands [% of final]\n"
	 << "# 10 TEMPORARILY DISABLED (always 0)           expansion speed (wall) [1/s]\n"
	 << "# 11 TEMPORARILY DISABLED (always 0) cumul avg expansion speed (wall) [1/s]\n"
	 << "# 12 TEMPORARILY DISABLED (always 0)           expansion speed (user) [1/s]\n"
	 << "# 13 TEMPORARILY DISABLED (always 0) cumul avg expansion speed (user) [1/s]\n"
	 << "# 14 TEMPORARILY DISABLED (always 0) solution cost\n"
	 << "# 15 TEMPORARILY DISABLED (always 0) solution cost [% of final]\n"
	 << "# 16 TEMPORARILY DISABLED (always 0) solution epsilon\n"
	 << "# 17 plan length [m]\n"
	 << "# 18 plan length [% of final]\n"
	 << "# 19 plan rotation [rad]\n"
	 << "# 20 plan rotation [% of final]\n";
  
  tasklist_t const & task(setup->getTasks());
  
  // to be generalized... probably not in C++ though
  static size_t const task_begin(0);
  static size_t const task_end(numeric_limits<size_t>::max());
  static size_t const episode_begin(0);
  static size_t const episode_end(numeric_limits<size_t>::max());
  static size_t const iteration_begin(0);
  static size_t const iteration_end(numeric_limits<size_t>::max());
  result::view3_t view;
  try {
    result_collection->createView(result::TASK_ID, task_begin, task_end,
				  result::EPISODE_ID, episode_begin, episode_end,
				  result::ITERATION_ID, iteration_begin, iteration_end,
				  view);
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "print_gnuplot(): EXCEPTION %s", ee.what());
  }
  for (result::view3_t::const_iterator it(view.begin()); it != view.end(); ++it) {
    dataOs << "\n\n# task[" << it->first << "]: " << task[it->first]->description << "\n";
    if (it->second.empty()) {
      dataOs << "# no episodes\n";
      continue;
    }
    
    for (result::view2_t::const_iterator ie(it->second.begin()); ie != it->second.end(); ++ie) {
      if (ie->second.empty()) {
	dataOs << "# no iterations\n";
	continue;
      }
      dataOs << "# episode[" << ie->first << "]\n";
      
      // first loop to get all cumulative results, so that we can give
      // relative figures
      double final_cumul_wall(0);
      double final_cumul_user(0);
      ////int final_cumul_expands(0);
      ////int final_cost(0);
      double final_length(0);
      double final_rotation(0);
      for (result::view1_t::const_iterator ii(ie->second.begin()); ii != ie->second.end(); ++ii) {
	shared_ptr<result::entry> result(ii->second);
	if ( ! result) {		// "never" happens
	  dataOs << "# iteration[" << ii->first << "]: void result\n";
	  continue;
	}
	if ( ! result->stats) {	// "never" happens
	  dataOs << "# iteration[" << ii->first << "]: void stats\n";
	  continue;
	}
	if ( ! result->stats->success) { // does happen
	  dataOs << "# iteration[" << ii->first << "]: no solution\n";
	  continue;
	}
	final_cumul_wall += result->stats->actual_time_wall;
	final_cumul_user += result->stats->actual_time_user;
	////final_cumul_expands += result->stats->number_of_expands;
	////final_cost = result->stats->solution_cost;
	final_length = result->stats->plan_length;
	final_rotation = result->stats->plan_angle_change;
      }
      
      // second loop to actually dump the values to file
      double cumul_wall(0);
      double cumul_user(0);
      ////int cumul_expands(0);
      for (result::view1_t::const_iterator ii(ie->second.begin()); ii != ie->second.end(); ++ii) {
	shared_ptr<result::entry> result(ii->second);
	if ( ! result)		// already wrote a message in 1st loop
	  continue;
	if ( ! result->stats)
	  continue;
	if ( ! result->stats->success)
	  continue;
	cumul_wall += result->stats->actual_time_wall;
	cumul_user += result->stats->actual_time_user;
	////cumul_expands += result->stats->number_of_expands;
	dataOs
	  // wall time
	  << result->stats->actual_time_wall << "\t"
	  << cumul_wall << "\t"
	  << 100 * cumul_wall / final_cumul_wall << "\t"
	  // user time
	  << result->stats->actual_time_user << "\t"
	  << cumul_user << "\t"
	  << 100 * cumul_user / final_cumul_user << "\t"
	  // TEMPORARILY DISABLED (always 0) expands
	  ////<< result->stats->number_of_expands << "\t"
	  ////<< cumul_expands << "\t"
	  ////<< 100 * cumul_expands / final_cumul_expands << "\t"
	  << "0\t0\t0\t"
	  // TEMPORARILY DISABLED (always 0) expansion speed
	  ////<< result->stats->number_of_expands / result->stats->actual_time_wall << "\t"
	  ////<< cumul_expands / cumul_wall << "\t"
	  ////<< result->stats->number_of_expands / result->stats->actual_time_user << "\t"
	  ////<< cumul_expands / cumul_user << "\t"
	  << "0\t0\t0\t0\t"
	  // TEMPORARILY DISABLED (always 0) cost
	  ////<< result->stats->solution_cost << "\t"
	  ////<< (100.0 * result->stats->solution_cost) / final_cost << "\t"
	  << "0\t0\t"
	  // TEMPORARILY DISABLED (always 0) epsilon
	  ////<< result->stats->solution_epsilon << "\t"
	  << "0\t"
	  // plan quality
	  << result->stats->plan_length << "\t"
	  << 100 * result->stats->plan_length / final_length << "\t"
	  << result->stats->plan_angle_change << "\t"
	  << 100 * result->stats->plan_angle_change / final_rotation << "\n";
      }
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
  // XXXX to do: would make more sense to plot costhist for each
  // snapshot, or all combined, but for now just take the last episode
  // of the first planner
  boost::shared_ptr<CostmapAccessor const> cm(setup->getWorld()->getCostmap(0));
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
