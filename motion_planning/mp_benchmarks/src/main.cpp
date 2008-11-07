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

/** \file main.cpp  */

#include "setup.hpp"
#include <costmap_2d/costmap_2d.h>
#include <sbpl_util.hh>
#include <sfl/util/numeric.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>

extern "C" {
#include <err.h>
}

using namespace ompl;
using namespace boost;
using namespace std;

static void cleanup();
static void usage(ostream & os);
static void parse_options(int argc, char ** argv);

static string logfname;
static string setupName;
static double resolution;
static double robotRadius;
static double freespaceDistance;
static int obstacleCost;
static double doorWidth;
static double hallWidth;
static string travmapFilename;

static shared_ptr<OfficeBenchmark> setup;
static shared_ptr<EnvironmentWrapper> environment;
static shared_ptr<SBPLPlannerManager> plannerMgr;
static shared_ptr<SBPLPlannerStatistics> plannerStats;
static shared_ptr<ostream> logos;


int main(int argc, char ** argv)
{
  if (0 != atexit(cleanup))
    errx(EXIT_FAILURE, "atexit() failed");
  parse_options(argc, argv);
  
  logos.reset(new ofstream(logfname.c_str(), ios_base::app));
  
  *logos << "creating setup \"" << setupName << "\"\n" << flush;
  {
    shared_ptr<ostream> travmapos;
    if ( ! travmapFilename.empty()) {
      travmapos.reset(new ofstream(travmapFilename.c_str()));
      if ( ! (*travmapos)) {
	*logos << "could not open travmap file " << travmapFilename << "\n" << flush;
	travmapos.reset();
      }
    }
    setup.reset(OfficeBenchmark::create(setupName, resolution, robotRadius, freespaceDistance,
					obstacleCost, doorWidth, hallWidth,
					logos.get(), travmapos.get()));
    if ( ! setup)
      errx(EXIT_FAILURE, "could not create setup with name \"%s\"", setupName.c_str());
  }
  setup->dumpDescription(*logos, "", "  ");
  *logos << flush;
  
  *logos << "creating environment wrapper\n" << flush;
  environment.reset(new EnvironmentWrapper2D(setup->getCostmap(), 0, 0, 0, 0,
					     costmap_2d::CostMap2D::INSCRIBED_INFLATED_OBSTACLE));
  MDPConfig mdpConfig;
  if ( ! environment->InitializeMDPCfg(&mdpConfig))
    errx(EXIT_FAILURE, "environment->InitializeMDPCfg() failed on environment %s", environment->getName().c_str());
  *logos << "  environment name: " << environment->getName() << "\n" << flush;
  
  *logos << "creating planner manager\n" << flush;
  bool const forwardsearch(false);
  plannerMgr.reset(new SBPLPlannerManager(environment->getDSI(), forwardsearch, &mdpConfig));
  string const plannerType("ARAPlanner");
  if ( ! plannerMgr->select(plannerType, false))
    errx(EXIT_FAILURE, "plannerMgr->select(%s) failed", plannerType.c_str());
  *logos << "  planner name: " << plannerMgr->getName() << "\n" << flush;
  
  *logos << "creating planner stats\n" << flush;
  plannerStats.reset(new SBPLPlannerStatistics());
  
  *logos << "running tasks\n" << flush;
  {
    costmap_2d::CostMap2D const & costmap(setup->getCostmap());
    SBPLBenchmarkSetup::tasklist_t const & tasklist(setup->getTasks());
    for (size_t ii(0); ii < tasklist.size(); ++ii) {
      plannerStats->pushBack(plannerMgr->getName(), environment->getName());
      SBPLPlannerStatistics::entry & statsEntry(plannerStats->top());
      SBPLBenchmarkSetup::task const & task(tasklist[ii]);
      
      *logos << "\n  task " << ii << ": " << task.description << "\n" << flush;
      
      // set start
      statsEntry.start.x = task.start_x;
      statsEntry.start.y = task.start_y;
      statsEntry.start.th = task.start_th;
      costmap.WC_MC(statsEntry.start.x, statsEntry.start.y, statsEntry.startIx, statsEntry.startIy);
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
      costmap.WC_MC(statsEntry.goal.x, statsEntry.goal.y, statsEntry.goalIx, statsEntry.goalIy);
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
#warning 'TO DO: use a progression of ever longer time limits'
      statsEntry.allocated_time_sec = numeric_limits<double>::max();
      statsEntry.status = plannerMgr->replan(statsEntry.allocated_time_sec,
					     &statsEntry.actual_time_wall_sec,
					     &statsEntry.actual_time_user_sec,
					     &statsEntry.actual_time_system_sec,
					     &solutionStateIDs);
      
      // extract the plan and update statistics
      statsEntry.plan_length_m = 0;
      statsEntry.plan_angle_change_rad = 0;
      if ((1 == statsEntry.status) && (1 < solutionStateIDs.size())) {
	list<std_msgs::Pose2DFloat32> plan;
	double prevx(0), prevy(0), prevth(0);
	prevth = 42.17;	// to detect when it has been initialized (see 42 below)
	for(vector<int>::const_iterator it = solutionStateIDs.begin(); it != solutionStateIDs.end(); ++it){
	  std_msgs::Pose2DFloat32 const waypoint(environment->GetPoseFromState(*it));
	  
	  // update stats:
	  // - first round, nothing to do
	  // - second round, update path length only
	  // - third round, update path length and angular change
	  if (plan.empty()) {
	    prevx = waypoint.x;
	    prevy = waypoint.y;
	  }
	  else {
	    double const dx(waypoint.x - prevx);
	    double const dy(waypoint.y - prevy);
	    statsEntry.plan_length_m += sqrt(pow(dx, 2) + pow(dy, 2));
	    double const th(atan2(dy, dx));
	    if (42 > prevth) // see 42.17 above
	      statsEntry.plan_angle_change_rad += fabs(sfl::mod2pi(th - prevth));
	    prevx = waypoint.x;
	    prevy = waypoint.y;
	    prevth = th;
#warning 'add the cumulation of delta(waypoint.th) now that we can have 3D plans'
	  }
	  
	  plan.push_back(waypoint);
	}
      }
      
      char const * title("  SUCCESS");
      if ((1 != statsEntry.status) || (1 >= solutionStateIDs.size()))
	title = "  FAILURE";
      statsEntry.logStream(*logos, title, "    ");
      *logos << flush;
    }
  }
  
  *logos << "bye\n" << flush;
}


void cleanup()
{
  setup.reset();
  environment.reset();
  plannerMgr.reset();
  plannerStats.reset();
  logos.reset();
}




void usage(ostream & os)
{
  os << "options:\n"
     << "   -h               help (this message)\n"
     << "   -s  <name>       name of the setup\n"
     << "   -c  <cellsize>   set grid resolution\n"
     << "   -r  <radius>     set robot radius\n"
     << "   -f  <freedist>   set freespace distance\n"
     << "   -d  <doorwidth>  set width of doors\n"
     << "   -H  <hallwidth>  set width of hallways\n"
     << "   -l  <filename>   filename for logging\n"
     << "   -o  <filename>   write sfl::TraversabilityMap to file\n";
}


void parse_options(int argc, char ** argv)
{
  logfname = "/dev/stdout";
  setupName = "office1";
  resolution = 0.05; // at libsunflower -r939, using 0.025 eats up all RAM...
  robotRadius = 0.5;
  freespaceDistance = 1.2;
  obstacleCost = costmap_2d::CostMap2D::INSCRIBED_INFLATED_OBSTACLE;
  doorWidth = 1.2;
  hallWidth = 3;
  travmapFilename = "";
  
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
	
      case 's':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -s requires a name argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	setupName = argv[ii];
 	break;
	
      case 'c':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -c requires a cellsize argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> resolution;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading cellsize argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'r':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -r requires a radius argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> robotRadius;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading radius argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'f':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -f requires a freedist argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> freespaceDistance;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading freedist argument from \"" << argv[ii] << "\"\n";
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
	  is >> doorWidth;
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
	  is >> hallWidth;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading hallwidth argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'l':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -l requires a filename argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	logfname = argv[ii];
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
	
      default:
	cerr << argv[0] << ": invalid option '" << argv[ii] << "'\n";
	usage(cerr);
	exit(EXIT_FAILURE);
      }
  }
}
