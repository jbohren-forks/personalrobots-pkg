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
#include <npm/common/wrap_glu.hpp>
#include <npm/common/wrap_glut.hpp>
#include <npm/common/Manager.hpp>
#include <npm/common/View.hpp>
#include <npm/common/StillCamera.hpp>
#include <npm/common/TraversabilityDrawing.hpp>
#include <npm/common/SimpleImage.hpp>
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
static void display(int * argc, char ** argv);

static bool enableGfx;
static string plannerType;
static string costmapType;
static string logFilename;
static string pngFilename;
static string setupName;
static double resolution;
static double inscribedRadius;
static double circumscribedRadius;
static double inflationRadius;
static int obstacleCost;
static double doorWidth;
static double hallWidth;
static string travmapFilename;
static string costmapFilename;
static bool websiteMode;

static shared_ptr<OfficeBenchmark> setup;
static shared_ptr<EnvironmentWrapper> environment;
static shared_ptr<SBPLPlannerManager> plannerMgr;
static shared_ptr<SBPLPlannerStatistics> plannerStats;
static shared_ptr<ostream> logos;

static int glut_width;
static int glut_height;
static bool made_first_screenshot;

typedef list<std_msgs::Pose2DFloat32> plan_t;
typedef std::vector<plan_t> planList_t;
static planList_t planList;


int main(int argc, char ** argv)
{
  if (0 != atexit(cleanup))
    errx(EXIT_FAILURE, "atexit() failed");
  parse_options(argc, argv);
  logos.reset(new ofstream(logFilename.c_str(), ios_base::app));
  create_setup();
  run_tasks();  
  if (enableGfx) {
    made_first_screenshot = false;
    display(&argc, argv);
  }
}


void cleanup()
{
  if (logos)
    *logos << "byebye!\n" << flush;
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
     << "   -p  <name>       name of the SBPL planner\n"
     << "   -m  <name>       name of the costmap implementation\n"
     << "   -s  <name>       name of the setup\n"
     << "   -r  <cellsize>   set grid resolution\n"
     << "   -i  <in-radius>  set INSCRIBED radius\n"
     << "   -c  <out-radius> set CIRCUMSCRIBED radius\n"
     << "   -I  <inflate-r>  set INFLATION radius\n"
     << "   -d  <doorwidth>  set width of doors\n"
     << "   -H  <hallwidth>  set width of hallways\n"
     << "   -l  <filename>   overwrite filename for logging\n"
     << "   -P  <filename>   overwrite filename for screenshots\n"
     << "   -o  <filename>   write sfl::TraversabilityMap to file\n"
     << "   -O  <filename>   write costmap_2d::CostMap2D to file\n"
     << "   -X               dump filename base to stdout (use as last option)\n"
     << "   -W               run in website generation mode\n";
}


static string summarizeOptions()
{
  ostringstream os;
  os << "-p" << plannerType
     << "-s" << setupName
     << "-m" << costmapType
     << "-r" << (int) rint(1e3 * resolution)
     << "-i" << (int) rint(1e3 * inscribedRadius)
     << "-c" << (int) rint(1e3 * circumscribedRadius)
     << "-I" << (int) rint(1e3 * inflationRadius)
     << "-d" << (int) rint(1e3 * doorWidth)
     << "-H" << (int) rint(1e3 * hallWidth);
  return os.str();
}


void parse_options(int argc, char ** argv)
{
  // these should become options
  enableGfx = true;
  obstacleCost = costmap_2d::CostMap2D::INSCRIBED_INFLATED_OBSTACLE;
  
  // default values for options
  plannerType = "ARAPlanner";
  setupName = "office1";
  costmapType = "costmap_2d";
  resolution = 0.05;
  inscribedRadius = 0.5;
  circumscribedRadius = 1.2;
  inflationRadius = 2;
  doorWidth = 1.2;
  hallWidth = 3;
  logFilename = "";
  pngFilename = "";
  travmapFilename = "";
  costmapFilename = "";
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
	setupName = argv[ii];
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
	  is >> resolution;
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
	  is >> inscribedRadius;
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
	  is >> circumscribedRadius;
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
	  is >> inflationRadius;
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
 	  cerr << argv[0] << ": -l requires logging filename argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	logFilename = argv[ii];
 	break;
	
      case 'P':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -P requires screenshot filename argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	pngFilename = argv[ii];
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
	cout << "mpbench-" << summarizeOptions() << "\n";
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
  
  if (logFilename.empty())
    logFilename = "mpbench-" + summarizeOptions() + ".txt";
  
  if (pngFilename.empty())
    pngFilename = "mpbench-" + summarizeOptions() + ".png";
}


void create_setup()
{
  bool use_sfl_cost;
  if ("costmap_2d" == costmapType)
    use_sfl_cost = false;
  else if ("sfl" == costmapType)
    use_sfl_cost = true;
  else
    errx(EXIT_FAILURE, "create_setup(): unknown costmapType \"%s\", use costmap_2d or sfl", costmapType.c_str());
  
  *logos << "creating setup \"" << setupName << "\"\n" << flush;
  {
    shared_ptr<ostream> dump_os;
    if ( ! travmapFilename.empty()) {
      dump_os.reset(new ofstream(travmapFilename.c_str()));
      if ( ! (*dump_os)) {
	*logos << "could not open travmap file " << travmapFilename << "\n" << flush;
	dump_os.reset();
      }
    }
    setup.reset(OfficeBenchmark::create(setupName, resolution, inscribedRadius, circumscribedRadius, inflationRadius,
					obstacleCost, use_sfl_cost, doorWidth, hallWidth,
					logos.get(), dump_os.get()));
    if ( ! setup)
      errx(EXIT_FAILURE, "could not create setup with name \"%s\"", setupName.c_str());
    if ( ! costmapFilename.empty()) {
      dump_os.reset(new ofstream(costmapFilename.c_str()));
      if ( ! (*dump_os))
	*logos << "could not open costmap file " << costmapFilename << "\n" << flush;
      else {
	*logos << "writing costmap_2d::CostMap2d\n";
	*dump_os << setup->getCostmap().toString();
      }
    }
  }
  setup->dumpDescription(*logos, "", "  ");
  *logos << flush;
  
  *logos << "creating environment wrapper for map type " << costmapType << "\n" << flush;
  environment.reset(new EnvironmentWrapper2D(setup->getCostmap(), 0, 0, 0, 0,
					     costmap_2d::CostMap2D::INSCRIBED_INFLATED_OBSTACLE));
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
  
  *logos << "creating planner stats\n" << flush;
  plannerStats.reset(new SBPLPlannerStatistics());
}


void run_tasks()
{  
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
      costmap.WC_MC(statsEntry.start.x, statsEntry.start.y,
		    statsEntry.startIx, statsEntry.startIy);
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
      if (task.from_scratch)
	plannerMgr->force_planning_from_scratch();
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
	plan_t plan;
	double prevx(0), prevy(0), prevth(0);
	prevth = 42.17;	// to detect when it has been initialized (see 42 below)
	for(vector<int>::const_iterator it = solutionStateIDs.begin();
	    it != solutionStateIDs.end(); ++it){
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
	planList.push_back(plan);
      }
      
      char const * title("  SUCCESS");
      if ((1 != statsEntry.status) || (1 >= solutionStateIDs.size()))
	title = "  FAILURE";
      statsEntry.logStream(*logos, title, "    ");
      *logos << flush;
    }
  }
}


static void init_layout();
static void draw();
static void reshape(int width, int height);
static void keyboard(unsigned char key, int mx, int my);
////static void timer(int handle);

namespace npm {
  
  // I can't remember why I never put this into nepumuk... probably
  // there was a good reason (like supporting switchable layouts), so
  // I do it here instead of risking breakage elsewhere.
  template<>
  shared_ptr<UniqueManager<View> > Instance()
  {
    static shared_ptr<UniqueManager<View> > instance;
    if( ! instance)
      instance.reset(new UniqueManager<View>());
    return instance;
  }
  
}


void display(int * argc, char ** argv)
{
  init_layout();		// create views and such

  {
    double x0, y0, x1, y1;
    setup->getWorkspaceBounds(x0, y0, x1, y1);
    glut_width = (int) ceil((y1 - y0) / resolution);
    glut_height = (int) ceil((x1 - x0) / resolution);
  }
  
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowPosition(10, 10);
  glutInitWindowSize(glut_width, glut_height);
  int const handle(glutCreateWindow(setupName.c_str()));
  if (0 == handle)
    errx(EXIT_FAILURE, "glutCreateWindow(%s) failed",setupName.c_str());
  
  glutDisplayFunc(draw);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  ////glutTimerFunc(glut_timer_ms, timer, handle);
  
  glutMainLoop();
}


namespace {
  
  class PlanDrawing: npm::Drawing {
  public:
    PlanDrawing(std::string const & name);
    virtual void Draw();
  };
  
  class CostMapProxy: public npm::TravProxyAPI {
  public:
    CostMapProxy()
      : costmap(setup->getCostmap()), gframe(setup->resolution) {}
    
    virtual bool Enabled() const { return true; }
    virtual double GetX() const { return 0; }
    virtual double GetY() const { return 0; }
    virtual double GetTheta() const { return 0; }
    virtual double GetDelta() const { return gframe.Delta(); }
    virtual sfl::GridFrame const * GetGridFrame() { return &gframe; }
    virtual int GetObstacle() const
    { return costmap_2d::ObstacleMapAccessor::INSCRIBED_INFLATED_OBSTACLE; }
    virtual int GetFreespace() const { return 0; }
    virtual ssize_t GetXBegin() const { return 0; }
    virtual ssize_t GetXEnd() const { return costmap.getWidth(); }
    virtual ssize_t GetYBegin() const { return 0; }
    virtual ssize_t GetYEnd() const { return costmap.getHeight(); }
    virtual int GetValue(ssize_t ix, ssize_t iy) const { return costmap.getCost(ix, iy); }
    
    costmap_2d::CostMap2D const & costmap;
    sfl::GridFrame gframe;
  };
  
}


void init_layout()
{
  double x0, y0, x1, y1;
  setup->getWorkspaceBounds(x0, y0, x1, y1);
  new npm::StillCamera("travmap",
		       x0, y0, x1, y1,
   		       npm::Instance<npm::UniqueManager<npm::Camera> >());
  
  shared_ptr<npm::TravProxyAPI> rdt(new npm::RDTravProxy(setup->getRDTravmap()));
  // //  new npm::TraversabilityDrawing("travmap", rdt);
  new npm::TraversabilityDrawing("costmap", new CostMapProxy());
  new PlanDrawing("plan");
  
  npm::View * view;
  // //   view = new npm::View("travmap", npm::Instance<npm::UniqueManager<npm::View> >());
  // //   view->Configure(0, 0, 0.5, 1);
  // //   view->SetCamera("travmap");
  // //   if ( ! view->AddDrawing("travmap"))
  // //     errx(EXIT_FAILURE, "no drawing called \"travmap\"");
  // //   if ( ! view->AddDrawing("plan"))
  // //     errx(EXIT_FAILURE, "no drawing called \"plan\"");
  
  view = new npm::View("costmap", npm::Instance<npm::UniqueManager<npm::View> >());
  // //  view->Configure(0.5, 0, 0.5, 1);
  view->Configure(0, 0, 1, 1);
  view->SetCamera("travmap");
  if ( ! view->AddDrawing("costmap"))
    errx(EXIT_FAILURE, "no drawing called \"costmap\"");
  if ( ! view->AddDrawing("plan"))
    errx(EXIT_FAILURE, "no drawing called \"plan\"");
}


static void make_screenshot()
{
  npm::SimpleImage image(glut_width, glut_height);
  image.read_framebuf(0, 0);
  image.write_png(pngFilename);
  *logos << "saved screenshot " << pngFilename << "\n" << flush;
  cout << "saved screenshot " << pngFilename << "\n" << flush;
}


void draw()
{
  if (websiteMode) {
    double x0, y0, x1, y1;
    setup->getWorkspaceBounds(x0, y0, x1, y1);
    glut_width = (int) ceil((y1 - y0) / resolution);
    glut_height = (int) ceil((x1 - x0) / resolution);
    reshape(glut_width, glut_height);
    glClear(GL_COLOR_BUFFER_BIT);
    npm::Instance<npm::UniqueManager<npm::View> >()->Walk(npm::View::DrawWalker());
    glFlush();
    glutSwapBuffers();
    make_screenshot();
    
    while (glut_width > 200) { 	// wow what a hack
      glut_width /= 2;
      glut_height /= 2;
    }
    reshape(glut_width, glut_height);
    glClear(GL_COLOR_BUFFER_BIT);
    npm::Instance<npm::UniqueManager<npm::View> >()->Walk(npm::View::DrawWalker());
    glFlush();
    glutSwapBuffers();
    pngFilename = "small-" + pngFilename; // and another one
    make_screenshot();
    
    exit(EXIT_SUCCESS);
  }
  
  glClear(GL_COLOR_BUFFER_BIT);
  npm::Instance<npm::UniqueManager<npm::View> >()->Walk(npm::View::DrawWalker());
  glFlush();
  glutSwapBuffers();

  if ( ! made_first_screenshot) {
    make_screenshot();
    made_first_screenshot = true;
  }
}


void reshape(int width, int height)
{
  glut_width = width;
  glut_height = height;
  npm::Instance<npm::UniqueManager<npm::View> >()->Walk(npm::View::ReshapeWalker(width, height));
}


void keyboard(unsigned char key, int mx, int my)
{
  switch (key) {
  case 'p':
    make_screenshot();
    break;
  case 'q':
    errx(EXIT_SUCCESS, "key: q");
  }
}


namespace {
  
  PlanDrawing::
  PlanDrawing(std::string const & name)
    : npm::Drawing(name,
		   "the plans that ... were planned",
		   npm::Instance<npm::UniqueManager<npm::Drawing> >())
  {
  }
  
  
  void PlanDrawing::
  Draw()
  {
    for (size_t ii(0); ii < planList.size(); ++ii) {
      glColor3d(1, 1, 0);
      glLineWidth(2);
      glBegin(GL_LINE_STRIP);
      for (plan_t::const_iterator ip(planList[ii].begin()); ip != planList[ii].end(); ++ip)
	glVertex2d(ip->x, ip->y);
      glEnd();
    }
    
    SBPLBenchmarkSetup::tasklist_t const & tl(setup->getTasks());
    glColor3d(1, 0.5, 0);
    glLineWidth(1);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glMatrixMode(GL_MODELVIEW);
    for (size_t ii(0); ii < tl.size(); ++ii) {
      glPushMatrix();
      glTranslated(tl[ii].goal_x, tl[ii].goal_y, 0);
      gluDisk(wrap_glu_quadric_instance(), tl[ii].goal_tol_xy, tl[ii].goal_tol_xy, 36, 1);
      glPopMatrix();
    }
  }
  
}
