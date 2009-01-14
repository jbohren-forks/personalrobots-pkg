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

#include "gfx.h"

#include <mpglue/environment.h>
#include <costmap_2d/costmap_2d.h>
#include <npm/common/wrap_glu.hpp>
#include <npm/common/wrap_glut.hpp>
#include <npm/common/Manager.hpp>
#include <npm/common/View.hpp>
#include <npm/common/StillCamera.hpp>
#include <npm/common/TraversabilityDrawing.hpp>
#include <npm/common/SimpleImage.hpp>
#include <boost/shared_ptr.hpp>

extern "C" {
#include <err.h>
}

using namespace mpbench;
using namespace mpglue;
using namespace boost;
using namespace std;


static int glut_width(400);
static int glut_height(400);
static int glut_handle(0);
static bool made_first_screenshot(false);
static gfx::Configuration const * configptr(0);
static double glut_aspect(1); 	// desired width / height (defined in init_layout_X())

static void init_layout_one();
static void init_layout_two();
static void init_layout_three();
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


namespace mpbench {
  namespace gfx {
    
    
    Configuration::
    Configuration(Setup const & _setup,
		  std::vector<std::string> const & _planner_name,
		  Environment const & _environment,
		  SetupOptions const & opt,
		  bool _websiteMode,
		  std::string const & _baseFilename,
		  footprint_t const & _footprint,
		  resultlist_t const & _resultlist,
		  bool _ignorePlanTheta,
		  std::ostream & _logOs)
      : setup(_setup),
	planner_name(_planner_name),
	environment(_environment),
	resolution(opt.resolution),
	inscribedRadius(opt.inscribed_radius),
	circumscribedRadius(opt.circumscribed_radius),
	websiteMode(_websiteMode),
	baseFilename(_baseFilename),
	footprint(_footprint),
	resultlist(_resultlist),
	ignorePlanTheta(_ignorePlanTheta),
	logOs(_logOs)
    {
    }
    
    
    void display(Configuration const & config, char const * windowName,
		 size_t layoutID,
		 int * argc, char ** argv)
    {
      configptr = &config;

      // yes, this is a hack.
      switch (layoutID) {
      case 1:
	init_layout_one();
	break;
      case 2:
	init_layout_two();
	break;
      case 3:
	init_layout_three();
	break;
      default:
	errx(EXIT_FAILURE, "mpbench::gfx::display(): invalid layoutID %zu", layoutID);
      }
      
      double x0, y0, x1, y1;
      configptr->setup.getWorkspaceBounds(x0, y0, x1, y1);
      glut_width = (int) ceil(glut_aspect * (y1 - y0) / configptr->resolution);
      glut_height = (int) ceil((x1 - x0) / configptr->resolution);
      while (glut_height < 800) { // wow, hack
	glut_width *= 2;
	glut_height *= 2;
      }
      
      glutInit(argc, argv);
      glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
      glutInitWindowPosition(10, 10);
      glutInitWindowSize(glut_width, glut_height);
      
      glut_handle = glutCreateWindow(windowName);
      if (0 == glut_handle)
	errx(EXIT_FAILURE, "mpbench::gfx::display(): glutCreateWindow(%s) failed", windowName);
      
      glutDisplayFunc(draw);
      glutReshapeFunc(reshape);
      glutKeyboardFunc(keyboard);
      ////glutTimerFunc(glut_timer_ms, timer, handle);
      
      made_first_screenshot = false;
      glutMainLoop();
    }

  }
}


namespace {
  
  class PlanDrawing: npm::Drawing {
  public:
    PlanDrawing(std::string const & name,
		ssize_t task_id, ssize_t task_id, ssize_t episode_id,
		bool detailed);
    virtual void Draw();
    
    ssize_t const planner_id;
    ssize_t const task_id;
    ssize_t const episode_id;
    bool const detailed;
  };
  
  class CostMapProxy: public npm::TravProxyAPI {
  public:
    CostMapProxy()
      : costmap(configptr->setup.getRaw2DCostmap()), gframe(configptr->setup.resolution) {}
    
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
  
  class CostmapWrapProxy: public npm::TravProxyAPI {
  public:
    CostmapWrapProxy()
      : costmap(configptr->setup.getCostmap()), gframe(configptr->setup.resolution) {}
    
    virtual bool Enabled() const { return true; }
    virtual double GetX() const { return 0; }
    virtual double GetY() const { return 0; }
    virtual double GetTheta() const { return 0; }
    virtual double GetDelta() const { return gframe.Delta(); }
    virtual sfl::GridFrame const * GetGridFrame() { return &gframe; }
    virtual int GetObstacle() const { return costmap->getWSpaceObstacleCost(); }
    virtual int GetFreespace() const { return 0; }
    virtual ssize_t GetXBegin() const { return costmap->getXBegin(); }
    virtual ssize_t GetXEnd() const { return costmap->getXEnd(); }
    virtual ssize_t GetYBegin() const { return costmap->getYBegin(); }
    virtual ssize_t GetYEnd() const { return costmap->getYEnd(); }
    virtual int GetValue(ssize_t ix, ssize_t iy) const {
      int cost;
      if (costmap->getCost(ix, iy, &cost))
	return cost;
      return 0; }
    
    boost::shared_ptr<Costmap const> costmap;
    sfl::GridFrame gframe;
  };
  
  class EnvWrapProxy: public CostMapProxy {
  public:
    EnvWrapProxy() {
      if (dynamic_cast<Environment3DKIN const *>(&configptr->environment))
	env3d = true;
      else
	env3d = false; }
    
    virtual int GetObstacle() const {
      if (env3d)
	return 1;
      return costmap_2d::ObstacleMapAccessor::INSCRIBED_INFLATED_OBSTACLE; }

    virtual int GetValue(ssize_t ix, ssize_t iy) const {
      if (env3d) {
	if (configptr->environment.IsObstacle(ix, iy, false))
	  return 1;
	return 0;
      }
      return costmap.getCost(ix, iy); }
    
    bool env3d;
  };
  
}


void init_layout_one()
{
  double x0, y0, x1, y1;
  configptr->setup.getWorkspaceBounds(x0, y0, x1, y1);
  new npm::StillCamera("travmap",
		       x0, y0, x1, y1,
   		       npm::Instance<npm::UniqueManager<npm::Camera> >());
  glut_aspect = 3 * (x1 - x0) / (y1 - y0);
  
  shared_ptr<npm::TravProxyAPI> rdt(new npm::RDTravProxy(configptr->setup.getRawSFLTravmap()));
  new npm::TraversabilityDrawing("travmap", rdt);
  new npm::TraversabilityDrawing("costmap", new CostMapProxy());
  new npm::TraversabilityDrawing("envwrap", new EnvWrapProxy());
  new PlanDrawing("plan", -1, -1, -1, false);
  
  npm::View * view;
  
  view = new npm::View("travmap", npm::Instance<npm::UniqueManager<npm::View> >());
  // beware of weird npm::View::Configure() param order: x, y, width, height
  view->Configure(0, 0, 0.33, 1);
  view->SetCamera("travmap");
  if ( ! view->AddDrawing("travmap"))
    errx(EXIT_FAILURE, "no drawing called \"travmap\"");
  if ( ! view->AddDrawing("plan"))
    errx(EXIT_FAILURE, "no drawing called \"plan\"");
    
  view = new npm::View("costmap", npm::Instance<npm::UniqueManager<npm::View> >());
  // beware of weird npm::View::Configure() param order: x, y, width, height
  view->Configure(0.33, 0, 0.33, 1);
  view->SetCamera("travmap");
  if ( ! view->AddDrawing("costmap"))
    errx(EXIT_FAILURE, "no drawing called \"costmap\"");
  if ( ! view->AddDrawing("plan"))
    errx(EXIT_FAILURE, "no drawing called \"plan\"");
    
  view = new npm::View("envwrap", npm::Instance<npm::UniqueManager<npm::View> >());
  // beware of weird npm::View::Configure() param order: x, y, width, height
  view->Configure(0.66, 0, 0.33, 1);
  view->SetCamera("travmap");
  if ( ! view->AddDrawing("envwrap"))
    errx(EXIT_FAILURE, "no drawing called \"envwrap\"");
  if ( ! view->AddDrawing("plan"))
    errx(EXIT_FAILURE, "no drawing called \"plan\"");
}


void init_layout_two()
{
  tasklist_t const & tl(configptr->setup.getTasks());
  if (tl.empty()) {
    // avoid div by zero
    init_layout_one();
    return;
  }
  
  double x0, y0, x1, y1;
  configptr->setup.getWorkspaceBounds(x0, y0, x1, y1);
  new npm::StillCamera("travmap",
		       x0, y0, x1, y1,
   		       npm::Instance<npm::UniqueManager<npm::Camera> >());
  glut_aspect = ceil(tl.size() * 0.5) * (x1 - x0) / (2 * (y1 - y0));
  
  new npm::TraversabilityDrawing("costmap", new CostMapProxy());
  double const v_width(2.0 / tl.size());
  for (int ix(0), itask(0); itask < static_cast<int>(tl.size()); ++ix)
    for (int iy(1); iy >= 0; --iy, ++itask) {
      ostringstream pdname;
      pdname << "plan" << itask;
      new PlanDrawing(pdname.str(), -1, itask, -1, true);
      npm::View *
	view(new npm::View(pdname.str(), npm::Instance<npm::UniqueManager<npm::View> >()));
      // beware of weird npm::View::Configure() param order: x, y, width, height
      view->Configure(ix * v_width, iy * 0.5, v_width, 0.5);
      view->SetCamera("travmap");
      if ( ! view->AddDrawing("costmap"))
	errx(EXIT_FAILURE, "no drawing called \"costmap\"");
      if ( ! view->AddDrawing(pdname.str()))
	errx(EXIT_FAILURE, "no drawing called \"%s\"", pdname.str().c_str());
    }
}


void init_layout_three()
{  
  double x0, y0, x1, y1;
  configptr->setup.getWorkspaceBounds(x0, y0, x1, y1);
  new npm::StillCamera("travmap",
		       x0, y0, x1, y1,
   		       npm::Instance<npm::UniqueManager<npm::Camera> >());
  size_t const nplanners(configptr->planner_name.size());
  double const ncols(nplanners + 1.0);
  glut_aspect = ncols * (x1 - x0) / (y1 - y0);
  
  new npm::TraversabilityDrawing("costmap", new CostMapProxy());
  double const v_width(1.0 / ncols);
  npm::View * view(new npm::View("costmap", npm::Instance<npm::UniqueManager<npm::View> >()));
  // beware of weird npm::View::Configure() param order: x, y, width, height
  view->Configure(0, 0, v_width, 1);
  if ( ! view->SetCamera("travmap"))
    errx(EXIT_FAILURE, "no camera called \"travmap\"");
  if ( ! view->AddDrawing("costmap"))
    errx(EXIT_FAILURE, "no drawing called \"costmap\"");

  new npm::TraversabilityDrawing("costmap_dark", new CostMapProxy(),
				 npm::TraversabilityDrawing::MINIMAL_DARK);
  for (size_t ip(0); ip < nplanners; ++ip) {
    ostringstream pdname;
    pdname << "planner" << ip;
    new PlanDrawing(pdname.str(), ip, -1, -1, false);
    view = new npm::View(pdname.str(), npm::Instance<npm::UniqueManager<npm::View> >());
    // beware of weird npm::View::Configure() param order: x, y, width, height
    view->Configure((ip + 1) * v_width, 0, v_width, 1);
    if ( ! view->SetCamera("travmap"))
      errx(EXIT_FAILURE, "no camera called \"travmap\"");
    if ( ! view->AddDrawing("costmap_dark"))
      errx(EXIT_FAILURE, "no drawing called \"costmap_dark\"");
    if ( ! view->AddDrawing(pdname.str()))
      errx(EXIT_FAILURE, "no drawing called \"%s\"", pdname.str().c_str());
  }
}


static void make_screenshot(string const & namePrefix)
{
  npm::SimpleImage image(glut_width, glut_height);
  string pngFilename(namePrefix + configptr->baseFilename + ".png");
  image.read_framebuf(0, 0);
  image.write_png(pngFilename);
  configptr->logOs << "saved screenshot " << pngFilename << "\n" << flush;
  cout << "saved screenshot " << pngFilename << "\n" << flush;
}


void draw()
{
  if (configptr->websiteMode) {
    double x0, y0, x1, y1;
    configptr->setup.getWorkspaceBounds(x0, y0, x1, y1);
    //     glut_width = (int) ceil((y1 - y0) / configptr->resolution);
    //     glut_height = (int) ceil((x1 - x0) / configptr->resolution);
    //     reshape(glut_width, glut_height);
    glClear(GL_COLOR_BUFFER_BIT);
    npm::Instance<npm::UniqueManager<npm::View> >()->Walk(npm::View::DrawWalker());
    glFlush();
    glutSwapBuffers();
    make_screenshot("");
    
    while (glut_width > 400) { 	// wow what a hack
      glut_width /= 2;
      glut_height /= 2;
    }
    reshape(glut_width, glut_height);
    glClear(GL_COLOR_BUFFER_BIT);
    npm::Instance<npm::UniqueManager<npm::View> >()->Walk(npm::View::DrawWalker());
    glFlush();
    glutSwapBuffers();
    make_screenshot("small-");
    
    exit(EXIT_SUCCESS);
  }
  
  glClear(GL_COLOR_BUFFER_BIT);
  npm::Instance<npm::UniqueManager<npm::View> >()->Walk(npm::View::DrawWalker());
  glFlush();
  glutSwapBuffers();

  if ( ! made_first_screenshot) {
    make_screenshot("");
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
    make_screenshot("");
    break;
  case 'q':
    errx(EXIT_SUCCESS, "key: q");
  }
}


namespace {
  
  PlanDrawing::
  PlanDrawing(std::string const & name,
	      ssize_t _planner_id, ssize_t _task_id, ssize_t _episode_id,
	      bool _detailed)
    : npm::Drawing(name,
		   "the plans that ... were planned",
		   npm::Instance<npm::UniqueManager<npm::Drawing> >()),
      planner_id(_planner_id),
      task_id(_task_id),
      episode_id(_episode_id),
      detailed(_detailed)
  {
  }
  
  
  static void drawFootprint()
  {
    glBegin(GL_LINE_LOOP);
    for (size_t jj(0); jj < configptr->footprint.size(); ++jj)
      glVertex2d(configptr->footprint[jj].x, configptr->footprint[jj].y);
    glEnd();
  }
  
  
  static void drawResult(task::result const & result, bool detailed)
  {
    typedef waypoint_plan_t::const_iterator wpi_t;
    shared_ptr<mpglue::waypoint_plan_t> plan(result.plan);
    
    glMatrixMode(GL_MODELVIEW);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    
    if (detailed) {
      // trace of thin footprints or inscribed circles along path
      double const llen(configptr->inscribedRadius / configptr->resolution / 2);
      size_t const skip(static_cast<size_t>(ceil(llen)));
      glColor3d(0.5, 1, 0);
      glLineWidth(1);
      if (plan) {
	for (wpi_t iw(plan->begin()); iw != plan->end(); iw += skip) {
	  glPushMatrix();
	  glTranslated(iw->x, iw->y, 0);
	  if (configptr->ignorePlanTheta) {
	    gluDisk(wrap_glu_quadric_instance(),
		    configptr->inscribedRadius,
		    configptr->inscribedRadius,
		    36, 1);
	  }
	  else {
	    glRotated(180 * iw->th / M_PI, 0, 0, 1);
	    drawFootprint();
	  }
	  glPopMatrix();
	} // endfor(plan)
      }	// endif(plan)
    } // endif(detailed)
    
    task::startspec const & start(result.start);
    glPushMatrix();
    glTranslated(start.px, start.py, 0);
    //       glColor3d(0.5, 1, 0);
    //       glLineWidth(1);
    //       gluDisk(wrap_glu_quadric_instance(),
    // 	      configptr->inscribedRadius,
    // 	      configptr->inscribedRadius,
    // 	      36, 1);
    //       gluDisk(wrap_glu_quadric_instance(),
    // 	      configptr->circumscribedRadius,
    // 	      configptr->circumscribedRadius,
    // 	      36, 1);
    glColor3d(1, 1, 0);
    glLineWidth(3);
    glRotated(180 * start.pth / M_PI, 0, 0, 1);
    drawFootprint();
    glPopMatrix();
    
    task::goalspec const & goal(result.goal);
    glPushMatrix();
    glTranslated(goal.px, goal.py, 0);
    glColor3d(1, 0.5, 0);
    glLineWidth(1);
    gluDisk(wrap_glu_quadric_instance(),
	    goal.tol_xy,
	    goal.tol_xy,
	    36, 1);
    glPopMatrix();
    
    if (plan) {
      glColor3d(1, 1, 0);
      if (detailed)
	glLineWidth(3);		// make it stand out among the rest
      else
	glLineWidth(1);
      glBegin(GL_LINE_STRIP);
      for (wpi_t iw(plan->begin()); iw != plan->end(); ++iw)
	glVertex2d(iw->x, iw->y);
      glEnd();
    }
  }
  
  
  void PlanDrawing::
  Draw()
  {
    for (resultlist_t::const_iterator ir(configptr->resultlist.begin());
	 ir != configptr->resultlist.end(); ++ir) {
      shared_ptr<task::result> result(*ir);
      if ( ! result)
	continue;
      if ((planner_id >= 0) && (planner_id != result->planner_id))
	continue;
      if ((task_id >= 0) && (task_id != result->task_id))
	continue;
      if ((episode_id >= 0) && (episode_id != result->episode_id))
	continue;
      drawResult(*result, detailed);
    }
  }
  
}
