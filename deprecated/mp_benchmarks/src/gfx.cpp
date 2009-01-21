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

#include "gfx.hpp"

// XXXX should be <sbpl_util/blah> or so
#include <environment_wrap.h>

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

using namespace ompl;
using namespace boost;
using namespace std;


static int glut_width(400);
static int glut_height(400);
static int glut_handle(0);
static bool made_first_screenshot(false);
static ompl::gfx::Configuration const * configptr(0);
static double glut_aspect(1); 	// desired width / height (defined in init_layout_X())

static void init_layout_one();
static void init_layout_two();
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


namespace ompl {
  namespace gfx {
    
    
    Configuration::
    Configuration(SBPLBenchmarkSetup const & _setup,
		  EnvironmentWrapper const & _environment,
		  SBPLBenchmarkOptions const & opt,
		  bool _websiteMode,
		  std::string const & _baseFilename,
		  footprint_t const & _footprint,
		  planList_t const & _planList,
		  bool _ignorePlanTheta,
		  std::ostream & _logOs)
      : setup(_setup),
	environment(_environment),
	resolution(opt.resolution),
	inscribedRadius(opt.inscribed_radius),
	circumscribedRadius(opt.circumscribed_radius),
	websiteMode(_websiteMode),
	baseFilename(_baseFilename),
	footprint(_footprint),
	planList(_planList),
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
      default:
	errx(EXIT_FAILURE, "ompl::gfx::display(): invalid layoutID %zu", layoutID);
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
	errx(EXIT_FAILURE, "ompl::gfx::display(): glutCreateWindow(%s) failed", windowName);
      
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
    PlanDrawing(std::string const & name, ssize_t taskNumber);
    virtual void Draw();
    
    ssize_t const taskNumber;
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
    
    boost::shared_ptr<CostmapWrap const> costmap;
    sfl::GridFrame gframe;
  };
  
  class EnvWrapProxy: public CostMapProxy {
  public:
    EnvWrapProxy() {
      if (dynamic_cast<EnvironmentWrapper3DKIN const *>(&configptr->environment))
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
  new PlanDrawing("plan", -1);
  
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
  SBPLBenchmarkSetup::tasklist_t const & tl(configptr->setup.getTasks());
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
  
  shared_ptr<npm::TravProxyAPI> rdt(new npm::RDTravProxy(configptr->setup.getRawSFLTravmap()));
  new npm::TraversabilityDrawing("costmapwrap", new CostMapProxy());
  double const v_width(2.0 / tl.size());
  for (int ix(0), itask(0); itask < tl.size(); ++ix)
    for (int iy(1); iy >= 0; --iy, ++itask) {
      ostringstream pdname;
      pdname << "plan" << itask;
      new PlanDrawing(pdname.str(), itask);
      npm::View *
	view(new npm::View(pdname.str(), npm::Instance<npm::UniqueManager<npm::View> >()));
      // beware of weird npm::View::Configure() param order: x, y, width, height
      view->Configure(ix * v_width, iy * 0.5, v_width, 0.5);
      view->SetCamera("travmap");
      if ( ! view->AddDrawing("costmapwrap"))
	errx(EXIT_FAILURE, "no drawing called \"costmapwrap\"");
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
  PlanDrawing(std::string const & name, ssize_t _taskNumber)
    : npm::Drawing(name,
		   "the plans that ... were planned",
		   npm::Instance<npm::UniqueManager<npm::Drawing> >()),
      taskNumber(_taskNumber)
  {
  }
  
  
  static void drawFootprint()
  {
    glBegin(GL_LINE_LOOP);
    for (size_t jj(0); jj < configptr->footprint.size(); ++jj)
      glVertex2d(configptr->footprint[jj].x, configptr->footprint[jj].y);
    glEnd();
  }
  
  
  static void drawPlan(planList_t::const_iterator pbegin,
		       planList_t::const_iterator pend,
		       bool detailed)
  {
    typedef planList_t::const_iterator pli_t;
    typedef planBundle_t::const_iterator pbi_t;
    typedef waypoint_plan_t::const_iterator wpi_t;
    
    glMatrixMode(GL_MODELVIEW);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    
    // workaround for segfault on glTranslated(iw->x, iw->y, 0);
#ifdef UNDEFINED
    if (detailed) {
      // trace of thin footprints or inscribed circles along path
      double const llen(configptr->inscribedRadius / configptr->resolution / 2);
      size_t const skip(static_cast<size_t>(ceil(llen)));
      glColor3d(0.5, 1, 0);
      glLineWidth(1);
      if (configptr->ignorePlanTheta) {
	for (pli_t il(pbegin); il != pend; ++il)
	  for (pbi_t ib(il->second.begin()); ib != il->second.end(); ++ib)
	    for (wpi_t iw((*ib)->begin()); iw != (*ib)->end(); iw += skip) {
	      glPushMatrix();
	      glTranslated(iw->x, iw->y, 0);
	      gluDisk(wrap_glu_quadric_instance(),
		      configptr->inscribedRadius,
		      configptr->inscribedRadius,
		      36, 1);
	      glPopMatrix();
	    }
      }
      else {
	for (pli_t il(pbegin); il != pend; ++il)
	  for (pbi_t ib(il->second.begin()); ib != il->second.end(); ++ib)
	    for (wpi_t iw((*ib)->begin()); iw != (*ib)->end(); iw += skip) {
	      glPushMatrix();
	      glTranslated(iw->x, iw->y, 0);
	      glRotated(180 * iw->th / M_PI, 0, 0, 1);
	      drawFootprint();
	      glPopMatrix();
	    }
      }
    }
#endif // UNDEFINED
    
    // start and goal, with inscribed, circumscribed, and thick footprint
    SBPLBenchmarkSetup::tasklist_t const & tl(configptr->setup.getTasks());
    glColor3d(0.5, 1, 0);
    for (pli_t il(pbegin); il != pend; ++il) {
      glPushMatrix();
      glTranslated(tl[il->first].start_x, tl[il->first].start_y, 0);
      gluDisk(wrap_glu_quadric_instance(),
	      configptr->inscribedRadius,
	      configptr->inscribedRadius,
	      36, 1);
      glPopMatrix();
    }
    for (pli_t il(pbegin); il != pend; ++il) {
      glPushMatrix();
      glTranslated(tl[il->first].start_x, tl[il->first].start_y, 0);
      gluDisk(wrap_glu_quadric_instance(),
	      configptr->circumscribedRadius,
	      configptr->circumscribedRadius,
	      36, 1);
      glPopMatrix();
    }    
    glColor3d(1, 1, 0);
    glLineWidth(3);
    for (pli_t il(pbegin); il != pend; ++il) {
      glPushMatrix();
      glTranslated(tl[il->first].start_x, tl[il->first].start_y, 0);
      glRotated(180 * tl[il->first].start_th / M_PI, 0, 0, 1);
      drawFootprint();
      glPopMatrix();
    }

    // goal tolerance
    //  glColor3d(1, 0.5, 0);
    //  for (vector<size_t>::const_iterator itask(tnums.begin()); itask != tnums.end(); ++itask) {
    //     glPushMatrix();
    //     glTranslated(tl[il->first].goal_x, tl[il->first].goal_y, 0);
    //     gluDisk(wrap_glu_quadric_instance(), tl[il->first].goal_tol_xy, tl[il->first].goal_tol_xy, 36, 1);
    //     glPopMatrix();
    //   }
    
    // path yellow
    glColor3d(1, 1, 0);
    if (detailed)
      glLineWidth(3);		// make it stand out among the rest
    else
      glLineWidth(1);
    for (pli_t il(pbegin); il != pend; ++il)
      for (pbi_t ib(il->second.begin()); ib != il->second.end(); ++ib) {
	glBegin(GL_LINE_STRIP);
	for (wpi_t iw((*ib)->begin()); iw != (*ib)->end(); ++iw)
	  glVertex2d(iw->x, iw->y);
	glEnd();
      }
  }
  
  
  void PlanDrawing::
  Draw()
  {
    if (0 > taskNumber)
      drawPlan(configptr->planList.begin(), configptr->planList.end(), true);
    else {
      // this is historically overly generic (works for multimaps as
      // well as maps), but should not hurt
      pair<planList_t::const_iterator, planList_t::const_iterator>
	prange(configptr->planList.equal_range(taskNumber));
      drawPlan(prange.first, prange.second, false);
    }
  }
  
}
