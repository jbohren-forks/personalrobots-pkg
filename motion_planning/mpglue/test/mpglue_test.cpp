/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include <gtest/gtest.h>
#include <costmap_2d/costmap_2d.h>
#include <mpglue/costmapper.h>

using namespace std;
using namespace boost;


namespace mpglue_test {
  
  class Factory {
  public:
    string const name;
    explicit Factory(string const & _name): name(_name) {}
    virtual ~Factory() {}
    virtual shared_ptr<mpglue::Costmapper> createCostmapper() = 0;
  };
  
  
  class LocalFactory: public Factory {
  public:
    LocalFactory(): Factory("local") {}
    virtual shared_ptr<mpglue::Costmapper> createCostmapper() {
      static double const bbx0_(-1);
      static double const bby0_(-1);
      static double const bbx1_(1);
      static double const bby1_(1);
      static double const costmap_resolution(0.1);
      static double const costmap_inscribed_radius(0.15);
      static double const costmap_circumscribed_radius(0.25);
      static double const costmap_inflation_radius(0.35);
      double const origin_x(bbx0_);
      double const origin_y(bby0_);
      unsigned int const
	cells_size_x(1+static_cast<unsigned int>(ceil((bbx1_ - bbx0_) / costmap_resolution)));
      unsigned int const
	cells_size_y(1+static_cast<unsigned int>(ceil((bby1_ - bby0_) / costmap_resolution)));
      double const obstacle_range(8); // whatever
      double const max_obstacle_height(1); // whatever
      double const raytrace_range(8);	   // whatever
      double const weight(1); // whatever: documentation says 0<w<=1 but default is 25???
      vector<unsigned char> static_data(cells_size_x * cells_size_y);
      unsigned char const lethal_threshold(1);
      shared_ptr<costmap_2d::Costmap2D>
	cm2d(new costmap_2d::Costmap2D(cells_size_x,
				       cells_size_y,
				       costmap_resolution,
				       origin_x,
				       origin_y,
				       costmap_inscribed_radius,
				       costmap_circumscribed_radius,
				       costmap_inflation_radius,
				       obstacle_range,
				       max_obstacle_height,
				       raytrace_range,
				       weight,
				       static_data,
				       lethal_threshold));
      return mpglue::createCostmapper(cm2d);
    }
  };
  
  
  class SFLFactory: public Factory {
  public:
    SFLFactory(): Factory("sfl") {}
    virtual shared_ptr<mpglue::Costmapper> createCostmapper() {
      static double const origin_x(-2);
      static double const origin_y(3);
      static double const resolution(0.05);
      static double const inscribed_radius(0.2);
      static double const circumscribed_radius(0.25);
      static double const inflation_radius(0.3);
      static mpglue::index_t const ix_end(100);
      static mpglue::index_t const iy_end(75);
      static double const origin_th(-0.032);
      static mpglue::index_t const ix_begin(-10);
      static mpglue::index_t const iy_begin(32);
      static int const obstacle_cost(87);
      return mpglue::createCostmapper(mpglue::sfl_costmapper_params(origin_x,
								    origin_y,
								    origin_th,
								    resolution,
								    inscribed_radius,
								    circumscribed_radius,
								    inflation_radius,
								    ix_begin,
								    iy_begin,
								    ix_end,
								    iy_end,
								    obstacle_cost));
    }
  };
  
  
  class CM2DFactory: public Factory {
  public:
    CM2DFactory(): Factory("cm2d") {}
    virtual shared_ptr<mpglue::Costmapper> createCostmapper() {
      static double const origin_x(-2);
      static double const origin_y(3);
      static double const resolution(0.05);
      static double const inscribed_radius(0.2);
      static double const circumscribed_radius(0.25);
      static double const inflation_radius(0.3);
      static mpglue::index_t const ix_end(100);
      static mpglue::index_t const iy_end(75);
      static double const obstacle_range(7);
      static double const max_obstacle_height(3.2);
      static double const raytrace_range(5.7);
      static double const weight(4.9);
      return mpglue::createCostmapper(mpglue::cm2d_costmapper_params(origin_x,
								     origin_y,
								     resolution,
								     inscribed_radius,
								     circumscribed_radius,
								     inflation_radius,
								     ix_end,
								     iy_end,
								     obstacle_range,
								     max_obstacle_height,
								     raytrace_range,
								     weight));
    }
  };
  
  
  class FactoryList {
  public:
    typedef list<Factory*> factory_list_t;
    typedef factory_list_t::iterator iterator;
    
    FactoryList() {
      factory_list.push_back(new LocalFactory());
      factory_list.push_back(new SFLFactory());
      factory_list.push_back(new CM2DFactory());
    }
    
    ~FactoryList() {
      for (iterator ii(begin()); ii != end(); ++ii)
	delete *ii;
    }
    
    inline iterator begin() { return factory_list.begin(); }
    inline iterator end() { return factory_list.end(); }
    
    factory_list_t factory_list;
  };
  
}

using namespace mpglue_test;


TEST (costmapper, factory)
{
  shared_ptr<Factory> factory;
  factory.reset(new LocalFactory());
  factory.reset(new SFLFactory());
  factory.reset(new CM2DFactory());
  FactoryList fl;
}


TEST (costmapper, creation)
{
  FactoryList fl;
  for (FactoryList::iterator ii(fl.begin()); ii != fl.end(); ++ii) {
    Factory * factory(*ii);
    ASSERT_TRUE (factory) << "NULL factory";
    shared_ptr<mpglue::Costmapper> cm(factory->createCostmapper());
    EXPECT_TRUE (cm) << "factory \"" << factory->name << "\" failed";
  }
}


TEST (costmapper, addition)
{
  mpglue::index_collection_t added_obstacle_indices;
  added_obstacle_indices.insert(mpglue::index_pair(2, 3));
  ASSERT_EQ (added_obstacle_indices.size(), size_t(1)) << "empty index_collection";
  FactoryList fl;
  for (FactoryList::iterator ii(fl.begin()); ii != fl.end(); ++ii) {
    Factory * factory(*ii);
    ASSERT_TRUE (factory) << "NULL factory";
    shared_ptr<mpglue::Costmapper> cm(factory->createCostmapper());
    EXPECT_TRUE (cm) << "factory \"" << factory->name << "\" failed";
    if ( ! cm)
      continue;
    size_t const count(cm->updateObstacles(&added_obstacle_indices, 0, &cerr));
    EXPECT_GT (count, size_t(0));
  }
}


TEST (costmapper, removal)
{
  mpglue::index_collection_t added_obstacle_indices;
  mpglue::index_collection_t removed_obstacle_indices;
  added_obstacle_indices.insert(mpglue::index_pair(2, 3));
  removed_obstacle_indices.insert(mpglue::index_pair(2, 3));
  FactoryList fl;
  for (FactoryList::iterator ii(fl.begin()); ii != fl.end(); ++ii) {
    Factory * factory(*ii);
    ASSERT_TRUE (factory) << "NULL factory";
    shared_ptr<mpglue::Costmapper> cm(factory->createCostmapper());
    EXPECT_TRUE (cm) << "factory \"" << factory->name << "\" failed";
    if ( ! cm)
      continue;
    size_t const add_count(cm->updateObstacles(&added_obstacle_indices, 0, &cerr));
    EXPECT_GT (add_count, size_t(0));
    size_t const rem_count(cm->updateObstacles(0, &removed_obstacle_indices, &cerr));
    EXPECT_GT (rem_count, size_t(0));
  }
}


TEST (costmapper, empty_removal)
{
  mpglue::index_collection_t removed_obstacle_indices;
  removed_obstacle_indices.insert(mpglue::index_pair(2, 3));
  FactoryList fl;
  for (FactoryList::iterator ii(fl.begin()); ii != fl.end(); ++ii) {
    Factory * factory(*ii);
    ASSERT_TRUE (factory) << "NULL factory";
    shared_ptr<mpglue::Costmapper> cm(factory->createCostmapper());
    EXPECT_TRUE (cm) << "factory \"" << factory->name << "\" failed";
    if ( ! cm)
      continue;
    size_t const count(cm->updateObstacles(0, &removed_obstacle_indices, &cerr));
    EXPECT_EQ (count, size_t(0));
  }
}


TEST (costmap, raw_accessor)
{
  static size_t const xsize(12);
  static size_t const ysize(43);
  typedef mpglue::RawCostmapAccessor<int, int*, int> raw_costmap_t;
  raw_costmap_t rcm(0, xsize, ysize, 17, 12, 4);
  rcm.raw = new int[xsize * ysize];
  {
    int cost(0);
    for (size_t ix(0); ix < xsize; ++ix)
      for (size_t iy(0); iy < ysize; ++iy) {
	int const raw_idx(rcm.indexToRaw(ix, iy));
	ASSERT_GE (raw_idx, 0) << "negative rcm.indexToRaw()";
	ASSERT_LT (raw_idx, xsize * ysize) << "rcm.indexToRaw() too big";
	rcm.raw[raw_idx] = cost;
	++cost;
	if (cost > 42)
	  cost = 0;
      }
  }
  {
    mpglue::cost_t check(0);
    mpglue::CostmapAccessor const * cma(&rcm);
    ASSERT_EQ (cma->getXBegin(), 0);
    ASSERT_EQ (cma->getXEnd(), rcm.ncells_x);
    ASSERT_EQ (cma->getYBegin(), 0);
    ASSERT_EQ (cma->getYEnd(), rcm.ncells_y);
    for (mpglue::index_t ix(cma->getXBegin()); ix < cma->getXEnd(); ++ix) {
      EXPECT_FALSE (cma->isValidIndex(ix, -1));
      EXPECT_FALSE (cma->isValidIndex(ix, rcm.ncells_y));
      for (mpglue::index_t iy(cma->getYBegin()); iy < cma->getYEnd(); ++iy) {
	mpglue::cost_t cost;
	ASSERT_TRUE (cma->getCost(ix, iy, &cost));
	EXPECT_EQ (cost, check);
	++check;
	if (check > 42)
	  check = 0;
	if (cost >= rcm.lethal_cost)
	  EXPECT_TRUE (cma->isLethal(ix, iy, false))
	    << "[" << ix << "][" << iy << "] should be lethal";
	else
	  EXPECT_FALSE (cma->isLethal(ix, iy, true))
	    << "[" << ix << "][" << iy << "] should not be lethal";
	if (cost >= rcm.inscribed_cost)
	  EXPECT_TRUE (cma->isInscribed(ix, iy, false))
	    << "[" << ix << "][" << iy << "] should be inscribed";
	else
	  EXPECT_FALSE (cma->isInscribed(ix, iy, true))
	    << "[" << ix << "][" << iy << "] should not be inscribed";
	if (cost >= rcm.circumscribed_cost)
	  EXPECT_TRUE (cma->isPossiblyCircumscribed(ix, iy, false))
	    << "[" << ix << "][" << iy << "] should be circumscribed";
	else
	  EXPECT_FALSE (cma->isPossiblyCircumscribed(ix, iy, true))
	    << "[" << ix << "][" << iy << "] should not be circumscribed";
      }
    }
  }
  delete rcm.raw;
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
