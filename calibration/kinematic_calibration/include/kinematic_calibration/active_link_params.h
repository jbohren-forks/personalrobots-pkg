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

//! \author Vijay Pradeep

#ifndef KINEMATIC_CALIBRATION_ACTIVE_LINK_PARAMS_H_
#define KINEMATIC_CALIBRATION_ACTIVE_LINK_PARAMS_H_

#include <vector>

#include "kdl/frames.hpp"

namespace kinematic_calibration
{

/**
 * A "smart" matrix that allows easy lookup for which link parameters in a
 * chain are active during a calibration
 */
class ActiveLinkParams
{
public:
  ActiveLinkParams() { }
  ~ActiveLinkParams() { }

  /**
   * Sets the number of links that the lookup table corresponds to. Data is not
   * necessarily preserved during a resize
   */
  inline void setNumLinks(unsigned int num_links)
  {
    table.resize(num_links) ;
  }

  inline void setAllInactive()
  {
    for (unsigned int i=0; i<table.size(); i++)
      for (unsigned int j=0; j<6; j++)
        table[i].active[j] = false ;
  }
  
  //! Gets the number of links in the current table
  inline unsigned int getNumLinks() const
  {
    return table.size() ;
  }
  
  //! Get the number of active params in the current table
  unsigned int getNumActive() const
  {
    unsigned int num_active = 0 ;
    for (unsigned int i=0; i <table.size(); i++)
      for (int j=0; j<6; j++)
        if (table[i].active[j])
          num_active++ ;
    return num_active ;
  }
  
  inline bool operator()(unsigned int param_num, unsigned int link_num) const
  {
    return table[link_num].active[param_num] ;
  }
  
  inline bool& operator()(unsigned int param_num, unsigned int link_num)
  {
    return table[link_num].active[param_num] ;
  }

private:
  //! Stores the 6 flags for a given link. (3 translational params, then 3 rotational params)
  struct SingleLinkActiveParams
  {
    bool active[6] ;
  };

  // Stores the full table of param flags
  std::vector<SingleLinkActiveParams>  table ;
};

}

#endif /* KINEMATIC_CALIBRATION_ACTIVE_PARAMS_MAT_H_ */
