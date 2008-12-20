/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Conor McGann
 */

#ifndef COSTMAP_2D_OBSERVATION_BUFFER_H
#define COSTMAP_2D_OBSERVATION_BUFFER_H

#include <costmap_2d/observation.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <vector>
#include <list>
#include <deque>

namespace costmap_2d {

  /**
   * @brief Base class for buffering observations with a time to live property
   * The inputs are in the map frame
   */
  class ObservationBuffer {
  public:
    ObservationBuffer(const std::string& frame_id, ros::Duration keep_alive, ros::Duration refresh_interval);

    virtual ~ObservationBuffer();

    /**
     * @brief Buffer a current observation
     * @return true if succeded, false if not (which might occur if there was no transform available for example)
     */
    bool buffer_observation(const Observation& observation);

    /**
     * @brief Queries for current observations. Any observations that are no longer
     * current will be deleted. Will append observations to the input vector
     */
    virtual void get_observations(std::vector<Observation>& observations);

    /**
     * @brief Checks if the buffered observations are up to date.
     *
     * In order to avoid working with stale data, we wish to check if the buffer has been updated
     * within its refresh interval
     */
    bool isCurrent() const;


    /**
     * @brief Translates a rate to an interval in ros duration
     */
    static ros::Duration computeRefreshInterval(double rate);

  protected:
    const std::string frame_id_;

  private:
    std::list<Observation> buffer_;
    bool received_obseration_;
    const ros::Duration keep_alive_;
    const ros::Duration refresh_interval_;
    ros::Time last_updated_;
  };

}
#endif
