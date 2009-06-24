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
 */


#include <topological_map/door_sensor_fusion.h>
#include <cmath>
#include <iostream>
#include <ros/console.h>
#include <ros/assert.h>

using std::cout;
using std::endl;

namespace topological_map
{


void DoorSensorFusion::observeTrainingExample (bool door_exists, const CueVector& cues)
{
  const uint ind = cueVectorIndex(cues);
  if (door_exists) {
    observeTrainingExample(&positive_examples_, ind);
  }
  else {
    observeTrainingExample(&negative_examples_, ind);
  }
}



double DoorSensorFusion::posteriorProbOfDoor (double prior_prob, const CueVector& cues) const
{
  const uint ind = cueVectorIndex(cues);
  const double prob_given_true = likelihood(positive_examples_, ind);
  const double prob_given_false = likelihood(negative_examples_, ind);
  const double denominator = prior_prob*prob_given_true + (1-prior_prob)*prob_given_false;
  ROS_DEBUG_STREAM_NAMED ("door_sensor_fusion", "Prob given true: " << prob_given_true << "; Prob given false: " << prob_given_false);

  if (fabs(denominator)<1e-6) {
    ROS_WARN ("Numerical instability when estimating posterior probability of door.  Using prior.");
    return prior_prob;
  }
  
  return prior_prob*prob_given_true/denominator;
}


double DoorSensorFusion::likelihood (const Counts& examples, const uint ind) const
{
  return examples.total ? (double)examples.counts[ind]/examples.total : 0.0;
}



uint DoorSensorFusion::cueVectorIndex (const CueVector& cues) const
{
  uint total=0, mult=1;
  for (uint i=0; i<num_cues_; i++) {
    ROS_ASSERT_MSG(cues[i] != UNKNOWN, "UNKNOWN not currently supported");
    total += mult * (cues[i]==POSITIVE ? 1 : 0);
    mult <<= 1;
  }
  return total;
}


void DoorSensorFusion::Counts::printCounts ()
{
  for (CueCounts::const_iterator iter=counts.begin(); iter!=counts.end(); ++iter) {
    cout << *iter << " ";
  }
  cout << "Total : " << total << endl;
  
}


void DoorSensorFusion::observeTrainingExample (Counts* counts, const uint ind)
{
  counts->counts[ind]++;
  counts->total++;
}


  
}












namespace tmap=topological_map;


int main (int, char**)
{
  
  tmap::DoorSensorFusion doors(2);
  tmap::CueVector c(2);
  c[0] = tmap::POSITIVE;
  c[1] = tmap::NEGATIVE;
  cout << doors.posteriorProbOfDoor (0.5, c) << " " << doors.posteriorProbOfDoor (0.0, c) << " " << doors.posteriorProbOfDoor (1.0, c) << endl;
  doors.observeTrainingExample(true, c);
  cout << doors.posteriorProbOfDoor (0.5, c) << " " << doors.posteriorProbOfDoor (0.0, c) << " " << doors.posteriorProbOfDoor (1.0, c) << endl;
  c[0] = tmap::POSITIVE;
  c[1] = tmap::POSITIVE;
  doors.observeTrainingExample(true, c);
  cout << doors.posteriorProbOfDoor (0.5, c) << " " << doors.posteriorProbOfDoor (0.0, c) << " " << doors.posteriorProbOfDoor (1.0, c) << endl;
  c[0] = tmap::NEGATIVE;
  cout << doors.posteriorProbOfDoor (0.5, c) << endl;
  c[0] = tmap::POSITIVE;
  c[1] = tmap::NEGATIVE;
  cout << doors.posteriorProbOfDoor (0.5, c) << endl;
}
