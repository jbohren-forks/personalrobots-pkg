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

#ifndef TOPOLOGICAL_MAP_DOOR_SENSOR_FUSION_H
#define TOPOLOGICAL_MAP_DOOR_SENSOR_FUSION_H

#include <vector>
#include <map>

namespace topological_map
{

enum Cue { POSITIVE, NEGATIVE, UNKNOWN };
typedef std::vector<Cue> CueVector;
typedef unsigned int uint;

class DoorSensorFusion
{
public:

  /// Constructor: create sensor fusion object with \a num_cues cues
  DoorSensorFusion (uint num_cues) : num_cues_(num_cues), positive_examples_(1<<num_cues), negative_examples_(1<<num_cues) {}

  /// \post Incorporates this training example into the sensor fusion model
  /// \param door_exists was there really a door?
  /// \param cues vector of whether each cue was positive, negative, or unknown (e.g. because that sensor is not active).  Unknown not currently handled.
  /// \throws WrongNumberOfCues
  void observeTrainingExample (bool door_exists, const CueVector& cues);

  /// \return Posterior probability of door existing
  /// \param prior_prob Prior probability of a door
  /// \param cues Observed cue vector
  double posteriorProbOfDoor (double prior_prob, const CueVector& cues) const;




private:

  typedef std::vector<uint> CueCounts;
  struct Counts
  {
    void printCounts ();
    Counts(uint size) : counts(size,1), total(size) {}

    CueCounts counts;
    uint total;
  };

  void observeTrainingExample (Counts* counts, uint ind);
  double likelihood (const Counts& examples, uint ind) const;

  uint cueVectorIndex(const CueVector& cues) const;

  uint num_cues_;
  Counts positive_examples_, negative_examples_;
};










} // namespace topological_map

#endif // TOPOLOGICAL_MAP_DOOR_INFO_H
