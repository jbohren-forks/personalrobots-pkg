/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
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
 * $Id$
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _SAMPLE_CONSENSUS_SACMODELLINE_H_
#define _SAMPLE_CONSENSUS_SACMODELLINE_H_

#include "sample_consensus/sac_model.h"
#include "sample_consensus/model_types.h"

namespace sample_consensus
{
  /** \brief A Sample Consensus Model class for 3D plane segmentation.
    */
  class SACModelLine : public SACModel
  {
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for base SACModelPlane. */
      SACModelLine () { }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for base SACModelPlane. */
      virtual ~SACModelLine () { }

      virtual std::vector<int> getSamples (int &iterations);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Test whether the given model coefficients are valid given the input point cloud data.
        * \param model_coefficients the model coefficients that need to be tested
        * \todo implement this
        */
      bool testModelCoefficients (std::vector<double> model_coefficients) { return true; }

      virtual bool computeModelCoefficients (std::vector<int> indices);

      virtual std::vector<double> refitModel (std::vector<int> inliers);
      virtual std::vector<double> getDistancesToModel (std::vector<double> model_coefficients);
      virtual std::vector<int>    selectWithinDistance (std::vector<double> model_coefficients, double threshold);

      virtual std_msgs::PointCloud projectPoints (std::vector<int> inliers, std::vector<double> model_coefficients);

      virtual void projectPointsInPlace (std::vector<int> inliers, std::vector<double> model_coefficients);
      virtual bool doSamplesVerifyModel (std::set<int> indices, double threshold);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return an unique id for this model (SACMODEL_LINE). */
      virtual int getModelType () { return (SACMODEL_LINE); }
  };
}

#endif
