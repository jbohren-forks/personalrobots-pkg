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

#include <gtest/gtest.h>
#include "std_msgs/PointCloud.h"

#include "sample_consensus/sac.h"
#include "sample_consensus/lmeds.h"
#include "sample_consensus/ransac.h"
#include "sample_consensus/rransac.h"
#include "sample_consensus/msac.h"
#include "sample_consensus/rmsac.h"
#include "sample_consensus/mlesac.h"
#include "sample_consensus/sac_model.h"
#include "sample_consensus/sac_model_cylinder.h"

using namespace sample_consensus;

TEST (LMedS, SACModelCylinder)
{
  std_msgs::PointCloud points;
  points.pts.resize (20);

  points.set_chan_size (3);
  points.chan[0].name = "nx";
  points.chan[1].name = "ny";
  points.chan[2].name = "nz";
  points.chan[0].vals.resize (points.pts.size ());
  points.chan[1].vals.resize (points.pts.size ());
  points.chan[2].vals.resize (points.pts.size ());

  points.pts[0].x = -0.499902; points.pts[0].y = 2.199701; points.pts[0].z = 0.000008;
  points.pts[1].x = -0.875397; points.pts[1].y = 2.030177; points.pts[1].z = 0.050104;
  points.pts[2].x = -0.995875; points.pts[2].y = 1.635973; points.pts[2].z = 0.099846;
  points.pts[3].x = -0.779523; points.pts[3].y = 1.285527; points.pts[3].z = 0.149961;
  points.pts[4].x = -0.373285; points.pts[4].y = 1.216488; points.pts[4].z = 0.199959;
  points.pts[5].x = -0.052893; points.pts[5].y = 1.475973; points.pts[5].z = 0.250101;
  points.pts[6].x = -0.036558; points.pts[6].y = 1.887591; points.pts[6].z = 0.299839;
  points.pts[7].x = -0.335048; points.pts[7].y = 2.171994; points.pts[7].z = 0.350001;
  points.pts[8].x = -0.745456; points.pts[8].y = 2.135528; points.pts[8].z = 0.400072;
  points.pts[9].x = -0.989282; points.pts[9].y = 1.803311; points.pts[9].z = 0.449983;
  points.pts[10].x = -0.900651; points.pts[10].y = 1.400701; points.pts[10].z = 0.500126;
  points.pts[11].x = -0.539658; points.pts[11].y = 1.201468; points.pts[11].z = 0.550079;
  points.pts[12].x = -0.151875; points.pts[12].y = 1.340951; points.pts[12].z = 0.599983;
  points.pts[13].x = -0.000724; points.pts[13].y = 1.724373; points.pts[13].z = 0.649882;
  points.pts[14].x = -0.188573; points.pts[14].y = 2.090983; points.pts[14].z = 0.699854;
  points.pts[15].x = -0.587925; points.pts[15].y = 2.192257; points.pts[15].z = 0.749956;
  points.pts[16].x = -0.927724; points.pts[16].y = 1.958846; points.pts[16].z = 0.800008;
  points.pts[17].x = -0.976888; points.pts[17].y = 1.549655; points.pts[17].z = 0.849970;
  points.pts[18].x = -0.702003; points.pts[18].y = 1.242707; points.pts[18].z = 0.899954;
  points.pts[19].x = -0.289916; points.pts[19].y = 1.246296; points.pts[19].z = 0.950075;

  points.chan[0].vals[0] = 0.000098;  points.chan[1].vals[0] = 1.000098;  points.chan[2].vals[0] = 0.000008;
  points.chan[0].vals[1] = -0.750891; points.chan[1].vals[1] = 0.660413;  points.chan[2].vals[1] = 0.000104;
  points.chan[0].vals[2] = -0.991765; points.chan[1].vals[2] = -0.127949; points.chan[2].vals[2] = -0.000154;
  points.chan[0].vals[3] = -0.558918; points.chan[1].vals[3] = -0.829439; points.chan[2].vals[3] = -0.000039;
  points.chan[0].vals[4] = 0.253627;  points.chan[1].vals[4] = -0.967447; points.chan[2].vals[4] = -0.000041;
  points.chan[0].vals[5] = 0.894105;  points.chan[1].vals[5] = -0.447965; points.chan[2].vals[5] = 0.000101;
  points.chan[0].vals[6] = 0.926852;  points.chan[1].vals[6] = 0.375543;  points.chan[2].vals[6] = -0.000161;
  points.chan[0].vals[7] = 0.329948;  points.chan[1].vals[7] = 0.943941;  points.chan[2].vals[7] = 0.000001;
  points.chan[0].vals[8] = -0.490966; points.chan[1].vals[8] = 0.871203;  points.chan[2].vals[8] = 0.000072;
  points.chan[0].vals[9] = -0.978507; points.chan[1].vals[9] = 0.206425;  points.chan[2].vals[9] = -0.000017;
  points.chan[0].vals[10] = -0.801227; points.chan[1].vals[10] = -0.598534; points.chan[2].vals[10] = 0.000126;
  points.chan[0].vals[11] = -0.079447; points.chan[1].vals[11] = -0.996697; points.chan[2].vals[11] = 0.000079;
  points.chan[0].vals[12] = 0.696154;  points.chan[1].vals[12] = -0.717889; points.chan[2].vals[12] = -0.000017;
  points.chan[0].vals[13] = 0.998685;  points.chan[1].vals[13] = 0.048502;  points.chan[2].vals[13] = -0.000118;
  points.chan[0].vals[14] = 0.622933;  points.chan[1].vals[14] = 0.782133;  points.chan[2].vals[14] = -0.000146;
  points.chan[0].vals[15] = -0.175948; points.chan[1].vals[15] = 0.984480;  points.chan[2].vals[15] = -0.000044;
  points.chan[0].vals[16] = -0.855476; points.chan[1].vals[16] = 0.517824;  points.chan[2].vals[16] = 0.000008;
  points.chan[0].vals[17] = -0.953769; points.chan[1].vals[17] = -0.300571; points.chan[2].vals[17] = -0.000030;
  points.chan[0].vals[18] = -0.404035; points.chan[1].vals[18] = -0.914700; points.chan[2].vals[18] = -0.000046;
  points.chan[0].vals[19] = 0.420154;  points.chan[1].vals[19] = -0.907445; points.chan[2].vals[19] = 0.000075;

  SACModel *model = new SACModelCylinder ();
  SAC *sac        = new LMedS (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->pts.size (), 20);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 20);

  std::vector<double> coeff = sac->computeCoefficients ();
  EXPECT_EQ ((int)coeff.size (), 7);
  //printf ("Cylinder coefficients: %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);
  EXPECT_NEAR (coeff[0], -0.5, 1e-1);
  EXPECT_NEAR (coeff[1], 1.7, 1e-1);
  EXPECT_NEAR (coeff[6], 0.5, 1e-1);

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 0);
}

TEST (RANSAC, SACModelCylinder)
{
  std_msgs::PointCloud points;
  points.pts.resize (20);

  points.set_chan_size (3);
  points.chan[0].name = "nx";
  points.chan[1].name = "ny";
  points.chan[2].name = "nz";
  points.chan[0].vals.resize (points.pts.size ());
  points.chan[1].vals.resize (points.pts.size ());
  points.chan[2].vals.resize (points.pts.size ());

  points.pts[0].x = -0.499902; points.pts[0].y = 2.199701; points.pts[0].z = 0.000008;
  points.pts[1].x = -0.875397; points.pts[1].y = 2.030177; points.pts[1].z = 0.050104;
  points.pts[2].x = -0.995875; points.pts[2].y = 1.635973; points.pts[2].z = 0.099846;
  points.pts[3].x = -0.779523; points.pts[3].y = 1.285527; points.pts[3].z = 0.149961;
  points.pts[4].x = -0.373285; points.pts[4].y = 1.216488; points.pts[4].z = 0.199959;
  points.pts[5].x = -0.052893; points.pts[5].y = 1.475973; points.pts[5].z = 0.250101;
  points.pts[6].x = -0.036558; points.pts[6].y = 1.887591; points.pts[6].z = 0.299839;
  points.pts[7].x = -0.335048; points.pts[7].y = 2.171994; points.pts[7].z = 0.350001;
  points.pts[8].x = -0.745456; points.pts[8].y = 2.135528; points.pts[8].z = 0.400072;
  points.pts[9].x = -0.989282; points.pts[9].y = 1.803311; points.pts[9].z = 0.449983;
  points.pts[10].x = -0.900651; points.pts[10].y = 1.400701; points.pts[10].z = 0.500126;
  points.pts[11].x = -0.539658; points.pts[11].y = 1.201468; points.pts[11].z = 0.550079;
  points.pts[12].x = -0.151875; points.pts[12].y = 1.340951; points.pts[12].z = 0.599983;
  points.pts[13].x = -0.000724; points.pts[13].y = 1.724373; points.pts[13].z = 0.649882;
  points.pts[14].x = -0.188573; points.pts[14].y = 2.090983; points.pts[14].z = 0.699854;
  points.pts[15].x = -0.587925; points.pts[15].y = 2.192257; points.pts[15].z = 0.749956;
  points.pts[16].x = -0.927724; points.pts[16].y = 1.958846; points.pts[16].z = 0.800008;
  points.pts[17].x = -0.976888; points.pts[17].y = 1.549655; points.pts[17].z = 0.849970;
  points.pts[18].x = -0.702003; points.pts[18].y = 1.242707; points.pts[18].z = 0.899954;
  points.pts[19].x = -0.289916; points.pts[19].y = 1.246296; points.pts[19].z = 0.950075;

  points.chan[0].vals[0] = 0.000098;  points.chan[1].vals[0] = 1.000098;  points.chan[2].vals[0] = 0.000008;
  points.chan[0].vals[1] = -0.750891; points.chan[1].vals[1] = 0.660413;  points.chan[2].vals[1] = 0.000104;
  points.chan[0].vals[2] = -0.991765; points.chan[1].vals[2] = -0.127949; points.chan[2].vals[2] = -0.000154;
  points.chan[0].vals[3] = -0.558918; points.chan[1].vals[3] = -0.829439; points.chan[2].vals[3] = -0.000039;
  points.chan[0].vals[4] = 0.253627;  points.chan[1].vals[4] = -0.967447; points.chan[2].vals[4] = -0.000041;
  points.chan[0].vals[5] = 0.894105;  points.chan[1].vals[5] = -0.447965; points.chan[2].vals[5] = 0.000101;
  points.chan[0].vals[6] = 0.926852;  points.chan[1].vals[6] = 0.375543;  points.chan[2].vals[6] = -0.000161;
  points.chan[0].vals[7] = 0.329948;  points.chan[1].vals[7] = 0.943941;  points.chan[2].vals[7] = 0.000001;
  points.chan[0].vals[8] = -0.490966; points.chan[1].vals[8] = 0.871203;  points.chan[2].vals[8] = 0.000072;
  points.chan[0].vals[9] = -0.978507; points.chan[1].vals[9] = 0.206425;  points.chan[2].vals[9] = -0.000017;
  points.chan[0].vals[10] = -0.801227; points.chan[1].vals[10] = -0.598534; points.chan[2].vals[10] = 0.000126;
  points.chan[0].vals[11] = -0.079447; points.chan[1].vals[11] = -0.996697; points.chan[2].vals[11] = 0.000079;
  points.chan[0].vals[12] = 0.696154;  points.chan[1].vals[12] = -0.717889; points.chan[2].vals[12] = -0.000017;
  points.chan[0].vals[13] = 0.998685;  points.chan[1].vals[13] = 0.048502;  points.chan[2].vals[13] = -0.000118;
  points.chan[0].vals[14] = 0.622933;  points.chan[1].vals[14] = 0.782133;  points.chan[2].vals[14] = -0.000146;
  points.chan[0].vals[15] = -0.175948; points.chan[1].vals[15] = 0.984480;  points.chan[2].vals[15] = -0.000044;
  points.chan[0].vals[16] = -0.855476; points.chan[1].vals[16] = 0.517824;  points.chan[2].vals[16] = 0.000008;
  points.chan[0].vals[17] = -0.953769; points.chan[1].vals[17] = -0.300571; points.chan[2].vals[17] = -0.000030;
  points.chan[0].vals[18] = -0.404035; points.chan[1].vals[18] = -0.914700; points.chan[2].vals[18] = -0.000046;
  points.chan[0].vals[19] = 0.420154;  points.chan[1].vals[19] = -0.907445; points.chan[2].vals[19] = 0.000075;

  SACModel *model = new SACModelCylinder ();
  SAC *sac        = new RANSAC (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->pts.size (), 20);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 20);

  std::vector<double> coeff = sac->computeCoefficients ();
  EXPECT_EQ ((int)coeff.size (), 7);
  //printf ("Cylinder coefficients: %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);
  EXPECT_NEAR (coeff[0], -0.5, 1e-1);
  EXPECT_NEAR (coeff[1], 1.7, 1e-1);
  EXPECT_NEAR (coeff[6], 0.5, 1e-1);

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 0);
}

TEST (MSAC, SACModelCylinder)
{
  std_msgs::PointCloud points;
  points.pts.resize (20);

  points.set_chan_size (3);
  points.chan[0].name = "nx";
  points.chan[1].name = "ny";
  points.chan[2].name = "nz";
  points.chan[0].vals.resize (points.pts.size ());
  points.chan[1].vals.resize (points.pts.size ());
  points.chan[2].vals.resize (points.pts.size ());

  points.pts[0].x = -0.499902; points.pts[0].y = 2.199701; points.pts[0].z = 0.000008;
  points.pts[1].x = -0.875397; points.pts[1].y = 2.030177; points.pts[1].z = 0.050104;
  points.pts[2].x = -0.995875; points.pts[2].y = 1.635973; points.pts[2].z = 0.099846;
  points.pts[3].x = -0.779523; points.pts[3].y = 1.285527; points.pts[3].z = 0.149961;
  points.pts[4].x = -0.373285; points.pts[4].y = 1.216488; points.pts[4].z = 0.199959;
  points.pts[5].x = -0.052893; points.pts[5].y = 1.475973; points.pts[5].z = 0.250101;
  points.pts[6].x = -0.036558; points.pts[6].y = 1.887591; points.pts[6].z = 0.299839;
  points.pts[7].x = -0.335048; points.pts[7].y = 2.171994; points.pts[7].z = 0.350001;
  points.pts[8].x = -0.745456; points.pts[8].y = 2.135528; points.pts[8].z = 0.400072;
  points.pts[9].x = -0.989282; points.pts[9].y = 1.803311; points.pts[9].z = 0.449983;
  points.pts[10].x = -0.900651; points.pts[10].y = 1.400701; points.pts[10].z = 0.500126;
  points.pts[11].x = -0.539658; points.pts[11].y = 1.201468; points.pts[11].z = 0.550079;
  points.pts[12].x = -0.151875; points.pts[12].y = 1.340951; points.pts[12].z = 0.599983;
  points.pts[13].x = -0.000724; points.pts[13].y = 1.724373; points.pts[13].z = 0.649882;
  points.pts[14].x = -0.188573; points.pts[14].y = 2.090983; points.pts[14].z = 0.699854;
  points.pts[15].x = -0.587925; points.pts[15].y = 2.192257; points.pts[15].z = 0.749956;
  points.pts[16].x = -0.927724; points.pts[16].y = 1.958846; points.pts[16].z = 0.800008;
  points.pts[17].x = -0.976888; points.pts[17].y = 1.549655; points.pts[17].z = 0.849970;
  points.pts[18].x = -0.702003; points.pts[18].y = 1.242707; points.pts[18].z = 0.899954;
  points.pts[19].x = -0.289916; points.pts[19].y = 1.246296; points.pts[19].z = 0.950075;

  points.chan[0].vals[0] = 0.000098;  points.chan[1].vals[0] = 1.000098;  points.chan[2].vals[0] = 0.000008;
  points.chan[0].vals[1] = -0.750891; points.chan[1].vals[1] = 0.660413;  points.chan[2].vals[1] = 0.000104;
  points.chan[0].vals[2] = -0.991765; points.chan[1].vals[2] = -0.127949; points.chan[2].vals[2] = -0.000154;
  points.chan[0].vals[3] = -0.558918; points.chan[1].vals[3] = -0.829439; points.chan[2].vals[3] = -0.000039;
  points.chan[0].vals[4] = 0.253627;  points.chan[1].vals[4] = -0.967447; points.chan[2].vals[4] = -0.000041;
  points.chan[0].vals[5] = 0.894105;  points.chan[1].vals[5] = -0.447965; points.chan[2].vals[5] = 0.000101;
  points.chan[0].vals[6] = 0.926852;  points.chan[1].vals[6] = 0.375543;  points.chan[2].vals[6] = -0.000161;
  points.chan[0].vals[7] = 0.329948;  points.chan[1].vals[7] = 0.943941;  points.chan[2].vals[7] = 0.000001;
  points.chan[0].vals[8] = -0.490966; points.chan[1].vals[8] = 0.871203;  points.chan[2].vals[8] = 0.000072;
  points.chan[0].vals[9] = -0.978507; points.chan[1].vals[9] = 0.206425;  points.chan[2].vals[9] = -0.000017;
  points.chan[0].vals[10] = -0.801227; points.chan[1].vals[10] = -0.598534; points.chan[2].vals[10] = 0.000126;
  points.chan[0].vals[11] = -0.079447; points.chan[1].vals[11] = -0.996697; points.chan[2].vals[11] = 0.000079;
  points.chan[0].vals[12] = 0.696154;  points.chan[1].vals[12] = -0.717889; points.chan[2].vals[12] = -0.000017;
  points.chan[0].vals[13] = 0.998685;  points.chan[1].vals[13] = 0.048502;  points.chan[2].vals[13] = -0.000118;
  points.chan[0].vals[14] = 0.622933;  points.chan[1].vals[14] = 0.782133;  points.chan[2].vals[14] = -0.000146;
  points.chan[0].vals[15] = -0.175948; points.chan[1].vals[15] = 0.984480;  points.chan[2].vals[15] = -0.000044;
  points.chan[0].vals[16] = -0.855476; points.chan[1].vals[16] = 0.517824;  points.chan[2].vals[16] = 0.000008;
  points.chan[0].vals[17] = -0.953769; points.chan[1].vals[17] = -0.300571; points.chan[2].vals[17] = -0.000030;
  points.chan[0].vals[18] = -0.404035; points.chan[1].vals[18] = -0.914700; points.chan[2].vals[18] = -0.000046;
  points.chan[0].vals[19] = 0.420154;  points.chan[1].vals[19] = -0.907445; points.chan[2].vals[19] = 0.000075;

  SACModel *model = new SACModelCylinder ();
  SAC *sac        = new MSAC (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->pts.size (), 20);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 20);

  std::vector<double> coeff = sac->computeCoefficients ();
  EXPECT_EQ ((int)coeff.size (), 7);
  //printf ("Cylinder coefficients: %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);
  EXPECT_NEAR (coeff[0], -0.5, 1e-1);
  EXPECT_NEAR (coeff[1], 1.7, 1e-1);
  EXPECT_NEAR (coeff[6], 0.5, 1e-1);

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 0);
}

TEST (MLESAC, SACModelCylinder)
{
  std_msgs::PointCloud points;
  points.pts.resize (20);

  points.set_chan_size (3);
  points.chan[0].name = "nx";
  points.chan[1].name = "ny";
  points.chan[2].name = "nz";
  points.chan[0].vals.resize (points.pts.size ());
  points.chan[1].vals.resize (points.pts.size ());
  points.chan[2].vals.resize (points.pts.size ());

  points.pts[0].x = -0.499902; points.pts[0].y = 2.199701; points.pts[0].z = 0.000008;
  points.pts[1].x = -0.875397; points.pts[1].y = 2.030177; points.pts[1].z = 0.050104;
  points.pts[2].x = -0.995875; points.pts[2].y = 1.635973; points.pts[2].z = 0.099846;
  points.pts[3].x = -0.779523; points.pts[3].y = 1.285527; points.pts[3].z = 0.149961;
  points.pts[4].x = -0.373285; points.pts[4].y = 1.216488; points.pts[4].z = 0.199959;
  points.pts[5].x = -0.052893; points.pts[5].y = 1.475973; points.pts[5].z = 0.250101;
  points.pts[6].x = -0.036558; points.pts[6].y = 1.887591; points.pts[6].z = 0.299839;
  points.pts[7].x = -0.335048; points.pts[7].y = 2.171994; points.pts[7].z = 0.350001;
  points.pts[8].x = -0.745456; points.pts[8].y = 2.135528; points.pts[8].z = 0.400072;
  points.pts[9].x = -0.989282; points.pts[9].y = 1.803311; points.pts[9].z = 0.449983;
  points.pts[10].x = -0.900651; points.pts[10].y = 1.400701; points.pts[10].z = 0.500126;
  points.pts[11].x = -0.539658; points.pts[11].y = 1.201468; points.pts[11].z = 0.550079;
  points.pts[12].x = -0.151875; points.pts[12].y = 1.340951; points.pts[12].z = 0.599983;
  points.pts[13].x = -0.000724; points.pts[13].y = 1.724373; points.pts[13].z = 0.649882;
  points.pts[14].x = -0.188573; points.pts[14].y = 2.090983; points.pts[14].z = 0.699854;
  points.pts[15].x = -0.587925; points.pts[15].y = 2.192257; points.pts[15].z = 0.749956;
  points.pts[16].x = -0.927724; points.pts[16].y = 1.958846; points.pts[16].z = 0.800008;
  points.pts[17].x = -0.976888; points.pts[17].y = 1.549655; points.pts[17].z = 0.849970;
  points.pts[18].x = -0.702003; points.pts[18].y = 1.242707; points.pts[18].z = 0.899954;
  points.pts[19].x = -0.289916; points.pts[19].y = 1.246296; points.pts[19].z = 0.950075;

  points.chan[0].vals[0] = 0.000098;  points.chan[1].vals[0] = 1.000098;  points.chan[2].vals[0] = 0.000008;
  points.chan[0].vals[1] = -0.750891; points.chan[1].vals[1] = 0.660413;  points.chan[2].vals[1] = 0.000104;
  points.chan[0].vals[2] = -0.991765; points.chan[1].vals[2] = -0.127949; points.chan[2].vals[2] = -0.000154;
  points.chan[0].vals[3] = -0.558918; points.chan[1].vals[3] = -0.829439; points.chan[2].vals[3] = -0.000039;
  points.chan[0].vals[4] = 0.253627;  points.chan[1].vals[4] = -0.967447; points.chan[2].vals[4] = -0.000041;
  points.chan[0].vals[5] = 0.894105;  points.chan[1].vals[5] = -0.447965; points.chan[2].vals[5] = 0.000101;
  points.chan[0].vals[6] = 0.926852;  points.chan[1].vals[6] = 0.375543;  points.chan[2].vals[6] = -0.000161;
  points.chan[0].vals[7] = 0.329948;  points.chan[1].vals[7] = 0.943941;  points.chan[2].vals[7] = 0.000001;
  points.chan[0].vals[8] = -0.490966; points.chan[1].vals[8] = 0.871203;  points.chan[2].vals[8] = 0.000072;
  points.chan[0].vals[9] = -0.978507; points.chan[1].vals[9] = 0.206425;  points.chan[2].vals[9] = -0.000017;
  points.chan[0].vals[10] = -0.801227; points.chan[1].vals[10] = -0.598534; points.chan[2].vals[10] = 0.000126;
  points.chan[0].vals[11] = -0.079447; points.chan[1].vals[11] = -0.996697; points.chan[2].vals[11] = 0.000079;
  points.chan[0].vals[12] = 0.696154;  points.chan[1].vals[12] = -0.717889; points.chan[2].vals[12] = -0.000017;
  points.chan[0].vals[13] = 0.998685;  points.chan[1].vals[13] = 0.048502;  points.chan[2].vals[13] = -0.000118;
  points.chan[0].vals[14] = 0.622933;  points.chan[1].vals[14] = 0.782133;  points.chan[2].vals[14] = -0.000146;
  points.chan[0].vals[15] = -0.175948; points.chan[1].vals[15] = 0.984480;  points.chan[2].vals[15] = -0.000044;
  points.chan[0].vals[16] = -0.855476; points.chan[1].vals[16] = 0.517824;  points.chan[2].vals[16] = 0.000008;
  points.chan[0].vals[17] = -0.953769; points.chan[1].vals[17] = -0.300571; points.chan[2].vals[17] = -0.000030;
  points.chan[0].vals[18] = -0.404035; points.chan[1].vals[18] = -0.914700; points.chan[2].vals[18] = -0.000046;
  points.chan[0].vals[19] = 0.420154;  points.chan[1].vals[19] = -0.907445; points.chan[2].vals[19] = 0.000075;

  SACModel *model = new SACModelCylinder ();
  SAC *sac        = new MLESAC (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->pts.size (), 20);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 20);

  std::vector<double> coeff = sac->computeCoefficients ();
  EXPECT_EQ ((int)coeff.size (), 7);
  //printf ("Cylinder coefficients: %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);
  EXPECT_NEAR (coeff[0], -0.5, 1e-1);
  EXPECT_NEAR (coeff[1], 1.7, 1e-1);
  EXPECT_NEAR (coeff[6], 0.5, 1e-1);

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 0);
}

TEST (RRANSAC, SACModelCylinder)
{
  std_msgs::PointCloud points;
  points.pts.resize (20);

  points.set_chan_size (3);
  points.chan[0].name = "nx";
  points.chan[1].name = "ny";
  points.chan[2].name = "nz";
  points.chan[0].vals.resize (points.pts.size ());
  points.chan[1].vals.resize (points.pts.size ());
  points.chan[2].vals.resize (points.pts.size ());

  points.pts[0].x = -0.499902; points.pts[0].y = 2.199701; points.pts[0].z = 0.000008;
  points.pts[1].x = -0.875397; points.pts[1].y = 2.030177; points.pts[1].z = 0.050104;
  points.pts[2].x = -0.995875; points.pts[2].y = 1.635973; points.pts[2].z = 0.099846;
  points.pts[3].x = -0.779523; points.pts[3].y = 1.285527; points.pts[3].z = 0.149961;
  points.pts[4].x = -0.373285; points.pts[4].y = 1.216488; points.pts[4].z = 0.199959;
  points.pts[5].x = -0.052893; points.pts[5].y = 1.475973; points.pts[5].z = 0.250101;
  points.pts[6].x = -0.036558; points.pts[6].y = 1.887591; points.pts[6].z = 0.299839;
  points.pts[7].x = -0.335048; points.pts[7].y = 2.171994; points.pts[7].z = 0.350001;
  points.pts[8].x = -0.745456; points.pts[8].y = 2.135528; points.pts[8].z = 0.400072;
  points.pts[9].x = -0.989282; points.pts[9].y = 1.803311; points.pts[9].z = 0.449983;
  points.pts[10].x = -0.900651; points.pts[10].y = 1.400701; points.pts[10].z = 0.500126;
  points.pts[11].x = -0.539658; points.pts[11].y = 1.201468; points.pts[11].z = 0.550079;
  points.pts[12].x = -0.151875; points.pts[12].y = 1.340951; points.pts[12].z = 0.599983;
  points.pts[13].x = -0.000724; points.pts[13].y = 1.724373; points.pts[13].z = 0.649882;
  points.pts[14].x = -0.188573; points.pts[14].y = 2.090983; points.pts[14].z = 0.699854;
  points.pts[15].x = -0.587925; points.pts[15].y = 2.192257; points.pts[15].z = 0.749956;
  points.pts[16].x = -0.927724; points.pts[16].y = 1.958846; points.pts[16].z = 0.800008;
  points.pts[17].x = -0.976888; points.pts[17].y = 1.549655; points.pts[17].z = 0.849970;
  points.pts[18].x = -0.702003; points.pts[18].y = 1.242707; points.pts[18].z = 0.899954;
  points.pts[19].x = -0.289916; points.pts[19].y = 1.246296; points.pts[19].z = 0.950075;

  points.chan[0].vals[0] = 0.000098;  points.chan[1].vals[0] = 1.000098;  points.chan[2].vals[0] = 0.000008;
  points.chan[0].vals[1] = -0.750891; points.chan[1].vals[1] = 0.660413;  points.chan[2].vals[1] = 0.000104;
  points.chan[0].vals[2] = -0.991765; points.chan[1].vals[2] = -0.127949; points.chan[2].vals[2] = -0.000154;
  points.chan[0].vals[3] = -0.558918; points.chan[1].vals[3] = -0.829439; points.chan[2].vals[3] = -0.000039;
  points.chan[0].vals[4] = 0.253627;  points.chan[1].vals[4] = -0.967447; points.chan[2].vals[4] = -0.000041;
  points.chan[0].vals[5] = 0.894105;  points.chan[1].vals[5] = -0.447965; points.chan[2].vals[5] = 0.000101;
  points.chan[0].vals[6] = 0.926852;  points.chan[1].vals[6] = 0.375543;  points.chan[2].vals[6] = -0.000161;
  points.chan[0].vals[7] = 0.329948;  points.chan[1].vals[7] = 0.943941;  points.chan[2].vals[7] = 0.000001;
  points.chan[0].vals[8] = -0.490966; points.chan[1].vals[8] = 0.871203;  points.chan[2].vals[8] = 0.000072;
  points.chan[0].vals[9] = -0.978507; points.chan[1].vals[9] = 0.206425;  points.chan[2].vals[9] = -0.000017;
  points.chan[0].vals[10] = -0.801227; points.chan[1].vals[10] = -0.598534; points.chan[2].vals[10] = 0.000126;
  points.chan[0].vals[11] = -0.079447; points.chan[1].vals[11] = -0.996697; points.chan[2].vals[11] = 0.000079;
  points.chan[0].vals[12] = 0.696154;  points.chan[1].vals[12] = -0.717889; points.chan[2].vals[12] = -0.000017;
  points.chan[0].vals[13] = 0.998685;  points.chan[1].vals[13] = 0.048502;  points.chan[2].vals[13] = -0.000118;
  points.chan[0].vals[14] = 0.622933;  points.chan[1].vals[14] = 0.782133;  points.chan[2].vals[14] = -0.000146;
  points.chan[0].vals[15] = -0.175948; points.chan[1].vals[15] = 0.984480;  points.chan[2].vals[15] = -0.000044;
  points.chan[0].vals[16] = -0.855476; points.chan[1].vals[16] = 0.517824;  points.chan[2].vals[16] = 0.000008;
  points.chan[0].vals[17] = -0.953769; points.chan[1].vals[17] = -0.300571; points.chan[2].vals[17] = -0.000030;
  points.chan[0].vals[18] = -0.404035; points.chan[1].vals[18] = -0.914700; points.chan[2].vals[18] = -0.000046;
  points.chan[0].vals[19] = 0.420154;  points.chan[1].vals[19] = -0.907445; points.chan[2].vals[19] = 0.000075;

  SACModel *model = new SACModelCylinder ();
  SAC *sac        = new RRANSAC (model, 0.2);
  reinterpret_cast<RRANSAC*>(sac)->setFractionNrPretest (50);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->pts.size (), 20);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 20);

  std::vector<double> coeff = sac->computeCoefficients ();
  EXPECT_EQ ((int)coeff.size (), 7);
  //printf ("Cylinder coefficients: %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);
  EXPECT_NEAR (coeff[0], -0.5, 1e-1);
  EXPECT_NEAR (coeff[1], 1.7, 1e-1);
  EXPECT_NEAR (coeff[6], 0.5, 1e-1);

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 0);
}

TEST (RMSAC, SACModelCylinder)
{
  std_msgs::PointCloud points;
  points.pts.resize (20);

  points.set_chan_size (3);
  points.chan[0].name = "nx";
  points.chan[1].name = "ny";
  points.chan[2].name = "nz";
  points.chan[0].vals.resize (points.pts.size ());
  points.chan[1].vals.resize (points.pts.size ());
  points.chan[2].vals.resize (points.pts.size ());

  points.pts[0].x = -0.499902; points.pts[0].y = 2.199701; points.pts[0].z = 0.000008;
  points.pts[1].x = -0.875397; points.pts[1].y = 2.030177; points.pts[1].z = 0.050104;
  points.pts[2].x = -0.995875; points.pts[2].y = 1.635973; points.pts[2].z = 0.099846;
  points.pts[3].x = -0.779523; points.pts[3].y = 1.285527; points.pts[3].z = 0.149961;
  points.pts[4].x = -0.373285; points.pts[4].y = 1.216488; points.pts[4].z = 0.199959;
  points.pts[5].x = -0.052893; points.pts[5].y = 1.475973; points.pts[5].z = 0.250101;
  points.pts[6].x = -0.036558; points.pts[6].y = 1.887591; points.pts[6].z = 0.299839;
  points.pts[7].x = -0.335048; points.pts[7].y = 2.171994; points.pts[7].z = 0.350001;
  points.pts[8].x = -0.745456; points.pts[8].y = 2.135528; points.pts[8].z = 0.400072;
  points.pts[9].x = -0.989282; points.pts[9].y = 1.803311; points.pts[9].z = 0.449983;
  points.pts[10].x = -0.900651; points.pts[10].y = 1.400701; points.pts[10].z = 0.500126;
  points.pts[11].x = -0.539658; points.pts[11].y = 1.201468; points.pts[11].z = 0.550079;
  points.pts[12].x = -0.151875; points.pts[12].y = 1.340951; points.pts[12].z = 0.599983;
  points.pts[13].x = -0.000724; points.pts[13].y = 1.724373; points.pts[13].z = 0.649882;
  points.pts[14].x = -0.188573; points.pts[14].y = 2.090983; points.pts[14].z = 0.699854;
  points.pts[15].x = -0.587925; points.pts[15].y = 2.192257; points.pts[15].z = 0.749956;
  points.pts[16].x = -0.927724; points.pts[16].y = 1.958846; points.pts[16].z = 0.800008;
  points.pts[17].x = -0.976888; points.pts[17].y = 1.549655; points.pts[17].z = 0.849970;
  points.pts[18].x = -0.702003; points.pts[18].y = 1.242707; points.pts[18].z = 0.899954;
  points.pts[19].x = -0.289916; points.pts[19].y = 1.246296; points.pts[19].z = 0.950075;

  points.chan[0].vals[0] = 0.000098;  points.chan[1].vals[0] = 1.000098;  points.chan[2].vals[0] = 0.000008;
  points.chan[0].vals[1] = -0.750891; points.chan[1].vals[1] = 0.660413;  points.chan[2].vals[1] = 0.000104;
  points.chan[0].vals[2] = -0.991765; points.chan[1].vals[2] = -0.127949; points.chan[2].vals[2] = -0.000154;
  points.chan[0].vals[3] = -0.558918; points.chan[1].vals[3] = -0.829439; points.chan[2].vals[3] = -0.000039;
  points.chan[0].vals[4] = 0.253627;  points.chan[1].vals[4] = -0.967447; points.chan[2].vals[4] = -0.000041;
  points.chan[0].vals[5] = 0.894105;  points.chan[1].vals[5] = -0.447965; points.chan[2].vals[5] = 0.000101;
  points.chan[0].vals[6] = 0.926852;  points.chan[1].vals[6] = 0.375543;  points.chan[2].vals[6] = -0.000161;
  points.chan[0].vals[7] = 0.329948;  points.chan[1].vals[7] = 0.943941;  points.chan[2].vals[7] = 0.000001;
  points.chan[0].vals[8] = -0.490966; points.chan[1].vals[8] = 0.871203;  points.chan[2].vals[8] = 0.000072;
  points.chan[0].vals[9] = -0.978507; points.chan[1].vals[9] = 0.206425;  points.chan[2].vals[9] = -0.000017;
  points.chan[0].vals[10] = -0.801227; points.chan[1].vals[10] = -0.598534; points.chan[2].vals[10] = 0.000126;
  points.chan[0].vals[11] = -0.079447; points.chan[1].vals[11] = -0.996697; points.chan[2].vals[11] = 0.000079;
  points.chan[0].vals[12] = 0.696154;  points.chan[1].vals[12] = -0.717889; points.chan[2].vals[12] = -0.000017;
  points.chan[0].vals[13] = 0.998685;  points.chan[1].vals[13] = 0.048502;  points.chan[2].vals[13] = -0.000118;
  points.chan[0].vals[14] = 0.622933;  points.chan[1].vals[14] = 0.782133;  points.chan[2].vals[14] = -0.000146;
  points.chan[0].vals[15] = -0.175948; points.chan[1].vals[15] = 0.984480;  points.chan[2].vals[15] = -0.000044;
  points.chan[0].vals[16] = -0.855476; points.chan[1].vals[16] = 0.517824;  points.chan[2].vals[16] = 0.000008;
  points.chan[0].vals[17] = -0.953769; points.chan[1].vals[17] = -0.300571; points.chan[2].vals[17] = -0.000030;
  points.chan[0].vals[18] = -0.404035; points.chan[1].vals[18] = -0.914700; points.chan[2].vals[18] = -0.000046;
  points.chan[0].vals[19] = 0.420154;  points.chan[1].vals[19] = -0.907445; points.chan[2].vals[19] = 0.000075;

  SACModel *model = new SACModelCylinder ();
  SAC *sac        = new RMSAC (model, 0.2);
  reinterpret_cast<RMSAC*>(sac)->setFractionNrPretest (50);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->pts.size (), 20);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 20);

  std::vector<double> coeff = sac->computeCoefficients ();
  EXPECT_EQ ((int)coeff.size (), 7);
  //printf ("Cylinder coefficients: %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);
  EXPECT_NEAR (coeff[0], -0.5, 1e-1);
  EXPECT_NEAR (coeff[1], 1.7, 1e-1);
  EXPECT_NEAR (coeff[6], 0.5, 1e-1);

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 0);
}

/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
