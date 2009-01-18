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
#include "sample_consensus/sac_model_circle.h"

using namespace sample_consensus;

TEST (LMedS, SACModelCircle2D)
{
  std_msgs::PointCloud points;
  points.pts.resize (18);

  points.pts[0].x = 3.587751;  points.pts[0].y = -4.190982;  points.pts[0].z = 0;
  points.pts[1].x = 3.808883;  points.pts[1].y = -4.412265;  points.pts[1].z = 0;
  points.pts[2].x = 3.587525;  points.pts[2].y = -5.809143;  points.pts[2].z = 0;
  points.pts[3].x = 2.999913;  points.pts[3].y = -5.999980;  points.pts[3].z = 0;
  points.pts[4].x = 2.412224;  points.pts[4].y = -5.809090;  points.pts[4].z = 0;
  points.pts[5].x = 2.191080;  points.pts[5].y = -5.587682;  points.pts[5].z = 0;
  points.pts[6].x = 2.048941;  points.pts[6].y = -5.309003;  points.pts[6].z = 0;
  points.pts[7].x = 2.000397;  points.pts[7].y = -4.999944;  points.pts[7].z = 0;
  points.pts[8].x = 2.999953;  points.pts[8].y = -6.000056;  points.pts[8].z = 0;
  points.pts[9].x = 2.691127;  points.pts[9].y = -5.951136;  points.pts[9].z = 0;
  points.pts[10].x = 2.190892; points.pts[10].y = -5.587838; points.pts[10].z = 0;
  points.pts[11].x = 2.048874; points.pts[11].y = -5.309052; points.pts[11].z = 0;
  points.pts[12].x = 1.999990; points.pts[12].y = -5.000147; points.pts[12].z = 0;
  points.pts[13].x = 2.049026; points.pts[13].y = -4.690918; points.pts[13].z = 0;
  points.pts[14].x = 2.190956; points.pts[14].y = -4.412162; points.pts[14].z = 0;
  points.pts[15].x = 2.412231; points.pts[15].y = -4.190918; points.pts[15].z = 0;
  points.pts[16].x = 2.691027; points.pts[16].y = -4.049060; points.pts[16].z = 0;
  points.pts[17].x = 2;        points.pts[17].y = -3;        points.pts[17].z = 0;

  SACModel *model = new SACModelCircle2D ();
  SAC *sac        = new LMedS (model, 0.1);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->pts.size (), 18);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 17);

  std::vector<double> coeff = sac->computeCoefficients ();
  EXPECT_EQ ((int)coeff.size (), 3);
  //printf ("Circle 2D coefficients: %f %f %f\n", coeff[0], coeff[1], coeff[2]);
  EXPECT_NEAR (coeff[0],  2.99, 1e-1);
  EXPECT_NEAR (coeff[1], -5.00, 1e-1);
  EXPECT_NEAR (coeff[2],  0.99, 1e-1);

/*  std::vector<double> coeff_ref = sac->refineCoefficients ();
  EXPECT_EQ ((int)coeff_ref.size (), 3);
  //printf ("Circle 2D coefficients (refined): %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3]);
*/
  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (RANSAC, SACModelCircle2D)
{
  std_msgs::PointCloud points;
  points.pts.resize (18);

  points.pts[0].x = 3.587751;  points.pts[0].y = -4.190982;  points.pts[0].z = 0;
  points.pts[1].x = 3.808883;  points.pts[1].y = -4.412265;  points.pts[1].z = 0;
  points.pts[2].x = 3.587525;  points.pts[2].y = -5.809143;  points.pts[2].z = 0;
  points.pts[3].x = 2.999913;  points.pts[3].y = -5.999980;  points.pts[3].z = 0;
  points.pts[4].x = 2.412224;  points.pts[4].y = -5.809090;  points.pts[4].z = 0;
  points.pts[5].x = 2.191080;  points.pts[5].y = -5.587682;  points.pts[5].z = 0;
  points.pts[6].x = 2.048941;  points.pts[6].y = -5.309003;  points.pts[6].z = 0;
  points.pts[7].x = 2.000397;  points.pts[7].y = -4.999944;  points.pts[7].z = 0;
  points.pts[8].x = 2.999953;  points.pts[8].y = -6.000056;  points.pts[8].z = 0;
  points.pts[9].x = 2.691127;  points.pts[9].y = -5.951136;  points.pts[9].z = 0;
  points.pts[10].x = 2.190892; points.pts[10].y = -5.587838; points.pts[10].z = 0;
  points.pts[11].x = 2.048874; points.pts[11].y = -5.309052; points.pts[11].z = 0;
  points.pts[12].x = 1.999990; points.pts[12].y = -5.000147; points.pts[12].z = 0;
  points.pts[13].x = 2.049026; points.pts[13].y = -4.690918; points.pts[13].z = 0;
  points.pts[14].x = 2.190956; points.pts[14].y = -4.412162; points.pts[14].z = 0;
  points.pts[15].x = 2.412231; points.pts[15].y = -4.190918; points.pts[15].z = 0;
  points.pts[16].x = 2.691027; points.pts[16].y = -4.049060; points.pts[16].z = 0;
  points.pts[17].x = 2;        points.pts[17].y = -3;        points.pts[17].z = 0;

  SACModel *model = new SACModelCircle2D ();
  SAC *sac        = new RANSAC (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->pts.size (), 18);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 17);

  std::vector<double> coeff = sac->computeCoefficients ();
  EXPECT_EQ ((int)coeff.size (), 3);
  //printf ("Circle 2D coefficients: %f %f %f\n", coeff[0], coeff[1], coeff[2]);
  EXPECT_NEAR (coeff[0],  2.99, 1e-1);
  EXPECT_NEAR (coeff[1], -5.00, 1e-1);
  EXPECT_NEAR (coeff[2],  0.99, 1e-1);

/*  std::vector<double> coeff_ref = sac->refineCoefficients ();
  EXPECT_EQ ((int)coeff_ref.size (), 3);
  //printf ("Circle 2D coefficients (refined): %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3]);
*/
  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (MSAC, SACModelCircle2D)
{
  std_msgs::PointCloud points;
  points.pts.resize (18);

  points.pts[0].x = 3.587751;  points.pts[0].y = -4.190982;  points.pts[0].z = 0;
  points.pts[1].x = 3.808883;  points.pts[1].y = -4.412265;  points.pts[1].z = 0;
  points.pts[2].x = 3.587525;  points.pts[2].y = -5.809143;  points.pts[2].z = 0;
  points.pts[3].x = 2.999913;  points.pts[3].y = -5.999980;  points.pts[3].z = 0;
  points.pts[4].x = 2.412224;  points.pts[4].y = -5.809090;  points.pts[4].z = 0;
  points.pts[5].x = 2.191080;  points.pts[5].y = -5.587682;  points.pts[5].z = 0;
  points.pts[6].x = 2.048941;  points.pts[6].y = -5.309003;  points.pts[6].z = 0;
  points.pts[7].x = 2.000397;  points.pts[7].y = -4.999944;  points.pts[7].z = 0;
  points.pts[8].x = 2.999953;  points.pts[8].y = -6.000056;  points.pts[8].z = 0;
  points.pts[9].x = 2.691127;  points.pts[9].y = -5.951136;  points.pts[9].z = 0;
  points.pts[10].x = 2.190892; points.pts[10].y = -5.587838; points.pts[10].z = 0;
  points.pts[11].x = 2.048874; points.pts[11].y = -5.309052; points.pts[11].z = 0;
  points.pts[12].x = 1.999990; points.pts[12].y = -5.000147; points.pts[12].z = 0;
  points.pts[13].x = 2.049026; points.pts[13].y = -4.690918; points.pts[13].z = 0;
  points.pts[14].x = 2.190956; points.pts[14].y = -4.412162; points.pts[14].z = 0;
  points.pts[15].x = 2.412231; points.pts[15].y = -4.190918; points.pts[15].z = 0;
  points.pts[16].x = 2.691027; points.pts[16].y = -4.049060; points.pts[16].z = 0;
  points.pts[17].x = 2;        points.pts[17].y = -3;        points.pts[17].z = 0;

  SACModel *model = new SACModelCircle2D ();
  SAC *sac        = new MSAC (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->pts.size (), 18);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 17);

  std::vector<double> coeff = sac->computeCoefficients ();
  EXPECT_EQ ((int)coeff.size (), 3);
  //printf ("Circle 2D coefficients: %f %f %f\n", coeff[0], coeff[1], coeff[2]);
  EXPECT_NEAR (coeff[0],  2.99, 1e-1);
  EXPECT_NEAR (coeff[1], -5.00, 1e-1);
  EXPECT_NEAR (coeff[2],  0.99, 1e-1);

/*  std::vector<double> coeff_ref = sac->refineCoefficients ();
  EXPECT_EQ ((int)coeff_ref.size (), 3);
  //printf ("Circle 2D coefficients (refined): %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3]);
*/
  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (MLESAC, SACModelCircle2D)
{
  std_msgs::PointCloud points;
  points.pts.resize (18);

  points.pts[0].x = 3.587751;  points.pts[0].y = -4.190982;  points.pts[0].z = 0;
  points.pts[1].x = 3.808883;  points.pts[1].y = -4.412265;  points.pts[1].z = 0;
  points.pts[2].x = 3.587525;  points.pts[2].y = -5.809143;  points.pts[2].z = 0;
  points.pts[3].x = 2.999913;  points.pts[3].y = -5.999980;  points.pts[3].z = 0;
  points.pts[4].x = 2.412224;  points.pts[4].y = -5.809090;  points.pts[4].z = 0;
  points.pts[5].x = 2.191080;  points.pts[5].y = -5.587682;  points.pts[5].z = 0;
  points.pts[6].x = 2.048941;  points.pts[6].y = -5.309003;  points.pts[6].z = 0;
  points.pts[7].x = 2.000397;  points.pts[7].y = -4.999944;  points.pts[7].z = 0;
  points.pts[8].x = 2.999953;  points.pts[8].y = -6.000056;  points.pts[8].z = 0;
  points.pts[9].x = 2.691127;  points.pts[9].y = -5.951136;  points.pts[9].z = 0;
  points.pts[10].x = 2.190892; points.pts[10].y = -5.587838; points.pts[10].z = 0;
  points.pts[11].x = 2.048874; points.pts[11].y = -5.309052; points.pts[11].z = 0;
  points.pts[12].x = 1.999990; points.pts[12].y = -5.000147; points.pts[12].z = 0;
  points.pts[13].x = 2.049026; points.pts[13].y = -4.690918; points.pts[13].z = 0;
  points.pts[14].x = 2.190956; points.pts[14].y = -4.412162; points.pts[14].z = 0;
  points.pts[15].x = 2.412231; points.pts[15].y = -4.190918; points.pts[15].z = 0;
  points.pts[16].x = 2.691027; points.pts[16].y = -4.049060; points.pts[16].z = 0;
  points.pts[17].x = 2;        points.pts[17].y = -3;        points.pts[17].z = 0;

  SACModel *model = new SACModelCircle2D ();
  SAC *sac        = new MLESAC (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->pts.size (), 18);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 17);

  std::vector<double> coeff = sac->computeCoefficients ();
  EXPECT_EQ ((int)coeff.size (), 3);
  //printf ("Circle 2D coefficients: %f %f %f\n", coeff[0], coeff[1], coeff[2]);
  EXPECT_NEAR (coeff[0],  2.99, 1e-1);
  EXPECT_NEAR (coeff[1], -5.00, 1e-1);
  EXPECT_NEAR (coeff[2],  0.99, 1e-1);

/*  std::vector<double> coeff_ref = sac->refineCoefficients ();
  EXPECT_EQ ((int)coeff_ref.size (), 3);
  //printf ("Circle 2D coefficients (refined): %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3]);
*/
  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (RRANSAC, SACModelCircle2D)
{
  std_msgs::PointCloud points;
  points.pts.resize (18);

  points.pts[0].x = 3.587751;  points.pts[0].y = -4.190982;  points.pts[0].z = 0;
  points.pts[1].x = 3.808883;  points.pts[1].y = -4.412265;  points.pts[1].z = 0;
  points.pts[2].x = 3.587525;  points.pts[2].y = -5.809143;  points.pts[2].z = 0;
  points.pts[3].x = 2.999913;  points.pts[3].y = -5.999980;  points.pts[3].z = 0;
  points.pts[4].x = 2.412224;  points.pts[4].y = -5.809090;  points.pts[4].z = 0;
  points.pts[5].x = 2.191080;  points.pts[5].y = -5.587682;  points.pts[5].z = 0;
  points.pts[6].x = 2.048941;  points.pts[6].y = -5.309003;  points.pts[6].z = 0;
  points.pts[7].x = 2.000397;  points.pts[7].y = -4.999944;  points.pts[7].z = 0;
  points.pts[8].x = 2.999953;  points.pts[8].y = -6.000056;  points.pts[8].z = 0;
  points.pts[9].x = 2.691127;  points.pts[9].y = -5.951136;  points.pts[9].z = 0;
  points.pts[10].x = 2.190892; points.pts[10].y = -5.587838; points.pts[10].z = 0;
  points.pts[11].x = 2.048874; points.pts[11].y = -5.309052; points.pts[11].z = 0;
  points.pts[12].x = 1.999990; points.pts[12].y = -5.000147; points.pts[12].z = 0;
  points.pts[13].x = 2.049026; points.pts[13].y = -4.690918; points.pts[13].z = 0;
  points.pts[14].x = 2.190956; points.pts[14].y = -4.412162; points.pts[14].z = 0;
  points.pts[15].x = 2.412231; points.pts[15].y = -4.190918; points.pts[15].z = 0;
  points.pts[16].x = 2.691027; points.pts[16].y = -4.049060; points.pts[16].z = 0;
  points.pts[17].x = 2;        points.pts[17].y = -3;        points.pts[17].z = 0;

  SACModel *model = new SACModelCircle2D ();
  SAC *sac        = new RRANSAC (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->pts.size (), 18);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 17);

  std::vector<double> coeff = sac->computeCoefficients ();
  EXPECT_EQ ((int)coeff.size (), 3);
  //printf ("Circle 2D coefficients: %f %f %f\n", coeff[0], coeff[1], coeff[2]);
  EXPECT_NEAR (coeff[0],  2.99, 1e-1);
  EXPECT_NEAR (coeff[1], -5.00, 1e-1);
  EXPECT_NEAR (coeff[2],  0.99, 1e-1);

/*  std::vector<double> coeff_ref = sac->refineCoefficients ();
  EXPECT_EQ ((int)coeff_ref.size (), 3);
  //printf ("Circle 2D coefficients (refined): %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3]);
*/
  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (RMSAC, SACModelCircle2D)
{
  std_msgs::PointCloud points;
  points.pts.resize (18);

  points.pts[0].x = 3.587751;  points.pts[0].y = -4.190982;  points.pts[0].z = 0;
  points.pts[1].x = 3.808883;  points.pts[1].y = -4.412265;  points.pts[1].z = 0;
  points.pts[2].x = 3.587525;  points.pts[2].y = -5.809143;  points.pts[2].z = 0;
  points.pts[3].x = 2.999913;  points.pts[3].y = -5.999980;  points.pts[3].z = 0;
  points.pts[4].x = 2.412224;  points.pts[4].y = -5.809090;  points.pts[4].z = 0;
  points.pts[5].x = 2.191080;  points.pts[5].y = -5.587682;  points.pts[5].z = 0;
  points.pts[6].x = 2.048941;  points.pts[6].y = -5.309003;  points.pts[6].z = 0;
  points.pts[7].x = 2.000397;  points.pts[7].y = -4.999944;  points.pts[7].z = 0;
  points.pts[8].x = 2.999953;  points.pts[8].y = -6.000056;  points.pts[8].z = 0;
  points.pts[9].x = 2.691127;  points.pts[9].y = -5.951136;  points.pts[9].z = 0;
  points.pts[10].x = 2.190892; points.pts[10].y = -5.587838; points.pts[10].z = 0;
  points.pts[11].x = 2.048874; points.pts[11].y = -5.309052; points.pts[11].z = 0;
  points.pts[12].x = 1.999990; points.pts[12].y = -5.000147; points.pts[12].z = 0;
  points.pts[13].x = 2.049026; points.pts[13].y = -4.690918; points.pts[13].z = 0;
  points.pts[14].x = 2.190956; points.pts[14].y = -4.412162; points.pts[14].z = 0;
  points.pts[15].x = 2.412231; points.pts[15].y = -4.190918; points.pts[15].z = 0;
  points.pts[16].x = 2.691027; points.pts[16].y = -4.049060; points.pts[16].z = 0;
  points.pts[17].x = 2;        points.pts[17].y = -3;        points.pts[17].z = 0;

  SACModel *model = new SACModelCircle2D ();
  SAC *sac        = new RMSAC (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->pts.size (), 18);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 17);

  std::vector<double> coeff = sac->computeCoefficients ();
  EXPECT_EQ ((int)coeff.size (), 3);
  //printf ("Circle 2D coefficients: %f %f %f\n", coeff[0], coeff[1], coeff[2]);
  EXPECT_NEAR (coeff[0],  2.99, 1e-1);
  EXPECT_NEAR (coeff[1], -5.00, 1e-1);
  EXPECT_NEAR (coeff[2],  0.99, 1e-1);

/*  std::vector<double> coeff_ref = sac->refineCoefficients ();
  EXPECT_EQ ((int)coeff_ref.size (), 3);
  //printf ("Circle 2D coefficients (refined): %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3]);
*/
  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
