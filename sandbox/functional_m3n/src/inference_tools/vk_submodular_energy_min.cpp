/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Daniel Munoz
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

#include <functional_m3n/inference_tools/vk_submodular_energy_min.h>

using namespace vk_energy;
using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
SubmodularEnergyMin::SubmodularEnergyMin(int node_num_max, int edge_num_max)
{
  graph_ = new Graph<double, double, double>(node_num_max, edge_num_max);
  const_offset_ = 0.0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
double SubmodularEnergyMin::minimize()
{

  double flow = graph_->maxflow();
  return flow + const_offset_;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
unsigned int SubmodularEnergyMin::getValue(const EnergyVar& x)
{
  if (graph_->what_segment(x) == Graph<double,double,double>::SOURCE)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SubmodularEnergyMin::addPairwise(const EnergyVar& x,
                                     const EnergyVar& y,
                                     double A,
                                     double B,
                                     double C,
                                     double D)
{
  // 0 = source
  // 1 = sink

  /* See Table 1 in Kolmogorov and Zabih, TPAMI 2004
   E00 = A
   E01 = B
   E10 = C
   E11 = D
   */

  /* check submodularity */
  if ((A + D) > (B + C))
  {
    ROS_ERROR("Function is not submodular.  This should hold true: %f + %f <= %f + %f", A, D, B, C);
    return -1;
  }

  /* 
   E = A A  +  0   B-A
   D D     C-D 0
   Add edges for the first term
   */
  graph_->add_tweights(x, D, A);

  /* Modify B and C to represent the second term compactly
   0   B-A
   C-D 0
   */
  B -= A;
  C -= D;

  /* Now only need to represent
   0 B
   C 0
   */
  if (B < 0)
  {
    /* Write it as
     B B  +  -B 0  +  0   0
     0 0     -B 0     B+C 0
     */
    graph_->add_tweights(x, 0, B); /* first term */
    graph_->add_tweights(y, 0, -B); /* second term */
    graph_->add_edge(x, y, 0, B + C); /* third term */
  }
  else if (C < 0)
  {
    /* Write it as
     -C -C  +  C 0  +  0 B+C
     0  0     C 0     0 0
     */
    graph_->add_tweights(x, 0, -C); /* first term */
    graph_->add_tweights(y, 0, C); /* second term */
    graph_->add_edge(x, y, B + C, 0); /* third term */
  }
  else /* B >= 0, C >= 0 */
  {
    graph_->add_edge(x, y, B, C);
  }
  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SubmodularEnergyMin::addPnPotts(const list<EnergyVar>& clique_vars, double Ec0, double Ec1, double Emax)
{
  // Emax >= Ec0, Ec1
  if (Ec0 > (Emax + 1e-5))
  {
    ROS_ERROR("Emax %f should be bigger than Ec0 %f", Emax, Ec0);
    return -1;
  }
  if (Ec1 > (Emax + 1e-5))
  {
    ROS_ERROR("Emax %f should be bigger than Ec1 %f", Emax, Ec1);
    return -1;
  }

  // calculate edge capacities
  double kappa = Emax - Ec0 - Ec1;
  double w_d = Ec1 + kappa;
  double w_e = Ec0 + kappa;

  // added kappa to capacities to ensure non-negative capacities,
  // this will not change optimal solution, but will change value
  // of min-cut so need to offset to get the true energy
  const_offset_ -= kappa;

  // declare auxiliary nodes
  EnergyVar m_s = addVariable();
  EnergyVar m_t = addVariable();

  // add capacities for auxiliary nodes to source and sink
  graph_->add_tweights(m_s, w_d, 0.0);
  graph_->add_tweights(m_t, 0.0, w_e);

  // add edges between extra nodes and clique nodes
  list<EnergyVar>::const_iterator iter_clique_vars;
  for (iter_clique_vars = clique_vars.begin(); iter_clique_vars != clique_vars.end() ; iter_clique_vars++)
  {
    graph_->add_edge(m_s, *iter_clique_vars, w_d, 0.0);
    graph_->add_edge(*iter_clique_vars, m_t, w_e, 0.0);
  }

  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SubmodularEnergyMin::addRobustPottsDominantExpand0(const list<EnergyVar>& node_vars,
                                                       const list<EnergyVar>& dominant_vars,
                                                       double gamma_alpha,
                                                       double gamma_dominant,
                                                       double gamma_max,
                                                       double Q)
{
  // Equations 21, 30
  // P = W(c)
  // P_d = W(c_d);
  double P = static_cast<double> (node_vars.size());
  double P_d = static_cast<double> (dominant_vars.size());

  // -------------------------------------------------
  // Double checks
  // gamma_max >= gamma_alpha, gamma_dominant
  if (gamma_alpha > (gamma_max + 1e-5))
  {
    ROS_ERROR("gamma_max %f should be bigger than gamma_alpha %f", gamma_max, gamma_alpha);
    return -1;
  }
  if (gamma_dominant > (gamma_max + 1e-5))
  {
    ROS_ERROR("gamma_max %f should be bigger than gamma_dominant %f", gamma_max, gamma_dominant);
    return -1;
  }

  // P > Q_a + Q_b for all labels a,b.
  // We assumed all Q_k are equal
  if ((2.0 * Q) > (P + 1e-5))
  {
    ROS_ERROR("P %f should be bigger than 2Q=%f", P, 2.0 * Q);
    return -1;
  }

  // W(c_d) > P - Q
  if ((P - Q) > (P_d + 1e-5))
  {
    ROS_ERROR("W(c_d) %f should be bigger than P-Q %f", P_d, (P - Q));
    return -1;
  }

  // -------------------------------------------------
  // Equation 24
  // theta_k = (gamma_max - gamma_k) / Q_k
  double theta_dominant = (gamma_max - gamma_dominant) / Q;
  double theta_alpha = (gamma_max - gamma_alpha) / Q;

  // -------------------------------------------------
  // Equation 41
  // R_d = W(c - c_d)
  // lambda_alpha = gamma_alpha
  // lambda_d = gamma_d + R_d * theta_d
  // lambda_max = gamma_max
  double R_dominant = P - P_d;
  double lambda_alpha = gamma_alpha;
  double lambda_dominant = gamma_dominant + R_dominant * theta_dominant;
  double lambda_max = gamma_max;

  // -------------------------------------------------
  // Equation 43
  // delta = lambda_max - lambda_alpha - lambda_d
  // r0 = lambda_alpha + delta
  // r1 = lambda_d + delta
  double delta = lambda_max - lambda_alpha - lambda_dominant;
  double r_0 = lambda_alpha + delta;
  double r_1 = lambda_dominant + delta;

  EnergyVar m_0 = addVariable();
  EnergyVar m_1 = addVariable();

  // Below is implementing Equation 43 into the graph

  // -----------------------------------
  // r_0 * (1-m_0)
  addUnary(m_0, r_0, 0.0);

  // -----------------------------------
  // theta_d * m_0 * \sum(w_i*(1-t_i))
  list<EnergyVar>::const_iterator iter_vars;
  for (iter_vars = dominant_vars.begin(); iter_vars != dominant_vars.end() ; iter_vars++)
  {
    // arguments: 00, 01, 10, 11
    addPairwise(m_0, *iter_vars, 0.0, 0.0, theta_dominant, 0.0);
  }

  // -----------------------------------
  // r1 * m1
  addUnary(m_1, 0.0, r_1);

  // -----------------------------------
  // theta_alpha * (1 - m_1) * \sum(w_i*t_i)
  for (iter_vars = node_vars.begin(); iter_vars != node_vars.end() ; iter_vars++)
  {
    // arguments: 00, 01, 10, 11
    addPairwise(m_1, *iter_vars, 0.0, theta_alpha, 0.0, 0.0);
  }

  // -----------------------------------
  // - delta
  const_offset_ -= delta;

  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SubmodularEnergyMin::addRobustPottsNoDominantExpand0(const list<EnergyVar>& node_vars,
                                                         double gamma_alpha,
                                                         double gamma_max,
                                                         double Q)
{
  double P = static_cast<double> (node_vars.size());

  // -------------------------------------------------
  // Double checks
  // gamma_max >= gamma_alpha, gamma_dominant
  if (gamma_alpha > (gamma_max + 1e-5))
  {
    ROS_ERROR("gamma_max %f should be bigger than gamma_alpha %f", gamma_max, gamma_alpha);
    return -1;
  }
  // P > Q_a + Q_b for all labels a,b.
  // We assumed all Q_k are equal
  if ((2.0 * Q) > (P + 1e-5))
  {
    ROS_ERROR("P %f should be bigger than 2Q=%f", P, 2.0 * Q);
    return -1;
  }

  // -------------------------------------------------
  // Equation 24
  // theta_k = (gamma_max - gamma_k) / Q_k
  double theta_alpha = (gamma_max - gamma_alpha) / Q;

  // -------------------------------------------------
  // Equation 44
  double r_1 = gamma_max - gamma_alpha;

  EnergyVar m_1 = addVariable();

  // r1 * m_1
  addUnary(m_1, 0.0, r_1);

  // theta_alpha * (1-m_1) * \sum(w_i*t_i)
  list<EnergyVar>::const_iterator iter_vars;
  for (iter_vars = node_vars.begin(); iter_vars != node_vars.end() ; iter_vars++)
  {
    // arguments: 00, 01, 10, 11
    addPairwise(m_1, *iter_vars, 0.0, theta_alpha, 0.0, 0.0);
  }

  // + lambda_alpha
  const_offset_ += gamma_alpha;

  return 0;
}
