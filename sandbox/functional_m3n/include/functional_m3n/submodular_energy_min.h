#ifndef __SUBMODULAR_ENERGY_MIN_H__
#define __SUBMODULAR_ENERGY_MIN_H__
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

#include <assert.h>
#include <string>

#include <boost/config.hpp>
#include <boost/graph/kolmogorov_max_flow.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graph_traits.hpp>

using namespace boost;
using namespace std;

// --------------------------------------------------------------
//* SubmodularEnergyMin
/** 
 * \brief Minimizes a submodular energy function with binary variables
 *
 * Minimization is done using the Graph Boost Library
 *
 * This implementation is based on a class originally written by Vladimir Kolmogorov,
 * implementing the graph structure as described in:
 *  What Energy Functions can be Minimized via Graph Cuts?
 *  Vladimir Kolmogorov and Ramin Zabih.
 *  IEEE Transactions on Pattern Analysis and Machine Intelligence, 2004
 */
// --------------------------------------------------------------
class SubmodularEnergyMin
{
  public:
    // Change properties of the graph as needed
    typedef adjacency_list_traits<vecS, vecS, directedS> Traits;
    typedef adjacency_list<vecS, vecS, directedS, property<vertex_name_t, std::string, property<
        vertex_index_t, unsigned int, property<vertex_color_t, boost::default_color_type, property<
            vertex_distance_t, double, property<vertex_predecessor_t, Traits::edge_descriptor> > > > > ,

    property<edge_capacity_t, double, property<edge_residual_capacity_t, double, property<edge_reverse_t,
        Traits::edge_descriptor> > > > Graph;

    typedef Traits::vertex_descriptor EnergyVar;

    // --------------------------------------------------------------
    /*!
     * \brief Instantiates empty energy function
     */
    // --------------------------------------------------------------
    SubmodularEnergyMin();

    ~SubmodularEnergyMin()
    {
    }
    ;

    // --------------------------------------------------------------
    /*!
     * \brief Adds a new variable to this function
     */
    // --------------------------------------------------------------
    const EnergyVar addVariable();

    // --------------------------------------------------------------
    /*!
     * \brief Adds a unary term E(x) to this function.
     *
     * This function can be called multiple times per variable
     *
     * \param x The variable
     * \param E0 The unary energy when x = 0
     * \param E1 The unary energy when x = 1
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    void addUnary(const EnergyVar& x, double E0, double E1);

    // --------------------------------------------------------------
    /*!
     * \brief Adds a submodular pairwise term E(x,y) to this function.
     *
     * This function can be called multiple times per variable pairs
     *
     * Warning, the term must be submodular:
     *         E00 + E11 <= E01 + E10
     *
     * \param x The variable
     * \param y The variable
     * \param E00 The pairwise energy when x = 0, y = 0
     * \param E01 The pairwise energy when x = 0, y = 1
     * \param E10 The pairwise energy when x = 1, y = 0
     * \param E11 The pairwise energy when x = 1, y = 1
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int addPairwise(const EnergyVar& x, const EnergyVar& y, double E00, double E01, double E10, double E11);

    // --------------------------------------------------------------
    /*!
     * \brief Adds a Pn Potts high-order term E(x_1,..,x_n) over n variables, as described
     *        in Kohli et al., "P3 & Beyond: Solving Energies with Higher Order Cliques", CVPR 2007
     *
     * If implementing alpha-expansion and using 0 to represent the alpha label and 1 for the current
     * label, then Ec0 = gamma_alpha and Ec1 = gamma (where gamma = gamma_beta if all nodes in the
     * clique are currently labeled beta, and gamma = Emax otherwise), as described in Equation 38.
     *
     * \param clique_vars List of energy variables that constitute the clique
     * \param Ec0 The energy if all variables in the clique take value 0
     * \param Ec1 The energy if all variables in the clique take value 1
     * \param Emax The max energy value the clique can have
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int addPnPotts(const list<EnergyVar>& clique_vars, double Ec0, double Ec1, double Emax);

    // --------------------------------------------------------------
    /*!
     * \brief Adds a Robust Pn Potts high-order term E(x_1,...,x_n) over n variables, as described
     *        in Kohli et al., "Robust Higher Order Potentials for Enforcing Label Consistency", IJCV 2009
     *
     * This method implements the alpha-expansion move for the high-order potential. It uses a basic form
     * of the potential where all nodes have equal weight for all labels and the truncation parameter Q is
     * the same for all labels.
     *
     * This method should be called when a dominant label d in the clique can be found, that is
     * when D > P - Q, where D is the number of nodes in the clique that take on label d != alpha,
     * P is the number of nodes in the clique, and Q is the truncation parameter.  Call the method
     * addRobustPottsNoDominantExpand0 if no dominant label can be found.
     *
     * IMPORTANT: this function assumes 0 represents the alpha-label and 1 represents the current
     * labeling.  Ensure to call the other energy functions appropriately.
     *
     * \param node_vars List of energy variables that constitute the clique
     * \param dominant_vars List of energy variables that currently take on the clique's dominant labels
     * \param gamma_alpha The energy if all variables in the clique take on the alpha label (value 0)
     * \param gamma_dominant The energy if dominant variables keep their label (value 1)
     * \param gamma_max The max energy value the clique can have
     * \param Q The truncation parameter, it is the number of nodes that can disagree with
     *          the dominant label.  IMPORTANT: 2Q < clique_vars.size()
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int addRobustPottsDominantExpand0(const list<EnergyVar>& node_vars,
                                      const list<EnergyVar>& dominant_vars,
                                      double gamma_alpha,
                                      double gamma_dominant,
                                      double gamma_max,
                                      double Q);

    // --------------------------------------------------------------
    /*!
     * \brief Adds a Robust Pn Potts high-order term E(x_1,...,x_n) over n variables, as described
     *        in Kohli et al., "Robust Higher Order Potentials for Enforcing Label Consistency", IJCV 2009
     *
     * This method implements the alpha-expansion move for the high-order potential. It uses a basic form
     * of the potential where all nodes have equal weight for all labels and the truncation parameter Q is
     * the same for all labels.
     *
     * This method should be called when NO dominant label in the clique can be found, that is
     * there is no label d != alpha such that D > P - Q, where D is the number of nodes in the clique that take
     * on label d, P is the number of nodes in the clique, and Q is the truncation parameter.
     *
     * IMPORTANT: this function assumes 0 represents the alpha-label and 1 represents the current
     * labeling.  Ensure to call the other energy functions appropriately.
     *
     * \param node_vars List of energy variables that constitute the clique
     * \param gamma_alpha The energy if all variables in the clique take on the alpha label (value 0)
     * \param gamma_max The max energy value the clique can have
     * \param Q The truncation parameter, it is the number of nodes that can disagree with
     *          the dominant label.  IMPORTANT: 2Q < clique_vars.size()
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int addRobustPottsNoDominantExpand0(const list<EnergyVar>& node_vars,
                                        double gamma_alpha,
                                        double gamma_max,
                                        double Q);

    // --------------------------------------------------------------
    /*!
     * \brief Minimizes the current energy function
     *
     * \return The minimum energy value
     */
    // --------------------------------------------------------------
    double minimize();

    // --------------------------------------------------------------
    /*!
     * \brief Returns the binary value for the variable
     *
     * This should be called after calling minimize()
     *
     * \return 0 or 1 for the value of x in the minimizing solution
     */
    // --------------------------------------------------------------
    unsigned int getValue(const EnergyVar& x);

  private:
    // --------------------------------------------------------------
    /*!
     * \brief Creates two edges: Source->x and x->Sink with given capacities
     *
     * Capacities can be negative.
     *
     * \param x Variable
     * \param cap_source_to_x The capacity of edge from Source -> x
     * \param cap_x_to_sink The capacity of edge from x -> Sink
     */
    // --------------------------------------------------------------
    void add_tweights(const EnergyVar& x, const double cap_source_to_x, const double cap_x_to_sink);

    // --------------------------------------------------------------
    /*!
     * \brief Creates two edges: from->to and to->from with given capacities
     *
     * Capacities can NOT be negative.
     *
     * \param from Variable
     * \param to Variable
     * \param cap_val The capacity of edge from->to
     * \param reverse_cap_val The capacity of edge to->from
     */
    // --------------------------------------------------------------
    void create_dedge(const EnergyVar& from, const EnergyVar& to, double cap_val, double reverse_cap_val);

    EnergyVar source_;
    EnergyVar sink_;
    Graph graph_;

    double const_offset_;
};

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
inline SubmodularEnergyMin::SubmodularEnergyMin()
{
  source_ = add_vertex(graph_);
  sink_ = add_vertex(graph_);

  const_offset_ = 0.0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
inline const SubmodularEnergyMin::EnergyVar SubmodularEnergyMin::addVariable()
{
  return add_vertex(graph_);
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
inline double SubmodularEnergyMin::minimize()
{

  double flow = kolmogorov_max_flow(graph_, source_, sink_);

  /*
   std::cout << "flow values: node_id node_id flow (capacity)" << std::endl;
   property_map < Graph, edge_capacity_t >::type capacity = get(edge_capacity, graph_);
   property_map < Graph, edge_residual_capacity_t >::type
   residual_capacity = get(edge_residual_capacity, graph_);
   graph_traits<Graph>::vertex_iterator u_iter, u_end;
   graph_traits<Graph>::out_edge_iterator ei, e_end;
   for (tie(u_iter, u_end) = vertices(graph_); u_iter != u_end; ++u_iter)
   for (tie(ei, e_end) = out_edges(*u_iter, graph_); ei != e_end; ++ei)
   if (capacity[*ei] > 0)
   std::cout << "f " << *u_iter << " " << target(*ei, graph_) << " "
   << (capacity[*ei] - residual_capacity[*ei]) << " (" << capacity[*ei] << ")" << std::endl;
   */

  return flow + const_offset_;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
inline unsigned int SubmodularEnergyMin::getValue(const EnergyVar& x)
{
  // 0 = source
  // 1 = sink
  property_map<Graph, vertex_color_t>::type v_colors = get(vertex_color, graph_);

  // From Boost documentation:
  // "If the color of a vertex after running the algorithm is black the vertex 
  //  belongs to the source tree else it belongs to the sink-tree."
  if (v_colors[x] == boost::black_color)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
inline void SubmodularEnergyMin::addUnary(const EnergyVar& x, double E0, double E1)
{
  // 0 = source
  // 1 = sink
  add_tweights(x, E1, E0); // it should be backwards
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
inline int SubmodularEnergyMin::addPairwise(const EnergyVar& x,
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
  add_tweights(x, D, A);

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
    add_tweights(x, 0, B); /* first term */
    add_tweights(y, 0, -B); /* second term */
    create_dedge(x, y, 0, B + C); /* third term */
  }
  else if (C < 0)
  {
    /* Write it as
     -C -C  +  C 0  +  0 B+C
     0  0     C 0     0 0
     */
    add_tweights(x, 0, -C); /* first term */
    add_tweights(y, 0, C); /* second term */
    create_dedge(x, y, B + C, 0); /* third term */
  }
  else /* B >= 0, C >= 0 */
  {
    create_dedge(x, y, B, C);
  }
  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
inline void SubmodularEnergyMin::create_dedge(const EnergyVar& from,
                                              const EnergyVar& to,
                                              double cap_val,
                                              double reverse_cap_val)
{
  property_map<Graph, edge_capacity_t>::type capacity = get(edge_capacity, graph_);
  property_map<Graph, edge_reverse_t>::type rev = get(edge_reverse, graph_);

  Traits::edge_descriptor e1;
  Traits::edge_descriptor e2;
  bool in1 = false;
  bool in2 = false;

  tie(e1, in1) = add_edge(from, to, graph_);
  tie(e2, in2) = add_edge(to, from, graph_);

  // if an edge already exists, augment capacity
  if (in1 == false)
  {
    capacity[e1] += cap_val;
  }
  else
  {
    capacity[e1] = cap_val;
    rev[e1] = e2;
  }
  if (in2 == false)
  {
    capacity[e2] += reverse_cap_val;
  }
  else
  {
    capacity[e2] = reverse_cap_val;
    rev[e2] = e1;
  }
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
inline void SubmodularEnergyMin::add_tweights(const EnergyVar& x,
                                              double cap_source_to_x,
                                              double cap_x_to_sink)
{
  // Make negative weights/capacities to be positive
  // This will not affect the solution of the variables,
  // but will change the value, so we need to offset.
  // (Subtract a little due to avoid precision errors)
  double min_val = min(cap_source_to_x, cap_x_to_sink) - 1e-5;
  if (min_val < 0.0)
  {
    cap_source_to_x -= min_val;
    cap_x_to_sink -= min_val;
    const_offset_ += min_val;

    // Debug: add extra to see flow over all edges
    //cap_source_to_x += 1.0;
    //cap_x_to_sink += 1.0;
    //const_offset_ -= 1.0;
  }

  create_dedge(source_, x, cap_source_to_x, 0.0);
  create_dedge(x, sink_, cap_x_to_sink, 0.0);
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
inline int SubmodularEnergyMin::addPnPotts(const list<EnergyVar>& clique_vars,
                                           double Ec0,
                                           double Ec1,
                                           double Emax)
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
  EnergyVar m_s = add_vertex(graph_);
  EnergyVar m_t = add_vertex(graph_);

  // add capacities for auxiliary nodes to source and sink
  add_tweights(m_s, w_d, 0.0);
  add_tweights(m_t, 0.0, w_e);

  // add edges between extra nodes and clique nodes
  list<EnergyVar>::const_iterator iter_clique_vars;
  for (iter_clique_vars = clique_vars.begin(); iter_clique_vars != clique_vars.end() ; iter_clique_vars++)
  {
    create_dedge(m_s, *iter_clique_vars, w_d, 0.0);
    create_dedge(*iter_clique_vars, m_t, w_e, 0.0);
  }

  return 0;
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
inline int SubmodularEnergyMin::addRobustPottsDominantExpand0(const list<EnergyVar>& node_vars, const list<
    EnergyVar>& dominant_vars, double gamma_alpha, double gamma_dominant, double gamma_max, double Q)
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

  EnergyVar m_0 = add_vertex(graph_);
  EnergyVar m_1 = add_vertex(graph_);

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
/*! See function definition */
// --------------------------------------------------------------
inline int SubmodularEnergyMin::addRobustPottsNoDominantExpand0(const list<EnergyVar>& node_vars,
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

  EnergyVar m_1 = add_vertex(graph_);

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
#endif
