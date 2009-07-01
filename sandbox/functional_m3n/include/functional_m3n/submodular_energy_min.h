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
    void addPairwise(const EnergyVar& x, const EnergyVar& y, double E00, double E01, double E10, double E11);

    // TODO
    void addPnPotts();
    void addRobustPnPotts();

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
inline void SubmodularEnergyMin::addPairwise(const EnergyVar& x,
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
    assert(false);
    // TODO: throw exception
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
  double min_val = min(cap_source_to_x, cap_x_to_sink);
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

#endif
