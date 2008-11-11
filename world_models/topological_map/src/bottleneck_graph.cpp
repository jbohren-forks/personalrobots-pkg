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

#include <topological_map/bottleneck_graph.h>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <boost/graph/breadth_first_search.hpp>     
#include <boost/graph/connected_components.hpp>     


using std::cout;
using std::endl;
using boost::get;

namespace topological_map 
{

// Typedefs for graphs
struct coords_t { typedef boost::vertex_property_tag kind; };
typedef boost::property<coords_t,Coords> coords_property; 

struct bottleneck_t { typedef boost::vertex_property_tag kind; };
typedef boost::property<bottleneck_t,bool,coords_property> bottleneck_property;

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, bottleneck_property> Graph; 
typedef boost::property_map<Graph, coords_t>::type CoordsMap; // The property map for coordinates
typedef boost::property_map<Graph, bottleneck_t>::type BottleneckMap; // And for whether or not a cell is a bottleneck
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
typedef boost::graph_traits<Graph>::out_edge_iterator edge_iter;
typedef std::map<Vertex,int> VertexCompMap;
typedef std::list<Coords> CoordsList;

// Struct containing a graph over the vertices of a 2d grid together with a 2d array allowing vertices to be looked up quickly
typedef boost::multi_array<Vertex, 2> VertexMap;
typedef boost::multi_array<bool, 2> OccMap;
struct GridGraph 
{
  Graph g;
  VertexMap m;
  OccMap o;

  GridGraph (const grid_size* dims) : m(boost::extents[dims[0]][dims[1]]), o(boost::extents[dims[0]][dims[1]]) {}
};



// We want to have a breadth-first search that terminates
// when a goal vertex is reached, or a distance threshold is
// exceeded.  This is done by adding a visitor to the 
// boost bfs algorithm that throws an exception when one
// of these things happens. 
struct TerminateBfs
{
  bool found;
  TerminateBfs (bool f) : found(f) {}
};
class BfsVisitor : public boost::default_bfs_visitor {
public:
  void discover_vertex (Vertex u, const Graph& g) const
  {
    int r, c;
    boost::tie (r, c) = get (coords_t(), g, u);
    // cout << "Discovering vertex " <<  r << ", " << c << endl;
    if ((r == rg_) && (c == cg_))
      throw TerminateBfs (true);
    else if (abs(r-r0_) + abs(c-c0_) > threshold_)
      throw TerminateBfs (false);
  }

  BfsVisitor (int r0, int c0, int threshold, int rg, int cg)
  {
    r0_ = r0;
    c0_ = c0;
    rg_ = rg;
    cg_ = cg;
    threshold_ = threshold;
  }

private:
  int threshold_, r0_, c0_, rg_, cg_;
};

// Return true iff l1 distance from r0, c0 to r1, c1 is < threshold
bool distLessThan (GridGraph* gr, int r0, int c0, int r1, int c1, int threshold)
{
  BfsVisitor vis (r0, c0, threshold, r1, c1);

  try {
    boost::breadth_first_search (gr->g, gr->m[r0][c0], boost::visitor(vis));
  }
  
  // Breadth first search can either throw an exception with found = true,
  // with found=false, or terminate naturally.  In the latter two cases, the
  // distance is above the threshold.
  catch (TerminateBfs e)
  {
    if (e.found) 
      return true;
  }
  return false;
}




  
GridGraph makeGraphFromGrid (const GridArray& grid)
{
  const grid_size* dims = grid.shape();
  GridGraph gr(dims);
  Vertex v;
  CoordsMap coords = get (coords_t(), gr.g);
  
  
  for (grid_size r=0; r!=dims[0]; r++) {
    for (grid_size c=0; c!=dims[1]; c++) {
      
      gr.o[r][c] = false;
      if (!grid[r][c]) {
        v = add_vertex(gr.g);
        gr.o[r][c] = true;
        gr.m[r][c] = v;
        boost::put (coords, v, *(new Coords(r,c)));
        
        if ((r>0) && !grid[r-1][c]) {
          boost::add_edge (v, gr.m[r-1][c], gr.g);
        }
        if ((c>0) && !grid[r][c-1]) {
          boost::add_edge (v, gr.m[r][c-1], gr.g);
        }
      }
    }
  }

  return gr;
}

void printGraph (const Graph& g) 
{
  vertex_iter vi, vg;
  edge_iter ei, eg;
  coords_t coords_key;
  
  cout << endl;
  for (boost::tie(vi,vg) = boost::vertices(g); vi != vg; vi++)
  {
    int r, c;
    boost::tie (r, c) = get (coords_key, g, *vi);
    std::cout << r << "," << c << " ";
    for (boost::tie(ei, eg) = boost::out_edges (*vi, g); ei != eg; ei++) {
      int r2, c2;
      boost::tie (r2, c2) = get (coords_key, g, boost::target (*ei, g));
      std::cout << r2 << "," << c2 << " ";
    }
    std::cout << std::endl;
  }
}

void printVertexMap (const GridGraph& gr)
{
  const grid_size* dims = gr.m.shape();  
  for (grid_size i=0; i<dims[0]; i++) {
    for (grid_size j=0; j<dims[1]; j++) {
      cout << std::setw(2) << gr.m[i][j] << " ";
    }
    cout << std::endl;
  }
}


void printBottleneckGraph (const BottleneckGraph& g)
{
  BottleneckVertexIterator i, end;
  for (boost::tie(i, end) = boost::vertices(g); i!=end; ++i) {
    BottleneckVertex v = *i;
    cout << "Vertex: " << endl << " Type: ";
    VertexDescription d = get (desc_t(), g, v);
    if (d.type == BOTTLENECK)
      cout << "bottleneck" << endl << " Region: ";
    else
      cout << "open" << endl << " Region: ";
    
    int rmax = 0;
    int rmin = 10000000;
    int cmax = 0;
    int cmin = 10000000;

    for (Region::iterator i = d.region.begin(); i!=d.region.end(); i++) {
      int r, c;
      boost::tie (r, c) = *i;

      rmin = std::min (rmin, r);
      rmax = std::max (rmax, r);
      cmin = std::min (cmin, c);
      cmax = std::max (cmax, c);
    }
    cout << d.region.size() << " cells contained in bounding box between (" << rmin << "," << cmin << ") and (" << rmax << "," << cmax << ")" << endl << endl;
    
  }
  cout << "done" << endl;
}

void printBottlenecks (const BottleneckGraph& g, const GridArray& gr)
{
  const grid_size* dims = gr.shape();  
  OccMap b(boost::extents[dims[0]][dims[1]]); 
  BottleneckVertexIterator i, end;
  for (boost::tie(i, end) = boost::vertices(g); i!=end; ++i) {
    BottleneckVertex v = *i;
    VertexDescription d = get (desc_t(), g, v);
    if (!(d.type == BOTTLENECK))
      continue;


    for (Region::iterator i = d.region.begin(); i!=d.region.end(); i++) {
      int r, c;
      boost::tie (r, c) = *i;
      b[r][c] = true;
    }
    
  }

  for (grid_size r=0; r<dims[0]; r++) {
    for (grid_size c=0; c<dims[1]; c++) {
      if (b[r][c] || gr[r][c]) {
        cout << r << " " << c << endl;
      }
    }
  }
}           


// If the given vertices both exist in the graph, remove edge between them
void possiblyRemove (GridGraph* gr, grid_size r, grid_size c, grid_size r2, grid_size c2)
{
  if (gr->o[r][c] && gr->o[r2][c2])
    remove_edge (gr->m[r][c], gr->m[r2][c2], gr->g);
}


// If the given vertices both exist in the graph, add an edge between them
void possiblyAdd (GridGraph* gr, grid_size r, grid_size c, grid_size r2, grid_size c2)
{
  if (gr->o[r][c] && gr->o[r2][c2])
    add_edge (gr->m[r][c], gr->m[r2][c2], gr->g);
}
    


// Disconnect the square of size s with top-left corner r0, c0 from the rest of the graph
void removeBlock (GridGraph* gr, grid_size r0, grid_size c0, int s)
{
  const grid_size* dims = gr->m.shape();
  for (int i=0; i<s; i++) {
    if (r0>0)
      possiblyRemove (gr, r0, c0+i, r0-1, c0+i);
    if (c0>0)
      possiblyRemove (gr, r0+i, c0, r0+i, c0-1);
    if (r0<dims[0]-s)
      possiblyRemove (gr, r0+s-1, c0+i, r0+s, c0+i);
    if (c0<dims[1]-s)
      possiblyRemove (gr, r0+i, c0+s-1, r0+i, c0+s);
  }

}


// Reconnect the square of size s with top-left corner r0, c0 to the rest of the graph
void addBlock (GridGraph* gr, grid_size r0, grid_size c0, int s)
{
  const grid_size* dims = gr->m.shape();
  for (int i=0; i<s; i++) {
    if (r0>0)
      possiblyAdd (gr, r0, c0+i, r0-1, c0+i);
    if (c0>0)
      possiblyAdd (gr, r0+i, c0, r0+i, c0-1);
    if (r0<dims[0]-s)
      possiblyAdd (gr, r0+s-1, c0+i, r0+s, c0+i);
    if (c0<dims[1]-s)
      possiblyAdd (gr, r0+i, c0+s-1, r0+i, c0+s);
  }

}



// Mark as being bottlenecks the cells in the square of size s with top-left corner r0, c0 
void markBottleneckCells (GridGraph* gr, grid_size r0, grid_size c0, int s)
{
  BottleneckMap bottleneckCells = get (bottleneck_t(), gr->g);
  for (grid_size r=r0; r<r0+s; r++) {
    for (grid_size c=c0; c<c0+s; c++) {
      if (gr->o[r][c]) {
        boost::put (bottleneckCells, gr->m[r][c], true);
      }
    }
  }
}


// Sever connections between bottleneck and non-bottleneck cells
void disconnectBottlenecks (GridGraph* gr)
{
  const grid_size* dims = gr->m.shape();
  BottleneckMap bottlenecks = get (bottleneck_t(), gr->g);
  for (grid_size r=0; r<dims[0]; r++) {
    for (grid_size c=0; c<dims[1]; c++) {
      if (gr->o[r][c]) {
        Vertex v1 = gr->m[r][c];
        if (r>0) {
          if (gr->o[r-1][c]) {
            Vertex v2 = gr->m[r-1][c];
            if (get (bottlenecks, v1) != get (bottlenecks, v2)) {
              boost::remove_edge(v1, v2, gr->g);
            }
          }
        }
        if (c>0) {
          if (gr->o[r][c-1]) {
            Vertex v2 = gr->m[r][c-1];
            if (get (bottlenecks, v1) != get (bottlenecks, v2)) {
              boost::remove_edge(v1, v2, gr->g);
            }
          }
        }
      }
    }
  }
}



// Given two vertices in the occupancy grid, if they correspond to different connected components, add an edge between those components in the bottleneck graph 
void possiblyAddBottleneckEdge (Vertex v1, Vertex v2, const VertexCompMap& vertexComp, const std::vector<BottleneckVertex>& vertices, BottleneckGraph* g)
{
  int r1 = vertexComp.find(v1)->second;
  int r2 = vertexComp.find(v2)->second;
  if (r1 != r2)
    add_edge (vertices[vertexComp.find(v1)->second], vertices[vertexComp.find(v2)->second], *g);
}
  
void printBlock (const GridGraph& g, int r, int c, int s, int r0, int c0, int r1, int c1, int nr, int nc)
{
  cout << r << " " << c << " " << r0 << " " << c0 << " " << r1 << " " << c1 << " " << endl;
  int rmin = std::max (r-1-s/2, 0);
  int rmax = std::min (r+s+s/2, nr-1);
  int cmin = std::max (c-1-s/2, 0);
  int cmax = std::min (c+s+s/2, nc-1);

  for (int i=rmin; i<=rmax; i++) {
    for (int j=cmin; j<=cmax; j++) {
      if ((i==r0) && (j==c0))
        cout << "S";
      else if ((i==r1) && (j==c1)) 
        cout << "G";
      else if (g.o[i][j])
        if ((i<r-1) || (i > r+s) || (j<c-1) || (j>c+s))
          cout << ".";
        else
          cout << " ";
      else
        if ((i<r-1) || (i > r+s) || (j<c-1) || (j>c+s))
          cout << "Y";
        else
          cout << "X";
    }
    cout << endl;
  }
}


// Find adjacent connected components.  Any such adjacent pair must consist of a
// bottleneck region and a nonbottleneck region.
void connectRegions (const GridGraph& gr, const VertexCompMap& vertexComp, const std::vector<BottleneckVertex>& bottleneckVertices, BottleneckGraph* g)
{
  const grid_size* dims = gr.m.shape();
  for (grid_size r=0; r<dims[0]; r++) {
    for (grid_size c=0; c<dims[1]; c++) {
      if (gr.o[r][c]) {
        Vertex v1 = gr.m[r][c];
        
        // Upward edge
        if (r>0) {
          if (gr.o[r-1][c]) {
            Vertex v2 = gr.m[r-1][c];
            possiblyAddBottleneckEdge (v1, v2, vertexComp, bottleneckVertices, g);
          }
        }

        // Leftward edge
        if (c>0) {
          if (gr.o[r][c-1]) {
            Vertex v2 = gr.m[r][c-1];
            possiblyAddBottleneckEdge (v1, v2, vertexComp, bottleneckVertices, g);
          }
        }
      }
    }
  }
}



// Find square regions such that removing the region from the graph significantly increases the distance
// between cells on the boundary
void findDisconnectingBlocks (GridGraph* gr, CoordsList* disconnectingBlocks, int bottleneckSize, int bottleneckSkip,
                              int distanceMultMin, int distanceMultMax)
{

  const grid_size* dims = gr->m.shape();
  int r0, c0, r1, c1;
  for (grid_size r=1; r<dims[0]-bottleneckSize; r+=bottleneckSkip) {
    cout << r << endl;
    for (grid_size c=0; c<=dims[1]-bottleneckSize; c+=bottleneckSkip) {

      // Check if the block disconnects either of the diagonally opposite corners
      bool disconnected = false;
      for (unsigned int i=0; i<2; i++) {

        r0=r-1+i*(bottleneckSize+1);
        c0=std::max((int)c-1,0);
        r1=r-1+(1-i)*(bottleneckSize+1);
        c1=std::min(c+bottleneckSize,dims[1]-1);

        // cout << r0 << " " << c0 << " " << r1 << " " << c1 << endl;
        // Check that rows are within range
        if ((std::min(r0,r1)<0) || (std::max(r0,r1)>=(int)dims[0])) continue;

        // But what if the cells are obstacles... slide along the box till you have vertices that aren't
        for (;c0<=c1 && !gr->o[r0][c0];c0++);// cout << r0 << "," << c0 << " ";
        //cout << " next" << endl;
        if (c0>c1) continue;
        for (;c1>=c0 && !gr->o[r1][c1];c1--);// cout << r1 << "," << c1 << " ";
        //cout << endl;
        if (c1<c0) continue;

        // Check if the distance is less than lower limit with the block, but greater than upper limit with block removed from graph
        if (distLessThan(gr, r0, c0, r1, c1, distanceMultMin*bottleneckSize)) {
          removeBlock (gr, r, c, bottleneckSize);
          disconnected = !distLessThan(gr, r0, c0, r1, c1, distanceMultMax*bottleneckSize);
          addBlock (gr, r, c, bottleneckSize);
          if (disconnected) 
            break;
        }
      }

      if (disconnected) {
        //cout << "Disconnected block " << r << " " << c << endl;
        //printBlock (*gr, r, c, bottleneckSize, r0, c0, r1, c1, dims[0], dims[1]);
        disconnectingBlocks->push_back (Coords(r,c));
      }
      else {
        //cout << "Connected block " << r << " " << c << endl;
        //printBlock (*gr, r, c, bottleneckSize, r0, c0, r1, c1, dims[0], dims[1]);
      }

    }
  }
}



// The top-level function that returns a topological graph containing bottleneck and open regions, given an occupancy grid
BottleneckGraph makeBottleneckGraph (GridArray grid, int bottleneckSize, int bottleneckSkip, int distanceMultMin, int distanceMultMax)
{
  GridGraph gr = makeGraphFromGrid(grid);
  BottleneckGraph g;
  CoordsList disconnectingBlocks;
  typedef std::vector<Region> RegionVector;

  // Find disconnecting blocks and disconnect them from rest of graph
  findDisconnectingBlocks (&gr, &disconnectingBlocks, bottleneckSize, bottleneckSkip, distanceMultMin, distanceMultMax);
  for (CoordsList::iterator i = disconnectingBlocks.begin(); i!=disconnectingBlocks.end(); i++)
    markBottleneckCells (&gr, i->first, i->second, bottleneckSize);
  disconnectBottlenecks (&gr);


  // Compute connected components of resulting graph
  VertexCompMap vertexComp;
  boost::associative_property_map<VertexCompMap> component(vertexComp);
  int numComps = boost::connected_components (gr.g, component);
  RegionVector regions(numComps);
  for (VertexCompMap::iterator i = vertexComp.begin(); i!=vertexComp.end(); i++) 
    regions[i->second].insert(get (coords_t(), gr.g, i->first));


  // Construct the bottleneck graph
  desc_t vertexDescriptions;
  std::vector<BottleneckVertex> bottleneckGraphVertices;
  for (unsigned int i=0; i<regions.size(); i++) {
    Coords c = *(regions[i].begin());
    BottleneckVertex v = add_vertex (g);
    bottleneckGraphVertices.push_back (v);
    VertexDescription d;
    d.type = get (bottleneck_t(), gr.g, gr.m[c.first][c.second]) ? BOTTLENECK : OPEN;
    d.region = regions[i];
    boost::put (vertexDescriptions, g, v, d);
  }
  connectRegions (gr, vertexComp, bottleneckGraphVertices, &g);

  return g;
  
}

    
    


} // namespace topological_map

