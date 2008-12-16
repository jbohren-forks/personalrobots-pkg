#ifndef __UNDIR_FEAT_GRAPH_H__
#define __UNDIR_FEAT_GRAPH_H__

#include <vector>
#include <utility>
#include <set>
#include <ext/hash_map>
#include <ext/hash_set>

#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>

#include "features/FeatureMatrix.hh"
#include "features/SuperpixelBlobber.h"

#include "cv.h"
#include "highgui.h"

typedef int NodeId;

typedef __gnu_cxx::hash_map<int,int> Int2IntMap;

typedef __gnu_cxx::hash_set<int> NodeSet;

/*
class EdgeComparer {
public:
  bool operator()(std::pair<int,int> e0, std::pair<int,int> e1) {
    return (e0.first + e0.second) < (e1.first + e1.second);
  }
};

typedef std::set<std::pair<int,int>, EdgeComparer> EdgeSet;
*/

//using namespace std;

//using namespace boost::lambda;

/**
   @brief A graph with attached node and edge features
   @tparam FM The type of feature matrix to use
 */
template <class FM>
class UndirectedFeatureGraph {
 public:
  /**
     @param anNodes Number of nodes in the graphs
     @param aedges Vector of edges 
     @param nodeFeat Node feature matrix
     @param edgeFeat Edge feature matrix
   */
  UndirectedFeatureGraph(int anNodes,
			 const vector<pair<int,int> >& aedges,
			 FM* nodeFeat, 
			 FM* edgeFeat) :
    nNodes(anNodes),
    edges(aedges),
    nodeFeatures(nodeFeat),
    edgeFeatures(edgeFeat)
  {
  };

  ~UndirectedFeatureGraph() {
    if (nodeFeatures != NULL)
      delete nodeFeatures;
    if (edgeFeatures != NULL)
      delete edgeFeatures;
  }

  UndirectedFeatureGraph(const UndirectedFeatureGraph& copyme) :
    nNodes(copyme.nNodes),
    edges(copyme.edges)    
  {
    nodeFeatures = 
      (copyme.nodeFeatures == NULL ? NULL : copyme.nodeFeatures->copy());
    edgeFeatures = 
      (copyme.edgeFeatures == NULL ? NULL : copyme.edgeFeatures->copy());
  }

  int numNodes() const { return nNodes; }
  int numEdges() const { return edges.size(); }

  int numNodeFeatures() const { return nodeFeatures->cols(); }
  int numEdgeFeatures() const { return edgeFeatures->cols(); }

  // NB: these can be NULL
  //  const FeatureMatrix<double>* getNodeFeat() const { return nodeFeatures; }
  //  const FeatureMatrix<double>* getEdgeFeat() const { return edgeFeatures; }
  const FM* getNodeFeat() const { return nodeFeatures; }
  const FM* getEdgeFeat() const { return edgeFeatures; }

  //vector<Node>* getNodeList() { return &nodeList; }

  const std::vector<std::pair<int,int> >* 
    getEdgeList() const { return &edges; }

  const void getEdgeListCopy(std::vector<std::pair<int,int> >& vec) const { 
    for (std::vector<std::pair<int,int> >::const_iterator it = edges.begin();
	 it != edges.end();
	 it++) {
      vec.push_back(*it);
    }
  }

  /*
  void getEdgeSet(EdgeSet& eset) { 
    for (vector<std::pair<int,int> >::iterator it = edges.begin();
	 it != edges.end();
	 it++) {
      eset.insert(*it);
    }
  }
  */

  // masks out references to nodes not in the set, re-number nodes, 
  // make everything consistent with renumbering 
  // Output argument is a preallocated map mapping old indices to new indices
  void subgraph(const NodeSet& nodeSubset, Int2IntMap& old2NewInd) {
    old2NewInd.clear();

    // make index mapping
    int newIndex = 0;
    for (int oldIndex = 0; oldIndex < nNodes; oldIndex++) {
      if (nodeSubset.find(oldIndex) != nodeSubset.end()) { 
	old2NewInd[oldIndex] = newIndex;
	newIndex++;
      }
    }

    nNodes = newIndex;

    // re-index edges, remove dangling edges
    vector<pair<int, int> > newEdges;
    vector<int> goodEdgeRows;
    int edgeNum = 0;
    for (vector<pair<int, int> >::iterator it = edges.begin();
	 it != edges.end();
	 it++) {
      pair<int,int> oldEdge = *it;

      if (nodeSubset.find(oldEdge.first) != nodeSubset.end() &&
	  nodeSubset.find(oldEdge.second) != nodeSubset.end()) {
	pair<int,int> newEdge(old2NewInd[oldEdge.first],
			      old2NewInd[oldEdge.second]);
	newEdges.push_back(newEdge);
	goodEdgeRows.push_back(edgeNum);
      }
      edgeNum++;
    }

    // convert nodeset to a vector
    vector<int> goodNodeRows;
    for (NodeSet::iterator it = nodeSubset.begin(); 
	 it != nodeSubset.end(); it++) {
      goodNodeRows.push_back(*it);
    }

    FM *newNodeFeatures = nodeFeatures->getRowSet(goodNodeRows);
    delete nodeFeatures;
    nodeFeatures = newNodeFeatures;

    //    if (nodeFeatures != NULL && goodRows.size() > 0) { 
      // remove rows from feature matrices
      /*
    } else {
      int nCols = nodeFeatures->getCols();
      delete nodeFeatures;
      nodeFeatures = new FeatureMatrix(0,nCols);
    }
      */

    FM *newEdgeFeatures = edgeFeatures->getRowSet(goodEdgeRows);
    delete edgeFeatures;
    edgeFeatures = newEdgeFeatures;    

    edges = newEdges;

    //    if (edgeFeatures != NULL && goodEdgeRows.size() > 0) {
    //    if (goodEdgeRows.size() > 0) {
      // remove rows

      /*
	} else {
	edges.clear();
	edgeFeatures = NULL;
      */
      //    }
  }

  /**
     @brief Converts graph to a subgraph of itself
     @param nodeList List of nodes in the subgraph
     @param old2NewInd Output argument containing mapping from 
     old to new node indices

     After calling this, this graph will become a subgraph 
     on the nodes in the provided node list.  Other nodes 
     and edges involving other nodes will be removed, 
     along with their corresponding features.
   */
  void subgraph(const vector<int>& nodeList, Int2IntMap& old2NewInd) {
    NodeSet nodes;
    
    for (vector<int>::const_iterator it = nodeList.begin();
	 it != nodeList.end();
	 it++) {
      nodes.insert(*it);
    }

    subgraph(nodes, old2NewInd);
  }

 private:
  int nNodes;
  vector<pair<int, int> > edges;

  FM* nodeFeatures;
  FM* edgeFeatures;
};

#endif
