/*
 *
 * Copyright (C) 2009 Jean-Bernard Hayet
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef VDDR_GRAPH_REDUCTION_H
#define VDDR_GRAPH_REDUCTION_H

#include <CGAL/basic.h>

#include <queue>      
#include <deque>      
#include <set>      
#include <map>      
#include <vddrTypedefs.h>
#include <vddrObstacles.h>

class trajectory;

namespace vddr {
  /**
   * Point with id
   */
  struct lPoint {
    /**
     * Spatial location
     */
    vddr::Point_2 location;
    
    /**
     * Landmark id to which this node is 
     * relative to
     */
    unsigned int lId;  
    
    /**
     * Equality operator
     * @param o Other lpoint to check
     * @return True if equal
     */
    inline bool operator==(const lPoint &o) const {
      return (location.x()==o.location.x() && 
	      location.y()==o.location.y() &&
	      lId == o.lId);
    };
    
    /**
     * Inequality operator
     * @param o Other lpoint to check
     * @return True if non-equal
     */
    inline bool operator!=(const lPoint &o) const {
      return (location.x()!=o.location.x() ||
	    location.y()!=o.location.y() ||
	      lId != o.lId);
    };
    
    /**
     * Default constructor
     */
    lPoint() {};
    
    /**
     * Constructor from point and landmark id
     * @param p Location
     * @param id Landmark id
     */
  lPoint(const Point_2 &p,const unsigned int &id) : location(p),lId(id) {};
  };
  
  
  /**
   * A template class for the reduced graph we will use
   */
  class Graph { 
  public:  
  /**
   * Edge structure: serves for edges
   */
  struct Edge {    
    /*
     * Node index
     */
    unsigned int id;
    
    /**
     * Next node
     */
    Edge* next;

    /**
     * Weight
     */
    double weight;

    /**
     * Clearance
     */
    double clearance;

    /**
     * Forward proportion: when needed this factor represent the part of the edge that would have
     * to be done forwards.
     */
    double forwardRatio;

    /**
     * Intermediate points
     */
    std::vector<lPoint> intermediatePoints;

    /**
     * Constructor
     * @param nid  The id of the point this edge is pointing to
     * @param it The set of intermediate points that make the arc
     * @param clearance A clearance factor computed (externally) along the edge
     * @param t Pointer to the next edge sharing the same departing node
     * @param penalty An additive penalty term for those edges that we really want to avoid (e.g., landmark switches)
     */
    Edge(const unsigned int &nid, 
	 const std::vector<lPoint> &it,
	 const double &clearance,
	 const Edge* t,
	 const double &penalty=0.0);
  };
  
  struct compPair {
    /**
     * A comparator for pairs vertex,shortest path so far
     * @param s1 First pair of points to compare
     * @param s2 Second pair of points to compare
     * @return Result of the comparison (compare second pair)
     */
    inline bool operator()(std::pair<unsigned int,double> &s1,
			   std::pair<unsigned int,double> &s2) const {
      return (s1.second < s2.second);
    }
  };

  /**
   * A comparator for points
   * @param s1 First point to compare
   * @param s2 Second point to compare
   * @return Result of the comparison (lexicographic order)
   */
  struct compPoint {
    inline bool operator()(const lPoint& s1,
			   const lPoint& s2) const {
      if (s1.location.x()==s2.location.x()) { 
	if (s1.location.y()==s2.location.y()) {
	  return (s1.lId<s2.lId);
	}
	return (s1.location.y()<s2.location.y());
      }
      return (s1.location.x()<s2.location.x());
    }
  };

  /**
   * A comparator for pairs of points
   */
  struct compPairPoint {
    bool operator()(const std::pair<Point_2,Point_2>& s1, 
		    const std::pair<Point_2,Point_2>& s2) const {
      if (s1.first.x()==s2.first.x()) {
	if (s1.first.y()==s2.first.y()) {
	  if (s1.second.x()==s2.second.x()) {
	    return (s1.second.y()<s2.second.y());
	  }
	  return (s1.second.x()<s2.second.x());	
	}
	return (s1.first.y()<s2.first.y());
      }
      return (s1.first.x()<s2.first.x());
    }
  };
  
  /**
   * Clearance factor
   */
  static double clearanceFactor;

  /**
   * Print graph
   */
  inline void print() {
    for (unsigned int i=0;i<vertices.size();i++) {
      std::cerr << i << " -- " << vertices.at(i).location << " ";
      for (Edge *e=adj[i];e!=NULL;e=e->next)
	std::cerr << e->id << "-";
      std::cerr << std::endl;
    }
    std::cerr << std::endl;
  }

 private:
  
  /**
   * Set for half-edges
   */
  std::set<std::pair<Point_2,Point_2>,compPairPoint> realHalfEdges;

  
  /**
   * Edges leaving the nodes
   */
  std::vector <Edge*> adj;  
  
  /**
   * Vertices
   */
  std::vector <lPoint> vertices;

  /**
   * Number of edges
   */
  unsigned int Acnt;
  
  /**
   * A map for storing pairs locations/landmarks
   * and their index
   */
  std::map<lPoint,unsigned int,compPoint> locations;

  /**
   * Test if an edge e is a "real" one
   */
  template<class E>
    bool isEdgeReal(E &e,const vddrObstacles &cobst);

  /**
   * Utility
   */
  int computeRealDegree(VD::Vertex &v,
			const vddrObstacles &cobst);

  /**
   * Compute clearance on the GVG arc
   * @param pi Reference to the first point 
   * @param pf Reference to the second point 
   * @param e The GVG edge
   * @return Computed clearance (distance from middle point [pi,pf] to corresponding site)
   */
  double computeClearance(const Point_2 &pi,
			  const Point_2 &pf,
			  VD::Halfedge_around_vertex_circulator &e);
  
 public:
  /**
   * Default constructor
   */
 Graph();
 
 /**
  * Copy constructor
  * @param other Another graph to copy this one from
  */
  Graph(const Graph &other);
  
  /**
   * Destructor
   */
  ~Graph();

  /**
   * Copy function
   * @param other Another graph to copy 
   */
  void copy(const Graph &other);
  
  /**
   * Number of vertices
   * @return The number of verticles
   */
  inline unsigned int V() const { return adj.size(); }
  
  /**
   * Number of edges
   * @return The number of edges
   */
  inline unsigned int A() const { return Acnt; }
  
  /**
   * Clear graph
   */
  void clear();
  
  /**
   * Insert a whole graph
   * @param g A graph to insert in this one
   */
  void insert(const Graph& g);
  
  /**
   * Insert node
   * @param p A node to insert in this one
   */
  void insert(const lPoint &p);
  
  /**
   * Insert oriented edge (the other one will follow)
   * @param p Starting point
   * @param q Ending point
   * @param intPoints The points that form the arc
   * @param clearance A clearance factor
   * @param clearance An additive penalty term for edge weight
   */
  void insert(const lPoint &p,
	      const lPoint &q, 
	      const std::vector<lPoint> &intPoints,
	      const double &clearance = 1.0,
	      const double &penalty   = 0.0);
  
  /**
   * Insert at vertex l, on the node that goes to pend, between k and k+1
   */
  void insertAt(const lPoint &p,
		const lPoint &pv,
		int l,
		int k,
		Edge *e);

 /**
  * Find closest point on the graph 
  */
  lPoint closestPointGraph(const lPoint &p,
			   const lPoint &pp,
			   const vddrObstacles &cobst,
			   int &iStart,
			   int &kInt,
			   Edge *&e);

  /**
   * Remove edge p-q passing through point
   */
  void  removeEdge(const unsigned int &p,
		   const unsigned int &q,
		   const lPoint &pint);
  
  /**
   * Get vertex
   */
  inline const lPoint &getVertex(int i) const {
    return vertices[i];
  }

  /**
   * Accesor
   */
  inline const Edge* getVertexEdges(int i) const {
    return adj[i];
  }
  
  /**
   * Get index of point q.
   * @param q lPoint tp locate in the graph. 
   * @return Index corresponding to the point, 
   *         -1 if the point does not belong to the graph.
   */
  inline int getIndex(const lPoint &q) {
    if (locations.find(q)!=locations.end()) 
      return locations[q];
    return -1;
  }

  /**
   * Get indices of nodes at location q, given
   * a maximum number of landmarks.
   * @param q Location.
   * @param nLmax Maximum number of landmarks.
   * @return Vector of indices corresponding to the nodes the location corresponds to.
   */
  inline std::vector<int> getIndex(const Point_2 &q,unsigned int nLMax) {
    std::vector<int> v;
    std::map<lPoint,int,compPoint>::iterator it;
    for (unsigned int k=0;k<nLMax;k++) {
      lPoint test = lPoint(q,k);
      if (locations.find(test)!=locations.end()) {
	v.push_back(locations[test]);
      }
    }
    return v;
  }

  /**
   * Dijsktra: shortest paths from l
   * @param p lPoint serving as starting point. 
   * @return Vector of shortest paths from each node.
   */
  std::vector<Edge *> shortestPaths(const lPoint &p);


  /**
   * Build "reduced" graph
   */
  void build(const VD& sdg,
	     const vddrObstacles &cobst,
	     unsigned int lid,
	     bool neo=true); 
  
  friend class trajectory;
  };
}
#endif
