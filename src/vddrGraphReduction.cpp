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

#include <vddrGraphReduction.h>

#undef DEBUG

namespace vddr {
  double Graph::clearanceFactor = 0.0;

  // Graph node
  Graph::Edge::Edge(const unsigned int &nodeid, 
		    const std::vector<lPoint> &it,
		    const double &clearance,
		    const Graph::Edge* t,
		    const double &penalty) : 
    id(nodeid),
    next(const_cast<Edge*>(t)),
    clearance(clearance),
    forwardRatio(1.0),
    intermediatePoints(it) {
    weight=penalty;
    // Add the total length along the Voronoi arc
    if (intermediatePoints.size()>1)
      for (unsigned int k=0;k<intermediatePoints.size()-1;k++)
	weight+= 
	  sqrt(CGAL::squared_distance(intermediatePoints.at(k).location,
				      intermediatePoints.at(k+1).location));
  }

  // Default constructor
  Graph::Graph() : Acnt(0)  {}

  // Copy constructor
  Graph::Graph(const Graph &other) {
    copy(other);
  }

  // Destructor
  Graph::~Graph() {
    clear();
  }

  // Copy method
  void Graph::copy(const Graph &other) {
    vertices = other.vertices;
    Acnt     = other.Acnt;
    locations= other.locations;
    // Clone edges too
    adj.assign(vertices.size(),NULL);
    for (unsigned int k=0;k<vertices.size();k++) {
      for (const Edge *n = other.getVertexEdges(k); n!= NULL; n=n->next)
	adj[k] = new Edge(n->id,n->intermediatePoints,n->clearance,adj[k]);
    }
  }

  // Clear method
  void Graph::clear() {
    for (unsigned int j=0;j<adj.size();j++) {
      while (adj[j]!=NULL) {
	const Edge *tmp = adj[j]; 
	adj[j] = adj[j]->next;
	delete tmp;
      }
    }
    adj.clear();
    vertices.clear();
    locations.clear();
  }


  // Insertion of a whole graph
  void Graph::insert(const Graph &g) {
    // Size of the graph at the beginning
    unsigned int s = vertices.size();
    // Insert all vertices
    for (unsigned int i=0;i<g.vertices.size();i++) {
      insert(g.vertices.at(i));
      for (const Edge *n= g.getVertexEdges(i);n!=NULL;n=n->next) {
	adj.at(adj.size()-1) = new Edge(s+n->id,n->intermediatePoints,
					n->clearance,adj.at(adj.size()-1));
	Acnt++;
      }
    }
  }

  // Insertion of a node (location p)
  void Graph::insert(const lPoint &p) { 
    // Insert if not present
    if (locations.find(p)==locations.end()) {
      vertices.push_back(p);
      adj.push_back(NULL);
      // Insert into map 
      std::pair<lPoint,unsigned int> toInsert;
      toInsert.first = p; toInsert.second = adj.size()-1;
      locations.insert(toInsert);
    }
  }

  // Insertion of an oriented edge
  void Graph::insert(const lPoint &p,
		     const lPoint &q, 
		     const std::vector<lPoint> &intPoints,
		     const double &clearance,
		     const double &penalty) { 
    insert(p);
    insert(q);
    adj.at(locations[p]) = new Edge(locations[q],intPoints,clearance,
				    adj.at(locations[p]),penalty);
    Acnt++;
  }

    // Insert at vertex l, on the node that goes to pend, between k and k+1
    void Graph::insertAt(const lPoint &p,
                         const lPoint &pv,
                         int l,
                         const int k,
                         Edge *e) {
        
        
        // Add two nodes, one for p, one for pv
        insert(p);
        insert(pv);
        
        // Check the edge
        Edge *temp = adj[l];
        for (; temp!=NULL; temp=temp->next)
        {
            bool gotit = false;
            if (temp == e)
            {
                // We got it !
                gotit = true;
#ifdef DEBUG
                std::cerr << "Edge to modify found" << std::endl;
                std::cerr << vertices.at(l).location << std::endl;
                std::cerr << vertices.at(e->id).location << std::endl;
#endif
                break;
            }
        }
        // Node indices
        int iv   = locations[pv];
        int ip   = locations[p];
        int iend = e->id;
        
        // Add edges p<->pv
        const double bigClearance = 1000.0;
        std::vector<lPoint> v1; v1.push_back(p);  v1.push_back(pv);
        std::vector<lPoint> v2; v2.push_back(pv); v2.push_back(p);
        adj[ip]  = new Edge(iv,v1,bigClearance,adj[ip]);
        adj[iv]  = new Edge(ip,v2,bigClearance,adj[iv]);
        Acnt+=2;
        
        // Add edges pi <-> pv
        std::vector<lPoint> vi1(k+2);
        for (int kk=0;kk<=k;kk++) {
            vi1.at(kk)  = temp->intermediatePoints.at(kk);
        }
        vi1.at(k+1) = pv;
        std::vector<lPoint> vi2(vi1.rbegin(),vi1.rend());
        adj[l]      = new Edge(iv,vi1,bigClearance,adj[l]);
        adj[iv]     = new Edge(l ,vi2,bigClearance,adj[iv]);
        Acnt+=2;
        
        // Add pf <-> pv
        int nInt = temp->intermediatePoints.size();
        std::vector<lPoint> vf1(nInt-k);
        for (int kk=k+1;kk<nInt;kk++)
        {
            
            //        int garbage = kk-k-1;
            //        std::cout<<garbage<<std::endl;
            vf1.at(kk-k-1)  = temp->intermediatePoints.at(nInt+k-kk);
        }
        vf1.at(nInt-k-1)  = pv;
        std::vector<lPoint> vf2(vf1.rbegin(),vf1.rend());
        adj[iend]   = new Edge(iv  ,vf1,bigClearance,adj[iend]);
        adj[iv]     = new Edge(iend,vf2,bigClearance,adj[iv]);
        Acnt+=2;
        
        // Remove edges pi<->pf
//        std::cout<<temp->intermediatePoints.size()<<std::endl;
//        removeEdge(iend,l,temp->intermediatePoints.at(nInt/2));
//        std::cout<<temp->intermediatePoints.size()<<std::endl;
//        removeEdge(l,iend,temp->intermediatePoints.at(nInt/2));
//        std::cout<<temp->intermediatePoints.size()<<std::endl;
        
        vddr::lPoint ppoint = temp->intermediatePoints.at(nInt/2);
        removeEdge(iend,l,ppoint);
        removeEdge(l,iend,ppoint);
        Acnt-=2;  
    } 


  // Find closest point on the graph 
  lPoint Graph::closestPointGraph(const lPoint &p,
				  const lPoint &pp,
				  const vddrObstacles &cobst,
				  int &iStart,
				  int &kInt,
				  Graph::Edge *&e) {
  
    double dminp = std::numeric_limits<double>::max();
    Line lp(p.location,pp.location);
    lPoint pt;
    for (unsigned int l=0;l<V();l++) {
      for (const Graph::Edge *n=getVertexEdges(l);n!= NULL;n=n->next) {
	for (unsigned int k=0;k<n->intermediatePoints.size()-1;k++) {
	  Point_2 ip; Segment curEdge(n->intermediatePoints.at(k).location,
				      n->intermediatePoints.at(k+1).location);
	  // Check if the line (p,pp) intersect with this segment
	  CGAL::Object obj = 
	    CGAL::intersection(lp,curEdge.supporting_line());
	  if (CGAL::assign(ip, obj) && curEdge.collinear_has_on(ip)) {
	    Segment s(p.location,ip); 
	    if (!cobst.doIntersect(s)) {
	      double d = CGAL::squared_distance(ip,p.location);
	      if (d<dminp) {
		dminp   = d;
		iStart  = l;
		kInt    = k;
		e       = const_cast<Graph::Edge *>(n);
		pt      = lPoint(ip,n->intermediatePoints.at(0).lId);
	      } 
	    }
	  }
	}
      }
    }
    return pt;
  }

  // Remove edge p-q passing through pint (to raise ambiguities)
  void Graph::removeEdge(const unsigned int &p,
			 const unsigned int &q,
			 const lPoint &pint) {
    Edge *temp  = adj[p];
    if (!temp) {
      std::cerr << "Vertex from which the edge is to remove not found !!!" << std::endl;
      return;
    }
    Edge *prev = NULL;
    for (; temp!=NULL; prev=temp,temp=temp->next) {
      bool gotit = false;
      if (temp->id == q) {
	for (unsigned k=0;k<temp->intermediatePoints.size();k++) { 
	  if (temp->intermediatePoints.at(k)==pint) {
	    // We got it !
	    gotit = true;
	    break;
	  }
	}
      }
      if (gotit)
	break;
    }
  
    if (!temp) {  // Not found
#ifdef DEBUG
      std::cerr << "Edge to remove not found !!!" << std::endl;
#endif
      return;
    }
    if (prev) 
      prev->next = temp->next;
    else 
      adj[p]  = temp->next;
    if (temp) {
#ifdef DEBUG
      std::cerr << "Removing edge from " << vertices.at(p).location
		<< " to " << vertices.at(q).location
		<< " passing by " << pint.location << std::endl;
      for (Edge *tmp  = adj[p]; tmp!=NULL;tmp=tmp->next) {
	std::cerr << vertices.at(tmp->id).location<< std::endl;
      }
#endif
      delete temp;
    }
  }

  // Dijsktra: shortest paths from p
  std::vector<Graph::Edge *> Graph::shortestPaths(const lPoint &p) {
    std::vector<Edge *> spt(V(),NULL);
    if (locations.find(p)==locations.end()){
      std::cerr << "Start point is not a graph node " << std::endl;
      return spt;
    }
 
    // Index of start point
    unsigned int l = locations[p];
#ifdef DEBUG
    std::cerr << "(sp) start node " << l << std::endl; 
#endif
    // This vector will hold the shortest path lengths
    std::vector<double> wt(V(),std::numeric_limits<double>::max());wt[l] = 0.0;
    // This vector will hold the path (link to other node)
    // Priority queue to manage best paths so far
    std::priority_queue<std::pair<unsigned int,double>,
      std::deque<std::pair<unsigned int,double> >,compPair> pq;
  // Shortest path from l to l is zero
  pq.push(std::pair<unsigned int,double>(l,wt[l]));
  while (!pq.empty()) {
    unsigned int v = pq.top().first; pq.pop(); // wt[v] = 0.0;
#ifdef DEBUG
    std::cerr << "(sp) node " << v << " best path so far " << wt[v] 
	      << " neighbours " << (adj[v]==NULL) << std::endl; 
#endif
    if (v != l && spt[v] == NULL) return spt;  
    for (Edge* e = adj[v]; e!=NULL; e = e->next){ 
#ifdef DEBUG
      std::cerr << "(sp) neighbour node " << e->id << std::endl;
#endif
      int w    = e->id; 
#ifdef DEBUG
      std::cerr << e->forwardRatio << std::endl;
#endif
      double P = wt[v] + std::max(0.1,1.0-e->forwardRatio)* 
	e->weight/(1.0+clearanceFactor*sqrt(e->clearance));
      if (P < wt[w]) {
	wt[w]  = P; pq.push(std::pair<unsigned int,double>(w,P)); 
	spt[w] = e; 
      }
    }
  }
  return spt;
  }

  // Build "reduced" graph for one particular landmark
  void Graph::build(const VD& sdg,const vddrObstacles &cobst,unsigned int lid,bool neo) {
    // Reference to the complete voronoi diagram
    const VD &vd=sdg;
    
    // Clear graph
    if (neo)
      clear();

    // Iterator to the vertices
    VD::Vertex_iterator vit = vd.vertices_begin();
    
    // Among all the vertices, we select the vertices that have degree 1 or 3
    // Starts at an arbitrary Voronoi vertex
    for (vit = vd.vertices_begin(); vit != vd.vertices_end(); vit++) {
      // Examine all incident edges and go along until
      // the next vertex of degree 1 or 3
      VD::Halfedge_around_vertex_circulator
	cit = vit->incident_halfedges();
      // Compute "true" vertex degree
      int nRealEdges=computeRealDegree(*vit,cobst); 
      if (nRealEdges%2==1) {
	// Scan all incident edges
	for (unsigned int k = 0; k < vit->degree(); k++, cit++) 
	  if (cit->has_source()) {
	    // Check if cit is a real edge
	      if (isEdgeReal(cit,cobst)) {
		// Intermediate points
		std::vector<lPoint> it; it.push_back(lPoint(vit->point(),lid));
		// Real edge: vertex point vit + source of cit
		Point_2 prev                = vit->point();
		VD::Vertex_handle w = cit->source(); 
		// Degree of w
		int r = computeRealDegree(*w,cobst);
		// Clearance min
		double clearanceMin = std::numeric_limits<double>::max();
		// Goes along
		while (r==2) {
		  // Compute clearance along cit
		  double clearance = computeClearance(prev,w->point(),cit);
		  if (clearance<clearanceMin) 
		    clearanceMin = clearance;
		  // Add w to the list of intermediate points for this edge
		  it.push_back(lPoint(w->point(),lid));
		  // Look for all edges around w
		  VD::Halfedge_around_vertex_circulator
		    citr = w->incident_halfedges(); 
		  // Go along them
		  for (unsigned int l = 0; l < w->degree(); l++, ++citr) 
		    if (citr->has_source()) {
		      // Search an unknown edge
		      if (isEdgeReal(citr,cobst) && 
			  citr->source()->point()!=prev) 
			{
			  prev= w->point();
			  w   = citr->source();
			  r   = computeRealDegree(*w,cobst);
			  break;
			}
		    }
		}
		if (r==1 || r==3) {
		  it.push_back(lPoint(w->point(),lid));
#ifdef DEBUG
		  std::cerr << "EDGE " << vit->point() 
			    << " " << w->point() 
			    << ", clearance " 
			    << clearanceMin << ", "
			    << it.size() << std::endl;
		  std::cerr << "info " << cit->up()->info() 
			    << " " << cit->down()->info() << std::endl;
#endif
		  insert(lPoint(vit->point(),lid),
			 lPoint(w->point(),lid),it,clearanceMin);
		}
	      }
	  }
      }
    }
	
    // Case of a NULL graph (at the beginning) : circular case
    if (V()==0) {
      std::cerr << "*** Circular Voronoi" << std::endl;
      // In that case, pick two of the vertices
      // Get a first "real" edge
      VD::Edge_iterator eit = vd.edges_begin();
      while (eit!=vd.edges_end() && !isEdgeReal(eit,cobst)) {
	eit++;
      }
      if (eit==vd.edges_end()) { 
	std::cerr << "*** Circular Voronoi, failure " << std::endl;
	throw std::runtime_error("Circular Voronoi, failure");
      }
      VD::Vertex_handle p = eit->source();
      VD::Halfedge_around_vertex_circulator
	cit = p->incident_halfedges();
      Point_2 prev                = p->point();
      std::vector<lPoint> it; it.push_back(lPoint(p->point(),lid));
      std::cerr << "*** Circular Voronoi, starting " << std::endl;
      // For all the incident edges from vertex p
      for (unsigned int k = 0; k < p->degree(); k++,cit++) 
	if (cit->has_source()) {
	    if (isEdgeReal(cit,cobst))
	      {
		VD::Vertex_handle w = cit->source(); 
		// Start adding all consecutive points
		while (w->point() != p->point()) {
		  it.push_back(lPoint(w->point(),lid));
		  VD::Halfedge_around_vertex_circulator
		    citr = w->incident_halfedges(); 
		  for (unsigned int l = 0; l < w->degree(); l++, ++citr) 
		    if (citr->has_source()) {
		      // Search the unknown edge
			if (isEdgeReal(citr,cobst) && 
			    citr->source()->point()!=prev)
			  {
			    prev= w->point();
			    w   = citr->source();
			    break;
			  }
		    }
		}
		it.push_back(lPoint(w->point(),lid));
		break;
	      }
	}
      insert(it.at(0),it.at(it.size()-1),it);
      std::cerr << "*** Circular Voronoi (done)" << std::endl;
    }
    // Go over the resulting graph 
  }
	
// Compute degree of the vertex
int Graph::computeRealDegree(VD::Vertex &v,
			     const vddrObstacles &cobst) {
  // Examine all incident edges starting from the vertex 
  VD::Halfedge_around_vertex_circulator
    cit = v.incident_halfedges(); 
  int nRealEdges=0;
  for (unsigned int k = 0; k < v.degree(); k++, cit++) {
    // Count the real edges
    if (isEdgeReal(cit,cobst)) {
      nRealEdges++;
    }
  }
  return nRealEdges;
}


// Test if an edge e is a "real" one
template <class E>
bool Graph::isEdgeReal(E &e,
		       const vddrObstacles &cobst) {
  if (e->is_segment() && 
      e->has_source() && 
      e->up()->info() != e->down()->info()) {
    if (cobst.isPointAdmissible(e->source()->point()) ||
	cobst.isPointAdmissible(e->target()->point())) {
      return true;
    }
  }
  return false;
}

// Compute clearance
double Graph::computeClearance(const Point_2 &pi,const Point_2 &pf,
			       VD::Halfedge_around_vertex_circulator &e) {
  if (e->has_source() && e->is_segment() && 
      e->up()->info() != e->down()->info()) {
    const VD::Site_2 &p = e->up()->site();
    Point_2 mid(0.5*(pi.x()+pf.x()),
		0.5*(pi.y()+pf.y()));
    if (p.is_segment()) {
      return squared_distance(mid,p.segment());
    } else if (p.is_point()) {
      return squared_distance(mid,p.point());
    }
  }
  return std::numeric_limits<double>::max();
}

}
