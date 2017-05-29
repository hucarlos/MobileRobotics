// Copyright (c) 2004,2005  INRIA Sophia-Antipolis (France) and
// Notre Dame University (U.S.A.).  All rights reserved.
//
// This file is part of CGAL (www.cgal.org); you may redistribute it under
// the terms of the Q Public License version 1.0.
// See the file LICENSE.QPL distributed with CGAL.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL: svn+ssh://scm.gforge.inria.fr/svn/cgal/branches/CGAL-3.3-branch/Segment_Delaunay_graph_2/demo/Segment_Delaunay_graph_2/include/pdg_insert.h $
// $Id: pdg_insert.h 37003 2007-03-10 16:55:12Z spion $
//
//
// Author(s)     : Menelaos Karavelas <mkaravel@cse.nd.edu>


#ifndef PDG_INSERT_H
#define PDG_INSERT_H

#include <vddrDelaunayInsert.h> 

namespace vddr {

  void initGenerator() {
    if (generator.hasLocalData()) return;
    int *a = new int(0);
    generator.setLocalData(a);
  }
  
  int get_new_id()
  {
    assert(generator.hasLocalData());
    int *gen = generator.localData();
    *gen = *gen + 1;
    assert( *gen != 0 ); // guards against unwanted phenomena due to
    // overflow
    return *gen;
  }
  
  
  template<class PDG, class Point>
  typename PDG::Vertex_handle
  insert_point(PDG& pdg, const Point& p)
  {
    typename PDG::Vertex_handle v = pdg.insert(p);
    v->set_info( get_new_id() );
    return v;
  }

  template<class PDG, class Point>
  typename PDG::Vertex_handle
  insert_segment(PDG& pdg, const Point& p1, const vddr::Point_2& p2, int id)
  {
    typedef typename PDG::Vertex_handle Vertex_handle;
#ifdef DEBUG
    std::cerr << "inserting " << p1 << " " << p2 << std::endl; 
#endif
  
    try {
      Vertex_handle v1 = pdg.insert(p1);
      Vertex_handle v2 = pdg.insert(p2);
      Vertex_handle v3 = pdg.insert(p1,p2);

      if ( v3 == Vertex_handle() ) {
      return v3;
      }
      v1->set_info( id );
      v2->set_info( id );
      v3->set_info( id );
      return v3;
    } catch (...) {
      std::cerr << "### Warning: exception not processed at " << __FILE__ 
		<< " line " << __LINE__  << std::endl;
    }
    return Vertex_handle();
  }

  template<class PDG, class Point>
  typename PDG::Vertex_handle
  insert_segment(PDG& pdg, const Point& p1, const vddr::Point_2& p2,
		 typename PDG::Vertex_handle v, int id)
  {
    typedef typename PDG::Vertex_handle Vertex_handle;
    Vertex_handle v1 = pdg.insert(p1, v);
    Vertex_handle v2 = pdg.insert(p2, v2);
    Vertex_handle v3 = pdg.insert(p1, p2, v1);
    
    if ( v3 == Vertex_handle() ) {
      return v3;
    }
    
    v1->set_info( id );
    v2->set_info( id );
    v3->set_info( id );
    return v3;
  }


  template<class PDG, class Point>
  typename PDG::Vertex_handle
  insert_segment(PDG& pdg, const Point& p1, const Point& p2)
  {
    int id = get_new_id();
    return insert_segment(pdg, p1, p2, id);
  }
  
  template<class PDG, class Point>
  typename PDG::Vertex_handle
  insert_segment(PDG& pdg, const Point& p1, const Point& p2,
		 typename PDG::Vertex_handle v)
  {
    int id = get_new_id();
    return insert_segment(pdg, p1, p2, v, id);
  }
  

  template<class PDG, class Polygon>
  typename PDG::Vertex_handle
  insert_polygon(PDG& pdg, const Polygon& pgn, int id)
  {
    typedef typename PDG::Vertex_handle Vertex_handle;
    if (id==-1)
      id = get_new_id();
    Vertex_handle v;
    int psize = pgn.size();
    for (int i = 0; i < psize; i++ ) {
      v = insert_segment( pdg, pgn[i], pgn[(i+1)%psize], id );
      if ( v == Vertex_handle() ) { break; }
    }
    return v;
  }
  
  
  // Insert triangle
  template<class PDG>
  void insert_triangle(PDG& pdg, 
		       const vddr::Point_2 &p1,
		       const vddr::Point_2 &p2,
		       const vddr::Polygon_2& p,
		       int id) {
    
    // Form a triangle with the segment as a base
    vddr::Polygon_2 q;
    q.push_back(p1); q.push_back(p2);
    // Third point:
    double ny = (p1.x()-p2.x());
    double nx =-(p1.y()-p2.y());
    double nor= sqrt(nx*nx+ny*ny);
    nx/=nor;ny/=nor;
    vddr::Point_2 m(0.5*(p1.x()+p2.x())+0.02*nx,
		    0.5*(p1.y()+p2.y())+0.02*ny);
    vddr::Point_2 n(0.5*(p1.x()+p2.x())-0.02*nx,
		    0.5*(p1.y()+p2.y())-0.02*ny); 
#if DEBUG
    std::cerr << "triangle "<< std::endl;
    std::cerr << p1 << " " << p2 << std::endl;
    std::cerr << m << " " << n << std::endl;
#endif
    CGAL::Bounded_side bside   = p.bounded_side(m);
    switch (bside) {
    case CGAL::ON_BOUNDED_SIDE:
      q.push_back(n);
      break;
    case CGAL::ON_BOUNDARY:
    case CGAL::ON_UNBOUNDED_SIDE:
      q.push_back(m);
      break;
    }
    insert_polygon( pdg, q, id);      
  }
  template<class PDG>
  void
  insert_polygon_with_holes(PDG& pdg, 
			    const vddr::Polygon_with_holes_2& pgn,
			    const std::vector<double> &seclengths,
			    bool complete)
  {
    typedef typename PDG::Vertex_handle Vertex_handle;
    // Count holes
    int nholes=0;
    std::list<vddr::Polygon_2>::const_iterator lit;
    for (lit = pgn.holes_begin();lit!=pgn.holes_end();lit++) nholes++;
    // If no hole, put it as is
    if (!nholes) {
#if DEBUG
      std::cerr << "No hole " << std::endl;
#endif
      std::vector<int> partition;partition.push_back(0);
      int psize = pgn.outer_boundary().size();
      const vddr::Polygon_2 &p = pgn.outer_boundary();
      // Assign ids
      for (unsigned int i = 1; i < p.size();i++) {
	// Check distances
	double d2=CGAL::squared_distance(p[i],p[(i-1)         ]);
	double d1=CGAL::squared_distance(p[i],p[(i+1)%p.size()]);
	// This is to detect one of the input circle
	for (unsigned int l=0;l<seclengths.size();l++) {
	  if (fabs(d2-seclengths[l])<0.00001 &&
	      fabs(d1-seclengths[l])<0.00001) {
	    continue;
	  }
	}
	// Check angle : this is to remove circles/lines transitions
	double alpha1 = atan2(p[i].y()-p[i-1].y(),
			      p[i].x()-p[i-1].x());
	double alpha2 = atan2(p[(i+1)%p.size()].y()-p[i].y(),
			      p[(i+1)%p.size()].x()-p[i].x());
	if (fabs(sin(alpha2-alpha1))<0.1) {
	  continue;
	}
	partition.push_back(i);
      }  
#if DEBUG
      std::cerr << "Partition size " << partition.size() << std::endl; 
#endif
      int firstid =  get_new_id();
      for (unsigned int j=0;j<partition.size()-1;j++) {
	int id = get_new_id();
	if (j==0) firstid = id;
	for (int i = partition[j];i<= partition[j+1];i++) {
	  insert_segment( pdg, p[i], p[(i+1)%psize], id );
	}
      }
      // Case of the last partition
      int id = get_new_id();
      if (partition[partition.size()-1]==(int)p.size()-1)
	for (unsigned int i = partition[partition.size()-1];i<p.size();i++) {
	  insert_segment( pdg, p[i], p[(i+1)%psize], id );
	}
      else {
	for (unsigned int i = partition[partition.size()-1];i<p.size();i++) {
	  insert_segment( pdg, p[i], p[(i+1)%psize], firstid );
	}
      }
    } else {
#if DEBUG
      std::cerr << "Hole " << std::endl;
#endif
      // much more complicated : if this is the hole, we will
      // form and introduce a set of triangles
      for (lit = pgn.holes_begin();lit!=pgn.holes_end();lit++) {
	vddr::Polygon_2 p = *lit;
	
	if (!complete) {
	  // Structure for the partition
	  // Partition the polygon into several pieces:
	  // - arcs of circles of radius rmax
	  // - arcs of circles of radius robotRadius
	  // - other
	  std::vector<int> partition;partition.push_back(0);
	  for (unsigned int i = 1; i <= p.size();i++) {
	    // Check distances
	    double d2=CGAL::squared_distance(p[i%p.size()],p[(i-1)         ]);
	    double d1=CGAL::squared_distance(p[i%p.size()],p[(i+1)%p.size()]);
#ifdef DEBUG
	    std::cerr << p[i-1] << " " << p[i] << std::endl;
#endif

	    // This is to detect one of the input circle
	    bool b(false);
	    for (unsigned int l=0;l<seclengths.size();l++) {
	      if (fabs(d2-seclengths[l])<0.00001 &&
		  fabs(d1-seclengths[l])<0.00001) {
#ifdef DEBUG
		std::cerr << "Detected chord " << l << " " << seclengths[l] << std::endl;
#endif
		b = true; break;
	      }
	    }
	    if (b) continue;
	    // Check angle : this is to remove circles/lines transitions
	    double alpha1 = atan2(p[i%p.size()].y()-p[i-1].y(),
				  p[i%p.size()].x()-p[i-1].x());
	    double alpha2 = atan2(p[(i+1)%p.size()].y()-p[i%p.size()].y(),
				  p[(i+1)%p.size()].x()-p[i%p.size()].x());
	    if (fabs(sin(alpha2-alpha1))<0.09) {
#ifdef DEBUG
	      std::cerr << "Continuity ?" << std::endl;
#endif
	      continue;
	    }
	    partition.push_back(i);
#ifdef DEBUG
	    std::cerr << "New partition" << std::endl;
#endif
	  }
#ifdef DEBUG
	  std::cerr << "Partition size " << partition.size() << std::endl;
#endif
	  int firstid = 0;
	  for (unsigned int j=0;j<partition.size()-1;j++) {
	    int id = get_new_id();
	    if (j==0) firstid = id;
	    for (int i = partition[j];i<= partition[j+1];i++) {
#ifdef DEBUG
	      std::cerr << "Inserting triangle " << id 
			<< " " << p[i] << " " << p[(i+1)%p.size()] << std::endl;
#endif
	      insert_triangle(pdg,p[i],p[(i+1)%p.size()],p,id);		  
	    }
	  }
#ifdef DEBUG
	std::cerr << "Last partition " << std::endl;
#endif
	  // Case of the last partition
	  int id = get_new_id();
	  if (partition[partition.size()-1]==(int)p.size()-1)
	    for (unsigned int i = partition[partition.size()-1];i<p.size();i++) {
	      insert_triangle(pdg,p[i],p[(i+1)%p.size()],p,id);	
	    }
	  else
	    for (unsigned int i = partition[partition.size()-1];i<p.size();i++) {
	      insert_triangle(pdg,p[i],p[(i+1)%p.size()],p,firstid);		  
	    }
	} else {
	  for (unsigned int i = 0; i < p.size();i++) {
	    int id = get_new_id();
	    insert_triangle(pdg,p[i],p[(i+1)%p.size()],p,id);		  
	  }
	}
#ifdef DEBUG
	std::cerr << "Triangles inserted " << std::endl;
#endif

      }
    }
  }


  template
  void
  insert_polygon_with_holes(SDG_2& pdg, 
			    const vddr::Polygon_with_holes_2& pgn,
			    const std::vector<double> &seclengths,
			    bool complete);
}
#endif // PDG_INSERT_H
