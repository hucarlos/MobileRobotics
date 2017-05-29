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


#ifndef VDDR_DELAUNAY_INSERT_H
#define VDDR_DELAUNAY_INSERT_H

#include <qthreadstorage.h> 
#include <vddrTypedefs.h> 

namespace vddr {

  static QThreadStorage<int*> generator;

  void initGenerator();

  int get_new_id();
  

  template<class PDG, class Point>
    typename PDG::Vertex_handle
    insert_point(PDG& pdg, const Point& p);
  
  template<class PDG, class Point>
    typename PDG::Vertex_handle
    insert_segment(PDG& pdg, const Point& p1, const vddr::Point_2& p2, int id);
  
  template<class PDG, class Point>
    typename PDG::Vertex_handle
    insert_segment(PDG& pdg, const Point& p1, const vddr::Point_2& p2,
		   typename PDG::Vertex_handle v, int id);

  template<class PDG, class Point>
    typename PDG::Vertex_handle
    insert_segment(PDG& pdg, const Point& p1, const Point& p2);
  
  template<class PDG, class Point>
    typename PDG::Vertex_handle
    insert_segment(PDG& pdg, const Point& p1, const Point& p2,
		   typename PDG::Vertex_handle v);

  template<class PDG, class Polygon>
    typename PDG::Vertex_handle
    insert_polygon(PDG& pdg, const Polygon& pgn, int id=-1);
  
  // Insert triangle
  template<class PDG>
    void insert_triangle(PDG& pdg, 
			 const vddr::Point_2 &p1,
			 const vddr::Point_2 &p2,
			 const vddr::Polygon_2& p,
			 int id);
    
  template<class PDG>
    void insert_polygon_with_holes(PDG& pdg, 
				   const vddr::Polygon_with_holes_2& pgn,
				   const std::vector<double> &seclengths,
				   bool complete);
}

#endif // VDDR_DELAUNAY_INSERT_H
