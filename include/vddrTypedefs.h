// Copyright (c) 2004,2005,2006  INRIA Sophia-Antipolis (France) and
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
// $URL: svn+ssh://scm.gforge.inria.fr/svn/cgal/branches/CGAL-3.3-branch/Segment_Delaunay_graph_2/demo/Segment_Delaunay_graph_2/pdg_typedefs.h $
// $Id: pdg_typedefs.h 37003 2007-03-10 16:55:12Z spion $
//
//
// Author(s)     : Menelaos Karavelas <mkaravel@cse.nd.edu>

#ifndef VDDR_TYPEDEFS_H
#define VDDR_TYPEDEFS_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Gmpq.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Segment_Delaunay_graph_hierarchy_2.h>
#include <CGAL/Segment_Delaunay_graph_traits_2.h>
#include <CGAL/Segment_Delaunay_graph_adaptation_traits_2.h>

#include <CGAL/Timer.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/basic.h>
#include <CGAL/Segment_Delaunay_graph_vertex_base_with_info_2.h>

// Polygons, polygon sets
#include <CGAL/Polygon_2.h>
#include <CGAL/General_polygon_with_holes_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polygon_set_2.h>

typedef CGAL::Tag_false      ITag;
typedef CGAL::Tag_true       STag;

// Kernels
#if 1
typedef CGAL::Exact_predicates_inexact_constructions_kernel CK;
#else
typedef CGAL::Simple_cartesian<double>     CK;
#endif

#ifdef CGAL_USE_CORE
#include <CGAL/CORE_Expr.h>
struct EK : public CGAL::Simple_cartesian<CORE::Expr> {};
#else
struct EK : public CGAL::Simple_cartesian<CGAL::Gmpq> {};
#endif
typedef CGAL::Field_with_sqrt_tag  MTag;
#ifdef CGAL_USE_CORE
typedef CGAL::Field_with_sqrt_tag  EMTag;
#else
typedef CGAL::Integral_domain_without_division_tag        EMTag;
#endif
// Delaunay and Voronoi graphs
typedef CGAL::Segment_Delaunay_graph_filtered_traits_without_intersections_2<CK,MTag,EK,EMTag> Gt;

typedef CGAL::Segment_Delaunay_graph_storage_traits_2<Gt>             ST;
typedef CGAL::Segment_Delaunay_graph_vertex_base_2<ST>                Vb;

typedef CGAL::Segment_Delaunay_graph_vertex_base_with_info_2<Vb,int>  Vbi;
typedef CGAL::Segment_Delaunay_graph_hierarchy_vertex_base_2<Vbi>     Vbh;
typedef CGAL::Triangulation_face_base_2<Gt>                           Fb;
typedef CGAL::Triangulation_data_structure_2<Vbh,Fb>                  DS;
typedef CGAL::Segment_Delaunay_graph_hierarchy_2<Gt,ST,STag,DS>   SDG_2;
typedef CGAL::Segment_Delaunay_graph_adaptation_traits_2<SDG_2> TRAIT;

typedef CGAL::Segment_Delaunay_graph_adaptation_traits_2<SDG_2> AT;
typedef CGAL::Voronoi_diagram_2<SDG_2,AT> VD;

typedef VD::Locate_result             Locate_result;
typedef VD::Vertex_handle             Vertex_handle;
typedef VD::Face_handle               Face_handle;
typedef VD::Halfedge_handle           Halfedge_handle;
typedef VD::Ccb_halfedge_circulator   Ccb_halfedge_circulator;
typedef VD::Site_iterator Site_iterator;

#define NMAX 32

namespace vddr {
  // Points and lines
  typedef CGAL::Exact_predicates_exact_constructions_kernel Ke;
  typedef CK::Point_2   Point_2;
  typedef CK::Point_3 Point_3;
  typedef CK::Segment_2 Segment;
  typedef CK::Triangle_2 Triangle;
  typedef CK::Circle_2 Circle;
  typedef CK::Line_2 Line;
  typedef Ke::Point_2 Point_2_e;
  typedef Ke::Segment_2 Segment_e;
  
  typedef CGAL::Polygon_2<CK,std::vector<CK::Point_2> > Polygon_2;
  typedef CGAL::Polygon_2<Ke,std::vector<Point_2_e> > Polygon_2_e;
  typedef CGAL::Polygon_with_holes_2<CK>  Polygon_with_holes_2;
  typedef CGAL::Polygon_with_holes_2<Ke>  Polygon_with_holes_2_e;
  typedef CGAL::Polygon_set_2<CK>         Polygon_set_2;
  typedef CGAL::Polygon_set_2<Ke>         Polygon_set_2_e;
  typedef std::list<Polygon_with_holes_2>            Pwh_list_2;
  typedef std::list<Polygon_with_holes_2_e>          Pwh_list_2_e;

  // Convert exact polygon 
  Polygon_with_holes_2 P2eToP2(const Polygon_with_holes_2_e &pe);

  // Convert exact polygon 
  Polygon_2 P2eToP2(const Polygon_2_e &pe);

  // Convert to exact polygon 
  Polygon_with_holes_2_e P2ToP2e(const Polygon_with_holes_2 &pe);
    
  // Discretization level
  static const unsigned int NB    = 80;
 
  // Robot radius.
  static const double robotRadius= 35.0;  

  // Min visibility radius.
  static const double rVisMin    = 40.0;

  // Max visibility radius.
  static const double rVisMax   = 650.0;
   
  // Angular sector corresponding to robot
  static const double angSectorRob =  2*M_PI/NB;
  
  // Chords corresponding to robot
  static const double lenSectorRob = 2.0*sin(angSectorRob*.5)*robotRadius;
  
  // Square chords corresponding to robot
  static const double lenSectorRobSq = lenSectorRob*lenSectorRob;

  // Robot radius
  const double epsilon       = 0.000000001; 

  // Maximum number of landmarks
  const int LMAX = 10;
}
#endif  // PDG_TYPEDEFS_H
