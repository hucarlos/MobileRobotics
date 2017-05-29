// Copyright (c) 2003,2004,2006  INRIA Sophia-Antipolis (France) and
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
// $URL: svn+ssh://scm.gforge.inria.fr/svn/cgal/branches/CGAL-3.3-branch/Segment_Delaunay_graph_2/demo/Segment_Delaunay_graph_2/include/CGAL/Segment_Delaunay_graph_vertex_base_with_info_2.h $
// $Id: Segment_Delaunay_graph_vertex_base_with_info_2.h 37003 2007-03-10 16:55:12Z spion $
//
//
// Author(s)     : Menelaos Karavelas <mkaravel@cse.nd.edu>


#ifndef CGAL_SEGMENT_DELAUNAY_GRAPH_VERTEX_BASE_WITH_INFO_2_H
#define CGAL_SEGMENT_DELAUNAY_GRAPH_VERTEX_BASE_WITH_INFO_2_H

#include <CGAL/Segment_Delaunay_graph_2/basic.h>

#include <CGAL/Triangulation_ds_vertex_base_2.h>
#include <CGAL/Segment_Delaunay_graph_storage_site_2.h>
#include <CGAL/Segment_Delaunay_graph_simple_storage_site_2.h>

CGAL_BEGIN_NAMESPACE


template < class Vbb, class Info_ >
class Segment_Delaunay_graph_vertex_base_with_info_2
  : public Vbb
{
public:
  // TYPES
  //------
  typedef Vbb                                      Base;
  typedef Info_                                    Info;
  template < typename DS2 >
  struct Rebind_TDS {
    typedef typename Vbb::template Rebind_TDS<DS2>::Other             Vb2;
    typedef Segment_Delaunay_graph_vertex_base_with_info_2<Vb2,Info> Other;
  };


  Segment_Delaunay_graph_vertex_base_with_info_2 () : Vbb(), info_() {}

  Segment_Delaunay_graph_vertex_base_with_info_2(const typename Base::Storage_site_2& ss,
						 typename Base::Data_structure::Face_handle f)
    : Vbb(ss, f), info_()  {}

  void set_info(const Info& info) { info_ = info; }
  const Info& info() const { return info_; }

private:
  Info info_;
};


CGAL_END_NAMESPACE

#endif // CGAL_SEGMENT_DELAUNAY_GRAPH_VERTEX_BASE_WITH_INFO_2_H
