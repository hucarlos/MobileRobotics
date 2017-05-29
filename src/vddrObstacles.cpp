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

#include <vddrObstacles.h>
#include <vddrTrajectories.h>
#include <CGAL/intersections.h> 

namespace vddr {
  // Admissibility, for a point
  bool vddrObstacles::isPointAdmissible(const CK::Point_2 &q) const {
    std::vector<Polygon_with_holes_2> res3;
    polygons_with_holes (std::back_inserter (res3));
    // Iterate on polygons
    for (unsigned int i=0;i<res3.size();i++) {
      // Distinguish between the inner obstacles and the holes
      if (res3[i].holes_begin()==res3[i].holes_end()) {
	// Inner obstacles
	const Polygon_2 &p = res3[i].outer_boundary(); CK::Point_2 qq(q.x(),q.y());
	if (p.bounded_side(qq) != CGAL::ON_UNBOUNDED_SIDE)
	  return false;
      } else {
	bool atleast = false;
	std::list<Polygon_2>::const_iterator lit;
	for (lit = res3[i].holes_begin();lit!=res3[i].holes_end();lit++) {
	  const Polygon_2 &p = *lit; CK::Point_2 qq(q.x(),q.y());
	  // Outer obstacles
	  if (p.bounded_side(qq)==CGAL::ON_BOUNDED_SIDE)
	    atleast = true;
	}
	if (!atleast) return false;
      }
    }
    return true;
  }


  // Check if a segment intersect the polygon set
  bool vddrObstacles::doIntersect(const Segment &s) const {
    std::list<Polygon_with_holes_2> res3;
    polygons_with_holes (std::back_inserter (res3));
    for (std::list<Polygon_with_holes_2>::const_iterator it3=res3.begin();
	 it3!=res3.end();it3++)  {
      // Distinguish between the inner obstacles and the holes
      int nholes=0;
      std::list<Polygon_2>::const_iterator lit;
      for (lit = it3->holes_begin();lit!=it3->holes_end();lit++) nholes++;
      if (!nholes) {
	// Inner obstacles
	Polygon_2 p = it3->outer_boundary();
	for (unsigned int k = 0;k<p.size();k++) {
	  CGAL::Object obj = 
	    CGAL::intersection(p.edge(k),s);
	  if (!obj.is_empty()) {
	    return true;
	  }
	}
      } else {
	for (lit = it3->holes_begin();lit!=it3->holes_end();lit++) {
	  Polygon_2 p = *lit;
	  for (unsigned int k = 0;k<p.size();k++) {
	    CGAL::Object obj = 
	      CGAL::intersection(p.edge(k),s);
	    if (!obj.is_empty()) {
	      return true;
	    }
	  }
	}
      }
    }
    return false;
  }

  // Find closest point from Cobst
  CK::Point_2 vddrObstacles::closestPointCobst(const CK::Point_2 &q) const {
  
    double dmin = std::numeric_limits<double>::max();
    Segment s;
    std::list<Polygon_with_holes_2> res3;
    polygons_with_holes (std::back_inserter (res3));
    for (std::list<Polygon_with_holes_2>::const_iterator it3=res3.begin();
	 it3!=res3.end();it3++)  {
      // Distinguish between the inner obstacles and the holes
      int nholes=0;
      std::list<Polygon_2>::const_iterator lit;
      for (lit = it3->holes_begin();lit!=it3->holes_end();lit++) nholes++;
      if (!nholes) {
	// Inner obstacles
	Polygon_2 p = it3->outer_boundary();
	for (unsigned int k=0;k<p.size();k++) {
	  double d = CGAL::squared_distance(p.edge(k),q);
	  if (d<dmin) {
	    dmin = d;
	    s = p.edge(k);
	  }
	}
      } else {
	for (lit = it3->holes_begin();lit!=it3->holes_end();lit++) {
	  Polygon_2 p = *lit;
	  // Holes
	  for (unsigned int k=0;k<p.size();k++) {
	    double d = CGAL::squared_distance(p.edge(k),q);
	    if (d<dmin) {
	      dmin = d;
	      s = p.edge(k);
	    }
	  }
	}
      }
    }
    CK::Point_2 result;
    // Now compute the exact closest point
    if (fabs(CGAL::squared_distance(s.source(),q)-dmin)<0.0001) {
      result = s.source();
    } 
    else if (fabs(CGAL::squared_distance(s.target(),q)-dmin)<0.0001) {
      result = s.target();
    }
    // Orthogonal projection
    else {
      Line l(s.source(),s.target());
      result=l.projection(q);
    }
    return result;  
  }
}
