#include "vddrTypedefs.h"

namespace vddr {

  // Convert exact polygon 
  Polygon_with_holes_2 P2eToP2(const Polygon_with_holes_2_e &pe) {
    Polygon_2 outerE;
    for (Polygon_2_e::Vertex_const_iterator it=pe.outer_boundary().vertices_begin();
	 it!=pe.outer_boundary().vertices_end();
	 it++)
      outerE.push_back(Point_2(to_double(it->x()),to_double(it->y())));
    std::list<Polygon_2> holes;
    for (Polygon_with_holes_2_e::Hole_const_iterator hit = pe.holes_begin();
	 hit!= pe.holes_end();
	 hit++) {
      Polygon_2  h;
      for (Polygon_2_e::Vertex_const_iterator it=hit->vertices_begin();
	   it!=hit->vertices_end();
	   it++)
	h.push_back(Point_2(to_double(it->x()),to_double(it->y())));
      holes.push_back(h);
    }
    Polygon_with_holes_2 p(outerE,holes.begin(),holes.end());
    return p;
  }

  // Convert exact polygon 
  Polygon_2 P2eToP2(const Polygon_2_e &pe) {
    Polygon_2 p; 
    for (Polygon_2_e::Vertex_const_iterator it=pe.vertices_begin();
	 it!=pe.vertices_end();
	 it++)
      p.push_back(Point_2(to_double(it->x()),to_double(it->y())));
    return p;
  }

  // Convert to exact polygon 
  Polygon_with_holes_2_e P2ToP2e(const Polygon_with_holes_2 &pe) {
    Polygon_2_e outerE;
    for (Polygon_2::Vertex_const_iterator it=pe.outer_boundary().vertices_begin();
	 it!=pe.outer_boundary().vertices_end();
	 it++)
      outerE.push_back(Point_2_e(it->x(),it->y()));
    std::vector<Polygon_2_e> holes;
    for (Polygon_with_holes_2::Hole_const_iterator hit = pe.holes_begin();
	 hit!= pe.holes_end();
	 hit++) {
      Polygon_2_e  h;
      for (Polygon_2::Vertex_const_iterator it=hit->vertices_begin();
	   it!=hit->vertices_end();
	   it++)
	h.push_back(Point_2_e(it->x(),it->y()));
      holes.push_back(h);
    }
    Polygon_with_holes_2_e p(outerE,holes.begin(),holes.end());
    return p;
  }
}
