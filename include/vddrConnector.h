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

#ifndef VDDR_CONNECTOR_H
#define VDDR_CONNECTOR_H

#include <CGAL/basic.h>
#include <CGAL/Bbox_2.h> 
#include "visilibity.h"
#include <vddrTypedefs.h>
#include "vddrGraphReduction.h"

namespace vddr {
  /**
   * A class for managing connector zones
   */
  class vddrConnector : public Polygon_with_holes_2_e {
  
    /**
     * Landmark 1
     */
    unsigned int from;

    /**
     * Landmark 2
     */
    unsigned int to;
  
    /**
     * Locations
     */
    std::vector<Point_2_e> nodes;

    /**
     * Its own delaunay graph
     */
    SDG_2 delaunay;
    
    /**
     * Reduced graph 
     */
    Graph *reducedGraph;

    /**
     * Chords lengths (to reduce the graph adequately)
     */
    std::vector<double> chords;

  public:
  
    /**
     * Constructor
     */
  vddrConnector(const Polygon_with_holes_2_e &p, 
		unsigned int i,
		unsigned int j,
		const std::vector<double> &chords) : Polygon_with_holes_2_e(p),
      from(i),to(j),chords(chords) {
      reducedGraph = new Graph();
    };
  
    /**
     * Destructor
     */
    ~vddrConnector() {
      std::cerr << "*** Destructing vddrConnector" << std::endl;
      if (reducedGraph)
	delete reducedGraph;
    }

    /**
     * Build a representation of the connector area connectivity
     */
    void buildConnectivity();

    /**
     * Test if a point is inside this connector
     * @param q The point to test
     * @return true if the point is inside the connector
     */
    bool isInside(const Point_2_e &q) const {
      // Take boundary
      Polygon_2_e p = this->outer_boundary(); Point_2_e qq(q.x(),q.y());
      CGAL::Bounded_side bside   = p.bounded_side(qq);
      if (bside != CGAL::ON_BOUNDED_SIDE)
	return false;
      // Check if inside hole
      std::list<Polygon_2_e>::const_iterator lit;
      for (lit = this->holes_begin();lit!=this->holes_end();lit++) {
	Polygon_2_e p = *lit; Point_2_e qq(q.x(),q.y());
	// Outer obstacles
	CGAL::Bounded_side bside   = p.bounded_side(qq);
	if (bside==CGAL::ON_BOUNDED_SIDE)
	  return false;
      }
      return true;
    };

    /**
     * Get complete delaunay graph
     * @return The delaunay graph
     */
    const SDG_2 &getDelaunay() const {
      return delaunay;
    }

    /**
     * Get reduced graph
     * @return The reduced delaunay graph
     */
    Graph *getGraph() {
      return reducedGraph;
    }

    /**
     * Accesor to to
     * @return The index of the second landmark 
     */
    const unsigned int &getTo() const {
      return to;
    }

    /**
     * Accesor to from
     * @return The index of the first landmark 
     */
    const unsigned int &getFrom() const {
      return from;
    }

    /**
     * Accesor to node
     * @return A node sampled inside the connector area
     */
    const std::vector<Point_2_e> &getNodes() const {
      return nodes;
    }
  };
}
#endif
