/*
 *
 * Copyright (C) 2010 Jean-Bernard Hayet
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

#ifndef VDDR_VISIBILITYAREAS_H
#define VDDR_VISIBILITYAREAS_H

#include <CGAL/basic.h>
#include <CGAL/Timer.h>
#include <visilibity.h>
#include <vddrObstacles.h>
#include <vddrGraphReduction.h>

namespace vddr {
  /**
   * A class for managing visibility zones
   */
  class vddrVisibilityArea : public vddr::Polygon_with_holes_2_e {

  protected:
    /**
     * Landmark id
     */
    unsigned int id;

    /**
     * The landmark to which this zone is relative
     */
    vddr::Point_2 landmark;

    /**
     * Inner boundary (rmin): depends on the landmark
     */
    vddr::Polygon_2 inner;  

    /**
     * Outter boundary (rmax): depends on the landmark
     */
    vddr::Polygon_2 outer;

    /**
     * Outter boundary (1.3 rmax)
     */
    vddr::Polygon_2 outerBigger;
  
    /**
     * Visibility structure
     */
    VisiLibity::Environment vEnvironment;

    /**
     * Visibility zone. Do not take obstacles into account. 
     */
    vddr::Polygon_with_holes_2_e freeSpaceNotDilated;

    /**
     * Obstacles (normal obstacles + distance limits)
     */
    vddrObstacles cobst;

    /**
     * Free space, with obstacles AND visibility
     */
    vddrObstacles fSpace;

    /**
     * Delaunay graph
     */
    SDG_2 sdg;

    /**
     * Reduced graph, per landmark
     */
    Graph *reducedGraph;
    
    /**
     * Set boundary to a visilibity environment
     * @param nboundary New boundary
     * @param e Reference to the visilibity environment
     */
    static void setBoundary(const vddr::Polygon_2&nboundary,
			    VisiLibity::Environment &e) {
      std::vector<VisiLibity::Point> out;
      VisiLibity::Polygon outerBoundaryV;
      for (vddr::Polygon_2::Vertex_const_iterator it=nboundary.vertices_begin();it!=nboundary.vertices_end();it++)
	out.push_back(VisiLibity::Point(it->x(),it->y()));
      outerBoundaryV.set_vertices(out);
      // Updates the environmente structire
      e.set_outer_boundary(outerBoundaryV);
    }

    /**
     * Angular sector corresponding to rVisMax
     */
    double angSectorMax;
  
    /**
     * Chords corresponding to rVisMax
     */
    double lenSectorMax;
  
    /**
     * Square chords corresponding to rVisMax
     */
    double lenSectorMaxSq;
 
  public:
      static const double alpha;

    /**
     * Constructor
     */
    vddrVisibilityArea(unsigned int i, const vddr::Point_2 &p);
  
    /**
     * Destructor
     */
    ~vddrVisibilityArea() {
      //if (reducedGraph)
      //delete reducedGraph;
    };

    /**
     * Build structure after update   */
    void update(const std::vector<vddr::Polygon_2> &obstaclesDilated,
		bool complete_diagram_mode);

    /**
     * Update the drawable diagram
     */
    void updateDrawDiagram(bool allTriangle);
  
    /**
     * Clear
     */
    void clear(bool complete_diagram_mode=false) {
      // Insert polygon into visibility environment structure in the cw order
      vEnvironment.clear_holes();
      std::vector<vddr::Polygon_2> empty;
      update(empty,complete_diagram_mode);
    }
    
    /**
     * update boundary when an obstacle is added
     */
    void updateOuterBoundary(const vddr::Polygon_2 &pgn,
			     const std::vector<VisiLibity::Point> &vp);

    /**
     * Check if a point is OK for this landmarl
     */
    bool isPointAdmissible(const Point_2&p) const {
      return cobst.isPointAdmissible(p);
    };
    
    /**
     * Get chord length
     */
    const double &getChord() const {
      return lenSectorMaxSq;
    }

    /**
     * The landmark to which this zone is relative
     */
    const vddr::Point_2 &getLandmark() const {
      return landmark;
    };
    
    /**
     * Get free space
     */
    const vddrObstacles &getFreeSpace() const {
      return fSpace;
    }
 
    /**
     * Get CObst
     */
    const vddrObstacles &getCobst() const {
      return cobst;
    }
 
   /**
     * Get CObst
     */
    const vddr::Polygon_with_holes_2_e &getFreeSpaceNotDilated() const {
      return freeSpaceNotDilated;
    }

   /**
     * Get outer boundary
     */
    const vddr::Polygon_2 &getOuter() const {
      return outer;
    }
  
    /**
     * Get inner boundary
     */
    const vddr::Polygon_2 &getInner() const {
      return inner;
    }
    
    /**
     * Get reduced graph
     */
    Graph *getGraph() {
      return reducedGraph;
    }

    /**
     * Get reduced graph const
     */
    const Graph *getGraphConst() const {
      return reducedGraph;
    }

   /**
     * Get delaunay graph
     * @return The delaunay graph
     */
    const SDG_2 &getDelaunay() const {
      return sdg;
    }

    /**
     * Get environment
     */
     VisiLibity::Environment &getVEnvironment() {
      return vEnvironment;
    }

    /**
     * Set outer boundary
     */
    void setOuter(vddr::Polygon_2 &p) {
      outer = p;
    }
  };
}
#endif
