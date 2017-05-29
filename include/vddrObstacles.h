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

#ifndef VDDR_OBSTACLES_H
#define VDDR_OBSTACLES_H

#include <CGAL/basic.h>
#include <vddrTypedefs.h>

namespace vddr {
  
  /**
   * A class for managing obstacles
   */
  class vddrObstacles : public vddr::Polygon_set_2 {
  public:
    /**
     * Default constructor
     */
    vddrObstacles() : vddr::Polygon_set_2() {
    }

    /**
     * Constructor from a single polygon
     */
    vddrObstacles(const vddr::Polygon_with_holes_2&p) : vddr::Polygon_set_2(p) {
    }

    /**
     * Check if the point is admissible
     */
    bool isPointAdmissible(const vddr::Point_2 &q) const;

    /**
     * Check if the point is admissible
     */
    inline bool isPointAdmissible(const vddr::Point_3 &q) const {
      return isPointAdmissible(vddr::Point_2(q.x(),q.y()));
    };

    /**
     *  Check if a segment intersect the polygon set
     */
    bool doIntersect(const vddr::Segment &s) const;
    
    /**
     * Find closest point from Cobst  
     */
    vddr::Point_2 closestPointCobst(const vddr::Point_2 &q) const;
  };
}
#endif
