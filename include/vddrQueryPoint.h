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

#ifndef VDDR_QUERYPOINT_H
#define VDDR_QUERYPOINT_H

#include <iostream>
#include <vddrObstacles.h>

#undef DEBUG

namespace vddr {
  
  /**
   * Class for handling query points
   */
  class queryPoint {
  
  public:
    
    /**
     * Visible landmarks from this point
     */
    std::vector<unsigned int> visLandmarks;
    
    /**
     * Point
     */
    Point_2 p;
    
    /**
     * Constructor with point
     */
  queryPoint(const Point_2 &p) : p(p) {};
    
    /**
     * Constructor with point and landmarks
     */
  queryPoint(const Point_2 &p,
	     const std::vector<unsigned int> &v) : visLandmarks(v),p(p) {
    };
  
    /**
     * Default constructor
     */
    queryPoint() {
    };
    
  queryPoint(const queryPoint &q) : visLandmarks(q.visLandmarks), p(q.p) {
    };
    
    queryPoint& operator=(const queryPoint& q) {
      p            = q.p;
      visLandmarks = q.visLandmarks;
      return *this;
    };
    
  };
}
#endif
