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

#ifndef VDDR_COMMONVIS_H
#define VDDR_COMMONVIS_H

#include <CGAL/Timer.h>
#include <vddrTypedefs.h>
#include <vddrObstacles.h>
#include <vddrConnector.h>
#include <vddrVisibilityArea.h>

namespace vddr {

  /**
   * Structure that manages common visibility zones
   */
  class vddrCommonVis {
  
  protected:  
    /**
     * Common visibility zones (by pairs of landmarks)
     */
    vddr::Polygon_2 commonVisPolygons[LMAX][LMAX];

    /**
     * Chords of arcs of circles in each connector
     */
    std::vector<double> commonVisChords[LMAX][LMAX];

    /**
     * Structures that holds connectivity information
     * between two landmarks roadmaps
     */
    std::vector<vddrConnector *> connectorAreas;
  
    /**
     * Verbose flag
     */
    bool verbose;
      
  public:

    /**
     * Sensor view angle
     */
    static double viewAngle;

    /**
     * Constructor
     */
    vddrCommonVis();

    /**
     * Destructor
     */  
    ~vddrCommonVis();

    /**
     * Insertion of a new landmark
     * @param l Reference to the landmark position
     */
    bool addLandmark(const vddr::Point_2 &l);

    /**
     * Build connectors  */
    void updateConnectors(const std::vector<vddr::Polygon_2> &obstaclesDilated,
			  std::vector<vddrVisibilityArea> &landmarkAreas,
			  int landm=-1);

    /**
     * Build common visibility areas  */
    void update(const std::vector<vddrVisibilityArea> &landmarkAreas,
		int landm=-1);
    
    /**
     * Set verbose
     * @param Verbose parameters
     */
    inline void setVerbose(bool b) {
      verbose = b;
    }
  
    /**
     * Get verbose
     * @return Verbose parameter
     */
    inline bool getVerbose() const {
      return verbose;
    }
  
    /**
     * Sensor view angle
     */
    void setViewAngle(const double &v) {
      viewAngle = v;
    }
  
    /**
     * Get connectors
     * @return connectors
     */
    inline const std::vector<vddrConnector*> &getConnectors() const {
      return connectorAreas;
    }
  
 };
}
#endif
