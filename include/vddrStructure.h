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

#ifndef VDDR_STRUCTURE_H
#define VDDR_STRUCTURE_H

#include <CGAL/basic.h>
#include <CGAL/Timer.h>

#include <vddrTypedefs.h>
#include <vddrTrajectories.h>
#include <vddrHumanTrajectories.h>
#include <vddrObstacles.h>
#include <vddrConnector.h>
#include <vddrQueryPoint.h>
#include <vddrVisibilityArea.h>
#include <vddrCommonVis.h>

#include <QLine>

namespace vddr {

  /**
   * Main structure we use in this application
   */

  class vddrStructure {
  public:  

    /**
     * Enum for kind of mechanism we intend to use
     */
    enum mechanismType {
      DDR,
      Human
    };

    /**
     * Polygons to be drawn (does not take part of the planning computations)
     */
    std::vector<Polygon_2> cPolygons;

    /**
     * Colors of the polygons to be drawn
     */
    std::vector<QColor> cPolygonsColor;
  
    /**
     * Robot shape
     */
    Polygon_2 robot;
  
    /**
     * Set of obstacles
     */
    std::vector<Polygon_2> obstacles;

    /**
     * Set of dilated obstacles
     */
    std::vector<Polygon_2> obstaclesDilated;

    /**
     * Landmarks and all their relative data (Voronoi graphs..)
     */
    std::vector<vddrVisibilityArea> landmarks;

    /**
     * This object manages common visibility zones (by pairs of landmarks)
     * and connectors.
     */
    vddrCommonVis *commonVis;
 
    /**
     * Current trajectory
     */
    trajectory *nhPath;
 
    /**
     * Whole graph
     */
    Graph *wholeGraph;

    /**
     * Drawn graph
     */
    std::vector<std::vector<Segment> > drawnGraph;

    /**
     * Inheritance from Parabol to get p1 and p2
     */
    class myParabola : public CGAL::Parabola_segment_2<Gt> {
    public:
    myParabola(const CGAL::Parabola_segment_2<Gt> &p) :  CGAL::Parabola_segment_2<Gt>(p) {};
      inline const CGAL::Parabola_2<Gt>::Point_2 &getP1() {return p1;}
      inline const CGAL::Parabola_2<Gt>::Point_2 &getP2() {return p2;}
    };

    /**
     * Drawn graph
     */
    std::vector<std::vector<myParabola> > drawnGraphParabola;

    /**
     * Shortest holonomic path
     */
    std::vector<Graph::Edge *> sP;

    /**
     * Best landmark to start with
     */
    unsigned int lstart;

    /**
     * Best landmark to end with
     */
    unsigned int lend;

    /**
     * Clear current query
     */
    void clearQuery();

    /**
     * Is shortest path ready ?
     */
    bool shortestPath;

    /**
     * Is start point already chosen ?
     */
    bool startOn;

    /**
     * Do we use optmization ?
     */
    bool useOptimization;

    /**
     * Do we use forward/backward ratio in weights
     */
    bool useFwdRatio;

    /**
     * Do we perform optmization ?
     */
    bool performOptimization;
  
  private:
    /**
     * Mechanism type
     */
    mechanismType mechanism;

    /**
     * Start point
     */
    queryPoint start;

    /**
     * End point
     */
    queryPoint end;
  
    /**
     * Verbose flag
     */
    bool verbose;
  
  public:
  
    /**
     * Sensor view angle
     */
    inline void setViewAngle(const double &v) {
      commonVis->setViewAngle(v);
    }
 
    /**
     * Set Phimax
     */
    inline void setPhiMax(const double &val) {
      trajectory::phi2  = val;
      trajectory::tphi2 = tan(val);
      std::cerr << trajectory::phi2 << std::endl;
    }

    /**
     * Set Phimin
     */
    inline void setPhiMin(const double &val) {
      trajectory::phi1  = val;
      trajectory::tphi1 = tan(val);
      std::cerr << trajectory::phi1 << std::endl;
    }  

    /**
     * Constructor
     */
    vddrStructure();
 
    /**
     * Destructor
     */
    ~vddrStructure();
 
    /**
     * Insertion of a new landmark
     * @param l Reference to the landmark position
     */
    bool addLandmark(const Point_2 &l);

    /**
     * Insertion of an obstacle
     */
    void insertObstacle(const std::list<Point_2> &pgn, bool complete_diagram_mode);

    /**
     * Insertion of an obstacle
     */
    void insertPolygon(const Polygon_2 &cpgn,unsigned int j1,unsigned int j2,bool b);

    /**
     * Build structure after update   */
    void update(bool complete_diagram_mode, int landm=-1);

    /**
     * Update the drawable diagram
     */
    void updateDrawDiagram(bool allTriangle);
  
    /**
     * Process a query
     */
    bool query(const queryPoint& start,const queryPoint& end);

    /**
     *
     */
    inline const std::vector<vddrConnector*> &getConnectors() const {
      return commonVis->getConnectors();
    }

    /**
     * Set start point
     * @param Start point
     */
    void setStart(const queryPoint&s);
  
    /**
     * Set end point
     * @param End point
     */
    inline void setEnd(const queryPoint&e) {
      end    = e;
    }
    
    /**
     * Get start point
     * @return Start point
     */
    inline const queryPoint &getStart() const {
      return start;
    }

    /**
     * Get end point
     * @return End point
     */
    inline const queryPoint &getEnd() const {
      return end;
    }
  
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
     * Set mechanism
     * @param Mechanism type
     */
    inline void setMechanism(mechanismType type) {
      mechanism = type;
    }

    /**
     * Get mechanism
     * @return Mechanism type
     */
    inline mechanismType getMechanism() const {
      return mechanism;
    }

    /**
     * Set use lateral motion flag
     */
    inline void setUseLateralMotion(bool checked) {
      trajectory::useHolonomic = checked;
    }

    /**
     * Draw partition
     */
    inline void drawPartition(QPainter *p, 
			      QRectF bRect) {
      double dmin       = std::numeric_limits<double>::max();
      unsigned int imin = 0;
      // Determine the closest landmark
      for (unsigned int i=0;i<landmarks.size();i++) {
	double d = 
	  (landmarks.at(i).getLandmark().x()-start.p.x())*(landmarks.at(i).getLandmark().x()-start.p.x())+
	  (landmarks.at(i).getLandmark().y()-start.p.y())*(landmarks.at(i).getLandmark().y()-start.p.y());
	if (d<dmin) { dmin = d; imin = i;}
      }

      if (mechanism==Human) {
	humanTrajectory::drawPartition(start.p,landmarks.at(imin).getLandmark(),p,bRect);
      }
      else {
	trajectory::drawPartition(start.p,landmarks.at(imin).getLandmark(),p,bRect);
      }
    }

    /**
     * Save grid
     */
    inline void saveOrientationGrid(const QString &s) {
      double dmin       = std::numeric_limits<double>::max();
      unsigned int imin = 0;
      // Determine the closest landmark
      for (unsigned int i=0;i<landmarks.size();i++) {
	double d = 
	  (landmarks.at(i).getLandmark().x()-start.p.x())*(landmarks.at(i).getLandmark().x()-start.p.x())+
	  (landmarks.at(i).getLandmark().y()-start.p.y())*(landmarks.at(i).getLandmark().y()-start.p.y());
	if (d<dmin) { dmin = d; imin = i;}
      } 
      trajectory::saveOrientationGrid(s,start.p,landmarks.at(imin).getLandmark());
    }

    /**
     * Draw grid
     */
    inline void drawOrientationGrid(QPainter *p, 
				    QRectF bRect) {
      double dmin       = std::numeric_limits<double>::max();
      unsigned int imin = 0;
      // Determine the closest landmark
      for (unsigned int i=0;i<landmarks.size();i++) {
	double d = 
	  (landmarks.at(i).getLandmark().x()-start.p.x())*(landmarks.at(i).getLandmark().x()-start.p.x())+
	  (landmarks.at(i).getLandmark().y()-start.p.y())*(landmarks.at(i).getLandmark().y()-start.p.y());
	if (d<dmin) { dmin = d; imin = i;}
      }
      trajectory::drawOrientationGrid(start.p,landmarks.at(imin).getLandmark(),p,bRect);
    }

    /**
     * Set optimization flag
     */
    inline void setOptimization(bool b) {
      performOptimization = b;
    }

    /**
     * Set optimization use flag
     */
    inline void setUseOptimization(bool b) {
      useOptimization = b;
    }

    /**
     * Set forward/backward ratio flag
     */
    inline void setFwdRatio(bool b) {
      useFwdRatio = b;
    }
  
    /**
     * Update connectors and common visibility zones
     */
    inline void updateConnectors(int landm) {
      commonVis->update(landmarks,landm);
      commonVis->updateConnectors(obstaclesDilated,landmarks,landm);
    }

    /**
     * Save obstacles
     * @param fileName The file name
     */
    void saveObstacles(const QString &);

    /**
     * Save environment
     * @param fileName The file name
     */
    void saveEnvironment(const QString &);
  
    /**
     * Load obstacles
     * @param fileName The file name
     */
    void loadObstacles(const QString &fileName,
		       bool complete_diagram_mode);
    /**
     * Load environment : obstacles and landmarks
     * @param fileName The file name
     */
    int loadEnvironment(const QString &fileName,
			bool complete_diagram_mode);

    /**
     * Clear
     */
    void clear(bool complete_diagram_mode=false);

    /**
     * Clear all, inluding landmarks
     */
    void clearAll(bool complete_diagram_mode=false);

  };

}
#endif
