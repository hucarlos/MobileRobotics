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
 * Lesser General Public License for more details.a

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

// CGAL headers
#include <CGAL/Boolean_set_operations_2.h> 
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/minkowski_sum_2.h>

// Local headers
#include "vddrStructure.h"

// Qt headers
#include <QLine>
#include <QMutex>
extern QMutex voronoiMutex;
extern QMutex visibilityMutex;
extern QMutex obstaclesMutex;
extern QMutex cobstMutex;
extern QMutex reducedGraphMutex;
extern QMutex hPathMutex;
extern QMutex nhPathMutex;

namespace vddr {

  // Constructor
  vddrStructure::vddrStructure() : commonVis(NULL),nhPath(NULL),wholeGraph(NULL),
				   shortestPath(false),startOn(false),
				   useOptimization(true),useFwdRatio(false),performOptimization(true),
				   mechanism(vddrStructure::DDR),verbose(false) {
    // Initialize list of landmarks
    landmarks.clear();
    cobstMutex.lock();
    landmarks.push_back(vddrVisibilityArea(0,vddr::Point_2(0,0)));
    cobstMutex.unlock();
    // The robot shape
    for (unsigned int i = 0; i<NB;i++) {
      double theta = 2*M_PI*double(i)/NB+M_PI;        
      // Robot boundaries
      robot.push_back(Point_2(robotRadius*cos(theta),
			      robotRadius*sin(theta)));
    }

    // Allocate commonVis object that will manage common visibility 
    commonVis = new vddrCommonVis();
    
    std::vector<Segment> ls;
    std::vector<myParabola> lp;
    voronoiMutex.lock();
    drawnGraph.push_back(ls);
    drawnGraphParabola.push_back(lp);
    voronoiMutex.unlock();

    // Whole graph combining all landmarks
    wholeGraph = new Graph();
  }

  // Destructor
  vddrStructure::~vddrStructure() {
    std::cerr << "*** Deleting vddrStructure " << std::endl;
    delete commonVis;
    delete wholeGraph;
    delete nhPath;
  }

  // Build Delaunay graph and visibility structures
  void vddrStructure::update(bool complete_diagram_mode,int landm) {
    CGAL::Timer timer;  
    // Cycles on the landmarks
    for (unsigned int j=0;j<landmarks.size();j++) 
      if (landm<0 || static_cast<int>(j)==landm) {
	// Update all the data relative to each landmark
	landmarks[j].update(obstaclesDilated,complete_diagram_mode);
	// Update draw diagram
	timer.start();
	updateDrawDiagram(complete_diagram_mode);
	timer.stop();
	std::cerr << "*** Precomputation of the graph to display " 
		  << timer.time() << std::endl;
	std::cerr << std::endl;
      }
    // Update the connectors information
    commonVis->updateConnectors(obstaclesDilated,landmarks,landm);
    std::cerr << "*** Update done " << std::endl;
  }

  // Build Delaunay graph and visibility structures
  void vddrStructure::insertObstacle(const std::list<Point_2> &pgn,
				     bool complete_diagram_mode) {
    // Remove existing non holonomic path, if any
    nhPathMutex.lock();
    if (nhPath) {
      delete nhPath;
      nhPath = NULL;
    }
    nhPathMutex.unlock();
    Polygon_2 cpgn;   int s=0;
    for (std::list<Point_2>::const_iterator it =  pgn.begin(); 
	 s < static_cast<int>(pgn.size())-1;
	 it++,s++)
      cpgn.push_back(Point_2(it->x(),it->y()));
    insertPolygon(cpgn,0,landmarks.size(),true);
  }

  // Insert polygon pgn for the environments from landmarks j1 to j2
  void vddrStructure::insertPolygon(const Polygon_2 &pgn,
				    unsigned int j1,unsigned int j2, bool newp) {
    std::cerr << "*** Dilation and addition to the set of obstacles" << std::endl;
    // True if new obstacle
    if (newp) {
      obstaclesMutex.lock();
      // Insert polygon in list of obstacles
      obstacles.push_back(pgn);
      // Dilate the polygon by the shape of the robot
      Polygon_with_holes_2  offset = minkowski_sum_2(pgn, robot);
      // Insert corresponding dilated polygon in list of dilated obstacles
      obstaclesDilated.push_back(offset.outer_boundary());
      obstaclesMutex.unlock();
    }
    std::cerr << "*** Test cw order for vertices" << std::endl;
    // For the polygon to be inserted into visibility environment structure, it must be in cw order
    std::vector<VisiLibity::Point> vp; 
    double xmin=std::numeric_limits<double>::max();
    int imin=0;
    for (unsigned int i=0;i<pgn.size();i++) {
      if (pgn[i].x()<xmin) { imin=i; xmin=pgn[i].x();}
    }
    vp.push_back(VisiLibity::Point(pgn[imin].x(),pgn[imin].y()));
    for (int i=imin-1;i>=0;i--) {
      vp.push_back(VisiLibity::Point(pgn[i].x(),
				     pgn[i].y()));
    }
    for (int i=pgn.size()-1;i>imin;i--) {
      vp.push_back(VisiLibity::Point(pgn[i].x(),
				     pgn[i].y()));
    }
    std::cerr << "*** Test outer boundaries" << std::endl;
    // Test if cgpn intersects with outer boundary and makes the necessary updates
    for (unsigned int j=j1;j<j2;j++) {
      landmarks[j].updateOuterBoundary(pgn,vp);
    }
  }

  // Update draw diagram
  void vddrStructure::updateDrawDiagram(bool allTriangles) {
    std::vector<std::vector<Segment> > drawnGraphTmp(drawnGraph.size());
    std::vector<std::vector<myParabola> > 
      drawnGraphParabolaTmp(drawnGraphParabola.size());
    for (unsigned int j=0;j<landmarks.size();j++) {
      // The Voronoi
      VD vd(landmarks[j].getDelaunay());
      drawnGraphTmp[j].clear();
      drawnGraphParabolaTmp[j].clear();
      for (VD::Edge_iterator vit = vd.edges_begin();vit != vd.edges_end();++vit) {
	if (vit->is_segment()) {
	  if (vit->up()->info() != vit->down()->info()) {
	    if (landmarks[j].isPointAdmissible(vit->source()->point()) ||

		landmarks[j].isPointAdmissible(vit->target()->point())) {
	      vddr::Segment           s;
	      CGAL::Object o = landmarks[j].getDelaunay().primal(vit->dual());
	      if (CGAL::assign(s, o)) 
		drawnGraphTmp[j].push_back(s);
	      else {
		CGAL::Parabola_segment_2<Gt>              ps;
		if (CGAL::assign(ps, o))  
		  drawnGraphParabolaTmp[j].push_back(ps);
	      }
	    }
	  }
	}
      }
    }
    voronoiMutex.lock();
    drawnGraph         = drawnGraphTmp;
    drawnGraphParabola = drawnGraphParabolaTmp;
    voronoiMutex.unlock();
  }
  // Process a query
  bool vddrStructure::query(const queryPoint& s,const queryPoint& e) { 
    lPoint startToObst,startToVor;
    lPoint endToObst,endToVor;
    trajectory *tmp;
    // Reinitialize holonomic and non-holonomic paths
    hPathMutex.lock();
    start = s;
    end   = e;
    sP.clear();
    hPathMutex.unlock();
    switch (mechanism) {
    case DDR:
      // Compute admissible path
      tmp = trajectory::computeAdmissiblePath(nhPath,start,end,
					      commonVis->getConnectors(),
					      landmarks,
					      startToObst,endToObst,
					      startToVor,endToVor,
					      wholeGraph,sP,lstart,lend,
					      performOptimization,
					      useOptimization,useFwdRatio,verbose);
      nhPathMutex.lock();
      if (nhPath)
	delete nhPath;
      nhPath = tmp;
      nhPathMutex.unlock();
      break;
    case Human:
      // Compute admissible path
      tmp = humanTrajectory::computeAdmissiblePath(nhPath,start,end,
						   commonVis->getConnectors(),
						   landmarks,
						   startToObst,endToObst,
						   startToVor,endToVor,
						   wholeGraph,sP,lstart,lend,
						   performOptimization,
						   useOptimization,useFwdRatio,verbose);
      nhPathMutex.lock();
      if (nhPath)
	delete nhPath;
      nhPath = tmp;
      nhPathMutex.unlock();
      break;
    }
    shortestPath = true;
    startOn      = false;
    if (tmp==NULL)
      return false;
    return true;
  }

  // Add a landmark
  bool vddrStructure::addLandmark(const Point_2 &l) {
    cobstMutex.lock();
    reducedGraphMutex.lock();
    // Add a new vddrVisibilityArea object to the list
    std::cerr << "Adding landmark " << landmarks.size() << std::endl;
    landmarks.push_back(vddrVisibilityArea(landmarks.size(),l));
    reducedGraphMutex.unlock();
    cobstMutex.unlock();
    
    // Insert polygons already present in the environment
    unsigned int j=landmarks.size()-1;
    for (std::vector<Polygon_2>::const_iterator obsIt=obstacles.begin();
	 obsIt!=obstacles.end();
	 obsIt++)
      insertPolygon(*obsIt,j,j+1,false);
    
    // Common visibility
    commonVis->update(landmarks,j);

    // Add graph structures
    std::vector<Segment> ls;
    std::vector<myParabola> lp;
    voronoiMutex.lock();
    drawnGraph.push_back(ls);
    drawnGraphParabola.push_back(lp);
    voronoiMutex.unlock();
    std::cerr << "Landmark " << landmarks.size()-1 << " added." << std::endl;
    return true;
  }
  
  // Save obstacles
  void vddrStructure::saveObstacles(const QString &fileName) {

    if (!fileName.isNull() ) {
      std::ofstream f;
      f.open(fileName.toStdString().c_str(),std::ofstream::out);
      assert(f);
      f << obstacles.size() << std::endl;    
      for (std::vector<Polygon_2>::const_iterator it=obstacles.begin();
	   it!=obstacles.end();it++)
	f << *it << std::endl;
      f.close();
    }
  }

  // Save the whole environment (obstacles and landmarks)
  void vddrStructure::saveEnvironment(const QString &fileName) {

    if (!fileName.isNull() ) {
      std::ofstream f;
      f.open(fileName.toStdString().c_str(),std::ofstream::out);
      assert(f);
      // First, save the landmarks
      f << "LANDMARKS" << std::endl;    
      f << landmarks.size() << std::endl;    
      for (unsigned int i=0;i<landmarks.size();i++)
	f << landmarks.at(i).getLandmark() << std::endl;
      
      // Second, save the obstacles
      f << "OBSTACLES" << std::endl;    
      f << obstacles.size() << std::endl;    
      for (std::vector<Polygon_2>::const_iterator it=obstacles.begin();
	   it!=obstacles.end();it++)
	f << *it << std::endl;
      f.close();
    }
  }

  // Load obstacles file
  void vddrStructure::loadObstacles(const QString &fileName,
				    bool complete_diagram_mode) {

    if (!fileName.isNull() ) {
      std::ifstream f;
      f.open(fileName.toStdString().c_str(),std::ofstream::in);
      assert(f);
      unsigned int n;
      f >> n;    
      for (unsigned int k=0;k<n;k++) {
	Polygon_2 p;
	f >> p;
	insertPolygon(p,0,landmarks.size(),true);
      }
      f.close();
    }
    update(complete_diagram_mode);
  }

  // Load obstacles file
  int vddrStructure::loadEnvironment(const QString &fileName,
				     bool complete_diagram_mode) {

    // Clear landmarks and obstacles
    clearAll(complete_diagram_mode);
    update(complete_diagram_mode);
    // 
    if (!fileName.isNull() ) {
      std::ifstream f;
      f.open(fileName.toStdString().c_str(),std::ofstream::in);
      assert(f);
      std::string s;
      f >> s;
      if (s!="LANDMARKS") {
	std::cerr << "Error opening environment file" << std::endl;
	return 0;
      }
      // Read and insert landmarks
      unsigned int landm;
      f >> landm;    
      for (unsigned int k=0;k<landm;k++) {
	Point_2 l;
	f >> l;
	if (k>0) {
	  std::cout << "*** Adding landmark " << l.x() << " " << l.y() << std::endl; 
	  addLandmark(l); 
	}
      }
      f >> s;
      if (s!="OBSTACLES") {
	std::cerr << "Error opening environment file" << std::endl;
	return 0;
      }
      // Read and insert obstacles
      unsigned int nobst;
      f >> nobst;    
      for (unsigned int k=0;k<nobst;k++) {
	Polygon_2 p;
	f >> p;
	std::cout << "*** Inserting polygon " << std::endl; 
	insertPolygon(p,0,landmarks.size(),true);
	std::cout << "*** Inserting polygon ... done" << std::endl; 
      }
      f >> s;
      if (!f.eof()) {
	if (s!="COLORPOLYGONS") {
	  std::cerr << "Error opening environment file" << std::endl;
	  return 0;
	}
	// Read and insert obstacles
	unsigned int nobst;
	unsigned int r,g,b;
	f >> nobst;    
	for (unsigned int k=0;k<nobst;k++) {
	  Polygon_2 p;
	  f >> p; cPolygons.push_back(p);
	  f >> r;
	  f >> g;
	  f >> b; cPolygonsColor.push_back(QColor(r,g,b));
	}
	
	f.close();
      }
      update(complete_diagram_mode);
    }
    std::cout << "*** All polygons inserted ... done" << std::endl; 
    return landmarks.size()-1;
  }


  // Clear obstacles
  void vddrStructure::clear(bool complete_diagram_mode) {
    obstaclesMutex.lock();
    // Clear list of obstacles
    obstacles.clear();
    // Clear list of dilated obstacles
    obstaclesDilated.clear();
    obstaclesMutex.unlock();
    // Update landmarks structures
    for (unsigned int i=0;i<landmarks.size();i++)
      landmarks[i].clear(complete_diagram_mode); 
    // Clear the drawn graphs
    drawnGraph.clear();
    drawnGraphParabola.clear();
    // Delete commonVis and recreate one
    delete commonVis;
    delete wholeGraph;
  }

  // Clear every thing
  void vddrStructure::clearAll(bool complete_diagram_mode) {
    std::cerr << "CLEARALL" << std::endl;
    clear(complete_diagram_mode);
    std::cerr << "CLEARLAND" << std::endl;
    visibilityMutex.lock();
    landmarks.clear();
    visibilityMutex.unlock();
    std::cerr << "ADDLAND" << std::endl;
    cobstMutex.lock();
    landmarks.push_back(vddrVisibilityArea(0,vddr::Point_2(0,0)));
    cobstMutex.unlock();
    // Allocate commonVis object that will manage common visibility 
    commonVis = new vddrCommonVis();
    std::vector<Segment> ls;
    std::vector<myParabola> lp;
    voronoiMutex.lock();
    drawnGraph.push_back(ls);
    drawnGraphParabola.push_back(lp);
    voronoiMutex.unlock();
    // Whole graph combining all landmarks
    wholeGraph = new Graph();
  }
  

  // Clear current query
  void vddrStructure::clearQuery() {
    nhPathMutex.lock();
    shortestPath = false;
    if (nhPath) {
      delete nhPath; 
      nhPath = NULL;
    }
    sP.clear();
    nhPathMutex.unlock();
  }

  // Set start point
  void vddrStructure::setStart(const queryPoint&s) {
    start   = s;
    startOn = true;
    double dmin       = std::numeric_limits<double>::max();
    unsigned int imin = 0;
    // Determine the closest landmark
    for (unsigned int i=0;i<landmarks.size();i++) {
      double d = 
	(landmarks.at(i).getLandmark().x()-start.p.x())*(landmarks.at(i).getLandmark().x()-start.p.x())+
	(landmarks.at(i).getLandmark().y()-start.p.y())*(landmarks.at(i).getLandmark().y()-start.p.y());
      if (d<dmin) { dmin = d; imin = i;}
    }
    // Compute grid of optimal orientations
    trajectory::computeOrientationGrid(start.p,landmarks.at(imin).getLandmark()); 
    // Reinitialize path
    nhPathMutex.lock();
    if (nhPath) {
      delete nhPath;
      nhPath = NULL;
    }
    nhPathMutex.unlock();
  }
}
