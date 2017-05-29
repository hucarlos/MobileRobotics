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
 * Lesser General Public License for more details.a

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

// CGAL headers
#include <CGAL/Boolean_set_operations_2.h> 
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/Timer.h>

#include <QMutex>
extern QMutex visibilityMutex;
extern QMutex cobstMutex;
extern QMutex reducedGraphMutex;

// Local headers
#include "vddrVisibilityArea.h"
#include "vddrDelaunayInsert.h"

namespace vddr {
  // Static constants
 
  // Constructor
  vddrVisibilityArea::vddrVisibilityArea(unsigned int i, const vddr::Point_2 &p) : id(i),landmark(p) {
    
    // Initialize reducedGraph
    reducedGraph = new Graph();

    angSectorMax= 2*M_PI/(id+NB);
    lenSectorMax = 2.0*sin(angSectorMax*.5)*rVisMax;
    lenSectorMaxSq = lenSectorMax*lenSectorMax;

    // Obstacles boundaries : outerBigger and inner circles
    for (unsigned int i = 0; i<NB;i++) {
      double theta = 2*M_PI*double(i)/(NB)+M_PI;    
      // Outer boundaries (rmax)
      outerBigger.push_back(Point_2(landmark.x()+1.3*rVisMax*cos(theta),
				    landmark.y()+1.3*rVisMax*sin(theta)));
      
      // Inner boundaries (rmin)
      inner.push_back(Point_2(landmark.x()+rVisMin*cos(theta),
			      landmark.y()+rVisMin*sin(theta)));
    }

    // Visibility boundaries
    for (unsigned int i = 0; i<NB+id;i++) {
      double theta = 2*M_PI*double(i)/(NB+id)+M_PI;    
      // Outer boundaries (the larger circle of radius rVisMax)
      outer.push_back(Point_2(landmark.x()+rVisMax*cos(theta),
			      landmark.y()+rVisMax*sin(theta)));    
    }
    
    // Visibility stuff: set boundaries from outer list
    setBoundary(outer,vEnvironment);
  }

  void vddrVisibilityArea::update(const std::vector<vddr::Polygon_2> &obstaclesDilated,
				  bool complete_diagram_mode) {
    // Timer
    CGAL::Timer timer;  
    // Will be used for visibility polygons
    vddr::Polygon_2   vpgn;
    vddr::Polygon_2_e vpgne;
    vddr::Polygon_set_2_e freeSpace_e;
    std::fprintf(stderr, "*** Landmark area update: %d\n",id); 
    // Visibility: guard as the landmark
    VisiLibity::Point guard(landmark.x(),
			    landmark.y());
    timer.start();
    // vEnvironment checked. Note that vEnvironment should have 
    // been already updated here
    if( !vEnvironment.is_in_standard_form()) {
      std::cerr << "xxx Not standard form !!!";
      vEnvironment.enforce_standard_form();
    }
    timer.stop();
    std::fprintf(stderr, "*** Check if standard form: %f\n",timer.time()); 
      
    timer.start();
    if (vEnvironment.is_valid(epsilon)) {
    } else {
      std::cout << std::endl << "Warning:  Environment model "
		<< "is invalid." << std::endl
		<< "A valid environment model must have" << std::endl
		<< "   1)  Outer boundary and hole boundaries pairwise "
		<< "disjoint w/ epsilon clearance (play)," << std::endl
		<< "   2)  holes (including interiors) pairwise "
		<< "disjoint," << std::endl
		<< "   3)  ccw oriented outer boundary, "
		<< "cw oriented holes, and" << std::endl
		<< "   4)  each vertex list beginning with respective "
		<< "lexicographically smallest vertex." 
		<< std::endl;
      return;
    }
    timer.stop();
    std::fprintf(stderr, "*** Check if VisiLibity environment is valid: %f\n",timer.time());
    // Test Guard
    timer.start();
    if( !guard.in(vEnvironment, epsilon)) {
      std::cout << std::endl
		<< "Warning:  VisiLibity guard not in the environment."
		<< std::endl;
      exit(1);
    }
    timer.stop();
    std::fprintf(stderr,"*** Check VisiLibity guard: %f\n",timer.time()); 
    // Form visibility polygon
    timer.start();  
    // Construction of the visibility polygon by Visibility: 
    // - guard is the observer 
    // - vEnvironment contains the set of obstacles represented by simple polygonal outer boundary
    // - epsilon 
    VisiLibity::Visibility_Polygon
      visibilityPolygon(guard, vEnvironment, epsilon);

    // Copy the visibility polygon into a CGAL structure
    for (unsigned int i=0;i<visibilityPolygon.n();i++) {
      vpgn.push_back  (Point_2(visibilityPolygon[i].x(),
			       visibilityPolygon[i].y()));
      vpgne.push_back (Point_2_e(visibilityPolygon[i].x(),
				 visibilityPolygon[i].y()));
    }
    timer.stop();
    std::fprintf(stderr, "*** Get visibility polygon: %f\n",timer.time());
    // Boolean operations to get vis 
    timer.start();
    // Add visibility zones per se. freeSpace_e is a set of polygons (vddr::Polygon_set_2)
    freeSpace_e.insert (vpgne);
    // Remove inner circle
    freeSpace_e.difference(P2ToP2e(vddr::Polygon_with_holes_2(inner)));
    // Get the list of polygons with holes resulting from the difference
    std::vector<vddr::Polygon_with_holes_2_e> res1e;
    freeSpace_e.polygons_with_holes (std::back_inserter (res1e));
    std::fprintf(stderr, "*** Visibility minus inner let : %d polygons\n",(int)res1e.size());   
    visibilityMutex.lock();
    // TODO : there could be several connected components (?)
    freeSpaceNotDilated = res1e[0];
    visibilityMutex.unlock();
    timer.stop();
    std::fprintf(stderr, "*** Get vis polygon: %f\n",timer.time());   
    // Boolean operations to get visAndObst : quit the dilated obstacles from visibility area
    timer.start();
    vddr::Polygon_set_2_e T_e; // Dilated obstacles (as a vddr::Polygon_set_2_e)
    for (unsigned int i=0;i<obstaclesDilated.size();i++) {
      std::fprintf(stderr, "*** Joining polygon %d in union\n",i); 
      T_e.join(P2ToP2e(vddr::Polygon_with_holes_2(obstaclesDilated[i])));
    }
    try {
      // Makes a difference between freeSpace (set of polygons, vddr::Polygon_set_2) 
      // and T (also a set of polygons,vddr::Polygon_set_2) to remove the dilated obstacles 
      if (!freeSpace_e.is_valid())
	std::fprintf(stderr, "### Problem: freeSpace not valid\n"); 	
      if (!T_e.is_valid())
	std::fprintf(stderr, "### Problem: T not valid\n"); 	
      std::fprintf(stderr, "*** Taking difference to free space\n"); 
      freeSpace_e.difference(T_e);
    } catch (std::exception &e) {
      std::cerr << "### Problem: exception not processed" << std::endl;
      std::cerr <<  e.what() << std::endl;
    }
    std::fprintf(stderr, "*** Updating free space\n"); 
    cobstMutex.lock();
    fSpace.clear();
    // Get polygons in two lists (for exact/approx kernels)
    std::vector<vddr::Polygon_with_holes_2> allPolys;
    std::vector<vddr::Polygon_with_holes_2_e> allPolys_e;
    freeSpace_e.polygons_with_holes (std::back_inserter (allPolys_e));
    for (unsigned int i=0;i<allPolys_e.size();i++) {
      allPolys.push_back(P2eToP2(allPolys_e[i]));
      fSpace.insert(allPolys[i]);
    }
    // Fill fSpace
    cobstMutex.unlock();
    timer.stop();
    std::fprintf(stderr, "*** Get visAndObst polygon: %f\n",timer.time()); 
    // Boolean operations to get separate obstacles
    timer.start();
    std::vector<vddr::Polygon_2> allEnvelopes;
    for (unsigned int i=0;i<allPolys.size();i++) {
      vddr::Polygon_2 p = allPolys[i].outer_boundary();p.reverse_orientation();
      allEnvelopes.push_back(p);
    }
    timer.stop();
    std::fprintf(stderr, "*** Get separate polygons: %f\n",timer.time()); 
    // Construct P, a polygon with outerBigger as its outer boundary and 
    // all the other as holes
    timer.start();
    vddr::Polygon_with_holes_2 P (outerBigger,
				  allEnvelopes.begin(),
				  allEnvelopes.end());
    // Compute the symmetric difference of P and Q.
    try {
      cobstMutex.lock();
      cobst.clear();
      cobst.insert(P);
      cobst.join(inner);
      cobstMutex.unlock();
    } catch (std::exception &e) {
      std::cerr << "### Problem: exception not processed" << std::endl;
      std::cerr <<  e.what() << std::endl;
      cobstMutex.unlock();
    }
    timer.stop();
    std::fprintf(stderr, "*** Get cobst: %f\n",timer.time()); 
    // Insertion into the Delaunay graph
    sdg.clear();
    timer.start();
    std::vector<vddr::Polygon_with_holes_2> res3;
    cobst.polygons_with_holes (std::back_inserter (res3));
    initGenerator();
    std::fprintf(stderr, "*** Generator initalized\n"); 
    // The circles we have to detect
    std::vector<double> circlesToDetect;
    circlesToDetect.push_back(lenSectorMaxSq);circlesToDetect.push_back(lenSectorRobSq);
    for (std::vector<vddr::Polygon_with_holes_2>::const_iterator it3=res3.begin();
	 it3!=res3.end();it3++) {
      std::fprintf(stderr, "*** Inserting polygon with holes\n"); 
      vddr::insert_polygon_with_holes(sdg, *it3, circlesToDetect, complete_diagram_mode);
    }
    timer.stop();
    std::fprintf(stderr, "*** Computation of Delaunay graph (%d): %f\n",id,timer.time());
    sdg.is_valid(true,1);
    // Compute reduced graph
    timer.start();
    reducedGraphMutex.lock();
    reducedGraph->clear();
    reducedGraph->build(sdg,cobst,id,true);
    reducedGraphMutex.unlock();
    timer.stop();
    std::fprintf(stderr, "*** Reduction of Delaunay graph (%d): %f\n",id,timer.time());
  }
  
  //
  void vddrVisibilityArea::updateOuterBoundary(const vddr::Polygon_2 &pgn,
					       const std::vector<VisiLibity::Point> &vp) {
    // Test if cgpn intersects with outer boundary
    std::vector<vddr::Polygon_with_holes_2>  pwh;
    try {
      std::cout << "*** Difference outer - pgn " << std::endl;
      // Computes the difference between the outer circle and a new polygon (pgn)
      CGAL::difference(outer, pgn, std::back_inserter(pwh));
    } catch (...) {
      std::cout << "### Problem with the difference, keeping the boundary intact " << std::endl;
      return;
    }
    std::cout << "*** Size of difference " << pwh.size() << std::endl;
    vddr::Polygon_with_holes_2 p = pwh[0];
    if (p.holes_begin()!=p.holes_end()) {
      CGAL::difference(outer, pgn, std::back_inserter(pwh));
      std::cout << "*** Adding hole " << std::endl;
      // Add the new obstacle as a hole for the visibility
      vEnvironment.add_hole(vp);
    } else {
      setBoundary(p.outer_boundary(),vEnvironment);
      outer = p.outer_boundary();
    }
  }
}
