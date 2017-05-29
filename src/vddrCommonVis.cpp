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

// Vddr include
#include <vddrCommonVis.h>

// Qt headers
#include <QMutex>

extern QMutex commonVisMutex;
extern QMutex connectorAreaMutex;
extern QMutex reducedGraphMutex;

namespace vddr { 
  // Viewing angle
  double vddrCommonVis::viewAngle           = 2.2;
  
  // Constructor
  vddrCommonVis::vddrCommonVis() {}

  // Destructor
  vddrCommonVis::~vddrCommonVis() {}

  // Update visibility zone 
  void vddrCommonVis::update(const std::vector<vddrVisibilityArea> &landmarkAreas,
			     int landm) {
    // Scan all landmarks areas
    for (unsigned int j=0;j<landmarkAreas.size();j++) 
      if (landm<0 || static_cast<int>(j)==landm) {
	// For all previous landmarks, define the common visibility zone
	// this is formed by two arcs of circles
	for (unsigned int k=0;k<j;k++) {
	  commonVisChords[j][k].clear();
	  std::cerr << "*** Updating visibility between " << j << " and " << k << std::endl;
	  // Middle of landmarks segment
	  Point_2 m((landmarkAreas[k].getLandmark().x()+
		     landmarkAreas[j].getLandmark().x())*0.5,
		    (landmarkAreas[k].getLandmark().y()+
		     landmarkAreas[j].getLandmark().y())*0.5);
	  Point_2 orth((landmarkAreas[k].getLandmark().y()-
			landmarkAreas[j].getLandmark().y()) * (0.5/tan(viewAngle)),
		       (landmarkAreas[j].getLandmark().x()-
			landmarkAreas[k].getLandmark().x()) * (0.5/tan(viewAngle)));
	  // The two centers of the circles
	  Point_2 c1(m.x()+orth.x(),m.y()+orth.y());
	  Point_2 c2(m.x()-orth.x(),m.y()-orth.y());
	  double r     = sqrt(CGAL::squared_distance(c1,landmarkAreas[k].getLandmark())); 
	  double aleph = atan2(m.y()-c1.y(),m.x()-c1.x()); 
	  Polygon_2 cv;
	  if (viewAngle<M_PI/2.0) {
	    // First arc of circle
	    for (unsigned int i = 0; i<NB;i++) {
	      double theta = (2*M_PI-2*viewAngle)*double(i)/NB+viewAngle;    
	      cv.push_back(Point_2(c1.x()+r*cos(theta+aleph),
				   c1.y()+r*sin(theta+aleph)));
	    }
	    // Second arc of circle
	    for (unsigned int i = 0; i<NB+1;i++) {
	      double theta = (2*M_PI-2*viewAngle)*double(i)/(NB+1)+viewAngle;    
	      cv.push_back(Point_2(c2.x()-r*cos(theta+aleph),
				   c2.y()-r*sin(theta+aleph)));	  
	    }
	  }
	  else {
	    for (unsigned int i = 0; i<NB;i++) {
	    // First arc of circle
	      double theta = (2*M_PI-2*viewAngle)*double(i)/NB+viewAngle;    
	      cv.push_back(Point_2(c1.x()-r*cos(theta+aleph),
				   c1.y()-r*sin(theta+aleph)));	    
	    }
	    // Second arc of circle
	    for (unsigned int i = 0; i<NB+1;i++) {
	      double theta = (2*M_PI-2*viewAngle)*double(i)/(NB+1)+viewAngle;    
	      cv.push_back(Point_2(c2.x()+r*cos(theta+aleph),
				   c2.y()+r*sin(theta+aleph)));
	    }
	  }
	  commonVisChords[j][k].push_back(landmarkAreas[k].getChord());
	  commonVisChords[j][k].push_back(landmarkAreas[j].getChord());
	  commonVisChords[j][k].push_back(CGAL::squared_distance(cv[0 ],cv[1]));
	  commonVisChords[j][k].push_back(CGAL::squared_distance(cv[NB],cv[NB+1]));
	  // Add 
	  commonVisMutex.lock();
	  commonVisPolygons[j][k]=cv;
	  commonVisMutex.unlock();
	  std::cerr << "*** Updating visibility between " << j << " and " << k << " done." << std::endl;
	}
      }
  }


  // Build Delaunay graph and visibility structures
  void vddrCommonVis::updateConnectors(const std::vector<vddr::Polygon_2> &obstaclesDilated,
				       std::vector<vddrVisibilityArea> &landmarkAreas,
				       int landm) {
    CGAL::Timer timer;
    timer.start();

    // Update connectors
    std::cerr << "*** Cleaning connectors " << landm << std::endl;
    connectorAreaMutex.lock();
    std::vector<vddrConnector *> connectorAreasTmp;
    for (unsigned int i=0;i<connectorAreas.size();i++) {
      if (connectorAreas[i]!=NULL && 
	  (landm==-1 ||
	   static_cast<int>(connectorAreas[i]->getFrom())==landm || 
	   static_cast<int>(connectorAreas[i]->getTo())==landm)) {
	delete connectorAreas[i];
	connectorAreas[i] = NULL;
      }
      if (connectorAreas[i]!=NULL)
	connectorAreasTmp.push_back(connectorAreas[i]);
    }
    connectorAreas = connectorAreasTmp;
    connectorAreaMutex.unlock();
    for (unsigned int k=0;k<landmarkAreas.size();k++) if (landm==static_cast<int>(k) || landm==-1) {
      for (unsigned int l=0;l<k;l++) {
	std::cerr << "*** Generating connectors between " << k << " and " << l << std::endl;
	
	// If vV[k] and vV[l] intersect
	if (CGAL::do_intersect(landmarkAreas[k].getFreeSpaceNotDilated(),
			       landmarkAreas[l].getFreeSpaceNotDilated())) {
	  std::cerr <<  "*** Free spaces between " << k << " and " 
		    << l << " do intersect" <<  std::endl;
	  std::list<Polygon_with_holes_2_e> intR;
	  std::list<Polygon_with_holes_2_e>::iterator intRit;
	  // Take intersection of vV[k] and vV[l] 
	  CGAL::intersection (landmarkAreas[k].getFreeSpaceNotDilated(),
			      landmarkAreas[l].getFreeSpaceNotDilated(),
			      std::back_inserter(intR));
	  std::cerr <<  "*** Intersection ok " << std::endl;
	  // For each polygon in the intersection
	  for (intRit=intR.begin();intRit!=intR.end();intRit++) {
	    Polygon_with_holes_2_e &p =  *intRit;
	    // Take the difference with all the dilated obstacles
	    for (unsigned int i=0;i<obstaclesDilated.size();i++) {
	      std::cerr <<  "*** Obstacle " << i << std::endl;
	      std::list<Polygon_with_holes_2_e>  intR2;
	      Polygon_with_holes_2    q(obstaclesDilated[i]);
	      Polygon_with_holes_2_e qq = P2ToP2e(q); 
	      CGAL::difference(p,qq,std::back_inserter(intR2));
	      p = *(intR2.begin());
	    }
	    std::cerr <<  "*** Removed dilated obstacles " << std::endl;
	    // Take the difference with the common visibility area
	    // of landmarks k and l
	    const Polygon_2 &cvis = commonVisPolygons[k][l];
	    Polygon_with_holes_2    q(cvis);
	    Polygon_with_holes_2_e qq = P2ToP2e(q);
	    // Get components
	    std::list<Polygon_with_holes_2_e>  components;
	    std::cerr <<  "*** Difference " << std::endl;
	    CGAL::difference(p,qq,std::back_inserter(components));
	    std::cerr <<  "*** Difference ok " << std::endl;
	    // There may be several connected components
	    std::list<Polygon_with_holes_2_e>::const_iterator  componentsIterator;
	    for (componentsIterator =components.begin();
		 componentsIterator!=components.end();
		 componentsIterator++) {
	      std::cerr << "*** Connecting connected component on " << components.size() << std::endl;
	      // For each one, create a vddrConnector object
	      vddrConnector *vc = new vddrConnector(*componentsIterator,k,l,commonVisChords[k][l]);
	      std::cerr << "*** Building connectivity " << std::endl;
	      vc->buildConnectivity();
	      // For the last added nodes, connecto then to the graphs
	      for (unsigned m=0;
		   m<vc->getNodes().size();m++) {
		std::cerr << "*** Adding node  " << m << " on " << vc->getNodes().size() << std::endl;
		lPoint nnodek = 
		  lPoint(Point_2(to_double(vc->getNodes().at(m).x()),
				 to_double(vc->getNodes().at(m).y())),
			 k);
		lPoint nnodel = 
		  lPoint(Point_2(to_double(vc->getNodes().at(m).x()),
				 to_double(vc->getNodes().at(m).y())),
			 l);
		lPoint po(landmarkAreas[k].getFreeSpace().closestPointCobst(nnodek.location),k);
		lPoint qo(landmarkAreas[l].getFreeSpace().closestPointCobst(nnodel.location),l);
		int kp,kq,ipstart,iqstart;
		// Insert these two points pv and qv with the edges
		Graph::Edge *ek,*el;
		lPoint pk = 
		  landmarkAreas[k].getGraph()->closestPointGraph(nnodek,po,landmarkAreas[k].getFreeSpace(),
								 ipstart,kp,ek);
		lPoint pl = 
		  landmarkAreas[l].getGraph()->closestPointGraph(nnodel,qo,landmarkAreas[l].getFreeSpace(),
								 iqstart,kq,el);
		if (ek && el) {
		  reducedGraphMutex.lock();
		  landmarkAreas[k].getGraph()->insertAt(nnodek,pk,ipstart,kp,ek);
		  landmarkAreas[l].getGraph()->insertAt(nnodel,pl,iqstart,kq,el);
		  reducedGraphMutex.unlock();
		}
	      }
	      connectorAreaMutex.lock();
	      connectorAreas.push_back(vc);
	      connectorAreaMutex.unlock();
	    }
	  }
	}
      }
    }
    timer.stop();
    std::cerr << "*** Re-computed connectors: "
	      << timer.time() << std::endl;
    std::cerr << std::endl;
  }
}
