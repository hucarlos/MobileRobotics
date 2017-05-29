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

#include "vddrConnector.h"
#include "vddrDelaunayInsert.h"
#include <QMutex>
extern QMutex connectorAreaMutex;
extern QMutex connectorPointMutex;
extern QMutex connectorReducedGraphMutex;

namespace vddr {

  // Sample a point that will serve as a connecting node
  void vddrConnector::buildConnectivity() {
    // Delaunay
    vddr::Polygon_with_holes_2 pwh = P2eToP2(*this);
    vddr::Polygon_2 outerBigger;
    for (unsigned int i = 0; i<NB;i++) {
      double theta = 2*M_PI*double(i)/(NB)+M_PI;    
      // Outer boundaries (rmax)
      outerBigger.push_back(Point_2(pwh.outer_boundary()[0].x()+1.3*rVisMax*cos(theta),
				    pwh.outer_boundary()[0].y()+1.3*rVisMax*sin(theta)));
    }
    std::vector<vddr::Polygon_2> allEnvelopes;     
    vddr::Polygon_2 outb = pwh.outer_boundary();outb.reverse_orientation();
    allEnvelopes.push_back(outb);
    vddr::Polygon_with_holes_2 P (outerBigger,
				  allEnvelopes.begin(),
				  allEnvelopes.end());
    vddrObstacles vv;
    vv.clear();
    vv.insert(P);
    // Insertion into the Delaunay graph
    delaunay.clear();
    std::vector<vddr::Polygon_with_holes_2> res3;
    vv.polygons_with_holes (std::back_inserter (res3));
    // 
    initGenerator();
    // The circles we have to detect
    std::vector<double> circlesToDetect(chords.size());
    for (unsigned int l=0;l<chords.size();l++) {     
      circlesToDetect[l] = chords[l];
    }
    std::cerr << "Inserting polygons" << std::endl;
    for (std::vector<vddr::Polygon_with_holes_2>::const_iterator it3=res3.begin();
	 it3!=res3.end();it3++) {
      vddr::insert_polygon_with_holes(delaunay, *it3, circlesToDetect, false);
    }
    // Construct the polygon with holes set that correspond to CObst
    std::cerr << "Build graph" << std::endl;
    connectorReducedGraphMutex.lock();
    reducedGraph->clear();
    reducedGraph->build(delaunay,vv,LMAX*from+to,true);
    connectorReducedGraphMutex.unlock();
    // Add the graph nodes
    connectorPointMutex.lock();
    for (unsigned int k=0;k<reducedGraph->V();k++) {
      const Point_2 &p = reducedGraph->getVertex(k).location;
      // Check if the node is interior (not in the boundary)
      if (vv.isPointAdmissible(p))
	nodes.push_back(Point_2_e(p.x(),p.y()));
    }
    connectorPointMutex.unlock();

  }
}
  
