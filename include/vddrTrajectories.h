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

#ifndef VDDR_TRAJECTORIES_H
#define VDDR_TRAJECTORIES_H

#include <queue>      
#include <deque>      
#include <set>      
#include <iostream>

#include <QPainter>
#include <QMutex>
#include <CGAL/Qt/PainterOstream.h>

#include <vddrGraphReduction.h>
#include <vddrObstacles.h>
#include <vddrQueryPoint.h>
#include <vddrConnector.h>
#include <vddrVisibilityArea.h>

#undef DEBUG

namespace vddr {
  /**
   * Trajectory class: it is simply derived from a vector of Point_3
   */
  class trajectory : public std::vector<Point_3> {
  public:
    
    /**
     * Enum for the nature of primitive
     */
    enum pathType {
      LINE,   
      ROTATION,
      SPIRAL1,
      SPIRAL2,
      CHANGELANDMARK
    }; 
  
    /**
     * Counter for the path part ids
     */
    static int idCount;

  protected:
  
    /**
     * Find closest point on the diagram
     * @param p     Souce point
     * @param pp    
     * @param sdg   Reference to the diagram
     * @param cobst Reference to the set of obstacles
     * @return The point on the diagram that is closest to the input point
     */
    template <class T, class U>
      static Point_2 closestPointDiagram(const Point_2 &p,const Point_2 &pp,
					 const T& sdg,
					 const vddrObstacles &cobst);
    
    /**
     * Resolve transcendental equation for intersection line/spiral2/spiral1/line
     */
    static double solveDichotomyLS2S1L(const double &valInit,
				       const double &valFinal,
				       const double &rp,
				       const double &rq,
				       const double &aq,
				       const double &qval=1.0);

    /**
     * Resolve transcendental equation for intersection line/spiral2/spiral1/line
     */
    static double solveDichotomyAWLS2S1L(const double &valInit,
					 const double &valFinal,
					 const double &rp,
					 const double &rq,
					 const double &aq,
					 const double &q=1.0);
    /**
     * Resolve transcendental equation for the intersection line/spiral2/spiral1
     */
    static double solveDichotomyAWLS2S1(const double &valInit,
					const double &valFinal,
					const double &rp,
					const double &rq,
					const double &aq,
					const double &qval=1.0);

    /**
     * Resolve transcendental equation for the intersection line/spiral1/spiral2
     */
    static double solveDichotomyAWLS1S2(const double &valInit,
					const double &valFinal,
					const double &rp,
					const double &rq,
					const double &aq,
					const double &qval=1.0);
  
    /**
     * Resolve transcendental equation for the intersection line/spiral2/spiral1
     */
    static double solveDichotomyLS2S1(const double &valInit,
				      const double &valFinal,
				      const double &rp,
				      const double &rq,
				      const double &aq,
				      const double &qval=1.0);

    /**
     * Resolve transcendental equation for the intersection line/spiral1/spiral2
     */
    static double solveDichotomyLS1S2(const double &valInit,
				      const double &valFinal,
				      const double &rp,
				      const double &rq,
				      const double &aq,
				      const double &qval=1.0);

    /**
     * Resolve transcendental equation for the intersection line/spiral2
     */
    static double solveDichotomyLS2(const double &valInit,
				    const double &valFinal,
				    const double &rp,
				    const double &rq,
				    const double &aq);

    /**
     * Resolve transcendental equation for the intersection line/spiral1
     */
    static double solveDichotomyLS1(const double &valInit,
				    const double &valFinal,
				    const double &rp,
				    const double &rq,
				    const double &aq);
 
  public:
    /**
     * Trajectory constructor
     */
  trajectory(const Point_2 &lnd,pathType p=LINE): 
    next(NULL),better(NULL),type(p),landmark(lnd) {
    };

    /**
     * Trajectory constructor
     */
    inline void copy(const trajectory &t) {
      next =t.next;
      type =t.type;
    };
 
    /**
     * Operator =
     */
    inline const trajectory &operator=(const trajectory &t) {
      this->copy(t);
      return *this;
    } 

    /**
     * Destructor
     */
    ~trajectory() {
      if (next) {
	delete next;
	next = NULL;    
      }
      if (better.size()) {
	for (unsigned int i=0;i<better.size();i++) {
	  if (better.at(i)) {
	    delete better.at(i);
	    better.at(i) = NULL;
	  }
	}
	better.clear();
      }
    }
 
    /**
     * Determine rightest trajectory inside 
     */
    inline trajectory *rightest() {
      trajectory *t = this;
      // Go right
      while (t->next!=NULL) t = t->next;    
      return t;
    }
  
    /**
     * Given a piece of trajectory, determine the referent
     * and remove the trajectory
     */
    trajectory *remove(trajectory *t);

    /**
     * Generate new trajectory
     */
    inline virtual trajectory *generateNonHolonomicFreePathComplete(const Point_2 &lnd,
								    const Point_2 &p,
								    const Point_2 &q) const {
      trajectory *trnew = new trajectory(lnd);
      trnew->nonHolonomicFreePathComplete(lnd,p,q); 
      return trnew;
    }

    /**
     * Determine the non-holonomic shortest path between 
     * configurations p and q, without obstacles
     */
    virtual void nonHolonomicFreePathComplete(const Point_2 &lnd,
					      const Point_2 &p,
					      const Point_2 &q);

    /**
     * Optimization
     */
    static trajectory *optimize(trajectory **from,
				trajectory *to,
				const vddrObstacles &cobst,
				bool justShow,
				bool verbose=false);

    /**
     * Determine the non holonomic shortest path between configurations p and q, 
     * with obstacles, and for landmark lnd
     *  
     *
     */
    static trajectory *nonHolonomicPathRecursive(const trajectory *t,
						 const Point_2 &lnd,
						 const Point_2 &p,
						 const Point_2 &q,
						 const vddrObstacles &cobst,
						 const std::vector<lPoint> &hpath,
						 const int hstart,
						 const int hend,
						 bool verbose =false);

    /**
     * Determine the holonomic shortest path between configurations p and q, 
     * with obstacles
     */
    static std::vector<std::vector<lPoint> >
      holonomicPath(const queryPoint &p,
		    const queryPoint &q,
		    const std::vector<vddrConnector*> &connectors,
		    const std::vector<vddrVisibilityArea> &landmarks,
		    lPoint &po,lPoint &qo,
		    lPoint &pv,lPoint &qv,
		    Graph *queryGraph,
		    std::vector<Graph::Edge *> &sP,
		    unsigned int &lstart,
		    unsigned int &lend,
		    bool penalizeBckwd = false,
		    bool verbose       = false);
  
    /**
     * Add a line
     * @param p Start point
     * @param q End point
     **/
    virtual void addLineAtEnd(const Point_2 &lnd,
			      const Point_2 &p,
			      const Point_2 &q);


    /**
     * Make of this trajectory a line from p to q
     * @param p Start point
     * @param q End point
     */
    virtual void makeLine(const Point_2 &p,
			  const Point_2 &q);

    /**
     * Add a rotation just after the calling trajectory
     **/
    virtual void addRotation(const Point_2 &lnd,
			     const double &rp,
			     const double &ap,
			     const double &ai,
			     const double &af);

    /**
     * Add a rotation at the end of the list of trajectories
     **/
    virtual void addRotationAtEnd(const Point_2 &lnd,
				  const double &rp,
				  const double &ap,
				  const double &ai,
				  const double &af);

    /**
     * Make of this trajectory an in-site rotation
     */
    virtual void makeRotation(const double &rp,
			      const double &ap,
			      const double &ai,
			      const double &af);
  
    /**
     * Add a landmark change at end
     */
    virtual void addLandmarkChangeAtEnd(const Point_2 &lnd1,
					const Point_2 &lnd2,
					const double &rp,
					const double &ap,
					const double &ai);

    /**
     * Add a landmark change immediately after
     */
    virtual void addLandmarkChange(const Point_2 &lnd1,
				   const Point_2 &lnd2,
				   const double &rp,
				   const double &ap,
				   const double &ai);

    /**
     * Make of this trajectory a landmark change
     */
    virtual void makeLandmarkChange(const double &rp,
				    const double &ap,
				    const double &ai,
				    const Point_2 &lnd);

    /**
     * Add a spiral 2
     **/
    virtual void addSpiral2AtEnd(const Point_2 &lnd,
				 const double &rp,
				 const double &ap,
				 const double &ai,
				 const double &af);

    /**
     * Make of this trajectory a spiral 2 from p to q
     */
    virtual void makeSpiral2(const double &rp,
			     const double &ap,
			     const double &ai,
			     const double &af);

    /**
     * Add a spiral 1
     **/
    virtual void addSpiral1AtEnd(const Point_2 &lnd,
				 const double &rp,
				 const double &ap,
				 const double &ai,
				 const double &af);

    /**
     * Make of this trajectory a spiral 1 from p to q
     */
    virtual void makeSpiral1(const double &rp,
			     const double &ap,
			     const double &ai,
			     const double &af);
   
    /**
     * Save path (i.e. set of points) into file
     */ 
    virtual void savePath(const QString &s) const;

    /**
     * Save primitives path into file
     */ 
    virtual void savePrimitives(const QString &s) const;

    /**
     * Save Phis
     */ 
    void savePhis(const QString &s) const;
  
    /**
     *  Recursive printing of the trajectory
     */
    void printPathType(std::ostream &o) const;

    /**
     *  Recursive printing of the trajectory
     */
    void printPath(std::ostream &o) const;

    /**
     *  Recursive printing of the trajectory primitives
     */
    void printPathPrimitives(std::ostream &o) const;

    /**
     *  Recursive printing of the ids of the trajectory
     */
    void printPathIds(std::ostream &o) const;

    /**
     * Recursive aggregation of all constitutive points into the caller
     * @param t Composite trajectory
     */
    virtual void gluePaths(trajectory *t);
  
    /**
     * Recursive count of primitives
     * @param upto Trajectory
     * @return The number of primitives
     */
    int countPrimitives(trajectory *upto) const;
    
    /**
     * Computes the forward motion and backward motion
     */
    std::pair<double,double> fwdBckwd() const;
    
    /**
     * Get the ith primitive
     * @param i The primitive index
     * @return The ith primitive
     */
    inline trajectory *getPrimitive(int i) {
      trajectory *t=this;
      for (int j=1;j<=i && t!=NULL;t=t->next,j++) {}
      return t;
    }
  
    /**
     * Replace the different pieces between tr1 and tr2
     * by trnew 
     */
    static void replace(trajectory **t,
			trajectory *tr1,
			trajectory *tr2,
			trajectory *trnew);

    /**
     * Computation of length
     * @return Length of the trajectory
     */
    double length() const;

    /**
     * Computation of length: 
     * from the start of t1 to the end of t2 
     * @param t1 Pointer to left trajectory
     * @param t2 Pointer to right trajectory
     * @return Length of the trajectory
     */
    double length(const trajectory *t1,
		  const trajectory *t2) const;

    /**
     * Sub-trajectory, next
     */
    trajectory *next;

    /**
     * Sub-trajectories, candidate for optimizing
     */
    std::vector<trajectory *> better;
  
    /**
     * Trajectory type
     */
    pathType type;   
  
    /**
     * An id to know from which computed trajectory this part
     * corresponds to.
     */
    unsigned int id;

    /**
     * Landmark
     */
    Point_2 landmark;

    /**
     * First point of the sub-path
     */
    Point_3 start;

    /**
     * Last point of the sub-path
     */
    Point_3 end;

    /**
     * For final paths (fused):
     * set of landmark positions
     */
    std::vector<Point_2> lPositions;

    /**
     * Constants
     */
    static double phi1;
    static double phi2;
    static double tphi1;
    static double tphi2;
    static const double minAngleInsite;
    static const int muestreos;
    static double *orientationGrid;
    static int orientationGridSize;
    static double orientationGridDelta;
    static double *nhWeight;
    static bool useHolonomic;

    /**
     * Compute admissible path: use recursively the two previous methods
     * to determine an admissible path
     */
    static trajectory *computeAdmissiblePath(const trajectory *t,
					     const queryPoint &p,
					     const queryPoint &q,
					     const std::vector<vddrConnector*> &connectors,
					     const std::vector<vddrVisibilityArea> &landmarks,
					     lPoint &po,lPoint &qo,
					     lPoint &pv,lPoint &qv,
					     Graph *queryGraph,
					     std::vector<Graph::Edge *> &sP,
					     unsigned int &lstart,
					     unsigned int &lend,
					     bool dooptimize,
					     bool justshow,
					     bool fwdRatio,
					     bool verbose);

    /**
     * Is path admissible ?
     */
    bool isPathAdmissible(const vddrObstacles &path) const;

    /**
     * Draw underlying partition
     * @param s Starting point
     * @param lnd Landmark position
     * @param p Painter
     */
    static void drawPartition(const Point_2  &s,
			      const Point_2 &lnd,
			      QPainter *painter,
			      QRectF &bRect); 

    /**
     * Compute underlying grid
     * @param s Starting point
     * @param lnd Landmark position
     */
    static void computeOrientationGrid(const Point_2  &ss,
				       const Point_2 &land);

    /**
     * Save grid
     * @param s File name
     */    
    static void saveOrientationGrid(const QString &fileName,
				    const Point_2  &ss,
				    const Point_2 &land) {
      double ap= atan2(ss.y()-land.y(),ss.x()-land.x());
      int ind  = 0;
      if (!fileName.isNull() && orientationGrid) {
	std::ofstream f;
	f.open(fileName.toStdString().c_str(),std::ofstream::out);
	assert(f);
	f << ss << std::endl;    
	f << land << std::endl;    
	f << trajectory::orientationGridSize << std::endl;
	f << trajectory::orientationGridDelta << std::endl;
	for (int i = -orientationGridSize; i<= orientationGridSize; i++)
	  for (int j = -orientationGridSize; j<= orientationGridSize; j++) {
	    ind++;
	    Point_2 op(ss.x()+i*orientationGridDelta*cos(ap)-j*orientationGridDelta*sin(ap),
		       ss.y()+i*orientationGridDelta*sin(ap)+j*orientationGridDelta*cos(ap));
	    double alp = atan2(op.y(),op.x());
	    if ((op.x()-ss.x())*(-sin(alp))+
		(op.y()-ss.y())*cos(alp)>0)
	      alp += M_PI;
	  if (nhWeight[ind]>0)
	    f << op << " " << orientationGrid[ind] << " " << (int)(nhWeight[ind]) << std::endl;
	  else
	    f << op << " " << alp << " " << (int)(nhWeight[ind]) << std::endl;	    
	  }
	f.close();
      }
    }
    
    /**
     * Draw underlying grid
     * @param s Starting point
     * @param lnd Landmark position
     * @param p Painter
     */
    static void drawOrientationGrid(const Point_2  &ss,
				    const Point_2 &land,
				    QPainter *painter,
				    QRectF &bRect);


  };

  /**
   * Print trajectory type
   */
  std::ostream& operator << (std::ostream& outs,
			     const trajectory::pathType &p);

}
#endif
