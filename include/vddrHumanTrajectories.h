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

#ifndef VDDR_HUMANTRAJECTORIES_H
#define VDDR_HUMANTRAJECTORIES_H

#include <queue>      
#include <deque>      
#include <set>      

#include <vddrGraphReduction.h>
#include <vddrObstacles.h>
#include <vddrTrajectories.h>

#undef DEBUG

namespace vddr {
  /**
   * Human trajectory class
   */
  class humanTrajectory : public trajectory {
  public:
    static const double interFeet;
    static const double interStep;

    /**
     * Steps
     */
    std::vector<Point_3> steps;

    /**
     * Left foot
     */
    std::vector<Point_3> leftFoot;

    /**
     * Right foot
     */
    std::vector<Point_3> rightFoot;
  
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
					     bool optimize=true,
					     bool justshow=true,
					     bool penalizeBckwd =false,
					     bool verbose       =false);


  protected:

    /**
     * Resolve transcendental equation for intersection spiral2/circle
     */
    static double solveDichotomyS2C1(const double &valInit,
				     const double &valFinal);
  
    /**
     *
     */
    static double solveDichotomyS2PSL2(const double &valInit,
				       const double &valFinal);

    /**
     *
     */
    static double solveDichotomyS2PSL1(const double &valInit,
				       const double &valFinal);

    /**
     *
     */
    static double solveDichotomyL2L1(const double &valInit,
				     const double &valFinal);
    /**
     * Factor to penalize backward motion
     */
    static double qfac;

  public:
    /**
     * Trajectory constructor
     */
  humanTrajectory(const Point_2 &lnd): trajectory(lnd) {
    };
  
    /**
     * Generate new trajectory
     * @param p Start point
     * @param q End point
     */
    virtual trajectory *generateNonHolonomicFreePathComplete(const Point_2 &lnd,
							     const Point_2 &p,
							     const Point_2 &q) const {
      trajectory *trnew = new humanTrajectory(lnd);
      trnew->nonHolonomicFreePathComplete(lnd,p,q); 
      return trnew;
    }

    /**
     * Determine the non-holonomic shortest path between 
     * configurations p and q, without obstacles
     * @param p Start point
     * @param q End point
     */
    virtual void nonHolonomicFreePathComplete(const Point_2 &lnd,
					      const Point_2 &p,
					      const Point_2 &q);

    /**
     * Glue the composing  
     */
    virtual void gluePaths(trajectory *t) {
      // Glue the paths
      if (t!=this) {
	for (unsigned int i=0;i<t->size();i++) this->push_back(t->at(i));
	humanTrajectory *ht = dynamic_cast<humanTrajectory *>(t);
	if (ht) {
	  for (unsigned int i=0;i<ht->leftFoot.size();i++) 
	    leftFoot.push_back(ht->leftFoot.at(i));
	  for (unsigned int i=0;i<ht->rightFoot.size();i++) 
	    rightFoot.push_back(ht->rightFoot.at(i));
	} 
      }
      humanTrajectory *ht = dynamic_cast<humanTrajectory *>(t);
      if (ht) {
	if (ht->lPositions.size()==0)
	  for (unsigned int i=0;i<ht->size();i++) 
	    this->lPositions.push_back(ht->landmark);
	else
	  for (unsigned int i=0;i<ht->lPositions.size();i++) 
	    this->lPositions.push_back(ht->lPositions.at(i));
      } else {
	std::cerr << "conversion error " ;
	std::cerr << (pathType)t->type << std::endl ;

      }
      if (t->next)
	gluePaths(t->next);
    }
  
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
     * Add a rotation
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
    void addLandmarkChange(const Point_2 &lnd1,
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
     * Set value of qfac
     */
    static void setQ(const double &qf) {
      qfac = qf;
    }

    /**
     * Save results into file
     */
    virtual void savePath(const QString &s) const;

    /**
     * enum for handling the format of written trajectories
     */
    static enum trajFormat {
      KAJITA,
      STASSE
    } writtenFormat;
  };
}
#endif
