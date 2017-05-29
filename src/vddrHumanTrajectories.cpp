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

#include "vddrHumanTrajectories.h"

#define EPSILON   0.0000001

namespace vddr {
  // Factor to penalize backwards trajectories
  double humanTrajectory::qfac = 3.0;

  // Some constants
  const double humanTrajectory::interFeet  = 9.5;
  const double humanTrajectory::interStep  = 20.0;
  
  // Other static variables
  humanTrajectory::trajFormat humanTrajectory::writtenFormat= STASSE;

  // Save data into file
  void humanTrajectory::savePath(const QString &s) const {

    std::ofstream f;
    f.open(s.toStdString().c_str(),std::ofstream::out);
    if (!f) {
      std::cout << "Could not open file" << std::endl;
      return;
    }

    switch (writtenFormat) {
    case humanTrajectory::KAJITA: {
      // We suppose here that the glueing has been done !
      // Header
      f << "# HRP2.setup (for HRP2 \"Promet\")" << std::endl;
      f << "# Absolute position of the robot initially (centimeters):" << std::endl; 
      f << "# " << at(0)  << std::endl; 
      f << "############ Pattern generator default ###########" << std::endl;
      f << "[START]" << std::endl;
      // Determine if we start by right or left foot
      bool startLeft = false;
      if (size()>1) {
	if (fabs(leftFoot.at(1).x()-leftFoot.at(0).x())+
	    fabs(leftFoot.at(1).y()-leftFoot.at(0).y())>0) {
	  f << "START_WITH LEFT"<< std::endl;
	  startLeft = true;
	}
	else
	  f << "START_WITH RIGHT"<< std::endl;
      }
      f << "SY      0.19    # default foot R-L distance" << std::endl;
      f << "SX_MAX  0.25" << std::endl;
      f << "SY_MAX  0.3" << std::endl;
      f << "TH_MAX  15" << std::endl;
      f << "DSY_MAX 0.05" << std::endl;
      f << "Z_UP    0.07" << std::endl;
      f << "Tsup    0.7" << std::endl;
      f << "Tdbl    0.1" << std::endl;
      f << "HIPDOWN 0.9" << std::endl;
      f << "TOUCHDOWN_SPEED   0.0" << std::endl;
      f << "TD_MARGIN         0.05" << std::endl;
      f << std::endl;
      f << "LIFTOFF_PITCH    5.0" << std::endl;
      f << "LIFTOFF_FOOTX   -0.07" << std::endl;
      f << std::endl;
      f << "TOUCHDOWN_PITCH  -5.0" << std::endl;
      f << "TOUCHDOWN_FOOTX  0.1" << std::endl;
      f << std::endl;
      f << "TOUCHDOWN_ROLL   10" << std::endl;
      f << "TOUCHDOWN_FOOTY  0.1" << std::endl;
      f << std::endl;  
      f << "BLIFTOFF_PITCH   0.0" << std::endl;
      f << "BLIFTOFF_FOOTX   0.07" << std::endl;
      f << std::endl;  
      f << "BTOUCHDOWN_PITCH 15" << std::endl;
      f << "BTOUCHDOWN_FOOTX -0.05" << std::endl;
      f << std::endl;
      f << "ARM_GAIN_X -0.6" << std::endl;
      f << "ARM_GAIN_Y  0.0" << std::endl;
      f << "ARM_GAIN_Z -0.2" << std::endl;
      f << std::endl;
      f << "ZMP_OFFSET_X 0.0" << std::endl;
      f << "ZMP_OFFSET_Y 0.0" << std::endl;
      f << "NOZMPFIX_OFFSET_Y 0.01  # zmp offset at no zmpfix mode" << std::endl;
      f << std::endl;
      f << "NECK_INIT_DEG 0 0" << std::endl;
      f << "[END]" << std::endl;
      
      f << std::endl;
      f << std::endl;
      f << std::endl;
      // Save the points
      unsigned int modulo = (startLeft)?0:1;
      double tfirst = leftFoot.at(0).z(),tprev=tfirst,t;
      f << std::fixed << std::setprecision(3);
      f << "0.000 " << 0.020*humanTrajectory::interFeet << " 0.000" << std::endl;
            
      for (unsigned int i=1;i<size();i++) {
	if (i%2==modulo) {
	  // Express right foot position with respect to previous left foot position
	  double xp = leftFoot.at(i-1).x();
	  double yp = leftFoot.at(i-1).y();
	  double tp = leftFoot.at(i-1).z();
	  double x  = rightFoot.at(i).x();
	  double y  = rightFoot.at(i).y();
	  t  = rightFoot.at(i).z(); 
	  while (t-tprev>M_PI)  t-= 2*M_PI; 
	  while (t-tprev<-M_PI) t+= 2*M_PI; 
	  double sx =  (x-xp)*cos(tp)+(y-yp)*sin(tp);
	  double sy = -(x-xp)*sin(tp)+(y-yp)*cos(tp);
	  f << sx*0.01 << " " << sy*0.01 << " " << 180*(t-tfirst)/M_PI  << std::endl;
	} else {
	  // Express left foot position with respect to previous right foot position
	  double xp = rightFoot.at(i-1).x();
	  double yp = rightFoot.at(i-1).y();
	  double tp = rightFoot.at(i-1).z();
	  double x  = leftFoot.at(i).x();
	  double y  = leftFoot.at(i).y();
	  t  = leftFoot.at(i).z();
	  while (t-tprev>M_PI)  t-= 2*M_PI; 
	  while (t-tprev<-M_PI) t+= 2*M_PI; 
	  double sx = (x-xp)*cos(tp)+(y-yp)*sin(tp);
	  double sy = (x-xp)*sin(tp)-(y-yp)*cos(tp);
	  f << sx*0.01 << " " << sy*0.01 << " " <<  180*(t-tfirst)/M_PI  << std::endl;
	}
	tprev = t;
      }
      f << "0.000 " << 0.020*humanTrajectory::interFeet << " " << 180*(t-tfirst)/M_PI << std::endl;
      f << "0.000 " << 0.020*humanTrajectory::interFeet << " " << 180*(t-tfirst)/M_PI << std::endl;
      break;
    }
    case humanTrajectory::STASSE: {
      // Determine if we start by right or left foot
      bool startLeft = false;
      if (size()>1) {
	if (fabs(leftFoot.at(1).x()-leftFoot.at(0).x())+
	    fabs(leftFoot.at(1).y()-leftFoot.at(0).y())>0) {
	  startLeft = true;
	}
      }
      // Save the points
      unsigned int modulo = (startLeft)?0:1;
      double tfirst = leftFoot.at(0).z(),tprev=tfirst,t;
      f << std::fixed << std::setprecision(3);      
      /* garechav init */
      f << "0.000 " << 0.020*humanTrajectory::interFeet << " 0.000" << " " ;
      double y_lr = 0.020*humanTrajectory::interFeet;
      /* garechav end */
      for (unsigned int i=1;i<size();i++) {
	if (i%2==modulo) {
	  // Express right foot position with respect to previous left foot position
	  double xp = leftFoot.at(i-1).x();
	  double yp = leftFoot.at(i-1).y();
	  double tp = leftFoot.at(i-1).z();
	  double x  = rightFoot.at(i).x();
	  double y  = rightFoot.at(i).y();
	  t  = rightFoot.at(i).z(); 
	  /* garechav init */
	  while (t-tprev>M_PI)  t-= 2*M_PI; 
	  while (t-tprev<-M_PI) t+= 2*M_PI; 
	  double sx =  (x-xp)*cos(tp)+(y-yp)*sin(tp);
	  double sy =  -(x-xp)*sin(tp)+(y-yp)*cos(tp);
	  if (y_lr > 0) sy= sy*-1;
	  f << sx*0.01 << " " << sy*0.01 << " " << (t-tprev)*(180/M_PI) << " ";
	  y_lr = sy;
	  /* garechav end */
	} else {
	  // Express left foot position with respect to previous right foot position
	  double xp = rightFoot.at(i-1).x();
	  double yp = rightFoot.at(i-1).y();
	  double tp = rightFoot.at(i-1).z();
	  double x  = leftFoot.at(i).x();
	  double y  = leftFoot.at(i).y();
	  t  = leftFoot.at(i).z();
	  /* garechav init */
	  while (t-tprev>M_PI)  t-= 2*M_PI; 
	  while (t-tprev<-M_PI) t+= 2*M_PI; 
	  double sx = (x-xp)*cos(tp)+(y-yp)*sin(tp);
	  double sy = (x-xp)*sin(tp)-(y-yp)*cos(tp);
	  if (y_lr > 0) sy = sy*-1;
	  f << sx*0.01 << " " << sy*0.01 << " " <<  180*(t-tprev)/M_PI  << " ";
	  y_lr = sy;
	  /* garechav end */
	}
	tprev = t;
      }
      /* garechav init */
      if (y_lr < 0)
	f << "0.000 " << 0.020*humanTrajectory::interFeet << " " << 180*(t-tprev)/M_PI << " ";
      else
	f << "0.000 " << -0.020*humanTrajectory::interFeet << " " << 180*(t-tprev)/M_PI << " ";
      break;
    }
    } 
    f.close();
  }

  // Compute shortest path
  trajectory *humanTrajectory::computeAdmissiblePath(const trajectory *t,
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
						     bool verbose) {
    // Reset trajectory parts ids
    idCount = 0;
  
    // Compute holonomic path
    std::vector<std::pair<trajectory *,trajectory *> > subpaths; 
    std::vector<std::vector<lPoint> > hPath =
      holonomicPath(p,q,connectors,landmarks,po,qo,
		    pv,qv,queryGraph,sP,lstart,lend,fwdRatio,verbose);

    if (hPath.size()==0) {
      std::cerr << "*** Sorry, no admissible path found," 
		<< "your start and end points cannot be joined." 
		<< std::endl;
      return NULL;
    }
    // Now let us try to do the path with non-holonomic primitives
    trajectory *ret = NULL;
    for (unsigned int i=0;i<hPath.size();i++) {
      // The sub-path
      std::vector<lPoint> hp = hPath.at(i);
      // The landmark
      unsigned int l = hp.at(0).lId;
      if (verbose) {
	if (hp.size()>2)
	  std::cerr << "--- Non-holonomic human path, landmark " 
		    << l << std::endl;  
	else {
	  std::cerr << "--- Non-holonomic human path, landmark transition " 
		    << l << std::endl;  
	}
      }
      // Now let us try to do the path with non-holonomic primitives
      if (hp.size()>2) {
	if (t==NULL) {
	  t = new humanTrajectory(landmarks.at(l).getLandmark());
	}
	trajectory *piece = nonHolonomicPathRecursive(t,landmarks.at(l).getLandmark(),
						      hp.at(0).location, 
						      hp.at(hp.size()-1).location,
						      landmarks.at(l).getCobst(),hp,0,hp.size()-1,verbose); 
	if (piece) {
	  if (!ret) 
	    ret = piece;
	  else {
	    trajectory *r = ret->rightest();
	    r->next       = piece; 
	  }
	  subpaths.push_back(std::pair<trajectory*,trajectory*>(piece,piece->rightest()));
	}
      }
    }
    // Normally we should have one trajectory per part 
    // of size >1.
    // Here, we add the landmark changes
    trajectory *tmp = ret;
    unsigned int l = hPath.at(0).at(0).lId;
    for (unsigned int i=0;i<hPath.size()-1;i++) {
      // The sub-path
      std::vector<lPoint> hp = hPath.at(i);
      unsigned int l1 = hp.at(0).lId;
      while (tmp && tmp->next && tmp->next->landmark==landmarks.at(l1).getLandmark()) {
	tmp = tmp->next;
      }

      if (hp.size()==2) {
	unsigned int l2 = hp.at(1).lId;
	// Landmark switches
	if (tmp && tmp->next) {
	  const Point_3& p1 = tmp->at(tmp->size()-1);
	  const Point_3& p2 = tmp->next->at(0);
	  trajectory *next  = tmp->next;
	  tmp->next         = NULL;
	  tmp->addLandmarkChange(landmarks.at(l1).getLandmark(),
				 landmarks.at(l2).getLandmark(),
				 sqrt((p1.x()-landmarks.at(l1).getLandmark().x())*
				      (p1.x()-landmarks.at(l1).getLandmark().x())+
				      (p1.y()-landmarks.at(l1).getLandmark().y())*
				      (p1.y()-landmarks.at(l1).getLandmark().y())),
				 atan2(p1.y()-landmarks.at(l1).getLandmark().y(),
				       p1.x()-landmarks.at(l1).getLandmark().x()),p1.z());
	  tmp               =  tmp->next;
	  double a1      = p1.z(); 
	  double a2      = p2.z(); 
	  while (a1>+M_PI) a1 -= 2*M_PI;
	  while (a1<-M_PI) a1 += 2*M_PI;
	  while (a2>+M_PI) a2 -= 2*M_PI;
	  while (a2<-M_PI) a2 += 2*M_PI;
	  if (a2-a1>+M_PI) a1+=2*M_PI;
	  if (a2-a1<-M_PI) a2+=2*M_PI;

	  tmp->addRotation(landmarks.at(l2).getLandmark(),
			   sqrt((p1.x()-landmarks.at(l2).getLandmark().x())*
				(p1.x()-landmarks.at(l2).getLandmark().x())+
				(p1.y()-landmarks.at(l2).getLandmark().y())*
				(p1.y()-landmarks.at(l2).getLandmark().y())),
			   atan2(p1.y()-landmarks.at(l2).getLandmark().y(),
				 p1.x()-landmarks.at(l2).getLandmark().x()),a1,a2);
	  tmp               =  tmp->next;
	  tmp->next         = next;
	  tmp               = next;
	  l                 = l2;
	}
      } 
    }
    if (ret) {
      ret->printPathPrimitives(std::cerr);
      ret->gluePaths(ret);
      if (verbose)
	std::cerr << "--- Non-holonomic human path, OK " << std::endl;  
    } else {
      if (verbose)
	std::cerr << "--- Non-holonomic human path, failed " << std::endl;  
    }
    return ret;
    if (verbose) {
      std::cerr << std::endl << "PATH: " << std::endl;
      ret->printPathType(std::cerr);
      ret->printPathIds(std::cerr);
    }
#ifdef TODO
    // Optimization
    if (dooptimize) {
      optimize(&ret,cobst,justshow,verbose);
    }
    ret->gluePaths(ret);
#endif
    return ret;
  }

  // Resolve transcendental equation for the intersection 
  double humanTrajectory::solveDichotomyS2PSL2(const double &valInit,
					       const double &valFinal) {
    // Middle value 
    double middle = 0.5*(valInit+valFinal);
    double d2 = -2*trajectory::tphi2*log(sin(trajectory::phi2)/sqrt(0.5*(1+1.0/qfac)));
  
    double val    = pow(sin(trajectory::phi2),3.0)/sin(trajectory::phi2+d2-middle) - 
      exp(-middle/trajectory::tphi2)*pow(0.5*(1+1.0/qfac),2.0);
    if (fabs(val)<EPSILON) {
      return middle;
    } else if (val<0) {
      return solveDichotomyS2PSL2(middle,valFinal);
    } else {
      return solveDichotomyS2PSL2(valInit,middle);
    }
  }

  // Resolve transcendental equation for the intersection 
  double humanTrajectory::solveDichotomyS2PSL1(const double &valInit,
					       const double &valFinal) {
    // Middle value 
    double middle = 0.5*(valInit+valFinal);
    double val    = sin(trajectory::phi2)/sin(trajectory::phi2-middle) - 
      exp(-middle/trajectory::tphi2)*pow(0.5*(1+1.0/qfac),2.0)/pow(sin(trajectory::phi2),4.0);
    if (fabs(val)<EPSILON) {
      return middle;
    } else if (val<0) {
      return solveDichotomyS2PSL1(middle,valFinal);
    } else {
      return solveDichotomyS2PSL1(valInit,middle);
    }
  }

  // Resolve transcendental equation for the intersection 
  double humanTrajectory::solveDichotomyL2L1(const double &valInit,
					     const double &valFinal) {
    // Middle value 
    double middle = 0.5*(valInit+valFinal);
    double d2 = -2*trajectory::tphi2*log(sin(trajectory::phi2)/sqrt(0.5*(1+1.0/qfac)));
    double val    = sin(trajectory::phi2-middle)/sin(trajectory::phi2) - 
      sin(trajectory::phi2)*sin(trajectory::phi2+d2-middle);
    if (fabs(val)<EPSILON) {
      return middle;
    } else if (val>0) {
      return solveDichotomyL2L1(middle,valFinal);
    } else {
      return solveDichotomyL2L1(valInit,middle);
    }
  }

  // Resolve transcendental equation for the intersection 
  double humanTrajectory::solveDichotomyS2C1(const double &valInit,
					     const double &valFinal) {
    // Middle value 
    double middle = 0.5*(valInit+valFinal);
    double d1 = -2*trajectory::tphi2*log(sin(trajectory::phi2)/sqrt(0.5*(1+qfac)));

    double val    = sin(trajectory::phi2+d1-middle)*sin(phi2) - 
      exp(-middle/trajectory::tphi2);
    if (fabs(val)<EPSILON) {
      return middle;
    } else if (val>0) {
      return solveDichotomyS2C1(middle,valFinal);
    } else {
      return solveDichotomyS2C1(valInit,middle);
    }
  }

  // Determine the relative situation of p and q, with all primitives
  void humanTrajectory::nonHolonomicFreePathComplete(const Point_2 &lnd,
						     const Point_2 &p,
						     const Point_2 &q) {
  
    // 
    clear();
    // Angle pOq: tq
    double a =
      -atan2(p.y()-lnd.y(),p.x()-lnd.x()) 
      +atan2(q.y()-lnd.y(),q.x()-lnd.x());
    if (a> M_PI) a-=2*M_PI;
    if (a<-M_PI) a+=2*M_PI;
    // Polar coordinates of p and q
    double rp= sqrt((p.x()-lnd.x())*(p.x()-lnd.x())+(p.y()-lnd.y())*(p.y()-lnd.y()));
    double rq= sqrt((q.x()-lnd.x())*(q.x()-lnd.x())+(q.y()-lnd.y())*(q.y()-lnd.y()));
    double tq= a;

    double ap= atan2(p.y()-lnd.y(),p.x()-lnd.x());
    double aq= atan2(q.y()-lnd.y(),q.x()-lnd.x());
    // Express points in the O,p frame
    Point_2 pp(rp,0.0);
    Point_2 qq(rq*cos(a),
	       rq*sin(a));
  
    double t1 = tphi1;
    double t2 = tphi2;
    // Critical angles in the underlying partition
    double d1 = -2*trajectory::tphi2*log(sin(trajectory::phi2)/sqrt(0.5*(1+qfac)));
    double d2 = -2*trajectory::tphi2*log(sin(trajectory::phi2)/sqrt(0.5*(1+1.0/qfac)));
    double d3 = -2*trajectory::tphi2*log(pow(sin(trajectory::phi2),2.0)*sqrt(qfac)/(0.5*(1+qfac)));
    double dps = solveDichotomyS2PSL2(0,d2);
    double dps2= solveDichotomyS2PSL1(0,d2);
    double dps3= solveDichotomyL2L1(0,phi2);

    // Angle Opq
    double b = atan2(qq.y()-pp.y(),
		     qq.x()-pp.x());

    // The id the parts will be tagged with
    idCount++;

    // Line segment : part on the right + 2 arcs of circles
    if ((b>-phi2 && b<-phi1) ||
	(tq>0 && tq< phi2 && rq<rp*sin(phi2-tq)/sin(phi2)) ||
	(tq<0 && tq>-phi2 && rq<rp*sin(phi2+tq)/sin(phi2))) {
      makeLine(p,q);   
      return;
    } 
  
    // Line segment and spiral 2
    double dd =  (d1+trajectory::phi2<M_PI)?solveDichotomyS2C1(d1,M_PI):M_PI; 
    if ((tq>0  && tq< dd      && rq<rp*exp(-tq/t2)) ||                // S2(Pi)
	(tq>dd && tq< d1+phi2 && rq<rp*sin(phi2)*sin(phi2+d1-tq))) {  // C2(Pi)
      // Determine the alpha value where the circle and the spiral intersect
      double alphaI = solveDichotomyLS2(0.0,tq,rp,rq,tq);
      double rI     = rq*exp((tq-alphaI)/t2); 
      Point_2 pI(rI*cos(alphaI+ap)+lnd.x(),
		 rI*sin(alphaI+ap)+lnd.y()); 
      // Line 
      makeLine(p,pI);
      // Spiral 2
      addSpiral2AtEnd(lnd,rI,alphaI+ap,0.0,tq-alphaI);
      return;
    }
  
    // Line segment and spiral 1
    if ((tq<0   && tq>-dd      && rq<rp*exp( tq/t2)) ||               // S2(Pi)
	(tq<-dd && tq>-phi2-d1 && rq<rp*sin(-phi2)*sin(-phi2-d1-tq))) { // C2(Pi) 
      // Determine the alpha value where the circle and the spiral intersect
      double alphaI = solveDichotomyLS1(tq,0.0,rp,rq,tq);
      double rI     = rq*exp((tq-alphaI)/t1); 
      Point_2 pI(rI*cos(alphaI+ap)+lnd.x(),
		 rI*sin(alphaI+ap)+lnd.y()); 
      // Line
      makeLine(p,pI);
      // Spiral 1
      addSpiral1AtEnd(lnd,rI,alphaI+ap,0.0,tq-alphaI);    
      return;
    }

    // Spiral 1 and line segment
    if ((tq>0                  && tq< d2      && rq>rp*exp(tq/t2)) ||  // S1(Pi)
	(tq>std::max(dps,dps3) && tq< d2+phi2 && rq>rp/(sin(phi2)*sin(phi2+d2-tq)))) { // D2(Pi)
      // Spiral and line
      // Determine the alpha value where the circle and the spiral intersect
      // It is in the Q frame !!
      double alphaI = solveDichotomyLS1(-tq,0,rq,rp,-tq);
      double rI     = rp*exp((tq+alphaI)/t2); 
      Point_2 pI(rI*cos(ap+tq+alphaI)+lnd.x(),
		 rI*sin(ap+tq+alphaI)+lnd.y()); 
      // Spiral 1
      makeSpiral1(rp,ap,0.0,alphaI+tq);
      // Line
      addLineAtEnd(lnd,pI,q);  
      return;
    }

    // Spiral 2 and line segment
    if ((tq< 0                  && -tq< d2      && rq>rp*exp(tq/t1)) ||  // S1(Pi)
	(tq<-std::max(dps,dps3) && -tq< d2+phi2 && rq>rp/(sin(phi1)*sin(phi1-d2-tq)))) {  // D2(Pi)
      // Spiral and line
      // Determine the alpha value where the circle and the spiral intersect
      // It is in the Q frame !!
      double alphaI = solveDichotomyLS2(0,-tq,rq,rp,-tq);
      double rI     = rp*exp((tq+alphaI)/t1); 
      Point_2 pI(rI*cos(ap+tq+alphaI)+lnd.x(),
		 rI*sin(ap+tq+alphaI)+lnd.y()); 
      // Spiral 2
      makeSpiral2(rp,ap,0.0,alphaI+tq);
      // Line
      addLineAtEnd(lnd,pI,q);    
      return;
    }

    // Spiral 2 and spiral 1
    if ((tq>0  && tq< d2 && rq>rp*exp(-tq/t2) && rq<rp*exp(tq/t2)) ||
	(tq>d2 && tq< d1 && rq>rp*exp(-tq/t2) &&
	 rq<rp*exp(-tq/t2)/(pow(sin(phi2),4.0)/pow(0.5*(1+1.0/qfac),2.0))) ||
	(tq>d1 && tq< d3 && 
	 rq>rp*pow(sin(phi2),4.0)*exp(tq/t2)/pow(0.5*(1+qfac),2.0) &&  
	 rq<rp*exp(-tq/t2)/(pow(sin(phi2),4.0)/pow(0.5*(1+1.0/qfac),2.0)))) { 
      // Two spirals: find intersection point, first
      double aip = t2*log(rp/rq)/2+tq/2; 
      // First spiral
      makeSpiral2(rp,ap,0.0,aip);
      // In-site rotation
      addRotationAtEnd(lnd,rp*exp(-aip/t2),aip+ap,phi2,phi1);    
      // Second spiral
      addSpiral1AtEnd(lnd,rp*exp(-aip/t2),aip+ap,0,tq-aip);
      return;
    }

    // Spiral 1 and spiral 2
    if ((-tq>0  && -tq< d2 && rq>rp*exp(tq/t2) && rq<rp*exp(-tq/t2)) ||
	(-tq>d2 && -tq< d1 && rq>rp*exp(tq/t2) &&
	 rq<rp*exp(tq/t2)/(pow(sin(phi2),4.0)/pow(0.5*(1+1.0/qfac),2.0))) ||
	(-tq>d1 && -tq< d3 && 
	 rq>rp*pow(sin(phi2),4.0)*exp(-tq/t2)/pow(0.5*(1+qfac),2.0) &&  
	 rq<rp*exp(tq/t2)/(pow(sin(phi2),4.0)/pow(0.5*(1+1.0/qfac),2.0)))) { 
      // Two spirals: find intersection point, first
      double aip = t1*log(rp/rq)/2+tq/2; 
      // First spiral
      makeSpiral1(rp,ap,0.0,aip);
      // In-site rotation
      addRotationAtEnd(lnd,rp*exp(-aip/t1),aip+ap,phi1,phi2);    
      // Second spiral
      addSpiral2AtEnd(lnd,rp*exp(-aip/t1),aip+ap,0,tq-aip);
      return;
    }
    // Spiral 2, spiral 1 and line
    if ((tq>std::max(dps,dps2) && tq<d3 && 
	 rq>rp*exp(-tq/t2)/(pow(sin(phi2),4.0)/pow(0.5*(1+1.0/qfac),2.0))) || // S2(Ps)
	(tq>d3 && tq<phi2+d3 && rq>rp*sin(phi2)/sin(phi2+d3-tq)/qfac)) { // D3(Pi)
      double alphaW = solveDichotomyAWLS1S2(0,-tq,rq,rp,-tq,1.0/qfac);
      double alphaI = solveDichotomyLS1S2(0,alphaW,rq,rp,-tq,1.0/qfac);
      double rI     = rq*sin(-phi2-alphaI)/sin(-phi2); 
      Point_2 pI(rI*cos(alphaI+aq)+lnd.x(),
		 rI*sin(alphaI+aq)+lnd.y()); 
      // Two spirals: find intersection point, first
      double aip = -t2*log(rI/rp)/2+(-tq+alphaI)/2; 
      // First spiral
      makeSpiral2(rp,ap,0.0,aip+tq);
      // In-site rotation
      addRotationAtEnd(lnd,rp*exp((-tq-aip)/t2),aip+aq,phi2,phi1);    
      // Second spiral
      addSpiral1AtEnd(lnd,rI*exp(-alphaI/t2),aq,aip,alphaI);
      // Line
      addLineAtEnd(lnd,pI,q);
      return;
    }

    // Spiral 1, spiral 2 and line
    if ((tq<-std::max(dps,dps2) && -tq<d3 && 
	 rq>rp*exp(tq/t2)/(pow(sin(phi2),4.0)/pow(0.5*(1+1.0/qfac),2.0))) ||  // S2(Ps)
	(-tq>d3 && -tq< phi2+d3 && rq>rp*sin(-phi2)/sin(-phi2-d3-tq)/qfac)) {  // D3(Pi)
      double alphaW = solveDichotomyAWLS2S1(0,-tq,rq,rp,-tq,1.0/qfac);
      double alphaI = solveDichotomyLS2S1(0,alphaW,rq,rp,-tq,1.0/qfac);
      double rI     = rq*sin(phi2-alphaI)/sin(phi2); 
      Point_2 pI(rI*cos(alphaI+aq)+lnd.x(),
		 rI*sin(alphaI+aq)+lnd.y()); 
      // Two spirals: find intersection point, first
      double aip = t2*log(rI/rp)/2+(-tq+alphaI)/2; 
      // First spiral
      makeSpiral1(rp,ap,0.0,aip+tq);    
      // In-site rotation
      addRotationAtEnd(lnd,rp*exp(-(-tq-aip)/t2),aip+aq,phi1,phi2);    
      // Second spiral
      addSpiral2AtEnd(lnd,rI*exp(alphaI/t2),aq,aip,alphaI);
      // Line
      addLineAtEnd(lnd,pI,q);
      return;
    }

    // Line, spiral 2, spiral 1
    if ((tq>d1 && tq<d3      && rq<rp*pow(sin(trajectory::phi2),4.0)*
	 exp(tq/trajectory::tphi2)/pow(0.5*(1+qfac),2.0)) ||
	(tq>d3 && tq<phi2+d3 && rq<rp*sin(phi2+d3-tq)/sin(phi2))) { // C3(Pi)
      double alphaW = solveDichotomyAWLS2S1(0,tq,rp,rq,tq,qfac);
      double alphaI = solveDichotomyLS2S1(0,alphaW,rp,rq,tq,qfac);
      double rI     = rp*sin(phi2-alphaI)/sin(phi2); 
      Point_2 pI(rI*cos(alphaI+ap)+lnd.x(),
		 rI*sin(alphaI+ap)+lnd.y()); 
      // Line
      makeLine(p,pI);
      // Two spirals: find intersection point, first
      double aip = t2*log(rI/rq)/2+(tq+alphaI)/2; 
      // First spiral
      addSpiral2AtEnd(lnd,rI*exp(alphaI/t2),ap,alphaI,aip);
      // In-site rotation
      addRotationAtEnd(lnd,rI*exp((alphaI-aip)/t2),aip+ap,phi2,phi1);    
      // Second spiral
      addSpiral1AtEnd(lnd,rq*exp(tq/t1),ap,aip,tq);
      return;    
    }

    // Line, spiral 1, spiral 2
    if ((-tq>d1 && -tq<d3      && rq<rp*pow(sin(trajectory::phi2),4.0)*
	 exp(-tq/trajectory::tphi2)/pow(0.5*(1+qfac),2.0)) ||
	(-tq>d3 && -tq<phi2+d3 && rq<rp*sin(-phi2-d3-tq)/sin(-phi2))) { // C3(Pi)
      double alphaW = solveDichotomyAWLS1S2(0,tq,rp,rq,tq,qfac);
      double alphaI = solveDichotomyLS1S2(0,alphaW,rp,rq,tq,qfac);
      double rI     = rp*sin(-phi2-alphaI)/sin(-phi2); 
      Point_2 pI(rI*cos(alphaI+ap)+lnd.x(),
		 rI*sin(alphaI+ap)+lnd.y()); 
      // Line
      makeLine(p,pI);
      // Two spirals: find intersection point, first
      double aip = -t2*log(rI/rq)/2+(tq+alphaI)/2; 
      // First spiral
      addSpiral1AtEnd(lnd,rI*exp(-alphaI/t2),ap,alphaI,aip);
      // In-site rotation
      addRotationAtEnd(lnd,rI*exp(-(alphaI-aip)/t2),aip+ap,phi1,phi2);    
      // Second spiral
      addSpiral2AtEnd(lnd,rq*exp(-tq/t1),ap,aip,tq);
      return;    
    }
    if (tq>d3 && tq<2*phi2+d3) {
      // Check if the two arcs of circle intersect
      double alphaW = 
	(tq>2*phi2)?phi2:solveDichotomyAWLS2S1L(0,tq,rp,rq,tq,qfac);
      double alphaM = 
	solveDichotomyLS2S1L(0,alphaW,rp,rq,tq,qfac);
      double alphaN = alphaM + 0.5*d3 + 0.5*trajectory::tphi2*log(qfac);
      double rM     = rp*sin(phi2-alphaM)/sin(phi2); 
      Point_2 pM(rM*cos(alphaM+ap)+lnd.x(),
		 rM*sin(alphaM+ap)+lnd.y()); 
      // Line
      makeLine(p,pM);
      // First spiral
      addSpiral2AtEnd(lnd,rM*exp((alphaM)/t2),ap,alphaM,alphaN);
      // In-site rotation
      addRotationAtEnd(lnd,rM*exp((alphaM-alphaN)/t2),alphaN+ap,phi2,phi1);    
      // Second spiral
      addSpiral1AtEnd(lnd,rM*exp((alphaM+d3)/t1)/qfac,ap,alphaN,alphaM+d3);
      // Line
      Point_2 pM2(rM*cos(ap+alphaM+d3)/qfac+lnd.x(),
		  rM*sin(ap+alphaM+d3)/qfac+lnd.y());
      // Line
      addLineAtEnd(lnd,pM2,q);
      return;        
    }
    if (tq<-d3 && tq>-2*phi2-d3) {
      // Check if the two arcs of circle intersect
      double alphaW = 
	(-tq>2*phi2)?phi2:solveDichotomyAWLS2S1L(0,-tq,rp,rq,-tq,qfac);
      double alphaM = 
	solveDichotomyLS2S1L(0,alphaW,rp,rq,-tq,qfac);
      double alphaN = alphaM + 0.5*d3 + 0.5*trajectory::tphi2*log(qfac);
      double rM     = rp*sin(phi2-alphaM)/sin(phi2); 
      Point_2 pM(rM*cos(-alphaM+ap)+lnd.x(),
		 rM*sin(-alphaM+ap)+lnd.y()); 
      // Line
      makeLine(p,pM);
      // First spiral
      addSpiral1AtEnd(lnd,rM*exp((-alphaM)/t1),ap,-alphaM,-alphaN);
      // In-site rotation
      addRotationAtEnd(lnd,rM*exp((-alphaM+alphaN)/t1),-alphaN+ap,phi1,phi2);    
      // Second spiral
      addSpiral2AtEnd(lnd,rM*exp((alphaM+d3)/t1)/qfac,ap,-alphaN,-alphaM-d3);
      // Line
      Point_2 pM2(rM*cos(ap-alphaM-d3)/qfac+lnd.x(),
		  rM*sin(ap-alphaM-d3)/qfac+lnd.y());
      // Line
      addLineAtEnd(lnd,pM2,q);
      return;        
    }
    Point_2 pO(0,0);
    // Line
    makeLine(p,pO);
    // Line
    addLineAtEnd(lnd,pO,q);
  }


  // Draw underlying partition into a painter
  void humanTrajectory::drawPartition(const Point_2  &ss,
				      const Point_2 &lnd,
				      QPainter *painter,
				      QRectF &bRect) {
    double step = 0.005;
    double rp= sqrt(ss.x()*ss.x()+ss.y()*ss.y());
    double ap= atan2(ss.y(),ss.x());
    double d1 = -2*trajectory::tphi2*log(sin(trajectory::phi2)/sqrt(0.5*(1+qfac)));
    double d2 = -2*trajectory::tphi2*log(sin(trajectory::phi2)/sqrt(0.5*(1+1.0/qfac)));
    double d3 = -2*trajectory::tphi2*log(pow(sin(trajectory::phi2),2.0)*sqrt(qfac)/(0.5*(1+qfac)));
    double rmax,rit;
    Point_2 o,q,qq,qqq,s=ss;
    CGAL::Qt::PainterOstream<CK> painterostream = 
      CGAL::Qt::PainterOstream<CK>(painter, bRect);
   
    // Arcs of circle C1(Pi)
    for (double ait=-step;ait<=trajectory::phi2;ait+=step) {
      rit= rp*sin(trajectory::phi2-ait)/sin(trajectory::phi2);
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait>=0)
	painterostream << Segment(o,s);
      s = o;
    }
    for (double ait=step;ait>=-trajectory::phi2;ait-=step) {
      rit= rp*sin(-trajectory::phi2-ait)/sin(-trajectory::phi2);
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait<=0)
	painterostream << Segment(o,s);
      s = o;
    }

    double dd =  (d1+trajectory::phi2<M_PI)?solveDichotomyS2C1(d1,M_PI):M_PI; 
    // Spiral S2(Pi)
    for (double ait=-step;ait<=dd;ait+=step) {
      rit= rp*exp(-ait/trajectory::tphi2);
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait>=0)
	painterostream << Segment(o,s);
      s = o;
    }
    for (double ait=step;ait>=-dd;ait-=step) {
      rit= rp*exp(ait/trajectory::tphi2);
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait<=0)
	painterostream << Segment(o,s);
      s = o;
    }
    // Arc of circle C2(Pi)
    for (double ait=dd-step;ait<=d1+trajectory::phi2;ait+=step) {
      rit= rp*(sin(trajectory::phi2)*sin(trajectory::phi2+d1-ait));
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait>=dd)
	painterostream << Segment(o,s);
      s = o;
    }
    for (double ait=-dd+step;ait>=-d1-trajectory::phi2;ait-=step) {
      rit= rp*(sin(-trajectory::phi2)*sin(-trajectory::phi2-d1-ait));
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait<=-dd)
	painterostream << Segment(o,s);
      s = o;
    }
    // Arc of circle C3(Pi)
    for (double ait=d3-step;ait<=trajectory::phi2+d3;ait+=step) {
      rit= rp*sin(trajectory::phi2+d3-ait)/sin(trajectory::phi2)/qfac;
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait>=d3)
	painterostream << Segment(o,s);
      s = o;
    }
    for (double ait=-d3+step;ait>=-trajectory::phi2-d3;ait-=step) {
      rit= rp*sin(-trajectory::phi2-d3-ait)/sin(-trajectory::phi2)/qfac;
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait<=-d3)
	painterostream << Segment(o,s);
      s = o;
    }
    // Spiral S1(Pr)
    for (double ait=d1-step;ait<=d3;ait+=step) {
      rit= rp*pow(sin(trajectory::phi2),4.0)*exp(ait/trajectory::tphi2)/pow(0.5*(1+qfac),2.0);
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait>=d1)
	painterostream << Segment(o,s);
      s = o;
    }
    for (double ait=-d1+step;ait>=-d3;ait-=step) {
      rit= rp*pow(sin(trajectory::phi2),4.0)*exp(-ait/trajectory::tphi2)/pow(0.5*(1+qfac),2.0);
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait<=-d1)
	painterostream << Segment(o,s);
      s = o;
    }

    // Line  D3(Pi)
    rmax = 2000; rit = 0;
    for (double ait=d3-step;rit<=rmax;ait+=step) {
      rit= rp*sin(trajectory::phi2)/sin(trajectory::phi2+d3-ait)/qfac;
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait>=d3)
	painterostream << Segment(o,s);
      s = o;
    }
    rit = 0;
    for (double ait=-d3+step;rit<=rmax;ait-=step) {
      rit= rp*sin(-trajectory::phi2)/sin(-trajectory::phi2-d3-ait)/qfac;
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait<=-d3)
	painterostream << Segment(o,s);
      s = o;
    }
    // Spiral S1(Pi)
    for (double ait=-step;ait<=d2;ait+=step) {
      rit= rp*exp(ait/trajectory::tphi2);
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait>=0)
	painterostream << Segment(o,s);
      s = o;
    }
    for (double ait=step;ait>=-d2;ait-=step) {
      rit= rp*exp(-ait/trajectory::tphi2);
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait<=0)
	painterostream << Segment(o,s);
      s = o;
    }
    // Line  D1(Pi)
    rit = 0;
    for (double ait=-step;rit<=rmax;ait+=step) {
      rit= rp*sin(trajectory::phi2)/(sin(trajectory::phi2-ait));
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait>=0)
	painterostream << Segment(o,s);
      s = o;
    }
    rit = 0;
    for (double ait=step;rit<=rmax;ait-=step) {
      rit= rp*sin(-trajectory::phi2)/(sin(-trajectory::phi2-ait));
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait<=0)
	painterostream << Segment(o,s);
      s = o;
    }
    double dps = solveDichotomyS2PSL2(0,d2);
    double dps2= solveDichotomyS2PSL1(0,d2);
    double dps3= solveDichotomyL2L1(0,phi2);

    // Spiral S2(Ps)
    for (double ait=std::max(dps,dps2)-step;ait<=d3;ait+=step) {
      rit= rp*exp(-ait/trajectory::tphi2)/(pow(sin(trajectory::phi2),4.0)/pow(0.5*(1+1.0/qfac),2.0));
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait>=std::max(dps,dps2))
	painterostream << Segment(o,s);
      s = o;
    }
    for (double ait=-std::max(dps,dps2)+step;ait>=-d3;ait-=step) {
      rit= rp*exp(ait/trajectory::tphi2)/(pow(sin(trajectory::phi2),4.0)/pow(0.5*(1+1.0/qfac),2.0));
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait<=-std::max(dps,dps2))
	painterostream << Segment(o,s);
      s = o;
    }
    // Line  D2(Pi)
    rit = 0;
    for (double ait=std::max(dps,dps3)-step;rit<=rmax;ait+=step) {
      rit= rp/(sin(trajectory::phi2)*sin(trajectory::phi2+d2-ait));
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait>=std::max(dps,dps3))
	painterostream << Segment(o,s);
      s = o;
    }
    rit = 0;
    for (double ait=-std::max(dps,dps3)+step;rit<=rmax;ait-=step) {
      rit= rp/(sin(-trajectory::phi2)*sin(-trajectory::phi2-d2-ait));
      o  = Point_2(rit*cos(ait+ap),
		   rit*sin(ait+ap));
      if (ait<=-std::max(dps,dps3))
	painterostream << Segment(o,s);
      s = o;
    }
    // Line D4(Pi)
    if (2*trajectory::phi2+d3<M_PI) {
      s =  Point_2(0,0);
      o  = Point_2(rmax*cos(2*trajectory::phi2+d3+ap),
		   rmax*sin(2*trajectory::phi2+d3+ap));
      painterostream << Segment(o,s);
      s =  Point_2(0,0);
      o  = Point_2(rmax*cos(-2*trajectory::phi2-d3+ap),
		   rmax*sin(-2*trajectory::phi2-d3+ap));
      painterostream << Segment(o,s);
    }
  }


  /**
   * Add a line
   */
  void humanTrajectory::addLineAtEnd(const Point_2 &lnd,
				     const Point_2 &p,
				     const Point_2 &q) {
    trajectory *t = this;
    while (t->next!=NULL) t = t->next;
    t->next = new humanTrajectory(lnd);
    t->next->makeLine(p,q);
  }

  /**
   * Make of this trajectory a line from p to q
   */
  void humanTrajectory::makeLine(const Point_2 &p,
				 const Point_2 &q) {

    // Clear
    clear();
    leftFoot.clear();
    rightFoot.clear();
    // Id
    id = idCount;

    // Line
    double the1= atan2(q.y()-p.y(),q.x()-p.x());
    double the2= the1;
    if (cos(atan2(p.y()-landmark.y(),p.x()-landmark.x())-the2+M_PI)<0) the2+=M_PI;
    while (the2>+M_PI) the2 -= 2*M_PI;
    while (the2<-M_PI) the2 += 2*M_PI;
    double th = atan2(p.y()-landmark.y(),
		      p.x()-landmark.x())-the2+M_PI;
  
    leftFoot.push_back(Point_3(p.x()+humanTrajectory::interFeet*sin(the2),
			       p.y()-humanTrajectory::interFeet*cos(the2),
			       the2));
    rightFoot.push_back(Point_3(p.x()-humanTrajectory::interFeet*sin(the2),
				p.y()+humanTrajectory::interFeet*cos(the2),
				the2));
    push_back(Point_3(p.x(),p.y(),th));
  
    double xl = leftFoot.at(0).x()-humanTrajectory::interFeet*sin(the2);
    double yl = leftFoot.at(0).y()+humanTrajectory::interFeet*cos(the2);
    double xr = rightFoot.at(0).x()+humanTrajectory::interFeet*sin(the2);
    double yr = rightFoot.at(0).y()-humanTrajectory::interFeet*cos(the2);
    double dl = sqrt((xl-q.x())*(xl-q.x())+
		     (yl-q.y())*(yl-q.y()));
    double dr = sqrt((xr-q.x())*(xr-q.x())+
		     (yr-q.y())*(yr-q.y()));
    double d = std::min(dl,dr);
    int pflag=0;
    for (int i=1;;i++) {
      if (d>1.2*humanTrajectory::interFeet) {
	if (i%2==0) {
	  // Move left foot
	  leftFoot.push_back(Point_3(rightFoot.at(i-1).x()+
				     2*humanTrajectory::interFeet*sin(the2)+
				     cos(the1)*humanTrajectory::interStep,
				     rightFoot.at(i-1).y()-
				     2*humanTrajectory::interFeet*cos(the2)+
				     sin(the1)*humanTrajectory::interStep,
				     the2));
	  rightFoot.push_back(Point_3(rightFoot.at(i-1).x(),
				      rightFoot.at(i-1).y(),
				      the2));
	} else {
	  // Move right foot
	  rightFoot.push_back(Point_3(leftFoot.at(i-1).x()-
				      2*humanTrajectory::interFeet*sin(the2)+
				      cos(the1)*humanTrajectory::interStep,
				      leftFoot.at(i-1).y()+
				      2*humanTrajectory::interFeet*cos(the2)+
				      sin(the1)*humanTrajectory::interStep,
				      the2));
	  leftFoot.push_back(Point_3(leftFoot.at(i-1).x(),
				     leftFoot.at(i-1).y(),
				     the2));
	}
	double xl = leftFoot.at(i).x()-humanTrajectory::interFeet*sin(the2);
	double yl = leftFoot.at(i).y()+humanTrajectory::interFeet*cos(the2);
	double xr = rightFoot.at(i).x()+humanTrajectory::interFeet*sin(the2);
	double yr = rightFoot.at(i).y()-humanTrajectory::interFeet*cos(the2);
	dl = sqrt((xl-q.x())*(xl-q.x())+
		  (yl-q.y())*(yl-q.y()));
	dr = sqrt((xr-q.x())*(xr-q.x())+
		  (yr-q.y())*(yr-q.y()));
	d = std::min(dl,dr);
      } else if (pflag==0) {
	// Last but one step
	if (dl<dr) {
	  // Move right foot : dl
	  rightFoot.push_back(Point_3(q.x()-
				      humanTrajectory::interFeet*sin(the2),
				      q.y()+
				      humanTrajectory::interFeet*cos(the2),
				      the2));
	  leftFoot.push_back(Point_3(leftFoot.at(i-1).x(),
				     leftFoot.at(i-1).y(),
				     the2));
	} else {
	  // Move left foot : dr
	  leftFoot.push_back(Point_3(q.x()+
				     humanTrajectory::interFeet*sin(the2),
				     q.y()-
				     humanTrajectory::interFeet*cos(the2),
				     the2));
	  rightFoot.push_back(Point_3(rightFoot.at(i-1).x(),
				      rightFoot.at(i-1).y(),
				      the2));
	
	}
	pflag++;
      } else {
	if (dl<dr) {
	  // Move left foot
	  leftFoot.push_back(Point_3(q.x()+
				     humanTrajectory::interFeet*sin(the2),
				     q.y()-
				     humanTrajectory::interFeet*cos(the2),
				     the2));
	  rightFoot.push_back(Point_3(rightFoot.at(i-1).x(),
				      rightFoot.at(i-1).y(),
				      the2));
	} else {
	  // Move right foot
	  rightFoot.push_back(Point_3(q.x()-
				      humanTrajectory::interFeet*sin(the2),
				      q.y()+
				      humanTrajectory::interFeet*cos(the2),
				      the2));
	  leftFoot.push_back(Point_3(leftFoot.at(i-1).x(),
				     leftFoot.at(i-1).y(),
				     the2));
	}
	pflag++;
      }
    
      // Robot
      double x  = 0.5*(leftFoot.at(i).x()+rightFoot.at(i).x());
      double y  = 0.5*(leftFoot.at(i).y()+rightFoot.at(i).y());
      double th = atan2(y-landmark.y(),
			x-landmark.x())-the2+M_PI;
      push_back(Point_3(x,y,th));
      if (pflag==2) break;
    }
    // Start and end point
    start = at(0);
    end   = at(size()-1);
    type = LINE;
  } 

  /**
   * Add a spiral 1
   */
  void humanTrajectory::addSpiral1AtEnd(const Point_2 &lnd,
					const double &rp,
					const double &ap,
					const double &ai,
					const double &af) {
    trajectory *t = this;
    while (t->next!=NULL) t = t->next;
    t->next = new humanTrajectory(lnd);
    t->next->makeSpiral1(rp,ap,ai,af);
  }

  /**
   * Make of this trajectory a spiral 1 from p to q
   */
  void humanTrajectory::makeSpiral1(const double &rp,
				    const double &ap,
				    const double &ai,
				    const double &af) {
    // Clear
    clear();
    leftFoot.clear();
    rightFoot.clear();
    // Id
    id = idCount;

    Point_2 p(rp*exp(-ai/tphi1)*cos(ap+ai)+landmark.x(),
	      rp*exp(-ai/tphi1)*sin(ap+ai)+landmark.y());
    Point_2 q(rp*exp(-af/tphi1)*cos(ap+af)+landmark.x(),
	      rp*exp(-af/tphi1)*sin(ap+af)+landmark.y());
  
    double the= atan2((p.y()-landmark.y()),
		      (p.x()-landmark.x()))-phi1+M_PI;
    while (the>+M_PI) the -= 2*M_PI;
    while (the<-M_PI) the += 2*M_PI;

    // First position
    leftFoot.push_back(Point_3(p.x()+humanTrajectory::interFeet*sin(the),
			       p.y()-humanTrajectory::interFeet*cos(the),
			       the));
    rightFoot.push_back(Point_3(p.x()-humanTrajectory::interFeet*sin(the),
				p.y()+humanTrajectory::interFeet*cos(the),
				the));
    push_back(Point_3(p.x(),p.y(),phi1));
    double rnew,rprev=rp*exp(-ai/tphi1);
    int s = (ai<af)?-1:1;
    // Distance to goal
    double xl = leftFoot.at(0).x()-humanTrajectory::interFeet*sin(the);
    double yl = leftFoot.at(0).y()+humanTrajectory::interFeet*cos(the);
    double xr = rightFoot.at(0).x()+humanTrajectory::interFeet*sin(the);
    double yr = rightFoot.at(0).y()-humanTrajectory::interFeet*cos(the);
    double dl = sqrt((xl-q.x())*(xl-q.x())+
		     (yl-q.y())*(yl-q.y()));
    double dr = sqrt((xr-q.x())*(xr-q.x())+
		     (yr-q.y())*(yr-q.y()));
    double d = std::min(dl,dr);
    int pflag=0;
  
    // Spiral 1
    for (int i=1;;i++) {
      // Robot position
      rnew = rprev -  s*humanTrajectory::interStep*cos(phi1);
      double ait= -tphi1*log(rnew/rp);
      p=Point_2(rnew*cos(ait+ap)+landmark.x(),
		rnew*sin(ait+ap)+landmark.y());
      the= atan2(rnew*sin(ait+ap),
		 rnew*cos(ait+ap))-phi1+M_PI;
      while (the>+M_PI) the -= 2*M_PI;
      while (the<-M_PI) the += 2*M_PI;

      rprev = rnew;
      // Far from the goal
      if (d>1.2*humanTrajectory::interFeet) {
	if (i%2==0) {
	  // Move left foot
	  leftFoot.push_back(Point_3(p.x()+
				     humanTrajectory::interFeet*sin(the),
				     p.y()-
				     humanTrajectory::interFeet*cos(the),
				     the));
	  rightFoot.push_back(Point_3(rightFoot.at(i-1).x(),
				      rightFoot.at(i-1).y(),
				      rightFoot.at(i-1).z()));
	} else {
	  // Move right foot
	  rightFoot.push_back(Point_3(p.x()-
				      humanTrajectory::interFeet*sin(the),
				      p.y()+
				      humanTrajectory::interFeet*cos(the),
				      the));
	  leftFoot.push_back(Point_3(leftFoot.at(i-1).x(),
				     leftFoot.at(i-1).y(),
				     leftFoot.at(i-1).z()));
	}
	// Distance to goal
	double xl = leftFoot.at(i).x() -humanTrajectory::interFeet*sin(the);
	double yl = leftFoot.at(i).y() +humanTrajectory::interFeet*cos(the);
	double xr = rightFoot.at(i).x()+humanTrajectory::interFeet*sin(the);
	double yr = rightFoot.at(i).y()-humanTrajectory::interFeet*cos(the);
	dl = sqrt((xl-q.x())*(xl-q.x())+
		  (yl-q.y())*(yl-q.y()));
	dr = sqrt((xr-q.x())*(xr-q.x())+
		  (yr-q.y())*(yr-q.y()));
	d = std::min(dl,dr);
      }  else if (pflag==0) {
	the= atan2(q.y()-landmark.y(),q.x()-landmark.x())-phi1+M_PI;
	while (the>+M_PI) the -= 2*M_PI;
	while (the<-M_PI) the += 2*M_PI;
      
	// Last but one step
	if (dl<dr) {
	  // Move right foot : dl
	  rightFoot.push_back(Point_3(q.x()-
				      humanTrajectory::interFeet*sin(the),
				      q.y()+
				      humanTrajectory::interFeet*cos(the),
				      the));
	  leftFoot.push_back(Point_3(leftFoot.at(i-1).x(),
				     leftFoot.at(i-1).y(),
				     leftFoot.at(i-1).z()));
	} else {
	  // Move left foot : dr
	  leftFoot.push_back(Point_3(q.x()+
				     humanTrajectory::interFeet*sin(the),
				     q.y()-
				     humanTrajectory::interFeet*cos(the),
				     the));
	  rightFoot.push_back(Point_3(rightFoot.at(i-1).x(),
				      rightFoot.at(i-1).y(),
				      rightFoot.at(i-1).z()));
	
	}
	pflag++;
      } else {
	the= atan2(q.y()-landmark.y(),
		   q.x()-landmark.x())-phi1+M_PI;
	while (the>+M_PI) the -= 2*M_PI;
	while (the<-M_PI) the += 2*M_PI;

	if (dl<dr) {
	  // Move left foot
	  leftFoot.push_back(Point_3(q.x()+
				     humanTrajectory::interFeet*sin(the),
				     q.y()-
				     humanTrajectory::interFeet*cos(the),
				     the));
	  rightFoot.push_back(Point_3(rightFoot.at(i-1).x(),
				      rightFoot.at(i-1).y(),
				      rightFoot.at(i-1).z()));
	} else {
	  // Move right foot
	  rightFoot.push_back(Point_3(q.x()-
				      humanTrajectory::interFeet*sin(the),
				      q.y()+
				      humanTrajectory::interFeet*cos(the),
				      the));
	  leftFoot.push_back(Point_3(leftFoot.at(i-1).x(),
				     leftFoot.at(i-1).y(),
				     leftFoot.at(i-1).z()));
	}
	pflag++;
      }
      // Robot
      double x  = 0.5*(leftFoot.at(i).x()+rightFoot.at(i).x());
      double y  = 0.5*(leftFoot.at(i).y()+rightFoot.at(i).y());
      push_back(Point_3(x,y,phi1));
      if (pflag==2) break;
    }
    // Start and end point
    start = at(0);
    end   = at(size()-1);
    type = SPIRAL1;
  }

  /**
   * Add a spiral 2
   */
  void humanTrajectory::addSpiral2AtEnd(const Point_2 &lnd,
					const double &rp,
					const double &ap,
					const double &ai,
					const double &af) {
    trajectory *t = this;
    while (t->next!=NULL) t = t->next;
    t->next = new humanTrajectory(lnd);
    t->next->makeSpiral2(rp,ap,ai,af);
  }

  /**
   * Make of this trajectory a spiral 2 from p to q
   */
  void humanTrajectory::makeSpiral2(const double &rp,
				    const double &ap,
				    const double &ai,
				    const double &af) {
    // Clear
    clear();
    leftFoot.clear();
    rightFoot.clear();
    // Id
    id = idCount;

    Point_2 p(rp*exp(-ai/tphi2)*cos(ap+ai)+landmark.x(),
	      rp*exp(-ai/tphi2)*sin(ap+ai)+landmark.y());
    Point_2 q(rp*exp(-af/tphi2)*cos(ap+af)+landmark.x(),
	      rp*exp(-af/tphi2)*sin(ap+af)+landmark.y());
    double the= atan2(p.y()-landmark.y(),p.x()-landmark.x())-phi2+M_PI;
    while (the>+M_PI) the -= 2*M_PI;
    while (the<-M_PI) the += 2*M_PI;

    // First position
    leftFoot.push_back(Point_3(p.x()+humanTrajectory::interFeet*sin(the),
			       p.y()-humanTrajectory::interFeet*cos(the),
			       the));
    rightFoot.push_back(Point_3(p.x()-humanTrajectory::interFeet*sin(the),
				p.y()+humanTrajectory::interFeet*cos(the),
				the));
    push_back(Point_3(p.x(),p.y(),phi2));
    double rnew,rprev=rp*exp(-ai/tphi2);
    int s = (ai<af)?1:-1;
    // Distance to goal
    double xl = leftFoot.at(0).x() -humanTrajectory::interFeet*sin(the);
    double yl = leftFoot.at(0).y() +humanTrajectory::interFeet*cos(the);
    double xr = rightFoot.at(0).x()+humanTrajectory::interFeet*sin(the);
    double yr = rightFoot.at(0).y()-humanTrajectory::interFeet*cos(the);
    double dl = sqrt((xl-q.x())*(xl-q.x())+
		     (yl-q.y())*(yl-q.y()));
    double dr = sqrt((xr-q.x())*(xr-q.x())+
		     (yr-q.y())*(yr-q.y()));
    double d = std::min(dl,dr);
    int pflag=0;
  
    // Spiral 2
    for (int i=1;;i++) {
      // Robot position
      rnew = rprev -  s*humanTrajectory::interStep*cos(phi2);
      double ait= -tphi2*log(rnew/rp);
      p=Point_2(rnew*cos(ait+ap)+landmark.x(),
		rnew*sin(ait+ap)+landmark.y());
      the= atan2(rnew*sin(ait+ap),
		 rnew*cos(ait+ap))-phi2+M_PI;
      while (the>+M_PI) the -= 2*M_PI;
      while (the<-M_PI) the += 2*M_PI;

      rprev = rnew;
      // Far from the goal
      if (d>1.2*humanTrajectory::interFeet) {
	if (i%2==0) {
	  // Move left foot
	  leftFoot.push_back(Point_3(p.x()+
				     humanTrajectory::interFeet*sin(the),
				     p.y()-
				     humanTrajectory::interFeet*cos(the),
				     the));
	  rightFoot.push_back(Point_3(rightFoot.at(i-1).x(),
				      rightFoot.at(i-1).y(),
				      rightFoot.at(i-1).z()));
	} else {
	  // Move right foot
	  rightFoot.push_back(Point_3(p.x()-
				      humanTrajectory::interFeet*sin(the),
				      p.y()+
				      humanTrajectory::interFeet*cos(the),
				      the));
	  leftFoot.push_back(Point_3(leftFoot.at(i-1).x(),
				     leftFoot.at(i-1).y(),
				     leftFoot.at(i-1).z()));
	}
	// Distance to goal
	double xl = leftFoot.at(i).x() -humanTrajectory::interFeet*sin(the);
	double yl = leftFoot.at(i).y() +humanTrajectory::interFeet*cos(the);
	double xr = rightFoot.at(i).x()+humanTrajectory::interFeet*sin(the);
	double yr = rightFoot.at(i).y()-humanTrajectory::interFeet*cos(the);
	dl = sqrt((xl-q.x())*(xl-q.x())+
		  (yl-q.y())*(yl-q.y()));
	dr = sqrt((xr-q.x())*(xr-q.x())+
		  (yr-q.y())*(yr-q.y()));
	d = std::min(dl,dr);
      }  else if (pflag==0) {
	the= atan2(q.y()-landmark.y(),q.x()-landmark.x())-phi2+M_PI;
	while (the>+M_PI) the -= 2*M_PI;
	while (the<-M_PI) the += 2*M_PI;
      
	// Last but one step
	if (dl<dr) {
	  // Move right foot : dl
	  rightFoot.push_back(Point_3(q.x()-
				      humanTrajectory::interFeet*sin(the),
				      q.y()+
				      humanTrajectory::interFeet*cos(the),
				      the));
	  leftFoot.push_back(Point_3(leftFoot.at(i-1).x(),
				     leftFoot.at(i-1).y(),
				     leftFoot.at(i-1).z()));
	} else {
	  // Move left foot : dr
	  leftFoot.push_back(Point_3(q.x()+
				     humanTrajectory::interFeet*sin(the),
				     q.y()-
				     humanTrajectory::interFeet*cos(the),
				     the));
	  rightFoot.push_back(Point_3(rightFoot.at(i-1).x(),
				      rightFoot.at(i-1).y(),
				      rightFoot.at(i-1).z()));
	
	}
	pflag++;
      } else {
	the= atan2(q.y()-landmark.y(),q.x()-landmark.x())-phi2+M_PI;
	while (the>+M_PI) the -= 2*M_PI;
	while (the<-M_PI) the += 2*M_PI;

	if (dl<dr) {
	  // Move left foot
	  leftFoot.push_back(Point_3(q.x()+
				     humanTrajectory::interFeet*sin(the),
				     q.y()-
				     humanTrajectory::interFeet*cos(the),
				     the));
	  rightFoot.push_back(Point_3(rightFoot.at(i-1).x(),
				      rightFoot.at(i-1).y(),
				      rightFoot.at(i-1).z()));
	} else {
	  // Move right foot
	  rightFoot.push_back(Point_3(q.x()-
				      humanTrajectory::interFeet*sin(the),
				      q.y()+
				      humanTrajectory::interFeet*cos(the),
				      the));
	  leftFoot.push_back(Point_3(leftFoot.at(i-1).x(),
				     leftFoot.at(i-1).y(),
				     leftFoot.at(i-1).z()));
	}
	pflag++;
      }
      // Robot
      double x  = 0.5*(leftFoot.at(i).x()+rightFoot.at(i).x());
      double y  = 0.5*(leftFoot.at(i).y()+rightFoot.at(i).y());
      push_back(Point_3(x,y,phi2));
      if (pflag==2) break;
    }
    // Start and end point
    start = at(0);
    end   = at(size()-1);
    type = SPIRAL2;
  }

  // Add a rotation
  void humanTrajectory::addRotation(const Point_2 &lnd,
				    const double &rp,
				    const double &ap,
				    const double &ai,
				    const double &af) {
    trajectory *t = this;
    if (t->next!=NULL) return;
    t->next = new humanTrajectory(lnd);
    t->next->makeRotation(rp,ap,ai,af);
  }

  /**
   * Add a rotation
   */
  void humanTrajectory::addRotationAtEnd(const Point_2 &lnd,
					 const double &rp,
					 const double &ap,
					 const double &ai,
					 const double &af) {
    trajectory *t = this;
    while (t->next!=NULL) t = t->next;
    t->next = new humanTrajectory(lnd);
    t->next->makeRotation(rp,ap,ai,af);
  }

  // Add a landmark change at end
  void humanTrajectory::addLandmarkChangeAtEnd(const Point_2 &lnd1,
					       const Point_2 &lnd2,
					       const double &rp,
					       const double &ap,
					       const double &ai) {
    trajectory *t = this;
    while (t->next!=NULL) t = t->next;
    t->next = new humanTrajectory(lnd1);
    t->next->makeLandmarkChange(rp,ap,ai,lnd2);
  }

  // Add a landmark change at end
  void humanTrajectory::addLandmarkChange(const Point_2 &lnd1,
					  const Point_2 &lnd2,
					  const double &rp,
					  const double &ap,
					  const double &ai) {
    trajectory *t = this;
    if (t->next!=NULL){
      std::cerr << "Cannot insert motion" << std::endl;
      return;
    }
    t->next = new humanTrajectory(lnd1);
    t->next->makeLandmarkChange(rp,ap,ai,lnd2);
  }

  // Make of this trajectory an in-site rotation
  void humanTrajectory::makeLandmarkChange(const double &rp,
					   const double &ap,
					   const double &ai,
					   const Point_2 &lnd) {
    // Clear
    clear();
    leftFoot.clear();
    rightFoot.clear();
    // Id
    id = idCount;
    Point_2 p(rp*cos(ap)+landmark.x(),
	      rp*sin(ap)+landmark.y());
    double the= atan2(p.y()-landmark.y(),
		      p.x()-landmark.x())-ai+M_PI;
    while (the>+M_PI) the -= 2*M_PI;
    while (the<-M_PI) the += 2*M_PI;

    // First position
    leftFoot.push_back(Point_3(p.x()+humanTrajectory::interFeet*sin(the),
			       p.y()-humanTrajectory::interFeet*cos(the),
			       the));
    rightFoot.push_back(Point_3(p.x()-humanTrajectory::interFeet*sin(the),
				p.y()+humanTrajectory::interFeet*cos(the),
				the));
    push_back(Point_3(p.x(),p.y(),ai));
    lPositions.push_back(Point_2(landmark.x(),
				 landmark.y()));

    for (int i=1;i<=5;i++) {
      double t  = double(i)/double(5);
      push_back(Point_3(rp*cos(ap)+landmark.x(),
			rp*sin(ap)+landmark.y(),
			ai));
      double the= atan2(p.y()-((1-t)*landmark.y()+t*lnd.y()),
			p.x()-((1-t)*landmark.x()+t*lnd.x()))-ai+M_PI;
      if (i%2==0) {
	// Move left foot
	leftFoot.push_back(Point_3(p.x()+
				   humanTrajectory::interFeet*sin(the),
				   p.y()-
				   humanTrajectory::interFeet*cos(the),
				   the));
	rightFoot.push_back(Point_3(rightFoot.at(i-1).x(),
				    rightFoot.at(i-1).y(),
				    rightFoot.at(i-1).z()));
      } else {
	// Move right foot
	rightFoot.push_back(Point_3(p.x()-
				    humanTrajectory::interFeet*sin(the),
				    p.y()+
				    humanTrajectory::interFeet*cos(the),
				    the));
	leftFoot.push_back(Point_3(leftFoot.at(i-1).x(),
				   leftFoot.at(i-1).y(),
				   leftFoot.at(i-1).z()));
      }
      lPositions.push_back(Point_2((1-t)*landmark.x()+t*lnd.x(),
				   (1-t)*landmark.y()+t*lnd.y()));
    }
    // Start and end point
    start = at(0);
    end   = at(size()-1);
    type =  CHANGELANDMARK;
  }

  /**
   * Make of this trajectory an in-site rotation
   */
  void humanTrajectory::makeRotation(const double &rp,
				     const double &ap,
				     const double &ai,
				     const double &af) {
    // Clear
    clear();
    leftFoot.clear();
    rightFoot.clear();
    // Id
    id = idCount;

    Point_2 p(rp*cos(ap)+landmark.x(),
	      rp*sin(ap)+landmark.y());
    double the= atan2(p.y()-landmark.y(),
		      p.x()-landmark.x())-ai+M_PI;
    while (the>+M_PI) the -= 2*M_PI;
    while (the<-M_PI) the += 2*M_PI;

    // First position
    leftFoot.push_back(Point_3(p.x()+humanTrajectory::interFeet*sin(the),
			       p.y()-humanTrajectory::interFeet*cos(the),
			       the));
    rightFoot.push_back(Point_3(p.x()-humanTrajectory::interFeet*sin(the),
				p.y()+humanTrajectory::interFeet*cos(the),
				the));
    push_back(Point_3(p.x(),p.y(),ai));
    // Rotation
    for (int i=1;i<=5;i++) {
      double t  = double(i)/double(5);
      double ait= (1-t)*ai + t*af;
      the = atan2(p.y()-landmark.y(),p.x()-landmark.x())-ait+M_PI;
      while (the>+M_PI) the -= 2*M_PI;
      while (the<-M_PI) the += 2*M_PI;

      if (i%2==0) {
	// Move left foot
	leftFoot.push_back(Point_3(p.x()+
				   humanTrajectory::interFeet*sin(the),
				   p.y()-
				   humanTrajectory::interFeet*cos(the),
				   the));
	rightFoot.push_back(Point_3(rightFoot.at(i-1).x(),
				    rightFoot.at(i-1).y(),
				    rightFoot.at(i-1).z()));
      } else {
	// Move right foot
	rightFoot.push_back(Point_3(p.x()-
				    humanTrajectory::interFeet*sin(the),
				    p.y()+
				    humanTrajectory::interFeet*cos(the),
				    the));
	leftFoot.push_back(Point_3(leftFoot.at(i-1).x(),
				   leftFoot.at(i-1).y(),
				   leftFoot.at(i-1).z()));
      }
      push_back(Point_3(rp*cos(ap)+landmark.x(),
			rp*sin(ap)+landmark.y(),
			ait));
    }   
    // Start and end point
    start = at(0);
    end   = at(size()-1);   
    type = ROTATION;
  }
  
}
