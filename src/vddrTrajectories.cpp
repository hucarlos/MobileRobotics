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

#include "vddrTrajectories.h"
#include <fstream>
#include <QBrush>

extern QMutex wholeGraphMutex;
extern QMutex hPathMutex;

#define EPSILON   0.0000001
namespace vddr {
    const double trajectory::minAngleInsite=+0.02;
    double trajectory::phi1=-0.57156465696000658;
    double trajectory::phi2=+0.57156465696000658;
    bool   trajectory::useHolonomic=false;
    int    trajectory::orientationGridSize = 50;
    double trajectory::orientationGridDelta = 10.0;
    double*trajectory::orientationGrid = NULL;
    double*trajectory::nhWeight = NULL;
    double trajectory::tphi1=tan(trajectory::phi1);
    double trajectory::tphi2=tan(trajectory::phi2);
    const int trajectory::muestreos=20;
    
    int trajectory::idCount=0;
    
    // Admissibility, for a path
    bool trajectory::isPathAdmissible(const vddrObstacles &obst) const {
        for (const trajectory *p=this;p!=NULL;p=p->next)
            for (unsigned int i=0;i<p->size();i++) {
                if (!obst.isPointAdmissible(p->at(i))) {
                    return false;
                }
            }
        return true;
    }
    
    // Compute holonomic path
    std::vector<std::vector<lPoint> >
    trajectory::holonomicPath(const queryPoint &p,
                              const queryPoint &q,
                              const std::vector<vddrConnector*> &connectors,
                              const std::vector<vddrVisibilityArea> &landmarks,
                              lPoint &po,lPoint &qo,
                              lPoint &pv,lPoint &qv,
                              Graph *wholeGraph,
                              std::vector<Graph::Edge *> &sP,
                              unsigned int &lstart,
                              unsigned int &lfinal,
                              bool penalizeBckwd,
                              bool verbose) {
        
        std::vector<Graph *> qGraphs;
        // A clone of the graphs
        for (unsigned int k=0;k<landmarks.size();k++)
        {
            std::cerr << ">>> Adding graph from landmark " << k << std::endl;
            Graph *qGraph = new Graph();
            qGraph->copy(*(landmarks[k].getGraphConst()));
            qGraphs.push_back(qGraph);
        }
        
        // Introduction of p in all the necessary graphs
        for (unsigned int k=0;k<p.visLandmarks.size();k++)
        {
            int kp,ipstart;
            Graph::Edge *ep;
            unsigned int l = p.visLandmarks.at(k);
            // Look point closest from p, on the obstacles
            po = lPoint(landmarks[l].getCobst().closestPointCobst(p.p),l);
            // Insert this point pv with the corresponding edges
            lPoint lp(p.p,l);
            pv = qGraphs[l]->closestPointGraph(lp,po,landmarks[l].getCobst(),ipstart,kp,ep);
            
            qGraphs[l]->insertAt(lp,pv,ipstart,kp,ep);
            //if (verbose)
            std::cerr << "--- Start point connected to landmark graph "
            << l << std::endl;
#ifdef DEBUG
            qGraphs[l]->print();
#endif
        }
        // Introduction of q in all the necessary graphs
        for (unsigned int k=0;k<q.visLandmarks.size();k++) {
            int kq=-1,iqstart=-1; Graph::Edge *eq=NULL;
            unsigned int l = q.visLandmarks.at(k);
            // Look point closest from q, on the obstacles
            qo = lPoint(landmarks[l].getCobst().closestPointCobst(q.p),l);
            // Insert this point qv with the corresponding edges
            lPoint lq(q.p,l);
            qv = qGraphs[l]->closestPointGraph(lq,qo,landmarks[l].getCobst(),iqstart,kq,eq);
            if (eq!=NULL) {
                qGraphs[l]->insertAt(lq,qv,iqstart,kq,eq);
                //if (verbose)
                std::cerr << "--- End point connected to landmark graph "
                << l << std::endl;
            }
#ifdef DEBUG
            qGraphs[l]->print();
#endif
        }
        // Update whole graph
        wholeGraphMutex.lock();
        wholeGraph->clear();
        // Add all sub-graphs
        for (unsigned int k=0;k<qGraphs.size();k++)
            wholeGraph->insert(*qGraphs[k]);
        // Add connectors
        for (unsigned int i=0;i<connectors.size();i++) {
            for (unsigned int m=0;m<connectors.at(i)->getNodes().size();m++) {
                lPoint nnodek  = lPoint(Point_2(to_double(connectors.at(i)->getNodes().at(m).x()),
                                                to_double(connectors.at(i)->getNodes().at(m).y())),
                                        connectors.at(i)->getFrom());
                lPoint nnodel  = lPoint(Point_2(to_double(connectors.at(i)->getNodes().at(m).x()),
                                                to_double(connectors.at(i)->getNodes().at(m).y())),
                                        connectors.at(i)->getTo());
                std::vector<lPoint> intPoints;
                intPoints.push_back(nnodek);
                intPoints.push_back(nnodel);
                std::vector<lPoint> intPointsR(intPoints.rbegin(),intPoints.rend());
                wholeGraph->insert(nnodek,nnodel,intPoints ,0.001,1000000.0);
                wholeGraph->insert(nnodel,nnodek,intPointsR,0.001,1000000.0);
                if (verbose)
                    std::cerr << "--- Connected node " << m
                    << " of connector "      << i
                    << " to graph " << std::endl;
            }
        }
        wholeGraphMutex.unlock();
#ifdef DEBUG
        wholeGraph->print();
#endif
        // Do not need anymore qGraphs
        for (unsigned int k=0;k<qGraphs.size();k++)
            delete qGraphs[k];
        
        // Best shortest path from any node p to q:
        // gives the holonomic solution
        // Note that p and q may be connected several times
        hPathMutex.lock();
        sP.clear();
        hPathMutex.unlock();
        
        // This method update the weights in the case
        // we penalize backwards motion
        if (penalizeBckwd) {
            for (unsigned int v=0;v<wholeGraph->V();v++)
                for (Graph::Edge* e = wholeGraph->adj[v]; e!=NULL; e = e->next) {
                    int l    = e->intermediatePoints.at(0).lId;
                    trajectory *t = new trajectory(landmarks.at(l).getLandmark());
                    // Check angle
                    double angle =
                    atan2(e->intermediatePoints.at(e->intermediatePoints.size()-1).location.y()-
                          landmarks.at(l).getLandmark().y(),
                          e->intermediatePoints.at(e->intermediatePoints.size()-1).location.x()-
                          landmarks.at(l).getLandmark().x()) -
                    atan2(e->intermediatePoints.at(0).location.y()-
                          landmarks.at(l).getLandmark().y(),
                          e->intermediatePoints.at(0).location.x()-
                          landmarks.at(l).getLandmark().x());
                    while (angle>+M_PI) angle -= 2*M_PI;
                    while (angle<-M_PI) angle += 2*M_PI;
                    double angleMid =
                    atan2(e->intermediatePoints.at(e->intermediatePoints.size()/2).location.y()-
                          landmarks.at(l).getLandmark().y(),
                          e->intermediatePoints.at(e->intermediatePoints.size()/2).location.x()-
                          landmarks.at(l).getLandmark().x()) -
                    atan2(e->intermediatePoints.at(0).location.y()-
                          landmarks.at(l).getLandmark().y(),
                          e->intermediatePoints.at(0).location.x()-
                          landmarks.at(l).getLandmark().x());
                    while (angleMid>+M_PI) angleMid -= 2*M_PI;
                    while (angleMid<-M_PI) angleMid += 2*M_PI;
                    if (angleMid<0 && angle>0) angle -= 2*M_PI;
                    if (angleMid>0 && angle<0) angle += 2*M_PI;
                    if (fabs(angle)<M_PI) {
                        trajectory *root =
                        t->generateNonHolonomicFreePathComplete(landmarks.at(l).getLandmark(),
                                                                e->intermediatePoints.at(0).location,
                                                                e->intermediatePoints.at(e->intermediatePoints.size()-1).location);
                        root->gluePaths(root);
                        // Determine the ratio of fw/bw motion if the edge is traversed in this way
                        std::pair<double,double> p =  root->fwdBckwd();
                        e->forwardRatio =  0.1+0.9*p.first/(p.first+p.second);
                        delete root;
                    } else {
                        trajectory *root1 =
                        t->generateNonHolonomicFreePathComplete(landmarks.at(l).getLandmark(),
                                                                e->intermediatePoints.at(0).location,
                                                                e->intermediatePoints.at(e->intermediatePoints.size()/2).location);
                        trajectory *root2 =
                        t->generateNonHolonomicFreePathComplete(landmarks.at(l).getLandmark(),
                                                                e->intermediatePoints.at(e->intermediatePoints.size()/2).location,
                                                                e->intermediatePoints.at(e->intermediatePoints.size()-1).location);
                        root1->gluePaths(root1);
                        root2->gluePaths(root2);
                        std::pair<double,double> p1 =  root1->fwdBckwd();
                        std::pair<double,double> p2 =  root2->fwdBckwd();
                        e->forwardRatio = 0.1+0.9*(p1.first+p2.first)/(p1.first+p1.second+p2.first+p2.second);
                        delete root1;
                        delete root2;
                    }
                    delete t;
                }
        }
        // Get all nodes corresponding to final point
        std::vector<int> v = wholeGraph->getIndex(q.p,qGraphs.size());
        if (verbose) {
            std::cerr << "--- End point connected to whole graph " << v.size()
            << " time(s) " << std::endl;
            for (unsigned int l=0;l<v.size() ;l++)
                std::cerr << "--- End point connected to node " << v.at(l) << std::endl;
        }
        double smin = std::numeric_limits<double>::max();
        unsigned int ifinal=-1;
        // Iterates on all the landmarks the start point is connected to
        for (unsigned int k=0;k<p.visLandmarks.size();k++) {
            // Landmark index
            unsigned int l = p.visLandmarks.at(k);
            if (verbose)
                std::cerr << "--- Checking paths connected to start through landmark : "
                << l << std::endl;
            // Shortest paths
            std::vector<Graph::Edge *> sPcand = wholeGraph->shortestPaths(lPoint(p.p,l));
            // Check all paths arriving at q
            for (unsigned int i=0;i<v.size();i++) {
                if (sPcand.at(v.at(i))) {
                    double sum=0.0;
                    for (Graph::Edge *nod=sPcand.at(v.at(i)); nod!=NULL; nod = nod->next)
                        sum+=nod->weight/(1.0+Graph::clearanceFactor*sqrt(nod->clearance));
                    if (sum<smin) {
                        smin  = sum;
                        ifinal= v.at(i);
                        lfinal= wholeGraph->getVertex(sPcand.at(ifinal)->id).lId;
                        lstart= l;
                        hPathMutex.lock();
                        sP    = sPcand;
                        hPathMutex.unlock();
                    }
                }
                
            }
        }
        if (verbose) {
            std::cerr << "--- Best starting landmark " << lstart << std::endl;
            std::cerr << "--- Best ending landmark "   << lfinal << std::endl;
        }
#ifdef DEBUG
        std::cerr << "--- Holonomic path: "        << std::endl;
#endif
        
        // Get all points in hPath
        std::list<std::vector<lPoint> > hPathList;
        if (smin<std::numeric_limits<double>::max()) {
            lPoint s       = wholeGraph->getVertex(ifinal);
            lPoint r       = sP.at(ifinal)->intermediatePoints.at(0);
#ifdef DEBUG
            std::cerr << ifinal
            <<" " << s.location << " " << r.location << std::endl;
#endif
            while (r.location!=p.p) {
                std::list<lPoint> hPathSubList;
                for (int k=sP.at(ifinal)->intermediatePoints.size()-1;k>0;k--)
                    hPathSubList.push_front(sP.at(ifinal)->intermediatePoints.at(k));
#ifdef DEBUG
                std::cerr <<"Initial and final landmarks " << std::endl;
                std::cerr << r.lId
                <<" " << s.lId << std::endl;
                std::cerr << ifinal
                <<" " << s.location << " " << r.location << std::endl;
#endif
                while (r.location!=p.p && s.lId==r.lId) {
                    ifinal = wholeGraph->getIndex(r);
                    s      = wholeGraph->getVertex(ifinal);
                    r      = sP.at(ifinal)->intermediatePoints.at(0);
#ifdef DEBUG
                    std::cerr << ifinal  << " " << s.location << "(" << s.lId
                    << ") " << r.location << "(" << r.lId
                    << ") " << std::endl;
#endif
                    for (int k=sP.at(ifinal)->intermediatePoints.size()-1;k>0;k--) {
                        hPathSubList.push_front(sP.at(ifinal)->intermediatePoints.at(k));
                    }
                }
                if (s.lId==r.lId)
                    hPathSubList.push_front(r);
                std::vector<lPoint> hPathSubVector(hPathSubList.begin(),
                                                   hPathSubList.end());
                hPathList.push_front(hPathSubVector);
                
                if (r.location!=p.p) {
                    hPathSubList.clear();
                    hPathSubList.push_front(s);
                    hPathSubList.push_front(r);
                    std::vector<lPoint> hPathSubVector(hPathSubList.begin(),
                                                       hPathSubList.end());
                    hPathList.push_front(hPathSubVector);
                    ifinal = wholeGraph->getIndex(r);
                    s      = wholeGraph->getVertex(ifinal);
                    r      = sP.at(ifinal)->intermediatePoints.at(0);
                }
            }
        } else {
            std::vector<std::vector<lPoint> > hPath;
            return hPath;
        }
        std::vector<std::vector<lPoint> > hPath(hPathList.begin(),hPathList.end());
#ifdef DEBUG
        std::cerr << "--- Path made of : " << hPath.size()<< std::endl;
        for (unsigned int i=0;i<hPath.size();i++) {
            std::cerr << "--- Path part " << i << " made of " << hPath.at(i).size()<< std::endl;
            for (unsigned int k=0;k<hPath.at(i).size();k++) {
                std::cerr << hPath.at(i).at(k).location << " " << hPath.at(i).at(k).lId << std::endl;
            }
        }
#endif
        return hPath;
    }
    
    
    // Compute shortest path
    trajectory *trajectory::computeAdmissiblePath(const trajectory *t,
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
                                                  bool penalizeBckwd,
                                                  bool verbose) {
        // Reset trajectory parts ids
        idCount = 0;
        
        // Compute holonomic path
        std::vector<std::pair<trajectory *,trajectory *> > subpaths;
        std::vector<std::vector<lPoint> > hPath =
        holonomicPath(p,q,connectors,landmarks,po,qo,
                      pv,qv,queryGraph,sP,lstart,lend,penalizeBckwd,verbose);
        
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
            if (hp.size()>2) {
                if (verbose)
                    std::cerr << "--- Non-holonomic path, landmark "
                    << l << std::endl;
            }
            else {
                if (verbose)
                    std::cerr << "--- Non-holonomic path, landmark transition "
                    << l << std::endl;
            }
            // Now let us try to do the path with non-holonomic primitives
            if (hp.size()>2) {
                // Normal pieces of trajectories
                if (t==NULL) {
                    t = new trajectory(landmarks.at(l).getLandmark());
                }
                trajectory *piece = nonHolonomicPathRecursive(t,
                                                              landmarks.at(l).getLandmark(),
                                                              hp.at(0).location,
                                                              hp.at(hp.size()-1).location,
                                                              landmarks.at(l).getCobst(),hp,0,hp.size()-1,
                                                              verbose);
                std::cerr << "--- Non-holonomic path, landmark "
                << l << std::endl;
                if (piece) {
                    piece->printPathPrimitives(std::cerr);
                    if (!ret) {
                        ret = piece;
                    }
                    else {
                        trajectory *r = ret->rightest();
                        r->next       = piece;
                    }
                    subpaths.push_back(std::pair<trajectory*,trajectory*>(piece,piece->rightest()));
                } else {
                    std::cerr << "--- Could NOT perform the recursive planning " << std::endl;
                    return NULL;
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
                // Landmark change
                if (verbose)
                    std::cerr << "--- Non-holonomic path, changing landmark "
                    << std::endl;
                
                unsigned int l2 = hp.at(1).lId;
                // Landmark switches
                if (tmp && tmp->next) {
                    const Point_3& p1 = tmp->at(tmp->size()-1);
                    const Point_3& p2 = tmp->next->at(0);
                    trajectory *next  = tmp->next;
                    tmp->next         = NULL;
                    tmp->addLandmarkChange(landmarks.at(l1).getLandmark(),landmarks.at(l2).getLandmark(),
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
        if (ret)
            ret->printPathPrimitives(std::cerr);
        
        // Optimization
        if (dooptimize) {
            for (unsigned int i=0;i<subpaths.size();i++) {
                trajectory *from = subpaths.at(i).first;
                trajectory *to   = subpaths.at(i).second;
                // Determine which landmark this part correspond to
                unsigned int l=0;
                for (;l<landmarks.size();l++)
                    if (landmarks.at(l).getLandmark()==from->landmark)
                        break;
                // Determine the part of the trajectory just before and after
                trajectory *prev=ret;
                if (prev)
                    while (prev->next && prev->next!=from) {
                        prev = prev->next;
                    }
                if (prev && not prev->next) prev=NULL;
                // Optimize
                trajectory *lbetter = optimize(&from,to,landmarks[l].getCobst(),justshow,verbose);
                if (justshow) {
                    if (ret && lbetter)
                        ret->better.push_back(lbetter);
                } else {
                    // Important : if prev is NULL, it is because from is the first one
                    // In case that from changes, ret needs to be rewritten
                    if (not prev) {
                        std::cerr << "Changin prev" << std::endl;
                        ret            = from;
                        //trajectory *rr = ret->rightest();
                        //rr->next       = to->next;
                    } else {
                        prev->next     = from;
                    }
                }
            }
        }
        if (ret) {
            ret->gluePaths(ret);
            if (verbose)
                std::cerr << "--- Non-holonomic path, OK " << std::endl;
        } else {
            if (verbose)
                std::cerr << "--- Non-holonomic path, failed " << std::endl;
        }
        return ret;
    }
    
    // Compute the length
    double trajectory::length() const {
        double sum = 0.0;
        // Count the paths
        const trajectory *t = this;
        while (t!=NULL) {
            if (t->type!=ROTATION && t->type!=CHANGELANDMARK) {
                for (unsigned int i=1;i<t->size();i++)
                    sum += sqrt((t->at(i).x()-t->at(i-1).x())*
                                (t->at(i).x()-t->at(i-1).x())+
                                (t->at(i).y()-t->at(i-1).y())*
                                (t->at(i).y()-t->at(i-1).y()));
            }
            t=t->next;
        }
        return sum;
    }
    // Compute the length between two parts of the trajectory
    double trajectory::length(const trajectory *t1,
                              const trajectory *t2) const {
        double sum = 0.0;
        bool count = false;
        // Count the paths
        const trajectory *t=this;
        while (t!=NULL) {
            if (t==t1)
                count = true;
            if (t->type!=ROTATION && t->type!=CHANGELANDMARK && count) {
                for (unsigned int i=1;i<t->size();i++)
                    sum += sqrt((t->at(i).x()-t->at(i-1).x())*
                                (t->at(i).x()-t->at(i-1).x())+
                                (t->at(i).y()-t->at(i-1).y())*
                                (t->at(i).y()-t->at(i-1).y()));
            }
            if (t==t2)
                break;
            t=t->next;
        }
        return sum;
    }
    
    // Computes the ratio between forward motion and backward motion
    std::pair<double,double> trajectory::fwdBckwd() const {
        double sumFwd  = 0.0;
        double sumBckwd= 0.0;
        // Count the paths
        const trajectory *t = this;
        while (t!=NULL) {
            double sum = 0.0;
            if (t->type!=ROTATION && t->type!=CHANGELANDMARK) {
                // Check if the LINE or SPIRAL goes in the direction of the gaze
                double test =
                (t->at(1).x()-t->at(0).x())*(landmark.x()-t->at(0).x())+
                (t->at(1).y()-t->at(0).y())*(landmark.y()-t->at(0).y());
                for (unsigned int i=1;i<t->size();i++)
                    sum += sqrt((t->at(i).x()-t->at(i-1).x())*
                                (t->at(i).x()-t->at(i-1).x())+
                                (t->at(i).y()-t->at(i-1).y())*
                                (t->at(i).y()-t->at(i-1).y()));
                if (test>0) sumFwd  += sum;
                else        sumBckwd+= sum;
            }
            t=t->next;
        }
        return std::pair<double,double>(sumFwd,sumBckwd);
    }
    
    // Compute the count of primitives
    int trajectory::countPrimitives(trajectory *upto) const {
        int sum=1;
        const trajectory *t=this;
        while (t && (t=t->next)) {
            sum++;
            if (t==upto) {
                break;
            }
        }
        return sum;
    }
    
    // Given a piece of trajectory, determine the referent
    // and remove the trajectory
    trajectory *trajectory::remove(trajectory *t) {
        if (next) {
            if (next==t) {
                delete t;
                next = NULL;
                return this;
            }
            return next->remove(t);
        }
        return NULL;
    }
    
    // Glue the composing
    void trajectory::gluePaths(trajectory *t) {
#ifdef DEBUG
        std::cerr << "--- Glueing " << this->id << std::endl;
        std::cerr << "--- Glueing " << this->type << std::endl;
#endif
        // Glue the paths
        if (t!=this) {
            for (unsigned int i=0;i<t->size();i++)
                this->push_back(t->at(i));
        }
        if (t->lPositions.size()==0)
            for (unsigned int i=0;i<t->size();i++)
                this->lPositions.push_back(t->landmark);
        else
            for (unsigned int i=0;i<t->lPositions.size();i++)
                this->lPositions.push_back(t->lPositions.at(i));
        if (t->next)
            gluePaths(t->next);
    }
    
    // Recursive printing of the trajectory
    void trajectory::printPathType(std::ostream &o) const {
        o << (pathType)this->type;
        if (next) {
            o << "-";
            next->printPathType(o);
        }
        else
            o << std::endl;
    }
    
    // Recursive printing of the trajectory primitives
    void trajectory::printPathPrimitives(std::ostream &o) const {
        o << (pathType)this->type;
        o << " - ";
        o << this->landmark;
        o << " - ";
        o << this->start;
        o << " - ";
        o << this->end;
        if (next) {
            o << std::endl;
            next->printPathPrimitives(o);
        }
        else
            o << std::endl;
    }
    
    // Recursive printing of the ids of the parts of the trajectory
    void trajectory::printPathIds(std::ostream &o) const {
        o << this->id;
        if (next) {
            o << "-";
            next->printPathIds(o);
        }  else
            o << std::endl;
    }
    // Recursive printing of the points of the parts of the trajectory
    void trajectory::printPath(std::ostream &o) const {
        for (unsigned int i=0;i<this->size();i++)
            o << "(" << this->at(i) << ") ";
        if (next) {
            o << "-";
            next->printPath(o);
        }  else
            o << std::endl;
    }
    
    // Replace the different pieces between tr1 and tr2
    // by trnew
    void trajectory::replace(trajectory **t,
                             trajectory *tr1,
                             trajectory *tr2,
                             trajectory *trnew) {
        // Get tr2->next
        trajectory *tmp = tr2->next;
        // Eliminate next in-site rotation
        if (tmp && tmp->type==ROTATION) {
            trajectory *tmp4 = tmp;
            tmp = tmp->next;
            tmp4->next = NULL;
            delete tmp4;
        }
        // Check transition beween tmp   (original trajectory)
        //                    and  trnew (inserted part)
        if (tmp) {
            trajectory *tmp5= trnew->rightest();
            if ((tmp->type==SPIRAL1 && tmp5->type==SPIRAL1)||
                (tmp->type==SPIRAL2 && tmp5->type==SPIRAL2)) {
                std::cerr << "No need to make right in-site rotation" << std::endl;
            } else {
                std::cerr << "Need to make right in-site rotation "
                << tmp->type  << " "
                << tmp5->type << " "
                << std::endl;
                Point_3 first  = tmp->at(0);
                Point_3 last   = tmp5->at(tmp5->size()-1);
                double a1      = last.z();
                double a2      = first.z();
                while (a1>+M_PI) a1 -= 2*M_PI;
                while (a1<-M_PI) a1 += 2*M_PI;
                while (a2>+M_PI) a2 -= 2*M_PI;
                while (a2<-M_PI) a2 += 2*M_PI;
                if (fabs(a1-a2)>trajectory::minAngleInsite) {
                    tmp5->addRotation(trnew->landmark,
                                      sqrt((first.x()-trnew->landmark.x())*(first.x()-trnew->landmark.x())+
                                           (first.y()-trnew->landmark.y())*(first.y()-trnew->landmark.y())),
                                      atan2((first.y()-trnew->landmark.y()),
                                            (first.x()-trnew->landmark.x())),a1,a2);
                    tmp5->next->id   = tmp5->id;
                } else
                    std::cerr << "Right in-site rotation can be neglected " <<  std::endl;
            }
        }
        // Now tr2 points to nothing
        tr2->next       = NULL;
        // Eliminate previous in-site rotation
        if (*t!=tr1) {
            // Eliminate tr1->tr2
            trajectory *tmp2= (*t)->remove(tr1);
            if (tmp2->type==ROTATION) {
                tmp2= (*t)->remove(tmp2);
            }
            // Check transition beween tmp2   (end of the trajectory)
            //                     and  trnew (inserted part)
            if ((tmp2->type==SPIRAL1 && trnew->type==SPIRAL1)||
                (tmp2->type==SPIRAL2 && trnew->type==SPIRAL2)) {
                std::cerr << "No need to make left in-site rotation" << std::endl;
            } else {
                std::cerr << "Need to make left in-site rotation "
                << tmp2->type  << " "
                << trnew->type << " "
                << std::endl;
                Point_3 last   = tmp2->at(tmp2->size()-1);
                Point_3 first  = trnew->at(0);
                double a1      = last.z();
                double a2      = first.z();
                while (a1>+M_PI) a1 -= 2*M_PI;
                while (a1<-M_PI) a1 += 2*M_PI;
                while (a2>+M_PI) a2 -= 2*M_PI;
                while (a2<-M_PI) a2 += 2*M_PI;
                if (fabs(a1-a2)>trajectory::minAngleInsite) {
                    tmp2->addRotation(trnew->landmark,
                                      sqrt((first.x()-trnew->landmark.x())*(first.x()-trnew->landmark.x())+
                                           (first.y()-trnew->landmark.y())*(first.y()-trnew->landmark.y())),
                                      atan2((first.y()-trnew->landmark.y()),
                                            (first.x()-trnew->landmark.x())),a1,a2);
                    tmp2->next->id   = tmp2->id;
                    tmp2             = tmp2->next;
                } else
                    std::cerr << "Left in-site rotation can be neglected " <<  std::endl;
            }
            tmp2->next      = trnew;
        }
        else {
            std::cerr << "Removing original starting part" << std::endl;
            delete *t;
            *t            = trnew;
        }
        
        trajectory *tmp3= trnew->rightest();
        tmp3->next      = tmp;
    }
    
    // Post-computation optimization
    trajectory *trajectory::optimize(trajectory **from,
                                     trajectory *to,
                                     const vddrObstacles &cobst,
                                     bool justShow,
                                     bool verbs) {
        // Outer loop
        for (int k=0;k<10;k++) {
            trajectory *lbetter=NULL;
            bool lbetterok=false;
            // Initialize random seed
            srand(time(NULL));
            
            // Count the primitives
            int c     = (*from)->countPrimitives(to);
            if (c<4) return NULL;
            double improvement = 0.0;
            trajectory *tr1,*tr2,*tr1better,*tr2better,*trnew=NULL;
            // Inner loop
            for (int i=0;i<10;i++) {
                // Select one of them, randomly
                pathType t1= ROTATION;
                int      i1= c-1;
                while (t1==ROTATION || i1==c-1) {
                    i1 = rand()%c;
                    tr1= (*from)->getPrimitive(i1);
                    t1 = tr1->type;
                }
                // Select another one, randomly
                pathType t2= ROTATION;
                int      i2;
                while (t2==ROTATION) {
                    i2 = i1+1+rand()%(c-i1-1);
                    tr2= (*from)->getPrimitive(i2);
                    t2 = tr2->type;
                }
                // Discards the cases when tr2 is in the next list
                // i.e. part of the same composite trajectory
                if (tr1->id==tr2->id)
                    continue;
                // Try connecting first point of tr1 to last point of tr2
                Point_2 p1(tr1->at(0).x(),
                           tr1->at(0).y());
                Point_2 p2(tr2->at(tr2->size()-1).x(),
                           tr2->at(tr2->size()-1).y());
                trnew = (*from)->generateNonHolonomicFreePathComplete((*from)->landmark,p1,p2);
                if (trnew->isPathAdmissible(cobst)) {
                    if (improvement<-trnew->length() +(*from)->length(tr1,tr2)) {
                        improvement = -trnew->length() +(*from)->length(tr1,tr2);
                        if (verbs) {
                            std::cerr << ">>> Better alternative found "  << std::endl;
                            std::cerr << ">>> From " << p1 << " " << p2 << std::endl;
                            std::cerr << ">>> Improvement "
                            << improvement << std::endl;
                        }
                        lbetterok = true;
#ifdef TODO
                        std::cerr << *from << " " << (*from)->next << " " << lbetter << std::endl;
#endif
                        if (lbetter) { delete lbetter; lbetter = NULL;}
                        lbetter     = trnew;
                        tr1better   = tr1;
                        tr2better   = tr2;
                    } else {
                        if (trnew) {
                            delete trnew;
                            trnew = NULL;
                        }
                    }
                } else {
                    if (trnew) {
                        delete trnew;
                        trnew = NULL;
                    }
                }
            }
            // Replace the segments between tr1 and tr2 (included)
            // by trnew
            if (lbetterok) {
                if (!justShow) {
                    std::cerr << ">>> Replacing trajectory "  << std::endl;
                    replace(from,tr1better,tr2better,lbetter);
#ifdef DEBUG
                    std::cerr << ">>> Resulting trajectory "  << std::endl;
                    std::cerr << lbetter->next << std::endl;
                    std::cerr << (*from)->next << std::endl;
                    (*from)->printPathType(std::cerr);
                    (*from)->printPathIds(std::cerr);
#endif
                    break;
                } else {
                    std::cerr << ">>> Fusing alternative trajectory "  << std::endl;
                    // For just displaying the improvement
                    lbetter->gluePaths(lbetter);
                    return lbetter;
                }
            }
            // If the trajectory is not modified,
            // just return the improved part
            if (justShow) return lbetter;
        }
        return *from;
    }
    
    // Save all Phis for plotting/analysis purposes
    void trajectory::savePhis(const QString &s) const {
        std::ofstream f;
        f.open(s.toStdString().c_str(),std::ofstream::out);
        if (!f) {
            std::cout << "Could not open file" << std::endl;
            return;
        }
        // We suppose here that the glueing has been done
        f << "PHIS" << std::endl;
        f << size() << std::endl;
        // Save the points
        for (unsigned int i=0;i<size();i++) {
            double phi = at(i).z();
            if (phi> M_PI) phi -= 2*M_PI;
            if (phi<-M_PI) phi += 2*M_PI;
            f << phi << std::endl;
        }
        f.close();
    }
    
    // Save data into file
    void trajectory::savePath(const QString &s) const {
        std::ofstream f;
        f.open(s.toStdString().c_str(),std::ofstream::out);
        if (!f) {
            std::cout << "Could not open file" << std::endl;
            return;
        }
        // We suppose here that the glueing has been done
        f << "PATH" << std::endl;
        f << size() << std::endl;
        // Save the points
        for (unsigned int i=0;i<size();i++)
            f << at(i) << std::endl;
        f.close();
    }
    
    // Save data into file
    void trajectory::savePrimitives(const QString &s) const {
        std::ofstream f;
        f.open(s.toStdString().c_str(),std::ofstream::out);
        if (!f) {
            std::cout << "Could not open file" << std::endl;
            return;
        }
        // Save data recursively
        printPathPrimitives(f);
        f.close();
    }
    
    // Check if the non-holonomic path is OK, otherwise, do recursion
    trajectory *trajectory::nonHolonomicPathRecursive(const trajectory *t,
                                                      const Point_2 &lnd,
                                                      const Point_2 &p,
                                                      const Point_2 &q,
                                                      const vddrObstacles &cobst,
                                                      const std::vector<lPoint> &hpath,
                                                      const int hstart,
                                                      const int hend,
                                                      bool verbose) {
        // Clear the structure
#ifdef DEBUG
        std::cerr << p << " " << q << std::endl;
        std::cerr << hstart << " " << hend << std::endl;
#endif
        if (hstart==hend) return NULL;
        trajectory *root = t->generateNonHolonomicFreePathComplete(lnd,p,q);
        if (verbose)
            root->printPathType(std::cout);
        
        if (!root->isPathAdmissible(cobst)) {
            delete root;
            // Cut the path in two, intermediate point is given by the
            // middle of the piece of holonomic path
            int middle = (hstart+hend)/2;
            Point_2 r = hpath.at(middle).location;
            trajectory *tmp1,*tmp2;
            // Left
            tmp1 = nonHolonomicPathRecursive(t,lnd,p,r,cobst,hpath,hstart,middle,verbose);
            if (!tmp1) return NULL;
            trajectory *p1 = tmp1->rightest();
            // Right
            tmp2 = nonHolonomicPathRecursive(t,lnd,r,q,cobst,hpath,middle,hend  ,verbose);
            if (!tmp2) return NULL;
            trajectory *p2 = tmp2;
            if (p1->size()>=1) {
                Point_3 last = p1->at(p1->size()-1);
                if (p2->size()>=1) {
                    Point_3 first  = p2->at(0);
                    double a1      = last.z();
                    double a2      = first.z();
                    while (a1>+M_PI) a1 -= 2*M_PI;
                    while (a1<-M_PI) a1 += 2*M_PI;
                    while (a2>+M_PI) a2 -= 2*M_PI;
                    while (a2<-M_PI) a2 += 2*M_PI;
                    // Add another rotation
                    p1->addRotation(lnd,
                                    sqrt((first.x()-lnd.x())*
                                         (first.x()-lnd.x())+
                                         (first.y()-lnd.y())*
                                         (first.y()-lnd.y())),
                                    atan2((first.y()-lnd.y()),
                                          (first.x()-lnd.x())),a1,a2);
                    p1->next->id   = p1->id;
                }
            }
            p1       = tmp1->rightest();
            root     = tmp1;
            p1->next = tmp2;
        }
        return root;
    }
    
    // Determine the relative situation of p and q, with all primitives
    void trajectory::nonHolonomicFreePathComplete(const Point_2 &lnd,
                                                  const Point_2 &p,
                                                  const Point_2 &q) {
        
        
        //
        clear();
        // Angle pOq: tq
        double a =
        -atan2((p.y()-lnd.y()),(p.x()-lnd.x()))
        +atan2((q.y()-lnd.y()),(q.x()-lnd.x()));
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
        double d  = -2*tphi2*log(sin(phi2));
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
        if ((tq>0 && tq< d      && rq<rp*exp(-tq/t2)) ||
            (tq>d && tq< d+phi2 && rq<rp*sin(phi2)*sin(phi2+d-tq))) {
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
        if ((tq<0  && tq>-d      && rq<rp*exp( tq/t2)) ||
            (tq<-d && tq>-phi2-d && rq<rp*sin(phi2)*sin(phi2+d+tq))) {
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
        if ((tq>0 && tq< d      && rq>rp*exp(tq/t2)) ||
            (tq>d && tq< d+phi2 && rq>rp/(sin(phi2)*sin(phi2+d-tq)))) {
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
        if ((tq< 0 && -tq< d      && rq>rp*exp(tq/t1)) ||
            (tq<-d && -tq< d+phi2 && rq>rp/(sin(phi1)*sin(phi1-d-tq)))) {
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
        if ((tq>0 && tq< d   && rq>rp*exp(-tq/t2) && rq<rp*exp(tq/t2)) ||
            (tq>d && tq< d*2 && rq>rp*pow(sin(phi2),4.0)*exp(tq/t2) &&
             rq<rp*exp(-tq/t2)/pow(sin(phi2),4.0) )) {
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
        if (( tq<0 && -tq< d   && rq>rp*exp(-tq/t1) && rq<rp*exp(tq/t1)) ||
            (-tq>d && -tq< d*2 && rq>rp*pow(sin(phi2),4.0)*exp(tq/t1) &&
             rq<rp*exp(-tq/t1)/pow(sin(phi2),4.0) )) {
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
        if ((tq>d && tq<2*d && rq>rp*exp(-tq/t2)/pow(sin(phi2),4.0)) ||
            (tq>2*d && tq<phi2+2*d && rq>rp*sin(phi2)/sin(phi2+2*d-tq))) {
            double alphaW = solveDichotomyAWLS1S2(0,-tq,rq,rp,-tq);
            double alphaI = solveDichotomyLS1S2(0,alphaW,rq,rp,-tq);
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
        if ((tq<-d && -tq<2*d && rq>rp*exp(tq/t2)/pow(sin(phi2),4.0)) ||
            (-tq>2*d && -tq< phi2+2*d && rq>rp*sin(-phi2)/sin(-phi2-2*d-tq))) {
            double alphaW = solveDichotomyAWLS2S1(0,-tq,rq,rp,-tq);
            double alphaI = solveDichotomyLS2S1(0,alphaW,rq,rp,-tq);
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
        if (tq>d && tq<phi2+2*d && rq<rp*sin(phi2+2*d-tq)/sin(phi2))
        {
            double alphaW = solveDichotomyAWLS2S1(0,tq,rp,rq,tq);
            double alphaI = solveDichotomyLS2S1(0,alphaW,rp,rq,tq);
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
        if (tq<-d && tq>-phi2-2*d && rq<rp*sin(-phi2-2*d-tq)/sin(-phi2))
        {
            double alphaW = solveDichotomyAWLS1S2(0,tq,rp,rq,tq);
            double alphaI = solveDichotomyLS1S2(0,alphaW,rp,rq,tq);
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
        if (tq>0 && tq<2*phi2+2*d) {
            // Check if the two arcs of circle intersect
            double alphaW =
            (tq>2*phi2)?phi2:solveDichotomyAWLS2S1L(0,tq,rp,rq,tq);
            double alphaM =
            solveDichotomyLS2S1L(0,alphaW,rp,rq,tq);
            double rM     = rp*sin(phi2-alphaM)/sin(phi2);
            Point_2 pM(rM*cos(alphaM+ap)+lnd.x(),
                       rM*sin(alphaM+ap)+lnd.y());
            // Line
            makeLine(p,pM);
            // First spiral
            addSpiral2AtEnd(lnd,rM*exp(alphaM/t2),ap,alphaM,alphaM+d);
            // In-site rotation
            addRotationAtEnd(lnd,rM*exp(-d/t2),alphaM+d+ap,phi2,phi1);
            // Second spiral
            addSpiral1AtEnd(lnd,rM*exp((alphaM+2*d)/t1),ap,alphaM+d,alphaM+2*d);
            // Line
            Point_2 pM2(rM*cos(ap+alphaM+2*d)+lnd.x(),rM*sin(ap+alphaM+2*d)+lnd.y());
            // Line
            addLineAtEnd(lnd,pM2,q);
            return;
        }
        if (tq<0 && tq>-2*phi2-2*d) {
            // Check if the two arcs of circle intersect
            double alphaW =
            (-tq>2*phi2)?phi2:solveDichotomyAWLS2S1L(0,-tq,rp,rq,-tq);
            double alphaM =
            solveDichotomyLS2S1L(0,alphaW,rp,rq,-tq);
            double rM     = rp*sin(phi2-alphaM)/sin(phi2);
            Point_2 pM(rM*cos(-alphaM+ap)+lnd.x(),
                       rM*sin(-alphaM+ap)+lnd.y());
            // Line
            makeLine(p,pM);
            // First spiral
            addSpiral1AtEnd(lnd,rM*exp(alphaM/t2),ap,-alphaM,-alphaM-d);
            // In-site rotation
            addRotationAtEnd(lnd,rM*exp(d/t1),-alphaM-d+ap,phi1,phi2);
            // Second spiral
            addSpiral2AtEnd(lnd,rM*exp((-alphaM-2*d)/t2),ap,-alphaM-d,-alphaM-2*d);
            // Line
            Point_2 pM2(rM*cos(ap-alphaM-2*d)+lnd.x(),rM*sin(ap-alphaM-2*d)+lnd.y());
            // Line
            addLineAtEnd(lnd,pM2,q);
            return;
        }
        Point_2 pO(lnd.x(),lnd.y());
        // Line
        makeLine(p,pO);
        // Line
        addLineAtEnd(lnd,pO,q);
    }
    
    // Find closest point on the diagram
    template <class T, class U>
    Point_2 trajectory::closestPointDiagram(const Point_2 &p,const Point_2 &pp,
                                            const T& sdg,
                                            const vddrObstacles &cobst) {
        
        Point_2 pt;
        double dminp = std::numeric_limits<double>::max();
        U vd(sdg);
        typename U::Edge_iterator vit = vd.edges_begin();
        Line lp(p,pp);
        
        for (;vit != vd.edges_end();++vit) {
            if (vit->is_segment()) {
                if (vit->up()->info() != vit->down()->info()) {
                    Point_2 v1 = vit->source()->point();
                    Point_2 v2 = vit->target()->point();
                    if (cobst.isPointAdmissible(v1) ||
                        cobst.isPointAdmissible(v2)) {
                        Point_2 ip; Segment curEdge(v1,v2);
                        // Check if the line (p,pp) intersect with this segment
                        CGAL::Object obj =
                        CGAL::intersection(lp,curEdge.supporting_line());
                        if ( CGAL::assign(ip, obj) && curEdge.collinear_has_on(ip)) {
                            Segment s(p,ip);
                            if (!cobst.doIntersect(s)) {
                                double d = CGAL::squared_distance(ip,p);
                                if (d<dminp) {
                                    dminp = d;
                                    pt    = ip;
                                }
                            }
                        }
                    }
                }
            }
        }
        return pt;
    }
    
    // Resolve transcendental equation for the intersection
    double trajectory::solveDichotomyLS2S1L(const double &valInit,
                                            const double &valFinal,
                                            const double &rp,
                                            const double &rq,
                                            const double &aq,
                                            const double &q) {
        // Middle value
        double middle = 0.5*(valInit+valFinal);
        double d      = -2*tphi2*log(pow(sin(phi2),2.0)*2.0*sqrt(q)/(1.0+q));
        double val    = q*rq*sin(phi2+middle+d-aq) - rp*sin(phi2-middle);
        
        if (fabs(val)<EPSILON) {
            return middle;
        } else if (val<0) {
            return solveDichotomyLS2S1L(middle,valFinal,rp,rq,aq,q);
        } else {
            return solveDichotomyLS2S1L(valInit,middle,rp,rq,aq,q);
        }
    }
    
    // Resolve transcendental equation for the intersection
    double trajectory::solveDichotomyAWLS2S1L(const double &valInit,
                                              const double &valFinal,
                                              const double &rp,
                                              const double &rq,
                                              const double &aq,
                                              const double &q) {
        // Middle value
        double middle = 0.5*(valInit+valFinal);
        double val    = q*rq*sin(phi2+middle-aq) - rp*sin(phi2-middle);
        
        if (fabs(val)<EPSILON) {
            return middle;
        } else if (val<0) {
            return solveDichotomyAWLS2S1L(middle,valFinal,rp,rq,aq,q);
        } else {
            return solveDichotomyAWLS2S1L(valInit,middle,rp,rq,aq,q);
        }
    }
    
    // Resolve transcendental equation for the intersection
    double trajectory::solveDichotomyAWLS2S1(const double &valInit,
                                             const double &valFinal,
                                             const double &rp,
                                             const double &rq,
                                             const double &aq,
                                             const double &qval) {
        // Middle value
        double middle = 0.5*(valInit+valFinal);
        double val    = rq*exp((middle-aq)/tphi2) - rp*sin(phi2-middle)/sin(phi2);
        
        if (fabs(val)<EPSILON) {
            return middle;
        } else if (val<0) {
            return solveDichotomyAWLS2S1(middle,valFinal,rp,rq,aq,qval);
        } else {
            return solveDichotomyAWLS2S1(valInit,middle,rp,rq,aq,qval);
        }
    }
    
    // Resolve transcendental equation for the intersection
    double trajectory::solveDichotomyAWLS1S2(const double &valInit,
                                             const double &valFinal,
                                             const double &rp,
                                             const double &rq,
                                             const double &aq,
                                             const double &qval) {
        // Middle value
        double middle = 0.5*(valInit+valFinal);
        double val    = rq*exp(-(middle-aq)/tphi2) - rp*sin(-phi2-middle)/sin(-phi2);
        
        if (fabs(val)<EPSILON) {
            return middle;
        } else if (val<0) {
            return solveDichotomyAWLS1S2(middle,valFinal,rp,rq,aq,qval);
        } else {
            return solveDichotomyAWLS1S2(valInit,middle,rp,rq,aq,qval);
        }
    }
    
    
    // Resolve transcendental equation for the intersection
    double trajectory::solveDichotomyLS2S1(const double &valInit,
                                           const double &valFinal,
                                           const double &rp,
                                           const double &rq,
                                           const double &aq,
                                           const double &qval) {
        
        // Middle value
        double middle = 0.5*(valInit+valFinal);
        double val    = pow(0.5*(1+qval),2.0)*
        exp((middle-aq)/tphi2)/(pow(sin(phi2),3.0)*sin(phi2-middle))-rp/rq;
        
        
        if (fabs(val)<EPSILON) {
            return middle;
        } else if (val<0) {
            return solveDichotomyLS2S1(middle,valFinal,rp,rq,aq,qval);
        } else {
            return solveDichotomyLS2S1(valInit,middle,rp,rq,aq,qval);
        }
    }
    
    // Resolve transcendental equation for the intersection
    double trajectory::solveDichotomyLS1S2(const double &valInit,
                                           const double &valFinal,
                                           const double &rp,
                                           const double &rq,
                                           const double &aq,
                                           const double &qval) {
        
        // Middle value
        double middle = 0.5*(valInit+valFinal);
        double val    = pow(0.5*(1+qval),2.0)*
        exp(-(middle-aq)/tphi2)/(pow(-sin(phi2),3.0)*sin(-phi2-middle))
        - rp/rq;
        
        if (fabs(val)<EPSILON) {
            return middle;
        } else if (val<0) {
            return solveDichotomyLS1S2(middle,valFinal,rp,rq,aq,qval);
        } else {
            return solveDichotomyLS1S2(valInit,middle,rp,rq,aq,qval);
        }
    }
    
    
    // Resolve transcendental equation for the intersection
    double trajectory::solveDichotomyLS2(const double &valInit,
                                         const double &valFinal,
                                         const double &rp,
                                         const double &rq,
                                         const double &aq) {
        
        // Middle value
        double middle = 0.5*(valInit+valFinal);
        double val    = rp*sin(phi2-middle)/sin(phi2) - rq*exp((aq-middle)/tphi2);
        
        if (fabs(val)<EPSILON) {
            return middle;
        } else if (val>0) {
            return solveDichotomyLS2(middle,valFinal,rp,rq,aq);
        } else {
            return solveDichotomyLS2(valInit,middle,rp,rq,aq);
        }
    }
    
    
    // Resolve transcendental equation for the intersection
    double trajectory::solveDichotomyLS1(const double &valInit,
                                         const double &valFinal,
                                         const double &rp,
                                         const double &rq,
                                         const double &aq) {
        
        // Middle value
        double middle = 0.5*(valInit+valFinal);
        double val    = rp*sin(phi1-middle)/sin(phi1) - rq*exp((aq-middle)/tphi1);
        
        if (fabs(val)<EPSILON) {
            return middle;
        } else if (val<0) {
            return solveDichotomyLS1(middle,valFinal,rp,rq,aq);
        } else {
            return solveDichotomyLS1(valInit,middle,rp,rq,aq);
        }
    }
    
    
    // print trajectory type
    std::ostream& operator << (std::ostream& outs,
                               const trajectory::pathType &p) {
        
        switch (p) {
            case trajectory::LINE:
                outs << "LINE" ;
                break;
            case trajectory::SPIRAL2:
                outs << "SPIRAL2" ;
                break;
            case trajectory::SPIRAL1:
                outs << "SPIRAL1" ;
                break;
            case trajectory::ROTATION:
                outs << "INSITEROTATION";
                break;
            case trajectory::CHANGELANDMARK:
                outs << "LANDMARKCHANGE";
                break;
        }
        return outs;
    }
    
    
    
    // Draw underlying grid of local optimal orientations into a painter
    void trajectory::computeOrientationGrid(const Point_2  &ss,
                                            const Point_2 &lnd) {
        Point_2 op,o;
        double delta  = orientationGridDelta;
        // Polar coordinates of ss
        double rp= sqrt((ss.x()-lnd.x())*(ss.x()-lnd.x())+(ss.y()-lnd.y())*(ss.y()-lnd.y()));
        double ap= atan2(ss.y()-lnd.y(),ss.x()-lnd.x());
        // Express points in the O,p frame
        Point_2 pp(rp,0.0);
        // Constants to be used here
        double t1 = tphi1;
        double t2 = tphi2;
        double d  = -2*tphi2*log(sin(phi2));
        // Allocate/de-allocate
        if (orientationGrid)
            delete[] orientationGrid;
        if (nhWeight)
            delete[] nhWeight;
        int fullSize    = 2*orientationGridSize+1;
        int ind         = -1;
        orientationGrid = new double[fullSize*fullSize];
        nhWeight        = new double[fullSize*fullSize];
        // Grid
        for (int i = -orientationGridSize; i<= orientationGridSize; i++)
            for (int j = -orientationGridSize; j<= orientationGridSize; j++) {
                ind++;
                op = Point_2(ss.x()+i*delta*cos(ap)-j*delta*sin(ap),
                             ss.y()+i*delta*sin(ap)+j*delta*cos(ap));
                // Angle pOq: tq
                double a =
                -atan2((ss.y()-lnd.y()),(ss.x()-lnd.x()))
                +atan2((op.y()-lnd.y()),(op.x()-lnd.x()));
                if (a> M_PI) a-=2*M_PI;
                if (a<-M_PI) a+=2*M_PI;
                // Polar coordinates of op
                double rq= sqrt((op.x()-lnd.x())*(op.x()-lnd.x())+
                                (op.y()-lnd.y())*(op.y()-lnd.y()));
                double tq= a;
                double aq= atan2(op.y()-lnd.y(),op.x()-lnd.x());
                // Express points in the O,p frame
                Point_2 qq(rq*cos(a),
                           rq*sin(a));
                // Angle Opq
                double b = atan2(qq.y()-pp.y(),
                                 qq.x()-pp.x());
                
                // Line segment : part on the right + 2 arcs of circles
                if ((b>-phi2 && b<-phi1) ||
                    (tq>=0 && tq< phi2 && rq<rp*sin(phi2-tq)/sin(phi2)) ||
                    (tq<=0 && tq>-phi2 && rq<rp*sin(phi2+tq)/sin(phi2))) {
                    orientationGrid[ind] =  (qq.x()<pp.x())?atan2(op.y()-ss.y(),op.x()-ss.x()):M_PI+atan2(op.y()-ss.y(),op.x()-ss.x());
                    nhWeight[ind]        = 1.0;
                    continue;
                }
                // Line segment and spiral 2
                double tqq = tq,rqq;
                if ((tq>0 && tq< d      && rq<rp*exp(-tq/t2)) ||
                    (tq>d && tq< d+phi2 && rq<rp*sin(phi2)*sin(phi2+d-tq))) {
                    orientationGrid[ind] = aq-phi2+M_PI;
                    do {
                        tqq  += 0.01;
                        rqq    = rq*exp((tq-tqq)/t2);
                    } while ((tqq>0 && tqq< d      && rqq<rp*exp(-tqq/t2)) ||
                             (tqq>d && tqq< d+phi2 && rqq<rp*sin(phi2)*sin(phi2+d-tqq)));
                    double l      = fabs(rqq-rq)/cos(phi2);
                    if (useHolonomic && l<rp/20.0)
                        nhWeight[ind]        = 0.0;
                    else
                        nhWeight[ind]        = 1.0;
                    continue;
                }
                // Line segment and spiral 1
                if ((tq<0  && tq>-d      && rq<rp*exp( tq/t2)) ||
                    (tq<-d && tq>-phi2-d && rq<rp*sin(phi2)*sin(phi2+d+tq))) {
                    orientationGrid[ind] = aq+phi2+M_PI;
                    nhWeight[ind]        = 1.0;
                    continue;
                }
                // Spiral 1 and line segment
                if ((tq>0 && tq< d      && rq>rp*exp(tq/t2)) ||
                    (tq>d && tq< d+phi2 && rq>rp/(sin(phi2)*sin(phi2+d-tq)))) {
                    double alphaI = solveDichotomyLS1(-tq,0,rq,rp,-tq);
                    double rI     = rp*exp((tq+alphaI)/t2);
                    Point_2 pI(rI*cos(ap+tq+alphaI)+lnd.x(),
                               rI*sin(ap+tq+alphaI)+lnd.y());
                    orientationGrid[ind] = atan2(pI.y()-op.y(),
                                                 pI.x()-op.x());
                    nhWeight[ind]        = 1.0;
                    continue;
                }
                // Spiral 2 and line segment
                if ((tq< 0 && -tq< d      && rq>rp*exp(tq/t1)) ||
                    (tq<-d && -tq< d+phi2 && rq>rp/(sin(phi1)*sin(phi1-d-tq)))) {
                    double alphaI = solveDichotomyLS2(0,-tq,rq,rp,-tq);
                    double rI     = rp*exp((tq+alphaI)/t1);
                    Point_2 pI(rI*cos(ap+tq+alphaI)+lnd.x(),
                               rI*sin(ap+tq+alphaI)+lnd.y());
                    orientationGrid[ind] = atan2(pI.y()-op.y(),
                                                 pI.x()-op.x());
                    nhWeight[ind]        = 1.0;
                    continue;
                }
                // Spiral 2 and spiral 1
                if ((tq>0 && tq< d   && rq>rp*exp(-tq/t2) && rq<rp*exp(tq/t2)) ||
                    (tq>d && tq< d*2 && rq>rp*pow(sin(phi2),4.0)*exp(tq/t2) &&
                     rq<rp*exp(-tq/t2)/pow(sin(phi2),4.0) )) {
                        orientationGrid[ind] = aq+phi2+M_PI;
                        double aip = t2*log(rp/rq)/2+tq/2;
                        double l = fabs(rq-rp*exp(-aip/t2))/cos(phi2);
                        if (useHolonomic && l<rp/20.0)
                            nhWeight[ind]        = 0.0;
                        else
                            nhWeight[ind]        = 1.0;
                        continue;
                    }
                // Spiral 1 and spiral 2
                if (( tq<0 && -tq< d   && rq>rp*exp(-tq/t1) && rq<rp*exp(tq/t1)) ||
                    (-tq>d && -tq< d*2 && rq>rp*pow(sin(phi2),4.0)*exp(tq/t1) &&
                     rq<rp*exp(-tq/t1)/pow(sin(phi2),4.0) )) {
                        orientationGrid[ind] = aq-phi2+M_PI;
                        double aip = t1*log(rp/rq)/2+tq/2;
                        double l = fabs(rq-rp*exp(aip/t2))/cos(phi2);
                        if (useHolonomic && l<rp/20.0)
                            nhWeight[ind]        = 0.0;
                        else
                            nhWeight[ind]        = 1.0;
                        continue;
                    }
                // Spiral 2, spiral 1 and line
                if ((tq>d && tq<2*d && rq>rp*exp(-tq/t2)/pow(sin(phi2),4.0)) ||
                    (tq>2*d && tq<phi2+2*d && rq>rp*sin(phi2)/sin(phi2+2*d-tq))) {
                    double alphaW = solveDichotomyAWLS1S2(0,-tq,rq,rp,-tq);
                    double alphaI = solveDichotomyLS1S2(0,alphaW,rq,rp,-tq);
                    double rI     = rq*sin(-phi2-alphaI)/sin(-phi2);
                    Point_2 pI(rI*cos(alphaI+aq)+lnd.x(),
                               rI*sin(alphaI+aq)+lnd.y());
                    orientationGrid[ind] = atan2(pI.y()-op.y(),
                                                 pI.x()-op.x());
                    nhWeight[ind]        = 1.0;
                    continue;
                }
                
                // Spiral 1, spiral 2 and line
                if ((tq<-d && -tq<2*d && rq>rp*exp(tq/t2)/pow(sin(phi2),4.0)) ||
                    (-tq>2*d && -tq< phi2+2*d && rq>rp*sin(-phi2)/sin(-phi2-2*d-tq))) {
                    double alphaW = solveDichotomyAWLS2S1(0,-tq,rq,rp,-tq);
                    double alphaI = solveDichotomyLS2S1(0,alphaW,rq,rp,-tq);
                    double rI     = rq*sin(phi2-alphaI)/sin(phi2);
                    Point_2 pI(rI*cos(alphaI+aq)+lnd.x(),
                               rI*sin(alphaI+aq)+lnd.y());
                    orientationGrid[ind] = atan2(pI.y()-op.y(),
                                                 pI.x()-op.x());
                    nhWeight[ind]        = 1.0;
                    continue;
                }
                // Line, spiral 2, spiral 1
                if (tq>d && tq<phi2+2*d && rq<rp*sin(phi2+2*d-tq)/sin(phi2)) {
                    orientationGrid[ind] = aq+phi2+M_PI;
                    double alphaW = solveDichotomyAWLS2S1(0,tq,rp,rq,tq);
                    double alphaI = solveDichotomyLS2S1(0,alphaW,rp,rq,tq);
                    double rI     = rp*sin(phi2-alphaI)/sin(phi2);
                    // Two spirals: find intersection point, first
                    double aip = t2*log(rI/rq)/2+(tq+alphaI)/2;
                    double l   = fabs(rI*exp((alphaI-aip)/t2)-rq)/cos(phi2);
                    if (useHolonomic && l<rp/20.0)
                        nhWeight[ind]        = 0.0;
                    else
                        nhWeight[ind]        = 1.0;
                    continue;
                }
                // Line, spiral 1, spiral 2
                if (tq<-d && tq>-phi2-2*d && rq<rp*sin(-phi2-2*d-tq)/sin(-phi2)) {
                    orientationGrid[ind] = aq-phi2+M_PI;
                    double alphaW = solveDichotomyAWLS1S2(0,tq,rp,rq,tq);
                    double alphaI = solveDichotomyLS1S2(0,alphaW,rp,rq,tq);
                    double rI     = rp*sin(-phi2-alphaI)/sin(-phi2);
                    double aip = -t2*log(rI/rq)/2+(tq+alphaI)/2;
                    double l   = fabs(rI*exp(-(alphaI-aip)/t2)-rq)/cos(phi2);
                    if (useHolonomic && l<rp/20.0)
                        nhWeight[ind]        = 0.0;
                    else
                        nhWeight[ind]        = 1.0;
                    continue;
                }
                // Line, spiral 1, spiral 2, line
                if (tq>0 && tq<2*phi2+2*d) {
                    double alphaW =
                    (tq>2*phi2)?phi2:solveDichotomyAWLS2S1L(0,tq,rp,rq,tq);
                    double alphaM =
                    solveDichotomyLS2S1L(0,alphaW,rp,rq,tq);
                    double rM     = rp*sin(phi2-alphaM)/sin(phi2);
                    Point_2 pM2(rM*cos(ap+alphaM+2*d)+lnd.x(),
                                rM*sin(ap+alphaM+2*d)+lnd.y());
                    orientationGrid[ind] = atan2(pM2.y()-op.y(),
                                                 pM2.x()-op.x());
                    nhWeight[ind]        = 1.0;
                    continue;
                }
                // Line, spiral 2, spiral 1, line
                if (tq<0 && tq>-2*phi2-2*d) {
                    double alphaW =
                    (-tq>2*phi2)?phi2:solveDichotomyAWLS2S1L(0,-tq,rp,rq,-tq);
                    double alphaM =
                    solveDichotomyLS2S1L(0,alphaW,rp,rq,-tq);
                    double rM     = rp*sin(phi2-alphaM)/sin(phi2);
                    // Line
                    Point_2 pM2(rM*cos(ap-alphaM-2*d)+lnd.x(),
                                rM*sin(ap-alphaM-2*d)+lnd.y());
                    orientationGrid[ind] = atan2(pM2.y()-op.y(),
                                                 pM2.x()-op.x());
                    nhWeight[ind]        = 1.0;
                    continue;
                }
                orientationGrid[ind] = 100.0;
                orientationGrid[ind] = 100.0;
            }
        
        
    }
    
    // Draw underlying grid of local optimal orientations into a painter
    void trajectory::drawOrientationGrid(const Point_2  &ss,
                                         const Point_2 &lnd,
                                         QPainter *painter,
                                         QRectF &bRect) {
        
        CGAL::Qt::PainterOstream<CK> painterostream =
        CGAL::Qt::PainterOstream<CK>(painter, bRect);
        painter->setBrush(QBrush(QColor(0,0,0,100)));
        Point_2 op,o;
        double ap= atan2(ss.y(),ss.x());
        double radius = (orientationGridDelta/20.0)*(orientationGridDelta/20.0);
        double arrow  = orientationGridDelta/2.0;
        if (!orientationGrid) return;
        int ind = 0;
        // Grid
        for (int i = -orientationGridSize; i<= orientationGridSize; i++)
            for (int j = -orientationGridSize; j<= orientationGridSize; j++) {
                op = Point_2(ss.x()
                             +i*orientationGridDelta*cos(ap)
                             -j*orientationGridDelta*sin(ap),
                             ss.y()
                             +i*orientationGridDelta*sin(ap)
                             +j*orientationGridDelta*cos(ap));
                painter->setPen(QPen(QColor(0,0,0,100), 2, Qt::SolidLine));
                painterostream << Circle(op,radius);
                if (fabs(orientationGrid[ind])<100.0) {
                    o           = Point_2(op.x()+
                                          nhWeight[ind]*arrow*cos(orientationGrid[ind]),
                                          op.y()+
                                          nhWeight[ind]*arrow*sin(orientationGrid[ind]));
                    painterostream << Segment(op,o);
                    painter->setPen(QPen(QColor(0,0,255,100), 2, Qt::SolidLine));
                    double alp = atan2(op.y(),op.x());
                    if ((op.x()-ss.x())*(-sin(alp))+
                        (op.y()-ss.y())*cos(alp)>0)
                        alp += M_PI;
                    o           = Point_2(op.x()-
                                          (1.0-nhWeight[ind])*arrow*sin(alp),
                                          op.y()+
                                          (1.0-nhWeight[ind])*arrow*cos(alp));
                    painterostream << Segment(op,o);
                }
                ind++;
            }
    }
    
    // Draw underlying partition into a painter
    void trajectory::drawPartition(const Point_2  &ss,
                                   const Point_2 &land,
                                   QPainter *painter,
                                   QRectF &bRect) {
        double step = 0.015;
        double rp= sqrt(ss.x()*ss.x()+ss.y()*ss.y());
        double ap= atan2(ss.y(),ss.x());
        double d = -2*trajectory::tphi2*log(sin(trajectory::phi2));
        double t2= trajectory::tphi2;
        double rmax=5.0*sqrt((ss.x()-land.x())*(ss.x()-land.x())+
                             (ss.y()-land.y())*(ss.y()-land.y())),
        rit;
        Point_2 o,op,q,qq,qqq,s,sp,z =  Point_2(0.0,0.0);
        CGAL::Qt::PainterOstream<CK> painterostream =
        CGAL::Qt::PainterOstream<CK>(painter, bRect);
        ///////////////////////////
        // Red
        painter->setPen(Qt::NoPen);
        painter->setBrush(QBrush(QColor(255, 0, 0,100)));
        // Spiral S2(Ps)
        for (double ait=d;ait<=d*2;ait+=step) {
            rit= std::min(rmax,rp*exp(-ait/t2)/(pow(sin(trajectory::phi2),4.0)));
            op = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            rit= std::min(rmax,fabs(rp/(sin(trajectory::phi2)*sin(trajectory::phi2+d-ait))));
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait>d) {
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op;
        }
        for (double ait=2*d;ait<=2*d+trajectory::phi2;ait+=step) {
            rit = rmax;
            if (ait<trajectory::phi2+d) {
                rit = std::min(rmax,fabs(rp/(sin(trajectory::phi2)*sin(trajectory::phi2+d-ait))));
            }
            o   = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            rit = std::min(rmax,
                           rp*sin(trajectory::phi2)/sin(trajectory::phi2+2*d-ait));
            op  = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            painterostream << Triangle(s,sp,op);
            painterostream << Triangle(op,o,s);
            s  = o;
            sp = op;
        }
        for (double ait=-d;ait>=-d*2;ait-=step) {
            rit= std::min(rmax,rp*exp(ait/t2)/(pow(sin(trajectory::phi2),4.0)));
            op = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            rit= std::min(rmax,fabs(rp/(sin(-trajectory::phi2)*sin(-trajectory::phi2-d-ait))));
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait<-d) {     
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        for (double ait=-2*d;ait>=-2*d-trajectory::phi2;ait-=step) {
            rit = rmax;
            if (ait>-trajectory::phi2-d) {
                rit = std::min(rmax,fabs(rp/(sin(-trajectory::phi2)*sin(-trajectory::phi2-d-ait))));
            }
            o   = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            rit = std::min(rmax,
                           rp*sin(-trajectory::phi2)/sin(-trajectory::phi2-2*d-ait));
            op  = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            painterostream << Triangle(s,sp,op);
            painterostream << Triangle(op,o,s);
            s  = o;
            sp = op; 
        }
        
        ///////////////////////////
        // Blue (exterior)
        painter->setPen(Qt::NoPen);
        painter->setBrush(QBrush(QColor(0, 0, 255,100)));
        for (double ait=0;ait<=d;ait+=step) {
            rit= std::min(rmax,rp*exp(ait/t2));
            o   = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            rit = rmax;
            if (ait<trajectory::phi2)
                rit = std::min(rmax,
                               fabs(rp*sin(trajectory::phi2)/sin(trajectory::phi2-ait)));
            op  = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            if (ait>0) {     
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        for (double ait=d;ait<=trajectory::phi2+d;ait+=step) {
            rit= std::min(rmax,fabs(rp/(sin(trajectory::phi2)*sin(trajectory::phi2+d-ait))));
            o   = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            rit = rmax;
            if (ait<phi2) {
                rit = std::min(rmax,
                               fabs(rp*sin(trajectory::phi2)/sin(trajectory::phi2-ait)));
            }
            op  = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            painterostream << Triangle(s,sp,op);
            painterostream << Triangle(op,o,s);
            s  = o;
            sp = op; 
        }
        for (double ait=0;ait>=-d;ait-=step) {
            rit= std::min(rmax,rp*exp(-ait/t2));
            o   = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            rit = rmax;
            if (ait>-trajectory::phi2)
                rit = std::min(rmax,
                               fabs(rp*sin(-trajectory::phi2)/sin(-trajectory::phi2-ait)));
            op  = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            if (ait<0) {     
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        for (double ait=-d;ait>=-trajectory::phi2-d;ait-=step) {
            rit= std::min(rmax,fabs(rp/(sin(-trajectory::phi2)*sin(-trajectory::phi2-d-ait))));
            o   = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            rit = rmax;
            if (ait>-phi2) {
                rit = std::min(rmax,
                               fabs(rp*sin(-trajectory::phi2)/sin(-trajectory::phi2-ait)));
            }
            op  = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            painterostream << Triangle(s,sp,op);
            painterostream << Triangle(op,o,s);
            s  = o;
            sp = op; 
        }
        
        ///////////////////////////
        // Magenta
        painter->setPen(Qt::NoPen);
        painter->setBrush(QBrush(QColor(255, 0, 255,100)));
        // Arc of circle C3(Pi)
        for (double ait=2*d;ait<=std::min(M_PI,2*d+trajectory::phi2);ait+=step) {
            rit= rp*sin(trajectory::phi2+2*d-ait)/sin(trajectory::phi2);
            o   = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            rit = std::min(rmax,
                           rp*sin(trajectory::phi2)/sin(trajectory::phi2+2*d-ait));
            op  = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            if (ait>2*d) {     
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        for (double ait=std::min(M_PI,2*d+trajectory::phi2);
             ait<=std::min(M_PI,2*d+2*trajectory::phi2);ait+=step) {
            o   = Point_2(rmax*cos(ait+ap),
                          rmax*sin(ait+ap));
            painterostream << Triangle(z,o,op);
            op  = o;
        }
        for (double ait=-2*d;ait>=std::max(-M_PI,-2*d-trajectory::phi2);ait-=step) {
            rit= rp*sin(-trajectory::phi2-2*d-ait)/sin(-trajectory::phi2);
            o   = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            rit = std::min(rmax,
                           rp*sin(-trajectory::phi2)/sin(-trajectory::phi2-2*d-ait));
            op  = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            if (ait<-2*d) {     
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        for (double ait=std::max(-M_PI,-2*d-trajectory::phi2);
             ait>=std::max(-M_PI,-2*d-2*trajectory::phi2);ait-=step) {
            o   = Point_2(rmax*cos(ait+ap),
                          rmax*sin(ait+ap));
            painterostream << Triangle(z,o,op);
            op  = o;
        }
        
        ///////////////////////////
        // Cyan
        painter->setPen(Qt::NoPen);
        painter->setBrush(QBrush(QColor(0, 255, 255,100)));
        // Spiral S1(Pi)
        for (double ait=0;ait<=d;ait+=step) {
            rit= std::min(rmax,rp*exp(ait/t2));
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            rit = std::min(rmax,rp*exp(-ait/t2));
            op  = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            if (ait>0) {
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        // Spiral S2(Ps)
        for (double ait=d;ait<=d*2;ait+=step) {
            rit= std::min(rmax,rp*exp(-ait/t2)/(pow(sin(trajectory::phi2),4.0)));
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            rit= std::min(rmax,rp*pow(sin(trajectory::phi2),4.0)*exp(ait/t2));
            op = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait>0) {
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        // Spiral S1(Pi)
        for (double ait=0;ait>=-d;ait-=step) {
            rit= std::min(rmax,rp*exp(-ait/t2));
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            rit = std::min(rmax,rp*exp(ait/t2));
            op  = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            if (ait<0) {
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        // Spiral S2(Ps)
        for (double ait=-d;ait>=-d*2;ait-=step) {
            rit= std::min(rmax,rp*exp(ait/t2)/(pow(sin(trajectory::phi2),4.0)));
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            rit= std::min(rmax,rp*pow(sin(trajectory::phi2),4.0)*exp(-ait/t2));
            op = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait<0) {
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        
        ///////////////////////////
        // Yellow
        painter->setPen(Qt::NoPen);
        painter->setBrush(QBrush(QColor(255,255,0,100)));
        // Spiral S1(Pr)
        for (double ait=d;ait<=2*d;ait+=step) {
            rit= rp*pow(sin(trajectory::phi2),4.0)*exp(ait/t2);
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait<=trajectory::phi2+d) {
                rit= rp*(sin(trajectory::phi2)*sin(trajectory::phi2+d-ait));
                op = Point_2(rit*cos(ait+ap),
                             rit*sin(ait+ap));
            } else {
                op = z;
            }
            if (ait>d) {
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        // Arc of circle C3(Pi)
        for (double ait=2*d;ait<=2*d+trajectory::phi2;ait+=step) {
            rit= rp*sin(trajectory::phi2+2*d-ait)/sin(trajectory::phi2);
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait<=trajectory::phi2+d) {
                rit= rp*(sin(trajectory::phi2)*sin(trajectory::phi2+d-ait));
                op = Point_2(rit*cos(ait+ap),
                             rit*sin(ait+ap));
            } else {
                op = z;
            }
            painterostream << Triangle(s,sp,op);
            painterostream << Triangle(op,o,s);
            s  = o;
            sp = op; 
        }
        // Spiral S1(Pr)
        for (double ait=-d;ait>=-2*d;ait-=step) {
            rit= rp*pow(sin(trajectory::phi2),4.0)*exp(-ait/t2);
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait>=-trajectory::phi2-d) {
                rit= rp*(-sin(trajectory::phi2)*sin(-trajectory::phi2-d-ait));
                op = Point_2(rit*cos(ait+ap),
                             rit*sin(ait+ap));
            } else {
                op = z;
            }
            if (ait<-d) {
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        // Arc of circle C3(Pi)
        for (double ait=-2*d;ait>=-2*d-trajectory::phi2;ait-=step) {
            rit= rp*sin(-trajectory::phi2-2*d-ait)/sin(-trajectory::phi2);
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait>=-trajectory::phi2-d) {
                rit= rp*(sin(-trajectory::phi2)*sin(-trajectory::phi2-d-ait));
                op = Point_2(rit*cos(ait+ap),
                             rit*sin(ait+ap));
            } else {
                op = z;
            }
            painterostream << Triangle(s,sp,op);
            painterostream << Triangle(op,o,s);
            s  = o;
            sp = op; 
        }
        
        ///////////////////////////
        // Blue (interior)
        painter->setPen(Qt::NoPen);
        painter->setBrush(QBrush(QColor(0,0,255,100)));
        // Spiral S2(Pi)
        for (double ait=0;ait<=d;ait+=step) {
            rit= rp*exp(-ait/t2);
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            rit= 0;
            if (ait<trajectory::phi2)
                rit= rp*sin(trajectory::phi2-ait)/sin(trajectory::phi2);
            op = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait>0) {
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        // Arc of circle C2(Pi)
        for (double ait=d;ait<=d+trajectory::phi2;ait+=step) {
            rit= rp*(sin(trajectory::phi2)*sin(trajectory::phi2+d-ait));
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait<=trajectory::phi2) {
                rit= rp*sin(trajectory::phi2-ait)/sin(trajectory::phi2);
                op = Point_2(rit*cos(ait+ap),
                             rit*sin(ait+ap));
            } else {
                op = z;
            }
            painterostream << Triangle(s,sp,op);
            painterostream << Triangle(op,o,s);
            s  = o;
            sp = op; 
        }
        // Spiral S2(Pi)
        for (double ait=0;ait>=-d;ait-=step) {
            rit= rp*exp(ait/t2);
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            rit= 0;
            if (ait>-trajectory::phi2)
                rit= rp*sin(-trajectory::phi2-ait)/sin(-trajectory::phi2);
            op = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait<0) {
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        // Arc of circle C2(Pi)
        for (double ait=-d;ait>=-d-trajectory::phi2;ait-=step) {
            rit= rp*(sin(-trajectory::phi2)*sin(-trajectory::phi2-d-ait));
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait>=-trajectory::phi2) {
                rit= rp*sin(-trajectory::phi2-ait)/sin(-trajectory::phi2);
                op = Point_2(rit*cos(ait+ap),
                             rit*sin(ait+ap));
            } else {
                op = z;
            }
            painterostream << Triangle(s,sp,op);
            painterostream << Triangle(op,o,s);
            s  = o;
            sp = op; 
        }
        
        ///////////////////////////
        // Green
        painter->setPen(Qt::NoPen);
        painter->setBrush(QBrush(QColor(0,255,0,100)));
        // Arcs of circle C1(Pi)
        for (double ait=0;ait<=trajectory::phi2;ait+=step) {
            rit= rp*sin(trajectory::phi2-ait)/sin(trajectory::phi2);
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait>0)
                painterostream << Triangle(o,s,z);
            s = o;
        }
        // Line  D1(Pi)
        for (double ait=0;ait<trajectory::phi2;ait+=step) {
            o   = Point_2(rmax*cos(ait+ap),
                          rmax*sin(ait+ap));
            rit = std::min(rmax,
                           fabs(rp*sin(trajectory::phi2)/sin(trajectory::phi2-ait)));
            op  = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            if (ait>0) {     
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        // Arcs of circle C1(Pi)
        for (double ait=0;ait>=-trajectory::phi2;ait-=step) {
            rit= rp*sin(-trajectory::phi2-ait)/sin(-trajectory::phi2);
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            if (ait<0)
                painterostream << Triangle(o,s,z);
            s = o;
        }
        // Line  D1(Pi)
        for (double ait=0;ait>-trajectory::phi2;ait-=step) {
            o   = Point_2(rmax*cos(ait+ap),
                          rmax*sin(ait+ap));
            rit = std::min(rmax,
                           fabs(rp*sin(-trajectory::phi2)/sin(-trajectory::phi2-ait)));
            op  = Point_2(rit*cos(ait+ap),
                          rit*sin(ait+ap));
            if (ait<0) {     
                painterostream << Triangle(s,sp,op);
                painterostream << Triangle(op,o,s);
            }
            s  = o;
            sp = op; 
        }
        
        /////////////////////////
        // Same on the other side
        // Spiral S2(Pi)
        s = ss;
        for (double ait=0;ait>=-d;ait-=step) {
            rit= rp*exp(ait/t2);
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            painterostream << Segment(o,s);
            s = o;
        }
        q = s;
        // Arc of circle C2(Pi)
        for (double ait=-d;ait>=-d-trajectory::phi2;ait-=step) {
            rit= rp*(sin(-trajectory::phi2)*sin(-trajectory::phi2-d-ait));
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            painterostream << Segment(o,s);
            s = o;
        }
        s = q;
        // Spiral S1(Pr)
        for (double ait=-d;ait>=-2*d;ait-=step) {
            rit= rp*pow(sin(trajectory::phi2),4.0)*exp(-ait/t2);
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            painterostream << Segment(o,s);
            s = o;
        }
        // Arc of circle C3(Pi)
        for (double ait=-2*d;ait>=-2*d-trajectory::phi2;ait-=step) {
            rit= rp*sin(-trajectory::phi2-2*d-ait)/sin(-trajectory::phi2);
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            painterostream << Segment(o,s);
            s = o;
        }
        // Spiral S1(Pi)
        s = ss;
        for (double ait=0;ait>=-d;ait-=step) {
            rit= rp*exp(-ait/t2);
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            painterostream << Segment(o,s);
            s = o;
        }
        qq   = s;
        // Spiral S2(Ps)
        for (double ait=-d;ait>=-d*2;ait-=step) {
            rit= rp*exp(ait/t2)/(pow(sin(trajectory::phi2),4.0));
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            painterostream << Segment(o,s);
            s = o;
        }
        qqq = s;
        // Line  D1(Pi)
        s = ss; rit = 0;
        for (double ait=0;rit<=rmax;ait-=step) {
            rit= rp*sin(-trajectory::phi2)/(sin(-trajectory::phi2-ait));
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            painterostream << Segment(o,s);
            s = o;
        }
        // Line  D2(Pi)
        s = qq; rit = 0;
        for (double ait=-d;rit<=rmax;ait-=step) {
            rit= rp/(sin(-trajectory::phi2)*sin(-trajectory::phi2-d-ait));
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            painterostream << Segment(o,s);
            s = o;
        }
        // Line  D3(Pi)
        s = qqq; rit = 0;
        for (double ait=-2*d;rit<=rmax;ait-=step) {
            rit= rp*sin(-trajectory::phi2)/sin(-trajectory::phi2-2*d-ait);
            o  = Point_2(rit*cos(ait+ap),
                         rit*sin(ait+ap));
            painterostream << Segment(o,s);
            s = o;
        }
        // Line D4(Pi)
        if (2*trajectory::phi2+2*d<M_PI) {
            s =  Point_2(0,0);
            o  = Point_2(rmax*cos(2*trajectory::phi2+2*d+ap),
                         rmax*sin(2*trajectory::phi2+2*d+ap));
            painterostream << Segment(o,s);
            s =  Point_2(0,0);
            o  = Point_2(rmax*cos(-2*trajectory::phi2-2*d+ap),
                         rmax*sin(-2*trajectory::phi2-2*d+ap));
            painterostream << Segment(o,s);
        }
    }
    
    // Add a line
    void trajectory::addLineAtEnd(const Point_2 &lnd,
                                  const Point_2 &p,
                                  const Point_2 &q) {
        trajectory *t = this;
        while (t->next!=NULL) t = t->next;
        t->next = new trajectory(lnd);
        t->next->makeLine(p,q);
    }
    
    // Make of this trajectory a line from p to q
    void trajectory::makeLine(const Point_2 &p,
                              const Point_2 &q) {
        // Clear
        clear();
        // Id
        id = idCount;
        // Line
        double the= atan2(q.y()-p.y(),
                          q.x()-p.x());
        if (cos(atan2(p.y()-landmark.y(),
                      p.x()-landmark.x())-the+M_PI)<0) the+=M_PI;
        for (int i=0;i<=muestreos;i++) {
            double t  = double(i)/double(muestreos);
            double x  = (1-t)*p.x() + t*q.x();
            double y  = (1-t)*p.y() + t*q.y();
            push_back(Point_3(x,y,atan2(y-landmark.y(),x-landmark.x())-the+M_PI));
        }
        // Start and end point
        start = at(0);
        end   = at(size()-1);
        // Flag
        type = LINE;
    }
    
    // Add a rotation
    void trajectory::addRotationAtEnd(const Point_2 &lnd,
                                      const double &rp,
                                      const double &ap,
                                      const double &ai,
                                      const double &af) {
        trajectory *t = this;
        while (t->next!=NULL) t = t->next;
        t->next = new trajectory(lnd);
        t->next->makeRotation(rp,ap,ai,af);
    }
    
    // Add a rotation
    void trajectory::addRotation(const Point_2 &lnd,
                                 const double &rp,
                                 const double &ap,
                                 const double &ai,
                                 const double &af) {
        trajectory *t = this;
        if (t->next!=NULL) return;
        t->next = new trajectory(lnd);
        t->next->makeRotation(rp,ap,ai,af);
    }
    
    
    // Add a landmark change at end
    void trajectory::addLandmarkChangeAtEnd(const Point_2 &lnd1,
                                            const Point_2 &lnd2,
                                            const double &rp,
                                            const double &ap,
                                            const double &ai) {
        trajectory *t = this;
        while (t->next!=NULL) t = t->next;
        t->next = new trajectory(lnd1);
        t->next->makeLandmarkChange(rp,ap,ai,lnd2);
    }
    
    // Add a landmark change at end
    void trajectory::addLandmarkChange(const Point_2 &lnd1,
                                       const Point_2 &lnd2,
                                       const double &rp,
                                       const double &ap,
                                       const double &ai) {
        trajectory *t = this;
        if (t->next!=NULL){
            std::cerr << "Cannot insert motion" << std::endl;
            return;
        }
        t->next = new trajectory(lnd1);
        t->next->makeLandmarkChange(rp,ap,ai,lnd2);
    }
    
    // Make of this trajectory an in-site rotation
    void trajectory::makeLandmarkChange(const double &rp,
                                        const double &ap,
                                        const double &ai,
                                        const Point_2 &lnd) {
        // Clear
        clear();
        // Id
        id = idCount;
        for (int i=0;i<=muestreos;i++) {
            double t  = double(i)/double(muestreos);
            push_back(Point_3(rp*cos(ap)+landmark.x(),
                              rp*sin(ap)+landmark.y(),
                              ai));
            lPositions.push_back(Point_2((1-t)*landmark.x()+t*lnd.x(),
                                         (1-t)*landmark.y()+t*lnd.y()));
        }
        // Start and end point
        start = at(0);
        end   = at(size()-1);
        // Flag
        type =  CHANGELANDMARK;
    }
    
    
    // Make of this trajectory an in-site rotation
    void trajectory::makeRotation(const double &rp,
                                  const double &ap,
                                  const double &ai,
                                  const double &af) {
        // Clear
        clear();
        // Id
        id = idCount;
        // Rotation
        for (int i=0;i<=muestreos;i++) {
            double t  = double(i)/double(muestreos);
            double ait= (1-t)*ai + t*af;
            push_back(Point_3(rp*cos(ap)+landmark.x(),
                              rp*sin(ap)+landmark.y(),
                              ait));
        }      
        // Start and end point
        start = at(0);
        end   = at(size()-1);
        // Flag
        type = ROTATION;
    }
    
    // Add a spiral 2
    void trajectory::addSpiral2AtEnd(const Point_2 &lnd,
                                     const double &rp,
                                     const double &ap,
                                     const double &ai,
                                     const double &af) {
        trajectory *t = this;
        while (t->next!=NULL) t = t->next;
        t->next = new trajectory(lnd);
        t->next->makeSpiral2(rp,ap,ai,af);
    }
    
    // Make of this trajectory a spiral 2 from p to q
    void trajectory::makeSpiral2(const double &rp,
                                 const double &ap,
                                 const double &ai,
                                 const double &af) {
        // Clear
        clear();
        // Id
        id = idCount;
        // Spiral 2
        for (int i=0;i<=muestreos;i++) {
            double t  = double(i)/double(muestreos);
            double ait= (1-t)*ai+t*af;
            double rit= rp*exp(-ait/tphi2);
            push_back(Point_3(rit*cos(ait+ap)+landmark.x(),
                              rit*sin(ait+ap)+landmark.y(),
                              phi2));
        }
        // Start and end point
        start = at(0);
        end   = at(size()-1);
        // Flag
        type = SPIRAL2;
    }
    
    // Add a spiral 1
    void trajectory::addSpiral1AtEnd(const Point_2 &lnd,
                                     const double &rp,
                                     const double &ap,
                                     const double &ai,
                                     const double &af) {
        trajectory *t = this;
        while (t->next!=NULL) t = t->next;
        t->next = new trajectory(lnd);
        t->next->makeSpiral1(rp,ap,ai,af);
    }
    
    // Make of this trajectory a spiral 1 from p to q
    void trajectory::makeSpiral1(const double &rp,
                                 const double &ap,
                                 const double &ai,
                                 const double &af) {
        // Clear
        clear();
        // Id
        id = idCount;
        // Spiral 2
        for (int i=0;i<=muestreos;i++) {
            double t  = double(i)/double(muestreos);
            double ait= (1-t)*ai+t*af;
            double rit= rp*exp(-ait/tphi1);
            push_back(Point_3(rit*cos(ait+ap)+landmark.x(),
                              rit*sin(ait+ap)+landmark.y(),
                              phi1));
        }
        // Start and end point
        start = at(0);
        end   = at(size()-1);
        // Flag
        type = SPIRAL1;
    }
}
