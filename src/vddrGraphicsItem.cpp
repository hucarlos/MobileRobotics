#include <vddrGraphicsItem.h>

namespace vddr
{
    // Constructor
    vddrGraphicsItem::vddrGraphicsItem(vddrStructure *vS): vS(vS),lId(0),
    drawCObst(false),drawVis(false),drawDilated(false),drawCompleteVoronoi(false),
    drawVoronoi(true),drawConnectors(false) {
    }
    
    QRectF vddrGraphicsItem::boundingRect() const {
        return bounding_rect;
    }
    
    // Paint function
    void
    vddrGraphicsItem::paint(QPainter *painter,
                            const QStyleOptionGraphicsItem *option,
                            QWidget * widget) {
        static QColor black  = QColor(0,0,0);
        static QColor white  = QColor(255,255,255);
        static QColor yellow = QColor(255,255,0);
        static QColor red    = QColor(255,0,0);
        static QColor pink   = QColor(255,180,180);
        
        // Draw coloured polygons
        visibilityMutex.lock();
        if (vS)
            for (unsigned int j=0;j<vS->cPolygons.size();j++) {
                painter->setPen(QPen(QBrush(vS->cPolygonsColor[j]), 0,Qt::NoPen));
                draw_filled_poly(painter,vS->cPolygons[j],QPen(QBrush(vS->cPolygonsColor[j]), 0,Qt::NoPen),vS->cPolygonsColor[j]);
            }
        visibilityMutex.unlock();
        painter->setPen(this->edgesPen());
        
        // Draw CObst, i.e the region corresponding to both physical and
        // virtual obstacles (i.e. from visibility)
        // CObst
        // Thread safe
        if (drawCObst) {
            std::list<Polygon_with_holes_2> res;
            cobstMutex.lock();
            for (unsigned int j=0;j<vS->landmarks.size();j++)
                if (lId<0 || static_cast<int>(j)==lId) {
                    vS->landmarks[j].getCobst().polygons_with_holes (std::back_inserter (res));
                    for (std::list<Polygon_with_holes_2>::const_iterator it=res.begin();
                         it != res.end();it++) {
                        if (!it->is_unbounded())
                            draw_filled_poly(painter,it->outer_boundary(),edges_pen,black);
                        for (std::list<Polygon_2>::const_iterator lit=it->holes_begin();
                             lit!=it->holes_end();lit++)
                            draw_filled_poly(painter,*lit,edges_pen,white);
                    }
                }
            cobstMutex.unlock();
        }
        
        // Visibility
        // Thread safe
        if (drawVis) {
            if (!drawDilated) {
                visibilityMutex.lock();
                for (unsigned int j=0;j<vS->landmarks.size();j++) if (lId<0 || static_cast<int>(j)==lId) {
                    draw_filled_poly(painter,vS->landmarks[j].getFreeSpaceNotDilated().outer_boundary(),
                                     edges_pen,yellow);
                    
                }
                visibilityMutex.unlock();
            }
            else {
                cobstMutex.lock();
                for (unsigned int j=0;j<vS->landmarks.size();j++)
                    if (lId<0 || static_cast<int>(j)==lId) {
                        std::vector<Polygon_with_holes_2> res;
                        vS->landmarks[j].getFreeSpace().polygons_with_holes (std::back_inserter (res));
                        for (unsigned int i=0;i<res.size();i++) {
                            draw_filled_poly(painter,res[i].outer_boundary(),edges_pen,yellow);
                        }
                    }
                cobstMutex.unlock();
            }
            visibilityMutex.lock();
            for (unsigned int j=0;j<vS->landmarks.size();j++)
                if (lId<0 || static_cast<int>(j)==lId) {
                    for (std::list<Polygon_2_e>::const_iterator pit=vS->landmarks[j].getFreeSpaceNotDilated().holes_begin();
                         pit!=vS->landmarks[j].getFreeSpaceNotDilated().holes_end();pit++)
                        draw_filled_poly(painter,*pit,edges_pen,white);
                }
            visibilityMutex.unlock();
        }
        
        // Connectors
        // Thread safe
        if (drawConnectors) {
            QPen qp = QPen(QBrush(Qt::red), 6,Qt::SolidLine);
            painter->setPen(qp);
            connectorAreaMutex.lock();
            for (unsigned int j=0;j<vS->getConnectors().size();j++)
                if(lId<0 ||
                   static_cast<int>(vS->getConnectors()[j]->getFrom())==lId||
                   static_cast<int>(vS->getConnectors()[j]->getTo())==lId) {
                    draw_filled_poly(painter,vS->getConnectors()[j]->outer_boundary(),edges_pen,pink);
                    std::list<Polygon_2_e>::const_iterator lit;
                    for (lit = vS->getConnectors()[j]->holes_begin();lit!=vS->getConnectors()[j]->holes_end();lit++) {
                        const Polygon_2_e &p = *lit;
                        draw_filled_poly(painter,p,edges_pen,white);
                    }
                    connectorPointMutex.lock();
                    // Nodes
                    for (unsigned int m=0;m<vS->getConnectors()[j]->getNodes().size();m++)
                        draw_point(painter,vS->getConnectors()[j]->getNodes().at(m),edges_pen,red);
                    // Voronoi
                    QPen qp = QPen(QBrush(Qt::red), 4,Qt::SolidLine);
                    painter->setPen(qp);
                    CGAL::Qt::PainterOstream<CK> painterostream =
                    CGAL::Qt::PainterOstream<CK>(painter, boundingRect());
                    
                    // Paint nodes
                    for (unsigned int k=0;k<vS->getConnectors()[j]->getGraph()->V();k++) {
                        painterostream <<
                        CGAL::Circle_2<CK>(vS->getConnectors()[j]->getGraph()->getVertex(k).location,30.0);
                        // Edges from nodes
                        for (const Graph::Edge *nd = vS->getConnectors()[j]->getGraph()->getVertexEdges(k);
                             nd!=NULL;nd=nd->next) {
                            const std::vector<lPoint> &v = nd->intermediatePoints;
                            for (unsigned int l=0;l<v.size()-1;l++) {
                                painterostream << Segment(v.at(l).location,
                                                          v.at(l+1).location);
                            }
                        }
                    }
#ifdef DEBUG
                    // Voronoi
                    QPen qp2 = QPen(QBrush(Qt::blue), 2,Qt::SolidLine);
                    painter->setPen(qp2);
                    CGAL::Qt::PainterOstream<CK> painterostream2 =
                    CGAL::Qt::PainterOstream<CK>(painter, boundingRect());
                    // Paint all nodes
                    VD vd(vS->getConnectors()[j]->getDelaunay());
                    VD::Edge_iterator eit = vd.edges_begin();
                    for (eit = vd.edges_begin(); eit != vd.edges_end(); eit++)
                        if (eit->is_segment()) {
                            vddr::Segment           s;
                            CGAL::Object o = vS->getConnectors()[j]->getDelaunay().primal(eit->dual());
                            if (CGAL::assign(s, o))
                                painterostream2 << s;
                        }
#endif
                    connectorPointMutex.unlock();
                }
            connectorAreaMutex.unlock();
#ifdef DEBUG
            commonVisMutex.lock();
            qp = QPen(QBrush(Qt::blue), 8,Qt::SolidLine);
            painter->setPen(qp);
            if (vS->commonVis->size()) {
                unsigned int j=rand()%vS->commonVis->size();
                if (vS->commonVis.at(j)->size()) {
                    unsigned int k=rand()%(vS->commonVis->at(j).size());
                    draw_filled_poly(painter,vS->commonVis->at(j).at(k),edges_pen,yellow);
                }
            }
            commonVisMutex.unlock();
#endif
        }
        
        // Obstacles
        // Thread safe
        obstaclesMutex.lock();
        if (drawDilated) {
#if 1
            for (unsigned int i=0;i<vS->obstaclesDilated.size();i++) {
                draw_filled_poly(painter,(vS->obstaclesDilated)[i],edges_pen,red,Qt::BDiagPattern);
                draw_filled_poly(painter,(vS->obstacles)[i],edges_pen,red);
            }
#endif
        } else {
            for (unsigned int i=0;i<vS->obstacles.size();i++)
                draw_filled_poly(painter,(vS->obstacles)[i],edges_pen,red);
        }
        obstaclesMutex.unlock();
        
        // Voronois
        // Thread safe
        if (drawVoronoi) {
            QPen qp = QPen(QBrush(Qt::blue), 4,Qt::SolidLine);
            painter->setPen(qp);
            CGAL::Qt::PainterOstream<CK> painterostream =
            CGAL::Qt::PainterOstream<CK>(painter, boundingRect());
            voronoiMutex.lock();
            for (unsigned int j=0;j<vS->drawnGraph.size();j++)
                if (lId<0 || static_cast<int>(j)==lId) {
                    for (unsigned int k=0;k<vS->drawnGraph[j].size();k++)
                        try {
                            painterostream << vS->drawnGraph[j][k];
                        } catch (...) {
                        }
                }
            for (unsigned int j=0;j<vS->drawnGraphParabola.size();j++)
                if (lId<0 || static_cast<int>(j)==lId) {
                    for (unsigned int k=0;k<vS->drawnGraphParabola[j].size();k++) {
                        try {
                            float l =
                            fabs(vS->drawnGraphParabola[j][k].getP2().y()-
                                 vS->drawnGraphParabola[j][k].getP1().y())+
                            fabs(vS->drawnGraphParabola[j][k].getP2().x()-
                                 vS->drawnGraphParabola[j][k].getP1().x());
                            if (l>0.01)
                                painterostream << vS->drawnGraphParabola[j][k];
                        } catch (...) {
                        }
                    }
                }
            voronoiMutex.unlock();
        }
    }
    
    
    // Draw filled polygon
    bool vddrGraphicsItem::draw_filled_poly(QPainter *painter, 
                                            const Polygon_2 &sg,
                                            const QPen &b,
                                            const QColor &fillcol,
                                            const Qt::BrushStyle &qs) const {  
        CGAL::Qt::Converter<Traits> convert;
        painter->setPen(b);
        painter->setBrush(QBrush(fillcol,qs));
        QMatrix matrix = painter->matrix();
        painter->resetMatrix();
        QPolygonF poly(sg.size());
        for (unsigned int k=0;k<sg.size();k++) {
            poly[k] = matrix.map(convert(sg[k]));
        }
        painter->drawPolygon (poly); 
        painter->setMatrix(matrix);
        return true;
    }
    
    // Draw filled polygon
    bool vddrGraphicsItem::draw_filled_poly(QPainter *painter, 
                                            const Polygon_2_e &sg,
                                            const QPen &b,
                                            const QColor &fillcol,
                                            const Qt::BrushStyle &qs) const {  
        CGAL::Qt::Converter<Traits_e> convert;
        painter->setPen(edgesPen());
        painter->setBrush(QBrush(fillcol,qs));
        QMatrix matrix = painter->matrix();
        painter->resetMatrix();
        QPolygonF poly(sg.size());
        for (unsigned int k=0;k<sg.size();k++) {
            poly[k] = matrix.map(convert(sg[k]));
        }
        painter->drawPolygon (poly); 
        painter->setMatrix(matrix);
        return true;
    }
    
    // Draw filled polygon
    bool vddrGraphicsItem::draw_point(QPainter *painter, 
                                      const Point_2_e &p,
                                      const QPen &b,
                                      const QColor &fillcol,
                                      const Qt::BrushStyle &qs) const {  
        CGAL::Qt::Converter<Traits> convert;
        painter->setPen(edgesPen());
        painter->setBrush(QBrush(fillcol,qs));
        QMatrix matrix = painter->matrix();
        painter->resetMatrix();
        QPolygonF poly(16);
        for (int k=0;k<16;k++) {
            Point_2 q(to_double(p.x())+10*cos(k*M_PI/8.0),
                      to_double(p.y())+10*sin(k*M_PI/8.0));
            poly[k] = matrix.map(convert(q));
        }
        painter->drawPolygon (poly); 
        painter->setMatrix(matrix);
        return true;
    }
    
    // We let the bounding box only grow, so that when vertices get removed
    // the maximal bbox gets refreshed in the GraphicsView
    void vddrGraphicsItem::updateBoundingBox() {
        std::cerr << "*** Update bounding box" << std::endl;
        if(scene()){
            bounding_rect = CGAL::Qt::viewportsBbox(scene());
        } else {
            bounding_rect = QRectF();
        }
    }
    
    void vddrGraphicsItem::modelChanged() {
        updateBoundingBox();
        update();
    }

}