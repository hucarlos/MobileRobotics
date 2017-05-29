#include <vddrGraphicsQueryItem.h>

namespace vddr
{
    unsigned int vddrGraphicsQueryItem::t = 0;
    
    unsigned int vddrGraphicsQueryItem::animSpeed = 5;
    
    // Constructor
    vddrGraphicsQueryItem::vddrGraphicsQueryItem(vddrStructure *vS,
                                                 QGraphicsScene* s): lId(0),vS(vS),
    drawGraph(false),drawHolonomic(false), drawAnimate(true),drawPartition(false),drawSPGrid(false),saveSnapshotPng(false),saveSnapshotPdf(false),
    scene(s){
        updateBoundingBox();
    }
    
    // Get bounding rect
    QRectF vddrGraphicsQueryItem::boundingRect() const {
        return bounding_rect;
    }
    
    // The paint function
    void vddrGraphicsQueryItem::paintStep(QPainter *painter,
                                          const Point_2 &p,
                                          const double &th) {
        
        QPointF points[4] = {
            QPointF(p.x()+6.0*cos(th)-3.0*sin(th), p.y()+6.0*sin(th)+3.0*cos(th)),
            QPointF(p.x()+6.0*cos(th)+3.0*sin(th), p.y()+6.0*sin(th)-3.0*cos(th)),
            QPointF(p.x()-6.0*cos(th)+3.0*sin(th), p.y()-6.0*sin(th)-3.0*cos(th)),
            QPointF(p.x()-6.0*cos(th)-3.0*sin(th), p.y()-6.0*sin(th)+3.0*cos(th)),
        };
        painter->drawPolygon(points, 4, Qt::OddEvenFill);
    }
    
    // The paint function
    void vddrGraphicsQueryItem::paint(QPainter *painter,
                                      const QStyleOptionGraphicsItem *option,
                                      QWidget * widget) {
        static QColor black  = QColor(0,0,0);
        static QColor white  = QColor(255,255,255);
        static QColor yellow = QColor(255,255,0);
        static QColor red    = QColor(255,0,0);
        QPainter *usedPainter = painter;
        
        // If a start point is OK, draw partition
        if (drawPartition && vS->startOn) {
            vS->drawPartition(usedPainter, boundingRect());
        }
        if (drawSPGrid && vS->startOn) {
            vS->drawOrientationGrid(usedPainter, boundingRect());
        }
        
        // Draw the graph
        if (drawGraph) {
            setEdgesPen(QPen(::Qt::green,10));
            usedPainter->setPen(this->edgesPen());
            CGAL::Qt::PainterOstream<CK> painterostream =
            CGAL::Qt::PainterOstream<CK>(usedPainter, boundingRect());
            if (!vS->shortestPath) {
                reducedGraphMutex.lock();
                for (unsigned int j=0;j<vS->landmarks.size();j++) if (static_cast<int>(j)==lId || lId<0) {
                    // Paint nodes
                    for (unsigned int k=0;k<vS->landmarks[j].getGraph()->V();k++) {
                        painterostream <<
                        CGAL::Circle_2<CK>(vS->landmarks[j].getGraph()->getVertex(k).location,30.0);
                        // Edges from nodes
                        for (const Graph::Edge *nd = vS->landmarks[j].getGraph()->getVertexEdges(k);
                             nd!=NULL;nd=nd->next) {
                            const std::vector<lPoint> &v = nd->intermediatePoints;
                            for (unsigned int l=0;l<v.size()-1;l++) {
                                painterostream << Segment(v.at(l).location,
                                                          v.at(l+1).location);
                            }
                        }
                    }
                }
                reducedGraphMutex.unlock();
            }
            else {
                if (vS->wholeGraph) {
                    wholeGraphMutex.lock();
                    // If a query has been made, print the underlying graph through queryGrah
                    for (unsigned int k=0;k<vS->wholeGraph->V();k++) {
                        painterostream << CGAL::Circle_2<CK>(vS->wholeGraph->getVertex(k).location,20.0);
                        for (const Graph::Edge *nd = vS->wholeGraph->getVertexEdges(k);
                             nd!=NULL;nd=nd->next) {
                            const std::vector<lPoint> &v = nd->intermediatePoints;
                            for (unsigned int l=0;l<v.size()-1;l++) {
                                painterostream << Segment(v.at(l).location,
                                                          v.at(l+1).location);
                            }
                        }
                    }
#if 1
                    // Also draw (this is for debug) the optimal primitives between each nodes
                    // of the edges of this graph (to debug the fwdRatio stuff)
                    setEdgesPen(QPen(::Qt::blue,4));
                    usedPainter->setPen(this->edgesPen());
                    for (unsigned int v=0;v<vS->wholeGraph->V();v++)
                        for (const Graph::Edge* e = vS->wholeGraph->getVertexEdges(v); e!=NULL; e = e->next) {
                            int l    = e->intermediatePoints.at(0).lId;
                            trajectory *t = new trajectory(vS->landmarks.at(l).getLandmark());
                            // Check angle
                            double angle =
                            atan2(e->intermediatePoints.at(e->intermediatePoints.size()-1).location.y()-
                                  vS->landmarks.at(l).getLandmark().y(),
                                  e->intermediatePoints.at(e->intermediatePoints.size()-1).location.x()-
                                  vS->landmarks.at(l).getLandmark().x()) -
                            atan2(e->intermediatePoints.at(0).location.y()-
                                  vS->landmarks.at(l).getLandmark().y(),
                                  e->intermediatePoints.at(0).location.x()-
                                  vS->landmarks.at(l).getLandmark().x());
                            while (angle>+M_PI) angle -= 2*M_PI;
                            while (angle<-M_PI) angle += 2*M_PI;
                            double angleMid =
                            atan2(e->intermediatePoints.at(e->intermediatePoints.size()/2).location.y()-
                                  vS->landmarks.at(l).getLandmark().y(),
                                  e->intermediatePoints.at(e->intermediatePoints.size()/2).location.x()-
                                  vS->landmarks.at(l).getLandmark().x()) -
                            atan2(e->intermediatePoints.at(0).location.y()-
                                  vS->landmarks.at(l).getLandmark().y(),
                                  e->intermediatePoints.at(0).location.x()-
                                  vS->landmarks.at(l).getLandmark().x());
                            while (angleMid>+M_PI) angleMid -= 2*M_PI;
                            while (angleMid<-M_PI) angleMid += 2*M_PI;
                            if (angleMid<0 && angle>0) angle -= 2*M_PI;
                            if (angleMid>0 && angle<0) angle += 2*M_PI;
                            if (fabs(angle)<M_PI) {
                                trajectory *root =
                                t->generateNonHolonomicFreePathComplete(vS->landmarks.at(l).getLandmark(),
                                                                        e->intermediatePoints.at(0).location,
                                                                        e->intermediatePoints.at(e->intermediatePoints.size()-1).location);
                                root->gluePaths(root);
                                for (unsigned int l=0;l<root->size()-1;l++) {
                                    painterostream << Segment(Point_2(root->at(l).x(),
                                                                      root->at(l).y()),
                                                              Point_2(root->at(l+1).x(),
                                                                      root->at(l+1).y()));
                                }
                                delete root;
                            } else {
                                trajectory *root1 =
                                t->generateNonHolonomicFreePathComplete(vS->landmarks.at(l).getLandmark(),
                                                                        e->intermediatePoints.at(0).location,
                                                                        e->intermediatePoints.at(e->intermediatePoints.size()/2).location);
                                trajectory *root2 =
                                t->generateNonHolonomicFreePathComplete(vS->landmarks.at(l).getLandmark(),
                                                                        e->intermediatePoints.at(e->intermediatePoints.size()/2).location,
                                                                        e->intermediatePoints.at(e->intermediatePoints.size()-1).location);
                                root1->gluePaths(root1);
                                root2->gluePaths(root2);
                                for (unsigned int l=0;l<root1->size()-1;l++) {
                                    painterostream << Segment(Point_2(root1->at(l).x(),
                                                                      root1->at(l).y()),
                                                              Point_2(root1->at(l+1).x(),
                                                                      root1->at(l+1).y()));
                                }
                                for (unsigned int l=0;l<root2->size()-1;l++) {
                                    painterostream << Segment(Point_2(root2->at(l).x(),
                                                                      root2->at(l).y()),
                                                              Point_2(root2->at(l+1).x(),
                                                                      root2->at(l+1).y()));
                                }
                                delete root1;
                                delete root2;
                            }
                            painterostream <<
                            CGAL::Circle_2<CK>(e->intermediatePoints.at(e->intermediatePoints.size()/2).location,
                                               100.0*e->forwardRatio);
                            delete t;
                        }
#endif
                    wholeGraphMutex.unlock();
                }
            }
        }
        
        // If a query has been made, draw the holonomic shortest path
        if (drawHolonomic) {
            setEdgesPen(QPen(::Qt::red,4));
            usedPainter->setPen(this->edgesPen());
            CGAL::Qt::PainterOstream<CK> painterostream =
            CGAL::Qt::PainterOstream<CK>(usedPainter, boundingRect());
            hPathMutex.lock();
            wholeGraphMutex.lock();
            if (vS->wholeGraph&& vS->sP.size()>0) {
                // The holonomic path
                int i = vS->wholeGraph->getIndex(lPoint(vS->getEnd().p,vS->lend));
                if (vS->sP.at(i)!=NULL) {
                    // Base point (node of the graph)
                    lPoint s = vS->wholeGraph->getVertex(vS->sP.at(i)->id);
                    // Intermediate points (for the drawing)
                    lPoint r = vS->sP.at(i)->intermediatePoints.at(0);
                    painterostream << Segment(r.location,s.location);
                    while (r.location!=vS->getStart().p) {
                        i = vS->wholeGraph->getIndex(r);
                        s = vS->wholeGraph->getVertex(vS->sP.at(i)->id);
                        r = vS->sP.at(i)->intermediatePoints.at(0);
                        const std::vector<lPoint> &v = vS->sP.at(i)->intermediatePoints;
                        if (v.size()>1)
                            for (unsigned int l=0;l<v.size()-1;l++)
                                painterostream << Segment(v.at(l).location,
                                                          v.at(l+1).location);
                    }
                }
            }
            wholeGraphMutex.unlock();
            hPathMutex.unlock();
        }
        // The non-holonomic path
        nhPathMutex.lock();
        if (vS->nhPath) {
            setEdgesPen(QPen(::Qt::magenta,3));
            usedPainter->setPen(this->edgesPen());
            CGAL::Qt::PainterOstream<CK> painterostream =
            CGAL::Qt::PainterOstream<CK>(usedPainter, boundingRect());
            for (unsigned int l=0;l<vS->nhPath->size()-1;l++) {
                painterostream << Segment(Point_2(vS->nhPath->at(l).x(),
                                                  vS->nhPath->at(l).y()),
                                          Point_2(vS->nhPath->at(l+1).x(),
                                                  vS->nhPath->at(l+1).y()));
            }
        }
        if (vS->nhPath && vS->useOptimization && vS->nhPath->better.size()) {
            setEdgesPen(QPen(::Qt::blue,4));
            usedPainter->setPen(this->edgesPen());
            CGAL::Qt::PainterOstream<CK> painterostream =
            CGAL::Qt::PainterOstream<CK>(usedPainter, boundingRect());
            for (unsigned int k=0;k<vS->nhPath->better.size();k++) if (vS->nhPath->better.at(k)) {
                for (unsigned int l=0;l<vS->nhPath->better.at(k)->size()-1;l++) {
                    painterostream << Segment(Point_2(vS->nhPath->better.at(k)->at(l).x(),
                                                      vS->nhPath->better.at(k)->at(l).y()),
                                              Point_2(vS->nhPath->better.at(k)->at(l+1).x(),
                                                      vS->nhPath->better.at(k)->at(l+1).y()));
                }
            }
        }
        nhPathMutex.unlock();
        // Animation
        nhPathMutex.lock();
        if (drawAnimate && vS && vS->nhPath) {
            CGAL::Qt::PainterOstream<CK> painterostream =
            CGAL::Qt::PainterOstream<CK>(usedPainter, boundingRect());
            setEdgesPen(QPen(::Qt::blue,3));
            usedPainter->setPen(this->edgesPen());
            if (t/animSpeed<vS->nhPath->size())
                painterostream << CGAL::Circle_2<CK>(Point_2(vS->nhPath->at(t/animSpeed).x(),
                                                             vS->nhPath->at(t/animSpeed).y()),
                                                     robotRadius);
            
            // Robot direction
            if (t/animSpeed<vS->nhPath->size()) {
                if (vS->nhPath->size()!=vS->nhPath->lPositions.size()) {
                    return;
                }
                double phi  = vS->nhPath->at(t/animSpeed).z();
                double theta= atan2(vS->nhPath->at(t/animSpeed).y()-
                                    vS->nhPath->lPositions.at(t/animSpeed).y(),
                                    vS->nhPath->at(t/animSpeed).x()-
                                    vS->nhPath->lPositions.at(t/animSpeed).x())-phi+M_PI;
                painterostream <<  Segment(Point_2(vS->nhPath->at(t/animSpeed).x(),
                                                   vS->nhPath->at(t/animSpeed).y()),
                                           Point_2(vS->nhPath->at(t/animSpeed).x()+
                                                   robotRadius*cos(theta),
                                                   vS->nhPath->at(t/animSpeed).y()+
                                                   robotRadius*sin(theta)));
                // Robot gaze
                setEdgesPen(QPen(::Qt::red,3));
                usedPainter->setPen(this->edgesPen());
                painterostream <<  Segment(Point_2(vS->nhPath->at(t/animSpeed).x(),
                                                   vS->nhPath->at(t/animSpeed).y()),
                                           Point_2(vS->nhPath->at(t/animSpeed).x()+
                                                   2*robotRadius*cos(theta+phi),
                                                   vS->nhPath->at(t/animSpeed).y()+
                                                   2*robotRadius*sin(theta+phi)));
                painterostream <<  Segment(Point_2(vS->nhPath->at(t/animSpeed).x()+
                                                   2*robotRadius*cos(theta+phi)+
                                                   0.2*robotRadius*cos(theta+phi+4*M_PI/5),
                                                   vS->nhPath->at(t/animSpeed).y()+
                                                   2*robotRadius*sin(theta+phi)+
                                                   0.2*robotRadius*sin(theta+phi+4*M_PI/5)),
                                           Point_2(vS->nhPath->at(t/animSpeed).x()+
                                                   2*robotRadius*cos(theta+phi),
                                                   vS->nhPath->at(t/animSpeed).y()+
                                                   2*robotRadius*sin(theta+phi)));
                painterostream <<  Segment(Point_2(vS->nhPath->at(t/animSpeed).x()+
                                                   2*robotRadius*cos(theta+phi)+
                                                   0.2*robotRadius*cos(theta+phi-4*M_PI/5),
                                                   vS->nhPath->at(t/animSpeed).y()+
                                                   2*robotRadius*sin(theta+phi)+
                                                   0.2*robotRadius*sin(theta+phi-4*M_PI/5)),
                                           Point_2(vS->nhPath->at(t/animSpeed).x()+
                                                   2*robotRadius*cos(theta+phi),
                                                   vS->nhPath->at(t/animSpeed).y()+
                                                   2*robotRadius*sin(theta+phi)));
                // Steps, in case of a human mecanism
                if (vS->getMechanism()==vddrStructure::Human) {
                    humanTrajectory *ht = dynamic_cast<humanTrajectory *>(vS->nhPath);
                    if (ht && t/animSpeed<ht->leftFoot.size()) {
                        setEdgesPen(QPen(::Qt::darkBlue,3));
                        usedPainter->setPen(this->edgesPen());
                        paintStep(usedPainter,Point_2(ht->leftFoot.at(t/animSpeed).x(),
                                                      ht->leftFoot.at(t/animSpeed).y()),
                                  ht->leftFoot.at(t/animSpeed).z());
                        setEdgesPen(QPen(::Qt::darkGreen,3));
                        usedPainter->setPen(this->edgesPen());
                        paintStep(usedPainter,Point_2(ht->rightFoot.at(t/animSpeed).x(),
                                                      ht->rightFoot.at(t/animSpeed).y()),
                                  ht->rightFoot.at(t/animSpeed).z());
                        setEdgesPen(QPen(::Qt::red,3));
                        usedPainter->setPen(this->edgesPen());
                        painterostream <<  Segment(Point_2(ht->leftFoot.at(t/animSpeed).x(),
                                                           ht->leftFoot.at(t/animSpeed).y()),
                                                   Point_2(ht->rightFoot.at(t/animSpeed).x(),
                                                           ht->rightFoot.at(t/animSpeed).y()));
                    }
                }
            } 
        } 
        else if (vS && vS->nhPath) {
            // Steps, in case of a human mecanism
            if (vS->getMechanism()==vddrStructure::Human) {
                humanTrajectory *ht = dynamic_cast<humanTrajectory *>(vS->nhPath);
                if (ht) 
                    for (unsigned int i=0;i<ht->size();i++) {
                        CGAL::Qt::PainterOstream<CK> painterostream = 
                        CGAL::Qt::PainterOstream<CK>(usedPainter, boundingRect());
                        setEdgesPen(QPen(::Qt::darkBlue,3));
                        usedPainter->setPen(this->edgesPen());
                        paintStep(usedPainter,Point_2(ht->leftFoot.at(i).x(),
                                                      ht->leftFoot.at(i).y()),
                                  ht->leftFoot.at(i).z());
                        paintStep(usedPainter,Point_2(ht->rightFoot.at(i).x(),
                                                      ht->rightFoot.at(i).y()),
                                  ht->rightFoot.at(i).z());
                        setEdgesPen(QPen(::Qt::darkBlue,3));
                        usedPainter->setPen(this->edgesPen());
                        painterostream <<  Segment(Point_2(ht->leftFoot.at(i).x(),
                                                           ht->leftFoot.at(i).y()),
                                                   Point_2(ht->rightFoot.at(i).x(),
                                                           ht->rightFoot.at(i).y()));
                    }
            }
        } 
        nhPathMutex.unlock();  
    }
    
    // We let the bounding box only grow, so that when vertices get removed
    // the maximal bbox gets refreshed in the GraphicsView
    void 
    vddrGraphicsQueryItem::updateBoundingBox()
    {
        if(scene){
            bounding_rect = CGAL::Qt::viewportsBbox(scene);
        } else {
            bounding_rect = QRectF();
        }
    }
    
    
    void 
    vddrGraphicsQueryItem::modelChanged()
    {
        updateBoundingBox();
        update();
        
        if (vS && vS->nhPath && vS->nhPath->size()>0) {
            if (saveSnapshotPdf) {
                char fileName[100];
                sprintf(fileName,"%s-%05d.pdf",saveBasename.toStdString().c_str(),saveCounter++);
                QPrinter printer( QPrinter::HighResolution );
                printer.setPageSize( QPrinter::A0 );
                printer.setOrientation( QPrinter::Landscape );
                printer.setOutputFormat( QPrinter::PdfFormat );
                printer.setOutputFileName(fileName);
                // file will be created in your build directory (where debug/release directories are)
                QPainter p;
                if( !p.begin( &printer ) ) {
                    qDebug() << "Error!";
                    return;
                }
                scene->render( &p, QRectF(100, 100, 12000, 9000), QRectF(-scene->width()/2,-scene->height()/2, 1.5*scene->width(), 1.5*scene->height()) );
                p.end();
                if (t==animSpeed*(vS->nhPath->size()-1)+animSpeed-1) {
                    saveSnapshotPdf = false;
                    saveCounter  = 0;
                }
            }
            
            if (saveSnapshotPng) {
                int xmin=std::numeric_limits<int>::max(),xmax=0,
                ymin=std::numeric_limits<int>::max(),ymax=0;
                // 
                for (unsigned int i=0;i<vS->nhPath->size()-1;i++) {
                    if (vS->nhPath->at(i).x()<xmin) xmin = vS->nhPath->at(i).x();
                    if (vS->nhPath->at(i).x()>xmax) xmax = vS->nhPath->at(i).x();
                    if (vS->nhPath->at(i).y()<ymin) ymin = vS->nhPath->at(i).y();
                    if (vS->nhPath->at(i).y()>ymax) ymax = vS->nhPath->at(i).y();
                }
                char fileName[100];
                sprintf(fileName,"%s-%05d.png",saveBasename.toStdString().c_str(),saveCounter++);
                QImage image(QSize(xmax-xmin+200,ymax-ymin+200),
                             QImage::Format_ARGB32);
                
                QPainter imagePainter(&image);
                scene->render(&imagePainter,
                              QRectF(),
                              QRectF(xmin-100,ymin-100,
                                     xmax-xmin+200,ymax-ymin+200));
                QImage saved = image.mirrored();
                saved.save(fileName);	
                if (t==animSpeed*(vS->nhPath->size()-1)+animSpeed-1) {
                    saveSnapshotPng = false;
                    saveCounter  = 0;
                }
            }
            
            t++;
            if (t>vddrGraphicsQueryItem::animSpeed*vS->nhPath->size()-1) t=0; 
        }
        
        // Save
        if (saveSnapshotPdf && vS && !vS->nhPath) {
            // In that case, print to file
            QPainter altern;
            if (saveSnapshotPdf && !vS->nhPath) {
                char fileName[100];
                sprintf(fileName,"%s.pdf",saveBasename.toStdString().c_str());
                QPrinter printer( QPrinter::HighResolution );
                printer.setPageSize( QPrinter::A4 );
                printer.setOrientation( QPrinter::Portrait );
                printer.setOutputFormat( QPrinter::PdfFormat );
                printer.setOutputFileName(fileName); 
                // file will be created in your build directory (where debug/release directories are)
                if( !altern.begin( &printer ) ) {
                    qDebug() << "Error!";
                    return;
                }
                // Paint in this new painter
                scene->render( &altern );
                saveSnapshotPdf = false;	
                altern.end();
            }
        }
    }
    
    
    void vddrGraphicsQueryItem::saveTrajectorySnapshotsPng(const QString &s) {
        t              = 0;
        saveBasename   = s;
        saveCounter    = 0;
        saveSnapshotPng= true;
    }
    
    void vddrGraphicsQueryItem::saveTrajectorySnapshotsPdf(const QString &s) {
        t              = 0;
        saveBasename   = s;
        saveCounter    = 0;
        saveSnapshotPdf= true;
    }
    
    

}