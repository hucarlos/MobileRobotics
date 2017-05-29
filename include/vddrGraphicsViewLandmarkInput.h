#ifndef VDDR_GRAPHICS_VIEW_LANDMARK_INPUT_H
#define VDDR_GRAPHICS_VIEW_LANDMARK_INPUT_H

#include <QGraphicsView>
#include <QRectF>
#include <QPointF>
#include <QGraphicsItem>
#include <QGraphicsLineItem> 
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QStyleOption>

#include <CGAL/Qt/Converter.h>
#include <CGAL/Qt/GraphicsViewInput.h>
#include <CGAL/Qt/utility.h>

#include "vddrObstacles.h"

namespace vddr {

  template <typename K>
    class GraphicsViewLandmarkInput : public CGAL::Qt::GraphicsViewInput {
  public:
    GraphicsViewLandmarkInput(QObject *parent, QGraphicsScene* s, vddrStructure *vStruct);
    void clear();
    const Point_2 &getLandmark() const {return sl;};

  protected:
    
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    virtual void keyPressEvent(QKeyEvent *event);
  
    bool eventFilter(QObject *obj, QEvent *event);
  private:
  
    QRectF boundingRect();
    QPointF qsp;
    Point_2 sl;
    typename K::Line_2 l;
    QGraphicsScene *scene_;  
    CGAL::Qt::Converter<K> convert;
    vddrStructure *vS;
  };

  template <typename K>
    QRectF GraphicsViewLandmarkInput<K>::boundingRect() {
    QRectF rect = CGAL::Qt::viewportsBbox(scene_);
    return rect;
  }

  template <typename K>
    GraphicsViewLandmarkInput<K>::GraphicsViewLandmarkInput(QObject *parent, QGraphicsScene* s,vddrStructure *vStruct )
    : GraphicsViewInput(parent),scene_(s),vS(vStruct) {}

  template <typename K>
    void GraphicsViewLandmarkInput<K>::clear() {  
  }

  template <typename K>
    void GraphicsViewLandmarkInput<K>::mousePressEvent(QGraphicsSceneMouseEvent *event) {  
    if(event->modifiers()  & ::Qt::ShiftModifier){
      return;
    }
    clear();
    qsp = event->scenePos();
    sl   = Point_2(qsp.x(),qsp.y()); 
    bool collide = false;
    for (std::vector<Polygon_2>::iterator it=vS->obstaclesDilated.begin();
	 it!=vS->obstaclesDilated.end();
	 it++) {
      CGAL::Bounded_side bside   = it->bounded_side(sl); 
      if (bside==CGAL::ON_BOUNDED_SIDE) {
	collide = true;
	break;
      }
    }
    if (!collide)
      for (unsigned int i=0;i<vS->landmarks.size();i++) {
	CGAL::Bounded_side bside   = vS->landmarks[i].getInner().bounded_side(sl); 
	if (bside==CGAL::ON_BOUNDED_SIDE) {
	  collide = true;
	  break;
	}
      }
    if (!collide)  
      emit generate(CGAL::make_object(typename K::Point_2(sl)));
  }

  template <typename K>
    void  GraphicsViewLandmarkInput<K>::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
  }


  template <typename K>
    void GraphicsViewLandmarkInput<K>::keyPressEvent ( QKeyEvent * event ) {
    if(event->key() != Qt::Key_Delete){
      return;
    }
  }

  template <typename K>
    bool GraphicsViewLandmarkInput<K>::eventFilter(QObject *obj, QEvent *event) {
    if (event->type() == QEvent::GraphicsSceneMousePress) {
      QGraphicsSceneMouseEvent *mouseEvent = static_cast<QGraphicsSceneMouseEvent *>(event);
      mousePressEvent(mouseEvent);
      return true;
    } else if (event->type() == QEvent::GraphicsSceneMouseMove) {
      QGraphicsSceneMouseEvent *mouseEvent = static_cast<QGraphicsSceneMouseEvent *>(event);
      mouseMoveEvent(mouseEvent);
      return true;
    } else if (event->type() == QEvent::KeyPress) {
      QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
      keyPressEvent(keyEvent);
      return true;
    } else{
      // standard event processing
      return QObject::eventFilter(obj, event);
    }
  } 
}
#endif 
