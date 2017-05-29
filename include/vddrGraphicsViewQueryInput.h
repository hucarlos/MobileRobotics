#ifndef VDDR_GRAPHICS_VIEW_QUERY_INPUT_H
#define VDDR_GRAPHICS_VIEW_QUERY_INPUT_H

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
#include "vddrQueryPoint.h"

namespace vddr {

  template <typename K>
    class GraphicsViewQueryInput : public CGAL::Qt::GraphicsViewInput {
  public:
    GraphicsViewQueryInput(QObject *parent, QGraphicsScene* s, vddrStructure *vStruct);
    void clear();
    const queryPoint &getStart() const {return sp;};
    const queryPoint &getEnd() const {return fp;};

  protected:
    
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    virtual void keyPressEvent(QKeyEvent *event);
  
    bool eventFilter(QObject *obj, QEvent *event);
  private:
  
    QRectF boundingRect();
    bool second,cg1hOn,cg1vOn,cg2hOn,cg2vOn,t1On,t2On;
    QLineF c1h,c1v,c2h,c2v;
    QGraphicsLineItem *cg1h,*cg1v,*cg2h,*cg2v;
    QGraphicsTextItem *t1,*t2;
    QPointF qsp, qtp;
    queryPoint fp,sp;
    typename K::Line_2 l;
    QGraphicsScene *scene_;  
    CGAL::Qt::Converter<K> convert;

    /**
     * Pointer to the structure
     */
    vddrStructure *vS;
  };

  template <typename K>
    QRectF GraphicsViewQueryInput<K>::boundingRect() {
    QRectF rect = CGAL::Qt::viewportsBbox(scene_);
    return rect;
  }

  template <typename K>
    GraphicsViewQueryInput<K>::GraphicsViewQueryInput(QObject *parent, QGraphicsScene* s,
						      vddrStructure *vStruct )
    : GraphicsViewInput(parent), second(false), 
    cg1hOn(false),cg1vOn(false),cg2hOn(false),cg2vOn(false),
    t1On(false),t2On(false),scene_(s),vS(vStruct) {}

  template <typename K>
    void GraphicsViewQueryInput<K>::clear() {  
    if (cg1hOn) {
      scene_->removeItem(cg1h);
      cg1hOn = false;
    }
    if (cg2hOn) {
      scene_->removeItem(cg2h);
      cg2hOn = false;
    }
    if (cg1vOn) {
      scene_->removeItem(cg1v);
      cg1vOn = false;
    }
    if (cg2vOn) {
      scene_->removeItem(cg2v);
      cg2vOn = false;
    }
    if (t1On) {
      scene_->removeItem(t1);
      t1On = false;
    }
    if (t2On) {
      scene_->removeItem(t2);
      t2On = false;
    }
    second = false;
  }

  template <typename K>
    void GraphicsViewQueryInput<K>::mousePressEvent(QGraphicsSceneMouseEvent *event) {  
    if(event->modifiers()  & ::Qt::ShiftModifier){
      return;
    }
    bool ok=false;
    if(second){
      qsp = event->scenePos();
      // Determine for which landmarks the point is admissible
      std::vector<unsigned int> landmarks;
      for (unsigned int i=0;i<vS->landmarks.size();i++)
	if (vS->landmarks[i].isPointAdmissible(Point_2(qsp.x(),qsp.y()))) 
	  landmarks.push_back(i);
      if (landmarks.size()>0) { 
	fp   = queryPoint(Point_2(qsp.x(),qsp.y()),landmarks);
	cg2h = scene_->addLine(qsp.x(),qsp.y()-15,qsp.x(),qsp.y()+15); cg2hOn = true;
	cg2v = scene_->addLine(qsp.x()-15,qsp.y(),qsp.x()+15,qsp.y()); cg2vOn = true;
	t2   = scene_->addText(QString("End point"),QFont("Times", 20, QFont::Bold));
	//t2->scale(1,1);
	// t2->scale(1,-1);
	t2->setPos(qsp.x(),qsp.y()+40);
	t2On = true;
	ok = true;
	emit generate(CGAL::make_object(std::pair<queryPoint,queryPoint>(fp,sp)));
      }
    } else {
      clear();
      qsp = event->scenePos();
      // Determine for which landmarks the point is admissible
      std::vector<unsigned int> landmarks;
      for (unsigned int i=0;i<vS->landmarks.size();i++)
	if (vS->landmarks[i].isPointAdmissible(Point_2(qsp.x(),qsp.y()))) 
	  landmarks.push_back(i);
      if (landmarks.size()>0) { 
	sp   = queryPoint(Point_2(qsp.x(),qsp.y()),landmarks);
	cg1h = scene_->addLine(qsp.x(),qsp.y()-15,qsp.x(),qsp.y()+15); cg1hOn = true;
	cg1v = scene_->addLine(qsp.x()-15,qsp.y(),qsp.x()+15,qsp.y()); cg1vOn = true;
	//t1   = scene_->addText(QString("Start point"),QFont("Times", 20, QFont::Bold));
	t1   = scene_->addText(QString("End point"),QFont("Times", 20, QFont::Bold));
	//t1->scale(1,-1);
	//t1->scale(1,1);
       	t1->setPos(qsp.x(),qsp.y()+40);
	t1On = true;
	ok = true;
	emit generate(CGAL::make_object(sp));    
      }
    }
    if (ok)
      second = !second;
  }

  template <typename K>
    void  GraphicsViewQueryInput<K>::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
  }


  template <typename K>
    void GraphicsViewQueryInput<K>::keyPressEvent ( QKeyEvent * event ) {
    if(event->key() != Qt::Key_Delete){
      return;
    }
    if(second){
      second = false;
    }
  }

  template <typename K>
    bool GraphicsViewQueryInput<K>::eventFilter(QObject *obj, QEvent *event) {
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
