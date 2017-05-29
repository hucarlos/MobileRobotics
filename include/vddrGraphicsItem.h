#ifndef CGAL_VDDR_GRAPHICS_ITEM_H
#define CGAL_VDDR_GRAPHICS_ITEM_H

#include <CGAL/Bbox_2.h>
#include <CGAL/apply_to_range.h>
#include <CGAL/Qt/PainterOstream.h>
#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/Converter.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/Qt/utility.h>

#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>
#include <QWidget>
#include <QBrush>
#include <QPolygon>

#include <QMutex>

extern QMutex voronoiMutex;
extern QMutex visibilityMutex;
extern QMutex obstaclesMutex;
extern QMutex connectorAreaMutex;
extern QMutex connectorPointMutex;
extern QMutex cobstMutex;
extern QMutex commonVisMutex;

#include "vddrStructure.h"

namespace vddr {

  typedef Polygon_2::Traits Traits; 
  typedef Polygon_2_e::Traits Traits_e;

  /**
   * Graphics item class
   */
  class vddrGraphicsItem : public CGAL::Qt::GraphicsItem
    {
  public:
    /**
     * Constructor
     */
    vddrGraphicsItem(vddrStructure *vS);
  
    void modelChanged();
    
  public:
    /**
     * Bounding rectangle
     * @return Bounding rectangle
     */
    QRectF boundingRect() const;
  
    /**
     * Paint
     */
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  
    /**
     * Get pen
     */
    const QPen& edgesPen() const
    {
      return edges_pen;
    }
    
    /**
     * Set pen
     */
    void setEdgesPen(const QPen& pen)
    {
      edges_pen = pen;
    }

    void setCObst(const bool b)
    {
      drawCObst = b;
      update();
    }

    void setVisibility(const bool b)
    {
      drawVis = b;
      update();
    }

    void setDilated(const bool b)
    {
      drawDilated = b;
      update();
    }

    void setVoronoi(const bool b)
    {
      drawVoronoi = b;
      update();
    }

    void setCompleteVoronoi(const bool b)
    {
      drawCompleteVoronoi = b;
      update();
    }
  
    void setConnectors(const bool b)
    {
      drawConnectors = b;
      update();
    }

    /**
     * Set Id
     * @param l Landmark id
     */
    void setlId(int l) {
      lId = l;
    };
  
    /**
     * Get Id
     * @return Landmark id
     */
    int getlId() const { return lId;};

    /**
     * Set Id 
     * @param l Landmark id
     */
    void setOlId(int l) {
      olId = l;
    };
  
    /**
     * Get Id
     * @return Second landmark id (to visualize switch areas)
     */
    int getOlId() const { return olId;};
    
  protected:
    void updateBoundingBox();
    bool draw_filled_poly(QPainter *painter,
			  const Polygon_2 &sg,
			  const QPen& b,
			  const QColor &fillcol,
			  const Qt::BrushStyle &s=Qt::SolidPattern) const;
    bool draw_filled_poly(QPainter *painter,
			  const Polygon_2_e &sg,
			  const QPen& b,
			  const QColor &fillcol,
			  const Qt::BrushStyle &s=Qt::SolidPattern) const;  
    bool draw_point(QPainter *painter,
		    const Point_2_e &sg,
		    const QPen& b,
		    const QColor &fillcol,
		    const Qt::BrushStyle &s=Qt::SolidPattern) const;

    /**
     * Pointer to the painter
     */
    QPainter* m_painter;
    QRectF bounding_rect;
    QPen edges_pen;

    /**
     * Pointer to the main structure
     */
    vddrStructure *vS;

    /**
     * Landmark id to display
     */
    int lId;

    /**
     * Landmark id to display the connectors
     */
    int olId;

    bool drawCObst;
    bool drawVis;
    bool drawDilated;
    bool drawCompleteVoronoi;
    bool drawVoronoi;
    bool drawConnectors;
  };

  }
#endif
