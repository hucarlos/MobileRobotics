#ifndef CGAL_VDDR_QUERY_GRAPHICS_ITEM_H
#define CGAL_VDDR_QUERY_GRAPHICS_ITEM_H

#include <CGAL/Bbox_2.h>
#include <CGAL/apply_to_range.h>
#include <CGAL/Qt/PainterOstream.h>
#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/Converter.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/Circle_2.h>
#include <CGAL/Qt/utility.h>

#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>
#include <QWidget>
#include <QBrush>
#include <QPolygon>

#include <QDebug>
#include <QPrinter>

#include "vddrStructure.h"

extern QMutex reducedGraphMutex;
extern QMutex wholeGraphMutex;
extern QMutex hPathMutex;
extern QMutex nhPathMutex;


namespace vddr {

  typedef Polygon_2::Traits Traits;

  /**
   * Class that paints all stuff related to queries
   */
  class vddrGraphicsQueryItem : public CGAL::Qt::GraphicsItem
    {
    
  public:
    /**
     * Constructor
     */
    vddrGraphicsQueryItem(vddrStructure *vS,
			  QGraphicsScene* s);

    void modelChanged();
  
  public:
    QRectF boundingRect() const;

    void paintStep(QPainter *painter,const Point_2 &p,const double &th);  

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  
    const QPen& edgesPen() const
    {
      return edges_pen;
    }

    /**
     * Set edges pen
     * @param pen The pen to draw with.
     */
    void setEdgesPen(const QPen& pen)
    {
      edges_pen = pen;
    }

    /**
     * Set graph flag
     * @param b Underlying graph flag.
     */
    void setGraph(const bool b)
    {
      drawGraph = b;
      update();
    }
  
    /**
     * Set holonomic flag
     * @param b Holonomic path flag.
     */
    void setHolonomic(const bool b)
    {
      drawHolonomic = b;
      update();
    }

    /**
     * Set animation flag
     * @param b Animation flag.
     */ 
    void setAnimate(const bool b)
    {
      drawAnimate = b;
      if (b) t=0;
      update();
    }

    /**
     * Set partition flag
     * @param b Partition flag.
     */
    void setPartition(const bool b) {
      drawPartition = b;
      update();
    }

    /**
     * Set shortest paths grid flag
     * @param b grid flag.
     */
    void setSPGrid(const bool b) {
      drawSPGrid = b;
      update();
    }

    /**
     * Set speed
     * @param a Speed parameter.
     */
    void setSpeed(const int a) {
      animSpeed = a;
      update();
    }

    /**
     * Save trajectory (PNG)
     * @param s Filename.
     */
    void saveTrajectorySnapshotsPng(const QString &s);

    /**
     * Save trajectory (PDF)
     * @param s Filename.
     */
    void saveTrajectorySnapshotsPdf(const QString &s);

    /**
     * Set Id
     * @param l Id of the landmark.
     */
    void setlId(int l) {
      lId = l;
    };
  
    /**
     * Get Id
     * @return Id of the landmark.
     */
    int getlId() { return lId;};

  protected:
 
    /**
     * Landmark id to display
     */
    int lId;

    void updateBoundingBox();

    /**
     * Draw filled polygon.
     */
    bool draw_filled_poly(QPainter *painter,
			  const Polygon_2 &sg,
			  const QPen& b,
			  const QColor &fillcol,
			  const Qt::BrushStyle &s=Qt::SolidPattern) const;

    /**
     * Draw circles (external and internal circles).
     */
    void drawCircles(QPainter *painter) const;
  
    /**
     * Pointer to the underlying painter.
     */
    QPainter* m_painter;

    /**
     * Bounding rectanle of the drawing zone.
     */
    QRectF bounding_rect;

    /**
     * Pen to use when drawing.
     */
    QPen edges_pen;
  
    /**
     * Pointer to the VDDR structure.
     */
    vddrStructure *vS;
  
    /**
     * If set, draw the graph we use to answer the holonomic queries.
     */
    bool drawGraph;
  
    /**
     * If set, draw the shortest holonomic path onto the Voronoi graph.
     */
    bool drawHolonomic;

    /**
     * If set, draw animation of the computed path.
     */ 
    bool drawAnimate;

    /**
     * If set, draw the underlying partition from the starting point
     */
    bool drawPartition;

    /**
     * If set, draw the underlying shortest path starting orientations, on a grid
     */
    bool drawSPGrid;
  
    /**
     * If set, save snapshot png
     */
    bool saveSnapshotPng;
    bool saveSnapshotPdf;

    /**
     * Basename where to save snapshots
     */
    QString saveBasename;

    /**
     * Save counter
     */
    int saveCounter;
  
    /**
     * Time-stamp that is used in the animation process.
     */
    static unsigned int t;

    /**
     * Speed the animation process.
     */
    static unsigned int animSpeed;

    /**
     * Pointer to scene
     */
    QGraphicsScene* scene;
  };


}
#endif
