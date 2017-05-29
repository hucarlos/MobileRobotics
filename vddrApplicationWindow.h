#include <fstream>
#include <vector>

// Qt headers
#include <QtGui>
#include <QString>
#include <QActionGroup>
#include <QFileDialog>
#include <QInputDialog>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QThread>
#include <QMessageBox>

// the two base classes
#include "ui_vddr.h"
#include <CGAL/Qt/DemosMainWindow.h>
#include <CGAL/Qt/GraphicsViewPolylineInput.h>
#include <CGAL/Qt/PolygonGraphicsItem.h>
#include <CGAL/Qt/GraphicsViewNavigation.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

// Local includes
#include "vddrTypedefs.h"
#include "vddrObstacles.h"
#include "vddrGraphicsItem.h"
#include "vddrGraphicsQueryItem.h"
#include "vddrStructure.h"
#include "vddrGraphicsViewQueryInput.h"
#include "vddrGraphicsViewLandmarkInput.h"


extern QMutex hPathMutex;

namespace vddr {
  /**
   * Thread class for updating connectors
   */
  class updateConnectorsThread : public QThread {
  private:
    /**
     * Reference to the vddr structure (@see vddrStructure)
     */
    vddrStructure& vStruct;
  public:
    /**
     * Constructor
     * @param Reference to the vddr structure (@see vddrStructure)
     */
    updateConnectorsThread(vddrStructure& vStruct);
    
    /**
     * Run method
     */
    virtual void run();  
  };
  
 
  /**
   * Thread class for updating graphs
   */
  class updateThread : public QThread {
  private:
    /**
     * Reference to the vddr structure (@see vddrStructure)
     */
    vddrStructure& vStruct;
    
    /**
     * Flag for handling "complete" diagram
     */
    bool complete_diagram_mode;

    /**
     * Landmark id
     */
    int landmark;

  public:
    /**
     * Constructor
     * @param vStruct Reference to the vddr structure (@see vddrStructure)
     * @param complete_diagram_mode Flag for handling "complete" diagram
     */
    updateThread(vddrStructure& vStruct,
		 bool complete_diagram_mode=false);
    
    /**
     * Run method
     */
    virtual void run();  

    /**
     * Set landmark id
     * @param l Landmark id
     */
    inline void setLandmark(int l) {landmark=l;};

    /**
     * Set complete flag
     * @param c Flag for complete mode
     */
    inline void setComplete(bool c) {complete_diagram_mode=c;}
  };

  
  /**
   * Thread class for adding landmarks
   */
  class addLandmarkThread : public QThread { 
    Q_OBJECT 
      private:
    /**
     * Reference to the vddr structure (@see vddrStructure) 
     */
    vddrStructure& vStruct;

    /**
     * Pointer to an update thread (@see updateThread)
     */
    updateThread* updateT;

    /**
     * Flag for handling "complete" diagram
     */    
    bool complete_diagram_mode;

    /**
     * Point corresponding to the landmark position
     */
    Point_2 p;

  public:
    
    /**
     * Constructor
     * @param vStruct Reference to the vddr structure (@see vddrStructure)
     * @param update  Pointer to an update thread (@see updateThread)
     * @param complete_diagram_mode Flag for handling "complete" diagram
     */    
    addLandmarkThread(vddrStructure& vStruct,
		      updateThread* update,
		      bool complete_diagram_mode=false);

    /**
     * Set point
     * @param pp Point where the new landmark is located
     */
    inline void setP(const Point_2 &pp) {p=pp;};

    /**
     * Set complete flag
     * @param c Flag for complete mode
     */
    inline void setComplete(bool c) {complete_diagram_mode=c;}

    /**
     * Run method
     */   
    virtual void run();  
  signals:
    /**
     * Signal to specify that a new landmark has been added
     * @param Number of new landmarks (here that will be one)
     */   
    void newLandmark(int nL);
  };


  
  /**
   * Thread class for adding obstacles
   */
  class addObstacleThread : public QThread {
    /**
     * Reference to the vddr structure (@see vddrStructure)
     */
    vddrStructure& vStruct;

    /**
     * Pointer to an update thread (@see updateThread)
     */
    updateThread* updateT;

   /**
     * Flag for handling "complete" diagram
     */    
    bool complete_diagram_mode;
    
    /**
     * List of points that will make the polygon
     */
    std::list<Point_2> q;

  public:
   /**
     * Constructor
     * @param vStruct Reference to the vddr structure (@see vddrStructure)
     * @param update  Pointer to an update thread (@see updateThread)
     * @param complete_diagram_mode Flag for handling "complete" diagram
     */    
    addObstacleThread(vddrStructure& vStruct, updateThread* update,bool complete_diagram_mode=false);

    /**
     * Set list of points that will make the polygon
     * @param qq List of points that will make the polygon
     */
    inline void setQ(const std::list<Point_2> &qq) {q=qq;}

    /**
     * Set complete flag
     * @param c Flag for complete mode
     */
    inline void setComplete(bool c) {complete_diagram_mode=c;}

    /**
     * Run method
     */   
    virtual void run();  
  };

 
  /**
   * Thread class for computing paths
   */
  class computePathsThread : public QThread {
    /**
     * Reference to the vddr structure (@see vddrStructure)
     */
    vddrStructure& vStruct;

    /**
     * Pair of query points
     */
    std::pair<queryPoint,queryPoint> pr;
  public:
    /**
     * Constructor
     * @param vStruct Reference to the vddr structure (@see vddrStructure)
     */
    computePathsThread(vddrStructure& vStruct);

    /**
     * Run method
     */   
    virtual void run();  

    /**
     * Set pair of query points
     * @param pp The pair of query points
     */
    inline void setPair(const std::pair<queryPoint,queryPoint>&pp) {pr=pp;}
  };

  
  /**
   * Main windows class
   */
  class MainWindow :
  public CGAL::Qt::DemosMainWindow,
    public Ui::vddrMainWindow
    {
    Q_OBJECT
  
      private:  
    /**
     * Pointer to an addLandmark thread
     */
    addLandmarkThread *addLandT;

    /**
     * Pointer to an addObstacle thread
     */
    addObstacleThread *addObsT;

    /**
     * Pointer to a comPath thread
     */
    computePathsThread *comPathT;
    
    /**
     * Pointer to an update threads
     */
    updateThread* updateT;  

    /**
     * Pointer to an updateConnectors thread
     */
    updateConnectorsThread* updateC;  

    /**
     * Graphics scene
     */
    QGraphicsScene scene;

    /**
     * Pointer to a graphics item for the vddrStructure
     */
    vddrGraphicsItem *pgi;

    /**
     * Pointer to a graphics item for the vddrStructure
     */
    vddrGraphicsQueryItem *pgr;

    /**
     * Pointer to polygons input
     */
    CGAL::Qt::GraphicsViewPolylineInput<CK> * pi;
  
    /**
     * Pointer to query input
     */ 
    GraphicsViewQueryInput<CK> *po;
  
    /**
     * Pointer to landmark input
     */
    GraphicsViewLandmarkInput<CK> *pl;
    
    /**
     * The main structure holding all geometric data
     */
    vddrStructure vStruct;
  
    /**
     * Flag for switching between polygon or points as inputs
     */
    bool inputMode;

    /**
     * Flag for signaling we wait for end point
     */
    bool inputWaitingEnd;
  
    /**
     * Flag for switching between full/reduced Voronoi
     */
    bool complete_diagram_mode;
  
    /**
     * Pointer to timer
     */
    QTimer *timer;   
    
    /**
     * All items in the scene
     */
    std::deque<QGraphicsItem *> sceneItems;

  public:
    /**
     * Constructor
     */
    MainWindow();
    
  protected:
    
    void dragEnterEvent(QDragEnterEvent *event);
    
    void dropEvent(QDropEvent *event);
    
    /**
     * Method that build the structure
     */
    void buildStructures();

    /**
     * Clear landmarks
     */
    void clearLandmarks();
 
    public slots:

    void processPolygonInput(CGAL::Object o);

    void processQueryInput(CGAL::Object o);

    void processLandmarkInput(CGAL::Object o);

    void on_actionShowPartition_toggled(bool checked);

    void on_actionShowShortestPathsGrid_toggled(bool checked);

    void on_actionShowCObst_toggled(bool checked);

    void on_actionShowVisibility_toggled(bool checked);

    void on_actionShowConnectorRegions_toggled(bool checked);

    void on_actionShowDilatedObstacles_toggled(bool checked);

    void on_actionShowVoronoiDiagram_toggled(bool checked);

    void on_actionShowCompleteVoronoi_toggled(bool checked);

    void on_actionInsertPolyline_toggled(bool checked);

    void on_actionAddLandmark_toggled(bool checked);

    void on_actionMakeQuery_toggled(bool checked);

    void on_actionClear_triggered();

    void on_actionRecenter_triggered();
  
    void on_actionShowTrajInfo_triggered();

    void on_actionSaveOrientationGrid_triggered();

    void on_actionSaveTrajectory_triggered();

    void on_actionSaveTrajectoryPrimitives_triggered();

    void on_actionSaveTrajectorySnapshotsPdf_triggered();

    void on_actionSaveTrajectorySnapshotsPng_triggered();

    void on_actionSavePhis_triggered();

    void on_actionSaveObstacles_triggered();

    void on_actionSaveEnvironment_triggered();

    void on_actionLoadObstacles_triggered();

    void on_actionLoadEnvironmentFile_triggered();

    void on_actionRecomputePath_triggered();

    void on_actionShowHolonomicPath_toggled(bool checked);
  
    void on_actionShowUnderlyingGraph_toggled(bool checked);

    void on_actionShowAnimation_toggled(bool checked);

    void on_actionDDR_toggled(bool checked);
  
    void on_actionHuman_toggled(bool checked);

    void on_actionPerformOptimization_toggled(bool checked);

    void on_actionJustShowShortcuts_toggled(bool checked);

    void on_actionVerbose_toggled(bool checked);

    void on_qBackwardsChanged(int val);

    void on_qClearanceChanged(int val);

    void on_qSpeedChanged(int val);

    void on_qViewAngleChanged();

    void on_qVisAllCheckBoxChanged(int val);

    void on_qFwdRatioCheckBox_stateChanged(int checked);

    void on_qVisSpinBoxChanged(int val);

    void on_PhiMaxSpinBoxChanged(double val);

    void on_useLateralMotion_stateChanged(int);

    void on_qtrajFormatComboBoxChanged(int);
  
    void on_newLandmark(int);

  signals:
    void changed();
    void newLandmark(int);
  };


 } 
