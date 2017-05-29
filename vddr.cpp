#include <iostream>
#include <fstream>
#include <list>
#include <vector>
 
#include <CGAL/basic.h>
#include <CGAL/IO/pixmaps/demoicon.xpm>
#include <CGAL/Timer.h>

#include "visilibity.h"
#include "vddrTypedefs.h"
#include "vddrGraphReduction.h"
#include "vddrTrajectories.h" 
#include "vddrObstacles.h"

#include "vddrApplicationWindow.h"

#include <QApplication>
   
QMutex voronoiMutex; 
QMutex visibilityMutex;
QMutex obstaclesMutex;
QMutex connectorAreaMutex;
QMutex connectorPointMutex;
QMutex cobstMutex;
QMutex reducedGraphMutex; 
QMutex wholeGraphMutex;
QMutex hPathMutex;
QMutex nhPathMutex;
QMutex commonVisMutex;
QMutex connectorReducedGraphMutex;     
using namespace vddr; 

int 
main(int argc, char* argv[]) {  
  // Set inner/outer boundary and center
  std::cout << "VDDR starting..." << std::endl;
    
  QApplication app( argc, argv );
  app.setOrganizationDomain("cimat.mx");  
  app.setOrganizationName("CIMAT");
  app.setApplicationName("Visually Constrained Differential Drive Robot demo");

  // Import resources from libCGALQt4.
  // See http://doc.trolltech.com/4.4/qdir.html#Q_INIT_RESOURCE
  Q_INIT_RESOURCE(File);
  Q_INIT_RESOURCE(Triangulation_2);
  Q_INIT_RESOURCE(Input);
  Q_INIT_RESOURCE(CGAL);
  Q_INIT_RESOURCE(vddr);

  // The main window
  MainWindow mainWindow;
  mainWindow.show();
  return app.exec();
}
