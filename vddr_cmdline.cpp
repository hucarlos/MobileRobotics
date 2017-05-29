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
#include "vddrStructure.h"
   
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
using namespace std;

int main(int argc, char **argv)
{
    
    std::cout << ">>> VDDR starting in command line mode..." << std::endl;
    
//    // Check arguments
//    if (argc<6)
//    {
//        std::cerr << "### Not enough arguments..." << std::endl;
//        exit(-1);
//    }
    
    // Environment name
    string environmentFile;
    
    // Coordinates of the starting and ending query
    double x0 = -178 ;//atof(argv[2]);
    double y0 = 218;//atof(argv[3]);
    double x1 = -12.3093 ;//atof(argv[4]);
    double y1 = 66.8437;atof(argv[5]);
    
    // Main structure
    vddrStructure vStruct;
    //vStruct.loadEnvironment(environmentFile.c_str(),false);
    
    // Query points
    Point_2 la(0.0,0.0);
    queryPoint p1(vddr::Point_2(x0,y0));
    queryPoint p2(vddr::Point_2(x1,y1));
    vStruct.addLandmark(la);
    vStruct.query(p1,p2);
}
