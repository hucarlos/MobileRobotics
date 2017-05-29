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

#define NL  1

int main(int argc, char **argv)
{
    
    std::cout << ">>> VDDR starting in command line mode..." << std::endl;
    srand(time(NULL));
    
    // Maximal distance between two landmarks
    double dmx = 2*rVisMax*sin(0.5*vddrCommonVis::viewAngle);
    unsigned int nSuccesses = 0;
    
    // Generate 100 random configurations
    for (unsigned int i=0;nSuccesses<20;i++)
    {
        std::cerr << ">>> TEST " << i << std::endl;
        double x[NL],y[NL];
        
        // Generate landmarks
        for (unsigned int j=0;j<NL;j++)
        {
            bool connected = false;
            while (!connected) {
                // Random position
                x[j] = rand()%3500;
                y[j] = rand()%3500;
                if (j==0)
                    connected = true;
                // Check if it is connected to at least one of the previous landmarks
                for (unsigned k=0;k<j;k++)
                {
                    double d = sqrt((x[j]-x[k])*(x[j]-x[k])+(y[j]-y[k])*(y[j]-y[k]));
                    if (d<dmx) {
                        connected = true;
                    }
                }
                
                for (unsigned k=0;k<j;k++)
                {
                    double d = sqrt((x[j]-x[k])*(x[j]-x[k])+(y[j]-y[k])*(y[j]-y[k]));
                    if (d<rVisMax)
                    {
                        connected = false;
                    }
                }
            }
            std::cout << x[j] << " " << y[j] << std::endl;
        }
        
        for (unsigned int j=0;j<NL;j++)
        {
            for (unsigned k=0;k<j;k++)
            {
                double d = sqrt((x[j]-x[k])*(x[j]-x[k])+(y[j]-y[k])*(y[j]-y[k]));
                printf("%f ",d);
            }
            printf("\n");
        }
        
        double x0,y0,x1,y1;
        // Random starting position
        bool seen = false;
        while (!seen)
        {
            // Random position
            x0 = rand()%3500;
            y0 = rand()%3500;
            // Check if it is connected to at least one of the landmarks
            for (unsigned j=0;j<NL;j++)
            {
                double d = sqrt((x[j]-x0)*(x[j]-x0)+(y[j]-y0)*(y[j]-y0));
                if (d<rVisMax && d>rVisMin)
                {
                    seen = true;
                    std::cerr << ">>> Start point " << x0 << " " << y0 << " connected to " << j << std::endl;
                    break;
                }
            }
        }
        
        // Random ending position
        seen = false;
        while (!seen)
        {
            // Random position
            x1 = rand()%3500;
            y1 = rand()%3500;
            // Check if it is connected to at least one of the landmarks
            for (unsigned j=0;j<NL;j++) {
                double d = sqrt((x[j]-x1)*(x[j]-x1)+(y[j]-y1)*(y[j]-y1));
                if (d<rVisMax && d>rVisMin) {
                    seen = true;
                    std::cerr << ">>> End point " << x1 << " " << y1 << " connected to " << j << std::endl;
                    break;
                }
            }
        }
        
        // Main structure
        vddrStructure vStruct;
        vStruct.setOptimization(true);
        vStruct.clearAll();
        for (unsigned int j=0;j<NL;j++)
        {
            std::cerr << ">>> Adding landmark " << j << std::endl;
            vStruct.addLandmark(vddr::Point_2(x[j],y[j]));
        }
        
        try
        {
            vStruct.update(false);
        }
        catch (...)
        {
            // Failure
            std::cerr << ">>> Bad case " << std::endl;
            continue;
        }
        
        // Query points
        queryPoint p1;
        queryPoint p2;
        std::vector<unsigned int> landmarks;
        for (unsigned int i=0;i<vStruct.landmarks.size();i++)
            if (vStruct.landmarks[i].isPointAdmissible(Point_2(x0,y0)))
                landmarks.push_back(i);
        
        if (landmarks.size()>0)
        {
            p1   = queryPoint(Point_2(x0,y0),landmarks);
        }
        landmarks.clear();
        
        for (unsigned int i=0;i<vStruct.landmarks.size();i++)
            if (vStruct.landmarks[i].isPointAdmissible(Point_2(x1,y1))) 
                landmarks.push_back(i);
        
        if (landmarks.size()>0)
        {
            p2   = queryPoint(Point_2(x1,y1),landmarks);
        }
        
        if (vStruct.query(p1,p2) == true)
        {
            // Save environment
            char filenameEnv[100];
            sprintf(filenameEnv,"evaluation-%04d.env",nSuccesses);
            vStruct.saveEnvironment(filenameEnv);
            
            // Save trajectory
            char filenameTraj[100];
            sprintf(filenameTraj,"evaluation-%04d.prim",nSuccesses);
            vStruct.nhPath->savePrimitives(filenameTraj);
            
            // Increment number of successes
            nSuccesses++;
        }
        
    }
    
    return 0;
}
