#include <vddrApplicationWindow.h>

namespace vddr
{
    // Constructor
    updateConnectorsThread::updateConnectorsThread(vddrStructure& vStruct) :
    vStruct(vStruct){
    }
    
    // Overload of run
    void updateConnectorsThread::run() {
        // Update the structure
        vStruct.updateConnectors(-1);
    }
    
    // Constructor
    updateThread::updateThread(vddrStructure& vStruct,bool complete_diagram_mode) :
    vStruct(vStruct),complete_diagram_mode(complete_diagram_mode) {
    }
    
    // Overload of run
    void updateThread::run() {
        // Update the structure
        vStruct.update(complete_diagram_mode,landmark);
    }
    
    // Constructor
    addLandmarkThread::addLandmarkThread(vddrStructure& vStruct,
                                         updateThread* updatet,
                                         bool complete_diagram_mode) :
    vStruct(vStruct),updateT(updatet),complete_diagram_mode(complete_diagram_mode) {
    }
    
    // Overload of run
    void addLandmarkThread::run() {
        // Add landmark to the structure
        if (vStruct.addLandmark(p)) {
            // Specify that one landmark has been added
            emit newLandmark(1);
            // Create a qthread object and do the update
            updateT->wait();
            updateT->setLandmark(vStruct.landmarks.size()-1);
            updateT->start();
        }
    }
    
    // Constructor
    addObstacleThread::addObstacleThread(vddrStructure& vStruct, updateThread* update,bool complete_diagram_mode) :
    vStruct(vStruct),updateT(update),complete_diagram_mode(complete_diagram_mode){
    }
    
    // Overload of run
    void addObstacleThread::run() {
        vStruct.insertObstacle(q,complete_diagram_mode);
        updateT->wait();
        updateT->setLandmark(-1);
        updateT->start();
    }

    
    // Constructor
    computePathsThread::computePathsThread(vddrStructure& vStruct) : vStruct(vStruct){
    }
    
    // Overload of run
    void computePathsThread::run() {
        vStruct.query(pr.second,pr.first);
    }

    
    // Constructor
    MainWindow::MainWindow() : DemosMainWindow(), inputWaitingEnd(false) {
        setupUi(this);
        
        qRegisterMetaType<Point_2>("Point_2");
        // Add a GraphicItem for the Delaunay triangulation
        pgi = new vddrGraphicsItem(&vStruct);
        QObject::connect(this, SIGNAL(changed()),
                         pgi, SLOT(modelChanged()));
        pgi->setEdgesPen(QPen(Qt::red, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        pgi->setZValue(-1);
        scene.addItem(pgi);
        
        
        pgr = new vddrGraphicsQueryItem(&vStruct,&scene);
        pgr->setZValue(1);
        // Animation of the trajectory
        timer = new QTimer(this);
        QObject::connect(timer, SIGNAL(timeout()),pgr,SLOT(modelChanged()));
        timer->start(100);
        scene.addItem(pgr);
        
        setAcceptDrops(true);
        
        // Setup input handlers. They get events before the scene gets them
        // and the input they generate is passed to the triangulation with
        // the signal/slot mechanism
        pi = new CGAL::Qt::GraphicsViewPolylineInput<CK>(this, &scene, 0, true);
        QObject::connect(pi, SIGNAL(generate(CGAL::Object)),
                         this, SLOT(processPolygonInput(CGAL::Object)));
        scene.installEventFilter(pi);
        po = new GraphicsViewQueryInput<CK>(this, &scene,&vStruct);
        QObject::connect(po, SIGNAL(generate(CGAL::Object)),
                         this, SLOT(processQueryInput(CGAL::Object)));
        pl = new GraphicsViewLandmarkInput<CK>(this, &scene,&vStruct);
        QObject::connect(pl, SIGNAL(generate(CGAL::Object)),
                         this, SLOT(processLandmarkInput(CGAL::Object)));
        
        // Manual handling of actions
        QObject::connect(this->actionQuit, SIGNAL(triggered()),
                         this, SLOT(close()));
        
        // QSlider changes
        QObject::connect(this->qSliderBackwards, SIGNAL(valueChanged(int)),
                         this, SLOT(on_qBackwardsChanged(int)));
        QObject::connect(this->qSliderSpeed, SIGNAL(valueChanged(int)),
                         this, SLOT(on_qSpeedChanged(int)));
        QObject::connect(this->qSliderViewAngle, SIGNAL(sliderReleased()),
                         this, SLOT(on_qViewAngleChanged()));
        QObject::connect(this->qSliderClearance, SIGNAL(valueChanged(int)),
                         this, SLOT(on_qClearanceChanged(int)));
        QObject::connect(this->visAllCheckBox, SIGNAL(stateChanged(int)),
                         this, SLOT(on_qVisAllCheckBoxChanged(int)));
        QObject::connect(this->visSpinBox, SIGNAL(valueChanged(int)),
                         this, SLOT(on_qVisSpinBoxChanged(int)));
        QObject::connect(this->PhiMaxSpinBox, SIGNAL(valueChanged(double)),
                         this, SLOT(on_PhiMaxSpinBoxChanged(double)));
        QObject::connect(this->trajFormatComboBox, SIGNAL(activated(int)),
                         this, SLOT(on_qtrajFormatComboBoxChanged(int)));
        QObject::connect(this->qFwdCheckBox, SIGNAL(stateChanged(int)),
                         this, SLOT(on_qFwdRatioCheckBox_stateChanged(int)));
        
        
        // Get default values of the sliders
        vStruct.setViewAngle((double)(this->qSliderViewAngle->value())*3.14/100.0);
        vStruct.setFwdRatio((this->qFwdCheckBox->checkState()==Qt::Checked));
        
        // We put mutually exclusive actions in an QActionGroup
        QActionGroup* ag = new QActionGroup(this);
        ag->addAction(this->actionInsertPolyline);
        ag->addAction(this->actionMakeQuery);
        ag->addAction(this->actionAddLandmark);
        
        // Check two actions
        this->actionInsertPolyline->setChecked(true);
        this->actionShowVoronoiDiagram->setChecked(true);
        this->actionShowCompleteVoronoi->setChecked(false);
        this->actionShowDilatedObstacles->setChecked(false);
        this->actionShowCObst->setChecked(false);
        this->actionShowVisibility->setChecked(false);
        this->actionShowPartition->setChecked(false);
        this->actionShowShortestPathsGrid->setChecked(false);
        this->actionSaveOrientationGrid->setEnabled(false);
        this->actionShowCObst->setChecked(false);
        this->actionShowConnectorRegions->setChecked(false);
        // Set mechanism type actions as exclusive
        QActionGroup* tg = new QActionGroup(this);
        tg->addAction(this->actionDDR);
        tg->addAction(this->actionHuman);
        
        // Set phi min and max values
        PhiMaxSpinBox->setValue(trajectory::phi2);
        
        // Setup the scene and the view
        scene.setItemIndexMethod(QGraphicsScene::NoIndex);
        scene.setSceneRect(vStruct.landmarks[0].getLandmark().x()-rVisMax,
                           vStruct.landmarks[0].getLandmark().y()-rVisMax,
                           2*rVisMax,2*rVisMax);
        this->graphicsView->setScene(&scene);
        
        // Turn the vertical axis upside down
        this->graphicsView->scale(0.5, -0.5);
        
        // The navigation adds zooming and translation
        // functionality to the QGraphicsView
        this->addNavigation(this->graphicsView);
        
        this->setupStatusBar();
        this->setupOptionsMenu();
        this->addAboutDemo(":/cgal/help/about_Constrained_Delaunay_triangulation_2.html");
        this->addAboutCGAL();
        
        // Threads for heavy CG operations
        updateT  = new updateThread(vStruct);
        updateC  = new updateConnectorsThread(vStruct);
        addLandT = new addLandmarkThread(vStruct,updateT);
        QObject::connect(this->addLandT, SIGNAL(newLandmark(int)),
                         this, SLOT(on_newLandmark(int)));
        QObject::connect(this, SIGNAL(newLandmark(int)),
                         this, SLOT(on_newLandmark(int)));
        addObsT  = new addObstacleThread(vStruct,updateT);
        comPathT = new computePathsThread(vStruct);
        
        updateT->setLandmark(-1);
        updateT->start();
        // Initialize the first landmark
        on_newLandmark(1);
    }
    
    
    void
    MainWindow::dragEnterEvent(QDragEnterEvent *event)
    {
        if (event->mimeData()->hasFormat("text/uri-list"))
            event->acceptProposedAction();
    }
    
    void
    MainWindow::dropEvent(QDropEvent *event)
    {
    }
    
    void
    MainWindow::processPolygonInput(CGAL::Object o)
    {
        // Normal mode: input of new polygons
        std::list<Point_2> pgn;
        if(CGAL::assign(pgn, o)){
            comPathT->wait();
            addObsT->wait();
            addLandT->wait();
            updateT->wait();
            
            addObsT->setQ(pgn);
            addObsT->start();
        }
        hPathMutex.lock();
        vStruct.clearQuery();
        hPathMutex.unlock();
        po->clear();
        emit(changed());
    }
    
    void
    MainWindow::processQueryInput(CGAL::Object o)
    {
        queryPoint p;
        std::pair<queryPoint,queryPoint> s;
        if (CGAL::assign(p, o)) {
            hPathMutex.lock();
            vStruct.setStart(po->getStart());
            vStruct.sP.clear();
            // Waiting end flag
            inputWaitingEnd = true;
            // Only moment when save orientations possible
            this->actionSaveOrientationGrid->setEnabled(true);
            hPathMutex.unlock();
        } else if (CGAL::assign(s, o)) {
            comPathT->wait();
            addObsT->wait();
            addLandT->wait();
            updateT->wait();
            // Waiting end flag
            inputWaitingEnd = false;
            // Only moment when save orientations possible
            this->actionSaveOrientationGrid->setEnabled(false);
            comPathT->setPair(s);
            comPathT->start();
        }
        return;
    }
    
    void
    MainWindow::processLandmarkInput(CGAL::Object o)
    {
        Point_2 p;
        if (CGAL::assign(p, o)) {
            visSpinBox->setMaximum(visSpinBox->maximum()+1);
            comPathT->wait();
            addObsT->wait();
            addLandT->wait();
            updateT->wait();
            
            addLandT->setP(p);
            addLandT->start();
        }
        hPathMutex.lock();
        vStruct.clearQuery();
        hPathMutex.unlock();
        return;
    }
    
    void
    MainWindow::on_actionClear_triggered() {
        std::cerr << "*** on_actionClear_toggled" << std::endl;
        vStruct.clear(false);
        emit(changed());
    }
    
    void MainWindow::on_actionShowHolonomicPath_toggled(bool checked) {
        std::cerr << "*** on_actionShowHolonomicPath_toggled" << std::endl;
        if (pgr)
            pgr->setHolonomic(checked);
        emit(changed());
    }
    
    
    void MainWindow::on_actionShowUnderlyingGraph_toggled(bool checked) {
        std::cerr << "*** on_actionShowUnderlyingGraph_toggled" << std::endl;
        if (pgr)
            pgr->setGraph(checked);
        emit(changed());
    }
    
    void MainWindow::on_actionShowAnimation_toggled(bool checked) {
        std::cerr << "*** on_actionShowAnimation_toggled" << std::endl;
        if (pgr)
            pgr->setAnimate(checked);
        emit(changed());
    }
    
    void
    MainWindow::on_actionRecenter_triggered()
    {
        std::cerr << "*** on_actionRecenter_toggled" << std::endl;
        this->graphicsView->setSceneRect(pgi->boundingRect());
        this->graphicsView->fitInView(pgi->boundingRect(), Qt::KeepAspectRatio);
    }
    
    void
    MainWindow::on_actionSaveTrajectorySnapshotsPng_triggered()
    {
        std::cerr << "*** on_actionSaveTrajectorySnapshotsPng_toggled" << std::endl;
        if (vStruct.nhPath) {
            QString s = QFileDialog::getSaveFileName(this,
                                                     "Choose a basename to save under",
                                                     ".",
                                                     "PNG files (*.png)");
            pgr->saveTrajectorySnapshotsPng(s);
        } else {
            QMessageBox msgBox;
            msgBox.setWindowTitle("Saving trajectory snapshots");
            msgBox.setText("No trajectory has been computed.");
            msgBox.exec();
        }
    }
    
    void MainWindow::on_actionSaveOrientationGrid_triggered() {
        std::cerr << "*** on_actionSaveOrientationGrid_toggled" << std::endl;
        if (trajectory::orientationGrid) {
            QString s = QFileDialog::getSaveFileName(this,
                                                     "Choose a basename to save under",
                                                     ".",
                                                     "GRD files (*.grd)");
            vStruct.saveOrientationGrid(s);
        } else {
            QMessageBox msgBox;
            msgBox.setWindowTitle("Saving orientation grid");
            msgBox.setText("No grid has been computed.");
            msgBox.exec();
        }
    }
    
    void
    MainWindow::on_actionSaveTrajectorySnapshotsPdf_triggered()
    {
        std::cerr << "*** on_actionSaveTrajectorySnapshotsPdf_toggled" << std::endl;
        QString s = QFileDialog::getSaveFileName(this,
                                                 "Choose a basename to save under",
                                                 ".",
                                                 "Pdf file (*.pdf)");
        pgr->saveTrajectorySnapshotsPdf(s);
    }
    
    
    void
    MainWindow::on_actionSaveTrajectory_triggered()
    {
        std::cerr << "*** on_actionSaveTrajectory_toggled" << std::endl;
        if (vStruct.nhPath) {
            if (vStruct.getMechanism()==vddrStructure::Human) {
                QString s = QFileDialog::getSaveFileName(this,
                                                         "Choose a filename to save under",
                                                         ".",
                                                         "Human trajectory files (*.prm)");
                vStruct.nhPath->savePath(s);
            } else {
                QString s = QFileDialog::getSaveFileName(this,
                                                         "Choose a filename to save under",
                                                         ".",
                                                         "Trajectory files (*.traj)");
                vStruct.nhPath->savePath(s);
            }
        } else {
            QMessageBox msgBox;
            msgBox.setWindowTitle("Saving trajectory");
            msgBox.setText("No trajectory has been computed.");
            msgBox.exec();
        }
    }
    
    void
    MainWindow::on_actionSavePhis_triggered()
    {
        std::cerr << "*** on_actionSavePhis_toggled" << std::endl;
        if (vStruct.nhPath) {
            QString s = QFileDialog::getSaveFileName(this,
                                                     "Choose a filename to save under",
                                                     ".",
                                                     "Phi angles files (*.phi)");
            vStruct.nhPath->savePhis(s);
        } else {
            QMessageBox msgBox;
            msgBox.setWindowTitle("Saving phis");
            msgBox.setText("No trajectory has been computed.");
            msgBox.exec();
        }
    }
    
    void
    MainWindow::on_actionSaveTrajectoryPrimitives_triggered()
    {
        std::cerr << "*** on_actionSaveTrajectoryPrimitives_toggled" << std::endl;
        if (vStruct.nhPath) {
            if (vStruct.getMechanism()==vddrStructure::Human) {
                QString s = QFileDialog::getSaveFileName(this,
                                                         "Choose a filename to save under",
                                                         ".",
                                                         "Human trajectory primitives files (*.prim)");
                vStruct.nhPath->savePrimitives(s);
            } else {
                QString s = QFileDialog::getSaveFileName(this,
                                                         "Choose a filename to save under",
                                                         ".",
                                                         "Trajectory primitives files (*.prim)");
                vStruct.nhPath->savePrimitives(s);
            }
        } else {
            QMessageBox msgBox;
            msgBox.setWindowTitle("Saving trajectory");
            msgBox.setText("No trajectory has been computed.");
            msgBox.exec();
        }
    }
    
    void
    MainWindow::on_actionSaveObstacles_triggered()
    {
        std::cerr << "*** on_actionSaveObstacles_toggled" << std::endl;
        QString fileName = QFileDialog::getSaveFileName(this,
                                                        "Choose a filename to save under",
                                                        ".",
                                                        "Set of polygons (*.pln)");
        
        vStruct.saveObstacles(fileName);
    }
    
    void
    MainWindow::on_actionSaveEnvironment_triggered()
    {
        std::cerr << "*** on_actionSaveEnvironment_toggled" << std::endl;
        QString fileName = QFileDialog::getSaveFileName(this,
                                                        "Choose a filename to save under",
                                                        ".",
                                                        "Environment (*.env)");
        
        vStruct.saveEnvironment(fileName);
    }
    
    void
    MainWindow::on_actionLoadObstacles_triggered()
    {
        std::cerr << "*** on_actionLoadObstacles_toggled" << std::endl;
        QString fileName = QFileDialog::getOpenFileName(this,
                                                        "Choose a file to load",
                                                        ".",
                                                        "Set of polygons (*.pln)");
        
        vStruct.loadObstacles(fileName,complete_diagram_mode);
        
    }
    
    void
    MainWindow::on_actionLoadEnvironmentFile_triggered()
    {
        std::cerr << "*** on_actionLoadEnvironmentFile_toggled" << std::endl;
        // Choose environment file
        QString fileName = QFileDialog::getOpenFileName(this,
                                                        "Choose an environment file to load",
                                                        ".",
                                                        "Environment (*.env)");
        
        // Remove all landmarks, except the first one
        for (unsigned int i=4;i<sceneItems.size();i++) {
            scene.removeItem(sceneItems[i]);
            delete sceneItems[i];
        }
        
        // Load environment file
        int l = vStruct.loadEnvironment(fileName,complete_diagram_mode);
        sleep(1);
        emit newLandmark(l);
        sleep(1);
        visSpinBox->setMaximum(vStruct.landmarks.size()-1);
        sleep(1);
        std::cerr << "*** Environment loaded " << std::endl;
    }
    
    void
    MainWindow::on_actionShowTrajInfo_triggered()
    {
        std::cerr << "*** on_actionShowTrajInfo_toggled" << std::endl;
        QMessageBox msgBox;
        msgBox.setWindowTitle("Trajectory information");
        if (!vStruct.shortestPath)
            msgBox.setText("No trajectory has been computed.");
        else {
            std::stringstream ss(std::stringstream::out);
            if (vStruct.nhPath) {
                ss << "Initial point  : " << vStruct.nhPath->at(0) << std::endl;
                ss << "Final point    : " << vStruct.nhPath->at(vStruct.nhPath->size()-1) << std::endl;
                ss << "Length         : " << vStruct.nhPath->length() << std::endl;
                std::pair<double,double> p = vStruct.nhPath->fwdBckwd();
                ss << "Forward length : " << p.first << std::endl;
                ss << "Backward length: " << p.second << std::endl;
                ss << "Description    : " << std::endl;
#if TODO
                // Forward/backward part
#endif
                vStruct.nhPath->printPathType(ss);
                msgBox.setText(ss.str().c_str());
            } else {
                QMessageBox msgBox;
                msgBox.setWindowTitle("Saving trajectory");
                msgBox.setText("No trajectory has been computed.");
                msgBox.exec();
            }
        }
        msgBox.exec();
    }
    
    void
    MainWindow::on_actionInsertPolyline_toggled(bool checked) {
        std::cerr << "*** on_actionInsertPolyline_toggled" << std::endl;
        if(checked){
            scene.installEventFilter(pi);
        } else {
            scene.removeEventFilter(pi);
        }
    }
    
    void
    MainWindow::on_actionAddLandmark_toggled(bool checked) {
        std::cerr << "*** on_actionAddLandmark_toggled" << std::endl;
        if(checked){
            std::cerr << "*** Ready to add new landmarks" << std::endl;
            scene.installEventFilter(pl);
        } else {
            scene.removeEventFilter(pl);
        }
    }
    
    void
    MainWindow::on_actionMakeQuery_toggled(bool checked) {
        std::cerr << "*** on_actionMakeQuery_toggled" << std::endl;
        if(checked){
            scene.installEventFilter(po);
        } else {
            scene.removeEventFilter(po);
        }
    }
    
    void
    MainWindow::on_actionShowVoronoiDiagram_toggled(bool checked) {
        std::cerr << "*** on_actionShowVoronoiDiagram_toggled" << std::endl;
        if (pgi)
            pgi->setVoronoi(checked);
        emit(changed());
    }
    
    void
    MainWindow::on_actionShowCompleteVoronoi_toggled(bool checked) {
        std::cerr << "*** on_actionShowCompleteVoronoi_toggled" << std::endl;
        if (pgi)
            pgi->setCompleteVoronoi(checked);
        if (updateT) {
            updateT->setComplete(checked);
        }
        complete_diagram_mode =checked;
        emit(changed());
    }
    
    void
    MainWindow::on_actionShowCObst_toggled(bool checked) {
        std::cerr << "*** on_actionShowCObst_toggled" << std::endl;
        if (pgi)
            pgi->setCObst(checked);
        emit(changed());
    }
    
    void
    MainWindow::on_actionShowVisibility_toggled(bool checked) {
        std::cerr << "*** on_actionShowVisibility_toggled" << std::endl;
        if (pgi)
            pgi->setVisibility(checked);
        emit(changed());
    }
    
    void
    MainWindow::on_actionShowConnectorRegions_toggled(bool checked) {
        std::cerr << "*** on_actionShowConnectorRegions_toggled" << std::endl;
        if (pgi)
            pgi->setConnectors(checked);
        emit(changed());
    }
    
    void
    MainWindow::on_actionShowDilatedObstacles_toggled(bool checked) {
        std::cerr << "*** on_actionShowDilatedObstacles_toggled" << std::endl;
        if (pgi)
            pgi->setDilated(checked);
        emit(changed());
    }
    
    void
    MainWindow::on_actionShowPartition_toggled(bool checked) {
        std::cerr << "*** on_actionShowPartition_toggled" << std::endl;
        if (pgr)
            pgr->setPartition(checked);
        emit(changed());
    }
    
    void
    MainWindow::on_actionShowShortestPathsGrid_toggled(bool checked) {
        std::cerr << "*** on_actionShowShortestPathsGrid_toggled" << std::endl;
        if (pgr)
            pgr->setSPGrid(checked);
        emit(changed());
    }
    
    void MainWindow::on_actionDDR_toggled(bool checked) {
        std::cerr << "*** on_actionShowPartition_toggled" << std::endl;
        if (checked)
            vStruct.setMechanism(vddrStructure::DDR);
        emit(changed());
    }
    
    void MainWindow::on_actionHuman_toggled(bool checked) {
        std::cerr << "*** on_actionHuman_toggled" << std::endl;
        if (checked) {
            vStruct.setMechanism(vddrStructure::Human);
            double qfac = 1.0+(double)(this->qSliderBackwards->value())/50.0;
            humanTrajectory::setQ(qfac);
        }
        emit(changed());
    }
    
    void MainWindow::on_qBackwardsChanged(int val) {
        std::cerr << "*** on_qBackwardsChanged_toggled" << std::endl;
        if (vStruct.getMechanism()==vddrStructure::Human) {
            double qfac = 1.0+(double)(val)/50.0;
            humanTrajectory::setQ(qfac);
            std::cerr << "*** Penalizing backward motion set to " << qfac << std::endl;
        }
    }
    
    void MainWindow::on_qSpeedChanged(int val) {
        std::cerr << "*** on_qSpeedChanged_toggled" << std::endl;
        int qfac = (int)(200.0/(double)val);
        if (pgr)
            pgr->setSpeed(qfac);
        emit(changed());
    }
    
    void MainWindow::on_qViewAngleChanged() {
        std::cerr << "*** on_qViewAngleChanged_toggled" << std::endl;
        vStruct.setViewAngle((double)(qSliderViewAngle->value())*3.14/100.0);
        comPathT->wait();
        addObsT->wait();
        addLandT->wait();
        updateT->wait();
        updateC->wait();
        updateC->start();
    }
    
    void MainWindow::on_qClearanceChanged(int val) {
        std::cerr << "*** on_qClearanceChanged_toggled" << std::endl;
        double cfac = 1.0+(double)(val)/90.0;
        Graph::clearanceFactor = cfac;
        std::cerr << "*** Penalizing low clearance set to " << cfac << std::endl;
        
    }
    
    void MainWindow::on_qVisAllCheckBoxChanged(int val) {
        std::cerr << "*** on_qVisAllCheckBoxChanged_toggled" << std::endl;
        if (visAllCheckBox->isChecked()) {
            visSpinBox->setEnabled(false);
            pgi->setlId(-1);
            pgr->setlId(-1);
        } else {
            visSpinBox->setEnabled(true);
            pgi->setlId(visSpinBox->value());  
            pgr->setlId(visSpinBox->value());  
        }
    }
    
    void MainWindow::on_qVisSpinBoxChanged(int val) {
        std::cerr << "*** Changing visibility to " << val << std::endl;
        if (!visAllCheckBox->isChecked()) {
            pgi->setlId(val);  
            pgr->setlId(val);  
        } else {
            pgi->setlId(-1);  
            pgr->setlId(-1);  
        }
    }
    
    void MainWindow::on_useLateralMotion_stateChanged(int checked) {
        std::cerr << "*** Changing useLateralMotion to " << checked << std::endl;
        vStruct.setUseLateralMotion(checked==Qt::Checked);
        if (inputWaitingEnd)
            vStruct.setStart(po->getStart());
    }
    
    void MainWindow::on_PhiMaxSpinBoxChanged(double val) {
        std::cerr << "*** Changing phimax to " << val << std::endl;
        vStruct.setPhiMin(-val);
        vStruct.setPhiMax(val);
    }
    
    void MainWindow::on_qtrajFormatComboBoxChanged(int s) {
        std::cerr << "*** on_qtrajFormatComboBoxChanged_toggled" << std::endl;
        switch (s) {
            case 0:
                humanTrajectory::writtenFormat = humanTrajectory::STASSE;
                break;
            case 1:
                humanTrajectory::writtenFormat = humanTrajectory::KAJITA;
                break;
        }
    }
    
    void MainWindow::on_qFwdRatioCheckBox_stateChanged(int checked) {
        std::cerr << "*** on_qFwdRatioCheckBox_toggled" << std::endl;
        vStruct.setFwdRatio(checked==Qt::Checked);
    }
    
    void MainWindow::on_actionJustShowShortcuts_toggled(bool checked) {
        std::cerr << "*** on_actionJustShowShortcuts_toggled" << std::endl;
        vStruct.setUseOptimization(checked);
        emit(changed());
    }
    
    void MainWindow::on_actionPerformOptimization_toggled(bool checked) {
        std::cerr << "*** on_actionPerformOptimization_toggled" << std::endl;
        vStruct.setOptimization(checked);
        emit(changed());
    }
    
    void MainWindow::on_actionVerbose_toggled(bool checked) {
        std::cerr << "*** on_actionVerbose_toggled" << std::endl;
        vStruct.setVerbose(checked);
        emit(changed());
    }
    
    void MainWindow::on_actionRecomputePath_triggered() {
        std::cerr << "*** on_actionRecomputePath_toggled" << std::endl;
        if (vStruct.nhPath) {
            const queryPoint &start = po->getStart();
            const queryPoint &end   = po->getEnd();
            vStruct.query(start,end);
        }
    }
    
    void MainWindow::on_newLandmark(int nL) {
        int last = vStruct.landmarks.size()-1;
        int first= last + 1 - nL;
        std::cerr << "*** on_newLandmark " << first << " " << last << std::endl;
        for (int i=first;i<=last;i++) {
            // New landmark
            sceneItems.push_back(scene.addEllipse(vStruct.landmarks[i].getLandmark().x() - rVisMin,
                                                  vStruct.landmarks[i].getLandmark().y() - rVisMin,
                                                  2*rVisMin,2*rVisMin,
                                                  QPen(QBrush(Qt::darkBlue),3,Qt::SolidLine)));
            // Outer circle
            sceneItems.push_back(scene.addEllipse(vStruct.landmarks[i].getLandmark().x() - rVisMax,
                                                  vStruct.landmarks[i].getLandmark().y() - rVisMax,
                                                  2*rVisMax,2*rVisMax,
                                                  QPen(QBrush(Qt::darkBlue),3,Qt::SolidLine)));
            // Landmarks
            sceneItems.push_back(scene.addEllipse(vStruct.landmarks[i].getLandmark().x() - rVisMin/4,
                                                  vStruct.landmarks[i].getLandmark().y() - rVisMin/4,
                                                  rVisMin/2, rVisMin/2,
                                                  QPen(QBrush(Qt::darkBlue),3,Qt::SolidLine),
                                                  QBrush(Qt::darkBlue,Qt::SolidPattern)));
            // Text at origin
            char tmp[64];  sprintf(tmp,"Landmark %d",static_cast<int>(i));
            QGraphicsTextItem *ti   = scene.addText(QString(tmp),QFont("Times", 20, QFont::Bold));
            //ti->scale(1,-1);
            ti->scale(1,1);
            ti->setPos(vStruct.landmarks[i].getLandmark().x(),
                       vStruct.landmarks[i].getLandmark().y()+rVisMin/2+5);
            ti->setDefaultTextColor(Qt::darkBlue);
            sceneItems.push_back(ti);
        }
        std::cerr << "*** on_newLandmark done " << std::endl;
    }




}