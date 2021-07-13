/************************************************************************
**                                Re:sim
**
**   Copyright (C) 2020 Misaki Design.LLC
**   Copyright (C) 2020 Jun Tajima <tajima@misaki-design.co.jp>
**
**   This file is part of the Re:sim simulation software
**
**   This software is released under the GNU Lesser General Public
**   License version 3, see LICENSE.
*************************************************************************/


#include "mainwindow.h"
#include <QApplication>
#include <QInputDialog>
#include <QDebug>
#include <time.h>

#include "networkdrivecheck.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{

    ReadPreferenceSetting();


    //--------

    QWidget *cWidget = new QWidget();

    canvas = new GraphicCanvas();

    confWin = new ConfigWindow();
    confWin->setWindowIcon( QIcon(":images/resim-icon.png") );
    confWin->hide();

    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Eye Height") == true ){
        canvas->SetZeye( preferenceSetting.value( QString("Eye Height") ) );
    }


    //--------
    QVBoxLayout *controlLayout = new QVBoxLayout();

    startBtn = new QPushButton();
    startBtn->setIcon(QIcon(":/images/Play.png"));
    startBtn->setFixedWidth(50);
    connect(startBtn,SIGNAL(clicked(bool)),this,SLOT(StartSimulation()));

    stopBtn = new QPushButton();
    stopBtn->setIcon(QIcon(":/images/Stop.png"));
    stopBtn->setFixedWidth(50);
    connect(stopBtn,SIGNAL(clicked(bool)),this,SLOT(StopSimulation()));

    pauseBtn = new QPushButton();
    pauseBtn->setIcon(QIcon(":/images/pause.png"));
    pauseBtn->setFixedWidth(50);
    connect(pauseBtn,SIGNAL(clicked(bool)),this,SLOT(PauseSimulation()));

    resumeBtn = new QPushButton();
    resumeBtn->setIcon(QIcon(":/images/resume.png"));
    resumeBtn->setFixedWidth(50);
    connect(resumeBtn,SIGNAL(clicked(bool)),this,SLOT(ResumeSimulation()));

    animeSpeedAdjusterByValue = new QSpinBox();
    animeSpeedAdjusterByValue->setMaximum(0);
    animeSpeedAdjusterByValue->setMaximum(200);
    animeSpeedAdjusterByValue->setValue(50);
    animeSpeedAdjusterByValue->setFixedWidth(60);

    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Animation Speed Value Adjuster") == true ){
        animeSpeedAdjusterByValue->setValue( preferenceSetting.value( QString("Animation Speed Value Adjuster") ) );
    }

    animeSpeedAdjuster = new QSlider( Qt::Horizontal );
    animeSpeedAdjuster->setMinimum(0);
    animeSpeedAdjuster->setMaximum(200);
    animeSpeedAdjuster->setValue(50);
    animeSpeedAdjuster->setFixedWidth(100);

    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Animation Speed Adjuster") == true ){
        animeSpeedAdjuster->setValue( preferenceSetting.value( QString("Animation Speed Adjuster") ) );
    }

    connect(animeSpeedAdjuster,SIGNAL(valueChanged(int)),this,SLOT(SpeedAdjustMoved(int)));
    connect(animeSpeedAdjusterByValue,SIGNAL(valueChanged(int)),animeSpeedAdjuster,SLOT(setValue(int)));

    resetSpeedAdjuster = new QPushButton();
    resetSpeedAdjuster->setIcon(QIcon(":/images/reset.png"));
    resetSpeedAdjuster->setFixedWidth(25);
    connect(resetSpeedAdjuster,SIGNAL(clicked(bool)),this,SLOT(ResetSpeedAdjuster()));

    resetZeye = new QPushButton("Reset Eye Height");
    resetZeye->setIcon(QIcon(":/images/reset.png"));
    resetZeye->setFixedWidth( resetZeye->sizeHint().width() );
    connect(resetZeye,SIGNAL(clicked(bool)),this,SLOT(ResetZeye()));

    cbShowPathCG = new QCheckBox("Show Path");
    cbShowPathCG->setChecked(true);
    connect(cbShowPathCG,SIGNAL(toggled(bool)),this,SLOT(ShowPathCGChanged(bool)));

    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Show Path CG") == true ){
        if( preferenceSetting.value( QString("Show Path CG") ) == 1 ){
            cbShowPathCG->setChecked(true);
        }
        else{
            cbShowPathCG->setChecked(false);
        }
    }

    cbShowTSCG = new QCheckBox("Show TS");
    cbShowTSCG->setChecked(true);
    connect(cbShowTSCG,SIGNAL(toggled(bool)),this,SLOT(ShowTSCGChanged(bool)));

    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Show TS CG") == true ){
        if( preferenceSetting.value( QString("Show TS CG") ) == 1 ){
            cbShowTSCG->setChecked(true);
        }
        else{
            cbShowTSCG->setChecked(false);
        }
    }

    cbShowBaseMap = new QCheckBox("Show Base-Map Image");
    cbShowBaseMap->setChecked(true);
    connect(cbShowBaseMap,SIGNAL(toggled(bool)),this,SLOT(ShowBaseMapChanged(bool)));

    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Show Base-map Image") == true ){
        if( preferenceSetting.value( QString("Show Base-map Image") ) == 1 ){
            cbShowBaseMap->setChecked(true);
        }
        else{
            cbShowBaseMap->setChecked(false);
        }
    }

    cbShowVID = new QCheckBox("Show ID Label");
    cbShowVID->setChecked(true);
    connect(cbShowVID,SIGNAL(toggled(bool)),this,SLOT(ShowVIDChanged(bool)));

    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Show ID Label") == true ){
        if( preferenceSetting.value( QString("Show ID Label") ) == 0 ){
            cbShowVID->setChecked(false);
        }
    }

    cbShowPathID = new QCheckBox("Show Path Label");
    cbShowPathID->setChecked(false);
    connect(cbShowPathID,SIGNAL(toggled(bool)),this,SLOT(ShowPathIDChanged(bool)));

    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Show PATH Label") == true ){
        if( preferenceSetting.value( QString("Show PATH Label") ) == 1 ){
            cbShowPathID->setChecked(true);
        }
    }

    cbShowTSID = new QCheckBox("Show TS Label");
    cbShowTSID->setChecked(false);
    connect(cbShowTSID,SIGNAL(toggled(bool)),this,SLOT(ShowTSIDChanged(bool)));

    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Show TS Label") == true ){
        if( preferenceSetting.value( QString("Show TS Label") ) == 1 ){
            cbShowTSID->setChecked(true);
        }
    }

    cbStopGraphicUpdate = new QCheckBox("Stop Graphic Update");
    cbStopGraphicUpdate->setChecked(false);
    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Stop Graphic Update") == true ){
        if( preferenceSetting.value( QString("Stop Graphic Update") ) == 1 ){
            cbStopGraphicUpdate->setChecked(true);
        }
    }
    connect(cbStopGraphicUpdate,SIGNAL(toggled(bool)),this,SLOT(StopGraphicUpdateChanged(bool)));

    fontScaler = new QSlider( Qt::Horizontal );
    fontScaler->setMinimum(1);
    fontScaler->setMaximum(200);
    fontScaler->setValue(14);
    fontScaler->setFixedWidth(100);

    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Font Scale") == true ){
        fontScaler->setValue( preferenceSetting.value( QString("Font Scale") ) );
    }

    connect(fontScaler,SIGNAL(valueChanged(int)),this,SLOT(SetFontScale(int)));
    connect(canvas,SIGNAL(ChangeFontScale(int)),fontScaler,SLOT(setValue(int)));

    SInterfaceScalerLabel = new QLabel("S-Obj Scale:");
    SInterfaceScalerLabel->setHidden(true);

    SInterfaceScaler = new QSlider( Qt::Horizontal );
    SInterfaceScaler->setMinimum(0);
    SInterfaceScaler->setMaximum(800);
    SInterfaceScaler->setValue(0);
    SInterfaceScaler->setFixedWidth(100);
    connect(SInterfaceScaler,SIGNAL(valueChanged(int)),canvas,SLOT(SetScaleSInterface(int)));
    SInterfaceScaler->setHidden(true);

    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("S-Interface Object Scale") == true ){
        SInterfaceScaler->setValue( preferenceSetting.value( QString("S-Interface Object Scale") ) );
    }

    snapshotBtn = new QPushButton("Snapshot");
    snapshotBtn->setIcon(QIcon(":/images/Pin.png"));
    snapshotBtn->setFixedSize( snapshotBtn->sizeHint() );
    connect(snapshotBtn,SIGNAL(clicked(bool)),this,SLOT(OutputRestartData()));

    cbFixCameraToObj = new QCheckBox("Fix Camera to Object");
    cbFixCameraToObj->setChecked(false);
    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Fix Camera to Object") == true ){
        if( preferenceSetting.value( QString("Fix Camera to Object") ) == 1 ){
            cbFixCameraToObj->setChecked(true);
        }
    }
    connect(cbFixCameraToObj,SIGNAL(toggled(bool)),this,SLOT(FixCameraToObjChanged(bool)));

    cameraFixToObjID = new QSpinBox();
    cameraFixToObjID->setMaximum(0);
    cameraFixToObjID->setMaximum(10000);
    cameraFixToObjID->setValue(1);
    cameraFixToObjID->setPrefix("ID:");
    cameraFixToObjID->setFixedWidth(100);
    if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Camera Fix Object ID") == true ){
        cameraFixToObjID->setValue( preferenceSetting.value( QString("Camera Fix Object ID") ) );
    }

    QHBoxLayout *controlLayout1 = new QHBoxLayout();
    QHBoxLayout *controlLayout2 = new QHBoxLayout();
    QHBoxLayout *controlLayout3 = new QHBoxLayout();

    controlLayout1->addWidget( startBtn );
    controlLayout1->addWidget( pauseBtn );
    controlLayout1->addWidget( resumeBtn );
    controlLayout1->addWidget( stopBtn );
    controlLayout1->addWidget( new QLabel("Simulation Speed Control:"));
    controlLayout1->addWidget( animeSpeedAdjusterByValue );
    controlLayout1->addWidget( animeSpeedAdjuster );
    controlLayout1->addWidget( resetSpeedAdjuster );
    controlLayout1->addWidget( snapshotBtn );
    controlLayout1->addWidget( SInterfaceScalerLabel );
    controlLayout1->addWidget( SInterfaceScaler );
    controlLayout1->addStretch();

    controlLayout2->addWidget( cbShowPathCG );
    controlLayout2->addWidget( cbShowTSCG );
    controlLayout2->addWidget( cbShowBaseMap );
    controlLayout2->addWidget( cbShowVID );
    controlLayout2->addWidget( cbShowPathID );
    controlLayout2->addWidget( cbShowTSID );
    controlLayout2->addStretch();

    controlLayout3->addWidget( cbStopGraphicUpdate );
    controlLayout3->addWidget( new QLabel("Font Scaler:"));
    controlLayout3->addWidget( fontScaler );
    controlLayout3->addWidget( cbFixCameraToObj );
    controlLayout3->addWidget( cameraFixToObjID );
    controlLayout3->addWidget( resetZeye );
    controlLayout3->addStretch();

    controlLayout->addLayout( controlLayout1 );
    controlLayout->addLayout( controlLayout2 );
    controlLayout->addLayout( controlLayout3 );


    //--------
    simulationTimeDisplay = new QLabel();
    simulationTimeDisplay->setFixedHeight(40);

    QHBoxLayout *simTimeLayout = new QHBoxLayout();
    simTimeLayout->addWidget( new QLabel("Time: ") );
    simTimeLayout->addWidget( simulationTimeDisplay );
    simTimeLayout->addStretch();



    //--------
    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->addLayout( simTimeLayout );
    mainLayout->addWidget( canvas );
    mainLayout->addLayout( controlLayout );

    cWidget->setLayout( mainLayout );

    //--------
    setCentralWidget( cWidget );


    //--------
    openSimSetting = new QAction();
    openSimSetting->setStatusTip( tr("Open") );
    openSimSetting->setIcon(QIcon(":/images/load_document.png"));
    connect( openSimSetting, SIGNAL(triggered()), this, SLOT(OpenSettingFile()));

    editConfig = new QAction();
    editConfig->setStatusTip( tr("Edit Config") );
    editConfig->setIcon(QIcon(":/images/setting.png"));
    connect( editConfig, SIGNAL(triggered()), this, SLOT(ShowConfigWindow()));


    actionToolBar = addToolBar("Action");
    actionToolBar->addAction(openSimSetting);
    actionToolBar->addAction(editConfig);

    DSMode = false;
    simState = 0;
}

MainWindow::~MainWindow()
{
    WritePreferenceSetting();
}


void MainWindow::ExitProgram()
{
    if( confWin ){
        confWin->close();
    }

    close();
}


void MainWindow::ShowConfigWindow()
{
    confWin->show();
}


void MainWindow::LoadSettingFile(QString filename,bool withArg)
{
    qDebug() << "[MainWindow::LoadSettingFile] filename = " << filename;

    if( filename.isNull() ){
        return;
    }

    QString tmpfilename = filename;

    QStringList divFileName = tmpfilename.replace("\\","/").split("/");
    QString pureFileName = QString(divFileName.last());
    QString settingFolder = tmpfilename.remove( pureFileName );

    qDebug() << "pureFileName = " << pureFileName;
    qDebug() << "settingFolder = " << settingFolder;


    QFile file( CheckNetworkDrive(filename) );
    if( !file.open(QIODevice::ReadOnly | QIODevice::Text) ){
        QMessageBox::warning(this,"Error","Cannot open file: " + filename);
        return;
    }

    QTextStream in(&file);
    QString line;

    line = in.readLine();
    line = in.readLine();
    if( line.contains("Re:sim Configulation File") == false ){
        QMessageBox::warning(this,"Error","The file is not Re:sim Configulation File");
        file.close();
        return;
    }
    line = in.readLine();

    bool tmpDSmode = true;
    bool tmpLogOutput = false;
    QString tempLogFileName;
    while( in.atEnd() == false ){

        line = in.readLine();
        if( line.startsWith("#") || line.isEmpty() || line.contains(";") == false ){
            continue;
        }

        QStringList divLine = line.split(";");
        QString tag = QString( divLine[0] );
        if( tag.contains("Scenario File") ){
            QString scenarioFile = QString(divLine[1]).trimmed();

            QString reconstFilename = scenarioFile;

            // Check if the path is absolute or relative
            if( scenarioFile.contains(":") == false ){

                reconstFilename = settingFolder + scenarioFile;

                qDebug() << "Reconstructed scenario filename = " << reconstFilename;
            }

            emit SetScenraioFile( reconstFilename );
        }
        else if( tag.contains("Base-Map Files") ){
            QString basemapfile = QString(divLine[1]).trimmed();
            emit SetBasemapFile(basemapfile,settingFolder);
        }
        else if( tag.contains("Supplement File") ){
            QString supplementFile = QString(divLine[1]).trimmed();
            emit SetSupplementFile( supplementFile );
        }
        else if( tag.contains("DS Mode") ){
            QString modeStr = QString(divLine[1]).trimmed();
            if( modeStr == "yes" ){

                if( withArg == false ){
                    QMessageBox::warning(this,"DS Mode Configulation","Can not use DS Mode configulation file.");
                    return;
                }

                qDebug() << "!!!  SetDSMode emiited.";
                emit SetDSMode();

                if( preferenceSetting.empty() == false && preferenceSetting.keys().contains("Stop Graphic Update") == true ){
                    if( preferenceSetting.value( QString("Stop Graphic Update") ) == 1 ){
                        cbStopGraphicUpdate->setChecked(true);
                        emit SetStopGraphicUpdate(true);
                    }
                }
                else{
                    cbStopGraphicUpdate->setChecked(true);
                    emit SetStopGraphicUpdate(true);
                }

                DSMode = true;

                SInterfaceScalerLabel->setHidden( false );
                SInterfaceScaler->setHidden( false );
            }
            else{
                qDebug() << "Simulation Mode";
                tmpDSmode = false;
            }
        }
        else if( tag.contains("Calculation Time Step") ){

            double ts = QString(divLine[1]).trimmed().toDouble();
            int hz = (int)(1.0 / ts);

            qDebug() << "Calculation Time Step : " << ts << " , hz = " << hz;

            if( tmpDSmode == false ){

                emit SetSimulationFrequency( hz );
            }
        }
        else if( tag.contains("Start Trigger") ){
            int startTrigger = QString(divLine[1]).trimmed().toInt();
            emit SetStartTrigger( startTrigger );
        }
        else if( tag.contains("Output Log File") ){
            QString modeStr = QString(divLine[1]).trimmed();
            if( modeStr == "yes" ){
                tmpLogOutput = true;
            }
            else{
                tmpLogOutput = false;
            }
        }
        else if( tag.contains("Log Output Folder") ){
            QString logOutputFolder = QString(divLine[1]).trimmed();
            if( tmpLogOutput == true ){
                emit SetLogOutputFolder( logOutputFolder );
            }
        }
        else if( tag.contains("CSV Output File") ){
            QString outputFile = QString(divLine[1]).trimmed();
            if( tmpLogOutput == true &&
                    outputFile.isNull() == false &&
                    outputFile.isEmpty() == false &&
                    outputFile.contains(".") ){
                emit SetLogFileName( outputFile );

                tempLogFileName = outputFile;
            }
        }
        else if( tag.contains("Add Timestamp to Filename") ){
            QString addTS = QString(divLine[1]).trimmed();
            if( addTS == "yes" ){

                time_t ctime;
                ctime = time(NULL);
                struct tm *time_st = localtime( &ctime );

                QString TSStr = QString("_%1%2%3_%4%5_%6.vbf.csv")
                        .arg(time_st->tm_year+1900,4,10,QChar('0'))
                        .arg(time_st->tm_mon+1,2,10,QChar('0')).arg(time_st->tm_mday,2,10,QChar('0'))
                        .arg(time_st->tm_hour,2,10,QChar('0')).arg(time_st->tm_min,2,10,QChar('0'))
                        .arg(time_st->tm_sec,2,10,QChar('0'));

                qDebug() << "TSStr = " << TSStr;
                tempLogFileName.replace(".vbf.csv","_mod");
                tempLogFileName.replace("_mod",TSStr);

                emit SetLogFileName( tempLogFileName );
            }
        }
        else if( tag.contains("Log Output Interval") ){
            int interval = QString(divLine[1]).trimmed().toInt();
            emit SetLogOutputInterval(interval);
        }
        else if( tag.contains("Restart File") ){
            QString restartFilename = QString(divLine[1]).trimmed();
            emit SetRestartFile( restartFilename );
        }

    }
    file.close();

    qDebug() << "----- End of LoadSettingFile -----";
}


void MainWindow::OpenSettingFile()
{
    qDebug() << "[OpenSettingFile]";

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Choose File"),
                                                    ".",
                                                    tr("re:sim config file(*.rc.txt)"));

    if( fileName.isNull() == true ){
        qDebug() << "Open action canceled.";
        return;
    }

    LoadSettingFile(fileName,false);
}



void MainWindow::StartSimulation()
{
    qDebug() << "[MainWindow::StartSimulation]";
    emit SimulationStart();
    simState = 1;
}


void MainWindow::StopSimulation()
{
    qDebug() << "[MainWindow::StopSimulation]";
    emit SimulationStop();
    simState = 3;
}


void MainWindow::PauseSimulation()
{
    qDebug() << "[MainWindow::PauseSimulation]";
    emit SimulationPause();
    simState = 2;
}


void MainWindow::ResumeSimulation()
{
    qDebug() << "[MainWindow::ResumeSimulation]";
    emit SimulationResume();
    simState = 1;
}


void MainWindow::UpdateSimulationTimeDisplay(QString timeStr)
{
    simulationTimeDisplay->setText( timeStr );
}


void MainWindow::SetAgentPointerToCanvas(Agent **p)
{
    canvas->agent = p;
}

void MainWindow::SetTrafficSignalPointerToCanvas(QList<TrafficSignal *> p)
{
    canvas->trafficSignal = p;

    qDebug() << "[MainWindow::SetTrafficSignalPointerToCanvas] canvas->trafficSignal.size = " << canvas->trafficSignal.size();

    canvas->SetTSData();
}


void MainWindow::SetMaxNumberAgentToCanvas(int n)
{
    canvas->maxAgent = n;
}


void MainWindow::SetNumberTrafficSignalToCanvas(int n)
{
    canvas->numTrafficSignal = n;
}


void MainWindow::RedrawRequest()
{
    canvas->update();
}


void MainWindow::SetRoadDataToCanvas(Road *road)
{
    qDebug() << "[MainWindow::SetRoadDataToCanvas]";
    canvas->road = road;

    qDebug() << "  call SetRoadData()";
    canvas->SetRoadData();

    qDebug() << "  call SetTrafficParticipantsData()";
    canvas->SetTrafficParticipantsData();
}


void MainWindow::SpeedAdjustMoved(int val)
{
    animeSpeedAdjusterByValue->setValue(val);
    emit SetSpeedAdjustVal( val );
}


void MainWindow::ResetSpeedAdjuster()
{
    animeSpeedAdjuster->setValue( 50 );
}


void MainWindow::ResetZeye()
{
    canvas->SetZeye( -50.0 );
    canvas->ResetView();
    canvas->update();
}


void MainWindow::ShowPathCGChanged(bool v)
{
    canvas->SetPathCGFlag(v);
    canvas->update();
}


void MainWindow::ShowTSCGChanged(bool v)
{
    canvas->SetTSCGFlag(v);
    canvas->update();
}


void MainWindow::ShowBaseMapChanged(bool v)
{
    canvas->SetBaseMapFlag(v);
    canvas->update();
}


void MainWindow::ShowVIDChanged(bool v)
{
    canvas->SetVIDFlag(v);
    canvas->update();
}


void MainWindow::ShowTSIDChanged(bool v)
{
    canvas->SetTSIDFlag(v);
    canvas->update();
}


void MainWindow::ShowPathIDChanged(bool v)
{
    canvas->SetPathIDFlag(v);
    canvas->update();
}


void MainWindow::StopGraphicUpdateChanged(bool v)
{
    qDebug() << "emit StopGraphicUpdateChanged = " << v;
    emit SetStopGraphicUpdate(v);
}


void MainWindow::FixCameraToObjChanged(bool v)
{
    qDebug() << "FixCameraToObjChanged";

    canvas->trackingMode = v;

    if( v == true ){
        canvas->trackingObjID = cameraFixToObjID->value();
        qDebug() << "trackingObjID = " << canvas->trackingObjID;

        canvas->PushEyeCoord();
        qDebug() << "PushEyeCoord";
    }
    else{
        canvas->PopEyeCoord();
        qDebug() << "PopEyeCoord";
    }
}


void MainWindow::SetFontScale(int s)
{
    canvas->SetFontScale(s);
    canvas->update();
}


void MainWindow::keyPressEvent(QKeyEvent *e)
{
    Qt::KeyboardModifiers modifi = QApplication::keyboardModifiers();
    int key = e->key();

    if( modifi & Qt::AltModifier ){
        if( key == Qt::Key_F ){

            SimulationPause();

            int id = QInputDialog::getInt(this,"Search Agent","ID");
            canvas->LocateAtAgent(id);
        }
        else if( key == Qt::Key_T ){

            SimulationPause();

            QString tmpStopTimeStr = QInputDialog::getText(this,"Set Temp-stop Time","H:M:S");
            QStringList tmpStopTimeStrDiv = tmpStopTimeStr.split(":");
            if( tmpStopTimeStrDiv.size() == 1 ){
                int sec = QString( tmpStopTimeStrDiv[0] ).trimmed().toInt();
                emit SetTmpStpoTime(0,0,sec);
            }
            else if( tmpStopTimeStrDiv.size() == 2 ){
                int min = QString( tmpStopTimeStrDiv[0] ).trimmed().toInt();
                int sec = QString( tmpStopTimeStrDiv[1] ).trimmed().toInt();
                emit SetTmpStpoTime(0,min,sec);
            }
            else if( tmpStopTimeStrDiv.size() == 3 ){
                int hur = QString( tmpStopTimeStrDiv[0] ).trimmed().toInt();
                int min = QString( tmpStopTimeStrDiv[1] ).trimmed().toInt();
                int sec = QString( tmpStopTimeStrDiv[2] ).trimmed().toInt();
                emit SetTmpStpoTime(hur,min,sec);
            }

            SimulationResume();
        }
        else if( key >= Qt::Key_0 && key <= Qt::Key_9 ){

            emit SetDSMoveTarget( key - Qt::Key_0 );
        }
    }
}


void MainWindow::OutputRestartData()
{
    if( DSMode == true ){
        return;
    }
    if( simState == 0 ){
        return;
    }

    SimulationPause();

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Output Restart Data"),
                                                    ".",
                                                    tr("re:sim snapshot file(*.ss.txt)"));

    if( fileName.isNull() == false ){

        if( fileName.endsWith(".ss.txt") == false ){
            fileName += QString(".ss.txt");
        }

        qDebug() << "[MainWindow::OutputRestartData] fileName = " << fileName;

        emit OutputRestartData( fileName );
    }

    SimulationResume();
}


void MainWindow::WritePreferenceSetting()
{
    QFile pFile( QApplication::applicationDirPath() + QString("/preference_setting.txt") );

    if( pFile.open( QIODevice::WriteOnly | QIODevice::Text) == false ){
        qDebug() << "Cannot open preference_setting.txt";
        return;
    }


    QTextStream out(&pFile);

    out << "Stop Graphic Update ; " << (cbStopGraphicUpdate->isChecked() == true ? 1 : 0) << "\n";
    out << "Fix Camera to Object ; " << (cbFixCameraToObj->isChecked() == true ? 1 : 0) << "\n";
    out << "Camera Fix Object ID ; " << cameraFixToObjID->value() << "\n";
    out << "Show Path CG ; " << (cbShowPathCG->isChecked() == true ? 1 : 0) << "\n";
    out << "Show TS CG ; " << (cbShowTSCG->isChecked() == true ? 1 : 0) << "\n";
    out << "Show Base-map Image ; " << (cbShowBaseMap->isChecked() == true ? 1 : 0) << "\n";
    out << "Eye Height ; " << canvas->GetZeye() << "\n";
    out << "S-Interface Object Scale ; " << SInterfaceScaler->value() << "\n";
    out << "Show ID Label ; " << (cbShowVID->isChecked() == true ? 1 : 0) << "\n";
    out << "Show TS Label ; " << (cbShowTSID->isChecked() == true ? 1 : 0) << "\n";
    out << "Show PATH Label ; " << (cbShowPathID->isChecked() == true ? 1 : 0) << "\n";
    out << "Font Scale ; " << fontScaler->value() << "\n";
    out << "Animation Speed Adjuster ; " << animeSpeedAdjuster->value() << "\n";
    out << "Animation Speed Value Adjuster ; " << animeSpeedAdjusterByValue->value() << "\n";

    pFile.close();
}


void MainWindow::ReadPreferenceSetting()
{
    qDebug() << "[ReadPreferenceSetting]" << QApplication::applicationDirPath() + QString("/preference_setting.txt");

    QFile pFile( QApplication::applicationDirPath() + QString("/preference_setting.txt") );

    if( pFile.open( QIODevice::ReadOnly | QIODevice::Text) == false ){
        qDebug() << "No preference_setting.txt";
        return;
    }

    preferenceSetting.clear();

    QTextStream in(&pFile);

    qDebug() << "Preference:";

    while(in.atEnd() == false ){
        QString aLine = in.readLine();
        if( aLine.isNull() == true || aLine.isEmpty() == true || aLine.startsWith("#") == true ){
            continue;
        }

        QStringList divLine = aLine.split(";");
        if( divLine.size() != 2 ){
            continue;
        }

        QString key = QString(divLine[0]).trimmed();
        if( key.contains("Eye Height")){
            int val = (int)QString(divLine[1]).trimmed().toFloat();

            preferenceSetting.insert( key, val );
            qDebug() << key << " " << val;
        }
        else{
            int val = QString(divLine[1]).trimmed().toInt();

            preferenceSetting.insert( key, val );
            qDebug() << key << " " << val;
        }
    }

    pFile.close();
}


void MainWindow::ShowOptionalImage(int id,float x,float y,float z,float rot,float s)
{
    if( id < 0 || id >= canvas->optionalImages.size() ){
        return;
    }

    canvas->optionalImages[id]->x = x;
    canvas->optionalImages[id]->y = y;
    canvas->optionalImages[id]->z = z;
    canvas->optionalImages[id]->rotate = rot;
    canvas->optionalImages[id]->scale = s;

    canvas->optionalImages[id]->showImage = true;
}


void MainWindow::HideOptionalImage(int id)
{
    if( id < 0 || id >= canvas->optionalImages.size() ){
        return;
    }

    canvas->optionalImages[id]->showImage = false;
}
