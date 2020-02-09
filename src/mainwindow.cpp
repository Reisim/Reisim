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
#include <QDebug>
#include <time.h>

#include "networkdrivecheck.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    QWidget *cWidget = new QWidget();

    canvas = new GraphicCanvas();

    confWin = new ConfigWindow();
    confWin->hide();


    //--------
    QHBoxLayout *controlLayout = new QHBoxLayout();

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

    animeSpeedAdjuster = new QSlider( Qt::Horizontal );
    animeSpeedAdjuster->setMinimum(0);
    animeSpeedAdjuster->setMaximum(200);
    animeSpeedAdjuster->setValue(50);
    animeSpeedAdjuster->setFixedWidth(100);
    connect(animeSpeedAdjuster,SIGNAL(valueChanged(int)),this,SLOT(SpeedAdjustMoved(int)));
    connect(animeSpeedAdjusterByValue,SIGNAL(valueChanged(int)),animeSpeedAdjuster,SLOT(setValue(int)));


    resetSpeedAdjuster = new QPushButton();
    resetSpeedAdjuster->setIcon(QIcon(":/images/reset.png"));
    resetSpeedAdjuster->setFixedWidth(25);
    connect(resetSpeedAdjuster,SIGNAL(clicked(bool)),this,SLOT(ResetSpeedAdjuster()));

    cbShowVID = new QCheckBox("Show ID Label");
    cbShowVID->setChecked(true);
    connect(cbShowVID,SIGNAL(toggled(bool)),this,SLOT(ShowVIDChanged(bool)));

    cbShowPathID = new QCheckBox("Show Path Label");
    cbShowPathID->setChecked(false);
    connect(cbShowPathID,SIGNAL(toggled(bool)),this,SLOT(ShowPathIDChanged(bool)));

    cbShowTSID = new QCheckBox("Show TS Label");
    cbShowTSID->setChecked(false);
    connect(cbShowTSID,SIGNAL(toggled(bool)),this,SLOT(ShowTSIDChanged(bool)));

    cbStopGraphicUpdate = new QCheckBox("Stop Graphic Update");
    cbStopGraphicUpdate->setChecked(false);
    connect(cbStopGraphicUpdate,SIGNAL(toggled(bool)),this,SLOT(StopGraphicUpdateChanged(bool)));


    controlLayout->addWidget( startBtn );
    controlLayout->addWidget( pauseBtn );
    controlLayout->addWidget( resumeBtn );
    controlLayout->addWidget( stopBtn );
    controlLayout->addWidget( animeSpeedAdjusterByValue );
    controlLayout->addWidget( animeSpeedAdjuster );
    controlLayout->addWidget( resetSpeedAdjuster );
    controlLayout->addWidget( cbShowVID );
    controlLayout->addWidget( cbShowPathID );
    controlLayout->addWidget( cbShowTSID );
    controlLayout->addWidget( cbStopGraphicUpdate );
    controlLayout->addStretch();

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

}

MainWindow::~MainWindow()
{

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


void MainWindow::LoadSettingFile(QString filename)
{
    qDebug() << "[MainWindow::LoadSettingFile] filename = " << filename;

    if( filename.isNull() ){
        return;
    }

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

            emit SetScenraioFile( scenarioFile );

        }
        else if( tag.contains("Supplement File") ){
            QString supplementFile = QString(divLine[1]).trimmed();
            emit SetSupplementFile( supplementFile );
        }
        else if( tag.contains("DS Mode") ){
            QString modeStr = QString(divLine[1]).trimmed();
            if( modeStr == "yes" ){
                qDebug() << "!!!  SetDSMode emiited.";
                emit SetDSMode();
                cbStopGraphicUpdate->setChecked(true);
                emit SetStopGraphicUpdate(true);
            }
            else{
                tmpDSmode = false;
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

    }
    file.close();
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

    LoadSettingFile(fileName);
}



void MainWindow::StartSimulation()
{
    qDebug() << "[MainWindow::StartSimulation]";
    emit SimulationStart();
}


void MainWindow::StopSimulation()
{
    qDebug() << "[MainWindow::StopSimulation]";
    emit SimulationStop();
}


void MainWindow::PauseSimulation()
{
    qDebug() << "[MainWindow::PauseSimulation]";
    emit SimulationPause();
}


void MainWindow::ResumeSimulation()
{
    qDebug() << "[MainWindow::ResumeSimulation]";
    emit SimulationResume();
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


void MainWindow::CopyVehicleShapeParameter()
{
    canvas->CopyVehicleShapeParameter();
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

