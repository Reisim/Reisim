//-----------------------------------------------------------------------
//    MDS02 Misaki Design Simulation product - Canopus Project -
//
//                   Misaki Design.LLC,  2019
//-----------------------------------------------------------------------

#include <QApplication>
#include <QtGui>
#include <QStyleFactory>
#include <QTextCodec>
#include <QString>
#include <QDir>
#include <QDebug>

#include <windows.h>

#include "mainwindow.h"
#include "systemthread.h"
#include "networkdrivecheck.h"

#include <QMutex>
#include <QWaitCondition>

QMutex *mutex = NULL;
QWaitCondition *cond = NULL;

QMutex *mutex_sync = NULL;
QWaitCondition *cond_sync = NULL;

QMutex *mutex_log = NULL;
QWaitCondition *cond_log = NULL;


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    AllocConsole();

    QString SysFilePath      = QString();
    QString ConfFilePath     = QString();


    // Get Network Drive Info
    GetNetworkDrive();


    // Mutex and Condition Variable
    mutex = new QMutex();
    cond  = new QWaitCondition();

    mutex_sync = new QMutex();
    cond_sync  = new QWaitCondition();

    mutex_log = new QMutex();
    cond_log  = new QWaitCondition();

    qDebug() << "Current Directory = " << QDir::currentPath();


#ifdef _WINDOWS_
    HANDLE hProcess;
    hProcess = GetCurrentProcess();
    if( SetPriorityClass(hProcess, REALTIME_PRIORITY_CLASS) ){
        qDebug() << "SetPriorityClass: succeeded.";
    }
    else{
        qDebug() << "SetPriorityClass: failed.";
    }


    switch (GetPriorityClass(hProcess)) {
        case IDLE_PRIORITY_CLASS:
            qDebug() << " -> IDLE_PRIORITY_CLASS";
            break;

        case BELOW_NORMAL_PRIORITY_CLASS:
            qDebug() << " -> BELOW_NORMAL_PRIORITY_CLASS";
            break;

        case NORMAL_PRIORITY_CLASS:
            qDebug() << " -> NORMAL_PRIORITY_CLASS";
            break;

        case ABOVE_NORMAL_PRIORITY_CLASS:
            qDebug() << " -> ABOVE_NORMAL_PRIORITY_CLASS";
            break;

        case HIGH_PRIORITY_CLASS:
            qDebug() << " -> HIGH_PRIORITY_CLASS";
            break;

        case REALTIME_PRIORITY_CLASS:
            qDebug() << " -> REALTIME_PRIORITY_CLASS";
            break;
    }
#endif


    qDebug() << "+--- Start Application";

    qDebug() << " Avaiable styles are : " << QStyleFactory::keys();

    //
    //  Create Main Window
    //
    MainWindow w;

    qDebug() << "+--- setCodecForLocale";
    QTextCodec::setCodecForLocale( QTextCodec::codecForLocale() );

    qDebug() << "+--- setStyle -> Fusion";
    QApplication::setStyle(QStyleFactory::create("Fusion"));

    w.setWindowTitle("MDS02-Canopus | Re:sim");
    w.setMinimumSize( QSize(800,600) );
    w.show();



    //
    //  Create System Thread
    //
    SystemThread *sys = new SystemThread();
    if( !sys ){
        qDebug() << "! Cannot create SystemThread !";
        getchar();
        return -1;
    }


    //
    //  Set connections
    //
    QObject::connect( &w, SIGNAL(SetScenraioFile(QString)), sys, SLOT(SetScenarioFile(QString)) );
    QObject::connect( &w, SIGNAL(SetSupplementFile(QString)), sys, SLOT(SetSupplementFile(QString)) );
    QObject::connect( &w, SIGNAL(SetDSMode()), sys, SLOT(SetDSmode()) );
    QObject::connect( &w, SIGNAL(SetStartTrigger(int)), sys, SLOT(SetStartTrigger(int)) );
    QObject::connect( &w, SIGNAL(SetLogFileName(QString)), sys, SLOT(SetLogFileName(QString)) );
    QObject::connect( &w, SIGNAL(SetLogOutputInterval(int)), sys, SLOT(SetLogOutputInterval(int)) );
    QObject::connect( &w, SIGNAL(SetLogOutputFolder(QString)), sys, SLOT(SetLogOutputFolder(QString)) );
    QObject::connect( &w, SIGNAL(SimulationStart()), sys, SLOT(SimulationStart()) );
    QObject::connect( &w, SIGNAL(SimulationStop()), sys, SLOT(SimulationStop()) );
    QObject::connect( &w, SIGNAL(SimulationPause()), sys, SLOT(SimulationPause()) );
    QObject::connect( &w, SIGNAL(SimulationResume()), sys, SLOT(SimulationResume()) );
    QObject::connect( &w, SIGNAL(SetSpeedAdjustVal(int)), sys, SLOT(SetSpeedAdjustVal(int)) );
    QObject::connect( &w, SIGNAL(SetStopGraphicUpdate(bool)), sys, SLOT(SetStopGraphicUpdate(bool)) );
    QObject::connect( w.GetPointerGraphicCanvas(), SIGNAL(SetVehicleParameter(int,int,float,float,float,float,float,float)),
                      sys, SLOT(WrapSetVehicleParameter(int,int,float,float,float,float,float,float)) );
    QObject::connect( w.GetPointerGraphicCanvas(), SIGNAL(ShowAgentData(float,float)),sys, SLOT(ShowAgentData(float,float)) );

    QObject::connect( sys, SIGNAL(UpdateSimulationTimeDisplay(QString)), &w, SLOT(UpdateSimulationTimeDisplay(QString)) );
    QObject::connect( sys, SIGNAL(SetAgentPointer(Agent**)), &w, SLOT(SetAgentPointerToCanvas(Agent**)) );
    QObject::connect( sys, SIGNAL(SetTrafficSignalPointer(QList<TrafficSignal*>)), &w, SLOT(SetTrafficSignalPointerToCanvas(QList<TrafficSignal*>)) );
    QObject::connect( sys, SIGNAL(SetMaxNumberAgent(int)), &w, SLOT(SetMaxNumberAgentToCanvas(int)) );
    QObject::connect( sys, SIGNAL(SetNumberTrafficSignal(int)), &w, SLOT(SetNumberTrafficSignalToCanvas(int)) );
    QObject::connect( sys, SIGNAL(RedrawRequest()), &w, SLOT(RedrawRequest()) );
    QObject::connect( sys, SIGNAL(SetRoadDataToCanvas(Road*)), &w, SLOT(SetRoadDataToCanvas(Road*)) );
    QObject::connect( sys, SIGNAL(ExitProgram()), &w, SLOT(ExitProgram()) );


    //
    //  Copy vehicle shape parameter in graphic canvas to simManager
    //
    w.CopyVehicleShapeParameter();



    //
    //  Check arguments
    //
    qDebug() << "num arg = " << argc;
    for(int i=0;i<argc;++i){
        qDebug() << "[" << i << "]" << argv[i];

        QString tmpStr = QString( argv[i] );
        if( tmpStr.startsWith("-WithArg") ){

            QString argFileName = QApplication::applicationDirPath() + QString("/Reisim.arg.txt");
            qDebug() << "argFileName = " << argFileName;

            QFile argFile( CheckNetworkDrive(argFileName) );
            if( argFile.open(QIODevice::ReadOnly | QIODevice::Text) == true ){

                QTextStream in(&argFile);

                QString NetLine = in.readLine();
                SysFilePath = NetLine.remove("-Net=").trimmed();

                qDebug() << "SysFilePath = " << SysFilePath;

                QString ConfLine = in.readLine();
                ConfFilePath = ConfLine.remove("-Conf=").trimmed();

                qDebug() << "ConfFilePath = " << ConfFilePath;

                argFile.close();
            }
            else{
                qDebug() << "Cannot open argFile";
            }
        }
    }

    if( ConfFilePath.isNull() == false ){
        qDebug() << "Set Config File";
        w.LoadSettingFile( ConfFilePath );
    }

    if( SysFilePath.isNull() == false ){
        qDebug() << "Set Network File";
        sys->SetSysFile( SysFilePath );
    }

    qDebug() << "Enter main loop ...";
    qDebug() << "-----";


    //
    //
    //
    int ret = a.exec();

    ReleaseNetworkDriveInfo();

    return ret;
}

