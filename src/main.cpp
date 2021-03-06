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
#include "caldistributethread.h"


#include <QMutex>
#include <QWaitCondition>



int get_logical_processor_info (int type);


QMutex *mutex = NULL;
QWaitCondition *cond = NULL;

QMutex *mutex_sync = NULL;
QWaitCondition *cond_sync = NULL;

QMutex *mutex_log = NULL;
QWaitCondition *cond_log = NULL;


int maxWorkerThread = 8;
QList<QWaitCondition *> workerCond;
QList<int> statusWorkerThread;
QMutex *workerMutex = NULL;
bool allWorkerThreadFinshed = false;
QWaitCondition *condSimMain;


QFile sysLogOutFile;


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    AllocConsole();

    QString SysFilePath      = QString();
    QString ConfFilePath     = QString();


    qDebug() << "Application Directory = " << QApplication::applicationDirPath();


    sysLogOutFile.setFileName(QApplication::applicationDirPath() + QString("/resim_exe_log.txt"));
    if( sysLogOutFile.open( QIODevice::WriteOnly | QIODevice::Text ) ){

        qDebug() << "Open exe-log file: " << sysLogOutFile.fileName();

        QTextStream sysLogOut(&sysLogOutFile);
        sysLogOut << "+--- Start Re:sim" << "\n";
        sysLogOut.flush();
        sysLogOutFile.close();
    }
    else{
        qDebug() << "Cannot open exe-log file: " << sysLogOutFile.fileName();
    }


    // Get CoreCount
    maxWorkerThread = get_logical_processor_info(4);
    qDebug() << "Logical Core Count = " << maxWorkerThread;


    // Get Network Drive Info
    GetNetworkDrive();


    // Mutex and Condition Variable
    mutex = new QMutex();
    cond  = new QWaitCondition();

    mutex_sync = new QMutex();
    cond_sync  = new QWaitCondition();

    mutex_log = new QMutex();
    cond_log  = new QWaitCondition();

    workerMutex = new QMutex();
    for(int i=0;i<maxWorkerThread;++i){
        QWaitCondition *wc = new QWaitCondition();
        workerCond.append( wc );
        statusWorkerThread.append( 0 );
    }
    condSimMain = new QWaitCondition();




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
    w.setMinimumSize( QSize(800,800) );
    w.setWindowIcon( QIcon(":images/resim-icon.png") );
    w.show();

    if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
        QTextStream sysLogOut(&sysLogOutFile);
        sysLogOut << "Main Windows created." << "\n";
        sysLogOut.flush();
        sysLogOutFile.close();
    }


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
    //  Create Worker Thread
    CalDistributeThread **workerThread = new CalDistributeThread* [maxWorkerThread];
    for(int i=0;i<maxWorkerThread;++i){
        workerThread[i] = new CalDistributeThread();
        if( !workerThread ){
            qDebug() << "! Cannot create CalDistributeThread !";
            getchar();
            return -1;
        }

        workerThread[i]->evalIDs = new QList<int>;
        workerThread[i]->pWorkingMode = sys->pWorkingMode;

        workerThread[i]->setThreadID(i);
        workerThread[i]->start();

        sys->evalIDs.append( workerThread[i]->evalIDs );
    }

    if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
        QTextStream sysLogOut(&sysLogOutFile);
        sysLogOut << "Threads created." << "\n";
        sysLogOut.flush();
        sysLogOutFile.close();
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
    QObject::connect( &w, SIGNAL(SetSimulationFrequency(int)), sys, SLOT(SetSimulationFrequency(int)) );
    QObject::connect( &w, SIGNAL(SimulationStart()), sys, SLOT(SimulationStart()) );
    QObject::connect( &w, SIGNAL(SimulationStop()), sys, SLOT(SimulationStop()) );
    QObject::connect( &w, SIGNAL(SimulationPause()), sys, SLOT(SimulationPause()) );
    QObject::connect( &w, SIGNAL(SimulationResume()), sys, SLOT(SimulationResume()) );
    QObject::connect( &w, SIGNAL(SetSpeedAdjustVal(int)), sys, SLOT(SetSpeedAdjustVal(int)) );
    QObject::connect( &w, SIGNAL(SetStopGraphicUpdate(bool)), sys, SLOT(SetStopGraphicUpdate(bool)) );

    QObject::connect( w.GetPointerGraphicCanvas(), SIGNAL(ShowAgentData(float,float)),sys, SLOT(ShowAgentData(float,float)) );
    QObject::connect( w.GetPointerGraphicCanvas(), SIGNAL(DSMove(float,float)),sys, SLOT(DSMove(float,float)) );

    QObject::connect( &w, SIGNAL(SetTmpStpoTime(int,int,int)), sys, SLOT(SetTmpStopTime(int,int,int)) );
    QObject::connect( &w, SIGNAL(OutputRestartData(QString)), sys, SLOT(OutputRestartData(QString)) );
    QObject::connect( &w, SIGNAL(SetRestartFile(QString)), sys, SLOT(SetRestartFile(QString)) );
    QObject::connect( &w, SIGNAL(SetDSMoveTarget(int)), sys, SLOT(SetDSMoveTarget(int)) );

    QObject::connect( sys, SIGNAL(UpdateSimulationTimeDisplay(QString)), &w, SLOT(UpdateSimulationTimeDisplay(QString)) );
    QObject::connect( sys, SIGNAL(SetAgentPointer(Agent**)), &w, SLOT(SetAgentPointerToCanvas(Agent**)) );
    QObject::connect( sys, SIGNAL(SetTrafficSignalList(QList<TrafficSignal*>)), &w, SLOT(SetTrafficSignalPointerToCanvas(QList<TrafficSignal*>)) );
    QObject::connect( sys, SIGNAL(SetMaxNumberAgent(int)), &w, SLOT(SetMaxNumberAgentToCanvas(int)) );
    QObject::connect( sys, SIGNAL(SetNumberTrafficSignal(int)), &w, SLOT(SetNumberTrafficSignalToCanvas(int)) );
    QObject::connect( sys, SIGNAL(RedrawRequest()), &w, SLOT(RedrawRequest()) );
    QObject::connect( sys, SIGNAL(SetRoadPointer(Road*)), &w, SLOT(SetRoadDataToCanvas(Road*)) );
    QObject::connect( sys, SIGNAL(TmpStopSimulation()), &w, SLOT(PauseSimulation()) );
    QObject::connect( sys, SIGNAL(ExitProgram()), &w, SLOT(ExitProgram()) );

    for(int i=0;i<maxWorkerThread;++i){
        QObject::connect( sys, SIGNAL(SetAgentPointer(Agent**)), workerThread[i], SLOT(SetAgentPointer(Agent**)) );
        QObject::connect( sys, SIGNAL(SetMaxNumberAgent(int)), workerThread[i], SLOT(SetMaxNumberAgent(int)) );
        QObject::connect( sys, SIGNAL(SetRoadPointer(Road*)), workerThread[i], SLOT(SetRoadPointer(Road*)) );
        QObject::connect( sys, SIGNAL(SetTrafficSignalPointer(TrafficSignal*)), workerThread[i], SLOT(SetTrafficSignalPointer(TrafficSignal*)) );
    }

    if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
        QTextStream sysLogOut(&sysLogOutFile);
        sysLogOut << "Connection created." << "\n";
        sysLogOut.flush();
        sysLogOutFile.close();
    }



    //
    //  Check arguments
    //
    qDebug() << "num arg = " << argc;
    if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
        QTextStream sysLogOut(&sysLogOutFile);
        sysLogOut << "Number arg = " << argc << "\n";
        sysLogOutFile.close();
    }

    for(int i=0;i<argc;++i){
        qDebug() << "[" << i << "]" << argv[i];

        if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
            QTextStream sysLogOut(&sysLogOutFile);
            sysLogOut << "[" << i << "]" << argv[i] << "\n";
            sysLogOut.flush();
            sysLogOutFile.close();
        }


        QString tmpStr = QString( argv[i] );
        if( tmpStr.startsWith("-WithArg") ){

            QString argFileName = QApplication::applicationDirPath() + QString("/Reisim.arg.txt");
            qDebug() << "argFileName = " << argFileName;

            if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
                QTextStream sysLogOut(&sysLogOutFile);
                sysLogOut << "argFileName = " << argFileName << "\n";
                sysLogOut.flush();
                sysLogOutFile.close();
            }


            QFile argFile( CheckNetworkDrive(argFileName) );
            if( argFile.open(QIODevice::ReadOnly | QIODevice::Text) == true ){

                QTextStream in(&argFile);

                QString NetLine = in.readLine();
                SysFilePath = NetLine.remove("-Net=").trimmed();

                qDebug() << "SysFilePath = " << SysFilePath;
                if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
                    QTextStream sysLogOut(&sysLogOutFile);
                    sysLogOut << "SysFilePath = " << SysFilePath << "\n";
                    sysLogOut.flush();
                    sysLogOutFile.close();
                }


                QString ConfLine = in.readLine();
                ConfFilePath = ConfLine.remove("-Conf=").trimmed();

                qDebug() << "ConfFilePath = " << ConfFilePath;
                if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
                    QTextStream sysLogOut(&sysLogOutFile);
                    sysLogOut << "ConfFilePath = " << ConfFilePath << "\n";
                    sysLogOut.flush();
                    sysLogOutFile.close();
                }


                argFile.close();
            }
            else{
                qDebug() << "Cannot open argFile";

                qDebug() << "ConfFilePath = " << ConfFilePath;
                if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
                    QTextStream sysLogOut(&sysLogOutFile);
                    sysLogOut << "Cannot open argFile." << "\n";
                    sysLogOut.flush();
                    sysLogOutFile.close();
                }

            }
        }
    }

    if( ConfFilePath.isNull() == false ){
        qDebug() << "Set Config File";

        if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
            QTextStream sysLogOut(&sysLogOutFile);
            sysLogOut << "Set Config File ... ";
            sysLogOut.flush();
            sysLogOutFile.close();
        }

        w.LoadSettingFile( ConfFilePath );

        if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
            QTextStream sysLogOut(&sysLogOutFile);
            sysLogOut << "done." << "\n";
            sysLogOut.flush();
            sysLogOutFile.close();
        }
    }

    if( SysFilePath.isNull() == false ){
        qDebug() << "Set Network File";

        if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
            QTextStream sysLogOut(&sysLogOutFile);
            sysLogOut << "Set Network File ... ";
            sysLogOut.flush();
            sysLogOutFile.close();
        }

        sys->SetSysFile( SysFilePath );

        if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
            QTextStream sysLogOut(&sysLogOutFile);
            sysLogOut << "done." << "\n";
            sysLogOut.flush();
            sysLogOutFile.close();
        }
    }

    qDebug() << "Enter main loop ...";
    qDebug() << "-----";

    if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
        QTextStream sysLogOut(&sysLogOutFile);
        sysLogOut << "+--- Enter main loop" << "\n";
        sysLogOut.flush();
        sysLogOutFile.close();
    }



    //
    //
    //
    int ret = a.exec();

    for(int i=0;i<maxWorkerThread;++i){
        workerThread[i]->stop();
        workerMutex->lock();
        workerCond[i]->wakeAll();
        workerMutex->unlock();
    }


    ReleaseNetworkDriveInfo();

    if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
        QTextStream sysLogOut(&sysLogOutFile);
        sysLogOut << "+--- End Re:sim" << "\n";
        sysLogOut.flush();
        sysLogOutFile.close();
    }

    return ret;
}




//
//  Following : for getting Processor infomation
//
typedef BOOL (WINAPI *LPFN_GLPI)(
    PSYSTEM_LOGICAL_PROCESSOR_INFORMATION,
    PDWORD);


// Helper function to count set bits in the processor mask.
DWORD CountSetBits(ULONG_PTR bitMask)
{
    DWORD LSHIFT = sizeof(ULONG_PTR)*8 - 1;
    DWORD bitSetCount = 0;
    ULONG_PTR bitTest = (ULONG_PTR)1 << LSHIFT;
    DWORD i;

    for (i = 0; i <= LSHIFT; ++i)
    {
        bitSetCount += ((bitMask & bitTest)?1:0);
        bitTest/=2;
    }

    return bitSetCount;
}

int get_logical_processor_info (int type)
{
    int ret = -1;

    LPFN_GLPI glpi;
    BOOL done = FALSE;
    PSYSTEM_LOGICAL_PROCESSOR_INFORMATION buffer = NULL;
    PSYSTEM_LOGICAL_PROCESSOR_INFORMATION ptr = NULL;
    DWORD returnLength = 0;
    DWORD logicalProcessorCount = 0;
    DWORD numaNodeCount = 0;
    DWORD processorCoreCount = 0;
    DWORD processorL1CacheCount = 0;
    DWORD processorL2CacheCount = 0;
    DWORD processorL3CacheCount = 0;
    DWORD processorPackageCount = 0;
    DWORD byteOffset = 0;
    PCACHE_DESCRIPTOR Cache;

    glpi = (LPFN_GLPI) GetProcAddress(
                            GetModuleHandle(TEXT("kernel32")),
                            "GetLogicalProcessorInformation");
    if (NULL == glpi)
    {
        qDebug() << "\nGetLogicalProcessorInformation is not supported.\n";
        return (1);
    }

    while (!done)
    {
        DWORD rc = glpi(buffer, &returnLength);

        if (FALSE == rc)
        {
            if (GetLastError() == ERROR_INSUFFICIENT_BUFFER)
            {
                if (buffer)
                    free(buffer);

                buffer = (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION)malloc(
                        returnLength);

                if (NULL == buffer)
                {
                    qDebug() << "\nError: Allocation failure\n";
                    return (2);
                }
            }
            else
            {
                qDebug() << "\nError << " << GetLastError();
                return (3);
            }
        }
        else
        {
            done = TRUE;
        }
    }

    ptr = buffer;

    while (byteOffset + sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION) <= returnLength)
    {
        switch (ptr->Relationship)
        {
        case RelationNumaNode:
            // Non-NUMA systems report a single record of this type.
            numaNodeCount++;
            break;

        case RelationProcessorCore:
            processorCoreCount++;

            // A hyperthreaded core supplies more than one logical processor.
            logicalProcessorCount += CountSetBits(ptr->ProcessorMask);
            break;

        case RelationCache:
            // Cache data is in ptr->Cache, one CACHE_DESCRIPTOR structure for each cache.
            Cache = &ptr->Cache;
            if (Cache->Level == 1)
            {
                processorL1CacheCount++;
            }
            else if (Cache->Level == 2)
            {
                processorL2CacheCount++;
            }
            else if (Cache->Level == 3)
            {
                processorL3CacheCount++;
            }
            break;

        case RelationProcessorPackage:
            // Logical processors share a physical package.
            processorPackageCount++;
            break;

        default:
            qDebug() << "\nError: Unsupported LOGICAL_PROCESSOR_RELATIONSHIP value.\n";
            break;
        }
        byteOffset += sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION);
        ptr++;
    }

    free(buffer);


    switch(type){
    case 1: ret = numaNodeCount; break;
    case 2: ret = processorPackageCount; break;
    case 3: ret = processorCoreCount; break;
    case 4: ret = logicalProcessorCount; break;
    case 5: ret = processorL1CacheCount; break;
    case 6: ret = processorL2CacheCount; break;
    case 7: ret = processorL3CacheCount; break;
    default: ret = -1; break;
    }


    return ret;
}
