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


#include "systemthread.h"
#include <QDebug>
#include <QTimer>

#include <QMutex>
#include <QWaitCondition>

#include "networkdrivecheck.h"

#include <windows.h>

extern QMutex *mutex;
extern QWaitCondition *cond;

int g_DSTimingFlag = 0;

extern QMutex *mutex_sync;
extern QWaitCondition *cond_sync;

int allowDataGetForDS = 0;

extern QMutex *mutex_log;
extern QWaitCondition *cond_log;

extern int logThreadOpeFlag;



SystemThread::SystemThread(QObject *parent) :
    QThread(parent)
{
    stopped   = false;
    DSMode    = false;
    sysFile   = QString();
    confFile  = QString();
    udpThread = NULL;

    road = new Road();
    if( !road ){
        qDebug() << "[SystemThread::SystemThread] cannot allocate road instance.";
    }

    simManage = new SimulationManager();
    if( !simManage ){
        qDebug() << "[SystemThread::SystemThread] cannot allocate simManage instance.";
    }

    logThread = new LogOutputThread();
    if( !logThread ){
        qDebug() << "[SystemThread::SystemThread] cannot allocate logThread instance.";
    }

    agent = NULL;

    scenarioFile = QString();
    simulationState = SIMULATION_STATE::INITIATED;
    maxAgent = 100;
    numTrafficSignals = 0;

    logFileName = QString();
    logOutputFolder = QString();
    logOutputInterval = 1;

    speedAdjustVal = 50;
    stopGraphicUpdate = false;

    signalDataFile  = QString();
}

void SystemThread::Stop()
{
    stopped = true;
}

void SystemThread::run()
{
    qDebug() << "[SystemThread]Start thread";

    int redrawIntervalCount = 0;

    // check number of traffic signals
    if( DSMode == true ){
        if( udpThread->getNumberTrafficSignal() != trafficSignal.size() ){
            qDebug() << "!!!! Number of Traffic Signal mismatch:";
            qDebug() << "   in systemThread: N = " << trafficSignal.size();
            qDebug() << "   in udpThread: N = " << udpThread->getNumberTrafficSignal();
            qDebug() << " set N in systemThread.";
            udpThread->SetNumberTrafficSignal( trafficSignal.size() );
        }
    }


    if( logFileName.isNull() == false ){

        QString filename = logOutputFolder;
        if( filename.endsWith("/") == false ){
            filename += QString("/");
        }
        filename += logFileName;

        // First set reference to log thread
        logThread->SetDataReference( simManage, agent, maxAgent, trafficSignal, trafficSignal.size() );
        qDebug() << "SetDataReference";

        // Second open file, and output title line
        logThread->SetLogFileName( filename );
        qDebug() << "SetLogFileName: " << filename;


        logThread->SetLogOutputInterval( logOutputInterval );
        qDebug() << "SetLogOutputInterval";

        logThread->start();
    }

    for(int i=0;i<trafficSignal.size();++i){
        trafficSignal[i]->SetSignalStartTime( trafficSignal[i]->startOffset );
    }

    g_DSTimingFlag = 0;

    emit RedrawRequest();


#ifdef _PERFORMANCE_CHECK
    LARGE_INTEGER start, end;

    LARGE_INTEGER freq;
    QueryPerformanceFrequency(&freq);

    double calTime[10];
    int calCount[10];
    for(int i=0;i<10;++i){
        calTime[i] = 0.0;
        calCount[i] = 0;
    }
#endif


    stopped = false;
    while( stopped == false ){

        if( simulationState == SIMULATION_STATE::PAUSED ){
            continue;
        }

        //
        // if DS mode, synchronize execution
        if( DSMode == true ){

            mutex->lock();
            if( g_DSTimingFlag == 0 ){
                cond->wait(mutex);
            }
            g_DSTimingFlag = 0;
            mutex->unlock();

        }
        else{

            if( speedAdjustVal <= 50 ){
                int w = (51 - speedAdjustVal) * 2;
                msleep( w );
            }
            else{
                if( speedAdjustVal < 60 ){
                    int w = 60 - speedAdjustVal;
                    msleep( w );
                }
            }
        }



        //
        // Get current time to display
        QString simTime = simManage->GetSimulationTimeStr();
        float simTimeFVal = simManage->GetSimulationTimeInSec();

        //qDebug() << simTime;

        emit UpdateSimulationTimeDisplay(simTime);



        //
        // Generate agents
#ifdef _PERFORMANCE_CHECK
        QueryPerformanceCounter(&start);
#endif

        simManage->AppearAgents( agent, maxAgent, road );

#ifdef _PERFORMANCE_CHECK
        QueryPerformanceCounter(&end);
        calTime[0] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
        calCount[0]++;
#endif

        //
        // Raise Event
        simManage->RaiseEvent( agent, maxAgent, road );



        //
        // Update Traffic Signal Display
        //
        for(int i=0;i<numTrafficSignals;++i){
            trafficSignal[i]->CheckDisplayInfoUpdate( simTimeFVal );
        }



        //
        // Control
        for(int i=0;i<maxAgent;++i){
            if( agent[i]->agentStatus == 0 ){
                continue;
            }
            if( agent[i]->isSInterfaceObject == true ){
                continue;
            }
            if( agent[i]->isBehaviorEmbeded == true ){
                continue;
            }

#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&start);
#endif

            agent[i]->Perception( agent, maxAgent, road, trafficSignal );

#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&end);
            calTime[1] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            calCount[1]++;
#endif

#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&start);
#endif

            agent[i]->Recognition( agent, maxAgent, road );

#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&end);
            calTime[2] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            calCount[2]++;
#endif

#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&start);
#endif

            agent[i]->HazardIdentification( agent, maxAgent, road );

#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&end);
            calTime[3] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            calCount[3]++;
#endif

#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&start);
#endif

            agent[i]->RiskEvaluation( agent, maxAgent, road );

#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&end);
            calTime[4] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            calCount[4]++;
#endif

#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&start);
#endif

            agent[i]->Control( road );

#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&end);
            calTime[5] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            calCount[5]++;
#endif


#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&start);
#endif

            agent[i]->UpdateState();

#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&end);
            calTime[6] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            calCount[6]++;
#endif

        }


        //
        // Update simulation time
        simManage->UpdateSimulationTime();



        //
        // Graphic
        if( stopGraphicUpdate == false ){

            if( speedAdjustVal < 60 ){
                emit RedrawRequest();
                redrawIntervalCount = 0;
            }
            else{
                if( speedAdjustVal < 200 ){
                    redrawIntervalCount++;
                    int count = speedAdjustVal - 60;
                    if( redrawIntervalCount >= count ){
                        emit RedrawRequest();
                        redrawIntervalCount = 0;
                    }
                }
            }
        }


        //
        // Log File Output
        mutex_log->lock();
        logThreadOpeFlag = 1;
        cond_log->wakeAll();
        mutex_log->unlock();



        //
        // Check vehicles that reach to end of route
        for(int i=0;i<maxAgent;++i){
            if( agent[i]->agentStatus == 0 ){
                continue;
            }
            if( agent[i]->isSInterfaceObject == true ){
                continue;
            }
            if( agent[i]->isBehaviorEmbeded == true ){
                continue;
            }

#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&start);
#endif

            agent[i]->CheckPathList( road );

#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&end);
            calTime[7] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            calCount[7]++;
#endif
        }


#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&start);
#endif

        simManage->DisappearAgents( agent, maxAgent);

#ifdef _PERFORMANCE_CHECK
        QueryPerformanceCounter(&end);
        calTime[8] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
        calCount[8]++;
#endif



#ifdef _PERFORMANCE_CHECK
        if( calCount[0] >= 50 ){
            for(int i=0;i<9;i++){
                calTime[i] /= calCount[i];
                qDebug() << "Mean Time[" << i << "] = " << calTime[i];
                calCount[i] = 0;
            }
        }
#endif

    }

    logThread->SetStopFlag();
    logThread->CloseFile();

    qDebug() << "[SystemThread]Now leave thread";
}


void SystemThread::SetSysFile(QString filename)
{
    sysFile = filename;
    DSMode = true;

    //
    //  Prepare UDP Sockets
    if( udpThread == NULL ){
        udpThread = new UDPThread();
        connect( udpThread, SIGNAL(SimulationStart()), this, SLOT(SimulationStart()) );
        connect( udpThread, SIGNAL(SimulationStop()), this, SLOT(SimulationStop()) );
        connect( udpThread, SIGNAL(ExitProgram()), this, SLOT(wrapExitProgram()));
        connect( udpThread, SIGNAL(RequestSetSendData(char*,int,int *)), this, SLOT(SetSendData(char*,int,int *)) );
        connect( udpThread, SIGNAL(ReceiveContinueCommand()), this, SLOT(quit()) );
        connect( udpThread, SIGNAL(SetSimulationFrequency(int)), this, SLOT(SetSimulationFrequency(int)) );
        connect( udpThread, SIGNAL(ReceiveTireHeight(int,float,float,float,float)), this, SLOT(SetTireHeight(int,float,float,float,float)) );
        connect( udpThread, SIGNAL(ReceiveSInterObjData(char,int,AgentState*,struct SInterObjInfo *)), this, SLOT(SetSInterObjData(char,int,AgentState*,struct SInterObjInfo *)) );
        connect( udpThread, SIGNAL(ReceiveTSColorChange(int,int,float)),this,SLOT(ForceChangeTSColor(int,int,float)));
        connect( udpThread, SIGNAL(WarpVehicle(int,float,float,float)),this,SLOT(WarpVehicle(int,float,float,float)) );
        connect( udpThread, SIGNAL(DisposeAgent(int)),this,SLOT(DisposeAgent(int)) );
        connect( udpThread, SIGNAL(AppearAgent(int)),this,SLOT(AppearAgent(int)) );
        connect( udpThread, SIGNAL(EmbedBehavior(int,float*,int)),this,SLOT(EmbedAgentBehavior(int,float*,int)) );
        connect( udpThread, SIGNAL(ChangeReferenceSpeed(int,float)),this,SLOT(ChangeReferenceSpeed(int,float)) );
        connect( udpThread, SIGNAL(CopyPathData(int,int)),this,SLOT(CopyPathData(int,int)) );
        connect( udpThread, SIGNAL(ChangeControlModeStopAt(int,float,float)),this,SLOT(ChangeControlModeStopAt(int,float,float)) );
        connect( udpThread, SIGNAL(ChangeControlModeHeadwayControl(int,float,float,float,float,int)),this,SLOT(ChangeControlModeHeadwayControl(int,float,float,float,float,int)) );
        connect( udpThread, SIGNAL(ChangeControlModeAgent(int)),this,SLOT(ChangeControlModeAgent(int)) );
        connect( udpThread, SIGNAL(ChangeControlModeIntersectionTurn(int,float,float)),this,SLOT(ChangeControlModeIntersectionTurn(int,float,float)) );
        connect( udpThread, SIGNAL(CopyScenarioData(int,int)),this,SLOT(CopyScenarioData(int,int)) );
        connect( udpThread, SIGNAL(ChangeVehicleWinkers(int,int)),this,SLOT(ChangeVehicleWinkers(int,int)) );
        connect( udpThread, SIGNAL(SetLateralShift(int,float)),this,SLOT(SetLateralShift(int,float)) );
        connect( udpThread, SIGNAL(SetLateralGainMultiplier(int,float)),this,SLOT(SetLateralGainMultiplier(int,float)) );
        connect( udpThread, SIGNAL(SetAgentGenerationNotAllowFlag(int,bool)),this,SLOT(SetAgentGenerationNotAllowFlag(int,bool)) );
        connect( udpThread, SIGNAL(ForceChangeSpeed(int,float)),this,SLOT(ForceChangeSpeed(int,float)) );
        connect( this, SIGNAL(SetMaxNumberAgent(int)), udpThread, SLOT(SetMaxAgentNumber(int)) );
        connect( this, SIGNAL(SetNumberTrafficSignal(int)), udpThread, SLOT(SetNumberTrafficSignal(int)) );
        if( logThread ){
            connect( udpThread, SIGNAL(FEDataReceived(int,QString)), logThread, SLOT(SetFuncExtenderLogData(int,QString)) );
        }
    }
    else{
        udpThread->destroySocks();
    }

    udpThread->loadSysFile( sysFile );
}


void SystemThread::wrapExitProgram()
{
    emit ExitProgram();

    udpThread->Stop();

    this->exit();
}

void SystemThread::SetDSmode()
{
    qDebug() << "!!! SystemThread::SetDSmode : DSMode = true";

    DSMode = true;
    if( simManage ){
        simManage->SetDSMode();
    }
    else{
        qDebug() << "[FATAL] simManage not allocated. Failed to set DS Mode.";
    }
}


void SystemThread::SetStartTrigger(int t)
{
    startTrigger = t;
}



void SystemThread::SetScenarioFile(QString filename)
{
    qDebug() << "[SystemThread::SetScenarioFile] scenarioFile = " << filename;

    scenarioFile = filename;
    LoadScenarioFile();
}


void SystemThread::LoadScenarioFile()
{
    qDebug() << "[SystemThread::LoadScenarioFile] scenarioFile = " << scenarioFile;

    if( scenarioFile == QString() ){
        return;
    }

    QFile file( CheckNetworkDrive(scenarioFile) );
    if( file.open(QIODevice::ReadOnly | QIODevice::Text) == false ){
        QMessageBox::warning(NULL,QString("Error"),QString("Cannot open scenario file."));
        return;
    }

    QString line;
    QStringList divLine;

    QTextStream in(&file);

    line = in.readLine();
    line = in.readLine();
    if( line.contains("Re:sim Scenario File") == false ){
        file.close();
        QMessageBox::warning(NULL,QString("Error"),QString("This is not Re:sim Scenario File."));
        return;
    }
    line = in.readLine();


    if( agent != NULL ){
        for(int i=0;i<maxAgent;++i){
            delete agent[i];
        }
        delete [] agent;

        maxAgent = 100;
    }


    simManage->ClearScenarioData();


    int currentScenarioID = 0;
    struct ScenarioTriggers* trigger = NULL;
    struct ObjectTriggerData* trigObj = NULL;
    struct ScenarioObjectControlInfo* controlInfo = NULL;
    struct ScenarioItem* scenarioItem = NULL;
    struct ScenarioEvents* scenarioEvent = NULL;

    int eventNo = 0;

    while( in.atEnd() == false ){
        line = in.readLine();
        if( line.startsWith("#") || line.isEmpty() || line.contains(";") == false ){
            continue;
        }

        divLine = line.split(";");

        QString tag = QString( divLine[0] ).trimmed();
        if( tag == QString("Road Data File") ){
            roadDataFile = QString( divLine[1] ).trimmed();
        }
        else if( tag == QString("Signal Data File") ){
            signalDataFile = QString( divLine[1] ).trimmed();
        }
        else if( tag == QString("Max Number of Agent") ){
            maxAgent = QString( divLine[1] ).trimmed().toInt();
        }
        else if( tag == QString("Scenario ID") ){
            currentScenarioID = QString( divLine[1] ).trimmed().toInt();
            simManage->AllocateScenario( currentScenarioID );
        }
        else if( tag == QString("End Condition") ){
            trigger = new struct ScenarioTriggers;
            trigger->mode = QString( divLine[1] ).trimmed().toInt();
            trigger->byExternalTriggerFlag = 0;
        }
        else if( tag == QString("End Target") ){
            trigObj = new struct ObjectTriggerData;
            trigObj->targetObjectID = QString( divLine[1] ).trimmed().toInt();
        }
        else if( tag == QString("End Trigger Position") ){
            if( divLine.size() >= 5 ){
                trigObj->x         = QString( divLine[1] ).trimmed().toFloat();
                trigObj->y         = QString( divLine[2] ).trimmed().toFloat();
                trigObj->direction = QString( divLine[3] ).trimmed().toFloat() * 0.017452;
                trigObj->speed     = QString( divLine[4] ).trimmed().toFloat() / 3.6;
                trigObj->cosDirect = cos( trigObj->direction );
                trigObj->sinDirect = sin( trigObj->direction );
                trigger->objectTigger.append( trigObj );
            }
        }
        else if( tag == QString("End Time") ){
            if( divLine.size() >= 7 ){
                struct SimulationTime *t = new struct SimulationTime;
                t->day    = QString(divLine[3]).trimmed().toInt();
                t->hour   = QString(divLine[4]).trimmed().toInt();
                t->min    = QString(divLine[5]).trimmed().toInt();
                t->msec   = QString(divLine[6]).trimmed().toFloat();

                t->sec = (int)(t->msec);
                t->msec -= t->sec;
                trigger->timeTrigger = t;
                trigger->timeTriggerInSec = t->msec + t->sec + t->min * 60.0 + t->hour * 3600.0 + t->day * 86400.0;
            }

            simManage->SetScenarioEndTrigger( currentScenarioID, trigger );
        }
        else if( tag == QString("Pedestrian ID") ){
            scenarioItem = new struct ScenarioItem;
            scenarioItem->type = 'p';
            scenarioItem->objectID = QString( divLine[1] ).trimmed().toInt();
            scenarioItem->appearTriggers = NULL;
            scenarioItem->disappearTriggers = NULL;
            scenarioItem->repeat = false;
            scenarioItem->status = 0;

            qDebug() << "[Tag:Pedestrian ID] objectID = " << scenarioItem->objectID;
        }
        else if( tag == QString("Pedestrian Model ID") ){
            scenarioItem->objectModelID = QString( divLine[1] ).trimmed().toInt();

        }
        else if( tag == QString("Pedestrian Initital State") ){
            if( divLine.size() >= 6 ){
                scenarioItem->initialState.append( QString(divLine[1]).trimmed().toFloat() );  // X
                scenarioItem->initialState.append( QString(divLine[2]).trimmed().toFloat() );  // Y
                scenarioItem->initialState.append( QString(divLine[3]).trimmed().toFloat() );  // Z
                scenarioItem->initialState.append( QString(divLine[4]).trimmed().toFloat() * 0.017452 );  // YawAngle [rad]
                scenarioItem->initialState.append( QString(divLine[5]).trimmed().toFloat() );  // Speed [m/s]
            }
        }
        else if( tag == QString("Pedestrian Appearance Repeat") ){
            scenarioItem->repeat = (bool)QString( divLine[1] ).trimmed().toInt();
        }
        else if( tag == QString("Pedestrian Trigger Mode") ){
            trigger = new struct ScenarioTriggers;
            trigger->mode = QString( divLine[1] ).trimmed().toInt();
            trigger->ANDCondition = 0;
            trigger->func_keys = 0;
            trigger->byExternalTriggerFlag = 0;
        }
        else if( tag == QString("Pedestrian Trigger Function Key") ){
            trigger->func_keys = QString( divLine[1] ).trimmed().toInt();
        }
        else if( tag == QString("Pedestrian Trigger AND Condition") ){
            trigger->ANDCondition = QString( divLine[1] ).trimmed().toInt();
        }
        else if( tag == QString("Pedestrian Trigger Target") ){
            trigObj = new struct ObjectTriggerData;
            trigObj->targetObjectID = QString( divLine[1] ).trimmed().toInt();
        }
        else if( tag == QString("Pedestrian Trigger Position") ){
            if( divLine.size() >= 4 ){
                trigObj->x = QString(divLine[1]).trimmed().toFloat();
                trigObj->y = QString(divLine[2]).trimmed().toFloat();
                trigObj->direction = QString(divLine[3]).trimmed().toFloat() * 0.017452;  // [rad]
                trigObj->cosDirect = cos( trigObj->direction );
                trigObj->sinDirect = sin( trigObj->direction );
            }
        }
        else if( tag == QString("Pedestrian Trigger State") ){
            if( divLine.size() >= 3 ){
                trigObj->speed = QString(divLine[1]).trimmed().toFloat() / 3.6;  // [m/s]
                trigObj->TTC   = QString(divLine[2]).trimmed().toFloat();        // [sec]
            }
            trigger->objectTigger.append( trigObj );
        }
        else if( tag == QString("Pedestrian Trigger Time") ){
            if( divLine.size() >= 7 ){
                struct SimulationTime *t = new struct SimulationTime;
                t->day    = QString(divLine[3]).trimmed().toInt();
                t->hour   = QString(divLine[4]).trimmed().toInt();
                t->min    = QString(divLine[5]).trimmed().toInt();
                t->msec   = QString(divLine[6]).trimmed().toFloat();

                t->sec = (int)(t->msec);
                t->msec -= t->sec;
                trigger->timeTrigger = t;
                trigger->timeTriggerInSec = t->msec + t->sec + t->min * 60.0 + t->hour * 3600.0 + t->day * 86400.0;
            }
            scenarioItem->appearTriggers = trigger;
        }
        else if( tag == QString("Pedestrian Control Mode") ){

            controlInfo = new struct ScenarioObjectControlInfo;
            controlInfo->mode = QString(divLine[1]).trimmed().toInt();
            controlInfo->targetObjectID = -1;
            controlInfo->targetHeadwayDistance = 0.0;
            controlInfo->targetHeadwayTime     = 0.0;
            controlInfo->targetLateralOffset   = 0.0;
            controlInfo->stopAtX               = 0.0;
            controlInfo->stopAtY               = 0.0;

            scenarioItem->controlInfo = controlInfo;

            simManage->SetScenarioItem( currentScenarioID, scenarioItem );

            qDebug() << "SetScenraioItem: item size = " << simManage->GetNumberScenarioObject(currentScenarioID);
        }
        else if( tag == QString("Pedestrian Control Ref") ){
            if( divLine.size() >= 3 ){
                controlInfo->targetSpeed = QString(divLine[1]).trimmed().toFloat();   // speed in scenario file for pedestrian is given by [m/s]
                controlInfo->targetHeadwayDistance = QString(divLine[2]).trimmed().toFloat();
            }
        }
        else if( tag == QString("Pedestrian Control Profile X") ){
            for(int i=1;i<divLine.size();++i){
                controlInfo->profileAxis.append( QString(divLine[i]).trimmed().toFloat() );
            }
        }
        else if( tag == QString("Pedestrian Control Profile V") ){
            for(int i=1;i<divLine.size();++i){
                controlInfo->speedProfile.append( QString(divLine[i]).trimmed().toFloat() / 3.6 );
            }
        }
        else if( tag == QString("Pedestrian Route Type") ){
            controlInfo->routeType = QString(divLine[1]).trimmed().toInt();
        }
        else if( tag == QString("Pedestrian Route Elem") ){
            if( divLine.size() >= 10 ){

                struct ScenarioPedestPathRoute *ppElem = new ScenarioPedestPathRoute;

                ppElem->isCrossWalk = false;
                if( QString(divLine[8]).trimmed() == 1 ){
                    ppElem->isCrossWalk = true;
                }

                ppElem->x1 = QString(divLine[1]).trimmed().toFloat();
                ppElem->y1 = QString(divLine[2]).trimmed().toFloat();
                ppElem->z1 = QString(divLine[3]).trimmed().toFloat();

                ppElem->x2 = QString(divLine[4]).trimmed().toFloat();
                ppElem->y2 = QString(divLine[5]).trimmed().toFloat();
                ppElem->z2 = QString(divLine[6]).trimmed().toFloat();

                ppElem->width = QString(divLine[7]).trimmed().toFloat();
                ppElem->roadSideInfo = QString(divLine[9]).trimmed().toInt();

                controlInfo->ppRouteElem.append( ppElem );
            }
        }
        else if( tag == QString("Pedestrian Route List") ){
            for(int i=1;i<divLine.size();++i){
                controlInfo->pedestPathRoute.append( QString(divLine[i]).trimmed().toInt() );
            }
        }
        else if( tag == QString("Vehicle ID") ){
            scenarioItem = new struct ScenarioItem;
            scenarioItem->type = 'v';
            scenarioItem->objectID = QString( divLine[1] ).trimmed().toInt();
            scenarioItem->appearTriggers = NULL;
            scenarioItem->disappearTriggers = NULL;
            scenarioItem->repeat = false;
            scenarioItem->status = 0;
        }
        else if( tag == QString("Vehicle Model ID") ){
            scenarioItem->objectModelID = QString( divLine[1] ).trimmed().toInt();
        }
        else if( tag == QString("Vehicle Initital State") ){
            if( divLine.size() >= 6 ){
                scenarioItem->initialState.append( QString(divLine[1]).trimmed().toFloat() );  // X
                scenarioItem->initialState.append( QString(divLine[2]).trimmed().toFloat() );  // Y
                scenarioItem->initialState.append( QString(divLine[3]).trimmed().toFloat() );  // Z
                scenarioItem->initialState.append( QString(divLine[4]).trimmed().toFloat() * 0.017452 );  // YawAngle [rad]
                scenarioItem->initialState.append( QString(divLine[5]).trimmed().toFloat() / 3.6 );  // Speed [m/s]
            }
        }
        else if( tag == QString("Vehcile Appearance Repeat") ){
            scenarioItem->repeat = (bool)QString( divLine[1] ).trimmed().toInt();
        }
        else if( tag == QString("Vehicle Trigger Mode") ){
            trigger = new struct ScenarioTriggers;
            trigger->mode = QString( divLine[1] ).trimmed().toInt();
            trigger->ANDCondition = 0;
            trigger->func_keys = 0;
            trigger->byExternalTriggerFlag = 0;
        }
        else if( tag == QString("Vehicle Trigger Function Key") ){
            trigger->func_keys = QString( divLine[1] ).trimmed().toInt();
        }
        else if( tag == QString("Vehicle Trigger AND Condition") ){
            trigger->ANDCondition = QString( divLine[1] ).trimmed().toInt();
        }
        else if( tag == QString("Vehicle Trigger Target") ){
            trigObj = new struct ObjectTriggerData;
            trigObj->targetObjectID = QString( divLine[1] ).trimmed().toInt();
        }
        else if( tag == QString("Vehicle Trigger Position") ){
            if( divLine.size() >= 4 ){
                trigObj->x = QString(divLine[1]).trimmed().toFloat();
                trigObj->y = QString(divLine[2]).trimmed().toFloat();
                trigObj->direction = QString(divLine[3]).trimmed().toFloat() * 0.017452;  // [rad]
                trigObj->cosDirect = cos( trigObj->direction );
                trigObj->sinDirect = sin( trigObj->direction );
            }
        }
        else if( tag == QString("Vehicle Trigger State") ){
            if( divLine.size() >= 3 ){
                trigObj->speed = QString(divLine[1]).trimmed().toFloat() / 3.6;  // [m/s]
                trigObj->TTC   = QString(divLine[2]).trimmed().toFloat();        // [sec]
            }
            trigger->objectTigger.append( trigObj );
        }
        else if( tag == QString("Vehicle Trigger Time") ){
            if( divLine.size() >= 7 ){
                struct SimulationTime *t = new struct SimulationTime;
                t->day    = QString(divLine[3]).trimmed().toInt();
                t->hour   = QString(divLine[4]).trimmed().toInt();
                t->min    = QString(divLine[5]).trimmed().toInt();
                t->msec   = QString(divLine[6]).trimmed().toFloat();

                t->sec = (int)(t->msec);
                t->msec -= t->sec;
                trigger->timeTrigger = t;
                trigger->timeTriggerInSec = t->msec + t->sec + t->min * 60.0 + t->hour * 3600.0 + t->day * 86400.0;
            }
            scenarioItem->appearTriggers = trigger;
        }
        else if( tag == QString("Vehicle Control Mode") ){

            controlInfo = new struct ScenarioObjectControlInfo;
            controlInfo->mode = QString(divLine[1]).trimmed().toInt();
            controlInfo->targetObjectID = -1;
            controlInfo->targetHeadwayDistance = 0.0;
            controlInfo->targetHeadwayTime     = 0.0;
            controlInfo->targetLateralOffset   = 0.0;
            controlInfo->stopAtX               = 0.0;
            controlInfo->stopAtY               = 0.0;

            scenarioItem->controlInfo = controlInfo;

            simManage->SetScenarioItem( currentScenarioID, scenarioItem );
        }
        else if( tag == QString("Vehicle Control Ref") ){
            if( divLine.size() >= 3 ){
                controlInfo->targetSpeed = QString(divLine[1]).trimmed().toFloat() / 3.6;
                controlInfo->targetHeadwayDistance = QString(divLine[2]).trimmed().toFloat();
            }
        }
        else if( tag == QString("Vehicle Control HeadwayTime") ){
            controlInfo->targetHeadwayTime = QString(divLine[1]).trimmed().toFloat();
        }
        else if( tag == QString("Vehicle Control LateralOffset") ){
            controlInfo->targetLateralOffset = QString(divLine[1]).trimmed().toFloat();
        }
        else if( tag == QString("Vehicle Control Stop Position") ){
            controlInfo->stopAtX = QString(divLine[1]).trimmed().toFloat();
            controlInfo->stopAtY = QString(divLine[2]).trimmed().toFloat();
        }
        else if( tag == QString("Vehicle Control Control Target ID") ){
            controlInfo->targetObjectID = QString(divLine[1]).trimmed().toInt();
        }
        else if( tag == QString("Vehicle Control Profile X") ){
            for(int i=1;i<divLine.size();++i){
                controlInfo->profileAxis.append( QString(divLine[i]).trimmed().toFloat() );
            }
        }
        else if( tag == QString("Vehicle Control Profile V") ){
            for(int i=1;i<divLine.size();++i){
                controlInfo->speedProfile.append( QString(divLine[i]).trimmed().toFloat() / 3.6 );
            }
        }
        else if( tag == QString("Vehicle Route Type") ){
            controlInfo->routeType = QString(divLine[1]).trimmed().toInt();
        }
        else if( tag == QString("Vehicle WP Route Elem") ){
            if( divLine.size() >= 6 ){

                struct ScenarioWPRoute* wpr = new struct ScenarioWPRoute;

                wpr->x = QString(divLine[1]).trimmed().toFloat();
                wpr->y = QString(divLine[2]).trimmed().toFloat();
                wpr->z = QString(divLine[3]).trimmed().toFloat();
                wpr->direct    = QString(divLine[4]).trimmed().toFloat() * 0.017452;
                wpr->speedInfo = QString(divLine[5]).trimmed().toFloat() / 3.6;

                controlInfo->wpRoute.append( wpr );
            }
        }
        else if( tag == QString("Vehicle Node Route Elem") ){
            if( divLine.size() >= 4 ){

                struct ScenarioNodeRoute* nr = new struct ScenarioNodeRoute;

                nr->node      = QString(divLine[1]).trimmed().toInt();
                nr->inDirect  = QString(divLine[2]).trimmed().toInt();
                nr->outDirect = QString(divLine[3]).trimmed().toInt();

                controlInfo->nodeRoute.append( nr );
            }
        }
        else if( tag == QString("Vehicle Event Type") ){
            scenarioEvent = new ScenarioEvents;

            scenarioEvent->eventID = eventNo;
            eventNo++;

            scenarioEvent->eventType = SCENARIO_EVENT_TYPE::OBJECT_EVENT;
            scenarioEvent->eventState = 0;
            scenarioEvent->targetObjectID = scenarioItem->objectID;

            scenarioEvent->eventKind = QString(divLine[1]).trimmed().toInt();

            trigger = new ScenarioTriggers;
            trigger->mode = 0;
            scenarioEvent->eventTrigger = trigger;

            simManage->SetScenarioEvent( currentScenarioID, scenarioEvent );
        }
        else if( tag == QString("Vehicle Event Trigger External") ){
            int extTrig = QString(divLine[1]).trimmed().toInt();
            if( extTrig >= 1 ){
                trigger->mode = TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY;
                trigger->func_keys = extTrig;
            }
        }
        else if( tag == QString("Vehicle Event Trigger Internal") ){
            if( trigger->mode != TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
                trigger->mode = QString(divLine[1]).trimmed().toInt();
            }
            trigger->byExternalTriggerFlag = 0;
        }
        else if( tag == QString("Vehicle Event Trigger Position") ){
            trigObj = new struct ObjectTriggerData;
            trigObj->targetObjectID = -1;
            trigger->objectTigger.append( trigObj );
            if( divLine.size() >= 5 ){
                trigObj->x         = QString(divLine[1]).trimmed().toFloat();
                trigObj->y         = QString(divLine[2]).trimmed().toFloat();
                trigObj->direction = QString(divLine[4]).trimmed().toFloat() * 0.017452;
                trigObj->cosDirect = cos( trigObj->direction );
                trigObj->sinDirect = sin( trigObj->direction );
            }
        }
        else if( tag == QString("Vehicle Event Trigger Speed") ){
            trigObj->speed = QString(divLine[1]).trimmed().toFloat() / 3.6;
        }
        else if( tag == QString("Vehicle Event Trigger T2TP") ){
            trigObj->TTC = QString(divLine[1]).trimmed().toFloat();
        }
        else if( tag == QString("Vehicle Event Target ID") ){
            trigObj->targetObjectID = QString(divLine[1]).trimmed().toInt();
        }
        else if( tag == QString("Vehicle Event Trigger Time") ){

            if( divLine.size() >= 4 ){

                int etmode = QString(divLine[1]).trimmed().toInt();
                float val1 = QString(divLine[2]).trimmed().toFloat();
                float val2 = QString(divLine[3]).trimmed().toFloat();

                if( etmode == 1 ){
                    float r = simManage->GenUniform();
                    val1 = val1 + r * (val2 - val1);
                }
                else if( etmode == 2 ){
                    val1 = simManage->GetNormalDist(val1, val2);
                }

                if( val1 < 0.0 ){
                    val1 = 0.0;
                }

                struct SimulationTime *t = new struct SimulationTime;

                t->day    = 0;
                t->hour   = val1 / 3600;
                t->min    = val1 / 60 - t->hour * 3600;
                t->msec   = val1 - t->min * 60 - t->hour * 3600;

                t->sec = (int)(t->msec);
                t->msec -= t->sec;

                trigger->timeTrigger = t;
                trigger->timeTriggerInSec = t->msec + t->sec + t->min * 60.0 + t->hour * 3600.0 + t->day * 86400.0;
            }
        }
        else if( tag == QString("Vehicle Event Params") ){
            for(int i=1;i<divLine.size();++i){
                scenarioEvent->eventFloatData.append( QString(divLine[i]).trimmed().toFloat() );
            }
        }
        else if( tag == QString("Scenario Event ID") ){

            scenarioEvent = new ScenarioEvents;

            scenarioEvent->eventID = eventNo;
            eventNo++;

            scenarioEvent->eventType = SCENARIO_EVENT_TYPE::SYSTEM_EVENT;
            scenarioEvent->eventState = 0;
            scenarioEvent->targetObjectID = -1;

            trigger = new ScenarioTriggers;
            trigger->mode = 0;
            scenarioEvent->eventTrigger = trigger;

            simManage->SetScenarioEvent( currentScenarioID, scenarioEvent );
        }
        else if( tag == QString("Scenario Event Type") ){
            scenarioEvent->eventKind = QString(divLine[1]).trimmed().toInt();
        }
        else if( tag == QString("Scenario Event Trigger External") ){
            int extTrig = QString(divLine[1]).trimmed().toInt();
            if( extTrig >= 1 ){
                trigger->mode = TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY;
                trigger->func_keys = extTrig;
            }
        }
        else if( tag == QString("Scenario Event Trigger Internal") ){
            if( trigger->mode != TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
                trigger->mode = QString(divLine[1]).trimmed().toInt();
            }
            trigger->byExternalTriggerFlag = 0;
        }
        else if( tag == QString("Scenario Event Trigger Position") ){
            trigObj = new struct ObjectTriggerData;
            trigObj->targetObjectID = -1;
            trigObj->targetEventID  = -1;
            trigger->objectTigger.append( trigObj );
            if( divLine.size() >= 4 ){
                trigObj->x         = QString(divLine[1]).trimmed().toFloat();
                trigObj->y         = QString(divLine[1]).trimmed().toFloat();
                trigObj->direction = QString(divLine[1]).trimmed().toFloat() * 0.017452;
                trigObj->cosDirect = cos( trigObj->direction );
                trigObj->sinDirect = sin( trigObj->direction );
            }
        }
        else if( tag == QString("Scenario Event Trigger Speed ") ){
            trigObj->speed = QString(divLine[1]).trimmed().toFloat() / 3.6;
        }
        else if( tag == QString("Scenario Event TTC") ){
            trigObj->TTC = QString(divLine[1]).trimmed().toFloat();
        }
        else if( tag == QString("Scenario Event Target Object ID") ){
            trigObj->targetObjectID = QString(divLine[1]).trimmed().toInt();
        }
        else if( tag == QString("Scenario Event Related Event ID") ){
            trigObj->targetEventID = QString(divLine[1]).trimmed().toInt();
        }
        else if( tag == QString("Scenario Event Trigger Time") ){

            if( divLine.size() >= 4 ){

                int etmode = QString(divLine[1]).trimmed().toInt();
                float val1 = QString(divLine[2]).trimmed().toFloat();
                float val2 = QString(divLine[3]).trimmed().toFloat();

                if( etmode == 1 ){
                    float r = simManage->GenUniform();
                    val1 = val1 + r * (val2 - val1);
                }
                else if( etmode == 2 ){
                    val1 = simManage->GetNormalDist(val1, val2);
                }

                if( val1 < 0.0 ){
                    val1 = 0.0;
                }

                struct SimulationTime *t = new struct SimulationTime;

                t->day    = 0;
                t->hour   = val1 / 3600;
                t->min    = val1 / 60 - t->hour * 3600;
                t->msec   = val1 - t->min * 60 - t->hour * 3600;

                t->sec = (int)(t->msec);
                t->msec -= t->sec;

                trigger->timeTrigger = t;
                trigger->timeTriggerInSec = t->msec + t->sec + t->min * 60.0 + t->hour * 3600.0 + t->day * 86400.0;
            }
        }
        else if( tag == QString("Scenario Event Param Float") ){
            for(int i=1;i<divLine.size();++i){
                scenarioEvent->eventFloatData.append( QString(divLine[i]).trimmed().toFloat() );
            }
        }
        else if( tag == QString("Scenario Event Param Integer") ){
            for(int i=1;i<divLine.size();++i){
                scenarioEvent->eventIntData.append( QString(divLine[i]).trimmed().toInt() );
            }
        }
    }
    file.close();


    emit SetMaxNumberAgent( maxAgent );

    //
    //  Allocate Agents
    agent = new Agent* [maxAgent];
    if( !agent ){
        QMessageBox::warning(NULL,QString("Error"),QString("Cannot allocate agent instances. Max=%1").arg(maxAgent));
        return;
    }
    for(int i=0;i<maxAgent;++i){
        agent[i] = new Agent();
        if( !agent[i] ){
            QMessageBox::warning(NULL,QString("Error"),QString("Cannot allocate agent instances. Max=%1").arg(maxAgent));
            return;
        }
        agent[i]->ID = i;
    }

    emit SetAgentPointer(agent);


    //
    //  Load Road Data
    LoadRoadData( roadDataFile );



    //
    //  Traffic Signals
    //
    if( signalDataFile.isNull() == false ){

        qDebug() << "Now call LoadTrafficSignalData()";
//        getchar();

        LoadTrafficSignalData( signalDataFile );

        qDebug() << "emit SetNumberTrafficSignal:  numTrafficSignals = " << trafficSignal.size();

        emit SetNumberTrafficSignal( trafficSignal.size() );

        if( trafficSignal.size() > 0 ){

            qDebug() << "emit SetTrafficSignalPointer";

            emit SetTrafficSignalPointer(trafficSignal);
        }

        //qDebug() << "end of TS";
    }


    simManage->DumpScenarioData();


    simulationState = SIMULATION_STATE::READY_TO_START;
}


void SystemThread::LoadTrafficSignalData(QString signalFile)
{
    qDebug() << "[SystemThread::LoadTrafficSignalData] signalFile = " << signalFile;

    QFile file( CheckNetworkDrive(signalFile) );
    if( file.open(QIODevice::ReadOnly | QIODevice::Text) == false ){
        QMessageBox::warning(NULL,QString("Error"),QString("Cannot open Traffic Signal Data file."));
        return;
    }

    QString line;
    QStringList divLine;

    QTextStream in(&file);

    line = in.readLine();
    line = in.readLine();
    if( line.contains("Re:sim Traffic Signal Data File") == false ){
        file.close();
        QMessageBox::warning(NULL,QString("Error"),QString("This is not Re:sim Traffic Signal Data File."));
        return;
    }
    line = in.readLine();

    qDebug() << "Re:sim Traffic Signal Data File";


    while( in.atEnd() == false ){

        line = in.readLine();
        if( line.startsWith("#") || line.isEmpty() || line.contains(";") == false ){
            continue;
        }

        divLine = line.split(";");

        QString tag = QString( divLine[0] ).trimmed();
        if( tag == QString("Type") ){

            QString dataStr = QString( divLine[1] ).trimmed();
            QStringList divDataStr = dataStr.split(",");

            int id = QString( divDataStr[0] ).trimmed().toInt();

            TrafficSignal *ts = new TrafficSignal;

            ts->id = id;


            QString typeStr = QString( divDataStr[1] ).trimmed();
            if( typeStr.contains("v") == true ){
                ts->type = 'v';
            }
            else if( typeStr.contains("p") == true ){
                ts->type = 'p';
            }

            trafficSignal.append( ts );
            tsId2Index.append( id );

        }
        else if( tag == QString("Location") ){

            QString dataStr = QString( divLine[1] ).trimmed();
            QStringList divDataStr = dataStr.split(",");
            if( divDataStr.size() == 4 ){

                int id = QString( divDataStr[0] ).trimmed().toInt();
                int tsIdx = tsId2Index.indexOf( id );
                if( tsIdx >= 0 ){
                    float x = QString( divDataStr[1] ).trimmed().toFloat();
                    float y = QString( divDataStr[2] ).trimmed().toFloat();
                    float dir = (QString( divDataStr[3] ).trimmed().toFloat() * 0.017452);  // [deg]->[rad]

                    trafficSignal[tsIdx]->SetLocationInfo( x, y, dir );

                    //qDebug() << "Location ; x = " << x << " y = " << y << " dir = " << dir;
                }
            }
        }
        else if( tag == QString("Control") ){

            QString dataStr = QString( divLine[1] ).trimmed();
            QStringList divDataStr = dataStr.split(",");
            if( divDataStr.size() == 5 ){
                int id = QString( divDataStr[0] ).trimmed().toInt();
                int tsIdx = tsId2Index.indexOf( id );
                if( tsIdx >= 0 ){

                    int nd = QString( divDataStr[1] ).trimmed().toInt();
                    int dir = QString( divDataStr[2] ).trimmed().toInt();
                    int lane = QString( divDataStr[3] ).trimmed().toInt();
                    int cw = QString( divDataStr[4] ).trimmed().toInt();

                    float z = 0.0;
                    if( road->nodeId2Index.size() > 0 ){
                        int nIdx = road->nodeId2Index.indexOf( nd );
                        if( nIdx >= 0 && nIdx < road->nodes.size() ){
                            int nAdd = 0;
                            for(int i=0;i<road->nodes[nIdx]->nCross;++i){
                                for(int j=0;j<road->nodes[nIdx]->inBoundaryWPs.size();++j){
                                    int inWP = road->nodes[nIdx]->inBoundaryWPs[j]->wpId;
                                    int inWPidx = road->wpId2Index.indexOf(inWP);
                                    if( inWPidx >= 0 && inWPidx < road->wps.size() ){
                                        z += road->wps[inWPidx]->pos.z();
                                        nAdd++;
                                    }
                                }
                            }
                            if( nAdd > 0 ){
                                z /= nAdd;
                            }
                        }
                    }

                    trafficSignal[tsIdx]->SetControlInfo(nd,dir,lane,cw,z);

                    //qDebug() << "Control ; Node = " << nd << " Dir = " << dir << " Lane = " << lane << " CrossWalk = " << cw << " Z = " << z;
                }
            }
        }
        else if( tag == QString("Display") ){

            QString dataStr = QString( divLine[1] ).trimmed();
            QStringList divDataStr = dataStr.split(",");
            if( divDataStr.size() > 2 ){
                int id = QString( divDataStr[0] ).trimmed().toInt();
                int tsIdx = tsId2Index.indexOf( id );
                if( tsIdx >= 0 ){

                    trafficSignal[tsIdx]->startOffset = QString( divDataStr[1] ).trimmed().toInt();

                    //qDebug() << "Display ; startOffset = " << trafficSignal[tsIdx]->startOffset ;


                    for(int j=2;j<divDataStr.size();++j){

                        QStringList valStrs = QString( divDataStr[j] ).trimmed().split("/");
                        if( valStrs.size() == 2 ){

                            struct SignalDisplayPattern *sp =new SignalDisplayPattern;

                            sp->displayInfo = QString( valStrs[0] ).trimmed().toInt();
                            sp->duration    = QString( valStrs[1] ).trimmed().toInt();

                            trafficSignal[tsIdx]->AddSignalDisplayPattern( sp );
                        }

                        //qDebug() << "Display pattern = ; " << valStrs;
                    }
                }
            }
        }
    }

    numTrafficSignals = trafficSignal.size();

    qDebug() << "size of trafficSignal = " << numTrafficSignals;

    file.close();
}


void SystemThread::LoadRoadData(QString roadDataFile)
{
    if( !road ){
        qDebug() << "[LoadRoadData] invalid road instance.";
        return;
    }
    qDebug() << "[SystemThread::LoadRoadData] roadDataFile = " << roadDataFile;
    road->LoadRoadData( roadDataFile );


    //
    //  Add WP and Path for Scenario Objects
    //
    int numScenario = simManage->GetNumberScenarioData();
    for(int i=0;i<numScenario;++i){
        int numObj = simManage->GetNumberScenarioObject(i);
        for(int j=0;j<numObj;++j){
            if( simManage->GetScenarioObjectType(i,j) != 'v' ){
                continue;
            }
            int numWP = simManage->GetNumberWPDataOfScenarioObject(i,j);
            if( numWP == 0 ){
                continue;
            }
            int objectID = simManage->GetScenarioObjectID(i,j);
            for(int k=0;k<numWP;++k){
                struct ScenarioWPRoute *wpdata = simManage->GetWPDataOfScenarioObject(i,j,k);

                int id = road->CreateWPforScenarioObject( wpdata->x,
                                                          wpdata->y,
                                                          wpdata->z,
                                                          wpdata->direct,
                                                          objectID,
                                                          k,
                                                          wpdata->speedInfo);
                wpdata->wpID = id;
            }
            road->CreatePathsforScenarioObject(objectID);
        }

        for(int j=0;j<numObj;++j){
            if( simManage->GetScenarioObjectType(i,j) != 'p' ){
                continue;
            }
            if( simManage->GetScenarioObjectRouteType(i,j) != 1 ){
                continue;
            }
            int nPPElem = simManage->GetNumberPPElemOfScenarioObject(i,j);
            if( nPPElem == 0 ){
                continue;
            }
            int objectID = simManage->GetScenarioObjectID(i,j);
            for(int k=0;k<nPPElem;++k){
                struct ScenarioPedestPathRoute *ppElem = simManage->GetPPElemOfScenarioObject(i,j,k);

                int id = road->CreatePedestPathsforScenarioObject(objectID,
                                                                  ppElem->x1,ppElem->y1,ppElem->z1,
                                                                  ppElem->x2,ppElem->y2,ppElem->z2,
                                                                  ppElem->width,
                                                                  ppElem->isCrossWalk,
                                                                  ppElem->roadSideInfo);
                if( id >= 0 ){
                    simManage->SetPedestPathRouteToScenarioObject(i,j,id);
                    qDebug() << "PedestPath " << id << " : created";
                }
            }
        }
    }
    road->SetPathConnection();

    road->CheckPedestPathConnection();


    qDebug() << "emit SetRoadDataToCanvas signal";
    emit SetRoadDataToCanvas( road );
}


void SystemThread::SetSupplementFile(QString filename)
{
    qDebug() << "[SystemThread::SetSupplementFile] suppleFileName = " << filename;

    suppleFileName = filename;

}


void SystemThread::SetLogOutputFolder(QString folder)
{
    qDebug() << "[SystemThread::SetLogOutputFolder] logOutputFolder = " << folder;

    logOutputFolder = folder;
}


void SystemThread::SetLogFileName(QString filename)
{
    qDebug() << "[SystemThread::SetLogFileName] filename = " << filename;

    logFileName = filename;
}


void SystemThread::SetLogOutputInterval(int interval)
{
    qDebug() << "[SystemThread::SetLogOutputInterval] interval = " << interval;

    logOutputInterval = interval;
}


void SystemThread::SimulationStart()
{
    qDebug() << "[SystemThread::SimulationStart]";
    //
    // Check
    if( scenarioFile == QString() ){
        qDebug() << "[SystemThread::SimulationStart] scenario file not assigned.";
        return;
    }

    if( simulationState != SIMULATION_STATE::READY_TO_START ){
        qDebug() << "[SystemThread::SimulationStart] simulation state is not ready to start.";
        return;
    }

    simManage->ResetSimulationTime();

    simulationState = SIMULATION_STATE::STARTED;

    start();
}


void SystemThread::SimulationPause()
{
    if( simulationState == SIMULATION_STATE::STARTED ){
        simulationState = SIMULATION_STATE::PAUSED;
    }
}


void SystemThread::SimulationResume()
{
    if( simulationState == SIMULATION_STATE::PAUSED ){
        simulationState = SIMULATION_STATE::STARTED;
    }
}


void SystemThread::SimulationStop()
{
    qDebug() << "[SystemThread::SimulationStop]";

    if( simulationState == STARTED ){
        simulationState = SIMULATION_STATE::READY_TO_START;
        Stop();
    }
}


void SystemThread::WrapSetVehicleParameter(int ID, int Type, float length, float width, float height, float wheelBase, float distRR2RE, float FRWeightRatio)
{
    if( simManage ){
        simManage->SetVehicleShapeParameter( ID, Type, length, width, height, wheelBase, distRR2RE, FRWeightRatio );
    }
}


void SystemThread::SetSpeedAdjustVal(int val)
{
    //qDebug() << "[SystemThread::SetSpeedAdjustVal] val = " << val;
    speedAdjustVal = val;
}


void SystemThread::SetSendData(char *sendData, int maxSize,int *pos)
{
    int n = 0;

    // count number of data
    for(int i=0;i<maxAgent;++i){
        if( agent[i]->agentStatus == 0 ){
            continue;
        }
        if( agent[i]->isSInterfaceObject == true ){
            continue;
        }
        n++;
    }


    int at = *pos;

    memcpy(&(sendData[at]), &n, sizeof(int));   // 3
    at += sizeof(int);

    float tempFVal = 0.0;
    int   tempIVal = 0;
    char  tempCVal = 0;


    for(int i=0;i<maxAgent;++i){

        if( agent[i]->agentStatus == 0 ){
            continue;
        }
        if( agent[i]->isSInterfaceObject == true ){
            continue;
        }

        if( at + 125 < maxSize ){

            tempCVal = 'a';
            memcpy(&(sendData[at]), &tempCVal, sizeof(char));   // 7
            at += sizeof(char);

            tempIVal = agent[i]->ID;
            memcpy(&(sendData[at]), &tempIVal, sizeof(int));   // 8
            at += sizeof(int);

            tempFVal = agent[i]->state.x;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 12
            at += sizeof(float);

            tempFVal = agent[i]->state.y * (-1.0);
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 16
            at += sizeof(float);

            tempFVal = agent[i]->state.z;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 20
            at += sizeof(float);

            tempFVal = agent[i]->state.roll;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 24
            at += sizeof(float);

            tempFVal = agent[i]->state.pitch;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 28
            at += sizeof(float);

            tempFVal = agent[i]->state.yaw * (-1.0);
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 32
            at += sizeof(float);

            tempFVal = agent[i]->state.V;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 36
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.ax;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 40
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.yawRate;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 44
            at += sizeof(float);

            tempFVal = 0.0;   // Ay
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 48
            at += sizeof(float);

            tempFVal = 0.0;   // Vy
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 52
            at += sizeof(float);

            tempCVal = 0;     // engineKeyState
            memcpy(&(sendData[at]), &tempCVal, sizeof(char));   // 56
            at += sizeof(char);

            tempCVal = 0;     // gearPosition
            memcpy(&(sendData[at]), &tempCVal, sizeof(char));   // 57
            at += sizeof(char);

            tempIVal = 0;     // Tachometer
            memcpy(&(sendData[at]), &tempIVal, sizeof(int));   // 58
            at += sizeof(int);

            tempCVal = 0;     // lightFlag1
            if( agent[i]->vehicle.GetBrakeLampState() == 1 ){
                tempCVal += 0x01;
            }
//            int winkerState = agent[i]->vehicle.GetWinerState();
            int winkerState = agent[i]->vehicle.GetWinkerIsBlink();
            if( winkerState == 1 ){   // Left Winker
                tempCVal += 0x02;
            }
            else if( winkerState == 2 ){  // Right Winker
                tempCVal += 0x04;
            }
            else if( winkerState == 3 ){  // Hazard
                tempCVal += 0x06;
            }

//            qDebug() << "Brake = " << agent[i]->vehicle.GetBrakeLampState() << " send = " << (int)tempCVal;

            memcpy(&(sendData[at]), &tempCVal, sizeof(char));   // 62
            at += sizeof(char);

            tempCVal = 0;     // lightFlag2
            memcpy(&(sendData[at]), &tempCVal, sizeof(char));   // 63
            at += sizeof(char);

            tempFVal = agent[i]->vehicle.param.Lf;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 64
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.param.Lr;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 68
            at += sizeof(float);

            tempFVal = 1.8;     // Tf
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 72
            at += sizeof(float);

            tempFVal = 1.8;     // Tr
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 76
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.tireRotAngle;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 80
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.steer * (-57.3 / 6.0) ;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 84
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.tireRotAngle;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 88
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.steer * (-57.3 / 6.0);
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 92
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.tireRotAngle;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 96
            at += sizeof(float);

            tempFVal = 0.0;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 100
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.tireRotAngle;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 104
            at += sizeof(float);

            tempFVal = 0.0;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 108
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.relZBody;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 112
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.relRollBody;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 116
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.relPitchBody;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 120
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.relYBody;
            memcpy(&(sendData[at]), &tempFVal, sizeof(float));   // 124
            at += sizeof(float);

            tempCVal = 0;     // optionalFlag1
            memcpy(&(sendData[at]), &tempCVal, sizeof(char));   // 128
            at += sizeof(char);

            tempCVal = 0;     // optionalFlag2
            memcpy(&(sendData[at]), &tempCVal, sizeof(char));   // 129
            at += sizeof(char);

            tempCVal = 0;     // optionalFlag3
            memcpy(&(sendData[at]), &tempCVal, sizeof(char));   // 130
            at += sizeof(char);

            tempCVal = 0;     // optionalFlag4
            memcpy(&(sendData[at]), &tempCVal, sizeof(char));   // 132
            at += sizeof(char);
        }
    }

    memcpy(&(sendData[at]), &numTrafficSignals, sizeof(int));
    at += sizeof(int);

    for(int i=0;i<numTrafficSignals;++i){

        memcpy(&(sendData[at]), &(trafficSignal[i]->id), sizeof(int));
        at += sizeof(int);

        sendData[at] = trafficSignal[i]->type;
        at += sizeof(char);

        int display = trafficSignal[i]->GetCurrentDisplayInfo();
        memcpy(&(sendData[at]), &display, sizeof(int));
        at += sizeof(int);

        float remainTime = trafficSignal[i]->remainingTimeToNextDisplay;
        memcpy(&(sendData[at]), &remainTime, sizeof(float));
        at += sizeof(float);
    }


    *pos = at;   // 132

    //qDebug() << "data size = " << at;
}


void SystemThread::SetSimulationFrequency(int hz)
{
    qDebug() << "[SetSimulationFrequency] SetFrequency(" << hz << ")";
    calHz = hz;
    intervalMSec = (int)(1000.0 / calHz);
    simManage->SetFrequency( hz );
}


void SystemThread::SetTireHeight(int id, float zFL, float zFR, float zRL, float zRR)
{
    int idx = -1;
    for(int i=0;i<maxAgent;++i){
        if( agent[i]->ID == id ){
            idx = i;
            break;
        }
    }
    if( idx < 0 ){
        return;
    }

    zFL *= 0.01;
    zFR *= 0.01;
    zRL *= 0.01;
    zRR *= 0.01;

    if( agent[idx]->agentKind < 100 ){
        if( agent[idx]->vehicle.tireFL4CG != NULL ){
            agent[idx]->vehicle.tireFL4CG->SetInput( zFL );
        }

        if( agent[idx]->vehicle.tireFR4CG != NULL ){
            agent[idx]->vehicle.tireFR4CG->SetInput( zFR );
        }

        if( agent[idx]->vehicle.tireRL4CG != NULL ){
            agent[idx]->vehicle.tireRL4CG->SetInput( zRL );
        }

        if( agent[idx]->vehicle.tireRR4CG != NULL ){
            agent[idx]->vehicle.tireRR4CG->SetInput( zRR );
        }

        if( agent[idx]->vehicle.suspentionFL4CG != NULL ){
            agent[idx]->vehicle.suspentionFL4CG->SetInput( zFL );
        }

        if( agent[idx]->vehicle.suspentionFR4CG != NULL ){
            agent[idx]->vehicle.suspentionFR4CG->SetInput( zFR );
        }

        if( agent[idx]->vehicle.suspentionRL4CG != NULL ){
            agent[idx]->vehicle.suspentionRL4CG->SetInput( zRL );
        }

        if( agent[idx]->vehicle.suspentionRR4CG != NULL ){
            agent[idx]->vehicle.suspentionRR4CG->SetInput( zRR );
        }
    }
    else if( agent[idx]->agentKind >= 100 ){

        agent[idx]->vehicle.state.Z = zFL;

    }

}


void SystemThread::SetSInterObjData(char type, int id, AgentState *as,struct SInterObjInfo *so)
{
    if( type == 'v' || type == 'm' ){
        agent[id]->agentKind = 0;
    }
    else if( type == 'p' || type == 'b' ){
        agent[id]->agentKind = 100;
    }

    agent[id]->ID = id;

    agent[id]->isSInterfaceObject = true;

    if( agent[id]->state.warpFlag == 0 ){
        float dx = agent[id]->state.x - as->x;
        float dy = agent[id]->state.y - as->y;
        float L = dx * dx + dy * dy;
        if( L > 400 ){
            agent[id]->state.warpFlag = 1;
        }
    }

    agent[id]->state.x = as->x;
    agent[id]->state.y = as->y;
    agent[id]->state.z = as->z;

    agent[id]->state.roll  = as->roll;
    agent[id]->state.pitch = as->pitch;
    agent[id]->state.yaw   = as->yaw;

    agent[id]->state.V      = as->V;
    agent[id]->state.cosYaw = as->cosYaw;
    agent[id]->state.sinYaw = as->sinYaw;

    agent[id]->state.accel = as->accel;
    agent[id]->state.brake = as->brake;
    agent[id]->state.steer = as->steer;  // [rad]

    agent[id]->state.accel_log = as->accel_log;
    agent[id]->state.brake_log = as->brake_log;
    agent[id]->state.steer_log = as->steer_log;  // [rad]

    agent[id]->agentStatus = 1;

    if( agent[id]->vehicle.GetVehicleModelID() < 0 ){
        float wheelbase = so->lf + so->lr;
        int vModelID = simManage->GetVehicleShapeByWheelbase( wheelbase );
        agent[id]->vehicle.SetVehicleModelID( vModelID );
    }

    if( so->brakeLamp == true ){
        agent[id]->vehicle.SetBrakeLampState( 1 );
    }
    else{
        agent[id]->vehicle.SetBrakeLampState( 0 );
    }

    int winkerVal = 0;
    if( so->winkerLeft == true && so->winkerRight == false ){
        winkerVal = 1;
    }
    else if( so->winkerLeft == false && so->winkerRight == true ){
        winkerVal = 2;
    }
    else if( so->winkerLeft == true && so->winkerRight == true ){
        winkerVal = 3;
    }

    agent[id]->vehicle.SetWinker( winkerVal );

    if( so->lowbeam == true ){
        if( so->highbeam == true ){
            agent[id]->vehicle.headlight = 2;
        }
        else{
            agent[id]->vehicle.headlight = 1;
        }
    }
    else{
        agent[id]->vehicle.headlight = 0;
    }

}


void SystemThread::ForceChangeTSColor(int tsID, int tsColor, float duration)
{
    if( tsID < 0 || tsID >= numTrafficSignals){
        return;
    }

    if( tsColor == 0 ){
        float simTimeFVal = simManage->GetSimulationTimeInSec();
        trafficSignal[tsID]->nextUpdateTimeFVal = simTimeFVal;
        trafficSignal[tsID]->CheckDisplayInfoUpdate( simTimeFVal );
        return;
    }
    else if( tsColor > 0 ){

        bool foundTSPattern = false;
        for(int i=0;i<trafficSignal[tsID]->displayPattern.size();++i){
            if( trafficSignal[tsID]->displayPattern[i]->displayInfo == tsColor ){

                trafficSignal[tsID]->displayPattern[i]->duration = duration;

                float simTimeFVal = simManage->GetSimulationTimeInSec();
                trafficSignal[tsID]->ForceChangeDisplayTo(simTimeFVal,i);

                foundTSPattern = true;
                break;
            }
        }

        if( foundTSPattern == false ){
            struct SignalDisplayPattern *sp = new struct SignalDisplayPattern;
            sp->displayInfo = tsColor;
            sp->duration = duration;
            trafficSignal[tsID]->AddSignalDisplayPattern( sp );

            float simTimeFVal = simManage->GetSimulationTimeInSec();
            int idx = trafficSignal[tsID]->displayPattern.size() - 1;
            trafficSignal[tsID]->ForceChangeDisplayTo(simTimeFVal,idx);
        }
    }
}

void SystemThread::WarpVehicle(int targetVID, float xTo, float yTo, float dirTo)
{

    qDebug() << "[WarpVehicle] xTo = " << xTo << " yTo = " << yTo << " dirTo = " << dirTo;

    agent[targetVID]->state.x = xTo;
    agent[targetVID]->state.y = yTo;
    agent[targetVID]->vehicle.state.X = xTo;
    agent[targetVID]->vehicle.state.Y = yTo;

    agent[targetVID]->vehicle.state.yawAngle = dirTo * 0.017452;
    agent[targetVID]->state.yaw = agent[targetVID]->vehicle.state.yawAngle;
    agent[targetVID]->state.cosYaw = cos( agent[targetVID]->state.yaw );
    agent[targetVID]->state.sinYaw = sin( agent[targetVID]->state.yaw );

    agent[targetVID]->vehicle.state.XRear = agent[targetVID]->vehicle.state.X;
    agent[targetVID]->vehicle.state.XRear -= agent[targetVID]->state.cosYaw * agent[targetVID]->vehicle.param.Lr;
    agent[targetVID]->vehicle.state.YRear = agent[targetVID]->vehicle.state.Y;
    agent[targetVID]->vehicle.state.YRear -= agent[targetVID]->state.sinYaw * agent[targetVID]->vehicle.param.Lr;

    agent[targetVID]->justWarped = true;

    agent[targetVID]->CheckPathList( road );
}


void SystemThread::DisposeAgent(int agentID)
{
    if( agentID >= 0 && agentID < maxAgent ){
        agent[agentID]->agentStatus = 2;
        simManage->DisappearAgents(agent,maxAgent);
    }
}

void SystemThread::AppearAgent(int agentID)
{
    if( agentID >= 0 && agentID < maxAgent ){
        simManage->SetAppearFlagByFE( agentID );
        simManage->AppearAgents(agent,maxAgent,road);
    }
}

void SystemThread::ChangeReferenceSpeed(int agentID, float refSpeed)
{
    if( agentID >= 0 && agentID < maxAgent ){
        agent[agentID]->memory.targetSpeedByScenario = refSpeed;
        //qDebug() << "agentID=" << agentID << " targetSpeedByScenario=" << refSpeed;
    }
}


void SystemThread::ForceChangeSpeed(int agentID, float speed)
{
    if( agentID >= 0 && agentID < maxAgent ){
        agent[agentID]->state.V = speed;
        agent[agentID]->vehicle.state.vx = speed;
    }
}


void SystemThread::CopyScenarioData(int fromAID, int toAID)
{
    if( fromAID >= 0 && fromAID < maxAgent && toAID >= 0 && toAID < maxAgent ){
        simManage->CopyScenarioData(fromAID,toAID);
        agent[toAID]->memory.scenarioPathSelectID = fromAID;
    }
}


void SystemThread::ChangeVehicleWinkers(int agentID,int wState)
{
    if( agentID >= 0 && agentID < maxAgent ){
        agent[agentID]->vehicle.SetWinker(wState);
    }
}


void SystemThread::SetLateralShift(int agentID,float lateralShift)
{
    if( agentID >= 0 && agentID < maxAgent ){
        agent[agentID]->memory.targetLateralShiftByScenario = lateralShift;
    }
}


void SystemThread::SetLateralGainMultiplier(int agentID,float gainMultiplier)
{
    if( agentID >= 0 && agentID < maxAgent ){
        agent[agentID]->memory.steeringControlGainMultiplier = gainMultiplier;
    }
}


void SystemThread::SetAgentGenerationNotAllowFlag(int agentID, bool flag )
{
    if( agentID >= 0 && agentID < maxAgent ){
        agent[agentID]->notAllowedAppear = flag;
    }
}


void SystemThread::CopyPathData(int fromAID, int toAID)
{
    if( fromAID >= 0 && fromAID < maxAgent && toAID >= 0 && toAID < maxAgent ){

        agent[toAID]->memory.targetPathList.clear();

        qDebug() << "[CopyPathData]";
        qDebug() << "Path of Agent ID = " << fromAID << " size = " << agent[fromAID]->memory.targetPathList.size();

        if( agent[fromAID]->memory.targetPathList.size() == 0 ){
            simManage->SetTargetPathListScenarioVehicle( agent, road, fromAID );
            qDebug() << " reset : size = " << agent[fromAID]->memory.targetPathList.size();
        }

        for(int i=0;i<agent[fromAID]->memory.targetPathList.size();++i){

            qDebug() << " [" << i << "] " << agent[fromAID]->memory.targetPathList[i];

            agent[toAID]->memory.targetPathList.append( agent[fromAID]->memory.targetPathList[i] );
        }

        float dist = 0;
        float xi = agent[toAID]->state.x;
        float yi = agent[toAID]->state.y;
        float YAi = agent[toAID]->state.yaw;
        int currentPath = road->GetNearestPathFromList( xi, yi, YAi, dist, agent[toAID]->memory.targetPathList );
        if( currentPath < 0 ){
            qDebug() << "[Warning]----------------------------------";
            qDebug() << " Scenario Vehicle ID = " << toAID << " cannot determin nearest path from assigned list.";
            qDebug() << "   Assigned Path List : ";
            for(int j=0;j<agent[toAID]->memory.targetPathList.size();++j){
                qDebug() << "           Path " << agent[toAID]->memory.targetPathList[j];

                int pid = agent[toAID]->memory.targetPathList[j];
                int pdx = road->pathId2Index[pid];
                qDebug() << "ps=" << road->paths[pdx]->pos.first()->x() << "," << road->paths[pdx]->pos.first()->y();
                qDebug() << "pe=" << road->paths[pdx]->pos.last()->x() << "," << road->paths[pdx]->pos.last()->y();
            }
            qDebug() << "xi=" << xi << " yi=" << yi << " YAi=" << YAi;

            currentPath = agent[toAID]->memory.targetPathList.last();
            agent[toAID]->memory.currentTargetPath = currentPath;
            for(int i=0;i<agent[toAID]->memory.targetPathList.size();++i){
                if( agent[toAID]->memory.targetPathList[i] == currentPath ){
                    agent[toAID]->memory.currentTargetPathIndexInList = i;
                    break;
                }
            }

            qDebug() << "set currentPath = " << currentPath << " index = " << agent[toAID]->memory.currentTargetPathIndexInList;

        }
        else{
            agent[toAID]->memory.currentTargetPath = currentPath;
            for(int i=0;i<agent[toAID]->memory.targetPathList.size();++i){
                if( agent[toAID]->memory.targetPathList[i] == currentPath ){
                    agent[toAID]->memory.currentTargetPathIndexInList = i;
                    break;
                }
            }
            qDebug() << "Path Data copied: from aID=" << fromAID << " to aID=" << toAID;
        }
    }
}


void SystemThread::ChangeControlModeStopAt(int aID, float atX, float atY)
{
    if( aID >= 0 && aID < maxAgent ){

        agent[aID]->memory.controlMode = AGENT_CONTROL_MODE::STOP_AT;
        agent[aID]->memory.targetStopAtXByScenario = atX;
        agent[aID]->memory.targetStopAtYByScenario = atY;
        agent[aID]->memory.actualStopOnPathID = -1;
        agent[aID]->skipSetControlInfo = true;

        qDebug() << "Change control mode to Stop-At : aID=" << aID << " Xstop=" << atX << " Ystop=" << atY;
    }
}


void SystemThread::ChangeControlModeHeadwayControl(int aID, float V, float dist,float allowDev,float time,int targetID)
{
    if( aID >= 0 && aID < maxAgent ){

        agent[aID]->memory.controlMode = AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY;
        agent[aID]->memory.targetSpeedByScenario = V;
        agent[aID]->memory.targetHeadwayDistanceByScenario = dist;
        agent[aID]->memory.allowableHeadwayDistDeviation = allowDev;
        agent[aID]->memory.targetHeadwayTimeByScenario = time;
        agent[aID]->memory.precedingVehicleIDByScenario = targetID;
        agent[aID]->skipSetControlInfo = true;

        qDebug() << "Change control mode to Headway Control : aID=" << aID << " V=" << V*3.6 << " D=" << dist << " T=" << time << " target=" << targetID;
    }
}


void SystemThread::ChangeControlModeAgent(int aID)
{
    if( aID >= 0 && aID < maxAgent ){
        agent[aID]->memory.controlMode = AGENT_CONTROL_MODE::AGENT_LOGIC;
        agent[aID]->skipSetControlInfo = true;

//        qDebug() << "Change control mode to Agent Control";
    }
}


void SystemThread::ChangeControlModeIntersectionTurn(int aID,float speed,float insideSpeed)
{
    if( aID >= 0 && aID < maxAgent ){
        agent[aID]->memory.controlMode = AGENT_CONTROL_MODE::INTERSECTION_TURN_CONTROL;
        agent[aID]->memory.targetSpeedByScenario = speed;
        agent[aID]->memory.targetSpeedInsideIntersectionTurnByScenario = insideSpeed;
        agent[aID]->skipSetControlInfo = true;

        qDebug() << "Change control mode to Intersection Turn Control: Vn = " << speed*3.6 << " Vt=" << insideSpeed*3.6;
    }
}


void SystemThread::EmbedAgentBehavior(int id, float *data, int sizeData)
{
//    embedData[0] = X;
//    embedData[1] = Y;
//    embedData[2] = Z;
//    embedData[3] = Roll;
//    embedData[4] = Pitch;
//    embedData[5] = Yaw;
//    embedData[6] = V;
//    embedData[7] = Steer;
//    embedData[8] = Brake;
    if( sizeData < 9 ){
        return;
    }

    agent[id]->ID = id;

    agent[id]->isBehaviorEmbeded = true;

    agent[id]->state.x = data[0];
    agent[id]->state.y = data[1];
    agent[id]->state.z = data[2];

    agent[id]->state.roll  = data[3];
    agent[id]->state.pitch = data[4];
    agent[id]->state.yaw   = data[5];

    agent[id]->state.V      = data[6];
    agent[id]->state.cosYaw = cos(data[5]);
    agent[id]->state.sinYaw = sin(data[5]);

    agent[id]->state.accel = 0.0;
    agent[id]->state.brake = data[8];
    agent[id]->state.steer = data[7];

    agent[id]->agentStatus = 1;


    if( data[8] > 0.05 ){
        agent[id]->vehicle.SetBrakeLampState( 1 );
    }
    else{
        agent[id]->vehicle.SetBrakeLampState( 0 );
    }

    float deltaAngle = agent[id]->state.V / 0.270 * (intervalMSec * 0.001) * 57.3;
    agent[id]->vehicle.state.tireRotAngle += deltaAngle;
    while(1){
        if( agent[id]->vehicle.state.tireRotAngle > 180.0 ){
            agent[id]->vehicle.state.tireRotAngle -= 360.0;
        }
        else if( agent[id]->vehicle.state.tireRotAngle < -180.0 ){
            agent[id]->vehicle.state.tireRotAngle += 360.0;
        }
        else{
            break;
        }
    }

    agent[id]->vehicle.state.steer = agent[id]->state.steer * 0.017452;  // This is steering-wheel angle.
                                                                         // Front-wheel angle is divided by 6, assumed gear ratio
                                                                         // and is send to UE4
}


void SystemThread::SetStopGraphicUpdate(bool b)
{
    stopGraphicUpdate = b;
    qDebug() << " -> stopGraphicUpdate = " << stopGraphicUpdate;
}



void SystemThread::ShowAgentData(float x,float y)
{
    qDebug() << "[SystemThread::ShowAgentData]";
    if( agent == NULL ){
        return;
    }

    char labelStr[][30] = {
        "PRECEDING              ",
        "FOLLOWING              ",
        "LEFT_SIDE              ",
        "RIGHT_SIDE             ",
        "LEFT_SIDE_PRECEDING    ",
        "RIGHT_SIDE_PRECEDING   ",
        "LEFT_SIDE_FOLLOWING    ",
        "RIGHT_SIDE_FOLLOWING   ",
        "ONCOMING_STRAIGHT      ",
        "ONCOMING_LEFT          ",
        "ONCOMING_RIGHT         ",
        "LEFT_CROSSING_STRAIGHT ",
        "LEFT_CROSSING_LEFT     ",
        "LEFT_CROSSING_RIGHT    ",
        "RIGHT_CROSSING_STRAIGHT",
        "RIGHT_CROSSING_LEFT    ",
        "RIGHT_CROSSING_RIGHT   ",
        "UNDEFINED_RECOGNITION_LABEL"
    };


    int nearID = -1;
    float dist = 0.0;
    for(int i=0;i<maxAgent;++i){
        if(agent[i]->agentStatus != 1 ){
            continue;
        }

        //qDebug() << "agent " << agent[i]->ID;

        float dx = agent[i]->state.x - x;
        float dy = agent[i]->state.y - y;
        float D = dx * dx + dy * dy;

        //qDebug() << "D = " << D;

        if( D > 16.0 ){
            continue;
        }

        if( nearID < 0 || dist > D ){
            nearID = i;
            dist = D;
        }
    }
    //qDebug() << "nearID " << nearID;
    if( nearID < 0 ){
        return;
    }

    qDebug() << "Picked Agent ID = " << agent[nearID]->ID;

    qDebug() << "Vehicle State:";
    qDebug() << "  X = " << agent[nearID]->state.x;
    qDebug() << "  Y = " << agent[nearID]->state.y;
    qDebug() << "  Yaw = " << agent[nearID]->state.yaw;
    qDebug() << "  V = " << agent[nearID]->state.V;
    qDebug() << "  Accel = " << agent[nearID]->vehicle.input.accel;
    qDebug() << "  Winker = " << agent[nearID]->vehicle.GetWinerState();

    qDebug() << "Recognizied Object Info:";
    for(int i=0;i<agent[nearID]->memory.perceptedObjects.size();++i){
        if( agent[nearID]->memory.perceptedObjects[i]->isValidData == false ){
            continue;
        }
        qDebug() << "  OBJ:" << agent[nearID]->memory.perceptedObjects[i]->objectID
                 << " Label=" << labelStr[ agent[nearID]->memory.perceptedObjects[i]->recognitionLabel ]
                 << " Dist=" << agent[nearID]->memory.perceptedObjects[i]->distanceToObject
                 << " e = " << agent[nearID]->memory.perceptedObjects[i]->deviationFromNearestTargetPath
                 << " W = " << agent[nearID]->memory.perceptedObjects[i]->effectiveHalfWidth;
    }

    qDebug() << "Recognizied Traffic Signal Info:";
    for(int i=0;i<agent[nearID]->memory.perceptedSignals.size();++i){
        if(agent[nearID]->memory.perceptedSignals[i]->isValidData == false ){
            continue;
        }
        qDebug() << "  OBJ:" << agent[nearID]->memory.perceptedSignals[i]->objectID
                 << " Type=" << agent[nearID]->memory.perceptedSignals[i]->objectType
                 << " Disp = " << agent[nearID]->memory.perceptedSignals[i]->signalDisplay
                 << " L = " << agent[nearID]->memory.perceptedSignals[i]->distToSL;
    }

    qDebug() << "Collision/Merging Point Info:";
    for(int i=0;i<agent[nearID]->memory.perceptedObjects.size();++i){
        if( agent[nearID]->memory.perceptedObjects[i]->isValidData == false ){
            continue;
        }
        qDebug() << "  OBJ:" << agent[nearID]->memory.perceptedObjects[i]->objectID
                 << " hasCP = " << agent[nearID]->memory.perceptedObjects[i]->hasCollisionPoint;
        if( agent[nearID]->memory.perceptedObjects[i]->hasCollisionPoint == true ){
            qDebug() << "    xCP = " << agent[nearID]->memory.perceptedObjects[i]->xCP
                     << " yCP = " << agent[nearID]->memory.perceptedObjects[i]->yCP;
            qDebug() << "    myDist = " << agent[nearID]->memory.perceptedObjects[i]->myDistanceToCP
                     << " myTime = " << agent[nearID]->memory.perceptedObjects[i]->myTimeToCP;
            qDebug() << "    objDist = " << agent[nearID]->memory.perceptedObjects[i]->objectDistanceToCP
                     << " objTime = " << agent[nearID]->memory.perceptedObjects[i]->objectTimeToCP;
        }
    }

    qDebug() << "Hazard Identification & Risk Evaluation:";
    qDebug() << "  oncomingWaitPathList = " << agent[nearID]->memory.oncomingWaitPathList;
    qDebug() << "  oncomingWaitCPList = " << agent[nearID]->memory.oncomingWaitCPList;
    qDebug() << "  nextTurnNodeOncomingDir = " << agent[nearID]->memory.nextTurnNodeOncomingDir;
    qDebug() << "  nearOncomingWaitPathInfo = " << agent[nearID]->memory.nearOncomingWaitPathInfo;
    qDebug() << "  nearOncomingWaitCPInfo = " << agent[nearID]->memory.nearOncomingWaitCPInfo;
    qDebug() << "  farOncomingWaitPathInfo = " << agent[nearID]->memory.farOncomingWaitPathInfo;
    qDebug() << "  farOncomingWaitCPInfo = " << agent[nearID]->memory.farOncomingWaitCPInfo;
    qDebug() << "  distToNearOncomingCP = " << agent[nearID]->memory.distToNearOncomingCP;
    qDebug() << "  distToFatOncomingCP = " << agent[nearID]->memory.distToFatOncomingCP;
    qDebug() << "  shouldWaitOverCrossPoint = " << agent[nearID]->memory.shouldWaitOverCrossPoint;
    qDebug() << "  distToNearestCP = " << agent[nearID]->memory.distToNearestCP;
    qDebug() << "  shouldStopAtSignalSL = " << agent[nearID]->memory.shouldStopAtSignalSL;
    qDebug() << "  shouldYeild = " << agent[nearID]->memory.shouldYeild;
    qDebug() << "  distToYeildStopLine = " << agent[nearID]->memory.distToYeildStopLine;
    qDebug() << "  leftCrossIsClear = " << agent[nearID]->memory.leftCrossIsClear;
    qDebug() << "  leftCrossCheckCount = " << agent[nearID]->memory.leftCrossCheckCount;
    qDebug() << "  rightCrossIsClear = " << agent[nearID]->memory.rightCrossIsClear;
    qDebug() << "  rightCrossCheckCount = " << agent[nearID]->memory.rightCrossCheckCount;
    qDebug() << "  safetyConfimed = " << agent[nearID]->memory.safetyConfimed;


    qDebug() << "Control Info:";
    qDebug() << "  controlMode = " << agent[nearID]->memory.controlMode;
    qDebug() << "  accel = " << agent[nearID]->memory.accel;
    qDebug() << "  brake = " << agent[nearID]->memory.brake;

    qDebug() << "  targetSpeed = " << agent[nearID]->memory.targetSpeed;
    qDebug() << "  actualTargetSpeed = " << agent[nearID]->memory.actualTargetSpeed;
    qDebug() << "  distanceAdjustLowSpeed = " << agent[nearID]->memory.distanceAdjustLowSpeed;
    qDebug() << "  axSpeedControl = " << agent[nearID]->memory.axSpeedControl;

    qDebug() << "  precedingVehicleID = " << agent[nearID]->memory.precedingVehicleID;
    qDebug() << "  distanceToPrecedingVehicle = " << agent[nearID]->memory.distanceToPrecedingVehicle;
    qDebug() << "  speedPrecedingVehicle = " << agent[nearID]->memory.speedPrecedingVehicle;
    qDebug() << "  doHeadwayDistanceControl = " << agent[nearID]->memory.doHeadwayDistanceControl;
    qDebug() << "  targetHeadwayDistance = " << agent[nearID]->memory.targetHeadwayDistance;
    qDebug() << "  axHeadwayControl = " << agent[nearID]->memory.axHeadwayControl;

    qDebug() << "  doStopControl = " << agent[nearID]->memory.doStopControl;
    qDebug() << "  causeOfStopControl = " << agent[nearID]->memory.causeOfStopControl;
    qDebug() << "  speedZeroCount = " << agent[nearID]->memory.speedZeroCount;

    qDebug() << "  releaseStopCount = " << agent[nearID]->memory.releaseStopCount;
    qDebug() << "  axStopControl = " << agent[nearID]->memory.axStopControl;
    qDebug() << "  distanceToStopPoint = " << agent[nearID]->memory.distanceToStopPoint;
    qDebug() << "  distanceToZeroSpeed = " << agent[nearID]->memory.distanceToZeroSpeed;
    qDebug() << "  requiredDistToStopFromTargetSpeed = " << agent[nearID]->memory.requiredDistToStopFromTargetSpeed;

    qDebug() << "  lateralDeviationFromTargetPath = " << agent[nearID]->memory.lateralDeviationFromTargetPath;
    qDebug() << "  lateralDeviationFromTargetPathAtPreviewPoint = " << agent[nearID]->memory.lateralDeviationFromTargetPathAtPreviewPoint;
    qDebug() << "  lateralShiftTarget = " << agent[nearID]->memory.lateralShiftTarget;
    qDebug() << "  avoidTarget = " << agent[nearID]->memory.avoidTarget;
    qDebug() << "  steer = " << agent[nearID]->memory.steer;


    qDebug() << "Guidance Info:";
    qDebug() << "  targetPathList = " << agent[nearID]->memory.targetPathList;
    qDebug() << "  currentPath = " << agent[nearID]->memory.currentTargetPath;
    qDebug() << "  myNodeList = " << agent[nearID]->memory.myNodeList;
    qDebug() << "  myInDirList = " << agent[nearID]->memory.myInDirList;
    qDebug() << "  myOutDirList = " << agent[nearID]->memory.myOutDirList;
    qDebug() << "  myTurnDirectionList = " << agent[nearID]->memory.myTurnDirectionList;

    qDebug() << "  currentTargetNode = " << agent[nearID]->memory.currentTargetNode;
    qDebug() << "  currentTargetNodeIndexInNodeList = " << agent[nearID]->memory.currentTargetNodeIndexInNodeList;
    qDebug() << "  nextTurnDirection = " << agent[nearID]->memory.nextTurnDirection;
    qDebug() << "  nextTurnNode = " << agent[nearID]->memory.nextTurnNode;
    qDebug() << "  nextTurnNodeIndexInNodeList = " << agent[nearID]->memory.nextTurnNodeIndexInNodeList;
    qDebug() << "  distanceToTurnNodeWPIn = " << agent[nearID]->memory.distanceToTurnNodeWPIn;
    qDebug() << "  distanceToNodeWPIn = " << agent[nearID]->memory.distanceToNodeWPIn;

    qDebug() << "Parameters:";
    qDebug() << "  accelControlGain = " << agent[nearID]->param.accelControlGain;
    qDebug() << "  deadZoneSpeedControl = " << agent[nearID]->param.deadZoneSpeedControl;
    qDebug() << "  maxDeceleration = " << agent[nearID]->param.maxDeceleration;
    qDebug() << "  accelOffDeceleration = " << agent[nearID]->param.accelOffDeceleration;
    qDebug() << "  steeringControlGain = " << agent[nearID]->param.steeringControlGain;
    qDebug() << "  latAccelAtTurn = " << agent[nearID]->param.latAccelAtTurn;
    qDebug() << "  headwayTime = " << agent[nearID]->param.headwayTime;
    qDebug() << "  headwayControlGain = " << agent[nearID]->param.headwayControlGain;
    qDebug() << "  minimumPerceptibleDecelerationOfPreceding = " << agent[nearID]->param.minimumPerceptibleDecelerationOfPreceding;
    qDebug() << "  minimumHeadwayDistanceAtStop = " << agent[nearID]->param.minimumHeadwayDistanceAtStop;
    qDebug() << "  minimumDistanceToStopLine = " << agent[nearID]->param.minimumDistanceToStopLine;
    qDebug() << "  visibleDistance = " << agent[nearID]->param.visibleDistance;
    qDebug() << "  startRelay = " << agent[nearID]->param.startRelay;
    qDebug() << "  crossTimeSafetyMargin = " << agent[nearID]->param.crossTimeSafetyMargin;
    qDebug() << "  crossWaitPositionSafeyMargin = " << agent[nearID]->param.crossWaitPositionSafeyMargin;
    qDebug() << "  safetyConfirmTime = " << agent[nearID]->param.safetyConfirmTime;

    qDebug() << "Debug Info:";
    QStringList divStr = agent[nearID]->strForDebug.split("\n");
    for(int i=0;i<divStr.size();++i){
        qDebug() << "  " << QString(divStr[i]);
    }
}
