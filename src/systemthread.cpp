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
int g_SInterEmbedFlag = 0;
int g_simulationCount = 0;

extern QMutex *mutex_sync;
extern QWaitCondition *cond_sync;

int allowDataGetForDS = 0;

extern QMutex *mutex_log;
extern QWaitCondition *cond_log;

extern int logThreadOpeFlag;

extern int maxWorkerThread;
extern QList<QWaitCondition *> workerCond;
extern QMutex *workerMutex;
extern QList<int> statusWorkerThread;
extern bool allWorkerThreadFinshed;
extern QWaitCondition *condSimMain;

bool invalidXPivotData = false;


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

    pWorkingMode = new int;
    *pWorkingMode = 0;

    tmpStopHour   = -1;
    tmpStopMin    = -1;
    tmpStopSecond = -1;

    restartFile = QString();

    mutexDSStateWrite = new QMutex();
    DSMoveTarget = -1;

    sendDataBuf = NULL;
    sendDataMaxSize = 0;

    expIDText = QString();

    nSIObject = 0;
    asv = NULL;
    sov = NULL;

    UE4ObjectIDUseFlag = NULL;
    noClearByWarp = 0;
}

void SystemThread::Stop()
{
    stopped = true;
}

void SystemThread::run()
{
    qDebug() << "[SystemThread]Start thread";

    g_DSTimingFlag = 0;

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
        if( expIDText.isNull() == false ){
            filename += expIDText + QString("_");
            logThread->SetExpID( expIDText );
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


    if( DSMode == false ){

        qDebug() << "----- Restart File supplied: ";
        qDebug() << "[File]" << restartFile;

        SetRestartData();
    }


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


    if( DSMode == true ){

        nSIObject = udpThread->GetSizeSInterfaceObjIDs();
        asv = new struct AgentState [nSIObject];
        sov = new struct SInterObjInfo [nSIObject];
        if( !asv || !sov ){
            qDebug() << "!!!";
            qDebug() << "!!!----- Failed to allocate AgentState and SInterObjInfo";
            qDebug() << "!!!";
        }
        else{
            for(int i=0;i<nSIObject;++i){
                sov[i].objID = -1;
                sov[i].objType = -1;
            }
        }

    }


    stopped = false;
    while( stopped == false ){

        if( simulationState == SIMULATION_STATE::PAUSED ){
            continue;
        }


        //
        // if DS mode, synchronize execution

        //
        //  Start agent calculation
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


#ifdef _SHOW_AGENT_NUM_APPEAR
        if( simManage->GetSimulationTimeSecondAsInt() % 10 == 0 ){
            int nAppear = 0;
            for(int i=0;i<maxAgent;++i){
                if( agent[i]->agentStatus == 0 ){
                    continue;
                }
                nAppear++;
            }
            qDebug() << "Number of Agent = " << nAppear;
        }
#endif


        if( tmpStopHour >= 0 && tmpStopMin >= 0 && tmpStopSecond >= 0 ){

            float tmpStopSimTimeFVal = tmpStopHour * 3600 + tmpStopMin * 60 + tmpStopSecond;
            if( simTimeFVal >= tmpStopSimTimeFVal ){
                tmpStopHour   = -1;
                tmpStopMin    = -1;
                tmpStopSecond = -1;
                emit TmpStopSimulation();
            }

        }


        //qDebug() << simTime;


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



#ifdef _PERFORMANCE_CHECK
        QueryPerformanceCounter(&start);
#endif

        //
        // Raise Event
        simManage->RaiseEvent( agent, maxAgent, road );


        if( simManage->changeTSDisplayInfo.size() > 0 ){
            for(int i=0;i<simManage->changeTSDisplayInfo.size();++i){

                int TSID = simManage->changeTSDisplayInfo[i].x();
                if( TSID >= 0 ){
                    int SIdx = simManage->changeTSDisplayInfo[i].y();
                    if( SIdx >= 0 ){
                        int tsIdx = tsId2Index.indexOf( TSID );
                        int incNode = trafficSignal[tsIdx]->relatedNode;

                        int hastTime = trafficSignal[tsIdx]->GetTimeToDisplay( SIdx );
                        qDebug() << "hastTime = " << hastTime;

                        for(int j=0;j<numTrafficSignals;++j){
                            if( trafficSignal[j]->relatedNode != incNode ){
                                continue;
                            }
                            trafficSignal[j]->ProceedTime( hastTime );
                        }
                    }
                    else{
                        int tsIdx = tsId2Index.indexOf( TSID );
                        int incNode = trafficSignal[tsIdx]->relatedNode;

                        int hastTime = trafficSignal[tsIdx]->GetTimeToDisplay( SIdx );
                        qDebug() << "hastTime = " << hastTime;

                        for(int j=0;j<numTrafficSignals;++j){
                            if( trafficSignal[j]->relatedNode != incNode ){
                                continue;
                            }
                            trafficSignal[j]->SetDisplayOff();
                        }
                    }
                }
                else{
                    int SIdx = simManage->changeTSDisplayInfo[i].y();
                    if( SIdx < 0 ){
                        for(int j=0;j<numTrafficSignals;++j){
                            trafficSignal[j]->SetDisplayOff();
                        }
                    }
                }
            }
            simManage->changeTSDisplayInfo.clear();
        }

#ifdef _PERFORMANCE_CHECK
        QueryPerformanceCounter(&end);
        calTime[1] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
        calCount[1]++;
#endif



        //
        // Update Traffic Signal Display
        //

#ifdef _PERFORMANCE_CHECK
        QueryPerformanceCounter(&start);
#endif
        for(int i=0;i<numTrafficSignals;++i){
            if( trafficSignal[i]->isSensorType == 2 &&
                    trafficSignal[i]->sensorState == 0 ){  // Controlled by Sensor and for yeild road

                if( trafficSignal[i]->setSensorPos == false ){
                    int rNDID = trafficSignal[i]->relatedNode;
                    int rNdIdx = road->nodeId2Index.indexOf( rNDID );
                    if( rNdIdx >= 0 ){
                        for(int j=0;j<road->nodes[rNdIdx]->stopPoints.size();++j){
                            if( trafficSignal[i]->controlDirection == road->nodes[rNdIdx]->stopPoints[j]->relatedDir ){
                                trafficSignal[i]->sensorX = road->nodes[rNdIdx]->stopPoints[j]->pos.x();
                                trafficSignal[i]->sensorY = road->nodes[rNdIdx]->stopPoints[j]->pos.y();
                                trafficSignal[i]->setSensorPos = true;

                                qDebug() << "Sensor Pos = (" << trafficSignal[i]->sensorX << "," << trafficSignal[i]->sensorY << ")";
                            }
                        }
                    }
                }

                for(int j=0;j<maxAgent;++j){
                    if( agent[j]->agentStatus != 1 ){
                        continue;
                    }
                    if( agent[j]->state.V > 0.1 ){
                        continue;
                    }

                    //qDebug() << "Check V = " << agent[j]->ID;

                    float dx = agent[j]->state.x - trafficSignal[i]->sensorX;
                    if( fabs(dx) > 10.0 + agent[j]->vHalfLength ){
                        continue;
                    }
                    float dy = agent[j]->state.y - trafficSignal[i]->sensorY;
                    if( fabs(dy) > 10.0 + agent[j]->vHalfLength ){
                        continue;
                    }

                    trafficSignal[i]->sensorState = 1;
                    qDebug() << "Found Stopping Vehicle at Sensor: TS = " << trafficSignal[i]->id << " , V = " << agent[j]->ID;
                    break;
                }
            }
        }

        QList<int> sensorTSNodeChangeList;
        for(int i=0;i<numTrafficSignals;++i){
            trafficSignal[i]->CheckDisplayInfoUpdate( simTimeFVal,
                                                      simManage->GetCalculationInterval() );

            if( trafficSignal[i]->sensorState == 2 ){
                if( sensorTSNodeChangeList.indexOf( trafficSignal[i]->relatedNode ) < 0 ){
                    sensorTSNodeChangeList.append( trafficSignal[i]->relatedNode );
                }
            }
        }

        for(int i=0;i<sensorTSNodeChangeList.size();++i){
            for(int j=0;j<numTrafficSignals;++j){
                if( trafficSignal[j]->relatedNode == sensorTSNodeChangeList[i] && trafficSignal[j]->sensorState != 2 ){
                    trafficSignal[j]->sensorState = 2;
                }
            }
        }


#ifdef _PERFORMANCE_CHECK
        QueryPerformanceCounter(&end);
        calTime[2] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
        calCount[2]++;
#endif



        // Set Agent IDs to Working Threads
        QList<int> evalAgentQueue;
        QList<int> DSVehiclesQueue;
        for(int i=0;i<maxAgent;++i){
            if( agent[i]->agentStatus != 1 ){
                continue;
            }
            if( agent[i]->isSInterfaceObject == true ){
                DSVehiclesQueue.append( agent[i]->ID );
                continue;
            }
            if( agent[i]->isBehaviorEmbeded == true ){
                continue;
            }
            evalAgentQueue.append( agent[i]->ID );
        }

        for(int i=0;i<maxWorkerThread;++i){
            evalIDs[i]->clear();
            for(int j=i;j<evalAgentQueue.size();j+=maxWorkerThread){
                evalIDs[i]->append( evalAgentQueue[j] );
            }
        }


        // Backup agent's memory for referencing from other agent
        {
            *pWorkingMode = 1;


            // Wake up Working Threads

            allWorkerThreadFinshed = false;
            for(int i=0;i<maxWorkerThread;++i){
                statusWorkerThread[i] = 1;
            }

            for(int i=0;i<maxWorkerThread;++i){
                workerMutex->lock();
                workerCond[i]->wakeAll();
                workerMutex->unlock();
            }

            // Wait until all threads done their work
            workerMutex->lock();
            if( allWorkerThreadFinshed == false ){
                condSimMain->wait(workerMutex);
            }
            workerMutex->unlock();
        }


        // Agent's main logic
        {
            *pWorkingMode = 2;

            // Wake up Working Threads
            allWorkerThreadFinshed = false;
            for(int i=0;i<maxWorkerThread;++i){
                statusWorkerThread[i] = 1;
            }

            for(int i=0;i<maxWorkerThread;++i){
                workerMutex->lock();
                workerCond[i]->wakeAll();
                workerMutex->unlock();
            }

            // Wait until all threads done their work
            workerMutex->lock();
            if( allWorkerThreadFinshed == false ){
                condSimMain->wait(workerMutex);
            }
            workerMutex->unlock();
        }


        //  Update agent state
        {
            *pWorkingMode = 3;

            // Wake up Working Threads
            allWorkerThreadFinshed = false;
            for(int i=0;i<maxWorkerThread;++i){
                statusWorkerThread[i] = 1;
            }

            for(int i=0;i<maxWorkerThread;++i){
                workerMutex->lock();
                workerCond[i]->wakeAll();
                workerMutex->unlock();
            }

            // Wait until all threads done their work
            workerMutex->lock();
            if( allWorkerThreadFinshed == false ){
                condSimMain->wait(workerMutex);
            }
            workerMutex->unlock();
        }



        //
        // Update simulation time
        simManage->UpdateSimulationTime();

        emit UpdateSimulationTimeDisplay(simTime);


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



        //  Check vehicles that reach to end of route
        {
            *pWorkingMode = 4;

            // Wake up Working Threads
            allWorkerThreadFinshed = false;
            for(int i=0;i<maxWorkerThread;++i){
                statusWorkerThread[i] = 1;
            }

            for(int i=0;i<maxWorkerThread;++i){
                workerMutex->lock();
                workerCond[i]->wakeAll();
                workerMutex->unlock();
            }

            // Wait until all threads done their work
            workerMutex->lock();
            if( allWorkerThreadFinshed == false ){
                condSimMain->wait(workerMutex);
            }
            workerMutex->unlock();
        }


        if( DSMode == true ){
            for(int i=0;i<DSVehiclesQueue.size();++i){
                int id = DSVehiclesQueue[i];
                //mutexDSStateWrite->lock();
                agent[id]->BackupMemory();
                //mutexDSStateWrite->unlock();
            }
        }


#ifdef _PERFORMANCE_CHECK
            QueryPerformanceCounter(&start);
#endif

        simManage->DisappearAgents( agent, maxAgent );

#ifdef _PERFORMANCE_CHECK
        QueryPerformanceCounter(&end);
        calTime[3] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
        calCount[3]++;
#endif


        if( DSMode == true ){

            int maxAgentDataSend = udpThread->GetMaxAgentDataSend();
            int maxAgentDataSendToFE = udpThread->GetMaxAgentDataSendToFE();

            int maxTSDataSend = udpThread->GetMaxTSDataSend();

            if( sendDataBuf == NULL ){

                sendDataMaxSize = 125 * maxAgentDataSend + 13 * maxTSDataSend + 20 + 5;

                sendDataBuf = new char [sendDataMaxSize + 2000];
            }


            //
            //  Wait until all S-Interface Object data is recieved
            //
            mutex->lock();
            if( g_SInterEmbedFlag == 0 ){
                cond->wait(mutex);
            }
            g_SInterEmbedFlag = 0;
            mutex->unlock();


            //
            //  Embed S-Interface Object Data
            //
            for(int i=0;i<nSIObject;++i){
                char type = 'v';
                if( sov[i].objType >= 100 ){
                    type = 'p';
                }
                SetSInterObjData( type, sov[i].objID, &(asv[i]), &(sov[i]) );
            }


            //
            //  Send data to UE4
            //

            QList<int> SIObjIDList;

            for(int i=0;i<nSIObject;++i){

                int SIObjID = udpThread->GetSInterfaceObjID(i);
                SIObjIDList.append( SIObjID );

                int sendSize = SetSendData(maxAgentDataSend,maxTSDataSend,SIObjID);
//                qDebug() << "sendSize = " << sendSize;
                if( sendSize > 0 ){
                    udpThread->SendToUE4(SIObjID, sendDataBuf, sendSize );
                }
            }


            //
            //  Send data to FE
            //
            if( udpThread->GetHasFuncExtender() == true ){
                int sendSize = SetSendDataForFuncExtend(maxAgentDataSendToFE,maxTSDataSend,SIObjIDList);
                if( sendSize > 0 ){
                    udpThread->SendToFE( sendDataBuf, sendSize );
                }
            }


            //
            //  Send syncronization signal to Score
            //
            udpThread->SendToSCore();
        }


#ifdef _PERFORMANCE_CHECK
        if( calCount[0] >= 5000 ){
            for(int i=0;i<4;i++){
                calTime[i] /= calCount[i];
                qDebug() << "[System] Mean Time[" << i << "] = " << calTime[i];
                calCount[i] = 0;
            }
        }
#endif

    }


    delete [] sendDataBuf;


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

        qDebug() << "Start UDPThread.";

        udpThread = new UDPThread();

        connect( udpThread, SIGNAL(SimulationStart()), this, SLOT(SimulationStart()) );
        connect( udpThread, SIGNAL(SimulationStop()), this, SLOT(SimulationStop()) );
        connect( udpThread, SIGNAL(ExitProgram()), this, SLOT(wrapExitProgram()));
        connect( udpThread, SIGNAL(RedrawRequest()), this, SLOT(wrapRedrawRequest()));
        connect( udpThread, SIGNAL(ReceiveContinueCommand()), this, SLOT(quit()) );
        connect( udpThread, SIGNAL(SetSimulationFrequency(int)), this, SLOT(SetSimulationFrequency(int)) );
        connect( udpThread, SIGNAL(ReceiveTireHeight(int,int,float,float,float,float)), this, SLOT(SetTireHeight(int,int,float,float,float,float)) );        
        connect( udpThread, SIGNAL(ReceiveSInterInitState(char,int,float,float,float,float,float)), this, SLOT(SetSInterInitState(char,int,float,float,float,float,float)) );
        connect( udpThread, SIGNAL(ReceiveSInterObjData(char,int,AgentState*,struct SInterObjInfo *)), this, SLOT(SetSInterObjDataToBuffer(char,int,AgentState*,struct SInterObjInfo *)) );
        connect( udpThread, SIGNAL(ReceiveTSColorChange(int,int,float)),this,SLOT(ForceChangeTSColor(int,int,float)));
        connect( udpThread, SIGNAL(WarpVehicle(int,float,float,float)),this,SLOT(WarpVehicle(int,float,float,float)) );
        connect( udpThread, SIGNAL(WarpVehicleAdjustPosToLane(int,float,float,float)),this,SLOT(WarpVehicleAdjustPosToLane(int,float,float,float)) );
        connect( udpThread, SIGNAL(DisposeAgent(int)),this,SLOT(DisposeAgent(int)) );
        connect( udpThread, SIGNAL(AppearAgent(int)),this,SLOT(AppearAgent(int)) );
        connect( udpThread, SIGNAL(EmbedBehavior(int,float*,int)),this,SLOT(EmbedAgentBehavior(int,float*,int)) );
        connect( udpThread, SIGNAL(ChangeReferenceSpeed(int,float)),this,SLOT(ChangeReferenceSpeed(int,float)) );
        connect( udpThread, SIGNAL(CopyPathData(int,int)),this,SLOT(CopyPathData(int,int)) );
        connect( udpThread, SIGNAL(RequestLaneChange(int,int,int)),this,SLOT(RequestLaneChange(int,int,int)) );
        connect( udpThread, SIGNAL(RequestAssignedLaneChange(int,int,int,float)),this,SLOT(RequestAssignedLaneChange(int,int,int,float)) );
        connect( udpThread, SIGNAL(ChangeControlModeStopAt(int,float,float)),this,SLOT(ChangeControlModeStopAt(int,float,float)) );
        connect( udpThread, SIGNAL(ChangeControlModeHeadwayControl(int,float,float,float,float,int,bool)),this,SLOT(ChangeControlModeHeadwayControl(int,float,float,float,float,int,bool)) );
        connect( udpThread, SIGNAL(ChangeControlModeAgent(int)),this,SLOT(ChangeControlModeAgent(int)) );
        connect( udpThread, SIGNAL(ChangeControlModeIntersectionTurn(int,float,float)),this,SLOT(ChangeControlModeIntersectionTurn(int,float,float)) );
        connect( udpThread, SIGNAL(CopyScenarioData(int,int)),this,SLOT(CopyScenarioData(int,int)) );
        connect( udpThread, SIGNAL(ChangeVehicleWinkers(int,int)),this,SLOT(ChangeVehicleWinkers(int,int)) );
        connect( udpThread, SIGNAL(SetLateralShift(int,float)),this,SLOT(SetLateralShift(int,float)) );
        connect( udpThread, SIGNAL(SetLateralGainMultiplier(int,float)),this,SLOT(SetLateralGainMultiplier(int,float)) );
        connect( udpThread, SIGNAL(SetAgentGenerationNotAllowFlag(int,bool)),this,SLOT(SetAgentGenerationNotAllowFlag(int,bool)) );
        connect( udpThread, SIGNAL(ForceChangeSpeed(int,float)),this,SLOT(ForceChangeSpeed(int,float)) );
        connect( udpThread, SIGNAL(SetRestartData()),this,SLOT(SetRestartData()) );
        connect( udpThread, SIGNAL(ChangeSpeedProfile(int,QList<float>,QList<float>)),this,SLOT(ChangeSpeedProfile(int,QList<float>,QList<float>)) );
        connect( udpThread, SIGNAL(DirectAssignAcceleration(int,float)),this,SLOT(DirectAssignAcceleration(int,float)) );
        connect( udpThread, SIGNAL(SetTargetSpeedMode(int,int)),this,SLOT(SetTargetSpeedMode(int,int)) );
        connect( udpThread, SIGNAL(ChangeRoadLaneSpeedLimit(QList<int>, QList<float>)),this,SLOT(ChangeRoadLaneSpeedLimit(QList<int>, QList<float>)) );
        connect( udpThread, SIGNAL(ChangeVelocityControlParameters(float,float,float,float,float,int,QList<int>)),this,SLOT(ChangeVelocityControlParameters(float,float,float,float,float,int,QList<int>)) );
        connect( udpThread, SIGNAL(ChangeLaneAssignedVelocityControlParameters(float,float,float,float,float,QList<int>)),this,SLOT(ChangeLaneAssignedVelocityControlParameters(float,float,float,float,float,QList<int>)) );
        connect( udpThread, SIGNAL(ChangeMoveSpeedPedestrian(QList<float>)),this,SLOT(ChangeMoveSpeedPedestrian(QList<float>)) );
        connect( udpThread, SIGNAL(OverwriteAgentParameter(int,int,float)),this,SLOT(OverwriteAgentParameter(int,int,float)) );
        connect( udpThread, SIGNAL(ChangeOptionalImageParams(QList<float>)),this,SLOT(ChangeOptionalImageParams(QList<float>)) );
        connect( udpThread, SIGNAL(SetEventTriggerByFuncExtend(int,int,int,int)),this,SLOT(SetEventTriggerByFuncExtend(int,int,int,int)) );
        connect( udpThread, SIGNAL(SetBrakeLampOverride(int,int)),this,SLOT(SetBrakeLampOverride(int,int)) );
        connect( udpThread, SIGNAL(SetScenarioVehicleInitStates(QList<int>,QList<float>,QList<float>,QList<float>,QList<float>,QList<float>)),this,SLOT(SetScenarioVehicleInitStates(QList<int>,QList<float>,QList<float>,QList<float>,QList<float>,QList<float>)) );
        connect( udpThread, SIGNAL(SetAgentGenerationNotAllowFlagForNodes(QList<int>, bool)),this,SLOT(SetAgentGenerationNotAllowFlagForNodes(QList<int>, bool)) );
        connect( udpThread, SIGNAL(AppearStoppingVehicle(int,float,float,float)),this,SLOT(AppearStoppingVehicle(int,float,float,float)) );
        connect( udpThread, SIGNAL(SetAgentExternalControlParams(int,float,float)),this,SLOT(SetAgentExternalControlParams(int,float,float)) );
        connect( udpThread, SIGNAL(ChangePedestPathForScenarioPedestrian(int,QList<float>,QList<float>)),this,SLOT(ChangePedestPathForScenarioPedestrian(int,QList<float>,QList<float>)) );


        if( logThread ){
            connect( udpThread, SIGNAL(FEDataReceived(int,QString)), logThread, SLOT(SetFuncExtenderLogData(int,QString)) );
        }

        udpThread->SetMaxAgentNumber( maxAgent );
        udpThread->SetNumberTrafficSignal( trafficSignal.size() );

        simManage->udpthread = udpThread;
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


void SystemThread::wrapRedrawRequest()
{
    emit RedrawRequest();
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


    QString tmpfilename = scenarioFile;

    QStringList divFileName = tmpfilename.replace("\\","/").split("/");
    QString pureFileName = QString(divFileName.last());
    QString scenarioFolder = tmpfilename.remove( pureFileName );

    qDebug() << "pureFileName = " << pureFileName;
    qDebug() << "scenarioFolder = " << scenarioFolder;



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

            QString reconstFilename = roadDataFile;

            // Check if the path is absolute or relative
            if( roadDataFile.contains(":") == false ){

                reconstFilename = scenarioFolder + roadDataFile;

                qDebug() << "Reconstructed road filename = " << reconstFilename;
            }

            roadDataFile = reconstFilename;

        }
        else if( tag == QString("Signal Data File") ){
            signalDataFile = QString( divLine[1] ).trimmed();

            QString reconstFilename = signalDataFile;

            // Check if the path is absolute or relative
            if( signalDataFile.contains(":") == false ){

                reconstFilename = scenarioFolder + signalDataFile;

                qDebug() << "Reconstructed signal filename = " << reconstFilename;
            }

            signalDataFile = reconstFilename;
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
            trigger->byExternalTriggerFlag = false;
            trigger->extTriggerFlagState = false;
            trigger->byKeyTriggerFlag = false;
            trigger->combination = -1;
        }
        else if( tag == QString("End Target") ){
            trigObj = new struct ObjectTriggerData;
            trigObj->targetObjectID = QString( divLine[1] ).trimmed().toInt();
        }
        else if( tag == QString("End Trigger Position") ){
            if( divLine.size() >= 4 ){
                trigObj->x         = QString( divLine[1] ).trimmed().toFloat();
                trigObj->y         = QString( divLine[2] ).trimmed().toFloat();
                trigObj->direction = QString( divLine[3] ).trimmed().toFloat() * (0.017452);
                trigObj->cosDirect = cos( trigObj->direction );
                trigObj->sinDirect = sin( trigObj->direction );
            }
            if( divLine.size() >= 5 ){
                trigObj->speed     = QString( divLine[4] ).trimmed().toFloat() / 3.6;
            }
            trigger->objectTigger.append( trigObj );
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
        else if( tag == QString("Pedestrian Event Type") ){

            scenarioEvent = new ScenarioEvents;

            scenarioEvent->eventID = eventNo;
            eventNo++;

            scenarioEvent->eventType = SCENARIO_EVENT_TYPE::OBJECT_EVENT;
            scenarioEvent->targetObjectID = scenarioItem->objectID;

            scenarioEvent->eventState = 0;
            scenarioEvent->eventTimeCount = 0;
            scenarioEvent->eventTimeCount_sub = 0;
            scenarioEvent->repeatByFE = false;

            scenarioEvent->routeType = -1;
            scenarioEvent->ndRoute = NULL;

            // eventKind = 0 :  Appear
            //             1 :  Control
            //             2 :  Send UDP
            //             3 :  Disappear
            int ekval = QString(divLine[1]).trimmed().toInt();
            if( ekval == 0 ){
                scenarioEvent->eventKind = OBJECT_SCENARIO_ACTION_KIND::APPEAR_PEDESTRIAN;
            }
            else if( ekval == 1 ){
                scenarioEvent->eventKind = OBJECT_SCENARIO_ACTION_KIND::CONTROL_PEDESTRIAN;
            }
            else if( ekval == 2 ){
                scenarioEvent->eventKind = OBJECT_SCENARIO_ACTION_KIND::SEND_UDP_OBJECT;
            }
            else if( ekval == 3 ){
                scenarioEvent->eventKind = OBJECT_SCENARIO_ACTION_KIND::DISAPPEAR;
            }

            trigger = new struct ScenarioTriggers;
            trigger->mode = 0;

            trigger->combination = 0;

            trigger->byKeyTriggerFlag = false;
            trigger->byExternalTriggerFlag = false;
            trigger->extTriggerFlagState = false;
            trigger->func_keys = 0;

            trigger->timeTrigger = new struct SimulationTime;
            trigger->timeTrigger->dt = 0.0;
            trigger->timeTrigger->day = 0;
            trigger->timeTrigger->min = 0;
            trigger->timeTrigger->sec = 0;
            trigger->timeTrigger->hour = 0;
            trigger->timeTrigger->msec = 0.0;
            trigger->timeTrigger->exe_freq = 0;
            trigger->timeTrigger->msec_count = 0;

            trigger->timeTriggerInSec = 0.0;
            trigger->ANDCondition = false;
            trigger->objectTigger.clear();

            scenarioEvent->eventTrigger = trigger;

            simManage->SetScenarioEvent( currentScenarioID, scenarioEvent );
        }
        else if( tag == QString("Pedestrian Event Trigger External") ){

            int extTrig = QString(divLine[1]).trimmed().toInt();
            if( extTrig >= 1 ){
                trigger->byKeyTriggerFlag = true;
                trigger->func_keys = extTrig;
            }
            if( divLine.size() >= 3 ){
                trigger->byExternalTriggerFlag = ( QString(divLine[2]).trimmed().toInt() == 1 ? true : false );
                trigger->extTriggerFlagState = false;

                if( trigger->byExternalTriggerFlag == true ){
                    trigObj = new struct ObjectTriggerData;
                    trigObj->triggerType = TRIGGER_TYPE::BY_FUNCTION_EXTENDER;
                    trigObj->isTriggered = false;

                    trigger->objectTigger.append( trigObj );
                }
            }
        }
        else if( tag == QString("Pedestrian Event Trigger Internal") ){

            if( divLine.size() == 2 ){

                if( trigger->mode != TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
                    trigger->mode = QString(divLine[1]).trimmed().toInt();
                }
            }
            else if( divLine.size() >= 7 ){

                trigger->combination = QString(divLine[1]).trimmed().toInt();

                for(int i=2;i<divLine.size();++i){

                    if( QString(divLine[i]).trimmed().toInt() == 1 ){

                        trigObj = new struct ObjectTriggerData;

                        if( i == 2 ){
                            trigObj->triggerType = TRIGGER_TYPE::AT_ONCE;
                        }
                        else if( i == 3 ){
                            trigObj->triggerType = TRIGGER_TYPE::TIME_TRIGGER;
                        }
                        else if( i == 4 ){
                            trigObj->triggerType = TRIGGER_TYPE::POSITION_TRIGGER;
                        }
                        else if( i == 5 ){
                            trigObj->triggerType = TRIGGER_TYPE::VELOCITY_TRIGGER;
                        }
                        else if( i == 6 ){
                            trigObj->triggerType = TRIGGER_TYPE::TTC_TRIGGER;
                        }
                        else if( i == 7 ){
                            trigObj->triggerType = TRIGGER_TYPE::BY_FUNCTION_EXTENDER;
                        }

                        trigObj->isTriggered = false;

                        trigger->objectTigger.append( trigObj );
                    }
                }
            }
        }
        else if( tag == QString("Pedestrian Event Trigger Position") ){

            if( trigger->combination < 0 ){

                trigObj = new struct ObjectTriggerData;
                trigObj->targetObjectID = -1;
                trigObj->targetEventID  = -1;
                trigger->objectTigger.append( trigObj );
                if( divLine.size() >= 4 ){
                    trigObj->x         = QString(divLine[1]).trimmed().toFloat();
                    trigObj->y         = QString(divLine[2]).trimmed().toFloat();
                    trigObj->direction = QString(divLine[3]).trimmed().toFloat() * (0.017452);
                    trigObj->cosDirect = cos( trigObj->direction );
                    trigObj->sinDirect = sin( trigObj->direction );
                    trigObj->widthHalf = 5.0;
                }
            }
            else if( trigger->combination >= 0 ){

                for(int i=0;i<trigger->objectTigger.size();++i){

                    if( trigger->objectTigger[i]->triggerType == TRIGGER_TYPE::POSITION_TRIGGER ){

                        if( divLine.size() >= 5 ){

                            trigger->objectTigger[i]->x         = QString(divLine[1]).trimmed().toFloat();
                            trigger->objectTigger[i]->y         = QString(divLine[2]).trimmed().toFloat();
                            trigger->objectTigger[i]->direction = QString(divLine[3]).trimmed().toFloat() * (0.017452);
                            trigger->objectTigger[i]->cosDirect = cos( trigger->objectTigger[i]->direction );
                            trigger->objectTigger[i]->sinDirect = sin( trigger->objectTigger[i]->direction );
                            trigger->objectTigger[i]->targetObjectID = QString(divLine[4]).trimmed().toInt();

                            trigger->objectTigger[i]->widthHalf = 5.0;
                            if( divLine.size() >= 6 ){
                                trigger->objectTigger[i]->widthHalf = QString(divLine[5]).trimmed().toFloat();
                            }

                            trigger->objectTigger[i]->passCheckFlag = 0;
                        }
                        break;
                    }
                }
            }
        }
        else if( tag == QString("Pedestrian Event Trigger Speed") ){

            if( trigger->combination < 0 ){

                trigObj->speed = QString(divLine[1]).trimmed().toFloat() / 3.6;

            }
            else if( trigger->combination >= 0 ){

                for(int i=0;i<trigger->objectTigger.size();++i){

                    if( trigger->objectTigger[i]->triggerType == TRIGGER_TYPE::VELOCITY_TRIGGER ){

                        if( divLine.size() >= 4 ){

                            trigger->objectTigger[i]->speed = QString(divLine[1]).trimmed().toFloat() / 3.6;
                            // triggerParam = velocity trigger lower or higher
                            trigger->objectTigger[i]->triggerParam = QString(divLine[2]).trimmed().toInt();
                            // triggerParam2 = velocity trigger target object ID
                            trigger->objectTigger[i]->targetObjectID = QString(divLine[3]).trimmed().toInt();

                        }
                        break;
                    }
                }
            }
        }
        else if( tag == QString("Pedestrian Event Target ID") ){
            trigObj->targetObjectID = QString(divLine[1]).trimmed().toInt();
        }
        else if( tag == QString("Pedestrian Event Trigger TTC") ){

            if( trigger->combination >= 0 ){

                for(int i=0;i<trigger->objectTigger.size();++i){

                    if( trigger->objectTigger[i]->triggerType == TRIGGER_TYPE::TTC_TRIGGER ){

                        if( divLine.size() >= 7 ){

                            trigger->objectTigger[i]->TTC = QString(divLine[1]).trimmed().toFloat();
                            // triggerParam = TTC cal Type
                            trigger->objectTigger[i]->triggerParam = QString(divLine[2]).trimmed().toInt();
                            // triggerParams = Action Target Object ID
                            trigger->objectTigger[i]->targetObjectID = QString(divLine[3]).trimmed().toInt();
                            // triggerParams = TTC cal Object ID
                            trigger->objectTigger[i]->triggerParam2 = QString(divLine[4]).trimmed().toInt();
                            // TTC cal Pos X
                            trigger->objectTigger[i]->x = QString(divLine[5]).trimmed().toFloat();
                            // TTC cal Pos Y
                            trigger->objectTigger[i]->y = QString(divLine[6]).trimmed().toFloat();
                        }
                        else if( divLine.size() == 6 ){

                            trigger->objectTigger[i]->TTC = QString(divLine[1]).trimmed().toFloat();
                            // triggerParam = TTC cal Type
                            trigger->objectTigger[i]->triggerParam = QString(divLine[2]).trimmed().toInt();
                            // triggerParams = TTC cal Target Object ID
                            trigger->objectTigger[i]->targetObjectID = QString(divLine[3]).trimmed().toInt();
                            // TTC cal Pos X
                            trigger->objectTigger[i]->x = QString(divLine[4]).trimmed().toFloat();
                            // TTC cal Pos Y
                            trigger->objectTigger[i]->y = QString(divLine[5]).trimmed().toFloat();
                        }
                        break;
                    }
                }
            }
        }
        else if( tag == QString("Pedestrian Event Trigger Time") ){

            if( divLine.size() >= 4 ){

                int etmode = QString(divLine[1]).trimmed().toInt();
                float val1 = QString(divLine[2]).trimmed().toFloat();
                float val2 = QString(divLine[3]).trimmed().toFloat();

                if( trigger->combination < 0 ){

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
                else if( trigger->combination >= 0 ){

                    for(int i=0;i<trigger->objectTigger.size();++i){

                        if( trigger->objectTigger[i]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ){

                            trigger->objectTigger[i]->timeTriggerInSec = val1;
                            if( etmode == 1 ){
                                trigger->objectTigger[i]->timeFromAppear = true;
                            }
                            else{
                                trigger->objectTigger[i]->timeFromAppear = false;
                            }

                            break;
                        }
                    }
                }
            }
        }
        else if( tag == QString("Pedestrian Event Param Float") ){
            for(int i=1;i<divLine.size();++i){
                scenarioEvent->eventFloatData.append( QString(divLine[i]).trimmed().toFloat() );
            }
        }
        else if( tag == QString("Pedestrian Event Param Integer") ){
            for(int i=1;i<divLine.size();++i){
                scenarioEvent->eventIntData.append( QString(divLine[i]).trimmed().toInt() );
            }
        }
        else if( tag == QString("Pedestrian Event Param Boolean") ){
            for(int i=1;i<divLine.size();++i){
                scenarioEvent->eventBooleanData.append( QString(divLine[i]).trimmed().toInt() == 1 ? true : false );
            }
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
            trigger->byExternalTriggerFlag = false;
            trigger->extTriggerFlagState = false;
            trigger->byKeyTriggerFlag = false;
            trigger->combination = -1;
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
            trigger->byExternalTriggerFlag = false;
            trigger->extTriggerFlagState = false;
            trigger->byKeyTriggerFlag = false;
            trigger->combination = -1;
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
        else if( tag == QString("Duplicate Scenario Vehicle") ){

            int nDuplicate = QString( divLine[1] ).trimmed().toInt();
            int startID = QString( divLine[2] ).trimmed().toInt();
            int copyTargetID = QString( divLine[3] ).trimmed().toInt();

            qDebug() << "Duplicate Scenario Vehicle: nDuplicate = " << nDuplicate << " startID = " << startID << " copyTargetID = " << copyTargetID;

            struct ScenarioEvents *copyTarget = simManage->GetScenarioEvent( currentScenarioID, copyTargetID );

            if( copyTarget ){

                for(int i=0;i<nDuplicate;++i){

                    scenarioEvent = new ScenarioEvents;

                    scenarioEvent->eventID = eventNo;
                    eventNo++;

                    scenarioEvent->eventType = SCENARIO_EVENT_TYPE::OBJECT_EVENT;
                    scenarioEvent->targetObjectID = startID + i;

//                    qDebug() << "Set targetObjectID = " << scenarioEvent->targetObjectID;

                    scenarioEvent->eventState = 0;
                    scenarioEvent->eventTimeCount = 0;
                    scenarioEvent->eventTimeCount_sub = 0;
                    scenarioEvent->repeatByFE = copyTarget->repeatByFE;

                    for(int j=0;j<copyTarget->eventIntData.size();++j){
                        if( j == 1 ){
                            scenarioEvent->eventIntData.append( -1 );
                        }
                        else{
                            scenarioEvent->eventIntData.append( copyTarget->eventIntData[j] );
                        }
                    }

                    for(int j=0;j<copyTarget->eventFloatData.size();++j){
                        scenarioEvent->eventFloatData.append( copyTarget->eventFloatData[j] );
                    }

                    for(int j=0;j<copyTarget->eventBooleanData.size();++j){
                        scenarioEvent->eventBooleanData.append( copyTarget->eventBooleanData[j] );
                    }

                    scenarioEvent->routeType = copyTarget->routeType;

                    if( copyTarget->ndRoute ){

                        scenarioEvent->ndRoute = new struct ODRouteData;

                        scenarioEvent->ndRoute->originNode = copyTarget->ndRoute->originNode;
                        scenarioEvent->ndRoute->destinationNode = copyTarget->ndRoute->destinationNode;

                        for(int j=0;j<copyTarget->ndRoute->routeToDestination.size();++j){
                            struct RouteElem* re = new struct RouteElem;

                            re->node = copyTarget->ndRoute->routeToDestination[j]->node;
                            re->inDir = copyTarget->ndRoute->routeToDestination[j]->inDir;
                            re->outDir = copyTarget->ndRoute->routeToDestination[j]->outDir;

                            scenarioEvent->ndRoute->routeToDestination.append( re );
                        }

                        for(int j=0;j<copyTarget->ndRoute->laneListsToDestination.size();++j){
                            QList<int> laneList;
                            for(int k=0;k<copyTarget->ndRoute->laneListsToDestination[j].size();++k){
                                laneList.append( copyTarget->ndRoute->laneListsToDestination[j][k] );
                            }
                            scenarioEvent->ndRoute->laneListsToDestination.append(laneList);
                        }

                        for(int j=0;j<copyTarget->ndRoute->LCSupportLaneLists.size();++j){

                            struct RouteLaneData *rld = new struct RouteLaneData;

                            rld->startNode = copyTarget->ndRoute->LCSupportLaneLists[j]->startNode;
                            rld->goalNode = copyTarget->ndRoute->LCSupportLaneLists[j]->goalNode;
                            rld->sIndexInNodeList = copyTarget->ndRoute->LCSupportLaneLists[j]->sIndexInNodeList;
                            rld->gIndexInNodeList = copyTarget->ndRoute->LCSupportLaneLists[j]->gIndexInNodeList;
                            rld->LCDirect = copyTarget->ndRoute->LCSupportLaneLists[j]->LCDirect;
                            for(int k=0;k<copyTarget->ndRoute->LCSupportLaneLists[j]->laneList.size();++k){
                                QList<int> lcLL;
                                for(int l=0;l<copyTarget->ndRoute->LCSupportLaneLists[j]->laneList[k].size();++l){
                                    lcLL.append( copyTarget->ndRoute->LCSupportLaneLists[j]->laneList[k][l] );
                                }
                                rld->laneList.append( lcLL );
                            }

                            scenarioEvent->ndRoute->LCSupportLaneLists.append( rld );
                        }

                        for(int j=0;j<copyTarget->ndRoute->trafficVolumne.size();++j){
                            scenarioEvent->ndRoute->trafficVolumne.append( copyTarget->ndRoute->trafficVolumne[j] );
                        }
                        scenarioEvent->ndRoute->totalVolumne = copyTarget->ndRoute->totalVolumne;

                        for(int j=0;j<copyTarget->ndRoute->vehicleKindSelectProbability.size();++j){
                            scenarioEvent->ndRoute->vehicleKindSelectProbability.append( copyTarget->ndRoute->vehicleKindSelectProbability[j] );
                        }
                        scenarioEvent->ndRoute->meanArrivalTime = copyTarget->ndRoute->meanArrivalTime;
                        scenarioEvent->ndRoute->NextAppearTime = copyTarget->ndRoute->NextAppearTime;

                        for(int j=0;j<copyTarget->ndRoute->mergeLanesInfo.size();++j){
                            QList<QPoint> mli;
                            for(int k=0;k<copyTarget->ndRoute->mergeLanesInfo[j].size();++k){
                                mli.append( copyTarget->ndRoute->mergeLanesInfo[j][k] );
                            }
                            scenarioEvent->ndRoute->mergeLanesInfo.append( mli );
                        }
                        scenarioEvent->ndRoute->onlyForScenarioVehicle = copyTarget->ndRoute->onlyForScenarioVehicle;
                        scenarioEvent->ndRoute->relatedScenarioObjectID = scenarioEvent->targetObjectID;

//                        qDebug() << "Set route data ";

                    }

                    scenarioEvent->eventKind = copyTarget->eventKind;

                    if( copyTarget->eventTrigger ){

                        trigger = new struct ScenarioTriggers;

                        trigger->mode = copyTarget->eventTrigger->mode;

                        trigger->combination = copyTarget->eventTrigger->combination;

                        trigger->byKeyTriggerFlag = copyTarget->eventTrigger->byKeyTriggerFlag;
                        trigger->byExternalTriggerFlag = copyTarget->eventTrigger->byExternalTriggerFlag;
                        trigger->extTriggerFlagState = copyTarget->eventTrigger->extTriggerFlagState;
                        trigger->func_keys = copyTarget->eventTrigger->func_keys;

                        if( copyTarget->eventTrigger->timeTrigger ){
                            struct SimulationTime* tt = new struct SimulationTime;

                            tt->dt = copyTarget->eventTrigger->timeTrigger->dt;
                            tt->day = copyTarget->eventTrigger->timeTrigger->day;
                            tt->min = copyTarget->eventTrigger->timeTrigger->min;
                            tt->sec = copyTarget->eventTrigger->timeTrigger->sec;
                            tt->hour = copyTarget->eventTrigger->timeTrigger->hour;
                            tt->exe_freq = copyTarget->eventTrigger->timeTrigger->exe_freq;
                            tt->msec_count = copyTarget->eventTrigger->timeTrigger->msec_count;

                            trigger->timeTrigger = tt;
                        }
                        trigger->timeTriggerInSec = copyTarget->eventTrigger->timeTriggerInSec;

                        trigger->ANDCondition = copyTarget->eventTrigger->ANDCondition;

                        for(int j=0;j<copyTarget->eventTrigger->objectTigger.size();++j){

                            struct ObjectTriggerData* otd = new struct ObjectTriggerData;

                            otd->x = copyTarget->eventTrigger->objectTigger[j]->x;
                            otd->y = copyTarget->eventTrigger->objectTigger[j]->y;
                            otd->TTC = copyTarget->eventTrigger->objectTigger[j]->TTC;
                            otd->speed = copyTarget->eventTrigger->objectTigger[j]->speed;
                            otd->cosDirect = copyTarget->eventTrigger->objectTigger[j]->cosDirect;
                            otd->direction = copyTarget->eventTrigger->objectTigger[j]->direction;
                            otd->sinDirect = copyTarget->eventTrigger->objectTigger[j]->sinDirect;
                            otd->widthHalf = copyTarget->eventTrigger->objectTigger[j]->widthHalf;
                            otd->isTriggered = copyTarget->eventTrigger->objectTigger[j]->isTriggered;

                            otd->triggerType = TRIGGER_TYPE::DELAY_TRIGGER;

                            otd->triggerParam = copyTarget->eventTrigger->objectTigger[j]->triggerParam;
                            otd->triggerParam2 = copyTarget->eventTrigger->objectTigger[j]->triggerParam2;
                            otd->passCheckFlag = copyTarget->eventTrigger->objectTigger[j]->passCheckFlag;
                            otd->targetEventID = copyTarget->eventTrigger->objectTigger[j]->targetEventID;
                            otd->targetObjectID = copyTarget->eventTrigger->objectTigger[j]->targetObjectID;
                            otd->timeFromAppear = copyTarget->eventTrigger->objectTigger[j]->timeFromAppear;
                            otd->timeTriggerInSec = copyTarget->eventTrigger->objectTigger[j]->timeTriggerInSec;

                            trigger->objectTigger.append( otd );

//                            qDebug() << "Set objectTigger data ";
                        }

                        scenarioEvent->eventTrigger = trigger;

//                        qDebug() << "Set trigger data ";
                    }

                    for(int j=0;j<copyTarget->targetPathList.size();++j){
                        scenarioEvent->targetPathList.append( copyTarget->targetPathList[j] );
                    }

                    for(int j=0;j<copyTarget->wpRoute.size();++j){
                        struct ScenarioWPRoute* swpr = new struct ScenarioWPRoute;

                        swpr->x = copyTarget->wpRoute[j]->x;
                        swpr->y = copyTarget->wpRoute[j]->y;
                        swpr->z = copyTarget->wpRoute[j]->z;
                        swpr->wpID = copyTarget->wpRoute[j]->wpID;
                        swpr->direct = copyTarget->wpRoute[j]->direct;
                        swpr->speedInfo = copyTarget->wpRoute[j]->speedInfo;

                        scenarioEvent->wpRoute.append( swpr );
                    }

                    simManage->SetScenarioEvent( currentScenarioID, scenarioEvent );
                }
            }
        }
        else if( tag == QString("Vehicle Event Type") ){

            scenarioEvent = new ScenarioEvents;

            scenarioEvent->eventID = eventNo;
            eventNo++;

            scenarioEvent->eventType = SCENARIO_EVENT_TYPE::OBJECT_EVENT;
            scenarioEvent->targetObjectID = scenarioItem->objectID;

            scenarioEvent->eventState = 0;
            scenarioEvent->eventTimeCount = 0;
            scenarioEvent->eventTimeCount_sub = 0;
            scenarioEvent->repeatByFE = false;

            scenarioEvent->routeType = -1;
            scenarioEvent->ndRoute = NULL;

            // eventKind = 0 :  Appear
            //             1 :  Control
            //             2 :  Send UDP
            //             3 :  Disappear
            int ekval = QString(divLine[1]).trimmed().toInt();
            if( ekval == 0 ){
                scenarioEvent->eventKind = OBJECT_SCENARIO_ACTION_KIND::APPEAR_VEHICLE;
            }
            else if( ekval == 1 ){
                scenarioEvent->eventKind = OBJECT_SCENARIO_ACTION_KIND::CONTROL_VEHICLE;
            }
            else if( ekval == 2 ){
                scenarioEvent->eventKind = OBJECT_SCENARIO_ACTION_KIND::SEND_UDP_OBJECT;
            }
            else if( ekval == 3 ){
                scenarioEvent->eventKind = OBJECT_SCENARIO_ACTION_KIND::DISAPPEAR;
            }

            trigger = new struct ScenarioTriggers;
            trigger->mode = 0;

            trigger->combination = 0;

            trigger->byKeyTriggerFlag = false;
            trigger->byExternalTriggerFlag = false;
            trigger->extTriggerFlagState = false;
            trigger->func_keys = 0;

            trigger->timeTrigger = new struct SimulationTime;
            trigger->timeTrigger->dt = 0.0;
            trigger->timeTrigger->day = 0;
            trigger->timeTrigger->min = 0;
            trigger->timeTrigger->sec = 0;
            trigger->timeTrigger->hour = 0;
            trigger->timeTrigger->msec = 0.0;
            trigger->timeTrigger->exe_freq = 0;
            trigger->timeTrigger->msec_count = 0;

            trigger->timeTriggerInSec = 0.0;
            trigger->ANDCondition = false;
            trigger->objectTigger.clear();

            scenarioEvent->eventTrigger = trigger;

            simManage->SetScenarioEvent( currentScenarioID, scenarioEvent );
        }
        else if( tag == QString("Vehicle Event Trigger External") ){

            int extTrig = QString(divLine[1]).trimmed().toInt();
            if( extTrig >= 1 ){
                trigger->byKeyTriggerFlag = true;
                trigger->func_keys = extTrig;
            }
            if( divLine.size() >= 3 ){
                trigger->byExternalTriggerFlag = ( QString(divLine[2]).trimmed().toInt() == 1 ? true : false );
                trigger->extTriggerFlagState = false;

                if( trigger->byExternalTriggerFlag == true ){
                    trigObj = new struct ObjectTriggerData;
                    trigObj->triggerType = TRIGGER_TYPE::BY_FUNCTION_EXTENDER;
                    trigObj->isTriggered = false;

                    trigger->objectTigger.append( trigObj );
                }
            }
        }
        else if( tag == QString("Vehicle Event Trigger Internal") ){

            if( divLine.size() == 2 ){

                if( trigger->mode != TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
                    trigger->mode = QString(divLine[1]).trimmed().toInt();
                }
            }
            else if( divLine.size() >= 7 ){

                trigger->combination = QString(divLine[1]).trimmed().toInt();

                for(int i=2;i<divLine.size();++i){

                    if( QString(divLine[i]).trimmed().toInt() == 1 ){

                        trigObj = new struct ObjectTriggerData;

                        if( i == 2 ){
                            trigObj->triggerType = TRIGGER_TYPE::AT_ONCE;
                        }
                        else if( i == 3 ){
                            trigObj->triggerType = TRIGGER_TYPE::TIME_TRIGGER;
                        }
                        else if( i == 4 ){
                            trigObj->triggerType = TRIGGER_TYPE::POSITION_TRIGGER;
                        }
                        else if( i == 5 ){
                            trigObj->triggerType = TRIGGER_TYPE::VELOCITY_TRIGGER;
                        }
                        else if( i == 6 ){
                            trigObj->triggerType = TRIGGER_TYPE::TTC_TRIGGER;
                        }
                        else if( i == 7 ){
                            trigObj->triggerType = TRIGGER_TYPE::BY_FUNCTION_EXTENDER;
                        }

                        trigObj->isTriggered = false;

                        trigger->objectTigger.append( trigObj );
                    }
                }
            }
        }
        else if( tag == QString("Vehicle Event Trigger Position") ){

            if( trigger->combination < 0 ){

                trigObj = new struct ObjectTriggerData;
                trigObj->targetObjectID = -1;
                trigObj->targetEventID  = -1;
                trigger->objectTigger.append( trigObj );
                if( divLine.size() >= 4 ){
                    trigObj->x         = QString(divLine[1]).trimmed().toFloat();
                    trigObj->y         = QString(divLine[2]).trimmed().toFloat();
                    trigObj->direction = QString(divLine[3]).trimmed().toFloat() * (0.017452);
                    trigObj->cosDirect = cos( trigObj->direction );
                    trigObj->sinDirect = sin( trigObj->direction );
                    trigObj->widthHalf = 5.0;
                }
            }
            else if( trigger->combination >= 0 ){

                for(int i=0;i<trigger->objectTigger.size();++i){

                    if( trigger->objectTigger[i]->triggerType == TRIGGER_TYPE::POSITION_TRIGGER ){

                        if( divLine.size() >= 5 ){

                            trigger->objectTigger[i]->x         = QString(divLine[1]).trimmed().toFloat();
                            trigger->objectTigger[i]->y         = QString(divLine[2]).trimmed().toFloat();
                            trigger->objectTigger[i]->direction = QString(divLine[3]).trimmed().toFloat() * (0.017452);
                            trigger->objectTigger[i]->cosDirect = cos( trigger->objectTigger[i]->direction );
                            trigger->objectTigger[i]->sinDirect = sin( trigger->objectTigger[i]->direction );
                            trigger->objectTigger[i]->targetObjectID = QString(divLine[4]).trimmed().toInt();

                            trigger->objectTigger[i]->widthHalf = 5.0;
                            if( divLine.size() >= 6 ){
                                trigger->objectTigger[i]->widthHalf = QString(divLine[5]).trimmed().toFloat();
                            }

                            trigger->objectTigger[i]->passCheckFlag = 0;
                        }
                        break;
                    }
                }
            }
        }
        else if( tag == QString("Vehicle Event Trigger Speed") ){

            if( trigger->combination < 0 ){

                trigObj->speed = QString(divLine[1]).trimmed().toFloat() / 3.6;

            }
            else if( trigger->combination >= 0 ){

                for(int i=0;i<trigger->objectTigger.size();++i){

                    if( trigger->objectTigger[i]->triggerType == TRIGGER_TYPE::VELOCITY_TRIGGER ){

                        if( divLine.size() >= 4 ){

                            trigger->objectTigger[i]->speed = QString(divLine[1]).trimmed().toFloat() / 3.6;
                            // triggerParam = velocity trigger lower or higher
                            trigger->objectTigger[i]->triggerParam = QString(divLine[2]).trimmed().toInt();
                            // triggerParam2 = velocity trigger target object ID
                            trigger->objectTigger[i]->targetObjectID = QString(divLine[3]).trimmed().toInt();

                        }
                        break;
                    }
                }
            }
        }
        else if( tag == QString("Vehicle Event Trigger T2TP") ){
            trigObj->TTC = QString(divLine[1]).trimmed().toFloat();
        }
        else if( tag == QString("Vehicle Event Target ID") ){
            trigObj->targetObjectID = QString(divLine[1]).trimmed().toInt();
        }
        else if( tag == QString("Vehicle Event Trigger TTC") ){

            if( trigger->combination >= 0 ){

                for(int i=0;i<trigger->objectTigger.size();++i){

                    if( trigger->objectTigger[i]->triggerType == TRIGGER_TYPE::TTC_TRIGGER ){

                        if( divLine.size() >= 7 ){

                            trigger->objectTigger[i]->TTC = QString(divLine[1]).trimmed().toFloat();
                            // triggerParam = TTC cal Type
                            trigger->objectTigger[i]->triggerParam = QString(divLine[2]).trimmed().toInt();
                            // triggerParams = Action Target Object ID
                            trigger->objectTigger[i]->targetObjectID = QString(divLine[3]).trimmed().toInt();
                            // triggerParams = TTC cal Object ID
                            trigger->objectTigger[i]->triggerParam2 = QString(divLine[4]).trimmed().toInt();
                            // TTC cal Pos X
                            trigger->objectTigger[i]->x = QString(divLine[5]).trimmed().toFloat();
                            // TTC cal Pos Y
                            trigger->objectTigger[i]->y = QString(divLine[6]).trimmed().toFloat();
                        }
                        else if( divLine.size() == 6 ){

                            trigger->objectTigger[i]->TTC = QString(divLine[1]).trimmed().toFloat();
                            // triggerParam = TTC cal Type
                            trigger->objectTigger[i]->triggerParam = QString(divLine[2]).trimmed().toInt();
                            // triggerParams = TTC cal Target Object ID
                            trigger->objectTigger[i]->targetObjectID = QString(divLine[3]).trimmed().toInt();
                            // TTC cal Pos X
                            trigger->objectTigger[i]->x = QString(divLine[4]).trimmed().toFloat();
                            // TTC cal Pos Y
                            trigger->objectTigger[i]->y = QString(divLine[5]).trimmed().toFloat();
                        }
                        break;
                    }
                }
            }
        }
        else if( tag == QString("Vehicle Event Trigger Time") ){

            if( divLine.size() >= 4 ){

                int etmode = QString(divLine[1]).trimmed().toInt();
                float val1 = QString(divLine[2]).trimmed().toFloat();
                float val2 = QString(divLine[3]).trimmed().toFloat();

                if( trigger->combination < 0 ){

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
                else if( trigger->combination >= 0 ){

                    for(int i=0;i<trigger->objectTigger.size();++i){

                        if( trigger->objectTigger[i]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ){

                            trigger->objectTigger[i]->timeTriggerInSec = val1;
                            if( etmode == 1 ){
                                trigger->objectTigger[i]->timeFromAppear = true;
                            }
                            else{
                                trigger->objectTigger[i]->timeFromAppear = false;
                            }
                            break;
                        }
                    }
                }
            }
        }
        else if( tag == QString("Vehicle Event Param Float") ){
            for(int i=1;i<divLine.size();++i){
                scenarioEvent->eventFloatData.append( QString(divLine[i]).trimmed().toFloat() );
            }
        }
        else if( tag == QString("Vehicle Event Param Integer") ){
            for(int i=1;i<divLine.size();++i){
                scenarioEvent->eventIntData.append( QString(divLine[i]).trimmed().toInt() );
            }
        }
        else if( tag == QString("Vehicle Event Param Boolean") ){
            for(int i=1;i<divLine.size();++i){
                scenarioEvent->eventBooleanData.append( QString(divLine[i]).trimmed().toInt() == 1 ? true : false );
            }
        }
        else if( tag == QString("Vehicle Route Multi-Lanes") ){

            int mlm = QString(divLine[1]).trimmed().toUInt();
            if( mlm == 1 ){

                scenarioEvent->ndRoute = new struct ODRouteData;

                for(int i=2;i<divLine.size();++i){

                    struct RouteElem* re = new struct RouteElem;

                    re->node = QString(divLine[i]).trimmed().toUInt();

                    scenarioEvent->ndRoute->routeToDestination.append( re );

                    if( i == 2 ){
                        scenarioEvent->ndRoute->originNode = re->node;
                    }
                    else if( i == divLine.size() - 1 ){
                        scenarioEvent->ndRoute->destinationNode = re->node;
                    }
                }

                scenarioEvent->ndRoute->onlyForScenarioVehicle = true;
                scenarioEvent->ndRoute->relatedScenarioObjectID = scenarioEvent->targetObjectID;

            }
            else if( mlm == 2 ){

                struct RouteLaneData *rld = new struct RouteLaneData;

                rld->startNode = QString( divLine[2] ).trimmed().toInt();
                rld->goalNode  = QString( divLine[3] ).trimmed().toInt();
                rld->sIndexInNodeList = QString( divLine[4] ).trimmed().toInt();
                rld->gIndexInNodeList = QString( divLine[5] ).trimmed().toInt();

                rld->LCDirect = DIRECTION_LABEL::STRAIGHT;

                scenarioEvent->ndRoute->LCSupportLaneLists.append( rld );

            }
            else if( mlm == 3 ){

                struct RouteLaneData *rld = scenarioEvent->ndRoute->LCSupportLaneLists.last();

                QList<int> lanelist;
                for(int i=2;i<divLine.size();++i){

                    int lane = QString(divLine[i]).trimmed().toInt();
                    lanelist.append( lane );

                }

                rld->laneList.append( lanelist );
            }

        }
        else if( tag == QString("Scenario Event ID") ){

            scenarioEvent = new ScenarioEvents;

            //scenarioEvent->eventID = eventNo;
            scenarioEvent->eventID = QString(divLine[1]).trimmed().toInt();
            eventNo++;

            scenarioEvent->eventType = SCENARIO_EVENT_TYPE::SYSTEM_EVENT;
            scenarioEvent->eventState = 0;
            scenarioEvent->targetObjectID = -1;
            scenarioEvent->repeatByFE = false;

            trigger = new struct ScenarioTriggers;
            trigger->mode = 0;
            trigger->combination = -1;

            trigger->func_keys = 0;

            trigger->timeTrigger = new struct SimulationTime;
            trigger->timeTrigger->dt = 0.0;
            trigger->timeTrigger->day = 0;
            trigger->timeTrigger->min = 0;
            trigger->timeTrigger->sec = 0;
            trigger->timeTrigger->hour = 0;
            trigger->timeTrigger->msec = 0.0;
            trigger->timeTrigger->exe_freq = 0;
            trigger->timeTrigger->msec_count = 0;

            trigger->timeTriggerInSec = 0.0;
            trigger->ANDCondition = false;
            trigger->objectTigger.clear();

            scenarioEvent->eventTrigger = trigger;

            simManage->SetScenarioEvent( currentScenarioID, scenarioEvent );
        }
        else if( tag == QString("Scenario Event Type") ){

            int val = QString(divLine[1]).trimmed().toInt();

            // eventKind = 0 :  Warp
            //             1 :  Change Traffic Signal
            //             2 :  Change Speed Info
            //             3 :  Send UDP data
            if( val == 0 ){
                scenarioEvent->eventKind = SYSTEM_SCENARIO_ACTION_KIND::WARP;
            }
            else if( val == 1 ){
                scenarioEvent->eventKind = SYSTEM_SCENARIO_ACTION_KIND::CHANGE_TRAFFIC_SIGNAL_DISPLAY;
            }
            else if( val == 2 ){
                scenarioEvent->eventKind = SYSTEM_SCENARIO_ACTION_KIND::CHANGE_SPEED_INFO;
            }
            else if( val == 3 ){
                scenarioEvent->eventKind = SYSTEM_SCENARIO_ACTION_KIND::SEND_UDP_SYSTEM;
            }
        }
        else if( tag == QString("Scenario Event Trigger External") ){

            int extTrig = QString(divLine[1]).trimmed().toInt();
            if( extTrig >= 1 ){
                trigger->byKeyTriggerFlag = true;
                trigger->func_keys = extTrig;
            }
            if( divLine.size() >= 3 ){
                trigger->byExternalTriggerFlag = ( QString(divLine[2]).trimmed().toInt() == 1 ? true : false );
                trigger->extTriggerFlagState = false;

                if( trigger->byExternalTriggerFlag == true ){
                    trigObj = new struct ObjectTriggerData;
                    trigObj->triggerType = TRIGGER_TYPE::BY_FUNCTION_EXTENDER;
                    trigObj->isTriggered = false;

                    trigger->objectTigger.append( trigObj );
                }
            }
        }
        else if( tag == QString("Scenario Event Trigger Internal") ){

            if( divLine.size() == 2 ){

                if( trigger->mode != TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
                    trigger->mode = QString(divLine[1]).trimmed().toInt();
                }
            }
            else if( divLine.size() >= 7 ){

                trigger->combination = QString(divLine[1]).trimmed().toInt();

                for(int i=2;i<divLine.size();++i){

                    if( QString(divLine[i]).trimmed().toInt() == 1 ){

                        trigObj = new struct ObjectTriggerData;

                        if( i == 2 ){
                            trigObj->triggerType = TRIGGER_TYPE::AT_ONCE;
                        }
                        else if( i == 3 ){
                            trigObj->triggerType = TRIGGER_TYPE::TIME_TRIGGER;
                        }
                        else if( i == 4 ){
                            trigObj->triggerType = TRIGGER_TYPE::POSITION_TRIGGER;
                        }
                        else if( i == 5 ){
                            trigObj->triggerType = TRIGGER_TYPE::VELOCITY_TRIGGER;
                        }
                        else if( i == 6 ){
                            trigObj->triggerType = TRIGGER_TYPE::TTC_TRIGGER;
                        }
                        else if( i == 7 ){
                            trigObj->triggerType = TRIGGER_TYPE::BY_FUNCTION_EXTENDER;
                        }

                        trigObj->isTriggered = false;

                        trigger->objectTigger.append( trigObj );
                    }
                }
            }

        }
        else if( tag == QString("Scenario Event Trigger Position") ){

            if( trigger->combination < 0 ){

                trigObj = new struct ObjectTriggerData;
                trigObj->targetObjectID = -1;
                trigObj->targetEventID  = -1;
                trigger->objectTigger.append( trigObj );
                if( divLine.size() >= 4 ){
                    trigObj->x         = QString(divLine[1]).trimmed().toFloat();
                    trigObj->y         = QString(divLine[2]).trimmed().toFloat();
                    trigObj->direction = QString(divLine[3]).trimmed().toFloat() * (0.017452);
                    trigObj->cosDirect = cos( trigObj->direction );
                    trigObj->sinDirect = sin( trigObj->direction );
                    trigObj->widthHalf = 5.0;
                }
            }
            else if( trigger->combination >= 0 ){

                for(int i=0;i<trigger->objectTigger.size();++i){

                    if( trigger->objectTigger[i]->triggerType == TRIGGER_TYPE::POSITION_TRIGGER ){

                        if( divLine.size() >= 5 ){

                            trigger->objectTigger[i]->x         = QString(divLine[1]).trimmed().toFloat();
                            trigger->objectTigger[i]->y         = QString(divLine[2]).trimmed().toFloat();
                            trigger->objectTigger[i]->direction = QString(divLine[3]).trimmed().toFloat() * (0.017452);
                            trigger->objectTigger[i]->cosDirect = cos( trigger->objectTigger[i]->direction );
                            trigger->objectTigger[i]->sinDirect = sin( trigger->objectTigger[i]->direction );
                            trigger->objectTigger[i]->targetObjectID = QString(divLine[4]).trimmed().toInt();

                            trigger->objectTigger[i]->widthHalf = 5.0;
                            if( divLine.size() >= 6 ){
                                trigger->objectTigger[i]->widthHalf = QString(divLine[5]).trimmed().toFloat();
                            }

                            trigger->objectTigger[i]->passCheckFlag = 0;
                        }
                        break;
                    }
                }
            }
        }
        else if( tag == QString("Scenario Event Trigger Speed") ){

            if( trigger->combination < 0 ){
                trigObj->speed = QString(divLine[1]).trimmed().toFloat() / 3.6;
            }
            else{

                for(int i=0;i<trigger->objectTigger.size();++i){

                    if( trigger->objectTigger[i]->triggerType == TRIGGER_TYPE::VELOCITY_TRIGGER ){

                        if( divLine.size() >= 4 ){

                            trigger->objectTigger[i]->speed          = QString(divLine[1]).trimmed().toFloat() / 3.6;  // [km/h]->[m/s]
                            trigger->objectTigger[i]->triggerParam   = QString(divLine[2]).trimmed().toInt();   // vtLowOrHigh
                            trigger->objectTigger[i]->targetObjectID = QString(divLine[3]).trimmed().toInt();
                        }
                        break;
                    }
                }
            }
        }
        else if( tag == QString("Scenario Event Trigger TTC") ){

            if( trigger->combination >= 0 ){
                for(int i=0;i<trigger->objectTigger.size();++i){

                    if( trigger->objectTigger[i]->triggerType == TRIGGER_TYPE::TTC_TRIGGER ){

                        if( divLine.size() >= 7 ){

                            trigger->objectTigger[i]->TTC            = QString(divLine[1]).trimmed().toFloat();
                            trigger->objectTigger[i]->targetObjectID = QString(divLine[2]).trimmed().toInt();
                            trigger->objectTigger[i]->triggerParam   = QString(divLine[3]).trimmed().toInt();    // ttcCalType
                            trigger->objectTigger[i]->triggerParam2  = QString(divLine[4]).trimmed().toInt();    // ttcCalObjectID
                            trigger->objectTigger[i]->x              = QString(divLine[5]).trimmed().toFloat();
                            trigger->objectTigger[i]->y              = QString(divLine[6]).trimmed().toFloat();
                        }
                        break;
                    }
                }
            }

        }
        else if( tag == QString("Scenario Event TTC") ){

            if( trigger->combination < 0 ){
                trigObj->TTC = QString(divLine[1]).trimmed().toFloat();
            }

        }
        else if( tag == QString("Scenario Event Target Object ID") ){

            if( trigger->combination < 0 ){
                trigObj->targetObjectID = QString(divLine[1]).trimmed().toInt();
            }
        }
        else if( tag == QString("Scenario Event Related Event ID") ){

            if( trigger->combination < 0 ){
                trigObj->targetEventID = QString(divLine[1]).trimmed().toInt();
            }
        }
        else if( tag == QString("Scenario Event Trigger Time") ){

            if( trigger->combination < 0 ){
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
            else{

                if( divLine.size() >= 4 ){

                    float val = QString(divLine[2]).trimmed().toFloat();

                    for(int i=0;i<trigger->objectTigger.size();++i){

                        if( trigger->objectTigger[i]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ){

                            trigger->objectTigger[i]->timeTriggerInSec = val;
                            trigger->objectTigger[i]->timeFromAppear = false;
                            break;
                        }
                    }
                }
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
        else if( tag == QString("Scenario Event Param Boolean") ){
            for(int i=1;i<divLine.size();++i){
                int val = QString(divLine[i]).trimmed().toInt();
                scenarioEvent->eventBooleanData.append( ( val == 1 ? true : false) );
            }
        }

    }
    file.close();


    // Set routes for scenario objects
    simManage->SetScenarioObjectsRouteInfo();


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

            emit SetTrafficSignalList(trafficSignal);

            for(int i=0;i<trafficSignal.size();++i){

                emit SetTrafficSignalPointer( trafficSignal.at(i) );

                int ndID = trafficSignal[i]->relatedNode;
                int ndIdx = road->nodeId2Index.indexOf( ndID );
                if( ndIdx >= 0 ){
                    if( trafficSignal[i]->type == 'v' ){
                        road->nodes[ndIdx]->relatedVTSIndex.append( i );
                    }
                    else if( trafficSignal[i]->type == 'p' ){
                        road->nodes[ndIdx]->relatedPTSIndex.append( i );
                    }
                }
            }
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

            if( divDataStr.size() >= 4 ){
                ts->isSensorType = QString( divDataStr[2] ).trimmed().toInt();
                ts->timeChangeBySensor = QString( divDataStr[3] ).trimmed().toInt();
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

        int numEvent = simManage->GetNumberScenarioEvent(i);
        for(int j=0;j<numEvent;++j){

            if( simManage->GetScenarioEventType(i,j) != SCENARIO_EVENT_TYPE::OBJECT_EVENT ){
                continue;
            }
            if( simManage->GetScenarioEventKind(i,j) != OBJECT_SCENARIO_ACTION_KIND::APPEAR_VEHICLE ){
                continue;
            }

            if( simManage->GetScenarioEventRouteType(i,j) == ROUTE_TYPE::NODE_LIST_TYPE ){

                struct ODRouteData *rd = simManage->GetODRouteOfScenarioEvent(i,j);
                if( !rd ){
                    qDebug() << "!!!";
                    qDebug() << "Route Node-List data for scenario vehicle is not given.";
                    qDebug() << "Check SEdit Scenario Data.";
                    continue;
                }


                qDebug() << "GetODRouteOfScenarioEvent: i = " << i << " j = " << j;
                qDebug() << "onlyForScenarioVehicle = " << rd->onlyForScenarioVehicle;
                qDebug() << "originNode = " << rd->originNode;
                qDebug() << "destinationNode = " << rd->destinationNode;

                road->odRoute.append( rd );


                for(int k=0;k<rd->LCSupportLaneLists.size();++k){

                    for(int l=0;l<rd->LCSupportLaneLists[k]->laneList.size();++l){

                        QList<QPoint> pairData;

                        rd->mergeLanesInfo.append( pairData );

                        for(int m=0;m<rd->LCSupportLaneLists[k]->laneList[l].size()-1;++m){

                            int pIdx = road->pathId2Index.indexOf( rd->LCSupportLaneLists[k]->laneList[l][m] );
                            if( pIdx >= 0 ){
                                if( road->paths[pIdx]->followingPaths.size() == 2 ){

                                    int p1 = road->paths[pIdx]->followingPaths[0];
                                    int p2 = road->paths[pIdx]->followingPaths[1];

                                    if( p2 == rd->LCSupportLaneLists[k]->laneList[l][m+1] ){
                                        int t = p2;
                                        p2 = p1;
                                        p1 = t;
                                    }

                                    int p1Idx = road->pathId2Index.indexOf( p1 );
                                    int p2Idx = road->pathId2Index.indexOf( p2 );
                                    if( p1Idx >= 0 && p2Idx >= 0 ){

                                        if( road->paths[p1Idx]->connectingNodeInDir == road->paths[p2Idx]->connectingNodeInDir ){

                                            QPoint mergeLanePair;
                                            mergeLanePair.setX( p1 );
                                            mergeLanePair.setY( p2 );

                                            rd->mergeLanesInfo.last().append( mergeLanePair );
                                        }
                                    }
                                }
                            }
                            else{
                                qDebug() << "Invalid path found: id = " << rd->LCSupportLaneLists[k]->laneList[l][m] << " in LCSupportLaneLists[" << k << "]->laneList[" << l << "]";
                                qDebug() << "No index found.";
                                qDebug() << "odRoute: O = " << rd->originNode << " D = " << rd->destinationNode;
                            }
                        }
                    }
                }

                for(int k=1;k<rd->LCSupportLaneLists.size();++k){

                    struct RouteLaneData *rld = rd->LCSupportLaneLists.at( k );

                    int myWPin = -1;
                    for(int l=0;l<rld->laneList[0].size();++l){

                        int lIdx = road->pathId2Index.indexOf( rld->laneList[0][l] );
                        if( road->paths[lIdx]->connectingNode != rld->goalNode ){
                            continue;
                        }
                        int twp = road->paths[lIdx]->endWpId;
                        int twpIdx = road->wpId2Index.indexOf( twp );
                        if( road->wps[twpIdx]->isNodeInWP == true ){
                            myWPin = twp;
                            break;
                        }
                    }

                    struct RouteLaneData *nextrld = rd->LCSupportLaneLists.at( k-1 );

                    int targetWPin = -1;
                    for(int l=0;l<nextrld->laneList[0].size();++l){
                        int lIdx = road->pathId2Index.indexOf( nextrld->laneList[0][l] );
                        if( road->paths[lIdx]->connectingNode != rld->goalNode ){
                            continue;
                        }
                        int twp = road->paths[lIdx]->endWpId;
                        int twpIdx = road->wpId2Index.indexOf( twp );
                        if( road->wps[twpIdx]->isNodeInWP == true ){
                            targetWPin = twp;
                            break;
                        }
                    }

                    if( myWPin >= 0 && targetWPin >= 0 ){

                        int laneNoDiff = 0;

                        int ndIdx = road->nodeId2Index.indexOf( rld->goalNode );
                        for(int l=0;l<road->nodes[ndIdx]->inBoundaryWPs.size();++l){
                            if( road->nodes[ndIdx]->inBoundaryWPs[l]->wpId == myWPin ){
                                laneNoDiff += road->nodes[ndIdx]->inBoundaryWPs[l]->laneNo;
                            }
                            if( road->nodes[ndIdx]->inBoundaryWPs[l]->wpId == targetWPin ){
                                laneNoDiff -= road->nodes[ndIdx]->inBoundaryWPs[l]->laneNo;
                            }
                        }

                        if( laneNoDiff > 0 ){
                            rld->LCDirect = DIRECTION_LABEL::LEFT_CROSSING;
                        }
                        else{
                            rld->LCDirect = DIRECTION_LABEL::RIGHT_CROSSING;
                        }

                    }
                }

            }
            else if( simManage->GetScenarioEventRouteType(i,j) == ROUTE_TYPE::PATH_LIST_TYPE ){

                int numWP = simManage->GetNumberWPDataOfScenarioEvent(i,j);
                if( numWP == 0 ){
                    continue;
                }

                int objectID = simManage->GetScenarioEventObjectID(i,j);

                for(int k=0;k<numWP;++k){
                    struct ScenarioWPRoute *wpdata = simManage->GetWPDataOfScenarioEvent(i,j,k);

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

                QList<int> tp;
                for(int k=0;k<road->paths.size();++k){
                    if( road->paths[k]->scenarioObjectID != objectID ){
                        continue;
                    }
                    else{
                        tp.prepend( road->paths[k]->id );
                    }
                }

                simManage->SetTargetPathToScenarioEvent(i,j,tp);
            }

        }

        for(int j=0;j<numEvent;++j){

            if( simManage->GetScenarioEventType(i,j) != SCENARIO_EVENT_TYPE::OBJECT_EVENT ){
                continue;
            }
            if( simManage->GetScenarioEventKind(i,j) != OBJECT_SCENARIO_ACTION_KIND::APPEAR_PEDESTRIAN ){
                continue;
            }

            if( simManage->GetScenarioEventRouteType(i,j) == ROUTE_TYPE::PEDEST_PATH_LIST_TYPE ){

                int numWP = simManage->GetNumberWPDataOfScenarioEvent(i,j);
                if( numWP == 0 ){
                    continue;
                }

                int objectID = simManage->GetScenarioEventObjectID(i,j);

                int maxPedestPathID = -1;
                for(int k=0;k<road->pedestPaths.size();++k){
                    if( maxPedestPathID < road->pedestPaths[k]->id ){
                        maxPedestPathID = road->pedestPaths[k]->id;
                    }
                }
                maxPedestPathID++;

                struct PedestPath* path = new struct PedestPath;

                path->id = maxPedestPathID;
                path->scenarioObjectID = objectID;

                road->pedestPaths.append( path );
                road->pedestPathID2Index.append( path->id );

                for(int k=0;k<numWP;++k){

                    struct PedestPathShapeInfo *si = new struct PedestPathShapeInfo;

                    struct ScenarioWPRoute *wpdata = simManage->GetWPDataOfScenarioEvent(i,j,k);

                    si->pos.setX( wpdata->x );
                    si->pos.setY( wpdata->y );
                    si->pos.setZ( wpdata->z );
                    si->width = 1.5;

                    si->distanceToNextPos = 0.0;
                    si->angleToNextPos = 0.0;
                    si->cosA = 1.0;
                    si->sinA = 0.0;

                    si->isCrossWalk = false;
                    si->controlPedestSignalID = -1;
                    si->runOutDirect = 0;
                    si->runOutProb = 0.0;


                    road->pedestPaths.last()->shape.append( si );
                }

                for(int k=0;k<road->pedestPaths.last()->shape.size()-1;++k){

                    float dx = road->pedestPaths.last()->shape[k+1]->pos.x() - road->pedestPaths.last()->shape[k]->pos.x();
                    float dy = road->pedestPaths.last()->shape[k+1]->pos.y() - road->pedestPaths.last()->shape[k]->pos.y();

                    float len = sqrt( dx * dx + dy * dy );
                    float dir = atan2( dy, dx );

                    road->pedestPaths.last()->shape[k]->distanceToNextPos = len;
                    road->pedestPaths.last()->shape[k]->angleToNextPos = dir;
                    road->pedestPaths.last()->shape[k]->cosA = cos(dir);
                    road->pedestPaths.last()->shape[k]->sinA = sin(dir);

                    if( k == road->pedestPaths.last()->shape.size() - 2 ){
                        road->pedestPaths.last()->shape[k+1]->angleToNextPos = dir;
                        road->pedestPaths.last()->shape[k+1]->cosA = cos(dir);
                        road->pedestPaths.last()->shape[k+1]->sinA = sin(dir);
                    }
                }
            }
        }
    }

    road->SetPathConnection();

    road->CheckPedestPathConnection();


    qDebug() << "emit SetRoadPointer signal";
    emit SetRoadPointer( road );
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


void SystemThread::SetSpeedAdjustVal(int val)
{
    //qDebug() << "[SystemThread::SetSpeedAdjustVal] val = " << val;
    speedAdjustVal = val;
}


int SystemThread::SetSendData(int maxAgentSend,int maxTSSend,int pivotID)
{
    if( pivotID < 0 || pivotID >= 10 ){
        qDebug() << "!!!! Invalid pivotID = " << pivotID << "; SetSendData in System Thread.";
        return 0;
    }

    // set agent and TS data to UE4
    memset( sendDataBuf, 0, sendDataMaxSize );

    sendDataBuf[0] = 'R';
    sendDataBuf[1] = 'S';
    sendDataBuf[2] = 'd';

    int at = 3;

    int numAgentSend = 0;

    QList<int> sendIDList;

    // count number of data

    float intervalDist = 20.0;
    int countByDistance[20];   // 20[m] * 20 = 400[m]
    for(int i=0;i<20;++i){
        countByDistance[i] = 0;
    }
    QList<int> objListByDistance[20];

    QList<int> calDist;

    objListByDistance[0].append( pivotID );


    float xPivot = agent[pivotID]->state.x;
    float yPivot = agent[pivotID]->state.y;

    if( isnan(xPivot) || isnan(yPivot) ){
        qDebug() << "[FATAL] xPivot or yPivot is not a number. pivotID = " << pivotID;
        invalidXPivotData = true;
        return 0;
    }


//    qDebug() << "xPivot = " << xPivot << " yPivot = " << yPivot;

    numAgentSend = 0;
    for(int i=0;i<maxAgent;++i){

        calDist.append( -1 );

        int agentStatus = agent[i]->agentStatus;
        if( agentStatus != 1 ){
            continue;
        }

        if( i == pivotID ){
            continue;
        }

        float xTest = agent[i]->state.x;
        float yTest = agent[i]->state.y;
        float rx = xTest - xPivot;
        float ry = yTest - yPivot;
        float D = sqrt(rx * rx + ry * ry);
        if( D > 450 ){
            if( noClearByWarp == 0 ){
                agent[i]->UE4ObjIDDelCount[pivotID]++;
                if( agent[i]->UE4ObjIDDelCount[pivotID] >= 3 ){
                    agent[i]->UE4ObjectID[pivotID] = -1;
                    agent[i]->UE4ObjIDDelCount[pivotID] = 0;
                }
            }
            continue;
        }

        int idx = (int)(D / intervalDist);
        if( idx >=20 ){
            idx = 19;
        }
        else if( idx < 0){
            idx = 0;
        }

        objListByDistance[idx].append( i );

        calDist[i] = idx;

        countByDistance[idx]++;
    }

    if( noClearByWarp > 0 ){
        noClearByWarp++;
        if( noClearByWarp >= 5 ){
            noClearByWarp = 0;
        }
    }

//    for(int i=0;i<20;++i){
//        qDebug() << "[" << i << "]objListByDistance=" << objListByDistance[i] << ", N="<< countByDistance[i];
//    }



    int totalNum = 0;
    int thrIdx = 0;
    for(int i=0;i<20;++i){
        thrIdx = i;
        if( totalNum < maxAgentSend && totalNum + countByDistance[i] >= maxAgentSend ){
            break;
        }
        else{
            totalNum += countByDistance[i];
        }
    }


//    qDebug() << "thrIdx = " << thrIdx;
//    QString dbgStr = QString("countByDistance=");
//    for(int i=0;i<20;++i){
//        dbgStr += QString(" %1").arg( countByDistance[i] );
//    }
//    qDebug() << dbgStr;


    int numSpawn = road->numActorForUE4Model;
    int maxUE4ID = road->maxActorInUE4;

    if( !UE4ObjectIDUseFlag ){
       UE4ObjectIDUseFlag = new char[ maxUE4ID ];
    }

    for(int i=0;i<maxUE4ID;++i){
        UE4ObjectIDUseFlag[i] = 0;
    }


    numAgentSend = 0;
    sendIDList.clear();

    for(int i=0;i<=thrIdx;++i){
        for(int j=0;j<objListByDistance[i].size();++j){
            int aIdx = objListByDistance[i][j];
            if( aIdx == pivotID ){
                continue;
            }
            int objID = agent[aIdx]->UE4ObjectID[pivotID];
            if( objID >= 0 ){
                UE4ObjectIDUseFlag[objID] = 1;
            }
        }
    }


//    qDebug() << "UE4ObjectIDUseFlag:";
//    for(int i=0;i<nVKind + nPKind;++i){
//        QString dbgStr = QString("");
//        for(int j=0;j<numSpawn;++j){
//            dbgStr += QString(" %1").arg( (UE4ObjectIDUseFlag[i*numSpawn+j] == 1 ? 1 : 0 ) );
//        }
//        qDebug() << dbgStr;
//    }



    for(int i=0;i<=thrIdx;++i){
        for(int j=0;j<objListByDistance[i].size();++j){

            int aIdx = objListByDistance[i][j];

            // Reject SInterface Object
            if( aIdx == pivotID ){
                agent[aIdx]->UE4ObjectID[pivotID] = pivotID;
                continue;
            }

            // Send agentID if the vehicle is old-version scenario vehicle
            if( agent[aIdx]->isScenarioObject == true && agent[aIdx]->isOldScenarioType == true ){
                agent[aIdx]->UE4ObjectID[pivotID] = aIdx;
                calDist[aIdx] = -1;
                continue;
            }


            int baseID = 0;
            if( agent[aIdx]->objIDForUE4 < 0 ){
                if( agent[aIdx]->objTypeForUE4 == 0 ){       // Vehicle
                    baseID = 10 + agent[aIdx]->objNoForUE4 * numSpawn;
                }
                else if( agent[aIdx]->objTypeForUE4 == 1 ){  // Pedestrian
                    baseID = 500 + agent[aIdx]->objNoForUE4 * numSpawn;
                }
                else if( agent[aIdx]->objTypeForUE4 == 2 ){  // Bicycle
                    baseID = 580 + agent[aIdx]->objNoForUE4 * numSpawn;
                }
                else{
                    baseID = 0;
                }
            }
            else{
                baseID = agent[aIdx]->objIDForUE4;
            }


            if( agent[aIdx]->UE4ObjectID[pivotID] < 0 ){

                for(int k=0;k<numSpawn;++k){

                    int objID = baseID + k;

                    if( UE4ObjectIDUseFlag[objID] == 0 ){
                        agent[aIdx]->UE4ObjectID[pivotID] = objID;
                        UE4ObjectIDUseFlag[objID] = 1;
                        calDist[aIdx] = -1;
                        break;
                    }
                }
            }
            else{
                calDist[aIdx] = -1;
            }

            if( agent[aIdx]->UE4ObjectID[pivotID] < 0 ){

                for(int k=thrIdx;k>i;k--){
                    for(int l=0;l<objListByDistance[k].size();++l){
                        int tIdx = objListByDistance[k][l];
                        if( agent[tIdx]->UE4ObjectID[pivotID] >= baseID &&
                                agent[tIdx]->UE4ObjectID[pivotID] < baseID + numSpawn ){

                            //qDebug() << "tIdx=" << tIdx << " UE4=" << agent[tIdx]->UE4ObjectID[pivotID] << " -> aIdx" << aIdx;

                            qDebug() << "Short of UE4 Object ID; fetch from far vehicle ID";
                            qDebug() << "   Agent[" << tIdx <<"]: modelID = " << agent[tIdx]->UE4ObjectID[pivotID] << " -> Agent[" << aIdx << "]";

                            agent[aIdx]->UE4ObjectID[pivotID] = agent[tIdx]->UE4ObjectID[pivotID];
                            agent[tIdx]->UE4ObjectID[pivotID] = -1;

                            agent[aIdx]->UE4ObjIDDelCount[pivotID] = 0;
                            agent[tIdx]->UE4ObjIDDelCount[pivotID] = 0;

                            calDist[aIdx] = -1;
                            break;
                        }
                    }
                }
            }
        }
    }

    for(int i=0;i<=thrIdx;++i){
        for(int j=0;j<objListByDistance[i].size();++j){

            int aIdx = objListByDistance[i][j];
            if( agent[aIdx]->UE4ObjectID[pivotID] < 0 ){
                continue;
            }
            if( numAgentSend < maxAgentSend ){
                sendIDList.append( aIdx );
                numAgentSend++;
            }
            else{
                break;
            }
        }
    }

    if( noClearByWarp == 0 ){
        for(int i=0;i<maxAgent;++i){
            if( agent[i]->agentStatus != 1 ){
                continue;
            }
            if( i == pivotID ){
                continue;
            }

            if( calDist[i] > 0.0 ){
                agent[i]->UE4ObjIDDelCount[pivotID]++;
                if( agent[i]->UE4ObjIDDelCount[pivotID] >= 3 ){
                    agent[i]->UE4ObjectID[pivotID] = -1;
                    agent[i]->UE4ObjIDDelCount[pivotID] = 0;
                }
            }
        }
    }

    numAgentSend = sendIDList.size();


    memcpy(&(sendDataBuf[at]), &numAgentSend, sizeof(int));   // 3
    at += sizeof(int);

    float tempFVal = 0.0;
    int   tempIVal = 0;
    char  tempCVal = 0;

    int maxSize = 125 * maxAgentSend + 13 * maxTSSend + 20 + 5;


//    qDebug() << "numAgentSend = " << numAgentSend;
//    for(int n=0;n<numAgentSend;++n){
//        int i = sendIDList[n];
//        if( i == pivotID ){
//            qDebug() << "  [a]ID = " << agent[i]->ID << " CG=" << agent[i]->UE4ObjectID[ pivotID ];
//        }
//        else{
//            if( agent[i]->isScenarioObject == true && agent[i]->isOldScenarioType == true ){
//                qDebug() << "  [b]ID = " << agent[i]->ID << " CG=" << agent[i]->UE4ObjectID[ pivotID ];
//            }
//            else{
//                qDebug() << "  [c]ID = " << agent[i]->ID << " CG=" << (agent[i]->UE4ObjectID[ pivotID ] +10);
//            }
//        }
//    }


    for(int n=0;n<numAgentSend;++n){

        int i = sendIDList[n];

        if( at + 125 < maxSize ){

            tempCVal = 'a';
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 7
            at += sizeof(char);


            //tempIVal = agent[i]->ID;
            if( i == pivotID ){
                tempIVal = pivotID;
                agent[i]->UE4ObjectID[ pivotID ] = pivotID;
            }
            else{

                tempIVal = agent[i]->UE4ObjectID[ pivotID ];
            }

//            qDebug() << "[UE4]ID=" << agent[i]->ID << " objID=" << tempIVal << " mdlID=" << agent[i]->vehicle.GetVehicleModelID();
//                     << " x = " << agent[i]->state.x << " y = " << agent[i]->state.y;

            memcpy(&(sendDataBuf[at]), &tempIVal, sizeof(int));   // 8
            at += sizeof(int);

            tempFVal = agent[i]->state.x;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 12
            at += sizeof(float);

            tempFVal = agent[i]->state.y * (-1.0);
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 16
            at += sizeof(float);

//            tempFVal = agent[i]->state.z;
            tempFVal = agent[i]->state.z_path;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 20
            at += sizeof(float);

            tempFVal = agent[i]->state.roll;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 24
            at += sizeof(float);

            tempFVal = agent[i]->state.pitch;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 28
            at += sizeof(float);

            tempFVal = agent[i]->state.yaw * (-1.0);

            if( i != pivotID && agent[i]->vehicle.yawFiltered4CG != NULL && agent[i]->agentKind < 100 ){
                tempFVal = agent[i]->vehicle.yawFiltered4CG->GetOutput() * (-1.0);
            }

            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 32
            at += sizeof(float);

            tempFVal = agent[i]->state.V;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 36
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.ax;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 40
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.yawRate;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 44
            at += sizeof(float);

            tempFVal = agent[i]->memory.lateralDeviationFromTargetPath;   // latDev
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 48
            at += sizeof(float);

            tempFVal = agent[i]->memory.lateralDeviationFromTargetPathAtPreviewPoint;   // latDev at aim point
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 52
            at += sizeof(float);

            tempCVal = 0;     // engineKeyState
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 56
            at += sizeof(char);

            tempCVal = 0;     // gearPosition
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 57
            at += sizeof(char);

            tempIVal = agent[i]->ID;     // Tachometer --> Original ID
            memcpy(&(sendDataBuf[at]), &tempIVal, sizeof(int));   // 58
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

            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 62
            at += sizeof(char);

            tempCVal = 0;     // lightFlag2
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 63
            at += sizeof(char);

            if( agent[i]->agentKind < 100 ){
                tempFVal = agent[i]->vehicle.param.Lf;
            }
            else{
                tempFVal = 0.0;
            }
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 64
            at += sizeof(float);

            if( agent[i]->agentKind < 100 ){
                tempFVal = agent[i]->vehicle.param.Lr;
            }
            else{
                tempFVal = 0.0;
            }
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 68
            at += sizeof(float);

            if( agent[i]->agentKind < 100 ){
                tempFVal = 1.8;     // Tf
            }
            else{
                tempFVal = 0.0;
            }
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 72
            at += sizeof(float);

            if( agent[i]->agentKind < 100 ){
                tempFVal = 1.8;     // Tr
            }
            else{
                tempFVal = 0.0;
            }
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 76
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.tireRotAngle;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 80
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.steer * (-57.3 / 6.0) ;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 84
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.tireRotAngle;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 88
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.steer * (-57.3 / 6.0);
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 92
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.tireRotAngle;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 96
            at += sizeof(float);

            tempFVal = 0.0;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 100
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.tireRotAngle;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 104
            at += sizeof(float);

            tempFVal = 0.0;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 108
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.relZBody;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 112
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.relRollBody;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 116
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.relPitchBody;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 120
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.relYBody;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 124
            at += sizeof(float);

            tempCVal = 0;     // optionalFlag1
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 128
            at += sizeof(char);

            tempCVal = 0;     // optionalFlag2
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 129
            at += sizeof(char);

            tempCVal = 0;     // optionalFlag3
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 130
            at += sizeof(char);

            tempCVal = 1;     // optionalFlag4
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 131
            at += sizeof(char);
        }
    }


    int numTStSend = 0;

    QList<int> sendTSList;

    if( numTrafficSignals <= maxTSSend ){

        numTStSend = numTrafficSignals;
        for(int i=0;i<numTrafficSignals;++i){
            sendTSList.append(i);
        }

    }
    else{

        int countByDistance[20];   // 50[m] * 20 = 1000[m]
        for(int i=0;i<20;++i){
            countByDistance[i] = 0;
        }

        QList<int> calDist;

        for(int i=0;i<numTrafficSignals;++i){

            float rx = trafficSignal[i]->xTS - xPivot;
            float ry = trafficSignal[i]->yTS - yPivot;
            float D = sqrt(rx * rx + ry * ry);

            int idx = (int)(D / 50.0);
            if( idx >=20 ){
                idx = 19;
            }
            else if( idx < 0 ){
                idx = 0;
            }

            calDist.append( idx );

            countByDistance[idx]++;
        }

        int totalNum = 0;
        int thrIdx = 0;
        for(int i=0;i<20;++i){
            thrIdx = i;
            if( totalNum < maxTSSend && totalNum + countByDistance[i] >= maxTSSend ){
                break;
            }
            else{
                totalNum += countByDistance[i];
            }
        }

        numTStSend = 0;

        for(int i=0;i<numTrafficSignals;++i){

            if( calDist[i] > thrIdx ){
                continue;
            }

            if( numTStSend < maxTSSend ){
                sendTSList.append( i );
                numTStSend++;
            }
            else{
                break;
            }
        }

    }

    numTStSend = sendTSList.size();

    memcpy(&(sendDataBuf[at]), &numTStSend, sizeof(int));
    at += sizeof(int);

    for(int n=0;n<numTStSend;++n){

        int i = sendTSList[n];

        memcpy(&(sendDataBuf[at]), &(trafficSignal[i]->id), sizeof(int));
        at += sizeof(int);

        sendDataBuf[at] = trafficSignal[i]->type;
        at += sizeof(char);

        int display = trafficSignal[i]->GetCurrentDisplayInfo();
        memcpy(&(sendDataBuf[at]), &display, sizeof(int));
        at += sizeof(int);

        float remainTime = trafficSignal[i]->remainingTimeToNextDisplay;
        memcpy(&(sendDataBuf[at]), &remainTime, sizeof(float));
        at += sizeof(float);
    }


    // FrameCount
    memcpy(&(sendDataBuf[at]), &g_simulationCount, sizeof(int));
    at += sizeof(int);


    return at;
}


int SystemThread::SetSendDataForFuncExtend(int maxAgentSend,int maxTSSend,QList<int> pivotIDs)
{
    // set agent and TS data to UE4
    memset( sendDataBuf, 0, sendDataMaxSize );

    sendDataBuf[0] = 'R';
    sendDataBuf[1] = 'S';
    sendDataBuf[2] = 'd';

    int at = 3;

    int numAgentSend = 0;

    QList<int> sendIDList;

    // count number of data

    for(int i=0;i<maxAgent;++i){
        if( agent[i]->agentStatus != 1 ){
            continue;
        }

        if( numAgentSend < maxAgentSend ){
           sendIDList.append(i);
        }
        numAgentSend++;
    }

//    qDebug() << "numAgentSend = " << numAgentSend << " maxAgentSend = " << maxAgentSend;

    if( numAgentSend > maxAgentSend ){

        int countByDistance[20];   // 20[m] * 20 = 400[m]
        for(int i=0;i<20;++i){
            countByDistance[i] = 0;
        }

        QList<int> calDist;

        numAgentSend = 0;
        for(int i=0;i<maxAgent;++i){

            calDist.append( -1 );

            if( agent[i]->agentStatus != 1 ){
                continue;
            }

            if( pivotIDs.contains(i) == true ){
                calDist[i] = 0;
                countByDistance[0]++;
                continue;
            }

            int idx = -1;
            for(int j=0;j<pivotIDs.size();++j){
                float rx = agent[i]->state.x - agent[pivotIDs[j]]->state.x;
                float ry = agent[i]->state.y - agent[pivotIDs[j]]->state.y;
                float D = sqrt(rx * rx + ry * ry);
                if( D > 400.0 ){
                    continue;
                }
                int tidx = (int)(D / 20.0);
                if( tidx >=20 ){
                    tidx = 19;
                }
                else if( tidx < 0 ){
                    tidx = 0;
                }
                if( idx < 0 || idx > tidx ){
                    idx = tidx;
                }
            }
            if( idx < 0 ){
                continue;
            }

            calDist[i] = idx;

            countByDistance[idx]++;
        }

        int totalNum = 0;
        int thrIdx = 0;
        for(int i=0;i<20;++i){
            thrIdx = i;
            if( totalNum < maxAgentSend && totalNum + countByDistance[i] >= maxAgentSend ){
                break;
            }
            else{
                totalNum += countByDistance[i];
            }
        }

//        qDebug() << "thrIdx = " << thrIdx;
//        for(int i=0;i<thrIdx;++i){
//            qDebug() << " countByDistance[" << i << "] = " << countByDistance[i];
//        }

        numAgentSend = 0;
        sendIDList.clear();

        for(int i=0;i<=thrIdx;++i){
            for(int j=0;j<maxAgent;++j){

                if( calDist[j] != i ){
                    continue;
                }

                if( numAgentSend < maxAgentSend ){
                    sendIDList.append( j );
                    numAgentSend++;
                }
                else{
                    break;
                }
            }
        }
    }

    numAgentSend = sendIDList.size();
//    qDebug() << "sendIDList = " << sendIDList;


    memcpy(&(sendDataBuf[at]), &numAgentSend, sizeof(int));   // 3
    at += sizeof(int);

    float tempFVal = 0.0;
    int   tempIVal = 0;
    char  tempCVal = 0;

    int maxSize = 125 * maxAgentSend + 13 * maxTSSend + 20 + 5;

    for(int n=0;n<numAgentSend;++n){

        int i = sendIDList[n];

        if( at + 125 < maxSize ){

            tempCVal = 'a';
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 7
            at += sizeof(char);

            tempIVal = agent[i]->ID;
            memcpy(&(sendDataBuf[at]), &tempIVal, sizeof(int));   // 8
            at += sizeof(int);

            tempFVal = agent[i]->state.x;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 12
            at += sizeof(float);

            tempFVal = agent[i]->state.y * (-1.0);
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 16
            at += sizeof(float);

            tempFVal = agent[i]->state.z;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 20
            at += sizeof(float);

            tempFVal = agent[i]->state.roll;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 24
            at += sizeof(float);

            tempFVal = agent[i]->state.pitch;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 28
            at += sizeof(float);

            tempFVal = agent[i]->state.yaw * (-1.0);
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 32
            at += sizeof(float);

            tempFVal = agent[i]->state.V;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 36
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.ax;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 40
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.yawRate;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 44
            at += sizeof(float);

            tempFVal = agent[i]->memory.lateralDeviationFromTargetPath;   // latDev
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 48
            at += sizeof(float);

            tempFVal = agent[i]->memory.lateralDeviationFromTargetPathAtPreviewPoint;   // latDev at aim point
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 52
            at += sizeof(float);

            tempCVal = 0;     // engineKeyState
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 56
            at += sizeof(char);

            tempCVal = 0;     // gearPosition
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 57
            at += sizeof(char);

            tempIVal = agent[i]->UE4ObjectID[pivotIDs.first()];     // Tachometer ---> UE4ID
            memcpy(&(sendDataBuf[at]), &tempIVal, sizeof(int));   // 58
            at += sizeof(int);

//            qDebug() << "[FE]ID=" << agent[i]->ID << " objID=" << tempIVal;


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

            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 62
            at += sizeof(char);

            tempCVal = 0;     // lightFlag2
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 63
            at += sizeof(char);

            if( agent[i]->agentKind < 100 ){
                tempFVal = agent[i]->vehicle.param.Lf;
            }
            else{
                tempFVal = 0.0;
            }
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 64
            at += sizeof(float);

            if( agent[i]->agentKind < 100 ){
                tempFVal = agent[i]->vehicle.param.Lr;
            }
            else{
                tempFVal = 0.0;
            }
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 68
            at += sizeof(float);

            if( agent[i]->agentKind < 100 ){
                tempFVal = 1.8;     // Tf
            }
            else{
                tempFVal = 0.0;
            }
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 72
            at += sizeof(float);

            if( agent[i]->agentKind < 100 ){
                tempFVal = 1.8;     // Tr
            }
            else{
                tempFVal = 0.0;
            }
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 76
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.tireRotAngle;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 80
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.steer * (-57.3 / 6.0) ;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 84
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.tireRotAngle;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 88
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.steer * (-57.3 / 6.0);
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 92
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.tireRotAngle;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 96
            at += sizeof(float);

            tempFVal = 0.0;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 100
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.tireRotAngle;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 104
            at += sizeof(float);

            tempFVal = 0.0;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 108
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.relZBody;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 112
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.relRollBody;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 116
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.relPitchBody;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 120
            at += sizeof(float);

            tempFVal = agent[i]->vehicle.state.relYBody;
            memcpy(&(sendDataBuf[at]), &tempFVal, sizeof(float));   // 124
            at += sizeof(float);

            tempCVal = 0;     // optionalFlag1
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 128
            at += sizeof(char);

            tempCVal = 0;     // optionalFlag2
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 129
            at += sizeof(char);

            tempCVal = 0;     // optionalFlag3
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 130
            at += sizeof(char);

            tempCVal = 'v';     // optionalFlag4
            if( agent[i]->agentStatus >= 100 ){
                tempCVal = 'p';
            }
            memcpy(&(sendDataBuf[at]), &tempCVal, sizeof(char));   // 131
            at += sizeof(char);
        }
    }


    int numTStSend = 0;

    QList<int> sendTSList;

    if( numTrafficSignals <= maxTSSend ){

        numTStSend = numTrafficSignals;
        for(int i=0;i<numTrafficSignals;++i){
            sendTSList.append( i );
        }

    }
    else{

        int countByDistance[20];   // 50[m] * 20 = 1000[m]
        for(int i=0;i<20;++i){
            countByDistance[i] = 0;
        }

        QList<int> calDist;

        for(int i=0;i<numTrafficSignals;++i){

            calDist.append( -1 );

            int idx = -1;
            for(int j=0;j<pivotIDs.size();++j){
                float rx = trafficSignal[i]->xTS - agent[pivotIDs[j]]->state.x;
                float ry = trafficSignal[i]->yTS - agent[pivotIDs[j]]->state.y;
                float D = sqrt(rx * rx + ry * ry);
                int tidx = (int)(D / 50.0);
                if( tidx >=20 ){
                    tidx = 19;
                }
                else if( tidx < 0 ){
                    tidx = 0;
                }
                if( idx < 0 || idx > tidx ){
                    idx = tidx;
                }
            }

            calDist[i] = idx;

            countByDistance[idx]++;
        }

        int totalNum = 0;
        int thrIdx = 0;
        for(int i=0;i<20;++i){

            thrIdx = i;

            if( totalNum < maxTSSend && totalNum + countByDistance[i] >= maxTSSend ){
                break;
            }
            else{
                totalNum += countByDistance[i];
            }
        }

        numTStSend = 0;
        sendTSList.clear();

        for(int i=0;i<numTrafficSignals;++i){

            if( calDist[i] > thrIdx ){
                continue;
            }

            if( numTStSend < maxTSSend ){
                sendTSList.append( i );
                numTStSend++;
            }
            else{
                break;
            }
        }
    }

    numTStSend = sendTSList.size();

    memcpy(&(sendDataBuf[at]), &numTStSend, sizeof(int));
    at += sizeof(int);

    for(int n=0;n<numTStSend;++n){

        int i = sendTSList[n];

        memcpy(&(sendDataBuf[at]), &(trafficSignal[i]->id), sizeof(int));
        at += sizeof(int);

        sendDataBuf[at] = trafficSignal[i]->type;
        at += sizeof(char);

        int display = trafficSignal[i]->GetCurrentDisplayInfo();
        memcpy(&(sendDataBuf[at]), &display, sizeof(int));
        at += sizeof(int);

        float remainTime = trafficSignal[i]->remainingTimeToNextDisplay;
        memcpy(&(sendDataBuf[at]), &remainTime, sizeof(float));
        at += sizeof(float);
    }


    // FrameCount
    memcpy(&(sendDataBuf[at]), &g_simulationCount, sizeof(int));
    at += sizeof(int);

    return at;
}


void SystemThread::SetSimulationFrequency(int hz)
{
    qDebug() << "[SetSimulationFrequency] SetFrequency(" << hz << ")";
    calHz = hz;
    intervalMSec = (int)(1000.0 / calHz);
    simManage->SetFrequency( hz );
}




void SystemThread::SetTireHeight(int SInterObjID, int id, float zFL, float zFR, float zRL, float zRR)
{
    if( SInterObjID < 0 || SInterObjID >= 10 ){
        return;
    }

    int idx = -1;
    for(int i=0;i<maxAgent;++i){
        if( agent[i]->agentStatus != 1 ){
            continue;
        }
        if( agent[i]->agentKind >= 100 ){
            continue;
        }
        if( agent[i]->UE4ObjectID[SInterObjID] == id ){
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


void SystemThread::SetSInterInitState(char type, int id, float xi, float yi,float zi,float yawi,float wheelbase)
{
    if( id < 0 || id >= maxAgent ){
        return;
    }

    if( type == 'v' || type == 'm' ){
        agent[id]->agentKind = 0;
    }
    else if( type == 'p' || type == 'b' ){
        agent[id]->agentKind = 100;
    }

    agent[id]->ID = id;
    agent[id]->isSInterfaceObject = true;
    agent[id]->isSInterObjDataSet = false;

    agent[id]->state.x = xi;
    agent[id]->state.y = yi;
    agent[id]->state.z = zi;

    agent[id]->state.roll  = 0.0;
    agent[id]->state.pitch = 0.0;
    agent[id]->state.yaw   = yawi;

    agent[id]->agentStatus = 1;

    if( agent[id]->vehicle.GetVehicleModelID() < 0 ){
        int vModelID = simManage->GetVehicleShapeByWheelbase( wheelbase, road );
        agent[id]->vehicle.SetVehicleModelID( vModelID );
    }
}

void SystemThread::SetSInterObjDataToBuffer(char type, int id, AgentState *as,struct SInterObjInfo *so)
{
    int idx = -1;
    for(int i=0;i<nSIObject;++i){
        if( sov[i].objID == id ){
            idx = i;
            break;
        }
    }
    if( idx < 0 ){
        for(int i=0;i<nSIObject;++i){
            if( sov[i].objID < 0 ){
                idx = i;
                break;
            }
        }
    }
    if( idx < 0 ){
        return;
    }

    memcpy( &(sov[idx]) , so, sizeof(struct SInterObjInfo) );
    memcpy( &(asv[idx]), as, sizeof(struct AgentState) );

    sov[idx].objID = id;
    if( type == 'v' || type == 'm' ){
        sov[idx].objType = 0;
    }
    else if( type == 'p' || type == 'b' ){
        sov[idx].objType = 100;
    }
}


void SystemThread::SetSInterObjData(char type, int id, AgentState *as,struct SInterObjInfo *so)
{
//    mutexDSStateWrite->lock();

    //qDebug() << "SetSInterObjData: id = " << id;
    if( id < 0 || id >= maxAgent ){
        return;
    }

    if( type == 'v' || type == 'm' ){
        agent[id]->agentKind = 0;
    }
    else if( type == 'p' || type == 'b' ){
        agent[id]->agentKind = 100;
    }

    agent[id]->ID = id;

    agent[id]->isSInterfaceObject = true;
    agent[id]->isSInterObjDataSet = true;

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
    agent[id]->state.steer = as->steer * 57.3;  // [rad] -> [deg]

    agent[id]->state.accel_log = as->accel_log;
    agent[id]->state.brake_log = as->brake_log;
    agent[id]->state.steer_log = as->steer_log* 57.3;  // [rad] -> [deg]

    agent[id]->agentStatus = 1;

    //qDebug() << "SetSInterObjData : id = " << id;

    //qDebug() << " x = " << agent[id]->state.x;
    //qDebug() << " y = " << agent[id]->state.y;




    if( agent[id]->vehicle.GetVehicleModelID() < 0 ){
        float wheelbase = so->lf + so->lr;
        int vModelID = simManage->GetVehicleShapeByWheelbase( wheelbase, road );
        agent[id]->vehicle.SetVehicleModelID( vModelID );

        //qDebug() << "vModel = " << vModelID;

        agent[id]->vHalfLength = agent[id]->vehicle.GetVehicleLength() * 0.5;
        agent[id]->vHalfWidth  = agent[id]->vehicle.GetVehicleWidth()  * 0.5;
    }

    agent[id]->vehicle.param.Lf = so->lr;
    agent[id]->vehicle.param.Lr = so->lr;


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


    int pathId = -1;

    float deviation,xt,yt,xd,yd,s;

    int oldTargetPath = agent[id]->memory.currentTargetPath;
    if( oldTargetPath >= 0 ){

        int tpID = road->GetDeviationFromPath( oldTargetPath,
                                               agent[id]->state.x, agent[id]->state.y, agent[id]->state.yaw,
                                               deviation, xt, yt, xd, yd, s );

        if( tpID == oldTargetPath && fabs(deviation) < 1.5 ){
            pathId = tpID;
        }
        else{
            if( winkerVal == 0 && agent[id]->memory.targetPathList.size() > 1 ){

                for(int i=agent[id]->memory.targetPathList.size()-1;i>=0;i--){

                    tpID = road->GetDeviationFromPath( agent[id]->memory.targetPathList[i],
                                                           as->x, as->y, as->yaw,
                                                           deviation, xt, yt, xd, yd, s );

                    if( tpID != agent[id]->memory.targetPathList[i] || isnan(deviation) == true || fabs(deviation) > 1.0  ){
                        continue;
                    }

                    pathId = agent[id]->memory.targetPathList[i];
                    break;
                }
            }
        }
    }

    if( pathId < 0 ){
        pathId = road->GetNearestPath( as->x, as->y, as->yaw, deviation, xt, yt, xd, yd, s );
    }

//    qDebug() << "S-Interface Object: near path = " << pathId;

    agent[id]->memory.precedingVehicleID = -1;
    agent[id]->memory.distanceToNodeWPIn = 0.0;
    agent[id]->memory.distanceToStopPoint = 0.0;
    agent[id]->memory.distanceToZeroSpeed = as->V * as->V * 0.5f / (0.3 * 9.81) + 5.0;
    agent[id]->memory.requiredDistToStopFromTargetSpeed = 150.0;
    agent[id]->memory.nextTurnNode = -1;


    if( pathId < 0 ){

        agent[id]->memory.currentTargetPath = -1;
        agent[id]->memory.currentTargetPathIndexInList = -1;

        agent[id]->memory.lateralDeviationFromTargetPath = 0.0;
        agent[id]->memory.distanceFromStartWPInCurrentPath = 0.0;

        agent[id]->memory.lateralDeviationFromTargetPathAtPreviewPoint = 0.0;
        agent[id]->memory.previewPointPath = -1;

        agent[id]->memory.currentTargetNode = -1;
        agent[id]->memory.currentTargetNodeIndexInNodeList = -1;

    }
    else if( pathId >= 0 ){

        int cpath = pathId;
        int idx = road->pathId2Index.indexOf(cpath);

        //
        //  Set Target Path
        //
        agent[id]->memory.currentTargetPath = pathId;
        agent[id]->memory.lateralDeviationFromTargetPath = deviation;
        agent[id]->memory.distanceFromStartWPInCurrentPath = s;


        agent[id]->state.z_path = agent[id]->state.z;

        if( idx >= 0 ){

            float z = road->paths[idx]->pos.first()->z();
            if( agent[id]->memory.distanceFromStartWPInCurrentPath >= 0 &&
                    agent[id]->memory.distanceFromStartWPInCurrentPath <= road->paths[idx]->pathLength ){
                float dz = (road->paths[idx]->pos.last()->z() - z) * agent[id]->memory.distanceFromStartWPInCurrentPath / road->paths[idx]->pathLength;
                z += dz;
            }

            agent[id]->state.z_path = z;
        }


//        qDebug() << "deviation = " << deviation;


        //
        //  Set requiredDistToStopFromTargetSpeed
        //
        float targetSpeed = road->paths[idx]->speed85pt;
        agent[id]->memory.requiredDistToStopFromTargetSpeed = targetSpeed * targetSpeed * 0.5f / 2.943 + 5.0;


        //
        //  Set Target Node
        //
        agent[id]->memory.currentTargetNode = road->paths[idx]->connectingNode;
        agent[id]->memory.currentTargetNodeIndexInNodeList = -1;

        if( winkerVal == 1 || winkerVal == 2 ){
            agent[id]->memory.nextTurnNode = agent[id]->memory.currentTargetNode;
        }


        //
        // Set Target Path List
        //
        agent[id]->memory.targetPathList.clear();
        agent[id]->memory.targetPathList.append( pathId );

        float totalLen = 0.0;
        int tIdx = idx;
        while(1){
            int nNp = road->paths[tIdx]->forwardPaths.size();
            if( nNp == 0 ){
                break;
            }
            else{
                if( nNp == 1 ){
                    cpath = road->paths[tIdx]->forwardPaths[0];
                    agent[id]->memory.targetPathList.prepend( cpath );
                }
                else{

                    // select path with minimum curvature
                    int minIdx = -1;
                    float minCurvature = 0.0;

                    for(int j=0;j<nNp;++j){

                        int tpath = road->paths[tIdx]->forwardPaths[j];
                        int fidx = road->pathId2Index.indexOf(tpath);

                        float tmpCurvature = fabs(road->paths[fidx]->meanPathCurvature);
                        float tmpMaxCurvature = fabs(road->paths[fidx]->maxPathCurvature);

                        if( minIdx < 0 || (minCurvature > tmpCurvature && minCurvature > tmpMaxCurvature) ){
                            minIdx = j;
                            minCurvature = (tmpCurvature < tmpMaxCurvature ? tmpMaxCurvature : tmpCurvature);
                        }

                    }

                    cpath = road->paths[tIdx]->forwardPaths[minIdx];
                    agent[id]->memory.targetPathList.prepend( cpath );
                }

                totalLen += road->paths[tIdx]->pathLength;

                if( totalLen > 400.0 ){
                    break;
                }

                tIdx = road->pathId2Index.indexOf(cpath);
            }
        }

        agent[id]->memory.currentTargetPathIndexInList = agent[id]->memory.targetPathList.indexOf( agent[id]->memory.currentTargetPath );


        agent[id]->memory.targetPathLength.clear();
        for(int n=0;n<agent[id]->memory.targetPathList.size();++n){
            float len = road->GetPathLength( agent[id]->memory.targetPathList[n] );
            agent[id]->memory.targetPathLength.append( len );
        }


        //
        // Set lateralDeviationFromTargetPathAtPreviewPoint
        //
        float preview_dist = as->V;
        if( preview_dist < 5.0 ){
            preview_dist = 5.0;
        }

        float preview_x = as->x + preview_dist * cos( as->yaw );
        float preview_y = as->y + preview_dist * sin( as->yaw );


        agent[id]->memory.lateralDeviationFromTargetPathAtPreviewPoint = 0.0;
        agent[id]->memory.previewPointPath = -1;

        for(int i=agent[id]->memory.targetPathList.size()-1;i>=0;i--){

            float tdev,txt,tyt,txd,tyd,ts;
            int chk = road->GetDeviationFromPath( agent[id]->memory.targetPathList[i],
                                                   preview_x, preview_y, as->yaw,
                                                   tdev, txt, tyt, txd, tyd, ts, true );

            if( chk != agent[id]->memory.targetPathList[i] ){
                continue;
            }
            if( isnan(tdev) == true ){
                agent[id]->memory.lateralDeviationFromTargetPathAtPreviewPoint = 0.0;
                break;
            }

            agent[id]->memory.lateralDeviationFromTargetPathAtPreviewPoint = tdev;
            agent[id]->memory.previewPointPath = agent[id]->memory.targetPathList[i];
            break;
        }


//        qDebug() << "targetPathList = " << agent[id]->memory.targetPathList;
//        qDebug() << "currentTargetPath = " << agent[id]->memory.currentTargetPath << " dev= " << agent[id]->memory.lateralDeviationFromTargetPath;
//        qDebug() << "previewPointPath = " << agent[id]->memory.previewPointPath << " dev= " << agent[id]->memory.lateralDeviationFromTargetPathAtPreviewPoint;


        //
        // Set Node List
        //
        int inDir = road->paths[idx]->connectingNodeInDir;
        int lastNode = -1;
        int lastNodeOut = -1;
        int tNdIdx = road->nodeId2Index.indexOf( agent[id]->memory.currentTargetNode );
        for(int i=0;i<road->nodes[tNdIdx]->nodeConnectInfo.size();++i){
            if( road->nodes[tNdIdx]->nodeConnectInfo[i]->inDirectionID == inDir ){
                lastNode = road->nodes[tNdIdx]->nodeConnectInfo[i]->connectedNode;
                lastNodeOut = road->nodes[tNdIdx]->nodeConnectInfo[i]->outDirectionID;
                break;
            }
        }

        agent[id]->memory.myNodeList.clear();
        agent[id]->memory.myInDirList.clear();
        agent[id]->memory.myOutDirList.clear();


        agent[id]->memory.myNodeList.append( lastNode );
        agent[id]->memory.myInDirList.append( -1 );
        agent[id]->memory.myOutDirList.append( lastNodeOut );

        agent[id]->memory.myNodeList.append( agent[id]->memory.currentTargetNode );
        agent[id]->memory.myInDirList.append( inDir );

        int cNode = agent[id]->memory.currentTargetNode;
        for(int n=agent[id]->memory.currentTargetPathIndexInList;n>=0;n--){
            int tpath = agent[id]->memory.targetPathList[n];
            int lidx = road->pathId2Index.indexOf( tpath );

            if( cNode != road->paths[lidx]->connectingNode ){

                tNdIdx = road->nodeId2Index.indexOf( road->paths[lidx]->connectingNode );
                inDir = road->paths[lidx]->connectingNodeInDir;

                lastNodeOut = -1;
                for(int i=0;i<road->nodes[tNdIdx]->nodeConnectInfo.size();++i){
                    if( road->nodes[tNdIdx]->nodeConnectInfo[i]->inDirectionID == inDir ){
                        lastNodeOut = road->nodes[tNdIdx]->nodeConnectInfo[i]->outDirectionID;
                        break;
                    }
                }
                agent[id]->memory.myOutDirList.append( lastNodeOut );

                cNode = road->paths[lidx]->connectingNode;

                agent[id]->memory.myNodeList.append( cNode );
                agent[id]->memory.myInDirList.append( inDir );
            }
        }
        agent[id]->memory.myOutDirList.append( -1 );

        agent[id]->memory.currentTargetNodeIndexInNodeList = agent[id]->memory.myNodeList.indexOf( agent[id]->memory.currentTargetNode );

//        qDebug() << "targetPathList = " << agent[id]->memory.targetPathList;
//        qDebug() << "myNodeList = " << agent[id]->memory.myNodeList;
//        qDebug() << "myInDirList = " << agent[id]->memory.myInDirList;
//        qDebug() << "myOutDirList = " << agent[id]->memory.myOutDirList;



        //
        // Set distanceToWPIn
        //
        if( agent[id]->memory.currentTargetNodeIndexInNodeList >= 0 &&
                agent[id]->memory.currentTargetNodeIndexInNodeList < agent[id]->memory.myInDirList.size() ){

            QList<int> destPathsIn;

            int inDir = agent[id]->memory.myInDirList.at( agent[id]->memory.currentTargetNodeIndexInNodeList );

            int tnIdx = road->nodeId2Index.indexOf( agent[id]->memory.currentTargetNode );
            for(int i=0;i<road->nodes[tnIdx]->inBoundaryWPs.size();++i){
                if( road->nodes[tnIdx]->inBoundaryWPs[i]->relatedDirection == inDir ){
                    for(int j=0;j<road->nodes[tnIdx]->inBoundaryWPs[i]->PathWithEWP.size();++j){
                        destPathsIn.append( road->nodes[tnIdx]->inBoundaryWPs[i]->PathWithEWP[j] );
                    }
                }
            }

            float dist = 0.0;
            for(int i=agent[id]->memory.currentTargetPathIndexInList;i>=0;i--){
                int pIdx = road->pathId2Index.indexOf( agent[id]->memory.targetPathList.at(i) );
                dist += road->paths[pIdx]->pathLength;
                if( road->paths[pIdx]->connectingNode == agent[id]->memory.currentTargetNode ){
                    if( destPathsIn.indexOf(agent[id]->memory.targetPathList.at(i)) >= 0 ){
                        agent[id]->memory.distanceToNodeWPIn = dist - agent[id]->memory.distanceFromStartWPInCurrentPath;
                        break;
                    }
                }
            }
        }

        //
        // Set distanceToStopPoint
        //
        float dist = 0.0;
        for(int i=agent[id]->memory.currentTargetPathIndexInList;i>=0;i--){
            int tpath = agent[id]->memory.targetPathList[i];
            int lidx = road->pathId2Index.indexOf( tpath );
            if( road->paths[lidx]->stopPoints.size() > 0 ){
                dist += road->paths[lidx]->stopPoints[0]->distFromStartWP;
                break;
            }
            else{
                dist += road->paths[lidx]->pathLength;
            }
        }
        agent[id]->memory.distanceToStopPoint = dist - agent[id]->memory.distanceFromStartWPInCurrentPath;

    }
//    mutexDSStateWrite->unlock();
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
        "UNDEFINED              ",
        "PEDESTRIAN             ",
    };


    int nearID = -1;
    float dist = 0.0;
    for(int i=0;i<maxAgent;++i){
//        if(agent[i]->agentStatus != 1 ){
//            continue;
//        }

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

    if( agent[nearID]->agentStatus != 1 ){

        qDebug() << "Picked Agent ID = " << agent[nearID]->ID
                 << " agentStaus = " << agent[nearID]->agentStatus;
        return;
    }


    qDebug() << "Picked Agent ID = " << agent[nearID]->ID
             << " : agentKind = " << agent[nearID]->agentKind
             << " vHalfLength = " << agent[nearID]->vHalfLength
             << " vHalfWidth = " << agent[nearID]->vHalfWidth
             << " modelID = " << agent[nearID]->vehicle.GetVehicleModelID();

    qDebug() << "Vehicle State:";
    qDebug() << "  X = " << agent[nearID]->state.x;
    qDebug() << "  Y = " << agent[nearID]->state.y;
    qDebug() << "  Z = " << agent[nearID]->state.z;
    qDebug() << "  Yaw = " << agent[nearID]->state.yaw;
    qDebug() << "  V = " << agent[nearID]->state.V;
    qDebug() << "  Accel = " << agent[nearID]->vehicle.input.accel;
    qDebug() << "  Winker = " << agent[nearID]->vehicle.GetWinerState();

    qDebug() << "Recognizied Object Info:";
    for(int i=0;i<agent[nearID]->memory.perceptedObjects.size();++i){
        if( agent[nearID]->memory.perceptedObjects[i]->isValidData == false ){
            continue;
        }
        qDebug() << "  Agent:" << agent[nearID]->memory.perceptedObjects[i]->objectID
                 << " Label=" << labelStr[ agent[nearID]->memory.perceptedObjects[i]->recognitionLabel ]
                 << " Dist=" << agent[nearID]->memory.perceptedObjects[i]->distanceToObject
                 << " e = " << agent[nearID]->memory.perceptedObjects[i]->deviationFromNearestTargetPath
                 << " W = " << agent[nearID]->memory.perceptedObjects[i]->effectiveHalfWidth
                 << " Type = " << agent[nearID]->memory.perceptedObjects[i]->objectType
                 << " Evaled = " << agent[nearID]->memory.perceptedObjects[i]->relPosEvaled;

        qDebug() << "        N = " << agent[nearID]->memory.perceptedObjects[i]->innerProductToNearestPathNormal
                 << " T = " << agent[nearID]->memory.perceptedObjects[i]->innerProductToNearestPathTangent
                 << " nupc = " << agent[nearID]->memory.perceptedObjects[i]->noUpdateCount
                 << " nearPath = " << agent[nearID]->memory.perceptedObjects[i]->nearestTargetPath
                 << " t = " << agent[nearID]->memory.perceptedObjects[i]->objDistFromSWPOfNearTargetPath
                 << " objPath = " << agent[nearID]->memory.perceptedObjects[i]->objectPath;

        if( agent[nearID]->memory.checkSideVehicleForLC == true ){
            qDebug() << "        LCpath = " << agent[nearID]->memory.perceptedObjects[i]->objPathInLCTargetPathList
                     << " e = " << agent[nearID]->memory.perceptedObjects[i]->latDevObjInLaneChangeTargetPathList
                     << " D = " << agent[nearID]->memory.perceptedObjects[i]->distToObjInLaneChangeTargetPathList;
        }
    }

    qDebug() << "Recognizied Traffic Signal Info:";
    for(int i=0;i<agent[nearID]->memory.perceptedSignals.size();++i){
        if(agent[nearID]->memory.perceptedSignals[i]->isValidData == false ){
            continue;
        }
        qDebug() << "  TS:" << agent[nearID]->memory.perceptedSignals[i]->objectID
                 << " Type=" << agent[nearID]->memory.perceptedSignals[i]->objectType
                 << " Disp = " << agent[nearID]->memory.perceptedSignals[i]->signalDisplay
                 << " SL = " << agent[nearID]->memory.perceptedSignals[i]->SLonPathID
                 << " Dist = " << agent[nearID]->memory.perceptedSignals[i]->distToSL;
    }

    qDebug() << "Collision/Merging Point Info:";
    for(int i=0;i<agent[nearID]->memory.perceptedObjects.size();++i){
        if( agent[nearID]->memory.perceptedObjects[i]->isValidData == false ){
            continue;
        }
        if( agent[nearID]->memory.perceptedObjects[i]->hasCollisionPoint == true ){
            qDebug() << "  OBJ:" << agent[nearID]->memory.perceptedObjects[i]->objectID
                     << " hasCP = " << agent[nearID]->memory.perceptedObjects[i]->hasCollisionPoint
                     << " merge = " << agent[nearID]->memory.perceptedObjects[i]->mergingAsCP;
            if( agent[nearID]->memory.perceptedObjects[i]->hasCollisionPoint == true || agent[nearID]->memory.perceptedObjects[i]->mergingAsCP == true ){
                qDebug() << "    xCP = " << agent[nearID]->memory.perceptedObjects[i]->xCP
                         << " yCP = " << agent[nearID]->memory.perceptedObjects[i]->yCP;
                qDebug() << "    myDist = " << agent[nearID]->memory.perceptedObjects[i]->myDistanceToCP
                         << " myTime = " << agent[nearID]->memory.perceptedObjects[i]->myTimeToCP;
                qDebug() << "    objDist = " << agent[nearID]->memory.perceptedObjects[i]->objectDistanceToCP
                         << " objTime = " << agent[nearID]->memory.perceptedObjects[i]->objectTimeToCP;
            }
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
    qDebug() << "  nearCPInNode = " << agent[nearID]->memory.nearCPInNode;
    qDebug() << "  shouldStopAtSignalSL = " << agent[nearID]->memory.shouldStopAtSignalSL;
    qDebug() << "  shouldYeild = " << agent[nearID]->memory.shouldYeild;
    qDebug() << "  distToYeildStopLine = " << agent[nearID]->memory.distToYeildStopLine;
    qDebug() << "  leftCrossIsClear = " << agent[nearID]->memory.leftCrossIsClear;
    qDebug() << "  leftCrossCheckCount = " << agent[nearID]->memory.leftCrossCheckCount;
    qDebug() << "  rightCrossIsClear = " << agent[nearID]->memory.rightCrossIsClear;
    qDebug() << "  rightCrossCheckCount = " << agent[nearID]->memory.rightCrossCheckCount;
    qDebug() << "  safetyConfimed = " << agent[nearID]->memory.safetyConfimed;
    qDebug() << "  hazardusObject = " << agent[nearID]->memory.hazardusObject;
    qDebug() << "  lastHazardusObject = " << agent[nearID]->memory.lastHazardusObject;
    qDebug() << "  ignoreHazardusObject = " << agent[nearID]->memory.ignoreHazardusObject;

    qDebug() << "Lane-Change:";
    qDebug() << "  checkSideVehicleForLC = " << agent[nearID]->memory.checkSideVehicleForLC;
    qDebug() << "  LCDirection = " << agent[nearID]->memory.LCDirection;
    qDebug() << "  LCCheckState = " << agent[nearID]->memory.LCCheckState;
    qDebug() << "  LCInfoGetCount = " << agent[nearID]->memory.LCInfoGetCount;
    qDebug() << "  currentPathInLCTargetPathList = " << agent[nearID]->memory.currentPathInLCTargetPathList;
    qDebug() << "  latDeviFromLCTargetPathList = " << agent[nearID]->memory.latDeviFromLCTargetPathList;
    qDebug() << "  distFromSWPLCTargetPathList = " << agent[nearID]->memory.distFromSWPLCTargetPathList;


    qDebug() << "Control Info:";
    qDebug() << "  controlMode = " << agent[nearID]->memory.controlMode;
    qDebug() << "  accel = " << agent[nearID]->memory.accel;
    qDebug() << "  brake = " << agent[nearID]->memory.brake;

    qDebug() << "  targetSpeed = " << agent[nearID]->memory.targetSpeed;
    qDebug() << "  setTargetSpeedByScenarioFlag = " << agent[nearID]->memory.setTargetSpeedByScenarioFlag;
    qDebug() << "  targetSpeedByScenario = " << agent[nearID]->memory.targetSpeedByScenario;
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
    qDebug() << "  distanceToZeroSpeedByMaxBrake = " << agent[nearID]->memory.distanceToZeroSpeedByMaxBrake;
    qDebug() << "  requiredDistToStopFromTargetSpeed = " << agent[nearID]->memory.requiredDistToStopFromTargetSpeed;

    qDebug() << "  distanceFromStartWPInCurrentPath = " << agent[nearID]->memory.distanceFromStartWPInCurrentPath;
    qDebug() << "  lateralDeviationFromTargetPath = " << agent[nearID]->memory.lateralDeviationFromTargetPath;
    qDebug() << "  lateralDeviationFromTargetPathAtPreviewPoint = " << agent[nearID]->memory.lateralDeviationFromTargetPathAtPreviewPoint;
    qDebug() << "  targetLateralShiftByScenario = " << agent[nearID]->memory.targetLateralShiftByScenario;
    qDebug() << "  relativeAttitudeToLane = " << agent[nearID]->memory.relativeAttitudeToLane;

    qDebug() << "  lateralShiftTarget = " << agent[nearID]->memory.lateralShiftTarget;
    qDebug() << "  lateralShiftTarget_Backup = " << agent[nearID]->memory.lateralShiftTarget_backup;
    qDebug() << "  avoidTarget = " << agent[nearID]->memory.avoidTarget;
    qDebug() << "  steer = " << agent[nearID]->memory.steer;

    if( agent[nearID]->agentKind >= 100 ){
        qDebug() << "  exeRunOut = " << agent[nearID]->memory.exeRunOut;
        qDebug() << "  runOutState = " << agent[nearID]->memory.runOutState;
    }

    qDebug() << "Guidance Info:";
    qDebug() << "  targetPathList = " << agent[nearID]->memory.targetPathList;
    qDebug() << "  currentPath = " << agent[nearID]->memory.currentTargetPath;
    qDebug() << "  currentTargetPathIndexInList = " << agent[nearID]->memory.currentTargetPathIndexInList;

    if( agent[nearID]->agentKind >= 100 ){
        qDebug() << "  pedest-path width: " << road->GetPedestPathWidth( agent[nearID]->memory.currentTargetPath, agent[nearID]->memory.currentTargetPathIndexInList );
    }

    qDebug() << "  destinationNode = " << agent[nearID]->memory.destinationNode;
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
    qDebug() << "  distanceToTurnNodeWPOut = " << agent[nearID]->memory.distanceToTurnNodeWPOut;
    qDebug() << "  distanceToNodeWPOut = " << agent[nearID]->memory.distanceToNodeWPOut;

    qDebug() << "  routeIndex = " << agent[nearID]->memory.routeIndex;
    qDebug() << "  routeLaneIndex = " << agent[nearID]->memory.routeLaneIndex;
    qDebug() << "  LCStartRouteIndex = " << agent[nearID]->memory.LCStartRouteIndex;
    qDebug() << "  LCSupportRouteLaneIndex = " << agent[nearID]->memory.LCSupportRouteLaneIndex;


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
    qDebug() << "  pedestWaitPositionSafetyMargin = " << agent[nearID]->param.pedestWaitPositionSafetyMargin;
    qDebug() << "  safetyConfirmTime = " << agent[nearID]->param.safetyConfirmTime;
    qDebug() << "  LCInfoGetTime = " << agent[nearID]->param.LCInfoGetTime;
    qDebug() << "  LCCutInAllowTTC = " << agent[nearID]->param.LCCutInAllowTTC;
    qDebug() << "  vDevAllowPlus = " << agent[nearID]->param.vDevAllowPlus;
    qDebug() << "  vDevAllowMinus = " << agent[nearID]->param.vDevAllowMinus;
    qDebug() << "  accelAtVDev = " << agent[nearID]->param.accelAtVDev;
    qDebug() << "  decelAtVDev = " << agent[nearID]->param.decelAtVDev;


    qDebug() << "Debug Info:";

    QString debugStr = agent[nearID]->strForDebug + agent[nearID]->strForDebugRiskEval;

    QStringList divStr = debugStr.split("\n");
    for(int i=0;i<divStr.size();++i){
        qDebug() << "  " << QString(divStr[i]);
    }

    qDebug() << "cognitionCount = " << agent[nearID]->cognitionCount;
    qDebug() << "cognitionCountMax = " << agent[nearID]->cognitionCountMax;
    qDebug() << "decisionMakingCount = " << agent[nearID]->decisionMakingCount;
    qDebug() << "decisionMakingCountMax = " << agent[nearID]->decisionMakingCountMax;
    qDebug() << "controlCount = " << agent[nearID]->controlCount;
    qDebug() << "controlCountMax = " << agent[nearID]->controlCountMax;
    qDebug() << "calInterval = " << agent[nearID]->calInterval;

}


void SystemThread::SetTmpStopTime(int hour,int min,int sec)
{
    tmpStopHour   = hour;
    tmpStopMin    = min;
    tmpStopSecond = sec;
}


