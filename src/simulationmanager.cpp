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


#include "simulationmanager.h"
#include <windows.h>
#include <QDebug>
#include <QFile>


extern QFile sysLogOutFile;



SimulationManager::SimulationManager()
{
    simTime.day = 0;
    simTime.hour = 0;
    simTime.min = 0;
    simTime.sec = 0;
    simTime.msec = 0.0;
    simTime.msec_count = 0;
    simTime.exe_freq = 50;  // Default 50[Hz]
    simTime.dt = 0.02;

    DSMode = false;
    currentScenarioID = 0;
    IDAllowed = -1;

    udpthread = NULL;

    meanSpeedPedestrian[0] = 1.339;
    meanSpeedPedestrian[1] = 1.358;
    meanSpeedPedestrian[2] = 1.337;
}


void SimulationManager::ResetSimulationTime()
{
    simTime.day = 0;
    simTime.hour = 0;
    simTime.min = 0;
    simTime.sec = 0;
    simTime.msec = 0.0;
    simTime.msec_count = 0;
}


void SimulationManager::SetSimulationTime(int day,int hour,int minit,float sec)
{
    simTime.day = day;
    simTime.hour = hour;
    simTime.min = minit;

    int isec = (int)sec;

    simTime.sec = isec;
    simTime.msec = sec - (float)isec;
    simTime.msec_count = simTime.msec * simTime.exe_freq;
}


void SimulationManager::SetFrequency(int hz)
{
    simTime.exe_freq = hz;
    simTime.dt = 1.0f / simTime.exe_freq;
}


void SimulationManager::UpdateSimulationTime()
{
    simTime.msec_count++;
    if( simTime.msec_count >= simTime.exe_freq ){
        simTime.sec++;
        if( simTime.sec >= 60 ){
            simTime.min++;
            if( simTime.min >= 60 ){
                simTime.hour++;
                if( simTime.hour >= 24 ){
                    simTime.day++;
                    simTime.hour = 0;
                }
                simTime.min = 0;
            }
            simTime.sec = 0;
        }
        simTime.msec_count = 0;
    }
    simTime.msec = simTime.msec_count * simTime.dt;
}


QString SimulationManager::GetSimulationTimeStr()
{
    QString timeStr = QString("%1[d]:%2[h]:%3[m]:%4[s]").arg( simTime.day, 2, 10, QChar(' ') )
            .arg( simTime.hour, 2, 10, QChar(' ') ).arg( simTime.min, 2, 10, QChar(' ') )
            .arg( simTime.sec + simTime.msec, 6, 'f', 3, QChar(' ') );

    return timeStr;
}


float SimulationManager::GetSimulationTimeInSec()
{
    float ret = simTime.sec + simTime.msec + simTime.min * 60.0 + simTime.hour * 3600.0 + simTime.day * 86400.0;
    return ret;
}


int SimulationManager::GetSimulationTimeSecondAsInt()
{
    return simTime.sec;
}


int SimulationManager::GetVehicleShapeByWheelbase(float wl,Road *pRoad)
{
    int ret = -1;
    int mismatch = 0.0;
    for(int i=0;i<pRoad->vehicleKind.size();++i){
        float err = fabs(wl - pRoad->vehicleKind[i]->length);
        if(ret < 0 || mismatch > err ){
            ret = i;
            mismatch = err;
        }
    }
    return ret;
}



void SimulationManager::CopyScenarioData(int fromAID, int toAID)
{
    if( scenario.size() == 0 ){
        return;
    }
    int nItem = scenario[currentScenarioID]->scenarioItems.size();

    int fIdx = -1;
    int toIdx = -1;
    for(int i=0;i<nItem;++i){
        int objectID = scenario[currentScenarioID]->scenarioItems[i]->objectID;
        if( objectID == fromAID ){
            fIdx = i;
        }
        else if( objectID == toAID ){
            toIdx = i;
        }
    }

    if( fIdx < 0 || toIdx < 0 ){
        return;
    }

    scenario[currentScenarioID]->scenarioItems[toIdx]->repeat = scenario[currentScenarioID]->scenarioItems[fIdx]->repeat;
    scenario[currentScenarioID]->scenarioItems[toIdx]->status = scenario[currentScenarioID]->scenarioItems[fIdx]->status;

    scenario[currentScenarioID]->scenarioItems[toIdx]->initialState.clear();
    for(int i=0;i<scenario[currentScenarioID]->scenarioItems[fIdx]->initialState.size();++i){
        scenario[currentScenarioID]->scenarioItems[toIdx]->initialState.append( scenario[currentScenarioID]->scenarioItems[fIdx]->initialState[i] );
    }

    scenario[currentScenarioID]->scenarioItems[toIdx]->controlInfo->mode = scenario[currentScenarioID]->scenarioItems[fIdx]->controlInfo->mode;
    scenario[currentScenarioID]->scenarioItems[toIdx]->controlInfo->routeType = scenario[currentScenarioID]->scenarioItems[fIdx]->controlInfo->routeType;
    scenario[currentScenarioID]->scenarioItems[toIdx]->controlInfo->targetSpeed = scenario[currentScenarioID]->scenarioItems[fIdx]->controlInfo->targetSpeed;
    scenario[currentScenarioID]->scenarioItems[toIdx]->controlInfo->targetObjectID = scenario[currentScenarioID]->scenarioItems[fIdx]->controlInfo->targetObjectID;
    scenario[currentScenarioID]->scenarioItems[toIdx]->controlInfo->targetHeadwayTime = scenario[currentScenarioID]->scenarioItems[fIdx]->controlInfo->targetHeadwayTime;
    scenario[currentScenarioID]->scenarioItems[toIdx]->controlInfo->targetHeadwayDistance = scenario[currentScenarioID]->scenarioItems[fIdx]->controlInfo->targetHeadwayDistance;

    int nWP = scenario[currentScenarioID]->scenarioItems[toIdx]->controlInfo->wpRoute.size();
    for(int i=0;i<nWP;++i){
        delete scenario[currentScenarioID]->scenarioItems[toIdx]->controlInfo->wpRoute[i];
    }
    scenario[currentScenarioID]->scenarioItems[toIdx]->controlInfo->wpRoute.clear();

    nWP = scenario[currentScenarioID]->scenarioItems[fIdx]->controlInfo->wpRoute.size();
    for(int i=0;i<nWP;++i){
        struct ScenarioWPRoute* wpr = new struct ScenarioWPRoute;
        wpr->x = scenario[currentScenarioID]->scenarioItems[fIdx]->controlInfo->wpRoute[i]->x;
        wpr->y = scenario[currentScenarioID]->scenarioItems[fIdx]->controlInfo->wpRoute[i]->y;
        wpr->z = scenario[currentScenarioID]->scenarioItems[fIdx]->controlInfo->wpRoute[i]->z;
        wpr->direct = scenario[currentScenarioID]->scenarioItems[fIdx]->controlInfo->wpRoute[i]->direct;
        wpr->speedInfo = scenario[currentScenarioID]->scenarioItems[fIdx]->controlInfo->wpRoute[i]->speedInfo;
        wpr->wpID = scenario[currentScenarioID]->scenarioItems[fIdx]->controlInfo->wpRoute[i]->wpID;
        scenario[currentScenarioID]->scenarioItems[toIdx]->controlInfo->wpRoute.append( wpr );
    }

    qDebug() << "[CopyScenarioData] data copied: from " << fromAID << " to " << toAID;
}


void SimulationManager::SetTargetPathListScenarioVehicle(Agent** pAgent,Road *pRoad,int aID)
{
    if( scenario.size() == 0 ){
        return;
    }

    int nItem = scenario[currentScenarioID]->scenarioItems.size();

    //qDebug() << "[SetTargetPathListScenarioVehicle]";
    //qDebug() << "nItem = " << nItem;

    for(int i=0;i<nItem;++i){

        int objectID = scenario[currentScenarioID]->scenarioItems[i]->objectID;
        if( objectID != aID ){
            continue;
        }

        //qDebug() << "objectID = " << objectID;

        if( scenario[currentScenarioID]->scenarioItems[i]->type == 'v' ){

            // Set route information
            ScenarioObjectControlInfo* controlInfo = scenario[currentScenarioID]->scenarioItems[i]->controlInfo;

            //qDebug() << "routeType = " << controlInfo->routeType;

            if( controlInfo->routeType == ROUTE_TYPE::NODE_LIST_TYPE ){

                pAgent[objectID]->memory.routeType = ROUTE_TYPE::NODE_LIST_TYPE;

                // Not yet implemented
                continue;
            }
            else if( controlInfo->routeType == ROUTE_TYPE::PATH_LIST_TYPE ){

                //qDebug() << "paths.size = " << pRoad->paths.size();
                for(int j=0;j<pRoad->paths.size();++j){

                    //qDebug() << " p" << j << ": scenarioObjectID = " << pRoad->paths[j]->scenarioObjectID;
                    if( pRoad->paths[j]->scenarioObjectID != objectID ){
                        continue;
                    }
                    pAgent[objectID]->memory.targetPathList.prepend( pRoad->paths[j]->id );
                }
            }
        }
    }
}

void SimulationManager::AppearAgents(Agent** pAgent,int maxAgentNumber,Road *pRoad)
{
    int nAppear = 0;
    int maxNAppearAtATime = 3;
    if( DSMode == false ){
        maxNAppearAtATime = 100;
    }

    if( IDAllowed < 0 ){

        IDAllowed = 10;   // Reserve 10 agents for S-Interface objects
                          // This rule is applied for no-DS mode to use restart data in DS-mode

        if( scenario.size() > 0 ){

            int nItem = scenario[currentScenarioID]->scenarioItems.size();
            for(int i=0;i<nItem;++i){
                int objectID = scenario[currentScenarioID]->scenarioItems[i]->objectID;

//                qDebug() << "Scenario: objectID = " << objectID;

                if( IDAllowed <= objectID ){
                    IDAllowed = objectID + 1;
                }
            }

            int nEvent = scenario[currentScenarioID]->scenarioEvents.size();
            for(int i=0;i<nEvent;++i){

                if( scenario[currentScenarioID]->scenarioEvents[i]->eventType == SCENARIO_EVENT_TYPE::OBJECT_EVENT ){

                    int objectID = scenario[currentScenarioID]->scenarioEvents[i]->targetObjectID;

//                    qDebug() << "Scenario: objectID = " << objectID;

                    if( IDAllowed <= objectID ){
                        IDAllowed = objectID + 1;
                    }
                }
            }
        }

        qDebug() << "+--- IDAllowed = " << IDAllowed;
    }


    float simTimeSec = GetSimulationTimeInSec();


//    if( sysLogOutFile.open( QIODevice::Append | QIODevice::Text ) ){
//        QTextStream sysLogOut(&sysLogOutFile);
//        sysLogOut << "[AppearAgents] simTimeSec = " << simTimeSec << "\n";
//        sysLogOut.flush();
//        sysLogOutFile.close();
//    }



    //
    //  By OD Data
    //
    for(int i=0;i<pRoad->odRoute.size();++i){

        struct ODRouteData *rd = pRoad->odRoute[i];

        if( rd->onlyForScenarioVehicle == true ){
            continue;
        }

        if( rd->meanArrivalTime < 0.0 ){
//            qDebug() << "OD[" << i << "] meanArrivalTime < 0.0";
            continue;
        }

        if( rd->NextAppearTime <  0.0 ){
//            qDebug() << "OD[" << i << "] NextAppearTime < 0.0";
            rd->NextAppearTime = GetExponentialDist( rd->meanArrivalTime );
//            qDebug() << "  set NextAppearTime = " << rd->NextAppearTime;
        }
        else{

            bool genFlag = false;
            if( simTimeSec >= rd->NextAppearTime ){
                if( nAppear < maxNAppearAtATime ){
                    genFlag = true;
                    nAppear++;

                    // Set next timing
                    rd->NextAppearTime = simTimeSec;
                    rd->NextAppearTime += GetExponentialDist( rd->meanArrivalTime );
                    //qDebug() << "OD[" << i << "] : NextAppearTime = " <<  pRoad->odRoute[i]->NextAppearTime;
                }
            }

            if( genFlag == false ){
//                qDebug() << "OD[" << i << "] genFlag = false: nAppear = " << nAppear;
//                qDebug() << "simTimeSec = " << simTimeSec << " NextAppearTime = " << rd->NextAppearTime << " maxNAppearAtATime = " << maxNAppearAtATime;
                continue;
            }


            //
            //  Stop Generation ; directed by FE
            //
            if( rd->allowAgentGeneration == false ){
                continue;
            }


            //
            //  Generate Agent
            //
            int objID = -1;
            for(int n=IDAllowed;n<maxAgentNumber;++n){
                if( pAgent[n]->agentStatus == 0 ){
                    objID = n;
                    break;
                }
            }
            if( objID < 0 ){
                qDebug() << "OD[" << i << "] objID < 0";
                continue;
            }


            // Initialize
            pAgent[objID]->InitializeMemory();
            pAgent[objID]->calInterval = simTime.dt;
            pAgent[objID]->ID = objID;

            float simHz = 1.0 / simTime.dt;

            // Perception and Recognition; 10[Hz]
            pAgent[objID]->cognitionCountMax = (int)(simHz / 10.0);
            pAgent[objID]->cognitionCount    = 0;
            pAgent[objID]->cognitionSubCount = 0;

            // Hazard Identification and Risk Evaluation; 5[Hz]
            pAgent[objID]->decisionMakingCountMax = (int)(simHz / 5.0);
            pAgent[objID]->decisionMakingCount = 0;

            // Control; 10[Hz]
            pAgent[objID]->controlCountMax = (int)(simHz / 10.0);
            pAgent[objID]->controlCount = 0;


//            qDebug() << "Try to generate : objID = " << objID;


            //
            // Set targetPathList
            //
            pAgent[objID]->memory.routeLaneIndex = -1;

            pAgent[objID]->memory.destinationNode = pRoad->odRoute[i]->destinationNode;

            if( pRoad->odRoute[i]->laneListsToDestination.size() > 0 ){   // old version

                int numLaneLists = pRoad->odRoute[i]->laneListsToDestination.size();

                int selIdx = 0;

                float rnd = rndGen.GenUniform();
                float H = 1.0 / (float)numLaneLists;
                for(int k=0;k<numLaneLists;++k){
                    if( rnd >= k * H && rnd < (k+1) * H ){
                        selIdx = k;
                        break;
                    }
                }

                pAgent[objID]->memory.LCStartRouteIndex = -1;
                pAgent[objID]->memory.LCSupportRouteLaneIndex = -1;
                pAgent[objID]->memory.routeLaneIndex = selIdx;
                pAgent[objID]->memory.targetPathList = pRoad->odRoute[i]->laneListsToDestination[selIdx];
                pAgent[objID]->memory.currentTargetPath = pAgent[objID]->memory.targetPathList.last();

                pAgent[objID]->memory.laneMerge.clear();

                for(int k=0;k<pRoad->odRoute[i]->mergeLanesInfo[selIdx].size();++k){

                    QPoint pairData;
                    pairData.setX( pRoad->odRoute[i]->mergeLanesInfo[selIdx][k].x() );
                    pairData.setY( pRoad->odRoute[i]->mergeLanesInfo[selIdx][k].y() );

                    pAgent[objID]->memory.laneMerge.append( pairData );
                }
            }
            else if( pRoad->odRoute[i]->LCSupportLaneLists.size() > 0 ){    // new version

                int numLaneLists = pRoad->odRoute[i]->LCSupportLaneLists.last()->laneList.size();

                int selIdx = 0;
                if( numLaneLists > 1 ){
                    float rnd = rndGen.GenUniform();
                    float H = 1.0 / (float)numLaneLists;
                    for(int k=0;k<numLaneLists;++k){
                        if( rnd >= k * H && rnd < (k+1) * H ){
                            selIdx = k;
                            break;
                        }
                    }
                }

                if( pRoad->odRoute[i]->LCSupportLaneLists.size() > 1 ){
                    pAgent[objID]->memory.LCStartRouteIndex = pRoad->odRoute[i]->LCSupportLaneLists.last()->gIndexInNodeList;
                }
                else{
                    pAgent[objID]->memory.LCStartRouteIndex = -1;
                }

                pAgent[objID]->memory.LCSupportRouteLaneIndex = pRoad->odRoute[i]->LCSupportLaneLists.size() - 1;
                pAgent[objID]->memory.routeLaneIndex = selIdx;
                pAgent[objID]->memory.targetPathList = pRoad->odRoute[i]->LCSupportLaneLists.last()->laneList[selIdx];
                pAgent[objID]->memory.currentTargetPath = pAgent[objID]->memory.targetPathList.last();

                pAgent[objID]->memory.laneMerge.clear();


                int MLIidx = 0;
                for(int k=0;k<pAgent[objID]->memory.LCSupportRouteLaneIndex;++k){
                    MLIidx += pRoad->odRoute[i]->LCSupportLaneLists[k]->laneList.size();
                }
                MLIidx += pAgent[objID]->memory.routeLaneIndex;

                for(int k=0;k<pRoad->odRoute[i]->mergeLanesInfo[MLIidx].size();++k){

                    QPoint pairData;
                    pairData.setX( pRoad->odRoute[i]->mergeLanesInfo[MLIidx][k].x() );
                    pairData.setY( pRoad->odRoute[i]->mergeLanesInfo[MLIidx][k].y() );

                    pAgent[objID]->memory.laneMerge.append( pairData );
                }

            }
            else{

                qDebug() << "laneListsToDestination.size = 0: route[" << i << "] : from "
                         << pRoad->odRoute[i]->originNode << " to " << pRoad->odRoute[i]->destinationNode;


                int onIdx = pRoad->nodeId2Index.indexOf( pRoad->odRoute[i]->originNode );


                // This is for no route lanes data supplied
                QList<QList<int>> targetPathLists;
                QList<bool> needLCs;
                QList<int> nodeUntils;

                //
                // List up All possible path list
                for(int j=0;j<pRoad->nodes[onIdx]->outBoundaryWPs.size();++j){

                    for(int k=0;k<pRoad->nodes[onIdx]->outBoundaryWPs[j]->PathWithSWP.size();++k){

                        int tmpCurPath = pRoad->nodes[onIdx]->outBoundaryWPs[j]->PathWithSWP[k];

                        bool needLC = false;
                        int nodeUntil = -1;
                        QList<int> tmpTargetPathList = pRoad->GetPathList( i, tmpCurPath, needLC, nodeUntil, &(this->rndGen) );


                        targetPathLists.append( tmpTargetPathList );
                        needLCs.append( needLC );
                        nodeUntils.append( nodeUntil );

                    }
                }


                if( targetPathLists.size() == 0 ){
                    qDebug() << "-------";
                    qDebug() << "Trying to generate agent ID = " << objID << ": from Node "
                             << pRoad->odRoute[i]->originNode << " to "
                             << pRoad->odRoute[i]->destinationNode;
                    qDebug() << "No Path List extracted to reach destination.";
                    qDebug() << "Check Road Data.";
                    qDebug() << "-------";
                    continue;
                }


                int numNoNeedLCPaths = 0;
                for(int k=0;k<needLCs.size();++k){
                    if( needLCs[k] == false ){
                        numNoNeedLCPaths++;
                    }
                }


                if( numNoNeedLCPaths > 0 ){

                    int selIdx = 0;
                    if( numNoNeedLCPaths > 1 ){
                        float rnd = rndGen.GenUniform();
                        float H = 1.0 / (float)numNoNeedLCPaths;
                        for(int k=0;k<numNoNeedLCPaths;++k){
                            if( rnd >= k * H && rnd < (k+1) * H ){
                                selIdx = k;
                                break;
                            }
                        }
                    }

                    numNoNeedLCPaths = 0;
                    for(int k=0;k<needLCs.size();++k){
                        if( needLCs[k] == false ){

                            if( numNoNeedLCPaths == selIdx ){
                                pAgent[objID]->memory.targetPathList = targetPathLists[k];
                                pAgent[objID]->memory.currentTargetPath = targetPathLists[k].last();
                                break;
                            }
                            else{
                                numNoNeedLCPaths++;
                            }

                        }
                    }

                }
                else{

                    // random select
                    int selIdx = 0;
                    float rnd = rndGen.GenUniform();
                    float H = 1.0 / (float)targetPathLists.size();
                    for(int k=0;k<targetPathLists.size();++k){
                        if( rnd >= k * H && rnd < (k+1) * H ){
                            selIdx = k;
                            break;
                        }
                    }

                    pAgent[objID]->memory.targetPathList = targetPathLists[selIdx];
                    pAgent[objID]->memory.currentTargetPath = targetPathLists[selIdx].last();
                }

                for(int j=0;j<targetPathLists.size();++j){
                    targetPathLists[j].clear();
                }
                targetPathLists.clear();
                needLCs.clear();
                nodeUntils.clear();
            }




            if(  pAgent[objID]->memory.currentTargetPath < 0 || pAgent[objID]->memory.targetPathList.size() < 1 ){
                qDebug() << "-------";
                qDebug() << "Trying to generate agent ID = " << objID << ": from Node "
                         << pRoad->odRoute[i]->originNode << " to "
                         << pRoad->odRoute[i]->destinationNode;
                qDebug() << "Failed to set currentTargetPath.";
                qDebug() << "currentTargetPath = " << pAgent[objID]->memory.currentTargetPath;
                qDebug() << "targetPathList.size = " << pAgent[objID]->memory.targetPathList.size();
                qDebug() << "-------";
                continue;
            }


            pAgent[objID]->memory.targetPathLength.clear();
            for(int j=0;j<pAgent[objID]->memory.targetPathList.size();++j){

                float len = pRoad->GetPathLength( pAgent[objID]->memory.targetPathList[j] );
                pAgent[objID]->memory.targetPathLength.append( len );

                if( pAgent[objID]->memory.targetPathList[j] == pAgent[objID]->memory.currentTargetPath ){
                    pAgent[objID]->memory.currentTargetPathIndexInList = j;
                }
            }


            // Initial Position Data
            int tpIdx = pRoad->pathId2Index.indexOf( pAgent[objID]->memory.currentTargetPath );

            float xi  = pRoad->paths[tpIdx]->pos.first()->x();
            float yi  = pRoad->paths[tpIdx]->pos.first()->y();
            float zi  = pRoad->paths[tpIdx]->pos.first()->z();
            float cYAi = pRoad->paths[tpIdx]->derivative.first()->x();
            float sYAi = pRoad->paths[tpIdx]->derivative.first()->y();
            float YAi = atan2( sYAi , cYAi );

            float Vi  = pRoad->paths[tpIdx]->speed85pt;


            // set vehicle size
//            qDebug() << "Set Vehicle Type and Size";

            pAgent[objID]->agentKind = 0;

            pAgent[objID]->vehicle.SetVehicleModelID( 0 );

            {
                float rnd = rndGen.GenUniform();
                float p = 0.0;

                for(int j=0;j<pRoad->odRoute[i]->vehicleKindSelectProbability.size();++j){

                    if( p <= rnd && rnd < p + pRoad->odRoute[i]->vehicleKindSelectProbability[j] ){

                        pAgent[objID]->vehicle.SetVehicleModelID( j );

                        pAgent[objID]->objTypeForUE4 = pRoad->vehicleKind[j]->type;
                        pAgent[objID]->objNoForUE4   = pRoad->vehicleKind[j]->No;
                        if( pRoad->vehicleKind[j]->UE4ModelID > 0 ){
                            pAgent[objID]->objIDForUE4 = pRoad->vehicleKind[j]->UE4ModelID;
                        }
                        else{
                            pAgent[objID]->objIDForUE4 = -1;
                        }

                        float L = pRoad->vehicleKind[j]->length;
                        float W = pRoad->vehicleKind[j]->width;
                        float H = pRoad->vehicleKind[j]->height;

                        pAgent[objID]->vehicle.SetVehicleParam( L, W, H, L * 0.8, L * 0.1, 1.0 );

                        pAgent[objID]->vHalfLength = L * 0.5;
                        pAgent[objID]->vHalfWidth  = W  * 0.5;
                        break;
                    }
                    else{
                        p += pRoad->odRoute[i]->vehicleKindSelectProbability[j];
                    }
                }
            }


            // Check the lane has enough space for new vehicle
            bool enoughSpace = true;
            float minL = 200.0;
            for(int j=0;j<maxAgentNumber;++j){
                if( pAgent[j]->agentStatus == 0 ){
                    continue;
                }
                if( pAgent[j]->memory.currentTargetPath != pAgent[objID]->memory.currentTargetPath ){
                    continue;
                }

                float dx = pAgent[j]->state.x - xi;
                float dy = pAgent[j]->state.y - yi;
                float L = dx * cYAi + dy * sYAi;

                L -= pAgent[j]->vHalfLength;
                L -= pAgent[objID]->vHalfLength;

                if( L < 5.0 ){
                    enoughSpace = false;
                    break;
                }

                if( minL > L ){
                    minL = L;
                }
            }
            if( enoughSpace == false ){
                continue;
            }

            float minV = sqrt( 2.0 * pAgent[objID]->param.maxDeceleration * 0.8 * minL );
            if( Vi > minV ){
                Vi = minV;
            }


            // Set initial state
//            qDebug() << "Set Vehicle Initial State";

            pAgent[objID]->vehicle.SetVehicleID( objID );
            pAgent[objID]->vehicle.SetWinker(0);


            pAgent[objID]->vehicle.SetInitialState( Vi, xi, yi, zi, YAi );  // Yaw in [rad]

            pAgent[objID]->state.V = Vi;
            pAgent[objID]->state.x = xi;
            pAgent[objID]->state.y = yi;
            pAgent[objID]->state.z = zi;
            pAgent[objID]->state.yaw = YAi;
            pAgent[objID]->state.cosYaw = cos( YAi );
            pAgent[objID]->state.sinYaw = sin( YAi );

            pAgent[objID]->state.z_path = 0.0;

            // Set Control Information
            pAgent[objID]->memory.controlMode = AGENT_CONTROL_MODE::AGENT_LOGIC;


            pAgent[objID]->param.speedVariationFactor = rndGen.GetNormalDist(0.0,0.3);
            if( pAgent[objID]->param.speedVariationFactor > 1.5 ){
                pAgent[objID]->param.speedVariationFactor = 1.5;
            }
            else if( pAgent[objID]->param.speedVariationFactor < -0.8 ){
                pAgent[objID]->param.speedVariationFactor = -0.8;
            }

            pAgent[objID]->param.latAccelAtTurn = ( 0.30 + pAgent[objID]->param.speedVariationFactor * 0.07) * 9.81;

            pAgent[objID]->SetTargetSpeedIndividual( pRoad->paths[tpIdx]->speed85pt );


            pAgent[objID]->param.visibleDistance = 200.0;
            pAgent[objID]->param.minimumHeadwayDistanceAtStop = 4.5;


            pAgent[objID]->agentStatus = 1;
//            qDebug() << "Generate Agent : objID = " << objID << " currentTargetPath = " << pAgent[objID]->memory.currentTargetPath;
            qDebug() << "Generate Agent : objID = " << objID;

            pAgent[objID]->isScenarioObject = false;

            pAgent[objID]->isSInterfaceObject = false;
            pAgent[objID]->isBehaviorEmbeded  = false;
            pAgent[objID]->justWarped = false;

            pAgent[objID]->memory.routeType  = AGENT_ROUTE_TYPE::NODE_LIST;
            pAgent[objID]->memory.routeIndex = i;

            pAgent[objID]->SetTargetNodeListByTargetPaths( pRoad );

            pAgent[objID]->BackupMemory();

            pAgent[objID]->TimeOfAppear = GetSimulationTimeInSec();

//            pAgent[objID]->memory.runOncomingLane = true;


            // Fluctuation in Speed Control
            if( pRoad->paths[tpIdx]->setSpeedVariationParam == true ){

//                qDebug() << " Set vDev Param";

                pAgent[objID]->param.refVforDev     = pRoad->paths[tpIdx]->refVforDev;
                pAgent[objID]->param.vDevAllowPlus  = pRoad->paths[tpIdx]->vDevP;
                pAgent[objID]->param.vDevAllowMinus = pRoad->paths[tpIdx]->vDevM;
                pAgent[objID]->param.accelAtVDev    = pRoad->paths[tpIdx]->accelAtDev;
                pAgent[objID]->param.decelAtVDev    = pRoad->paths[tpIdx]->decelAtDev;
                pAgent[objID]->param.deadZoneSpeedControl = 0.0;

            }


            if( DSMode == true ){

                if( pAgent[objID]->vehicle.steerFiltered4CG == NULL ){
                    pAgent[objID]->vehicle.steerFiltered4CG = new LowPassFilter(1, simTime.dt, 6.0 , 0.0);
                }
                else{
                    pAgent[objID]->vehicle.steerFiltered4CG->SetParam(1, simTime.dt, 6.0 , 0.0);
                    pAgent[objID]->vehicle.steerFiltered4CG->Reset();
                }

                if( pAgent[objID]->vehicle.axFiltered4CG == NULL ){
                    pAgent[objID]->vehicle.axFiltered4CG = new LowPassFilter(1, simTime.dt, 6.0 , 0.0);
                }
                else{
                    pAgent[objID]->vehicle.axFiltered4CG->SetParam(1, simTime.dt, 6.0 , 0.0);
                    pAgent[objID]->vehicle.axFiltered4CG->Reset();
                }

                if( pAgent[objID]->vehicle.ayFiltered4CG == NULL ){
                    pAgent[objID]->vehicle.ayFiltered4CG = new LowPassFilter(1, simTime.dt, 6.0 , 0.0);
                }
                else{
                    pAgent[objID]->vehicle.ayFiltered4CG->SetParam(1, simTime.dt, 6.0 , 0.0);
                    pAgent[objID]->vehicle.ayFiltered4CG->Reset();
                }


                if( pAgent[objID]->vehicle.suspentionFL4CG == NULL ){
                    pAgent[objID]->vehicle.suspentionFL4CG = new LowPassFilter(2, simTime.dt, 10.0 , 0.6);
                }
                else{
                    pAgent[objID]->vehicle.suspentionFL4CG->SetParam(2, simTime.dt, 10.0 , 0.6);
                    pAgent[objID]->vehicle.suspentionFL4CG->Reset();
                }

                if( pAgent[objID]->vehicle.suspentionFR4CG == NULL ){
                    pAgent[objID]->vehicle.suspentionFR4CG = new LowPassFilter(2, simTime.dt, 10.0 , 0.6);
                }
                else{
                    pAgent[objID]->vehicle.suspentionFR4CG->SetParam(2, simTime.dt, 10.0 , 0.6);
                    pAgent[objID]->vehicle.suspentionFR4CG->Reset();
                }

                if( pAgent[objID]->vehicle.suspentionRL4CG == NULL ){
                    pAgent[objID]->vehicle.suspentionRL4CG = new LowPassFilter(2, simTime.dt, 10.0 , 0.6);
                }
                else{
                    pAgent[objID]->vehicle.suspentionRL4CG->SetParam(2, simTime.dt, 10.0 , 0.6);
                    pAgent[objID]->vehicle.suspentionRL4CG->Reset();
                }

                if( pAgent[objID]->vehicle.suspentionRR4CG == NULL ){
                    pAgent[objID]->vehicle.suspentionRR4CG = new LowPassFilter(2, simTime.dt, 10.0 , 0.6);
                }
                else{
                    pAgent[objID]->vehicle.suspentionRR4CG->SetParam(2, simTime.dt, 10.0 , 0.6);
                    pAgent[objID]->vehicle.suspentionRR4CG->Reset();
                }


                if( pAgent[objID]->vehicle.tireFL4CG == NULL ){
                    pAgent[objID]->vehicle.tireFL4CG = new LowPassFilter(1, simTime.dt, 5.0 , 0.0);
                }
                else{
                    pAgent[objID]->vehicle.tireFL4CG->SetParam(1, simTime.dt, 5.0 , 0.0);
                    pAgent[objID]->vehicle.tireFL4CG->Reset();
                }

                if( pAgent[objID]->vehicle.tireFR4CG == NULL ){
                    pAgent[objID]->vehicle.tireFR4CG = new LowPassFilter(1, simTime.dt, 5.0 , 0.0);
                }
                else{
                    pAgent[objID]->vehicle.tireFR4CG->SetParam(1, simTime.dt, 5.0 , 0.0);
                    pAgent[objID]->vehicle.tireFR4CG->Reset();
                }

                if( pAgent[objID]->vehicle.tireRL4CG == NULL ){
                    pAgent[objID]->vehicle.tireRL4CG = new LowPassFilter(1, simTime.dt, 5.0 , 0.0);
                }
                else{
                    pAgent[objID]->vehicle.tireRL4CG->SetParam(1, simTime.dt, 5.0 , 0.0);
                    pAgent[objID]->vehicle.tireRL4CG->Reset();
                }

                if( pAgent[objID]->vehicle.tireRR4CG == NULL ){
                    pAgent[objID]->vehicle.tireRR4CG = new LowPassFilter(1, simTime.dt, 5.0 , 0.0);
                }
                else{
                    pAgent[objID]->vehicle.tireRR4CG->SetParam(1, simTime.dt, 5.0 , 0.0);
                    pAgent[objID]->vehicle.tireRR4CG->Reset();
                }

                if( pAgent[objID]->vehicle.yawFiltered4CG == NULL ){
                    pAgent[objID]->vehicle.yawFiltered4CG = new LowPassFilter(1, simTime.dt, 3.0 , 0.0);
                }
                pAgent[objID]->vehicle.yawFiltered4CG->SetParam(1,simTime.dt, 3.0 , 0.0);
                pAgent[objID]->vehicle.yawFiltered4CG->SetInitialValue( YAi );
            }
        }
    }


    for(int i=0;i<pRoad->pedestPaths.size();++i){

        if( pRoad->pedestPaths[i]->scenarioObjectID >= 0 ){
            continue;
        }

        if( pRoad->pedestPaths[i]->meanArrivalTime < 0.0 ){
            continue;
        }

        if( pRoad->pedestPaths[i]->NextAppearTime <  0.0 ){

            pRoad->pedestPaths[i]->NextAppearTime = GetExponentialDist( pRoad->pedestPaths[i]->meanArrivalTime );

        }
        else{

            bool genFlag = false;
            if( simTimeSec >= pRoad->pedestPaths[i]->NextAppearTime ){
                if( nAppear < maxNAppearAtATime ){
                    genFlag = true;
                    nAppear++;

                    // Set next timing
                    pRoad->pedestPaths[i]->NextAppearTime = simTimeSec;
                    pRoad->pedestPaths[i]->NextAppearTime += GetExponentialDist( pRoad->pedestPaths[i]->meanArrivalTime );
                    //qDebug() << "OD[" << i << "] : NextAppearTime = " <<  pRoad->odRoute[i]->NextAppearTime;
                }
            }

            if( genFlag == false ){
                continue;
            }


            //
            //  Generate Agent
            //
            int objID = -1;
            for(int n=IDAllowed;n<maxAgentNumber;++n){
                if( pAgent[n]->agentStatus == 0 ){
                    objID = n;
                    break;
                }
            }
            if( objID < 0 ){
                continue;
            }


            // Initialize
            pAgent[objID]->InitializeMemory();
            pAgent[objID]->calInterval = simTime.dt;
            pAgent[objID]->ID = objID;


            float simHz = 1.0 / simTime.dt;

            // Perception and Recognition; 10[Hz]
            pAgent[objID]->cognitionCountMax = (int)(simHz / 10.0);
            pAgent[objID]->cognitionCount = 0;

            // Hazard Identification and Risk Evaluation; 3[Hz]
            pAgent[objID]->decisionMakingCountMax = (int)(simHz / 3.0);
            pAgent[objID]->decisionMakingCount = 0;

            // Control; 10[Hz]
            pAgent[objID]->controlCountMax = (int)(simHz / 10.0);
            pAgent[objID]->controlCount = 0;



            // random select
            int selPedestModelIdx = 0;
            {
                float rnd = rndGen.GenUniform();
                float H = 0.0;
                for(int k=0;k<pRoad->pedestPaths[i]->pedestKindSelectProbability.size();++k){
                    float pThr = pRoad->pedestPaths[i]->pedestKindSelectProbability[k];
                    if( H <= rnd && rnd < H + pThr ){
                        selPedestModelIdx = k;
                        break;
                    }
                    else{
                        H += pThr;
                    }
                }
            }


            if( pRoad->pedestrianKind[selPedestModelIdx]->type == 2 ){   // Bicycle

                float rnd = this->GenUniform();
                if( rnd <= 0.0638 ){
                    pAgent[objID]->attri.age = 0;
                    pAgent[objID]->memory.targetSpeed = GetNormalDist( 3.0, 0.107 );
                }
                else if( rnd >= 1.0 - 0.2637 ){
                    pAgent[objID]->attri.age = 2;
                    pAgent[objID]->memory.targetSpeed = GetNormalDist( 4.0, 0.104 );
                }
                else{
                    pAgent[objID]->attri.age = 1;
                    pAgent[objID]->memory.targetSpeed = GetNormalDist( 5.0, 0.093 );
                }

            }
            else{
                float rnd = this->GenUniform();
                if( rnd <= 0.0638 ){
                    pAgent[objID]->attri.age = 0;
                    pAgent[objID]->memory.targetSpeed = GetNormalDist( meanSpeedPedestrian[0], 0.107 );
                }
                else if( rnd >= 1.0 - 0.2637 ){
                    pAgent[objID]->attri.age = 2;
                    pAgent[objID]->memory.targetSpeed = GetNormalDist( meanSpeedPedestrian[2], 0.104 );
                }
                else{
                    pAgent[objID]->attri.age = 1;
                    pAgent[objID]->memory.targetSpeed = GetNormalDist( meanSpeedPedestrian[1], 0.093 );
                }

                if( pAgent[objID]->memory.targetSpeed <= 0.2 ){
                    pAgent[objID]->memory.targetSpeed = 0.2;
                }
            }


            pAgent[objID]->agentKind = 100 + selPedestModelIdx;

            pAgent[objID]->vehicle.SetVehicleModelID( selPedestModelIdx );

            if( selPedestModelIdx < pRoad->pedestrianKind.size() ){

                pAgent[objID]->objTypeForUE4 = pRoad->pedestrianKind[selPedestModelIdx]->type;
                pAgent[objID]->objNoForUE4   = pRoad->pedestrianKind[selPedestModelIdx]->No;
                if( pRoad->pedestrianKind[selPedestModelIdx]->UE4ModelID > 0 ){
                    pAgent[objID]->objIDForUE4 = pRoad->pedestrianKind[selPedestModelIdx]->UE4ModelID;
                }
                else{
                    pAgent[objID]->objIDForUE4 = -1;
                }

                if( DSMode == true && pAgent[objID]->objIDForUE4 < 0 ){  // Only for support of old-format data
                    if( pAgent[objID]->objTypeForUE4 == 1 ){
                        if( pAgent[objID]->objNoForUE4 == 3 ){   // Child
                            pAgent[objID]->attri.age = 0;
                            pAgent[objID]->memory.targetSpeed = GetNormalDist( meanSpeedPedestrian[0], 0.107 );
                        }
                        else if( pAgent[objID]->objNoForUE4 == 4 ){   // Aged
                            pAgent[objID]->attri.age = 2;
                            pAgent[objID]->memory.targetSpeed = GetNormalDist( meanSpeedPedestrian[2], 0.104 );
                        }
                        else{
                            pAgent[objID]->attri.age = 1;
                            pAgent[objID]->memory.targetSpeed = GetNormalDist( meanSpeedPedestrian[1], 0.093 );
                        }
                    }
                }


                //
                //  Set speed and age if assigned by SEdit; this is for new format data
                //
                if( pAgent[objID]->objIDForUE4 > 0 ){
                    if( pRoad->pedestrianKind[selPedestModelIdx]->meanSpeed > 0.0 ){

                        pAgent[objID]->memory.targetSpeed = GetNormalDist( pRoad->pedestrianKind[selPedestModelIdx]->meanSpeed,
                                                                           pRoad->pedestrianKind[selPedestModelIdx]->stdDevSpeed );


                        if( pAgent[objID]->memory.targetSpeed <= 0.55 ){
                            pAgent[objID]->memory.targetSpeed = 0.55;
                        }
                    }

                    pAgent[objID]->attri.age = pRoad->pedestrianKind[selPedestModelIdx]->ageInfo;
                }


                pAgent[objID]->vHalfWidth  = pRoad->pedestrianKind[selPedestModelIdx]->width * 0.5;
                pAgent[objID]->vHalfLength = pRoad->pedestrianKind[selPedestModelIdx]->length * 0.5;

//                // Assume minimum size of pedestrian as a circle of Diameter=0.75[m]
//                if( pAgent[objID]->vHalfWidth < 0.35 ){
//                    pAgent[objID]->vHalfWidth = 0.35;
//                }
//                if( pAgent[objID]->vHalfLength < 0.35 ){
//                    pAgent[objID]->vHalfLength = 0.35;
//                }
            }

            pAgent[objID]->state.V = 0.0;
            pAgent[objID]->state.x = pRoad->pedestPaths[i]->shape.first()->pos.x();
            pAgent[objID]->state.y = pRoad->pedestPaths[i]->shape.first()->pos.y();
            pAgent[objID]->state.z = pRoad->pedestPaths[i]->shape.first()->pos.z();
            pAgent[objID]->state.yaw = pRoad->pedestPaths[i]->shape.first()->angleToNextPos;
            pAgent[objID]->state.cosYaw = pRoad->pedestPaths[i]->shape.first()->cosA;
            pAgent[objID]->state.sinYaw = pRoad->pedestPaths[i]->shape.first()->sinA;


            pAgent[objID]->memory.targetPathList.clear();
            pAgent[objID]->memory.targetPathList.append( pRoad->pedestPaths[i]->id );

            pAgent[objID]->memory.currentTargetPath = pRoad->pedestPaths[i]->id;
            pAgent[objID]->memory.currentTargetPathIndexInList = 0;


            float maxWID = pRoad->pedestPaths[i]->shape.first()->width * 0.5 - pAgent[objID]->vHalfWidth;
            if( maxWID > 2.5 - pAgent[objID]->vHalfWidth ){
                maxWID = 2.5 - pAgent[objID]->vHalfWidth;
            }
            else if(maxWID < 0.5){
                maxWID = 0.5;
            }

            pAgent[objID]->memory.lateralShiftTarget = ( GetNormalDist( 0.0, 2.0 ) )* maxWID;
            if( pAgent[objID]->memory.lateralShiftTarget > maxWID ){
                pAgent[objID]->memory.lateralShiftTarget = maxWID;
            }
            else if( pAgent[objID]->memory.lateralShiftTarget < -maxWID ){
                pAgent[objID]->memory.lateralShiftTarget = -maxWID;
            }

            pAgent[objID]->memory.lateralShiftTarget_backup = pAgent[objID]->memory.lateralShiftTarget;


            // Set control information
            pAgent[objID]->memory.controlMode = AGENT_CONTROL_MODE::AGENT_LOGIC;

            pAgent[objID]->param.visibleDistance = 25.0;
            pAgent[objID]->param.minimumHeadwayDistanceAtStop = 1.5;


            // Set Flags
            pAgent[objID]->agentStatus = 1;
            //qDebug() << "Generate Agent[Pedestrian] : objID = " << objID;

            pAgent[objID]->isScenarioObject = false;

            pAgent[objID]->BackupMemory();

            pAgent[objID]->TimeOfAppear = GetSimulationTimeInSec();

            if( DSMode == true ){

                if( pAgent[objID]->vehicle.yawFiltered4CG == NULL ){
                    pAgent[objID]->vehicle.yawFiltered4CG = new LowPassFilter(1, simTime.dt, 10.0 , 0.0);
                }
                pAgent[objID]->vehicle.yawFiltered4CG->SetParam(1,simTime.dt, 10.0 , 0.0);
                pAgent[objID]->vehicle.yawFiltered4CG->SetInitialValue( pAgent[objID]->state.yaw );

                if( pAgent[objID]->vehicle.axFiltered4CG == NULL ){
                    pAgent[objID]->vehicle.axFiltered4CG = new LowPassFilter(1, simTime.dt, 6.0 , 0.0);
                }

                pAgent[objID]->vehicle.axFiltered4CG->SetParam(1,simTime.dt, 30.0 , 0.0);
                pAgent[objID]->vehicle.axFiltered4CG->SetInitialValue( cos( pAgent[objID]->state.yaw ) );

                if( pAgent[objID]->vehicle.ayFiltered4CG == NULL ){
                    pAgent[objID]->vehicle.ayFiltered4CG = new LowPassFilter(1, simTime.dt, 6.0 , 0.0);
                }

                pAgent[objID]->vehicle.ayFiltered4CG->SetParam(1,simTime.dt, 30.0 , 0.0);
                pAgent[objID]->vehicle.ayFiltered4CG->SetInitialValue( sin( pAgent[objID]->state.yaw ) );
            }
        }
    }





    //
    //  By Scenario
    //
    if( scenario.size() == 0 ){
        return;
    }

    nAppear = 0;


    for(int i=0;i<scenario[currentScenarioID]->scenarioEvents.size();++i){

        struct ScenarioEvents *se = scenario[currentScenarioID]->scenarioEvents[i];
        if( se->eventType != SCENARIO_EVENT_TYPE::OBJECT_EVENT ){
            continue;
        }
        if( se->eventKind != OBJECT_SCENARIO_ACTION_KIND::APPEAR_VEHICLE && se->eventKind != OBJECT_SCENARIO_ACTION_KIND::APPEAR_PEDESTRIAN ){
            continue;
        }
        if( se->eventState == 1 ){
            continue;
        }

        int objectID = se->targetObjectID;

        if( pAgent[objectID]->agentStatus != 0 ){
            // Already appear or Out from the stage
            continue;
        }

        if( pAgent[objectID]->notAllowedAppear == true ){
            continue;
        }


        // Check trigger
        struct ScenarioTriggers* trigger = se->eventTrigger;

        //qDebug() << " trigger mode = " << trigger->mode << " combination=" << trigger->combination;

        if( trigger->combination >= 0 ){

            int nTriggered = 0;

            bool afterCheckAfterTimeOrPosTrigger = false;
            bool hasTimeTrigger = false;
            if( trigger->combination == 0 && trigger->objectTigger.size() > 0 ){  // AND Combination
                bool hasTimeOrPosTrigger = false;
                bool hasVelOrTTCTrigger = false;
                for(int j=0;j<trigger->objectTigger.size();++j){
                    if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ){
                        hasTimeTrigger = true;
                    }
                    if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ||
                            trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::POSITION_TRIGGER ){
                        hasTimeOrPosTrigger = true;
                    }
                    if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::VELOCITY_TRIGGER ||
                            trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TTC_TRIGGER ){
                        hasVelOrTTCTrigger = true;
                    }
                }
                if( hasTimeOrPosTrigger == true && hasVelOrTTCTrigger == true ){
                    afterCheckAfterTimeOrPosTrigger = true;
                }
            }

            for(int j=0;j<trigger->objectTigger.size();++j){

                if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::DELAY_TRIGGER ){
                    continue;
                }

                if( trigger->objectTigger[j]->isTriggered == true ){
                    nTriggered++;
                    continue;
                }

                if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::AT_ONCE ){
                    trigger->objectTigger[j]->isTriggered = true;
                    nTriggered++;
                    continue;
                }
                else if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ){
                    if( trigger->objectTigger[j]->timeTriggerInSec <= simTimeSec ){
                        trigger->objectTigger[j]->isTriggered = true;
                        nTriggered++;
                        continue;
                    }
                }
                else if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::POSITION_TRIGGER ){

                    bool evalPosTrig = true;
                    if( hasTimeTrigger == true ){
                        bool evaluedTimeTrig = false;
                        for(int j=0;j<trigger->objectTigger.size();++j){
                            if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ){
                                if( trigger->objectTigger[j]->isTriggered == true ){
                                    evaluedTimeTrig = true;
                                    break;
                                }
                            }
                        }
                        if( evaluedTimeTrig == false ){
                            evalPosTrig = false;
                        }
                    }

                    if( evalPosTrig == true ){
                        int objID = trigger->objectTigger[j]->targetObjectID;
                        if( objID >= 0 && objID < maxAgentNumber && pAgent[objID]->agentStatus == 1 ){

                            if( pAgent[objID]->isSInterfaceObject == true && pAgent[objID]->isSInterObjDataSet == false ){
                                continue;
                            }

                            float rx = pAgent[objID]->state.x -  trigger->objectTigger[j]->x;
                            float ry = pAgent[objID]->state.y -  trigger->objectTigger[j]->y;
                            float ip = rx * trigger->objectTigger[j]->cosDirect + ry * trigger->objectTigger[j]->sinDirect;
                            float e = rx * trigger->objectTigger[j]->sinDirect * (-1.0) + ry * trigger->objectTigger[j]->cosDirect;

                            //qDebug() << "ip = " << ip << " e = " << e << " passCheckFlag = " << trigger->objectTigger[j]->passCheckFlag;

                            float w = trigger->objectTigger[j]->widthHalf;

                            if( trigger->objectTigger[j]->passCheckFlag == 0 && fabs(e) < w && ip < 0.0 && ip > -30.0 ){
                                trigger->objectTigger[j]->passCheckFlag = 1;
                                //qDebug() << "passCheckFlag = 1";
                            }
                            else if( trigger->objectTigger[j]->passCheckFlag == 1 && fabs(e) < w && ip >= 0.0 ){
                                trigger->objectTigger[j]->passCheckFlag = 2;
                                trigger->objectTigger[j]->isTriggered = true;
                                nTriggered++;
                                //qDebug() << "passCheckFlag = 2";
                                continue;
                            }
                        }
                    }
                }
                else if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::VELOCITY_TRIGGER ){

                    bool evalVelTrig = true;
                    if( afterCheckAfterTimeOrPosTrigger == true ){
                        bool evaluedTimeOrPosTrig = false;
                        for(int j=0;j<trigger->objectTigger.size();++j){
                            if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ||
                                    trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::POSITION_TRIGGER ){
                                if( trigger->objectTigger[j]->isTriggered == true ){
                                    evaluedTimeOrPosTrig = true;
                                    break;
                                }
                            }
                        }
                        if( evaluedTimeOrPosTrig == false ){
                            evalVelTrig = false;
                        }
                    }

                    if( evalVelTrig == true ){

                        int objID = trigger->objectTigger[j]->targetObjectID;
                        //qDebug() << "[VEL_TRIG] objID = " << objID;

                        if( objID >= 0 && objID < maxAgentNumber && pAgent[objID]->agentStatus == 1 ){

                            //qDebug() << "[VEL_TRIG] objID = " << objID
                            //         << " isSInterfaceObject = " << pAgent[objID]->isSInterfaceObject
                            //         << " isSInterObjDataSet = " << pAgent[objID]->isSInterObjDataSet;

                            if( pAgent[objID]->isSInterfaceObject == true && pAgent[objID]->isSInterObjDataSet == false ){
                                continue;
                            }

                            float objV = pAgent[objID]->state.V;

                            //qDebug() << "[VEL_TRIG] triggerParam = " << trigger->objectTigger[j]->triggerParam
                            //         << " objV = " << objV << " speed = " << trigger->objectTigger[j]->speed;

                            if( trigger->objectTigger[j]->triggerParam == 1 && objV < trigger->objectTigger[j]->speed ){
                                trigger->objectTigger[j]->isTriggered = true;
                                nTriggered++;

                                //qDebug() << "[VEL_TRIG] Triggered; slower";

                                continue;
                            }
                            else if( trigger->objectTigger[j]->triggerParam == 0 && objV > trigger->objectTigger[j]->speed ){
                                trigger->objectTigger[j]->isTriggered = true;
                                nTriggered++;

                                //qDebug() << "[VEL_TRIG] Triggered; faster";

                                continue;
                            }
                        }
                    }
                }
                else if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TTC_TRIGGER ){

                    //qDebug() << "[TTC_TRIG] triggerParam = " << trigger->objectTigger[j]->triggerParam;

                    bool evalTTCTrig = true;
                    if( afterCheckAfterTimeOrPosTrigger == true ){
                        bool evaluedTimeOrPosTrig = false;
                        for(int j=0;j<trigger->objectTigger.size();++j){
                            if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ||
                                    trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::POSITION_TRIGGER ){
                                if( trigger->objectTigger[j]->isTriggered == true ){
                                    evaluedTimeOrPosTrig = true;
                                    break;
                                }
                            }
                        }
                        if( evaluedTimeOrPosTrig == false ){
                            evalTTCTrig = false;
                        }
                    }

                    if( evalTTCTrig == true ){
                        if( trigger->objectTigger[j]->triggerParam == 1 ){    // calculate TTC to object

                            int objID = trigger->objectTigger[j]->targetObjectID;
                            int calID = trigger->objectTigger[j]->triggerParam2;

                            //qDebug() << "[TTC_TRIG] objID = " << objID << " calID = " << calID;

                            if( objID >= 0 && objID < maxAgentNumber && pAgent[objID]->agentStatus == 1 &&
                                   calID >= 0 && calID < maxAgentNumber && pAgent[calID]->agentStatus == 1 ){

                                if( pAgent[objID]->isSInterfaceObject == true && pAgent[objID]->isSInterObjDataSet == false ){
                                    continue;
                                }
                                if( pAgent[calID]->isSInterfaceObject == true && pAgent[calID]->isSInterObjDataSet == false ){
                                    continue;
                                }

                                float rx = pAgent[calID]->state.x - pAgent[objID]->state.x;
                                float ry = pAgent[calID]->state.y - pAgent[objID]->state.y;
                                float ip = rx * pAgent[objID]->state.cosYaw + ry * pAgent[objID]->state.sinYaw;
                                float e = rx * pAgent[objID]->state.sinYaw * (-1.0) + ry * pAgent[objID]->state.cosYaw;

                                float f = pAgent[calID]->state.cosYaw * pAgent[objID]->state.cosYaw + pAgent[calID]->state.sinYaw * pAgent[objID]->state.sinYaw;
                                float relV = pAgent[objID]->state.V - f * pAgent[calID]->state.V;

                                //qDebug() << "[TTC_TRIG] relV = " << relV << " ip = " << ip << " e = " << e;

                                if( ip >= 0.0 && relV > 0.1 && fabs(e) < 1.5 ){
                                    float ttc = ip / relV;

                                    //qDebug() << "[TTC_TRIG] ttc = " << ttc;

                                    if( ttc < trigger->objectTigger[j]->TTC ){
                                        trigger->objectTigger[j]->isTriggered = true;
                                        nTriggered++;
                                        continue;
                                    }
                                }
                            }
                        }
                        else if( trigger->objectTigger[j]->triggerParam == 0 ){   // calculate TTC to point

                            int objID = trigger->objectTigger[j]->targetObjectID;

                            //qDebug() << "[TTC_TRIG] objID = " << objID;

                            if( pAgent[objID]->isSInterfaceObject == true && pAgent[objID]->isSInterObjDataSet == false ){
                                continue;
                            }

                            float rx = trigger->objectTigger[j]->x - pAgent[objID]->state.x;
                            float ry = trigger->objectTigger[j]->y - pAgent[objID]->state.y;
                            float ip = rx * pAgent[objID]->state.cosYaw + ry * pAgent[objID]->state.sinYaw;
                            float e = rx * pAgent[objID]->state.sinYaw * (-1.0) + ry * pAgent[objID]->state.cosYaw;

                            if( ip >= 0.0 && pAgent[objID]->state.V > 0.1 && fabs(e) < 1.5 ){
                                float ttc = ip / pAgent[objID]->state.V;

                                //qDebug() << "[TTC_TRIG] ttc = " << ttc;

                                if( ttc < trigger->objectTigger[j]->TTC ){
                                    trigger->objectTigger[j]->isTriggered = true;
                                    nTriggered++;
                                    continue;
                                }
                            }
                        }
                    }
                }
            }

            bool exTrig = false;

            if( trigger->byKeyTriggerFlag == true ){
                int keyHit = 0;
                switch( trigger->func_keys ){
                case 1: keyHit = GetAsyncKeyState(VK_F1); break;
                case 2: keyHit = GetAsyncKeyState(VK_F2); break;
                case 3: keyHit = GetAsyncKeyState(VK_F3); break;
                case 4: keyHit = GetAsyncKeyState(VK_F4); break;
                case 5: keyHit = GetAsyncKeyState(VK_F5); break;
                case 6: keyHit = GetAsyncKeyState(VK_F6); break;
                case 7: keyHit = GetAsyncKeyState(VK_F7); break;
                case 8: keyHit = GetAsyncKeyState(VK_F8); break;
                case 9: keyHit = GetAsyncKeyState(VK_F9); break;
                case 10: keyHit = GetAsyncKeyState(VK_F10); break;
                case 11: keyHit = GetAsyncKeyState(VK_F11); break;
                case 12: keyHit = GetAsyncKeyState(VK_F12); break;
                case 13: keyHit = GetAsyncKeyState(VK_NUMPAD0); break;
                case 14: keyHit = GetAsyncKeyState(VK_NUMPAD1); break;
                case 15: keyHit = GetAsyncKeyState(VK_NUMPAD2); break;
                case 16: keyHit = GetAsyncKeyState(VK_NUMPAD3); break;
                case 17: keyHit = GetAsyncKeyState(VK_NUMPAD4); break;
                case 18: keyHit = GetAsyncKeyState(VK_NUMPAD5); break;
                case 19: keyHit = GetAsyncKeyState(VK_NUMPAD6); break;
                case 20: keyHit = GetAsyncKeyState(VK_NUMPAD7); break;
                case 21: keyHit = GetAsyncKeyState(VK_NUMPAD8); break;
                case 22: keyHit = GetAsyncKeyState(VK_NUMPAD9); break;
                }
                if( keyHit != 0 ){
                    exTrig = true;
                }
            }

            if( trigger->byExternalTriggerFlag == true ){
                if( trigger->extTriggerFlagState == true ){
                    exTrig = true;
                }
            }


//            qDebug() << "nTriggered = " << nTriggered << " size = " << trigger->objectTigger.size();

            if( exTrig == false ){
                if( trigger->combination == 0 && (nTriggered < trigger->objectTigger.size() || trigger->objectTigger.size() == 0) ){
                    continue;
                }
                else if( trigger->combination == 1 && nTriggered == 0 ){
                    continue;
                }
            }

        }
        else{
            qDebug() << "trigeer->combination is negative. data format of scenario data will be old. Should be updated. [EventID="
                     << se->eventID << "]";
            qDebug() << "Change to combination is 0";

            trigger->combination = 0;

            continue;
        }


        if( nAppear < maxNAppearAtATime ){
            nAppear++;
        }
        else{
            continue;
        }


        //
        // Event triggerred
        qDebug() << "Event[" << se->eventID << "] triggerred.";

        se->eventState = 1;
        se->eventTimeCount = 0;
        se->eventTimeCount_sub = 0;


        // Initialize agent data
        pAgent[objectID]->InitializeMemory();
        pAgent[objectID]->calInterval = simTime.dt;


        float simHz = 1.0 / simTime.dt;

        // Perception and Recognition; 10[Hz]
        pAgent[objectID]->cognitionCountMax = (int)(simHz / 10.0);
        pAgent[objectID]->cognitionCount = 0;

        // Hazard Identification and Risk Evaluation; 3[Hz]
        pAgent[objectID]->decisionMakingCountMax = (int)(simHz / 3.0);
        pAgent[objectID]->decisionMakingCount = 0;

        // Control; 10[Hz]
        pAgent[objectID]->controlCountMax = (int)(simHz / 10.0);
        pAgent[objectID]->controlCount = 0;



        if( se->eventKind == OBJECT_SCENARIO_ACTION_KIND::APPEAR_VEHICLE ){
            pAgent[objectID]->agentKind = 0;
        }
        else if( se->eventKind == OBJECT_SCENARIO_ACTION_KIND::APPEAR_PEDESTRIAN ){
            pAgent[objectID]->agentKind = 100;
        }


        if( pAgent[objectID]->agentKind < 100 ){

            // Set initial state
            pAgent[objectID]->vehicle.SetVehicleID( pAgent[objectID]->ID );
            pAgent[objectID]->vehicle.SetWinker( 0 );


            float xi  = se->eventFloatData[0];
            float yi  = se->eventFloatData[1];
            float zi  = 0.0;
            float YAi = se->eventFloatData[2] * 0.017452;
            float Vi  = se->eventFloatData[3] / 3.6;

            pAgent[objectID]->state.V = Vi;
            pAgent[objectID]->state.x = xi;
            pAgent[objectID]->state.y = yi;
            pAgent[objectID]->state.z = zi;
            pAgent[objectID]->state.yaw = YAi;
            pAgent[objectID]->state.cosYaw = cos( YAi );
            pAgent[objectID]->state.sinYaw = sin( YAi );

            int vmID = se->eventIntData[0];
            if( vmID < 0 || vmID >= pRoad->vehicleKind.size() ){
                vmID = 0;
            }

            pAgent[objectID]->vehicle.SetVehicleModelID( vmID );  // This ID is used to draw the object in canvas

            {
                pAgent[objectID]->objTypeForUE4 = pRoad->vehicleKind[vmID]->type;
                pAgent[objectID]->objNoForUE4   = pRoad->vehicleKind[vmID]->No;
                if( pRoad->vehicleKind[vmID]->UE4ModelID > 0 ){
                    pAgent[objectID]->objIDForUE4 = pRoad->vehicleKind[vmID]->UE4ModelID;
                }
                else{
                    pAgent[objectID]->objIDForUE4 = -1;
                }

                float L = pRoad->vehicleKind[vmID]->length;
                float W = pRoad->vehicleKind[vmID]->width;
                float H = pRoad->vehicleKind[vmID]->height;

                pAgent[objectID]->vehicle.SetVehicleParam( L, W, H, L * 0.8, L * 0.1, 1.0 );

                pAgent[objectID]->vHalfLength = L * 0.5;
                pAgent[objectID]->vHalfWidth  = W  * 0.5;
            }


            // Set route information
            if( se->eventIntData[2] == 0 ){

                pAgent[objectID]->memory.routeType = ROUTE_TYPE::NODE_LIST_TYPE;

                pAgent[objectID]->memory.routeIndex = -1;
                for(int k=0;k<pRoad->odRoute.size();++k){
                    if( pRoad->odRoute[k]->onlyForScenarioVehicle == false ){
                        continue;
                    }
                    if( pRoad->odRoute[k]->relatedScenarioObjectID == objectID ){

                        pAgent[objectID]->memory.routeIndex = k;
                        break;
                    }
                }
                if( pAgent[objectID]->memory.routeIndex < 0 ){
                    qDebug() << "-------";
                    qDebug() << "Trying to generate Scenario agent ID = " << objectID << ": routeType = NODE_LIST_TYPE";
                    qDebug() << "Can not find route data for the object.";
                    qDebug() << "Check Road Data.";
                    qDebug() << "-------";
                    continue;
                }

                int i = pAgent[objectID]->memory.routeIndex;

                //
                // Set targetPathList
                //
                pAgent[objectID]->memory.routeLaneIndex = -1;

                pAgent[objectID]->memory.destinationNode = pRoad->odRoute[i]->destinationNode;

                if( pRoad->odRoute[i]->LCSupportLaneLists.size() > 0 ){    // new version

                    int numLaneLists = pRoad->odRoute[i]->LCSupportLaneLists.last()->laneList.size();

                    int baseLane = se->eventIntData[1];
                    if( baseLane < 0 ){
                        float tdev,txt,tyt,txd,tyd,ts;
                        baseLane = pRoad->GetNearestPath( xi, yi, YAi,
                                                          tdev, txt, tyt, txd, tyd, ts );
                    }

                    int selIdx = 0;
                    if( numLaneLists > 1 ){

                        bool foundFromLists = false;
                        for(int k=0;k<numLaneLists;++k){
                            if( pRoad->odRoute[i]->LCSupportLaneLists.last()->laneList[k].lastIndexOf( baseLane ) >= 0 ){
                                foundFromLists = true;
                                selIdx = k;
                                break;
                            }
                        }
                        if( foundFromLists == false ){
                            float rnd = rndGen.GenUniform();
                            float H = 1.0 / (float)numLaneLists;
                            for(int k=0;k<numLaneLists;++k){
                                if( rnd >= k * H && rnd < (k+1) * H ){
                                    selIdx = k;
                                    break;
                                }
                            }
                        }
                    }

                    if( pRoad->odRoute[i]->LCSupportLaneLists.size() > 1 ){
                        pAgent[objectID]->memory.LCStartRouteIndex = pRoad->odRoute[i]->LCSupportLaneLists.last()->gIndexInNodeList;
                    }
                    else{
                        pAgent[objectID]->memory.LCStartRouteIndex = -1;
                    }

                    pAgent[objectID]->memory.LCSupportRouteLaneIndex = pRoad->odRoute[i]->LCSupportLaneLists.size() - 1;
                    pAgent[objectID]->memory.routeLaneIndex = selIdx;
                    pAgent[objectID]->memory.targetPathList = pRoad->odRoute[i]->LCSupportLaneLists.last()->laneList[selIdx];


                    float tdev,txt,tyt,txd,tyd,ts;
                    int currentPath = pRoad->GetNearestPathFromListWithZ( pAgent[objectID]->memory.targetPathList,
                                                                          xi, yi, YAi,
                                                                          tdev,txt,tyt,zi,txd,tyd,ts );
                    if( currentPath < 0 ){
                        qDebug() << "[Warning]----------------------------------";
                        qDebug() << " Scenario Vehicle ID = " << objectID << " cannot determin nearest path from assigned list.";
                        qDebug() << "   Assigned Path List : ";
                        for(int j=0;j<pAgent[objectID]->memory.targetPathList.size();++j){
                            qDebug() << "           Path " << pAgent[objectID]->memory.targetPathList[j];
                        }
                        continue;
                    }

                    pAgent[objectID]->vehicle.SetInitialState( Vi, xi, yi, zi, YAi );

                    pAgent[objectID]->state.z = zi;
                    pAgent[objectID]->state.z_path = zi;

                    pAgent[objectID]->memory.currentTargetPath = currentPath;

                    int tpIdx = pRoad->pathId2Index.lastIndexOf( currentPath );
                    if( tpIdx >= 0 ){
//                        qDebug() << "Call SetTargetSpeedIndividual: ID = " << objectID << " Path=" << currentPath
//                                 << " idx=" << tpIdx << " chk=" << pRoad->paths[tpIdx]->id
//                                 << " V85=" << pRoad->paths[tpIdx]->speed85pt;
                        if( pRoad->paths[tpIdx]->speed85pt == pRoad->paths[tpIdx]->speedInfo ){
                            pAgent[objectID]->refSpeedMode = 1;
                        }
                        else{
                            pAgent[objectID]->refSpeedMode = 0;
                        }
                        pAgent[objectID]->SetTargetSpeedIndividual( pRoad->paths[tpIdx]->speed85pt );
                    }


                    pAgent[objectID]->memory.laneMerge.clear();

                    int MLIidx = 0;
                    for(int k=0;k<pAgent[objectID]->memory.LCSupportRouteLaneIndex;++k){
                        MLIidx += pRoad->odRoute[i]->LCSupportLaneLists[k]->laneList.size();
                    }
                    MLIidx += pAgent[objectID]->memory.routeLaneIndex;

                    for(int k=0;k<pRoad->odRoute[i]->mergeLanesInfo[MLIidx].size();++k){

                        QPoint pairData;
                        pairData.setX( pRoad->odRoute[i]->mergeLanesInfo[MLIidx][k].x() );
                        pairData.setY( pRoad->odRoute[i]->mergeLanesInfo[MLIidx][k].y() );

                        pAgent[objectID]->memory.laneMerge.append( pairData );
                    }

                }
                else{
                    qDebug() << "-------";
                    qDebug() << "Trying to generate Scenario agent ID = " << objectID << ": routeType = NODE_LIST_TYPE";
                    qDebug() << "Can not find LCSupportLanes for the object.";
                    qDebug() << "Check Road Data.";
                    qDebug() << "-------";
                    continue;
                }


                pAgent[objectID]->memory.targetPathLength.clear();
                for(int j=0;j<pAgent[objectID]->memory.targetPathList.size();++j){

                    float len = pRoad->GetPathLength( pAgent[objectID]->memory.targetPathList[j] );
                    pAgent[objectID]->memory.targetPathLength.append( len );

                    if( pAgent[objectID]->memory.targetPathList[j] == pAgent[objectID]->memory.currentTargetPath ){
                        pAgent[objectID]->memory.currentTargetPathIndexInList = j;
                    }
                }

                pAgent[objectID]->param.speedVariationFactor = rndGen.GetNormalDist(0.0,0.3);
                if( pAgent[objectID]->param.speedVariationFactor > 1.5 ){
                    pAgent[objectID]->param.speedVariationFactor = 1.5;
                }
                else if( pAgent[objectID]->param.speedVariationFactor < -0.8 ){
                    pAgent[objectID]->param.speedVariationFactor = -0.8;
                }

                pAgent[objectID]->param.latAccelAtTurn = ( 0.20 + pAgent[objectID]->param.speedVariationFactor * 0.07) * 9.81;

                pAgent[objectID]->SetTargetNodeListByTargetPaths( pRoad );

                pAgent[objectID]->BackupMemory();

            }
            else if( se->eventIntData[2] == 1 ){

                pAgent[objectID]->memory.routeType = ROUTE_TYPE::PATH_LIST_TYPE;

                pAgent[objectID]->memory.routeIndex = -1;
                pAgent[objectID]->memory.routeLaneIndex = -1;
                pAgent[objectID]->memory.myNodeList.clear();
                pAgent[objectID]->memory.myInDirList.clear();
                pAgent[objectID]->memory.myOutDirList.clear();
                pAgent[objectID]->memory.myTurnDirectionList.clear();

                pAgent[objectID]->memory.currentTargetNode = -1;
                pAgent[objectID]->memory.currentTargetNodeIndexInNodeList = -1;
                pAgent[objectID]->memory.destinationNode = -1;

                pAgent[objectID]->memory.LCStartRouteIndex = -1;
                pAgent[objectID]->memory.LCSupportRouteLaneIndex = -1;


                pAgent[objectID]->memory.targetPathList = se->targetPathList;

                pAgent[objectID]->memory.targetPathLength.clear();
                for(int j=0;j<pAgent[objectID]->memory.targetPathList.size();++j){
                    int pidx = pRoad->pathId2Index.indexOf( pAgent[objectID]->memory.targetPathList[j] );
                    if( pidx >= 0 ){
                        pAgent[objectID]->memory.targetPathLength.append( pRoad->paths[pidx]->pathLength );
                    }
                }

                float tdev,txt,tyt,txd,tyd,ts;
                int currentPath = pRoad->GetNearestPathFromListWithZ( pAgent[objectID]->memory.targetPathList,
                                                                      xi, yi, YAi,
                                                                      tdev,txt,tyt,zi,txd,tyd,ts );
                if( currentPath < 0 ){
                    qDebug() << "[Warning]----------------------------------";
                    qDebug() << " Scenario Vehicle ID = " << objectID << " cannot determin nearest path from assigned list.";
                    qDebug() << "   Assigned Path List : ";
                    for(int j=0;j<pAgent[objectID]->memory.targetPathList.size();++j){
                        qDebug() << "           Path " << pAgent[objectID]->memory.targetPathList[j];
                    }
                    continue;
                }

                pAgent[objectID]->vehicle.SetInitialState( Vi, xi, yi, zi, YAi );

                pAgent[objectID]->state.z = zi;
                pAgent[objectID]->state.z_path = zi;

                pAgent[objectID]->memory.currentTargetPath = currentPath;

                pAgent[objectID]->memory.scenarioPathSelectID = -1;
            }


            qDebug() << "objectID=" << objectID << " currentTargetPath=" << pAgent[objectID]->memory.currentTargetPath;

            pAgent[objectID]->memory.controlMode = AGENT_CONTROL_MODE::AGENT_LOGIC;


            pAgent[objectID]->agentStatus = 1;
            qDebug() << "Generate Agent[Scenario] : objectID = " << objectID;

            pAgent[objectID]->isScenarioObject = true;
            pAgent[objectID]->isOldScenarioType = false;
            pAgent[objectID]->canAppearRepeatedly = se->eventBooleanData[0];

            pAgent[objectID]->isSInterfaceObject = false;
            pAgent[objectID]->isBehaviorEmbeded  = false;
            pAgent[objectID]->justWarped = false;

            pAgent[objectID]->TimeOfAppear = GetSimulationTimeInSec();

            // Fluctuation in Speed Control
            int tpIdx = pRoad->pedestPathID2Index.indexOf( pAgent[objectID]->memory.currentTargetPath );
            if( tpIdx >= 0 && pRoad->paths[tpIdx]->setSpeedVariationParam == true ){

                pAgent[objectID]->param.refVforDev     = pRoad->paths[tpIdx]->refVforDev;
                pAgent[objectID]->param.vDevAllowPlus  = pRoad->paths[tpIdx]->vDevP;
                pAgent[objectID]->param.vDevAllowMinus = pRoad->paths[tpIdx]->vDevM;
                pAgent[objectID]->param.accelAtVDev    = pRoad->paths[tpIdx]->accelAtDev;
                pAgent[objectID]->param.decelAtVDev    = pRoad->paths[tpIdx]->decelAtDev;
                pAgent[objectID]->param.deadZoneSpeedControl = 0.0;

            }

            if( DSMode == true ){

                if( pAgent[objectID]->vehicle.steerFiltered4CG == NULL ){
                    pAgent[objectID]->vehicle.steerFiltered4CG = new LowPassFilter(1, simTime.dt, 6.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.steerFiltered4CG->SetParam( 1, simTime.dt, 6.0 , 0.0 );
                    pAgent[objectID]->vehicle.steerFiltered4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.axFiltered4CG == NULL ){
                    pAgent[objectID]->vehicle.axFiltered4CG = new LowPassFilter(1, simTime.dt, 6.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.axFiltered4CG->SetParam( 1, simTime.dt, 6.0 , 0.0 );
                    pAgent[objectID]->vehicle.axFiltered4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.ayFiltered4CG == NULL ){
                    pAgent[objectID]->vehicle.ayFiltered4CG = new LowPassFilter(1, simTime.dt, 6.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.ayFiltered4CG->SetParam( 1, simTime.dt, 6.0 , 0.0 );
                    pAgent[objectID]->vehicle.ayFiltered4CG->Reset();
                }


                if( pAgent[objectID]->vehicle.suspentionFL4CG == NULL ){
                    pAgent[objectID]->vehicle.suspentionFL4CG = new LowPassFilter(2, simTime.dt, 10.0 , 0.6);
                }
                else{
                    pAgent[objectID]->vehicle.suspentionFL4CG->SetParam(2, simTime.dt, 10.0 , 0.6);
                    pAgent[objectID]->vehicle.suspentionFL4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.suspentionFR4CG == NULL ){
                    pAgent[objectID]->vehicle.suspentionFR4CG = new LowPassFilter(2, simTime.dt, 10.0 , 0.6);
                }
                else{
                    pAgent[objectID]->vehicle.suspentionFR4CG->SetParam(2, simTime.dt, 10.0 , 0.6);
                    pAgent[objectID]->vehicle.suspentionFR4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.suspentionRL4CG == NULL ){
                    pAgent[objectID]->vehicle.suspentionRL4CG = new LowPassFilter(2, simTime.dt, 10.0 , 0.6);
                }
                else{
                    pAgent[objectID]->vehicle.suspentionRL4CG->SetParam(2, simTime.dt, 10.0 , 0.6);
                    pAgent[objectID]->vehicle.suspentionRL4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.suspentionRR4CG == NULL ){
                    pAgent[objectID]->vehicle.suspentionRR4CG = new LowPassFilter(2, simTime.dt, 10.0 , 0.6);
                }
                else{
                    pAgent[objectID]->vehicle.suspentionRR4CG->SetParam(2, simTime.dt, 10.0 , 0.6);
                    pAgent[objectID]->vehicle.suspentionRR4CG->Reset();
                }


                if( pAgent[objectID]->vehicle.tireFL4CG == NULL ){
                    pAgent[objectID]->vehicle.tireFL4CG = new LowPassFilter(1, simTime.dt, 5.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.tireFL4CG->SetParam(1, simTime.dt, 5.0 , 0.0);
                    pAgent[objectID]->vehicle.tireFL4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.tireFR4CG == NULL ){
                    pAgent[objectID]->vehicle.tireFR4CG = new LowPassFilter(1, simTime.dt, 5.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.tireFR4CG->SetParam(1, simTime.dt, 5.0 , 0.0);
                    pAgent[objectID]->vehicle.tireFR4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.tireRL4CG == NULL ){
                    pAgent[objectID]->vehicle.tireRL4CG = new LowPassFilter(1, simTime.dt, 5.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.tireRL4CG->SetParam(1, simTime.dt, 5.0 , 0.0);
                    pAgent[objectID]->vehicle.tireRL4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.tireRR4CG == NULL ){
                    pAgent[objectID]->vehicle.tireRR4CG = new LowPassFilter(1, simTime.dt, 5.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.tireRR4CG->SetParam(1, simTime.dt, 5.0 , 0.0);
                    pAgent[objectID]->vehicle.tireRR4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.yawFiltered4CG == NULL ){
                    pAgent[objectID]->vehicle.yawFiltered4CG = new LowPassFilter(1, simTime.dt, 3.0 , 0.0);
                }
                pAgent[objectID]->vehicle.yawFiltered4CG->SetParam(1,simTime.dt, 3.0 , 0.0);
                pAgent[objectID]->vehicle.yawFiltered4CG->SetInitialValue( pAgent[objectID]->state.yaw );
            }
        }
        else if( pAgent[objectID]->agentKind >= 100 ){

            // Set initial state
            pAgent[objectID]->vehicle.SetVehicleID( pAgent[objectID]->ID );
            pAgent[objectID]->vehicle.SetWinker( 0 );

            int pedestMdlIdx = se->eventIntData[0];

            pAgent[objectID]->agentKind = 100 + pedestMdlIdx;
            pAgent[objectID]->vehicle.SetVehicleModelID( pedestMdlIdx );

            if( pedestMdlIdx < pRoad->pedestrianKind.size() ){

                pAgent[objectID]->objTypeForUE4 = pRoad->pedestrianKind[pedestMdlIdx]->type;
                pAgent[objectID]->objNoForUE4   = pRoad->pedestrianKind[pedestMdlIdx]->No;
                if( pRoad->pedestrianKind[pedestMdlIdx]->UE4ModelID > 0 ){
                    pAgent[objectID]->objIDForUE4 = pRoad->pedestrianKind[pedestMdlIdx]->UE4ModelID;
                }
                else{
                    pAgent[objectID]->objIDForUE4 = -1;
                }

                pAgent[objectID]->vHalfWidth  = pRoad->pedestrianKind[pedestMdlIdx]->width * 0.5;
                pAgent[objectID]->vHalfLength = pRoad->pedestrianKind[pedestMdlIdx]->length * 0.5;

                // Assume minimum size of pedestrian as a circle of 1[m]
//                if( pAgent[objectID]->vHalfWidth < 0.5 ){
//                    pAgent[objectID]->vHalfWidth = 0.5;
//                }
//                if( pAgent[objectID]->vHalfLength < 0.5 ){
//                    pAgent[objectID]->vHalfLength = 0.5;
//                }
            }


            pAgent[objectID]->state.V = se->eventFloatData[3];
            pAgent[objectID]->state.x = se->eventFloatData[0];
            pAgent[objectID]->state.y = se->eventFloatData[1];
            pAgent[objectID]->state.z = 0.0;
            pAgent[objectID]->state.yaw = se->eventFloatData[2] * 0.017452;
            pAgent[objectID]->state.cosYaw = cos( pAgent[objectID]->state.yaw );
            pAgent[objectID]->state.sinYaw = sin( pAgent[objectID]->state.yaw );

            pAgent[objectID]->memory.targetSpeedByScenario = se->eventFloatData[3];

            pAgent[objectID]->memory.currentTargetPath = -1;
            for(int k=0;k<pRoad->pedestPaths.size();++k){
                if( pRoad->pedestPaths[k]->scenarioObjectID == pAgent[objectID]->ID ){
                    pAgent[objectID]->memory.currentTargetPath = pRoad->pedestPaths[k]->id;
                    break;
                }
            }
            if( pAgent[objectID]->memory.currentTargetPath < 0 ){
                qDebug() << "[Warning]----------------------------------";
                qDebug() << " Scenario Pedestrian ID = " << objectID << " cannot determin target path. No pedestPath for this pedestrian.";
                continue;
            }

            pAgent[objectID]->memory.currentTargetPathIndexInList = 0;

            pAgent[objectID]->memory.targetPathList.clear();
            pAgent[objectID]->memory.targetPathList.append( pAgent[objectID]->memory.currentTargetPath );

            pAgent[objectID]->memory.lateralShiftTarget = 0.0;
            pAgent[objectID]->memory.controlMode = AGENT_CONTROL_MODE::AGENT_LOGIC;

            {
                float rnd = this->GenUniform();
                if( rnd <= 0.0638 ){
                    pAgent[objectID]->attri.age = 0;
                    pAgent[objectID]->memory.targetSpeed = GetNormalDist( 1.339, 0.107 );
                }
                else if( rnd >= 1.0 - 0.2637 ){
                    pAgent[objectID]->attri.age = 2;
                    pAgent[objectID]->memory.targetSpeed = GetNormalDist( 1.337, 0.104 );
                }
                else{
                    pAgent[objectID]->attri.age = 1;
                    pAgent[objectID]->memory.targetSpeed = GetNormalDist( 1.358, 0.093 );
                }

                if( pAgent[objectID]->memory.targetSpeed <= 0.55 ){
                    pAgent[objectID]->memory.targetSpeed = 0.55;
                }
            }


            // Set Flags
            pAgent[objectID]->agentStatus = 1;
            //qDebug() << "Generate Agent[Pedestrian] : objID = " << objID;

            pAgent[objectID]->isScenarioObject = true;
            pAgent[objectID]->isOldScenarioType = false;
            pAgent[objectID]->canAppearRepeatedly = se->eventBooleanData[0];

            pAgent[objectID]->BackupMemory();

            pAgent[objectID]->TimeOfAppear = GetSimulationTimeInSec();

            if( DSMode == true ){

                if( pAgent[objectID]->vehicle.yawFiltered4CG == NULL ){
                    pAgent[objectID]->vehicle.yawFiltered4CG = new LowPassFilter(1, simTime.dt, 10.0 , 0.0);
                }

                pAgent[objectID]->vehicle.yawFiltered4CG->SetParam(1,simTime.dt, 10.0 , 0.0);
                pAgent[objectID]->vehicle.yawFiltered4CG->SetInitialValue( pAgent[objectID]->state.yaw );
            }
        }
    }
}



void SimulationManager::DisappearAgents(Agent **pAgent, int maxAgent)
{
    for(int i=0;i<maxAgent;++i){

        if( pAgent[i]->agentStatus == 2 ){

            //
            // Implement some logic for counting somthing if necessary
            //

            //qDebug() << "[DisappearAgents] ID = " << i;

            pAgent[i]->agentStatus = 0;
            pAgent[i]->TimeOfAppear = -1.0;

            if( pAgent[i]->isScenarioObject == true  ){

                //qDebug() << "Scenario Object: canAppearRepeatedly = " << pAgent[i]->canAppearRepeatedly;

                int nItem = scenario[currentScenarioID]->scenarioItems.size();
                for(int j=0;j<nItem;++j){

                    int objectID = scenario[currentScenarioID]->scenarioItems[j]->objectID;
                    if( objectID == pAgent[i]->ID ){
                        // Reset trigger flag
                        ScenarioTriggers *trig = scenario[currentScenarioID]->scenarioItems[j]->appearTriggers;
                        if( trig->mode == TRIGGER_TYPE::BY_FUNCTION_EXTENDER ){
                            trig->extTriggerFlagState = false;
                        }
                        break;
                    }
                }


                if( pAgent[i]->canAppearRepeatedly == true ){

                    nItem = scenario[currentScenarioID]->scenarioEvents.size();
                    for(int j=0;j<nItem;++j){
                        ScenarioEvents *se = scenario[currentScenarioID]->scenarioEvents[j];
                        if( se->eventType != SCENARIO_EVENT_TYPE::OBJECT_EVENT ){
                            continue;
                        }
                        if( se->eventKind != OBJECT_SCENARIO_ACTION_KIND::APPEAR_VEHICLE && se->eventKind != OBJECT_SCENARIO_ACTION_KIND::APPEAR_PEDESTRIAN ){
                            continue;
                        }
                        if( se->targetObjectID != pAgent[i]->ID ){
                            continue;
                        }

                        qDebug() << "CanAppearRepeatedly ID = " << pAgent[i]->ID << ": Reset Appear Event: event[" << j << "]";

                        se->eventState = 0;
                        ScenarioTriggers *trig = se->eventTrigger;

                        trig->extTriggerFlagState = false;

                        for(int k=0;k<trig->objectTigger.size();++k){
                            trig->objectTigger[k]->isTriggered = false;
                            trig->objectTigger[k]->passCheckFlag = 0;
                        }
                        break;
                    }

                }
            }

            pAgent[i]->isScenarioObject = false;
            pAgent[i]->isOldScenarioType = false;
            pAgent[i]->skipSetControlInfo = false;

//            qDebug() << "Agent ID = " << i << " disposed.";
        }
    }
}


void SimulationManager::SetAppearFlagByFE(int id)
{
    if( scenario.size() == 0 ){
        return;
    }

    int nItem = scenario[currentScenarioID]->scenarioEvents.size();
//    qDebug() << "SetAppearFlagByFE: id= " << id << " scenarioEvents.size=" << nItem;
    for(int i=0;i<nItem;++i){

        if( scenario[currentScenarioID]->scenarioEvents[i]->eventType != SCENARIO_EVENT_TYPE::OBJECT_EVENT ){
            continue;
        }

        int objectID = scenario[currentScenarioID]->scenarioEvents[i]->targetObjectID;
        if( objectID != id ){
            continue;
        }

        if( scenario[currentScenarioID]->scenarioEvents[i]->eventKind != OBJECT_SCENARIO_ACTION_KIND::APPEAR_VEHICLE &&
                scenario[currentScenarioID]->scenarioEvents[i]->eventKind != OBJECT_SCENARIO_ACTION_KIND::APPEAR_PEDESTRIAN ){
            continue;
        }

        // Check trigger
        ScenarioTriggers *trig = scenario[currentScenarioID]->scenarioEvents[i]->eventTrigger;
        if( trig->byExternalTriggerFlag == true ){

            trig->extTriggerFlagState = true;
//            qDebug() << "ScenarioObject: ID=" << objectID << " set extTriggerFlagState = true";
            break;
        }
    }

    nItem = scenario[currentScenarioID]->scenarioItems.size();

//    qDebug() << "SetAppearFlagByFE: id= " << id << " scenarioItems.size=" << nItem;
    for(int i=0;i<nItem;++i){

        int objectID = scenario[currentScenarioID]->scenarioItems[i]->objectID;
        if( objectID != id ){
            continue;
        }

        ScenarioTriggers *trig = scenario[currentScenarioID]->scenarioItems[i]->appearTriggers;
        if( trig->mode == TRIGGER_TYPE::BY_FUNCTION_EXTENDER ){
            trig->extTriggerFlagState = true;
//            qDebug() << "ScenarioObject: ID=" << objectID << " set extTriggerFlagState = true";
            break;
        }
    }

}


void SimulationManager::SetScenarioObjectsRouteInfo()
{
    if( scenario.size() == 0 ){
        return;
    }

    for(int i=0;i<scenario.size();++i){

        for(int j=0;j<scenario[i]->scenarioEvents.size();++j){

            if( scenario[i]->scenarioEvents[j]->eventType != SCENARIO_EVENT_TYPE::OBJECT_EVENT ){
                continue;
            }

            if( scenario[i]->scenarioEvents[j]->eventKind == OBJECT_SCENARIO_ACTION_KIND::APPEAR_VEHICLE ){

                int routeType = scenario[i]->scenarioEvents[j]->eventIntData[2];

                if( routeType == 0 ){
                    // Node List Type
                    scenario[i]->scenarioEvents[j]->routeType = ROUTE_TYPE::NODE_LIST_TYPE;
                }
                else if( routeType == 1 ){
                    // Path List Type
                    scenario[i]->scenarioEvents[j]->routeType = ROUTE_TYPE::PATH_LIST_TYPE;

//                    int nElem = scenario[i]->scenarioEvents[j]->eventIntData[3];
                    for(int k=4;k<scenario[i]->scenarioEvents[j]->eventFloatData.size();k+=4){

                        struct ScenarioWPRoute* wp = new struct ScenarioWPRoute;

                        wp->x = scenario[i]->scenarioEvents[j]->eventFloatData[k];
                        wp->y = scenario[i]->scenarioEvents[j]->eventFloatData[k+1];
                        wp->z = scenario[i]->scenarioEvents[j]->eventFloatData[k+2];
                        wp->direct = scenario[i]->scenarioEvents[j]->eventFloatData[k+3] * 0.017452;

                        wp->speedInfo = scenario[i]->scenarioEvents[j]->eventFloatData[3] / 3.6;
                        wp->wpID = -1;

                        scenario[i]->scenarioEvents[j]->wpRoute.append( wp );
                    }
                }
            }
            else if( scenario[i]->scenarioEvents[j]->eventKind == OBJECT_SCENARIO_ACTION_KIND::APPEAR_PEDESTRIAN ){

                scenario[i]->scenarioEvents[j]->routeType = ROUTE_TYPE::PEDEST_PATH_LIST_TYPE;

//                int nElem = scenario[i]->scenarioEvents[j]->eventIntData[2];
                for(int k=4;k<scenario[i]->scenarioEvents[j]->eventFloatData.size();k+=4){

                    struct ScenarioWPRoute* wp = new struct ScenarioWPRoute;

                    wp->x = scenario[i]->scenarioEvents[j]->eventFloatData[k];
                    wp->y = scenario[i]->scenarioEvents[j]->eventFloatData[k+1];
                    wp->z = scenario[i]->scenarioEvents[j]->eventFloatData[k+2];
                    wp->direct = scenario[i]->scenarioEvents[j]->eventFloatData[k+3] * 0.017452;

                    wp->speedInfo = scenario[i]->scenarioEvents[j]->eventFloatData[3];
                    wp->wpID = -1;

                    scenario[i]->scenarioEvents[j]->wpRoute.append( wp );
                }

            }
        }
    }
}
