#include "simulationmanager.h"
#include <windows.h>
#include <QDebug>


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


int SimulationManager::GetVehicleShapeByWheelbase(float wl)
{
    int ret = -1;
    int mismatch = 0.0;
    for(int i=0;i<vehicleShape.size();++i){
        float err = fabs(wl - vehicleShape[i]->wheelBase);
        if(ret < 0 || mismatch > err ){
            ret = i;
            mismatch = err;
        }
    }
    return ret;
}


void SimulationManager::SetVehicleShapeParameter(int ID, int Type, float length, float width, float height, float wheelBase, float distRR2RE, float FRWeightRatio)
{
    if( ID == 0 ){
        if( vehicleShape.size() > 0 ){
            for(int i=0;i<vehicleShape.size();++i){
                delete vehicleShape[i];
            }
            vehicleShape.clear();
        }
    }

    struct VehicleShapeParameter *v = new struct VehicleShapeParameter();

    v->ID                     = ID;
    v->Type                   = Type;
    v->length                 = length;
    v->width                  = width;
    v->height                 = height;
    v->wheelBase              = wheelBase;
    v->distRearAxletoRearEdge = distRR2RE;
    v->FRWeightRatio          = FRWeightRatio;

    vehicleShape.append( v );
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
                    pAgent[objectID]->memory.targetPathListBackup.prepend( pRoad->paths[j]->id );
                }
            }
        }
    }
}

void SimulationManager::AppearAgents(Agent** pAgent,int maxAgentNumber,Road *pRoad)
{
    int nAppear = 0;
    int maxNAppearAtATime = 3;


    if( IDAllowed < 0 ){

        qDebug() << "[SimulationManager::AppearAgents] DS Mode = " << DSMode;

        IDAllowed = 1;
        if( DSMode == true ){
            IDAllowed = 10;  // Reserve 10 agents for S-Interface objects
        }
        if( scenario.size() > 0 ){
            int nItem = scenario[currentScenarioID]->scenarioItems.size();
            for(int i=0;i<nItem;++i){
                int objectID = scenario[currentScenarioID]->scenarioItems[i]->objectID;
                if( IDAllowed < objectID ){
                    IDAllowed = objectID + 1;
                }
            }
        }
        qDebug() << "[SimulationManager::AppearAgents] IDAllowed = " << IDAllowed;

    }


    float simTimeSec = GetSimulationTimeInSec();


    //
    //  By OD Data
    //
    for(int i=0;i<pRoad->odRoute.size();++i){

        if( pRoad->odRoute[i]->meanArrivalTime < 0.0 ){
            continue;
        }

        if( pRoad->odRoute[i]->NextAppearTime <  0.0 ){

            pRoad->odRoute[i]->NextAppearTime = GetExponentialDist( pRoad->odRoute[i]->meanArrivalTime );
            //qDebug() << "OD[" << i << "] : NextAppearTime = " <<  pRoad->odRoute[i]->NextAppearTime;

        }
        else{

            bool genFlag = false;
            if( simTimeSec >= pRoad->odRoute[i]->NextAppearTime ){
                if( nAppear < maxNAppearAtATime ){
                    genFlag = true;
                    nAppear++;

                    // Set next timing
                    pRoad->odRoute[i]->NextAppearTime = simTimeSec;
                    pRoad->odRoute[i]->NextAppearTime += GetExponentialDist( pRoad->odRoute[i]->meanArrivalTime );
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



            qDebug() << "Try to generate : objID = " << objID;

            //
            // Set targetPathList
            //
            int onIdx = pRoad->nodeId2Index.indexOf( pRoad->odRoute[i]->originNode );

            QVector<QVector<int>> targetPathLists;
            QVector<bool> needLCs;
            QVector<int> nodeUntils;

            // List up All possible path list
            for(int j=0;j<pRoad->nodes[onIdx]->outBoundaryWPs.size();++j){

                qDebug() << "outBoundaryWPs = " << pRoad->nodes[onIdx]->outBoundaryWPs[j]->wpId;

                for(int k=0;k<pRoad->nodes[onIdx]->outBoundaryWPs[j]->PathWithSWP.size();++k){

                    qDebug() << "Path = " << pRoad->nodes[onIdx]->outBoundaryWPs[j]->PathWithSWP[k];

                    int tmpCurPath = pRoad->nodes[onIdx]->outBoundaryWPs[j]->PathWithSWP[k];

                    qDebug() << "Now Call Road::GetPathList";

                    bool needLC = false;
                    int nodeUntil = -1;
                    QVector<int> tmpTargetPathList = pRoad->GetPathList( i, tmpCurPath, needLC, nodeUntil, &(this->rndGen) );


                    qDebug() << "GetPathList Result for outBoundart-WP = " << pRoad->nodes[onIdx]->outBoundaryWPs[j]->wpId
                             << " and Path = " << tmpCurPath;
                    qDebug() << "  tmpTargetPathList.size = " << tmpTargetPathList.size();
                    qDebug() << "  tmpTargetPathList = " << tmpTargetPathList;
                    qDebug() << "  needLC = " << needLC << " nodeUntil = " << nodeUntil;

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

            qDebug() << "numNoNeedLCPaths = " << numNoNeedLCPaths;

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

                            qDebug() << "targetPathList = " << pAgent[objID]->memory.targetPathList;
                            qDebug() << "currentTargetPath = " << pAgent[objID]->memory.currentTargetPath;

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


            if(  pAgent[objID]->memory.currentTargetPath < 0 ){
                qDebug() << "-------";
                qDebug() << "Trying to generate agent ID = " << objID << ": from Node "
                         << pRoad->odRoute[i]->originNode << " to "
                         << pRoad->odRoute[i]->destinationNode;
                qDebug() << "Failed to set currentTargetPath.";
                qDebug() << "-------";
                continue;
            }



            // Initial Position Data
            int tpIdx = pRoad->pathId2Index.indexOf( pAgent[objID]->memory.currentTargetPath );

            float xi  = pRoad->paths[tpIdx]->pos.first()->x();
            float yi  = pRoad->paths[tpIdx]->pos.first()->y();
            float zi  = pRoad->paths[tpIdx]->pos.first()->z();
            float cYAi = pRoad->paths[tpIdx]->derivative.first()->x();
            float sYAi = pRoad->paths[tpIdx]->derivative.first()->y();
            float YAi = atan2( sYAi , cYAi );
            float Vi  = 0.0;



            // Check the lane has enough space for new vehicle
            bool enoughSpace = true;
            for(int i=0;i<maxAgentNumber;++i){
                if( pAgent[i]->agentStatus != 1 ){
                    continue;
                }
                if( pAgent[i]->memory.currentTargetPath != pAgent[objID]->memory.currentTargetPath ){
                    continue;
                }
                if( pAgent[i]->state.V > 5.0 ){
                    continue;
                }

                float dx = pAgent[i]->state.x - xi;
                float dy = pAgent[i]->state.y - yi;
                float L = dx * cYAi + dy * sYAi;
                if( L < 10.0 ){
                    enoughSpace = false;
                    break;
                }
            }
            if( enoughSpace == false ){
                continue;
            }



            // set vehicle size
            qDebug() << "Set Vehicle Type and Size";

            pAgent[objID]->agentKind = 0;

            pAgent[objID]->vehicle.SetVehicleModelID( 0 );
            for(int j=0;j<vehicleShape.size();++j){
                if( vehicleShape[j]->ID == 0 ){
                    pAgent[objID]->vehicle.SetVehicleParam(  vehicleShape[j]->length,
                                                                vehicleShape[j]->width,
                                                                vehicleShape[j]->height,
                                                                vehicleShape[j]->wheelBase,
                                                                vehicleShape[j]->distRearAxletoRearEdge,
                                                                vehicleShape[j]->FRWeightRatio );

                    pAgent[objID]->vHalfLength = vehicleShape[j]->length * 0.5;
                    pAgent[objID]->vHalfWidth  = vehicleShape[j]->width  * 0.5;

                    qDebug() << "VHalfLen = " << pAgent[objID]->vHalfLength
                             << " VHalfWid = " << pAgent[objID]->vHalfWidth;
                }
            }





            // Set initial state
            qDebug() << "Set Vehicle Initial State";

            pAgent[objID]->vehicle.SetVehicleID( objID );



            pAgent[objID]->vehicle.SetInitialState( Vi, xi, yi, zi, YAi );  // Yaw in [rad]

            pAgent[objID]->state.V = Vi;
            pAgent[objID]->state.x = xi;
            pAgent[objID]->state.y = yi;
            pAgent[objID]->state.z = zi;
            pAgent[objID]->state.yaw = YAi;
            pAgent[objID]->state.cosYaw = cos( YAi );
            pAgent[objID]->state.sinYaw = sin( YAi );

            // Set Control Information
            pAgent[objID]->memory.controlMode = AGENT_CONTROL_MODE::AGENT_LOGIC;
            pAgent[objID]->memory.targetSpeed = pRoad->paths[tpIdx]->speedInfo;

            pAgent[objID]->agentStatus = 1;
            pAgent[objID]->isScenarioObject = false;

            pAgent[objID]->isSInterfaceObject = false;
            pAgent[objID]->isBehaviorEmbeded  = false;
            pAgent[objID]->justWarped = false;

            pAgent[objID]->memory.routeType  = AGENT_ROUTE_TYPE::NODE_LIST;
            pAgent[objID]->memory.routeIndex = i;

            pAgent[objID]->SetTargetNodeListByTargetPaths( pRoad );

            qDebug() << "Generate Agent : objID = " << objID;
        }

    }




    //
    //  By Scenario
    //
    if( scenario.size() == 0 ){
        return;
    }

    nAppear = 0;

    int nItem = scenario[currentScenarioID]->scenarioItems.size();
    //qDebug() << "nItem = " << nItem;
    for(int i=0;i<nItem;++i){

        int objectID = scenario[currentScenarioID]->scenarioItems[i]->objectID;

        if( pAgent[objectID]->agentStatus != 0 ){
            // Already appear or Out from the stage
            continue;
        }

        if( scenario[currentScenarioID]->scenarioItems[i]->status == 1 &&
                scenario[currentScenarioID]->scenarioItems[i]->repeat == false ){
            continue;
        }

        if( pAgent[objectID]->notAllowedAppear == true ){
            continue;
        }

        // Check trigger
        ScenarioTriggers *trig = scenario[currentScenarioID]->scenarioItems[i]->appearTriggers;
        //qDebug() << "objectID = " << objectID << " trigger mode = " << trig->mode;
        if( trig->mode == TRIGGER_TYPE::TIME_TRIGGER ||
            trig->mode == TRIGGER_TYPE::POSITION_TIME_TRIGGER ){

            if( trig->timeTriggerInSec > simTimeSec ){
                continue;
            }
        }

        if( trig->mode == TRIGGER_TYPE::POSITION_TRIGGER ||
            trig->mode == TRIGGER_TYPE::POSITION_SPEED_TRIGGER ||
            trig->mode == TRIGGER_TYPE::POSITION_TIME_TRIGGER ){

            int nCondPass = 0;
            for(int j=0;j<trig->objectTigger.size();++j){
                int targetObjectID = trig->objectTigger[j]->targetObjectID;
                //qDebug() << "targetObjectID = " << targetObjectID;
                if( targetObjectID < 0 || targetObjectID >= maxAgentNumber ){
                    continue;
                }
                if( pAgent[targetObjectID]->agentStatus != 1 ){
                    continue;
                }

                float dx = pAgent[targetObjectID]->state.x - trig->objectTigger[j]->x;
                float dy = pAgent[targetObjectID]->state.y - trig->objectTigger[j]->y;
                float ip = dx * trig->objectTigger[j]->cosDirect + dy * trig->objectTigger[j]->sinDirect;
//                qDebug() << "target (x,y) = (" << pAgent[targetObjectID]->state.x << "," << pAgent[targetObjectID]->state.y << ")";
//                qDebug() << "trigger (x,y) = (" << trig->objectTigger[j]->x << "," << trig->objectTigger[j]->y << ")";
//                qDebug() << "ip = " << ip;

                if( ip < 0.0 ){
                    continue;
                }
                float cp = dy * trig->objectTigger[j]->cosDirect - dx * trig->objectTigger[j]->sinDirect;
                //qDebug() << "cp = " << cp;
                if( fabs(cp) > 2.5 ){
                    continue;
                }
                float ip2 = pAgent[targetObjectID]->state.cosYaw * trig->objectTigger[j]->cosDirect
                        + pAgent[targetObjectID]->state.sinYaw * trig->objectTigger[j]->sinDirect;
                //qDebug() << "ip2 = " << ip2;
                if( ip2 < 0.0 ){
                    continue;
                }

                if( trig->mode == TRIGGER_TYPE::POSITION_SPEED_TRIGGER ){
                    if( pAgent[targetObjectID]->state.V < trig->objectTigger[j]->speed ){
                        continue;
                    }
                }

                nCondPass++;
            }
            if( trig->ANDCondition == 0 && nCondPass == 0 ){
                continue;
            }
            else if( trig->ANDCondition == 1 && nCondPass < trig->objectTigger.size() ){
                continue;
            }
        }


        if( nAppear < maxNAppearAtATime ){
            nAppear++;
        }
        else{
            continue;
        }


        if( trig->mode == TRIGGER_TYPE::BY_FUNCTION_EXTENDER ){
            if( trig->byExternalTriggerFlag == 0 ){
                continue;
            }
        }

        if( trig->mode == TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
            int keyHit = 0;
            switch( trig->func_keys ){
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
            }
            if( keyHit == 0 ){
                continue;
            }
        }

        // Initialize agent data
        pAgent[objectID]->InitializeMemory();
        pAgent[objectID]->calInterval = simTime.dt;

        if( scenario[currentScenarioID]->scenarioItems[i]->type == 'v' ){
            pAgent[objectID]->agentKind = 0;
        }
        else if( scenario[currentScenarioID]->scenarioItems[i]->type == 'p' ){
            pAgent[objectID]->agentKind = 100;
        }


        if( pAgent[objectID]->agentKind < 100 ){

            // Set initial state
            pAgent[objectID]->vehicle.SetVehicleID( pAgent[objectID]->ID );

            float xi  = scenario[currentScenarioID]->scenarioItems[i]->initialState[0];
            float yi  = scenario[currentScenarioID]->scenarioItems[i]->initialState[1];
            float zi  = scenario[currentScenarioID]->scenarioItems[i]->initialState[2];
            float YAi = scenario[currentScenarioID]->scenarioItems[i]->initialState[3];
            float Vi  = scenario[currentScenarioID]->scenarioItems[i]->initialState[4];
            pAgent[objectID]->vehicle.SetInitialState( Vi, xi, yi, zi, YAi );

            pAgent[objectID]->state.V = Vi;
            pAgent[objectID]->state.x = xi;
            pAgent[objectID]->state.y = yi;
            pAgent[objectID]->state.z = zi;
            pAgent[objectID]->state.yaw = YAi;
            pAgent[objectID]->state.cosYaw = cos( YAi );
            pAgent[objectID]->state.sinYaw = sin( YAi );


            pAgent[objectID]->vehicle.SetVehicleModelID( scenario[currentScenarioID]->scenarioItems[i]->objectModelID );
            for(int j=0;j<vehicleShape.size();++j){
                if( vehicleShape[j]->ID == scenario[currentScenarioID]->scenarioItems[i]->objectModelID ){
                    pAgent[objectID]->vehicle.SetVehicleParam(  vehicleShape[j]->length,
                                                                vehicleShape[j]->width,
                                                                vehicleShape[j]->height,
                                                                vehicleShape[j]->wheelBase,
                                                                vehicleShape[j]->distRearAxletoRearEdge,
                                                                vehicleShape[j]->FRWeightRatio );

                    pAgent[objectID]->vHalfLength = vehicleShape[j]->length * 0.5;
                    pAgent[objectID]->vHalfWidth  = vehicleShape[j]->width  * 0.5;
                }
            }

            // Set route information
            ScenarioObjectControlInfo* controlInfo = scenario[currentScenarioID]->scenarioItems[i]->controlInfo;

            if( controlInfo->routeType == ROUTE_TYPE::NODE_LIST_TYPE ){

                pAgent[objectID]->memory.routeType = ROUTE_TYPE::NODE_LIST_TYPE;

                // Not yet implemented
                continue;
            }
            else if( controlInfo->routeType == ROUTE_TYPE::PATH_LIST_TYPE ){

                pAgent[objectID]->memory.routeType = ROUTE_TYPE::PATH_LIST_TYPE;

                int pathSelectID = objectID;
                if( pAgent[objectID]->memory.scenarioPathSelectID > 0 ){
                    pathSelectID = pAgent[objectID]->memory.scenarioPathSelectID;
                }

                for(int j=0;j<pRoad->paths.size();++j){
                    if( pRoad->paths[j]->scenarioObjectID != pathSelectID ){
                        continue;
                    }
                    pAgent[objectID]->memory.targetPathList.prepend( pRoad->paths[j]->id );
                    pAgent[objectID]->memory.targetPathListBackup.prepend( pRoad->paths[j]->id );
                }

                float dist = 0;
                int currentPath = pRoad->GetNearestPathFromList( xi, yi, YAi, dist, pAgent[objectID]->memory.targetPathList );
                if( currentPath < 0 ){
                    qDebug() << "[Warning]----------------------------------";
                    qDebug() << " Scenario Vehicle ID = " << objectID << " cannot determin nearest path from assigned list.";
                    qDebug() << "   Assigned Path List : ";
                    for(int j=0;j<pAgent[objectID]->memory.targetPathList.size();++j){
                        qDebug() << "           Path " << pAgent[objectID]->memory.targetPathList[j];
                    }
                    continue;
                }

                pAgent[objectID]->memory.currentTargetPath = currentPath;

                pAgent[objectID]->memory.scenarioPathSelectID = -1;
            }


            qDebug() << "objectID=" << objectID << " currentTargetPath=" << pAgent[objectID]->memory.currentTargetPath;


            // Set control information
            if( pAgent[objectID]->skipSetControlInfo == false ){

		            pAgent[objectID]->memory.controlMode = controlInfo->mode;
		
		            if( pAgent[objectID]->memory.controlMode == AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY ){
		
		                pAgent[objectID]->memory.precedingVehicleIDByScenario = controlInfo->targetObjectID;
		
		                pAgent[objectID]->memory.targetSpeedByScenario = controlInfo->targetSpeed;
		
		                pAgent[objectID]->memory.targetHeadwayDistanceByScenario = controlInfo->targetHeadwayDistance;
		                pAgent[objectID]->memory.targetHeadwayTimeByScenario     = controlInfo->targetHeadwayTime;
		
		                pAgent[objectID]->memory.doHeadwayDistanceControl = false;  // This flag is set when detected preceding vehicle
		                pAgent[objectID]->memory.doStopControl = false;
		
		            }
		            else if( pAgent[objectID]->memory.controlMode == AGENT_CONTROL_MODE::SPEED_PROFILE ){
		
		                pAgent[objectID]->memory.speedProfileCount = 0;
		                pAgent[objectID]->memory.profileTime  = controlInfo->profileAxis;
		                pAgent[objectID]->memory.profileSpeed = controlInfo->speedProfile;
		
		                pAgent[objectID]->vehicle.SetInitialSpeed( pAgent[objectID]->memory.profileSpeed[0] );
		                pAgent[objectID]->memory.profileTime[0] = 0.0f;
		
		                pAgent[objectID]->state.V = pAgent[objectID]->memory.profileSpeed[0];
		            }
		            else if( pAgent[objectID]->memory.controlMode == AGENT_CONTROL_MODE::STOP_AT ){
		
		                pAgent[objectID]->memory.targetSpeedByScenario = controlInfo->targetSpeed;
		
		                pAgent[objectID]->memory.targetStopAtXByScenario = controlInfo->stopAtX;
		                pAgent[objectID]->memory.targetStopAtYByScenario = controlInfo->stopAtY;
		
		                pAgent[objectID]->memory.actualStopOnPathID = -1;
		
		                pAgent[objectID]->memory.doStopControl = true;
		            }
		
		            pAgent[objectID]->memory.targetLateralShiftByScenario = controlInfo->targetLateralOffset;
            }
            scenario[currentScenarioID]->scenarioItems[i]->status = 1;

            pAgent[objectID]->agentStatus = 1;
            pAgent[objectID]->isScenarioObject = true;

            pAgent[objectID]->isSInterfaceObject = false;
            pAgent[objectID]->isBehaviorEmbeded  = false;
            pAgent[objectID]->justWarped = false;


            if( DSMode == true ){

                if( pAgent[objectID]->vehicle.steerFiltered4CG == NULL ){
                    pAgent[objectID]->vehicle.steerFiltered4CG = new LowPassFilter(1, simTime.dt, 6.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.steerFiltered4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.axFiltered4CG == NULL ){
                    pAgent[objectID]->vehicle.axFiltered4CG = new LowPassFilter(1, simTime.dt, 6.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.axFiltered4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.ayFiltered4CG == NULL ){
                    pAgent[objectID]->vehicle.ayFiltered4CG = new LowPassFilter(1, simTime.dt, 6.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.axFiltered4CG->Reset();
                }


                if( pAgent[objectID]->vehicle.suspentionFL4CG == NULL ){
                    pAgent[objectID]->vehicle.suspentionFL4CG = new LowPassFilter(2, simTime.dt, 10.0 , 0.6);
                }
                else{
                    pAgent[objectID]->vehicle.suspentionFL4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.suspentionFR4CG == NULL ){
                    pAgent[objectID]->vehicle.suspentionFR4CG = new LowPassFilter(2, simTime.dt, 10.0 , 0.6);
                }
                else{
                    pAgent[objectID]->vehicle.suspentionFR4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.suspentionRL4CG == NULL ){
                    pAgent[objectID]->vehicle.suspentionRL4CG = new LowPassFilter(2, simTime.dt, 10.0 , 0.6);
                }
                else{
                    pAgent[objectID]->vehicle.suspentionRL4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.suspentionRR4CG == NULL ){
                    pAgent[objectID]->vehicle.suspentionRR4CG = new LowPassFilter(2, simTime.dt, 10.0 , 0.6);
                }
                else{
                    pAgent[objectID]->vehicle.suspentionRR4CG->Reset();
                }


                if( pAgent[objectID]->vehicle.tireFL4CG == NULL ){
//                    pAgent[objectID]->vehicle.tireFL4CG = new LowPassFilter(2, simTime.dt, 20.0 , 1.0);
                    pAgent[objectID]->vehicle.tireFL4CG = new LowPassFilter(1, simTime.dt, 5.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.tireFL4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.tireFR4CG == NULL ){
//                    pAgent[objectID]->vehicle.tireFR4CG = new LowPassFilter(2, simTime.dt, 20.0 , 1.0);
                    pAgent[objectID]->vehicle.tireFR4CG = new LowPassFilter(1, simTime.dt, 5.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.tireFR4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.tireRL4CG == NULL ){
//                    pAgent[objectID]->vehicle.tireRL4CG = new LowPassFilter(2, simTime.dt, 20.0 , 1.0);
                    pAgent[objectID]->vehicle.tireRL4CG = new LowPassFilter(1, simTime.dt, 5.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.tireRL4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.tireRR4CG == NULL ){
//                    pAgent[objectID]->vehicle.tireRR4CG = new LowPassFilter(2, simTime.dt, 20.0 , 1.0);
                    pAgent[objectID]->vehicle.tireRR4CG = new LowPassFilter(1, simTime.dt, 5.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.tireRR4CG->Reset();
                }
            }
        }
        else if( pAgent[objectID]->agentKind >= 100 ){

            ScenarioObjectControlInfo* controlInfo = scenario[currentScenarioID]->scenarioItems[i]->controlInfo;

            // Set initial state
            float xi  = scenario[currentScenarioID]->scenarioItems[i]->initialState[0];
            float yi  = scenario[currentScenarioID]->scenarioItems[i]->initialState[1];
            float zi  = scenario[currentScenarioID]->scenarioItems[i]->initialState[2];
            float YAi = scenario[currentScenarioID]->scenarioItems[i]->initialState[3];
            float Vi  = scenario[currentScenarioID]->scenarioItems[i]->initialState[4];

            pAgent[objectID]->state.V = Vi;
            pAgent[objectID]->state.x = xi;
            pAgent[objectID]->state.y = yi;
            pAgent[objectID]->state.z = zi;
            pAgent[objectID]->state.yaw = YAi;
            pAgent[objectID]->state.cosYaw = cos( YAi );
            pAgent[objectID]->state.sinYaw = sin( YAi );


            // Set route information
            for(int j=0;j<scenario[currentScenarioID]->scenarioItems[i]->controlInfo->pedestPathRoute.size();++j){
                int ppath = scenario[currentScenarioID]->scenarioItems[i]->controlInfo->pedestPathRoute[j];
                pAgent[objectID]->memory.targetPathList.prepend( ppath );
            }

            int overEdge = 0;
            float dist = 0.0;
            int nearPedestPathID = pRoad->GetNearestPedestPath(xi,yi,dist,overEdge,objectID);
            for(int j=0;j<pAgent[objectID]->memory.targetPathList.size();++j){
                if( pAgent[objectID]->memory.targetPathList[j] == nearPedestPathID ){
                    pAgent[objectID]->memory.currentTargetPath = nearPedestPathID;
                    pAgent[objectID]->memory.currentTargetPathIndexInList = j;

                    if( j > 0 ){
                        int nextPPath = pAgent[objectID]->memory.targetPathList[j-1];
                        int linkID = pRoad->GetDirectionByPedestPathLink( pAgent[objectID]->memory.currentTargetPath, nextPPath);
                        pAgent[objectID]->memory.currentTargetDirectionPedestPath = linkID;
                        if( linkID < 1 ){
                            qDebug() << "[Warning]----------------------------------";
                            qDebug() << " Scenario Pedestrian ID = " << objectID << " cannot determin direction to move on pedest-path.";
                            qDebug() << "   Assigned Path List : ";
                            for(int j=0;j<pAgent[objectID]->memory.targetPathList.size();++j){
                                qDebug() << "           PedestPath " << pAgent[objectID]->memory.targetPathList[j];
                            }
                            qDebug() << "   Current Target Pedest Path = " << pAgent[objectID]->memory.currentTargetPath;
                            qDebug() << "   Next Target Pedest Path = " << nextPPath;
                            qDebug() << "   This pedestrian will not move and disappear.";
                        }
                    }
                    else{

                        int idx = pRoad->pedestPathID2Index.indexOf( pAgent[objectID]->memory.currentTargetPath );
                        if( idx >= 0 ){
                            float ip = pAgent[objectID]->state.cosYaw * pRoad->pedestPaths[idx]->eX + pAgent[objectID]->state.sinYaw * pRoad->pedestPaths[idx]->eY;
                            if( ip >= 0.0 ){
                                pAgent[objectID]->memory.currentTargetDirectionPedestPath = 1;
                            }
                            else{
                                pAgent[objectID]->memory.currentTargetDirectionPedestPath = 2;
                            }
                        }
                        else{
                            pAgent[objectID]->memory.currentTargetDirectionPedestPath = 0;

                            qDebug() << "[Warning]----------------------------------";
                            qDebug() << " Scenario Pedestrian ID = " << objectID << " cannot determin direction to move on pedest-path.";
                            qDebug() << "   Assigned Path List : ";
                            for(int j=0;j<pAgent[objectID]->memory.targetPathList.size();++j){
                                qDebug() << "           PedestPath " << pAgent[objectID]->memory.targetPathList[j];
                            }
                            qDebug() << "   Current Target Pedest Path = " << pAgent[objectID]->memory.currentTargetPath;
                            qDebug() << "   This pedestrian will not move and disappear.";
                        }

                    }
                }
            }

            // Set control information
            pAgent[objectID]->memory.controlMode = controlInfo->mode;

            pAgent[objectID]->memory.targetSpeedByScenario = controlInfo->targetSpeed;
            pAgent[objectID]->memory.targetSpeed           = controlInfo->targetSpeed;

            if( pAgent[objectID]->memory.controlMode == AGENT_CONTROL_MODE::SPEED_PROFILE ){

                pAgent[objectID]->memory.speedProfileCount = 0;
                pAgent[objectID]->memory.profileTime  = controlInfo->profileAxis;
                pAgent[objectID]->memory.profileSpeed = controlInfo->speedProfile;
                pAgent[objectID]->memory.profileTime[0] = 0.0f;
                pAgent[objectID]->state.V = pAgent[objectID]->memory.profileSpeed[0];
            }


            if( DSMode == true ){

                // filter for yaw motion
                if( pAgent[objectID]->vehicle.axFiltered4CG == NULL ){
                    pAgent[objectID]->vehicle.axFiltered4CG = new LowPassFilter(1, simTime.dt, 6.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.axFiltered4CG->Reset();
                }

                if( pAgent[objectID]->vehicle.ayFiltered4CG == NULL ){
                    pAgent[objectID]->vehicle.ayFiltered4CG = new LowPassFilter(1, simTime.dt, 6.0 , 0.0);
                }
                else{
                    pAgent[objectID]->vehicle.axFiltered4CG->Reset();
                }

            }

            // Set status
            scenario[currentScenarioID]->scenarioItems[i]->status = 1;

            pAgent[objectID]->agentStatus = 1;
            pAgent[objectID]->isScenarioObject = true;
        }


        qDebug() << "Agent ID = " << objectID << " generated by Scenario.";
    }

}



void SimulationManager::DisappearAgents(Agent **pAgent, int maxAgent)
{
    for(int i=0;i<maxAgent;++i){

        if( pAgent[i]->agentStatus == 2 ){

            //
            // Implement some logic for counting somthing if necessary
            //


            pAgent[i]->agentStatus = 0;

            if( pAgent[i]->isScenarioObject == true  ){

                int nItem = scenario[currentScenarioID]->scenarioItems.size();
                for(int j=0;j<nItem;++j){

                    int objectID = scenario[currentScenarioID]->scenarioItems[j]->objectID;
                    if( objectID == pAgent[i]->ID ){
                        // Reset trigger flag
                        ScenarioTriggers *trig = scenario[currentScenarioID]->scenarioItems[j]->appearTriggers;
                        if( trig->mode == TRIGGER_TYPE::BY_FUNCTION_EXTENDER ){
                            trig->byExternalTriggerFlag = 0;
                        }
                        break;
                    }
                }
            }

            pAgent[i]->isScenarioObject = false;
            pAgent[i]->skipSetControlInfo = false;

            qDebug() << "Agent ID = " << i << " disposed.";
        }
    }
}


void SimulationManager::SetAppearFlagByFE(int id)
{
    if( scenario.size() == 0 ){
        return;
    }

    int nItem = scenario[currentScenarioID]->scenarioItems.size();
    for(int i=0;i<nItem;++i){

        int objectID = scenario[currentScenarioID]->scenarioItems[i]->objectID;
        if( objectID != id ){
            continue;
        }

        // Check trigger
        ScenarioTriggers *trig = scenario[currentScenarioID]->scenarioItems[i]->appearTriggers;
        if( trig->mode == TRIGGER_TYPE::BY_FUNCTION_EXTENDER ){
            trig->byExternalTriggerFlag = 1;
            break;
        }
    }
}