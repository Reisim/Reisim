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


void SystemThread::DSMove(float x, float y)
{
    // This function is valid only for DS Mode
    if( DSMode == false ){
        return;
    }

    qDebug() << "[SystemThread::DSMove] DSMoveTarget = " << DSMoveTarget;

    float deviation,xt,yt,xd,yd,s;
    int pathId = road->GetNearestPath( x, y, 0.0, deviation, xt, yt, xd, yd, s, true );

    float angle = 0.0;
    if( pathId >= 0 ){
        angle = atan2( (-1.0) * yd, xd ) * 57.3;
    }

    qDebug() << "pathId = " << pathId << " angle = " << angle;

    y *= (-1.0);  // Coordinate Transform from Re:sim to UE4


    if( udpThread != NULL ){

        udpThread->SendDSMoveCommand( DSMoveTarget, x, y, angle );
    }
}


void SystemThread::ChangeOptionalImageParams(QList<float> p)
{
    if( p.size() < 2 ){
        return;
    }

    int id = (int)(p[0]);
    if( p[1] > 0.5 && p.size() >= 7 ){
        float x = p[2];
        float y = p[3];
        float z = p[4];
        float rot = p[5];
        float s = p[6];

        emit ShowOptionalImage(id,x,y,z,rot,s);
    }
    else{
        emit HideOptionalImage(id);
    }
}


void SystemThread::SetEventTriggerByFuncExtend(int eventType,int id,int idx,int option)
{
    simManage->SetScenarioTriggeredByFuncExtend(eventType, id, idx,option);
}


void SystemThread::SetScenarioVehicleInitStates(QList<int> aIDs,
                                                QList<float> Xs,
                                                QList<float> Ys,
                                                QList<float> Zs,
                                                QList<float> Psis,
                                                QList<float> Vs)
{
    qDebug() << "[SystemThread::SetScenarioVehicleInitStates] aIDs = " << aIDs;

    int sID = simManage->GetCurrentScenarioID();
    for(int i=0;i<aIDs.size();++i){
        if( aIDs[i] < 0 || aIDs[i] >= maxAgent ){
            continue;
        }
        if( agent[aIDs[i]]->agentStatus == 1 ){
            continue;
        }
        simManage->SetScenarioVehicleInitState( sID, aIDs[i], Xs[i], Ys[i], Zs[i], Psis[i], Vs[i] );
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
        trafficSignal[tsID]->CheckDisplayInfoUpdate( simTimeFVal, simManage->GetCalculationInterval() );
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
    noClearByWarp = 1;

    qDebug() << "[WarpVehicle] xTo = " << xTo << " yTo = " << yTo << " dirTo = " << dirTo;

    agent[targetVID]->state.x = xTo;
    agent[targetVID]->state.y = yTo;
    agent[targetVID]->vehicle.state.X = xTo;
    agent[targetVID]->vehicle.state.Y = yTo;

    dirTo *= 0.017452;
    agent[targetVID]->vehicle.state.yawAngle = dirTo;
    agent[targetVID]->state.yaw = agent[targetVID]->vehicle.state.yawAngle;
    agent[targetVID]->state.cosYaw = cos( agent[targetVID]->state.yaw );
    agent[targetVID]->state.sinYaw = sin( agent[targetVID]->state.yaw );

    agent[targetVID]->justWarped = true;

    if( agent[targetVID]->vehicle.yawFiltered4CG != NULL ){
        agent[targetVID]->vehicle.yawFiltered4CG->SetInitialValue( agent[targetVID]->state.yaw );
    }

    if( agent[targetVID]->agentKind < 100 ){

        agent[targetVID]->vehicle.state.XRear = agent[targetVID]->vehicle.state.X;
        agent[targetVID]->vehicle.state.XRear -= agent[targetVID]->state.cosYaw * agent[targetVID]->vehicle.param.Lr;
        agent[targetVID]->vehicle.state.YRear = agent[targetVID]->vehicle.state.Y;
        agent[targetVID]->vehicle.state.YRear -= agent[targetVID]->state.sinYaw * agent[targetVID]->vehicle.param.Lr;

        float deviation = 0;
        float xt = 0.0;
        float yt = 0.0;
        float xd = 0.0;
        float yd = 0.0;
        float s = 0.0;

        int pathId = road->GetNearestPath( xTo, yTo, dirTo, deviation, xt, yt, xd, yd, s );
        if( pathId >= 0 ){
            agent[targetVID]->memory.currentTargetPath = pathId;
        }

        agent[targetVID]->CheckPathList( road );
    }
    else{

        agent[targetVID]->memory.currentTargetPath = -1;
        if( agent[targetVID]->isScenarioObject == true ){

            for(int k=0;k<road->pedestPaths.size();++k){
                if( road->pedestPaths[k]->scenarioObjectID == agent[targetVID]->ID ){
                    agent[targetVID]->memory.currentTargetPath = road->pedestPaths[k]->id;
                    break;
                }
            }

            agent[targetVID]->memory.currentTargetPathIndexInList = 0;
            int overEdge = 0;
            float dist = 0.0;
            int nearPPSectIndex = road->GetNearestPedestPathSectionIndex(xTo,yTo,dist,overEdge,targetVID);
            if( nearPPSectIndex >= 0 ){
                agent[targetVID]->memory.currentTargetPathIndexInList = nearPPSectIndex;
            }
        }
        else{

            int onPPath = -1;
            int ppathSect = -1;
            float deviation = 0.0;
            float distInSect = 0.0;
            bool ret = road->GetNearestPedestPath(xTo,yTo,dirTo,deviation,onPPath,ppathSect,distInSect);
            if( ret == true ){
                agent[targetVID]->memory.currentTargetPathIndexInList = ppathSect;
                agent[targetVID]->memory.currentTargetPath = onPPath;
            }
            else{
                qDebug() << "[WarpVehicle] can not determine nearest pedestrian path for agent " << targetVID;
            }
        }

        if( agent[targetVID]->memory.currentTargetPath >= 0 ){
            agent[targetVID]->memory.targetPathList.clear();
            agent[targetVID]->memory.targetPathList.append( agent[targetVID]->memory.currentTargetPath );
        }
    }
}


void SystemThread::WarpVehicleAdjustPosToLane(int targetVID, float xTo, float yTo, float dirTo)
{
    noClearByWarp = 1;

    qDebug() << "[WarpVehicleAdjustPosToLane] xTo = " << xTo << " yTo = " << yTo << " dirTo = " << dirTo;

    dirTo *= 0.017452;

    if( agent[targetVID]->agentKind < 100 ){

        float deviation = 0;
        float xt = 0.0;
        float yt = 0.0;
        float xd = 0.0;
        float yd = 0.0;
        float s = 0.0;

        int pathId = road->GetNearestPath( xTo, yTo, dirTo, deviation, xt, yt, xd, yd, s );
        if( pathId >= 0 ){

            agent[targetVID]->memory.currentTargetPath = pathId;

            xTo = xt;
            yTo = yt;
            dirTo = atan2( yd, xd );

        }

        agent[targetVID]->state.x = xTo;
        agent[targetVID]->state.y = yTo;
        agent[targetVID]->vehicle.state.X = xTo;
        agent[targetVID]->vehicle.state.Y = yTo;

        agent[targetVID]->vehicle.state.yawAngle = dirTo;
        agent[targetVID]->state.yaw = agent[targetVID]->vehicle.state.yawAngle;
        agent[targetVID]->state.cosYaw = cos( agent[targetVID]->state.yaw );
        agent[targetVID]->state.sinYaw = sin( agent[targetVID]->state.yaw );

        agent[targetVID]->vehicle.state.XRear = agent[targetVID]->vehicle.state.X;
        agent[targetVID]->vehicle.state.XRear -= agent[targetVID]->state.cosYaw * agent[targetVID]->vehicle.param.Lr;
        agent[targetVID]->vehicle.state.YRear = agent[targetVID]->vehicle.state.Y;
        agent[targetVID]->vehicle.state.YRear -= agent[targetVID]->state.sinYaw * agent[targetVID]->vehicle.param.Lr;

        agent[targetVID]->justWarped = true;

        if( agent[targetVID]->vehicle.yawFiltered4CG != NULL ){
            agent[targetVID]->vehicle.yawFiltered4CG->SetInitialValue( agent[targetVID]->state.yaw );
        }

        agent[targetVID]->CheckPathList( road );
    }
    else{

        int onPPath = -1;
        int ppathSect = -1;
        float deviation = 0.0;
        float distInSect = 0.0;
        bool ret = road->GetNearestPedestPath(xTo,yTo,dirTo,deviation,onPPath,ppathSect,distInSect);

        if( ret == true ){

            agent[targetVID]->memory.currentTargetPathIndexInList = ppathSect;
            agent[targetVID]->memory.currentTargetPath = onPPath;

            int pidx = road->pedestPathID2Index.indexOf( onPPath );
            xTo = road->pedestPaths[pidx]->shape[ppathSect]->pos.x() + road->pedestPaths[pidx]->shape[ppathSect]->cosA * distInSect;
            yTo = road->pedestPaths[pidx]->shape[ppathSect]->pos.y() + road->pedestPaths[pidx]->shape[ppathSect]->sinA * distInSect;
            dirTo = atan2( road->pedestPaths[pidx]->shape[ppathSect]->sinA, road->pedestPaths[pidx]->shape[ppathSect]->cosA );

            agent[targetVID]->state.x = xTo;
            agent[targetVID]->state.y = yTo;
            agent[targetVID]->vehicle.state.X = xTo;
            agent[targetVID]->vehicle.state.Y = yTo;

            agent[targetVID]->vehicle.state.yawAngle = dirTo;
            agent[targetVID]->state.yaw = agent[targetVID]->vehicle.state.yawAngle;
            agent[targetVID]->state.cosYaw = cos( agent[targetVID]->state.yaw );
            agent[targetVID]->state.sinYaw = sin( agent[targetVID]->state.yaw );

            agent[targetVID]->justWarped = true;

            agent[targetVID]->memory.targetPathList.clear();
            agent[targetVID]->memory.targetPathList.append( agent[targetVID]->memory.currentTargetPath );

            if( agent[targetVID]->vehicle.yawFiltered4CG != NULL ){
                agent[targetVID]->vehicle.yawFiltered4CG->SetInitialValue( agent[targetVID]->state.yaw );
            }
        }
        else{
            qDebug() << "[WarpVehicleAdjustPosToLane] can not determine nearest pedestrian path for agent " << targetVID;
        }
    }
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
        qDebug() << "Received Appear Agent: agentID = " << agentID;
        simManage->SetAppearFlagByFE( agentID );
        simManage->AppearAgents(agent,maxAgent,road);
    }
}

void SystemThread::ChangeReferenceSpeed(int agentID, float refSpeed)
{
    if( agentID >= 0 && agentID < maxAgent ){
        agent[agentID]->memory.targetSpeedByScenario = refSpeed;
        //qDebug() << "agentID=" << agentID << " targetSpeedByScenario=" << refSpeed;
        agent[agentID]->memory.setTargetSpeedByScenarioFlag = true;

        if( agent[agentID]->refSpeedMode == 4 ){
            agent[agentID]->SetTargetSpeedIndividual( agent[agentID]->memory.targetSpeedByScenario );
        }
    }
}


void SystemThread::ForceChangeSpeed(int agentID, float speed)
{
    if( agentID >= 0 && agentID < maxAgent ){
        agent[agentID]->state.V = speed;
        agent[agentID]->vehicle.state.vx = speed;
    }
    else if( agentID == -1 ){
        int csID = simManage->GetCurrentScenarioID();
        int nE = simManage->GetNumberScenarioEvent( csID );
        for(int i=0;i<nE;++i){
            int objectID = simManage->GetScenarioEventObjectID(csID,i);
            if( objectID >= 0 && objectID < maxAgent ){
                agent[objectID]->state.V = speed;
                agent[objectID]->vehicle.state.vx = speed;
            }
        }
    }
}


void SystemThread::ChangeSpeedProfile(int agentID, QList<float> td, QList<float> vd )
{
    if( agentID >= 0 && agentID < maxAgent ){
        agent[agentID]->memory.profileTime.clear();
        agent[agentID]->memory.profileSpeed.clear();

        for(int i=0;i<td.size();++i){
            agent[agentID]->memory.profileTime.append( td[i] );
        }

        for(int i=0;i<vd.size();++i){
            agent[agentID]->memory.profileSpeed.append( vd[i] );
        }

        agent[agentID]->memory.protectProfileData = true;
    }
}


void SystemThread::SetTargetSpeedMode(int spmode, int agentID)
{
    if( agentID >= 0 && agentID < maxAgent ){
        agent[agentID]->refSpeedMode = spmode;
        qDebug() << "Set agent " << agentID << " refSpeedMode = " << agent[agentID]->refSpeedMode;

        if( agent[agentID]->refSpeedMode == 0 ){
            int pIdx = road->pathId2Index.indexOf( agent[agentID]->memory.currentTargetPath );
            if( pIdx >= 0 && pIdx < road->paths.size() ){
                float vTarget = road->paths[pIdx]->speed85pt;
                agent[agentID]->SetTargetSpeedIndividual( vTarget );
            }
        }
    }
    else{
        for(int i=0;i<maxAgent;++i){
            if( agent[i]->refSpeedMode != 4 ){
                agent[i]->refSpeedMode = spmode;
            }
        }
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


void SystemThread::SetBrakeLampOverride(int agentID,int bState)
{
    if( agentID >= 0 && agentID < maxAgent ){
        agent[agentID]->brakeLampOverride = bState;
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


void SystemThread::SetAgentGenerationNotAllowFlagForNodes(QList<int> nodeList, bool flag )
{
    for(int i=0;i<nodeList.size();++i){
        for(int j=0;j<road->odRoute.size();++j){
            if( road->odRoute[j]->originNode == nodeList[i] ){
                road->odRoute[j]->allowAgentGeneration = flag;
            }
        }
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

        float tdev,txt,tyt,txd,tyd,ts;
        float xi = agent[toAID]->state.x;
        float yi = agent[toAID]->state.y;
        float zi = agent[toAID]->state.z_path;
        float YAi = agent[toAID]->state.yaw;
        int currentPath = road->GetNearestPathFromList( agent[toAID]->memory.targetPathList,
                                                        xi, yi, zi, YAi,
                                                        tdev,txt,tyt,txd,tyd,ts );
        if( currentPath < 0 ){
            qDebug() << "[Warning]----------------------------------";
            qDebug() << " Scenario Vehicle ID = " << toAID << " cannot determin nearest path from assigned list.";
            qDebug() << "   Assigned Path List : ";
            for(int j=0;j<agent[toAID]->memory.targetPathList.size();++j){
                qDebug() << "           Path " << agent[toAID]->memory.targetPathList[j];

                int pid = agent[toAID]->memory.targetPathList[j];
                int pdx = road->pathId2Index.indexOf(pid);
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


void SystemThread::RequestLaneChange(int aID, int dir, int mode)
{
    if( aID >= 0 && aID < maxAgent ){
        agent[aID]->ProcessLaneChangeRequest(road, dir, mode, -1.0);
    }
}


void SystemThread::RequestAssignedLaneChange(int aID, int dir, int mode, float moveLatDist)
{
    if( aID >= 0 && aID < maxAgent ){
        agent[aID]->ProcessLaneChangeRequest(road, dir, mode, moveLatDist);
    }
}


void SystemThread::OverwriteAgentParameter(int aID,int paramID,float val)
{
    if( aID >= 0 && aID < maxAgent ){
        switch(paramID){
        case 0: agent[aID]->param.maxLateralSpeedForLaneChange = val; break;
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


void SystemThread::ChangeControlModeHeadwayControl(int aID, float V, float dist,float allowDev,float time,int targetID, bool disableSpeedAdjustForCurve)
{
    if( aID >= 0 && aID < maxAgent ){

        agent[aID]->memory.controlMode = AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY;
        agent[aID]->memory.targetSpeedByScenario = V;
        agent[aID]->memory.setTargetSpeedByScenarioFlag = true;
        agent[aID]->memory.targetHeadwayDistanceByScenario = dist;
        agent[aID]->memory.allowableHeadwayDistDeviation = allowDev;
        agent[aID]->memory.targetHeadwayTimeByScenario = time;
        agent[aID]->memory.precedingVehicleIDByScenario = targetID;
        agent[aID]->skipSetControlInfo = true;
        agent[aID]->memory.disableSpeedAdjustForCurveByScenario = disableSpeedAdjustForCurve;

        qDebug() << "Change control mode to Headway Control : aID=" << aID << " V=" << V*3.6 << " D=" << dist << " T=" << time << " target=" << targetID;
    }
    else if( aID == -1 ){

        int csID = simManage->GetCurrentScenarioID();
        int nE = simManage->GetNumberScenarioEvent( csID );
        for(int i=0;i<nE;++i){
            int objectID = simManage->GetScenarioEventObjectID(csID,i);
            if( objectID >= 0 && objectID < maxAgent ){
                agent[objectID]->memory.controlMode = AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY;
                agent[objectID]->memory.targetSpeedByScenario = V;
                agent[objectID]->memory.setTargetSpeedByScenarioFlag = true;
                agent[objectID]->memory.targetHeadwayDistanceByScenario = dist;
                agent[objectID]->memory.allowableHeadwayDistDeviation = allowDev;
                agent[objectID]->memory.targetHeadwayTimeByScenario = time;
                agent[objectID]->memory.precedingVehicleIDByScenario = targetID;
                agent[objectID]->skipSetControlInfo = true;
                agent[objectID]->memory.disableSpeedAdjustForCurveByScenario = disableSpeedAdjustForCurve;
            }
        }
        qDebug() << "Change control mode to Headway Control of All Scenario Vehicle; V=" << V*3.6 << " D=" << dist << " T=" << time << " target=" << targetID;
    }
}


void SystemThread::ChangeControlModeAgent(int aID)
{
    if( aID >= 0 && aID < maxAgent ){
        agent[aID]->memory.controlMode = AGENT_CONTROL_MODE::AGENT_LOGIC;
        agent[aID]->skipSetControlInfo = true;
        agent[aID]->memory.setTargetSpeedByScenarioFlag = false;
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


void SystemThread::DirectAssignAcceleration(int aID,float a_com)
{
    if( aID >= 0 && aID < maxAgent ){
        agent[aID]->memory.controlMode = AGENT_CONTROL_MODE::DIRECT_ACCEL_ASSIGN;
        agent[aID]->memory.accel = 0.0;
        agent[aID]->memory.brake = 0.0;
        if( a_com >= 0.0f ){
            agent[aID]->memory.accel = a_com;
        }
        else{
            agent[aID]->memory.brake = -a_com;
        }
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
    agent[id]->state.steer = data[7];    // The unit of steer is [deg]

    agent[id]->agentStatus = 1;

    //qDebug() << "EmbedAgentBehavior : id = " << id;


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


void SystemThread::ChangeVelocityControlParameters(float refV,float vDevP,float vDevM,float accel,float decel,int selMode,QList<int> aIDs)
{
    qDebug() << "ChangeVelocityControlParameters:";

    if( selMode == 0 ){

        qDebug() << "  select = 0";

        for(int i=0;i<aIDs.size();++i){
            int id = aIDs[i];
            if( id >= 0 && id < maxAgent ){
                agent[id]->param.refVforDev     = refV;
                agent[id]->param.vDevAllowPlus  = vDevP;
                agent[id]->param.vDevAllowMinus = vDevM;
                agent[id]->param.accelAtVDev    = accel;
                agent[id]->param.decelAtVDev    = decel;
                agent[id]->param.deadZoneSpeedControl = 0.0;
            }
        }
    }
    else if( selMode == 1 ){

        qDebug() << "  select = 1";

        for(int i=0;i<aIDs.size();++i){
            int id = aIDs[i];
            if( id >= 0 && id < maxAgent ){
                if( agent[id]->refSpeedMode == 0 ){
                    agent[id]->param.refVforDev     = refV;
                    agent[id]->param.vDevAllowPlus  = vDevP;
                    agent[id]->param.vDevAllowMinus = vDevM;
                    agent[id]->param.accelAtVDev    = accel;
                    agent[id]->param.decelAtVDev    = decel;
                    agent[id]->param.deadZoneSpeedControl = 0.0;
                }
            }
        }
    }
    else if( selMode == 2 ){

        qDebug() << "  select = 2";

        for(int i=0;i<aIDs.size();++i){
            int id = aIDs[i];
            if( id >= 0 && id < maxAgent ){
                if( agent[id]->refSpeedMode == 1 ){
                    agent[id]->param.refVforDev     = refV;
                    agent[id]->param.vDevAllowPlus  = vDevP;
                    agent[id]->param.vDevAllowMinus = vDevM;
                    agent[id]->param.accelAtVDev    = accel;
                    agent[id]->param.decelAtVDev    = decel;
                    agent[id]->param.deadZoneSpeedControl = 0.0;
                }
            }
        }
    }
    else if( selMode == 3 ){

        qDebug() << "  select = 3";
        qDebug() << "  maxAgent = " << maxAgent;

        for(int id=0;id<maxAgent;++id){
            agent[id]->param.refVforDev     = refV;
            agent[id]->param.vDevAllowPlus  = vDevP;
            agent[id]->param.vDevAllowMinus = vDevM;
            agent[id]->param.accelAtVDev    = accel;
            agent[id]->param.decelAtVDev    = decel;
            agent[id]->param.deadZoneSpeedControl = 0.0;
        }
    }
}

void SystemThread::ChangeLaneAssignedVelocityControlParameters(float refV,float vDevP,float vDevM,float accel,float decel,QList<int> laneIDs)
{
    qDebug() << "ChangeLaneAssignedVelocityControlParameters:";

    for(int i=0;i<laneIDs.size();++i){
        int lIdx = road->pathId2Index.indexOf( laneIDs[i] );
        if( lIdx >= 0 ){
            qDebug() << " Set Speed Variation Parameter to Lane " << laneIDs[i];

            road->paths[lIdx]->setSpeedVariationParam = true;
            road->paths[lIdx]->refVforDev = refV;
            road->paths[lIdx]->vDevP = vDevP;
            road->paths[lIdx]->vDevM = vDevM;
            road->paths[lIdx]->accelAtDev = accel;
            road->paths[lIdx]->decelAtDev = decel;
        }
    }
}


void SystemThread::ChangeRoadLaneSpeedLimit(QList<int> laneID, QList<float> vLimit)
{
    if( laneID.size() == 0 || laneID.size() != vLimit.size() ){
        return;
    }

    for(int i=0;i<laneID.size();++i){
        int lIdx = road->pathId2Index.indexOf( laneID[i] );
        if( lIdx >= 0 ){
            road->paths[lIdx]->speed85pt = vLimit[i];
            road->paths[lIdx]->speedInfo = vLimit[i];
//            qDebug() << "Speed Limit Changed: path = " << laneID[i] << " Vlim = " << vLimit[i] << " [km/h]";
        }
    }

    for(int i=0;i<maxAgent;++i){
        if( agent[i]->agentStatus != 1 ){
            continue;
        }
        if( agent[i]->isScenarioObject == true ){
            continue;
        }
        if( agent[i]->isSInterfaceObject == true ){
            continue;
        }
        if( agent[i]->refSpeedMode == 4 ){
            continue;
        }
        int currentPath = agent[i]->memory.currentTargetPath;
        if( laneID.indexOf( currentPath ) >= 0 ){
            int pIdx = road->pathId2Index.indexOf( currentPath );
            if( pIdx >= 0 ){
                if( agent[i]->isScenarioObject == true ){
                    if( road->paths[pIdx]->speed85pt == road->paths[pIdx]->speedInfo ){
                        agent[i]->refSpeedMode = 1;
                    }
                    else{
                        agent[i]->refSpeedMode = 0;
                    }
                }
                agent[i]->SetTargetSpeedIndividual( road->paths[pIdx]->speed85pt );
            }
        }
    }
}


void SystemThread::ChangeMoveSpeedPedestrian(QList<float> vPedest)
{
    if( vPedest.size() >= 3 ){
        for(int i=0;i<3;++i){
            simManage->meanSpeedPedestrian[i] = vPedest.at(i);
        }

        for(int i=0;i<maxAgent;++i){
            if( agent[i]->agentStatus != 1 ){
                continue;
            }
            if( agent[i]->objTypeForUE4 == 1 ){
                if( agent[i]->objNoForUE4 == 3 ){   // Child
                    agent[i]->attri.age = 0;
                    agent[i]->memory.targetSpeed = simManage->GetNormalDist( simManage->meanSpeedPedestrian[0], 0.107 );
                }
                else if( agent[i]->objNoForUE4 == 4 ){   // Aged
                    agent[i]->attri.age = 2;
                    agent[i]->memory.targetSpeed = simManage->GetNormalDist( simManage->meanSpeedPedestrian[2], 0.104 );
                }
                else{
                    agent[i]->attri.age = 1;
                    agent[i]->memory.targetSpeed = simManage->GetNormalDist( simManage->meanSpeedPedestrian[1], 0.093 );
                }
            }
        }
    }
}

void SystemThread::AppearStoppingVehicle(int aID,float x,float y,float psi)
{
    int csID = simManage->GetCurrentScenarioID();
    struct ScenarioEvents *se = simManage->GetScenarioEventAppearVehicle( csID, aID );
    if( !se ){
        qDebug() << "[AppearStoppingVehicle] Can not find corresponding scenario event.";
        return;
    }

    float tdev,txt,tyt,txd,tyd,ts;
    int nearPath = road->GetNearestPath( x, y, psi, tdev, txt, tyt, txd, tyd, ts);
    if( nearPath < 0 ){
        qDebug() << "[AppearStoppingVehicle] Can not find nearest path.";
        return;
    }

    int nPIdx = road->pathId2Index.indexOf( nearPath );
    int toNode = road->paths[nPIdx]->connectingNode;
    int toNdIn = road->paths[nPIdx]->connectingNodeInDir;

    int fmNode = -1;
    int fmNdOt = -1;
    for(int i=0;i<road->nodes.size();++i){
        for(int j=0;j<road->nodes[i]->nodeConnectInfo.size();++j){
            if( road->nodes[i]->nodeConnectInfo[j]->connectedNode == toNode &&
                    road->nodes[i]->nodeConnectInfo[j]->inDirectionID == toNdIn ){
                fmNode = road->nodes[i]->id;
                fmNdOt = road->nodes[i]->nodeConnectInfo[j]->outDirectionID;
                break;
            }
        }
    }

    if( fmNode < 0 || fmNdOt < 0 ){
        qDebug() << "[AppearStoppingVehicle] Can not determine from-Node or from-Node-out-Direction";
        return;
    }

    // Try to find appropriate route from existing route
    QList<int> rIdxL;
    for(int i=0;i<road->odRoute.size();++i){
        for(int j=1;j<road->odRoute[i]->routeToDestination.size();++j){
            if( road->odRoute[i]->routeToDestination[j-1]->node == fmNode &&
                    road->odRoute[i]->routeToDestination[j-1]->outDir == fmNdOt &&
                    road->odRoute[i]->routeToDestination[j]->node == toNode &&
                    road->odRoute[i]->routeToDestination[j]->inDir == toNdIn ){
                rIdxL.append( i );
                break;
            }
        }
    }

    if( rIdxL.size() > 0 ){
        qDebug() << "[AppearStoppingVehicle] Found existing route.";

        int rIdx = rIdxL[0];

        struct ODRouteData* odr = new struct ODRouteData;

        odr->originNode = road->odRoute[rIdx]->originNode;
        odr->destinationNode = road->odRoute[rIdx]->destinationNode;

        for(int j=0;j<road->odRoute[rIdx]->routeToDestination.size();++j){
            struct RouteElem* re = new struct RouteElem;
            re->node   = road->odRoute[rIdx]->routeToDestination[j]->node;
            re->inDir  = road->odRoute[rIdx]->routeToDestination[j]->inDir;
            re->outDir = road->odRoute[rIdx]->routeToDestination[j]->outDir;
            odr->routeToDestination.append( re );
        }

        for(int j=0;j<road->odRoute[rIdx]->LCSupportLaneLists.size();++j){
            struct RouteLaneData* rd = new struct RouteLaneData;

            rd->LCDirect  = road->odRoute[rIdx]->LCSupportLaneLists[j]->LCDirect;
            rd->goalNode  = road->odRoute[rIdx]->LCSupportLaneLists[j]->goalNode;
            rd->startNode = road->odRoute[rIdx]->LCSupportLaneLists[j]->startNode;
            rd->gIndexInNodeList = road->odRoute[rIdx]->LCSupportLaneLists[j]->gIndexInNodeList;
            rd->sIndexInNodeList = road->odRoute[rIdx]->LCSupportLaneLists[j]->sIndexInNodeList;
            for(int k=0;k<road->odRoute[rIdx]->LCSupportLaneLists[j]->laneList.size();++k){
                QList<int> LL;
                for(int l=0;l<road->odRoute[rIdx]->LCSupportLaneLists[j]->laneList[k].size();++l){
                    LL.append( road->odRoute[rIdx]->LCSupportLaneLists[j]->laneList[k][l] );
                }
                rd->laneList.append( LL );
            }

            odr->LCSupportLaneLists.append( rd );
        }

        odr->totalVolumne = 0;
        odr->meanArrivalTime = 0.0;
        odr->NextAppearTime = 0.0;

        odr->onlyForScenarioVehicle = true;
        odr->relatedScenarioObjectID = aID;
        odr->allowAgentGeneration = true;

        road->odRoute.append( odr );
    }
    else{
        // Create
        qDebug() << "[AppearStoppingVehicle] Create new route.";

        struct ODRouteData* odr = new struct ODRouteData;

        odr->originNode = fmNode;
        odr->destinationNode = toNode;

        int fNIdx = road->nodeId2Index.indexOf( fmNode );
        int inDir = road->nodes[fNIdx]->directionMap[fmNdOt]->oncomingDirect;

        struct RouteElem* re = new struct RouteElem;
        re->node   = fmNode;
        re->inDir  = inDir;
        re->outDir = fmNdOt;
        odr->routeToDestination.append( re );

        int tNIdx = road->nodeId2Index.indexOf( toNode );
        int otDir = road->nodes[tNIdx]->directionMap[toNdIn]->oncomingDirect;

        re = new struct RouteElem;
        re->node   = toNode;
        re->inDir  = toNdIn;
        re->outDir = otDir;
        odr->routeToDestination.append( re );


        struct RouteLaneData* rd = new struct RouteLaneData;

        rd->LCDirect  = 0;
        rd->goalNode  = toNode;
        rd->startNode = fmNode;
        rd->gIndexInNodeList = 1;
        rd->sIndexInNodeList = 0;

        QList<int> LL;
        for(int j=0;j<road->nodes[tNIdx]->pathLists.size();++j){
            for(int k=0;k<road->nodes[tNIdx]->pathLists[j]->pathList.size();++k){
                if( road->nodes[tNIdx]->pathLists[j]->pathList[k].indexOf( nearPath ) >= 0 ){
                    for(int l=0;l<road->nodes[tNIdx]->pathLists[j]->pathList[k].size();++l){
                        LL.append( road->nodes[tNIdx]->pathLists[j]->pathList[k][l] );
                    }
                    break;
                }
            }
            if( LL.size() > 0 ){
                break;
            }
        }

        rd->laneList.append( LL );

        odr->LCSupportLaneLists.append( rd );

        odr->totalVolumne = 0;
        odr->meanArrivalTime = 0.0;
        odr->NextAppearTime = 0.0;

        odr->onlyForScenarioVehicle = true;
        odr->relatedScenarioObjectID = aID;
        odr->allowAgentGeneration = true;

        road->odRoute.append( odr );
    }

    se->eventIntData[2] = 0;

    se->eventFloatData[0] = x;
    se->eventFloatData[1] = y;
    se->eventFloatData[2] = psi;
    se->eventFloatData[3] = 0.0;

    simManage->SetAppearFlagByFE( aID );
    simManage->AppearAgents(agent,maxAgent,road);
}


void SystemThread::SetAgentExternalControlParams(int agentID,float aimPointFactor,float steerGain)
{
    if( agentID >= 0 && agentID < maxAgent ){
        agent[agentID]->memory.aimPointFactorForExternalControl   = aimPointFactor;
        agent[agentID]->memory.steerControlGainForExternalControl = steerGain;
    }
}


void SystemThread::ChangePedestPathForScenarioPedestrian(int aId,QList<float> xPos,QList<float> yPos)
{
    if( xPos.size() != yPos.size() ){
        qDebug() << "[ChangePedestPathForScenarioPedestrian] error : xPos and yPos not same size. ";
        qDebug() << "  xPos.size = " << xPos.size() << "  yPos.size = " << yPos.size();
        return;
    }

    if( aId >= 0 && aId < maxAgent ){

        for(int i=0;i<road->pedestPaths.size();++i){
            if( road->pedestPaths[i]->scenarioObjectID != aId ){
                continue;
            }

            qDebug() << "[ChangePedestPathForScenarioPedestrian] found target pedestPath, id = " << road->pedestPaths[i]->id;

            if( road->pedestPaths[i]->shape.size() != xPos.size() ){
                qDebug() << "[ChangePedestPathForScenarioPedestrian] error : shape size and data size not same.";
                qDebug() << "  shape.size = " << road->pedestPaths[i]->shape.size()  << "  xPos, yPos.size = " << xPos.size();
                break;
            }

            for(int j=0;j<road->pedestPaths[i]->shape.size();++j){
                road->pedestPaths[i]->shape[j]->pos.setX( xPos[j] );
                road->pedestPaths[i]->shape[j]->pos.setY( yPos[j] );
            }

            for(int j=0;j<road->pedestPaths[i]->shape.size()-1;++j){

                float dx = road->pedestPaths[i]->shape[j+1]->pos.x() - road->pedestPaths[i]->shape[j]->pos.x();
                float dy = road->pedestPaths[i]->shape[j+1]->pos.y() - road->pedestPaths[i]->shape[j]->pos.y();

                road->pedestPaths[i]->shape[j]->angleToNextPos = atan2( dy, dx );
                road->pedestPaths[i]->shape[j]->cosA = cos( road->pedestPaths[i]->shape[j]->angleToNextPos );
                road->pedestPaths[i]->shape[j]->sinA = sin( road->pedestPaths[i]->shape[j]->angleToNextPos );

                road->pedestPaths[i]->shape[j]->distanceToNextPos = sqrt( dx * dx + dy * dy );

            }

            int j = road->pedestPaths[i]->shape.size() - 1;
            road->pedestPaths[i]->shape[j]->angleToNextPos = road->pedestPaths[i]->shape[j-1]->angleToNextPos;
            road->pedestPaths[i]->shape[j]->cosA = road->pedestPaths[i]->shape[j-1]->cosA;
            road->pedestPaths[i]->shape[j]->sinA = road->pedestPaths[i]->shape[j-1]->sinA;
            road->pedestPaths[i]->shape[j]->distanceToNextPos = 100.0;

            qDebug() << "[ChangePedestPathForScenarioPedestrian] pedestPath shape changed.";

            break;
        }
    }
    else{
        qDebug() << "[ChangePedestPathForScenarioPedestrian] error : invalid agent id. aId = " << aId;
    }
}


