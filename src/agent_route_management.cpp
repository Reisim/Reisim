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


#include "agent.h"
#include "simulationmanager.h"
#include <QDebug>


void Agent::CheckPathList(Road* pRoad)
{
    //qDebug() << "[CheckPathList] currentTargetPath = " << memory.currentTargetPath;

    if( agentKind < 100 ){

        int idx = pRoad->pathId2Index.indexOf( memory.currentTargetPath );
        if( idx < 0 ){
            qDebug() << "[Warning]-----------------------------------------";
            qDebug() << "   ?? cannot get index of currentTargetPath";
            qDebug() << " Agent ID = " << ID;
            qDebug() << " currentTargetPath = " << memory.currentTargetPath;
            return;
        }

        float xp = pRoad->paths[idx]->pos.last()->x();
        float yp = pRoad->paths[idx]->pos.last()->y();
        float xd = pRoad->paths[idx]->derivative.last()->x();
        float yd = pRoad->paths[idx]->derivative.last()->y();

        float rx = state.x - xp;
        float ry = state.y - yp;
        float ip = rx * xd + ry * yd;

        if( ip > 0.0 ){

//            qDebug() << "currentTargetPAth = " << memory.currentTargetPath << " idx = " << idx;
//            qDebug() << "xp = " << xp << " yp = " << yp;
//            qDebug() << "state.x = " << state.x << " state.y = " << state.y;
//            qDebug() << "justWarped = " << justWarped;


            if( memory.targetPathList.indexOf( memory.currentTargetPath ) == 0 ){
                if( justWarped == false ){
                    // Agent reached goal
                    agentStatus = 2;
                }
                else{
                    justWarped = false;
                }

            }
            else{

                if( memory.currentTargetPathIndexInList <= 2 ){
                    int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[1] );
                    memory.targetPathList[0] = pRoad->paths[pIdx]->forwardPaths[0];
                }

                float tdev,txt,tyt,txd,tyd,ts;
                int currentPath = pRoad->GetNearestPathFromList( memory.targetPathList ,
                                                                 state.x,
                                                                 state.y,
                                                                 state.z_path,
                                                                 state.yaw,
                                                                 tdev,txt,tyt,txd,tyd,ts );
                if( currentPath < 0 ){

                    qDebug() << "[CheckPathList:Warning]----------------------------------";
                    qDebug() << " Agent ID = " << ID << " cannot determine nearest path from assigned list.";
                    qDebug() << "   currentTargetPath = " << memory.currentTargetPath;
                    qDebug() << "   Assigned Path List : ";
                    for(int j=0;j<memory.targetPathList.size();++j){
                        qDebug() << "           Path " << memory.targetPathList[j];
                    }
                    qDebug() << "   x = " << state.x << " y = " << state.y << "  ip = " << ip;
                    return;
                }

                memory.currentTargetPath = currentPath;

                int pIdx = pRoad->pathId2Index.indexOf( currentPath );
                SetTargetSpeedIndividual( pRoad->paths[pIdx]->speed85pt );

                int tmpCurrentTargetNode = memory.currentTargetNode;


                //
                //   Path-List Type Scenario Objects escape here
                //
                if( isScenarioObject == true && memory.routeType == ROUTE_TYPE::PATH_LIST_TYPE ){
                    return;
                }


                SetTargetNodeListByTargetPaths( pRoad );

                if( tmpCurrentTargetNode != memory.currentTargetNode &&
                        memory.LCStartRouteIndex >= 0 &&
                        memory.currentTargetNode == pRoad->odRoute[memory.routeIndex]->routeToDestination[memory.LCStartRouteIndex]->node &&
                        memory.checkSideVehicleForLC == false ){


                    memory.laneChangeTargetPathList.clear();

                    int i = memory.routeIndex;
                    int j = memory.LCSupportRouteLaneIndex - 1;

                    int numLaneLists = pRoad->odRoute[i]->LCSupportLaneLists[j]->laneList.size();

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

                    if( j > 0 ){
                        memory.LCStartRouteIndex = pRoad->odRoute[i]->LCSupportLaneLists[j]->gIndexInNodeList;
                    }
                    else{
                        memory.LCStartRouteIndex = -1;
                    }


                    memory.routeLaneIndex = selIdx;
                    memory.laneChangeTargetPathList = pRoad->odRoute[i]->LCSupportLaneLists[j]->laneList[selIdx];

//                    qDebug() << "laneChangeTargetPathList = " << memory.laneChangeTargetPathList;


                    memory.laneChangePathLength.clear();
                    for(int k=0;k<memory.laneChangeTargetPathList.size();++k){
                        float len = pRoad->GetPathLength( memory.laneChangeTargetPathList[k] );
                        memory.laneChangePathLength.append( len );
                    }


                    memory.checkSideVehicleForLC = true;
                    memory.LCCheckState          = 1;
                    memory.LCInfoGetCount        = 0;

                    // Determine LC Direction
                    if( memory.LCSupportRouteLaneIndex >= 0 && memory.LCSupportRouteLaneIndex < pRoad->odRoute[memory.routeIndex]->LCSupportLaneLists.size() ){
                        memory.LCDirection = pRoad->odRoute[memory.routeIndex]->LCSupportLaneLists[memory.LCSupportRouteLaneIndex]->LCDirect;
                    }

                    if( vehicle.GetWinerState() == 0 ){
                        if( memory.LCDirection == DIRECTION_LABEL::LEFT_CROSSING ){
                            vehicle.SetWinker( 1 );
                        }
                        else if( memory.LCDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
                            vehicle.SetWinker( 2 );
                        }
                    }

                    memory.LCSupportRouteLaneIndex--;


//                    qDebug() << "Agent[" << ID << "]: Set Lane-Change Flag: Direction = " << (memory.LCDirection == DIRECTION_LABEL::RIGHT_CROSSING ? "RIGHT" : (memory.LCDirection == DIRECTION_LABEL::LEFT_CROSSING ? "LEFT" : "STRAIGHT") );
//                    qDebug() << "  currentTargetNode = " << memory.currentTargetNode << " tmpCurrentTargetNode = " << tmpCurrentTargetNode;
//                    qDebug() << "  LCStartRouteIndex = " << memory.LCStartRouteIndex;
                }
            }

        }
    }
    else if( agentKind >= 100 ){

        int idx = pRoad->pedestPathID2Index.indexOf( memory.currentTargetPath );

//        qDebug() << "currentTargetPath = " << memory.currentTargetPath << "  idx = " << idx;


        if( idx < 0 ){
            qDebug() << "[Warning]-----------------------------------------";
            qDebug() << "   ?? cannot get index of currentTargetPath";
            qDebug() << " Agent ID = " << ID;
            qDebug() << " currentTargetPath = " << memory.currentTargetPath;
            return;
        }

        int sIdx = memory.currentTargetPathIndexInList;

        float dx = state.x - pRoad->pedestPaths[idx]->shape[sIdx]->pos.x();
        float dy = state.y - pRoad->pedestPaths[idx]->shape[sIdx]->pos.y();
        float S = dx * (pRoad->pedestPaths[idx]->shape[sIdx]->cosA) + dy * (pRoad->pedestPaths[idx]->shape[sIdx]->sinA);

//        qDebug() << "S = " << S << " Dist = " << pRoad->pedestPaths[idx]->shape[sIdx]->distanceToNextPos;

        // if arrived at near(0.5[m]) edge of the pedest-path,
        if( pRoad->pedestPaths[idx]->shape[sIdx]->distanceToNextPos - S < 1.0 ){

//            qDebug() << "Near Termination";

            if( sIdx == pRoad->pedestPaths[idx]->shape.size() - 2 ){
                // Agent reached goal
                agentStatus = 2;
            }
            else{
                memory.currentTargetPathIndexInList++;
            }
        }
    }
}


void Agent::SetTargetNodeListByTargetPaths(Road* pRoad)
{
//    qDebug() << "[SetTargetNodeListByTargetPaths] ID = " << ID;

    memory.myNodeList.clear();
    memory.myInDirList.clear();

    for(int i=memory.targetPathList.size()-1;i>=0;--i){
        int tpIdx   = pRoad->pathId2Index.indexOf( memory.targetPathList[i] );
        int tNode   = pRoad->paths[tpIdx]->connectingNode;
        int tInDirt = pRoad->paths[tpIdx]->connectingNodeInDir;
        if( memory.myNodeList.indexOf( tNode ) < 0 || (memory.myNodeList.size() > 0 && memory.myNodeList.last() != tNode) ){
            memory.myNodeList.append( tNode );
            memory.myInDirList.append( tInDirt );
        }
    }

    memory.myOutDirList.clear();
    for(int i=0;i<memory.myNodeList.size()-1;++i){
        int nnIdx = pRoad->nodeId2Index.indexOf( memory.myNodeList.at(i+1) );
        for(int j=0;j<pRoad->nodes[nnIdx]->nodeConnectInfo.size();++j){
            if( pRoad->nodes[nnIdx]->nodeConnectInfo[j]->connectedNode == memory.myNodeList.at(i) &&
                  j == memory.myInDirList.at(i+1)  ){
                memory.myOutDirList.append( pRoad->nodes[nnIdx]->nodeConnectInfo[j]->outDirectionID );
                break;
            }
        }
    }
    memory.myOutDirList.append(-1);

    memory.myTurnDirectionList.clear();
    for(int i=0;i<memory.myNodeList.size()-1;++i){
        int TurnDir = pRoad->GetDirectionLabel( memory.myNodeList.at(i), memory.myInDirList.at(i), memory.myOutDirList.at(i) );
        memory.myTurnDirectionList.append( TurnDir );
    }
    memory.myTurnDirectionList.append(0);


    int pIdx = pRoad->pathId2Index.indexOf( memory.currentTargetPath );
    if( memory.currentTargetNode != pRoad->paths[pIdx]->connectingNode && memory.isChaningLane == false ){
        vehicle.SetWinker(0);
    }

    memory.currentTargetNode = pRoad->paths[pIdx]->connectingNode;

    int cNdInDir = pRoad->paths[pIdx]->connectingNodeInDir;

    memory.currentTargetNodeIndexInNodeList = -1;
    memory.nextTurnNode = -1;
    memory.nextTurnDirection = -1;
    memory.nextTurnNodeIndexInNodeList = -1;

    memory.oncomingWaitPathList.clear();
    memory.oncomingWaitCPList.clear();
    memory.nextTurnNodeOncomingDir = -1;

    for(int i=0;i<memory.myNodeList.size();++i){
        if( memory.myNodeList[i] == memory.currentTargetNode && memory.myInDirList[i] == cNdInDir ){
            memory.currentTargetNodeIndexInNodeList = i;
        }
        if( memory.currentTargetNodeIndexInNodeList >= 0 ){
            if( memory.myTurnDirectionList.at(i) == DIRECTION_LABEL::LEFT_CROSSING ||
                    memory.myTurnDirectionList.at(i) == DIRECTION_LABEL::RIGHT_CROSSING ){
                memory.nextTurnNodeIndexInNodeList = i;
                memory.nextTurnNode = memory.myNodeList.at(i);
                memory.nextTurnDirection = memory.myTurnDirectionList.at(i);

                int ndIdx = pRoad->nodeId2Index.indexOf( memory.nextTurnNode );
                for(int j=0;j<pRoad->nodes[ndIdx]->nCross;++j){
                    if( pRoad->GetDirectionLabel( memory.myNodeList.at(i), memory.myInDirList.at(i), pRoad->nodes[ndIdx]->legIDs[j] ) == DIRECTION_LABEL::ONCOMING ){
                        memory.nextTurnNodeOncomingDir = pRoad->nodes[ndIdx]->legIDs[j];
                        break;
                    }
                }

//                qDebug() << "memory.nextTurnNodeOncomingDir = " << memory.nextTurnNodeOncomingDir;


                for(int j=memory.currentTargetPathIndexInList;j>=0;j--){

                    pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );

//                    qDebug() << "Checking path: " << memory.targetPathList[j];

                    if( pRoad->paths[pIdx]->connectingNode == memory.nextTurnNode ){

//                        qDebug() << "crossPoints.size = " << pRoad->paths[pIdx]->crossPoints.size();

                        for( int k=0;k<pRoad->paths[pIdx]->crossPoints.size(); k++ ){

                            int cpIdx = pRoad->pathId2Index.indexOf( pRoad->paths[pIdx]->crossPoints[k]->crossPathID );

//                            qDebug() << "CrossPath: " << pRoad->paths[pIdx]->crossPoints[k]->crossPathID
//                                     << " InDir = " << pRoad->paths[cpIdx]->connectingNodeInDir;

                            if( pRoad->paths[cpIdx]->connectingNodeInDir == memory.nextTurnNodeOncomingDir ){
                                memory.oncomingWaitPathList.append( memory.targetPathList[j] );
                                memory.oncomingWaitCPList.append( k );
                            }
                        }
                    }
                }

                break;
            }
        }
    }


    memory.nearOncomingWaitPathInfo = -1;
    memory.nearOncomingWaitCPInfo   = -1;
    memory.farOncomingWaitPathInfo  = -1;
    memory.farOncomingWaitCPInfo    = -1;

    float minDist = 0.0;
    float maxDist = 0.0;
    for(int i=0;i<memory.oncomingWaitPathList.size();++i){
        float dist = 0;
        for(int j=memory.currentTargetPathIndexInList;j>=0;j--){

            pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );

            if( memory.targetPathList[j] == memory.oncomingWaitPathList[i] ){

                dist += pRoad->paths[pIdx]->crossPoints[ memory.oncomingWaitCPList[i] ]->distFromStartWP;
                break;
            }


            dist += pRoad->paths[pIdx]->pathLength;
        }
        dist -= memory.distanceFromStartWPInCurrentPath;

        if( memory.nearOncomingWaitPathInfo < 0 || minDist > dist ){

            memory.nearOncomingWaitPathInfo = memory.oncomingWaitPathList[i];
            memory.nearOncomingWaitCPInfo   = memory.oncomingWaitCPList[i];
            minDist = dist;
        }

        if( memory.farOncomingWaitPathInfo < 0 || maxDist < dist ){

            memory.farOncomingWaitPathInfo = memory.oncomingWaitPathList[i];
            memory.farOncomingWaitCPInfo   = memory.oncomingWaitCPList[i];
            maxDist = dist;
        }
    }

}

