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

                float dist = 0;
                int currentPath = pRoad->GetNearestPathFromList( state.x,
                                                                 state.y,
                                                                 state.yaw,
                                                                 dist,
                                                                 memory.targetPathList );
                if( currentPath < 0 ){
                    qDebug() << "[CheckPathList:Warning]----------------------------------";
                    qDebug() << " Agent ID = " << ID << " cannot determin nearest path from assigned list.";
                    qDebug() << "   currentTargetPath = " << memory.currentTargetPath;
                    qDebug() << "   Assigned Path List : ";
                    for(int j=0;j<memory.targetPathList.size();++j){
                        qDebug() << "           Path " << memory.targetPathList[j];
                    }
                    qDebug() << "   x = " << state.x << " y = " << state.y << "  ip = " << ip;
                    return;
                }

                memory.currentTargetPath = currentPath;

                SetTargetNodeListByTargetPaths( pRoad );
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

        float dx = state.x - pRoad->pedestPaths[idx]->x1;
        float dy = state.y - pRoad->pedestPaths[idx]->y1;
        float S = dx * pRoad->pedestPaths[idx]->eX + dy * pRoad->pedestPaths[idx]->eY;

//        qDebug() << "x = " << state.x << " y = " << state.y;
//        qDebug() << "x1 = " << pRoad->pedestPaths[idx]->x1 << " y1 = " << pRoad->pedestPaths[idx]->y1;
//        qDebug() << "x1 = " << pRoad->pedestPaths[idx]->eX << " y1 = " << pRoad->pedestPaths[idx]->eY;
//        qDebug() << "S = " << S;

        // if arrived at near(0.5[m]) edge of the pedest-path,
        if( (memory.currentTargetDirectionPedestPath == 1 && S < 0.5) ||
             (memory.currentTargetDirectionPedestPath == 2 && S > pRoad->pedestPaths[idx]->Length - 0.5) ){

            if( memory.targetPathList.indexOf( memory.currentTargetPath ) == 0 ){
                // Agent reached goal
                agentStatus = 2;
            }
            else{
                for(int i=memory.targetPathList.size()-1;i>=1;i--){
                    if( memory.targetPathList[i] == memory.currentTargetPath ){
                        int lastTargetPath = memory.currentTargetPath;
                        memory.currentTargetPath = memory.targetPathList[i-1];
                        int dir = pRoad->GetDirectionByPedestPathLink( memory.currentTargetPath, lastTargetPath );
                        if( dir == 1 ){
                            memory.currentTargetDirectionPedestPath = 2;
                        }
                        else if( dir == 2){
                            memory.currentTargetDirectionPedestPath = 1;
                        }
                    }
                }

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
        if( memory.myNodeList.indexOf( tNode ) < 0 ){
            memory.myNodeList.append( tNode );
            memory.myInDirList.append( tInDirt );
        }
    }

    memory.myOutDirList.clear();
    for(int i=0;i<memory.myNodeList.size()-1;++i){
        int nnIdx = pRoad->nodeId2Index.indexOf( memory.myNodeList.at(i+1) );
        for(int j=0;j<pRoad->nodes[nnIdx]->nodeConnectInfo.size();++j){
            if( pRoad->nodes[nnIdx]->nodeConnectInfo[j]->connectedNode == memory.myNodeList.at(i) ){
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

    memory.currentTargetNodeIndexInNodeList = -1;
    memory.nextTurnNode = -1;
    memory.nextTurnDirection = -1;
    memory.nextTurnNodeIndexInNodeList = -1;

    memory.oncomingWaitPathList.clear();
    memory.oncomingWaitCPList.clear();
    memory.nextTurnNodeOncomingDir = -1;

    for(int i=0;i<memory.myNodeList.size();++i){
        if( memory.myNodeList[i] == memory.currentTargetNode ){
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

