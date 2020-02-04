/****************************************************************************
**                                 Re:sim
**
**   Copyright (C) 2020 Misaki Design.LLC
**   Copyright (C) 2020 Jun Tajima <tajima@misaki-design.co.jp>
**
**   This file is part of the Re:sim simulation software
**
**   This file may be used under the terms of the GNU Lesser General Public
**   License version 3 as published by the Free Software Foundation.
**   For more detail, visit https://www.gnu.org/licenses/gpl-3.0.html
**
*************************************************************************** */


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

            qDebug() << "currentTargetPAth = " << memory.currentTargetPath << " idx = " << idx;
            qDebug() << "xp = " << xp << " yp = " << yp;
            qDebug() << "state.x = " << state.x << " state.y = " << state.y;
            qDebug() << "justWarped = " << justWarped;


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
}

