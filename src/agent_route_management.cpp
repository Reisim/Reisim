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
                    qDebug() << "   Dispose this agent.";
                    return;

                }

                memory.currentTargetPath = currentPath;
                memory.distanceFromStartWPInCurrentPath = 0.0;

                int pIdx = pRoad->pathId2Index.indexOf( currentPath );

                //qDebug() << "Call SetTargetSpeedIndividual: ID = " << ID << " Path=" << currentPath << " idx=" << pIdx << " chk=" << pRoad->paths[pIdx]->id << " V85=" << pRoad->paths[pIdx]->speed85pt;
                if( isScenarioObject == true ){
                    if( pRoad->paths[pIdx]->speed85pt == pRoad->paths[pIdx]->speedInfo ){
                        refSpeedMode = 1;
                    }
                    else{
                        refSpeedMode = 0;
                    }
                }
                SetTargetSpeedIndividual( pRoad->paths[pIdx]->speed85pt );

                int tmpCurrentTargetNode = memory.currentTargetNode;


                //
                //   Path-List Type Scenario Objects escape here
                //
                if( isScenarioObject == true && memory.routeType == ROUTE_TYPE::PATH_LIST_TYPE ){
                    return;
                }


                SetTargetNodeListByTargetPaths( pRoad );


//                qDebug() << "tmpCurrentTargetNode = " << tmpCurrentTargetNode << " memory.currentTargetNode = " << memory.currentTargetNode;
//                qDebug() << "memory.LCStartRouteIndex = " << memory.LCStartRouteIndex;
//                qDebug() << "routeToDestination = " << pRoad->odRoute[memory.routeIndex]->routeToDestination[memory.LCStartRouteIndex]->node;
//                for(int i=0;i<pRoad->odRoute[memory.routeIndex]->routeToDestination.size();++i){
//                    qDebug() << " node = " << pRoad->odRoute[memory.routeIndex]->routeToDestination[i]->node;
//                }


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

                    if( memory.LCbyEventMode != 2 ){
                        if( vehicle.GetWinerState() == 0 ){
                            if( memory.LCDirection == DIRECTION_LABEL::LEFT_CROSSING ){
                                vehicle.SetWinker( 1 );
                            }
                            else if( memory.LCDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
                                vehicle.SetWinker( 2 );
                            }
                        }
                    }

                    memory.LCSupportRouteLaneIndex--;


//                    qDebug() << "Agent[" << ID << "]: Set Lane-Change Flag: Direction = " << (memory.LCDirection == DIRECTION_LABEL::RIGHT_CROSSING ? "RIGHT" : (memory.LCDirection == DIRECTION_LABEL::LEFT_CROSSING ? "LEFT" : "STRAIGHT") );
//                    qDebug() << "  currentTargetNode = " << memory.currentTargetNode << " tmpCurrentTargetNode = " << tmpCurrentTargetNode;
//                    qDebug() << "  LCStartRouteIndex = " << memory.LCStartRouteIndex;
                }
            }

        }
        else{

            // This is in case when warp to different lane
            if( memory.targetPathList.indexOf( memory.currentTargetPath ) < 0 ){

                int i = memory.routeIndex;

                if( pRoad->odRoute[i]->LCSupportLaneLists.size() > 0 ){    // new version

                    for(int j=0;j<pRoad->odRoute[i]->LCSupportLaneLists.size();++j){

                        int numLaneLists = pRoad->odRoute[i]->LCSupportLaneLists[j]->laneList.size();
                        QList<int> validLanes;
                        for(int k=0;k<numLaneLists;++k){
                            if( pRoad->odRoute[i]->LCSupportLaneLists[j]->laneList[k].indexOf(memory.currentTargetPath) >= 0 ){
                                validLanes.append(k);
                            }
                        }
                        if( validLanes.size() == 0 ){
                            continue;
                        }

                        numLaneLists = validLanes.size();

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
                            memory.LCStartRouteIndex = pRoad->odRoute[i]->LCSupportLaneLists[j]->gIndexInNodeList;
                        }
                        else{
                            memory.LCStartRouteIndex = -1;
                        }

                        selIdx = validLanes[selIdx];

                        memory.LCSupportRouteLaneIndex = j;
                        memory.routeLaneIndex = selIdx;
                        memory.targetPathList = pRoad->odRoute[i]->LCSupportLaneLists[j]->laneList[selIdx];

                        memory.laneMerge.clear();

                        int MLIidx = 0;
                        for(int k=0;k<memory.LCSupportRouteLaneIndex;++k){
                            MLIidx += pRoad->odRoute[i]->LCSupportLaneLists[k]->laneList.size();
                        }
                        MLIidx += memory.routeLaneIndex;

                        for(int k=0;k<pRoad->odRoute[i]->mergeLanesInfo[MLIidx].size();++k){

                            QPoint pairData;
                            pairData.setX( pRoad->odRoute[i]->mergeLanesInfo[MLIidx][k].x() );
                            pairData.setY( pRoad->odRoute[i]->mergeLanesInfo[MLIidx][k].y() );

                            memory.laneMerge.append( pairData );
                        }

                        break;
                    }
                }

                memory.targetPathLength.clear();
                for(int j=0;j<memory.targetPathList.size();++j){

                    float len = pRoad->GetPathLength( memory.targetPathList[j] );
                    memory.targetPathLength.append( len );

                    if( memory.targetPathList[j] == memory.currentTargetPath ){
                        memory.currentTargetPathIndexInList = j;
                    }
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


        // Determine Run Out
        if( memory.runOutChecked == false ){
            if( pRoad->pedestPaths[idx]->shape[sIdx]->runOutProb > 0.0 ){
                double rnd = GenUniform();
                double threshold = pRoad->pedestPaths[idx]->shape[sIdx]->runOutProb;
                if( rnd < threshold ){
                    memory.exeRunOut = true;
                    memory.runOutState = 0;

                    rnd = GenUniform();

                    memory.runOutPosInPedestPath = pRoad->pedestPaths[idx]->shape[sIdx]->distanceToNextPos * rnd;

                    if( pRoad->pedestPaths[idx]->shape[sIdx]->runOutDirect == 1 ){
                        memory.runOutDir = state.yaw + 90.0 * 0.0174532;
                    }
                    else{
                        memory.runOutDir = state.yaw -90.0 * 0.0174532;
                    }

                    memory.cosRunOutDir = cos(memory.runOutDir);
                    memory.sinRunOutDir = sin(memory.runOutDir);
                }
            }
            memory.runOutChecked = true;
        }


        float dx = state.x - pRoad->pedestPaths[idx]->shape[sIdx]->pos.x();
        float dy = state.y - pRoad->pedestPaths[idx]->shape[sIdx]->pos.y();
        float S = dx * (pRoad->pedestPaths[idx]->shape[sIdx]->cosA) + dy * (pRoad->pedestPaths[idx]->shape[sIdx]->sinA);


        // If at Run Out Point
        if( memory.exeRunOut == true && memory.runOutState == 0 && S > memory.runOutPosInPedestPath ){
            memory.runOutState = 1;
        }


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
                memory.runOutChecked = false;
                memory.exeRunOut = false;
                memory.runOutState = 0;
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
    memory.currentTargetNodeInDirect = pRoad->paths[pIdx]->connectingNodeInDir;

    int cNdInDir = pRoad->paths[pIdx]->connectingNodeInDir;

    memory.currentTargetNodeIndexInNodeList = -1;
    memory.nextTurnNode = -1;
    memory.nextTurnDirection = -1;
    memory.nextTurnNodeIndexInNodeList = -1;
    memory.isMergeNode = -1;

    memory.oncomingWaitPathList.clear();
    memory.oncomingWaitCPList.clear();
    memory.nextTurnNodeOncomingDir = -1;

    for(int i=0;i<memory.myNodeList.size();++i){
        if( memory.myNodeList[i] == memory.currentTargetNode && memory.myInDirList[i] == cNdInDir ){
            memory.currentTargetNodeIndexInNodeList = i;
            memory.currentTargetNodeOutDirect = memory.myOutDirList[i];
            int ndIdx = pRoad->nodeId2Index.indexOf( memory.currentTargetNode );
            for(int j=0;j<pRoad->nodes[ndIdx]->nCross;++j){
                if( pRoad->GetDirectionLabel( memory.myNodeList.at(i), memory.myInDirList.at(i), pRoad->nodes[ndIdx]->legIDs[j] ) == DIRECTION_LABEL::ONCOMING ){
                    memory.currentTargetNodeOncomingDirect = pRoad->nodes[ndIdx]->legIDs[j];
                    break;
                }
            }
        }
        if( memory.currentTargetNodeIndexInNodeList >= 0 ){
            if( memory.myTurnDirectionList.at(i) == DIRECTION_LABEL::LEFT_CROSSING ||
                    memory.myTurnDirectionList.at(i) == DIRECTION_LABEL::RIGHT_CROSSING ){
                memory.nextTurnNodeIndexInNodeList = i;
                memory.nextTurnNode = memory.myNodeList.at(i);
                memory.nextTurnDirection = memory.myTurnDirectionList.at(i);
                memory.isMergeNode = -1;

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


void Agent::ProcessLaneChangeRequest(Road *road, int LCdir, int LCmode,float moveLatDist)
{
    qDebug() << "[ProcessLaneChangeRequest]";

    if( LCdir != DIRECTION_LABEL::LEFT_CROSSING && LCdir != DIRECTION_LABEL::RIGHT_CROSSING ){
        return;
    }

    int kk = memory.routeIndex;
    int ll = memory.LCSupportRouteLaneIndex;
    int mm = memory.routeLaneIndex;

    qDebug() << "routeIndex = " << kk;
    qDebug() << "LCSupportRouteLaneIndex = " << ll;
    qDebug() << "routeLaneIndex = " << mm;
    qDebug() << "laneList.size = " << road->odRoute[kk]->LCSupportLaneLists[ll]->laneList.size();

    QList<int> candidateList;
    QList<float> tdevList;

    for(int i=0;i<road->odRoute[kk]->LCSupportLaneLists[ll]->laneList.size();++i){

//        if( i == mm ){
//            continue;
//        }

        float tdev,txt,tyt,txd,tyd,ts;
        int objPathInLCPath = road->GetNearestPathFromList( road->odRoute[kk]->LCSupportLaneLists[ll]->laneList[i],
                                                            state.x,
                                                            state.y,
                                                            state.z_path,
                                                            state.yaw,
                                                            tdev, txt, tyt, txd, tyd, ts);

        qDebug() << "i = " << i << " objPathInLCPath = " << objPathInLCPath << " tdev = " << tdev;

        if( objPathInLCPath >= 0 ){
            if( LCdir == DIRECTION_LABEL::RIGHT_CROSSING && tdev > 1.0 ){
                candidateList.append( i );
                tdevList.append( tdev );
            }
            else if( LCdir == DIRECTION_LABEL::LEFT_CROSSING && tdev < -1.0 ){
                candidateList.append( i );
                tdevList.append( tdev );
            }
        }
    }

    qDebug() << "candidateList = " << candidateList;
    qDebug() << "tdevList = " << tdevList;

    if( candidateList.size() == 0 ){

        // Try to find LC path from other route data
        bool addLaneList = false;
        for(int i=0;i<road->odRoute.size();++i){
            if( i == kk ){  // reject self
                continue;
            }
            if( road->odRoute[i]->destinationNode != road->odRoute[kk]->destinationNode ){
                continue;
            }
            int hasTargetNodeInRoute = -1;
            for(int j=0;j<road->odRoute[i]->routeToDestination.size();++j){
                if( road->odRoute[i]->routeToDestination[j]->node == memory.currentTargetNode &&
                        road->odRoute[i]->routeToDestination[j]->outDir == memory.currentTargetNodeOutDirect ){
                    hasTargetNodeInRoute = j;
                    break;
                }
            }
            if( hasTargetNodeInRoute < 0 ){
                continue;
            }

            for(int j=0;j<road->odRoute[i]->LCSupportLaneLists.size();++j){
                if( road->odRoute[i]->LCSupportLaneLists[j]->sIndexInNodeList <= hasTargetNodeInRoute &&
                        hasTargetNodeInRoute <= road->odRoute[i]->LCSupportLaneLists[j]->gIndexInNodeList ){

                    // This Lane-list contains target node
                    for(int k=0;k<road->odRoute[i]->LCSupportLaneLists[j]->laneList.size();++k){
                        if( road->odRoute[kk]->LCSupportLaneLists[ll]->laneList.contains( road->odRoute[i]->LCSupportLaneLists[j]->laneList[k] ) == false ){
                            road->odRoute[kk]->LCSupportLaneLists[ll]->laneList.append( road->odRoute[i]->LCSupportLaneLists[j]->laneList[k] );
                            addLaneList = true;
                        }
                    }
                }
            }
        }

        if( addLaneList == false ){
            return;
        }

        // Evaluate again
        for(int i=0;i<road->odRoute[kk]->LCSupportLaneLists[ll]->laneList.size();++i){

            float tdev,txt,tyt,txd,tyd,ts;
            int objPathInLCPath = road->GetNearestPathFromList( road->odRoute[kk]->LCSupportLaneLists[ll]->laneList[i],
                                                                state.x,
                                                                state.y,
                                                                state.z_path,
                                                                state.yaw,
                                                                tdev, txt, tyt, txd, tyd, ts);

            qDebug() << "i = " << i << " objPathInLCPath = " << objPathInLCPath << " tdev = " << tdev;

            if( objPathInLCPath >= 0 ){
                if( LCdir == DIRECTION_LABEL::RIGHT_CROSSING && tdev > 1.0 ){
                    candidateList.append( i );
                    tdevList.append( tdev );
                }
                else if( LCdir == DIRECTION_LABEL::LEFT_CROSSING && tdev < -1.0 ){
                    candidateList.append( i );
                    tdevList.append( tdev );
                }
            }
        }

        qDebug() << "candidateList = " << candidateList;
        qDebug() << "tdevList = " << tdevList;

        if( candidateList.size() == 0 ){
            return;
        }
    }

    int LCTargetLane = -1;
    if( candidateList.size() > 1 ){

        float minTDev = fabs(tdevList[0]);
        if( moveLatDist > 0.0 ){
            minTDev = moveLatDist;
        }
        else{
            for(int j=1;j<tdevList.size();++j){
                if( minTDev > fabs(tdevList[j]) ){
                    minTDev = fabs(tdevList[j]);
                }
            }
        }

        qDebug() << "minTDev = " << minTDev;

        QList<int> finalCandidate;
        for(int j=0;j<tdevList.size();++j){
            if( fabs(minTDev - fabs(tdevList[j])) < 1.0 ){
                finalCandidate.append( candidateList[j] );
            }
        }

        qDebug() << "finalCandidate = " << finalCandidate;


        int numLaneLists = finalCandidate.size();

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

        qDebug() << "selIdx = " << selIdx;


        LCTargetLane = finalCandidate[selIdx];
    }
    else{
        LCTargetLane = candidateList[0];
    }

    qDebug() << "LCTargetLane = " << LCTargetLane;

    if( LCTargetLane < 0 ){
        return;
    }

    memory.routeLaneIndex = LCTargetLane;
    memory.laneChangeTargetPathList = road->odRoute[kk]->LCSupportLaneLists[ll]->laneList[LCTargetLane];

    qDebug() << "laneChangeTargetPathList = " << memory.laneChangeTargetPathList;


    memory.laneChangePathLength.clear();
    for(int k=0;k<memory.laneChangeTargetPathList.size();++k){
        float len = road->GetPathLength( memory.laneChangeTargetPathList[k] );
        memory.laneChangePathLength.append( len );
    }

    if( LCmode == 0 ){

        // Check Risk of Side Lane
        memory.checkSideVehicleForLC = true;
        memory.LCCheckState          = 1;
        memory.LCInfoGetCount        = 0;

    }
    else{

        // Execute Lane change immediately
        memory.checkSideVehicleForLC = true;
        memory.LCCheckState          = 3;
        memory.LCInfoGetCount        = 0;

        memory.targetPathList.clear();
        memory.targetPathList = memory.laneChangeTargetPathList;

        //qDebug() << "targetPathList = " << memory.targetPathList;

        memory.targetPathLength.clear();
        for(int j=0;j<memory.targetPathList.size();++j){

            float len = road->GetPathLength( memory.targetPathList[j] );
            memory.targetPathLength.append( len );

            if( memory.targetPathList[j] == memory.currentTargetPath ){
                memory.currentTargetPathIndexInList = j;
            }
        }


        float tdev,txt,tyt,txd,tyd,ts;
        int currentPath = road->GetNearestPathFromList( memory.targetPathList,
                                                         state.x,
                                                         state.y,
                                                         state.z_path,
                                                         state.yaw,
                                                         tdev, txt, tyt, txd, tyd, ts );

        //qDebug() << "searched currentPath = " << currentPath;

        if( currentPath >= 0 ){

            memory.currentTargetPath = currentPath;
            memory.lateralDeviationFromTargetPath = tdev;

            int pIdx = road->pathId2Index.indexOf( currentPath );
            if( isScenarioObject == true ){
                if( road->paths[pIdx]->speed85pt == road->paths[pIdx]->speedInfo ){
                    refSpeedMode = 1;
                }
                else{
                    refSpeedMode = 0;
                }
            }
            SetTargetSpeedIndividual( road->paths[pIdx]->speed85pt );

            SetTargetNodeListByTargetPaths( road );

        }
        else{
            qDebug() << "Lane-Change : exchange targetPathList";
            qDebug() << "Agent[" << ID << "]: Cannot determine nearest path from targetPathList;" << memory.targetPathList;
        }

        memory.laneMerge.clear();

        int i = memory.routeIndex;
        int selIdx = 0;
        for(int j=0;j<memory.LCSupportRouteLaneIndex;++j){
            selIdx += road->odRoute[i]->LCSupportLaneLists[j]->laneList.size();
        }
        selIdx += memory.routeLaneIndex;

        if( selIdx < road->odRoute[i]->mergeLanesInfo.size() ){
            for(int k=0;k<road->odRoute[i]->mergeLanesInfo[selIdx].size();++k){

                QPoint pairData;
                pairData.setX( road->odRoute[i]->mergeLanesInfo[selIdx][k].x() );
                pairData.setY( road->odRoute[i]->mergeLanesInfo[selIdx][k].y() );

                memory.laneMerge.append( pairData );
            }
        }
        else{
            qDebug() << "Agent[" << ID << "]: Cannot determine mergeLanesInfo index";
            qDebug() << "  selIdx = " << selIdx << " size = " << road->odRoute[i]->mergeLanesInfo.size();
        }

        memory.LCCheckState = 4;
        memory.LCSteerMax = 0.0;
        memory.checkSideVehicleForLC = false;

        controlCount = controlCountMax;                 // to immediately process at control
    }

    memory.LCDirection = LCdir;
    memory.LCbyEventMode = LCmode;

    if( LCmode < 2 && vehicle.GetWinerState() == 0 ){
        if( memory.LCDirection == DIRECTION_LABEL::LEFT_CROSSING ){
            vehicle.SetWinker( 1 );
        }
        else if( memory.LCDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
            vehicle.SetWinker( 2 );
        }
    }

    qDebug() << "End of ProcessLaneChangeRequest.";
}
