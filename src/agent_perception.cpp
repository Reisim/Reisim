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

void Agent::Perception( Agent** pAgent, int maxAgent, Road* pRoad, QList<TrafficSignal*> trafficSignal )
{
    strForDebug = QString("");


    //
    //  Calculation of position on path
    //
    memory.currentTargetPathIndexInList = -1;
    for(int i = memory.targetPathList.size()-1 ; i>=0 ; i--){
        if( memory.currentTargetPathIndexInList < 0 && memory.targetPathList[i] != memory.currentTargetPath ){
            continue;
        }
        memory.currentTargetPathIndexInList = i;
        break;
    }


    float preview_dist = 3.5;
    float preview_x = state.x + preview_dist * state.cosYaw;
    float preview_y = state.y + preview_dist * state.sinYaw;

    int minIdx = -1;
    float minDev = 0.0;
    for(int i=memory.currentTargetPathIndexInList;i>=0;i--){

        float tdev,txt,tyt,txd,tyd,ts;

        if( i == memory.currentTargetPathIndexInList ){
            int chk = pRoad->GetDeviationFromPath( memory.targetPathList[i],
                                                   state.x, state.y, state.yaw,
                                                   tdev, txt, tyt, txd, tyd, ts );

            if( chk == memory.targetPathList[i] ){
                memory.lateralDeviationFromTargetPath = tdev;
                memory.distanceFromStartWPInCurrentPath = ts;
            }
        }

        int chk = pRoad->GetDeviationFromPath( memory.targetPathList[i],
                                               preview_x, preview_y, state.yaw,
                                               tdev, txt, tyt, txd, tyd, ts, false, true );

//        qDebug() << "i=" << i << " chk=" << chk << " dev=" << tdev;

//        if( ID == 13 ){
//            qDebug() << "path = " << memory.targetPathList[i] << " chk = " << chk << " tdev=" << tdev;
//        }
        if( chk != memory.targetPathList[i] ){
            continue;
        }
        if( isnan(tdev) == true ){
            memory.lateralDeviationFromTargetPathAtPreviewPoint = 0.0;
            break;
        }

        if( minIdx < 0 || fabs(minDev) > fabs(tdev) ){
            minIdx = i;
            minDev = tdev;
        }
    }

    if( minIdx >= 0 ){
        memory.lateralDeviationFromTargetPathAtPreviewPoint = minDev;
        memory.previewPointPath = memory.targetPathList[minIdx];
    }
    else{
        memory.lateralDeviationFromTargetPathAtPreviewPoint = 0.0;
        memory.previewPointPath = -1;
    }


    memory.distanceToTurnNodeWPIn = -1.0;
    if( memory.nextTurnNode >= 0 ){

        QList<int> destPaths;
        int inDir = memory.myInDirList.at( memory.nextTurnNodeIndexInNodeList );
        int tnIdx = pRoad->nodeId2Index.indexOf( memory.nextTurnNode );
        for(int i=0;i<pRoad->nodes[tnIdx]->inBoundaryWPs.size();++i){
            if( pRoad->nodes[tnIdx]->inBoundaryWPs[i]->relatedDirection == inDir ){
                for(int j=0;j<pRoad->nodes[tnIdx]->inBoundaryWPs[i]->PathWithEWP.size();++j){
                    destPaths.append( pRoad->nodes[tnIdx]->inBoundaryWPs[i]->PathWithEWP[j] );
                }
            }
        }

        float dist = 0.0;
        for(int i=memory.currentTargetPathIndexInList;i>=0;i--){
            int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList.at(i) );
            dist += pRoad->paths[pIdx]->pathLength;
            if( pRoad->paths[pIdx]->connectingNode == memory.nextTurnNode ){
                if( destPaths.indexOf(memory.targetPathList.at(i)) >= 0 ){
                    break;
                }
            }
        }
        dist -= memory.distanceFromStartWPInCurrentPath;

        memory.distanceToTurnNodeWPIn = dist;
    }




    //
    //  Perception
    //
    float VD2 = param.visibleDistance * param.visibleDistance;


    // Agents
    for(int j=0;j<memory.perceptedObjects.size();++j){
        memory.perceptedObjects[j]->inView = false;
        memory.perceptedObjects[j]->noUpdateCount++;
    }

    for(int i=0;i<maxAgent;++i){

        if( i == ID ){
            continue;
        }

        if( pAgent[i]->agentStatus == 0 ){
            continue;
        }

        float rx = pAgent[i]->state.x - state.x;
        float ry = pAgent[i]->state.y - state.y;
        if( rx * rx + ry * ry > VD2 ){
            continue;
        }

        int alreadyPercepted = -1;
        for(int j=0;j<memory.perceptedObjects.size();++j){
            if( memory.perceptedObjects[j]->objectID == pAgent[i]->ID ){
                alreadyPercepted = j;
                break;
            }
        }
        if( alreadyPercepted >= 0 ){

            memory.perceptedObjects[alreadyPercepted]->inView = true;
            memory.perceptedObjects[alreadyPercepted]->noUpdateCount = 0;

            memory.perceptedObjects[alreadyPercepted]->x       = pAgent[i]->state.x;
            memory.perceptedObjects[alreadyPercepted]->y       = pAgent[i]->state.y;
            memory.perceptedObjects[alreadyPercepted]->yaw     = pAgent[i]->state.yaw;
            memory.perceptedObjects[alreadyPercepted]->cos_yaw = pAgent[i]->state.cosYaw;
            memory.perceptedObjects[alreadyPercepted]->sin_yaw = pAgent[i]->state.sinYaw;
            memory.perceptedObjects[alreadyPercepted]->V       = pAgent[i]->state.V;
            memory.perceptedObjects[alreadyPercepted]->Ax      = pAgent[i]->state.accel - pAgent[i]->state.brake;

            memory.perceptedObjects[alreadyPercepted]->objectPath              = pAgent[i]->memory.currentTargetPath;
            memory.perceptedObjects[alreadyPercepted]->deviationFromObjectPath = pAgent[i]->memory.lateralDeviationFromTargetPath;

            memory.perceptedObjects[alreadyPercepted]->nearestTargetPath              = -1;
            memory.perceptedObjects[alreadyPercepted]->deviationFromNearestTargetPath = 0.0;
            memory.perceptedObjects[alreadyPercepted]->distanceToObject               = 0.0;

            int latMinIndex = -1;
            float latMin = 0.0;
            for(int k=0;k<memory.targetPathList.size();++k){

                float tdev,txt,tyt,txd,tyd,ts;
                int chk = pRoad->GetDeviationFromPath( memory.targetPathList[k],
                                                       pAgent[i]->state.x, pAgent[i]->state.y, pAgent[i]->state.yaw,
                                                       tdev, txt, tyt, txd, tyd, ts,
                                                       false, true );
                if( chk != memory.targetPathList[k] ){
                    continue;
                }

                if( latMinIndex < 0 || latMin > fabs(tdev) ){
                    latMinIndex = k;
                    latMin = fabs(tdev);
                }
            }


            if( latMinIndex >= 0 ){

                int k = latMinIndex;

//                qDebug() << "Selected: " << memory.targetPathList[k];

                float tdev,txt,tyt,txd,tyd,distFromSWP;
                int chk = pRoad->GetDeviationFromPath( memory.targetPathList[k],
                                                       pAgent[i]->state.x, pAgent[i]->state.y, pAgent[i]->state.yaw,
                                                       tdev, txt, tyt, txd, tyd, distFromSWP,
                                                       false, true );

//                qDebug() << "ts=" << ts;
//                qDebug() << "distanceFromStartWPInCurrentPath=" << memory.distanceFromStartWPInCurrentPath;
//                qDebug() << "k=" << k << " currentTargetPathIndexInList=" << memory.currentTargetPathIndexInList;


                AgentPerception *ap = memory.perceptedObjects[alreadyPercepted];
                ap->nearestTargetPath              = memory.targetPathList[k];
                ap->deviationFromNearestTargetPath = tdev;
                ap->xOnTargetPath                  = txt;
                ap->yOnTargetPath                  = tyt;

                ap->innerProductToNearestPathTangent = txd * (pAgent[i]->state.cosYaw) + tyd * (pAgent[i]->state.sinYaw);
                ap->innerProductToNearestPathNormal  = (-tyd) * ap->cos_yaw + txd * ap->sin_yaw;
                ap->effectiveHalfWidth = fabs(ap->innerProductToNearestPathTangent) * ap->vHalfWidth + fabs(ap->innerProductToNearestPathNormal) * ap->vHalfLength;


                float dist = 0.0;

                bool foundMe = false;
                bool foundObj = false;
                float distMe = 0.0;
                float distObj = 0.0;
                for(int l=memory.targetPathList.size()-1;l>=0;l--){
                    if( memory.targetPathList[l] == memory.currentTargetPath ){
                        foundMe = true;
                        distMe += memory.distanceFromStartWPInCurrentPath;
                    }
                    if( l == k ){
                        foundObj = true;
                        distObj += distFromSWP;
                    }
                    if( foundMe == true && foundObj == true ){
                        break;
                    }
                    else if(foundMe == false && foundObj == false ){
                        continue;
                    }
                    float pLen = pRoad->paths[ pRoad->pathId2Index.indexOf(memory.targetPathList[l]) ]->pathLength;
                    if( foundMe == false ){
                        distMe += pLen;
                    }
                    if( foundObj == false ){
                        distObj += pLen;
                    }
                }
                dist = distObj - distMe;

                memory.perceptedObjects[alreadyPercepted]->distanceToObject = dist;

//                qDebug() << "distanceToObject=" << memory.perceptedObjects[alreadyPercepted]->distanceToObject;

                continue;
            }

        }
        else{

            struct AgentPerception *ap = new struct AgentPerception;

            ap->inView = true;
            ap->noUpdateCount = 0;

            ap->objectID   = pAgent[i]->ID;
            ap->objectType = pAgent[i]->agentKind;

            ap->vHalfLength  = pAgent[i]->vHalfLength;
            ap->vHalfWidth   = pAgent[i]->vHalfWidth;

            ap->x       = pAgent[i]->state.x;
            ap->y       = pAgent[i]->state.y;
            ap->yaw     = pAgent[i]->state.yaw;
            ap->cos_yaw = pAgent[i]->state.cosYaw;
            ap->sin_yaw = pAgent[i]->state.sinYaw;
            ap->V       = pAgent[i]->state.V;
            ap->Ax      = pAgent[i]->state.accel - pAgent[i]->state.brake;

            ap->objectPath              = pAgent[i]->memory.currentTargetPath;
            ap->deviationFromObjectPath = pAgent[i]->memory.lateralDeviationFromTargetPath;

            ap->nearestTargetPath              = -1;
            ap->deviationFromNearestTargetPath = 0.0;
            ap->distanceToObject               = 0.0;

            int latMinIndex = -1;
            float latMin = 0.0;
            for(int k=0;k<memory.targetPathList.size();++k){

                float tdev,txt,tyt,txd,tyd,ts;
                int chk = pRoad->GetDeviationFromPath( memory.targetPathList[k],
                                                       pAgent[i]->state.x, pAgent[i]->state.y, pAgent[i]->state.yaw,
                                                       tdev, txt, tyt, txd, tyd, ts,
                                                       false, true );
                if( chk != memory.targetPathList[k] ){
                    continue;
                }

                if( latMinIndex < 0 || latMin > fabs(tdev) ){
                    latMinIndex = k;
                    latMin = fabs(tdev);
                }
            }

            if( latMinIndex >= 0 ){

                int k = latMinIndex;

                float tdev,txt,tyt,txd,tyd,distFromSWP;
                int chk = pRoad->GetDeviationFromPath( memory.targetPathList[k],
                                                       pAgent[i]->state.x, pAgent[i]->state.y, pAgent[i]->state.yaw,
                                                       tdev, txt, tyt, txd, tyd, distFromSWP,
                                                       false, true );

                ap->nearestTargetPath              = memory.targetPathList[k];
                ap->deviationFromNearestTargetPath = tdev;
                ap->xOnTargetPath                  = txt;
                ap->yOnTargetPath                  = tyt;
                ap->innerProductToNearestPathTangent = txd * ap->cos_yaw + tyd * ap->sin_yaw;
                ap->innerProductToNearestPathNormal  = (-tyd) * ap->cos_yaw + txd * ap->sin_yaw;
                ap->effectiveHalfWidth = fabs(ap->innerProductToNearestPathTangent) * ap->vHalfWidth + fabs(ap->innerProductToNearestPathNormal) * ap->vHalfLength;

                float dist = 0.0;

                bool foundMe = false;
                bool foundObj = false;
                float distMe = 0.0;
                float distObj = 0.0;
                for(int l=memory.targetPathList.size()-1;l>=0;l--){
                    if( memory.targetPathList[l] == memory.currentTargetPath ){
                        foundMe = true;
                        distMe += memory.distanceFromStartWPInCurrentPath;
                    }
                    if( l == k ){
                        foundObj = true;
                        distObj += distFromSWP;
                    }
                    if( foundMe == true && foundObj == true ){
                        break;
                    }
                    else if(foundMe == false && foundObj == false ){
                        continue;
                    }
                    float pLen = pRoad->paths[ pRoad->pathId2Index.indexOf(memory.targetPathList[l]) ]->pathLength;
                    if( foundMe == false ){
                        distMe += pLen;
                    }
                    if( foundObj == false ){
                        distObj += pLen;
                    }
                }
                dist = distObj - distMe;

                ap->distanceToObject               = dist;
            }


            memory.perceptedObjects.append( ap );
        }
    }

    for(int j=memory.perceptedObjects.size()-1;j>=0;--j){
        if( memory.perceptedObjects[j]->noUpdateCount > 3 ){
            delete memory.perceptedObjects[j];
            memory.perceptedObjects.removeAt(j);
        }
    }



//    qDebug() << "[Agent ID = " << ID << "] Perception: ";
//    qDebug() << " num perceptedObjects = " << memory.perceptedObjects.size();
//    for(int i=0;i<memory.perceptedObjects.size();++i){
//        qDebug() << "  percepted object[" << i << "]";
//        qDebug() << "    ID = " << memory.perceptedObjects[i]->objectID;
//        qDebug() << "    Type = " << memory.perceptedObjects[i]->objectType;
//        qDebug() << "    x = " << memory.perceptedObjects[i]->x;
//        qDebug() << "    y = " << memory.perceptedObjects[i]->y;
//        qDebug() << "    yaw = " << memory.perceptedObjects[i]->yaw;
//        qDebug() << "    V = " << memory.perceptedObjects[i]->V;
//        qDebug() << "    objectPath = " << memory.perceptedObjects[i]->objectPath;
//        qDebug() << "    deviation = " << memory.perceptedObjects[i]->deviationFromObjectPath;
//        qDebug() << "    nearPath = " << memory.perceptedObjects[i]->nearestTargetPath;
//        qDebug() << "    deviation = " << memory.perceptedObjects[i]->deviationFromNearestTargetPath;
//        qDebug() << "    distance = " << memory.perceptedObjects[i]->distanceToObject;
//        qDebug() << "    inView = " << memory.perceptedObjects[i]->inView;
//        qDebug() << "    noUpdateCount = " << memory.perceptedObjects[i]->noUpdateCount;
//    }



    // Traffic Signals

    for(int i=0;i<memory.perceptedSignals.size();++i){
        memory.perceptedSignals[i]->inView = false;
        memory.perceptedSignals[i]->noUpdateCount++;
    }

    if( memory.routeType == AGENT_ROUTE_TYPE::NODE_LIST ){

        QList<int> checkedNode;

        int cIdx = memory.currentTargetPathIndexInList;
        for(int i = cIdx; i >= 0; i--){

            int tPath = memory.targetPathList[i];
            int tpIdx = pRoad->pathId2Index.indexOf( tPath );
            int cNode = pRoad->paths[tpIdx]->connectingNode;
            if( checkedNode.indexOf(cNode) >= 0 ){
                continue;
            }
            else{
                checkedNode.append(cNode);
            }
            int ndIdx = pRoad->nodeId2Index.indexOf( cNode );
            if( pRoad->nodes[ndIdx]->hasTS == false ){
                continue;
            }

            float dx = pRoad->nodes[ndIdx]->xc - state.x;
            float dy = pRoad->nodes[ndIdx]->yc - state.y;
            float D = dx * dx + dy * dy;
            if( D > VD2 ){
                continue;
            }

            // find ts
            int inDir = pRoad->paths[tpIdx]->connectingNodeInDir;

            for(int j=0;j<trafficSignal.size();++j){

                if( trafficSignal[j]->type != 'v' ){
                    continue;
                }
                if( trafficSignal[j]->relatedNode != cNode ){
                    continue;
                }
                if( trafficSignal[j]->controlDirection != inDir ){
                    continue;
                }

                int psIdx = -1;
                for(int k=0;k<memory.perceptedSignals.size();++k){
                    if( memory.perceptedSignals[k]->objectID == trafficSignal[j]->id ){
                        psIdx = k;
                        break;
                    }
                }

                if( psIdx < 0 ){

                    struct TrafficSignalPerception* ts = new struct TrafficSignalPerception;

                    ts->objectID    = trafficSignal[j]->id;
                    ts->objectType  = 'v';
                    ts->x           = trafficSignal[j]->xTS;
                    ts->y           = trafficSignal[j]->yTS;
                    ts->yaw         = trafficSignal[j]->direction;
                    ts->relatedNode = cNode;
                    ts->SLonPathID  = -1;

                    memory.perceptedSignals.append( ts );

                    psIdx = memory.perceptedSignals.size() - 1;
                }

                memory.perceptedSignals[psIdx]->signalDisplay = trafficSignal[j]->GetCurrentDisplayInfo();

                if( memory.perceptedSignals[psIdx]->SLonPathID < 0 ){

                    memory.perceptedSignals[psIdx]->distToSL = (-1.0) * memory.distanceFromStartWPInCurrentPath;

                    for(int k = cIdx; k >= 0; k--){
                        tPath = memory.targetPathList[k];
                        tpIdx = pRoad->pathId2Index.indexOf( tPath );
                        bool foundTs = false;
                        for(int l=0;l<pRoad->paths[tpIdx]->stopPoints.size();++l){
                            if( pRoad->paths[tpIdx]->stopPoints[l]->relatedNode == cNode ){

                                memory.perceptedSignals[psIdx]->stopLineX = pRoad->paths[tpIdx]->stopPoints[l]->pos.x();
                                memory.perceptedSignals[psIdx]->stopLineY = pRoad->paths[tpIdx]->stopPoints[l]->pos.y();

                                memory.perceptedSignals[psIdx]->distToSL += pRoad->paths[tpIdx]->stopPoints[l]->distFromStartWP;
                                memory.perceptedSignals[psIdx]->SLonPathID = pRoad->paths[tpIdx]->id;
                                memory.perceptedSignals[psIdx]->stopPointIndex = l;

                                foundTs = true;
                                break;
                            }
                        }
                        if( foundTs == true ){
                            break;
                        }

                        memory.perceptedSignals[psIdx]->distToSL += pRoad->paths[tpIdx]->pathLength;
                    }
                }
                else{

                    bool foundCurPath = false;
                    bool foundSLPath = false;
                    float distCurPath = 0.0;
                    float distSLPath  = 0.0;
                    for(int k = memory.targetPathList.size()-1; k >= 0; k--){

                        tPath = memory.targetPathList[k];
                        tpIdx = pRoad->pathId2Index.indexOf( tPath );

                        if( memory.targetPathList[k] == memory.currentTargetPath ){
                            foundCurPath = true;
                            distCurPath += memory.distanceFromStartWPInCurrentPath;
                        }
                        if( memory.targetPathList[k] == memory.perceptedSignals[psIdx]->SLonPathID ){
                            foundSLPath = true;
                            int l = memory.perceptedSignals[psIdx]->stopPointIndex;
                            distSLPath += pRoad->paths[tpIdx]->stopPoints[l]->distFromStartWP;
                        }

                        if( foundSLPath == true && foundCurPath == true ){
                            break;
                        }
                        else if( foundSLPath == false && foundCurPath == false ){
                            continue;
                        }

                        float pLen = pRoad->paths[tpIdx]->pathLength;
                        if( foundCurPath == false ){
                            distCurPath += pLen;
                        }
                        if( foundSLPath == false ){
                            distSLPath  += pLen;
                        }
                    }

                    memory.perceptedSignals[psIdx]->distToSL = distSLPath - distCurPath;
                }

                memory.perceptedSignals[psIdx]->inView = true;
                memory.perceptedSignals[psIdx]->noUpdateCount = 0;
            }
        }
    }


    for(int j=memory.perceptedSignals.size()-1;j>=0;--j){

    }


    for(int j=memory.perceptedSignals.size()-1;j>=0;--j){
        if( memory.perceptedSignals[j]->noUpdateCount > 3 ){
            delete memory.perceptedSignals[j];
            memory.perceptedSignals.removeAt(j);
        }
    }


//    qDebug() << "Percetion of Traffic Signals:";
//    for(int i=0;i<memory.perceptedSignals.size();++i){
//        qDebug() << " ID = " << memory.perceptedSignals[i]->objectID
//                 << " Color = " << memory.perceptedSignals[i]->signalDisplay
//                 << " Dist = " << memory.perceptedSignals[i]->distToSL;
//    }
}


