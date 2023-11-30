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



void Agent::HazardIdentification( Agent** pAgent, int maxAgent, Road* pRoad )
{

    decisionMakingCount++;
    if( decisionMakingCount >= decisionMakingCountMax ){
        decisionMakingCount = 0;

        strForDebugRiskEval = QString("-----\n");
    }
    else{
        return;
    }


    if( memory.controlMode == AGENT_CONTROL_MODE::AGENT_LOGIC ){

        if( agentKind < 100 ){


            // If far from intersection, skip CP check
            bool skipCPCheckOfVehicle = false;
            if( memory.distanceToCurrentTargetNodeWPIn > 6.0 * ( state.V > 5.0 ? state.V : 5.0) ){

                skipCPCheckOfVehicle = true;

            }

            if( onlyCheckPreceding == true ){

                skipCPCheckOfVehicle = true;

            }


#ifdef _PERFORMANCE_CHECK_AGENT_HAZARD
            QueryPerformanceCounter(&start);
#endif

            // Collision or Mergining Point Check
            for(int i=0;i<memory.perceptedObjects.size();++i){

                memory.perceptedObjects[i]->hasCollisionPoint = false;

                if( skipCPCheckOfVehicle == true && memory.perceptedObjects[i]->objectType < 100 ){
                    continue;
                }

                if( memory.perceptedObjects[i]->isValidData == false ){
                    continue;
                }

//                if( pAgent[memory.perceptedObjects[i]->objectID]->isSInterfaceObject == true ){
//                    continue;
//                }

                if( memory.perceptedObjects[i]->relPosEvaled == false ){
                    continue;
                }


                // Not evaluate far object
                if( memory.perceptedObjects[i]->distanceToObject > 100.0 ){
                    continue;
                }
                // Not evaluate behind object
                else if( memory.perceptedObjects[i]->distanceToObject < -5.0 ){
                    continue;
                }


                memory.perceptedObjects[i]->myCPPathIndex  = -1;
                memory.perceptedObjects[i]->objCPPathIndex = -1;


                if( memory.perceptedObjects[i]->objectType >= 100 ){

                    int plIdx = pRoad->pedestPathID2Index.indexOf( memory.perceptedObjects[i]->objectPath );
                    if( plIdx >= 0 ){

                        memory.perceptedObjects[i]->hasCollisionPoint = false;

                        for(int j=memory.currentTargetPathIndexInList;j>=0;j--){

                            int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
                            if( pRoad->paths[pIdx]->pedestCrossPoints.size() == 0 ){
                                continue;
                            }

                            for(int k=0;k<pRoad->paths[pIdx]->pedestCrossPoints.size();++k){

                                if( pRoad->paths[pIdx]->pedestCrossPoints[k]->crossPathID != memory.perceptedObjects[i]->objectPath ){
                                    continue;
                                }

                                memory.perceptedObjects[i]->hasCollisionPoint = true;

                                memory.perceptedObjects[i]->xCP = pRoad->paths[pIdx]->pedestCrossPoints[k]->pos.x();
                                memory.perceptedObjects[i]->yCP = pRoad->paths[pIdx]->pedestCrossPoints[k]->pos.y();

                                memory.perceptedObjects[i]->myCPPathIndex  = j;
                                memory.perceptedObjects[i]->objCPPathIndex = k;

                                break;
                            }

                            if( memory.perceptedObjects[i]->hasCollisionPoint == true ){
                                break;
                            }
                        }

//                        strForDebugRiskEval += QString("P%1 myIdx=%2 ObjIdx=%3\n").arg( memory.perceptedObjects[i]->objectID ).arg( memory.perceptedObjects[i]->myCPPathIndex ).arg( memory.perceptedObjects[i]->objCPPathIndex );


                        // Update distance to CP
                        if( memory.perceptedObjects[i]->myCPPathIndex >= 0 && memory.perceptedObjects[i]->objCPPathIndex >= 0 ){

                            int j = memory.perceptedObjects[i]->myCPPathIndex;
                            int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
                            int k = memory.perceptedObjects[i]->objCPPathIndex;

                            float myDist = 0.0;
                            if( memory.currentTargetPathIndexInList >= j ){
                                for(int l=memory.currentTargetPathIndexInList;l>=0;l--){
                                    if( l == j ){

                                        myDist += pRoad->paths[pIdx]->pedestCrossPoints[k]->distFromStartWP;
                                        break;
                                    }
                                    int tpIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[l] );
                                    myDist += pRoad->paths[tpIdx]->pathLength;
                                }
                                myDist -= memory.distanceFromStartWPInCurrentPath;
                            }
                            else{

                                myDist -= pRoad->paths[pIdx]->pedestCrossPoints[k]->distFromStartWP;

                                for(int l=j;l<memory.currentTargetPathIndexInList;l++){
                                    int tpIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[l] );
                                    myDist -= pRoad->paths[tpIdx]->pathLength;
                                }

                                myDist += memory.distanceFromStartWPInCurrentPath;
                            }

                            memory.perceptedObjects[i]->myDistanceToCP = myDist;
                            memory.perceptedObjects[i]->myTimeToCP = myDist / (state.V + 0.5);

                            int secIdx =  pRoad->paths[pIdx]->pedestCrossPoints[k]->sectionIndex;
                            float dx = memory.perceptedObjects[i]->xCP - memory.perceptedObjects[i]->x;
                            float dy = memory.perceptedObjects[i]->yCP - memory.perceptedObjects[i]->y;
                            float objDist = dx * (pRoad->pedestPaths[plIdx]->shape[secIdx]->cosA) + dy * (pRoad->pedestPaths[plIdx]->shape[secIdx]->sinA);
                            float objWidth = dx * (pRoad->pedestPaths[plIdx]->shape[secIdx]->sinA) * (-1.0) + dy * (pRoad->pedestPaths[plIdx]->shape[secIdx]->cosA);
                            if( objWidth > 4.5 ){
                                objDist += objWidth;
                            }
                            else if( objWidth < -4.5 ){
                                objDist -= objWidth;
                            }

                            memory.perceptedObjects[i]->objectDistanceToCP = objDist;
                            memory.perceptedObjects[i]->objectTimeToCP = objDist / (memory.perceptedObjects[i]->V + 0.1);


                            if( memory.perceptedObjects[i]->myDistanceToCP < 0.0 ){
                                memory.perceptedObjects[i]->hasCollisionPoint = false;
                            }
                        }
                    }

                    continue;
                }

                if( memory.perceptedObjects[i]->recognitionLabel < AGENT_RECOGNITION_LABEL::ONCOMING_STRAIGHT ){
                    memory.perceptedObjects[i]->hasCollisionPoint = false;
                    continue;
                }


                memory.perceptedObjects[i]->hasCollisionPoint = false;

                int objID = memory.perceptedObjects[i]->objectID;
                float distEvaled = 0.0;
                for(int j=memory.currentTargetPathIndexInList;j>=0;j--){
                    int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );

                    if( pRoad->paths[pIdx]->crossPoints.size() == 0 ){
                        distEvaled += pRoad->paths[pIdx]->pathLength;
                        if( distEvaled > 200.0 ){
                            break;
                        }
                        continue;
                    }
                    for(int k=0;k<pRoad->paths[pIdx]->crossPoints.size();++k){
                        int cpath = pRoad->paths[pIdx]->crossPoints[k]->crossPathID;
                        if( pAgent[objID]->memory_reference.targetPathList.size() > 0 &&
                                pAgent[objID]->memory_reference.targetPathList.indexOf( cpath ) >= 0 ){

                            memory.perceptedObjects[i]->hasCollisionPoint = true;
                            memory.perceptedObjects[i]->mergingAsCP       = false;

                            memory.perceptedObjects[i]->xCP = pRoad->paths[pIdx]->crossPoints[k]->pos.x();
                            memory.perceptedObjects[i]->yCP = pRoad->paths[pIdx]->crossPoints[k]->pos.y();
                            memory.perceptedObjects[i]->CPinNode = pRoad->paths[pIdx]->connectingNode;

                            memory.perceptedObjects[i]->myCPPathIndex = j;
                            memory.perceptedObjects[i]->objCPPathIndex = k;
                            memory.perceptedObjects[i]->objPathCPChecked = memory.perceptedObjects[i]->objectPath;

                            break;
                        }
                    }
                    distEvaled += pRoad->paths[pIdx]->pathLength;
                    if( distEvaled > 200.0 ){
                        break;
                    }
                    if( memory.perceptedObjects[i]->hasCollisionPoint == true ){
                        break;
                    }
                }

                if( memory.perceptedObjects[i]->hasCollisionPoint == false ){

                    // Check merging point as collision point; this is only for vehicles

                    QList<int> myEWPList;
                    float distEvaled = 0.0;
                    for(int j=memory.currentTargetPathIndexInList;j>=0;j--){
                        int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
                        myEWPList.append( pRoad->paths[pIdx]->endWpId );
                        distEvaled += pRoad->paths[pIdx]->pathLength;
                        if( distEvaled > 200.0 ){
                            break;
                        }
                    }

                    QList<int> objEWPList;
                    distEvaled = 0.0;
                    if( pAgent[objID]->memory_reference.currentTargetPathIndexInList >= 0 &&
                            pAgent[objID]->memory_reference.targetPathList.size() > 0 &&
                            pAgent[objID]->memory_reference.currentTargetPathIndexInList < pAgent[objID]->memory_reference.targetPathList.size() ){
                        for(int j=pAgent[objID]->memory_reference.currentTargetPathIndexInList;j>=0;j--){
                            int tpath = pAgent[objID]->memory_reference.targetPathList[j];
                            int pIdx = pRoad->pathId2Index.indexOf( tpath );
                            if( pIdx >= 0 ){
                                objEWPList.append( pRoad->paths[pIdx]->endWpId );
                                distEvaled += pRoad->paths[pIdx]->pathLength;
                                if( distEvaled > 200.0 ){
                                    break;
                                }
                            }

                        }
                    }


                    for(int j=0;j<myEWPList.size();++j){
                        for(int k=0;k<objEWPList.size();++k){
                            if( myEWPList[j] == objEWPList[k] ){

                                memory.perceptedObjects[i]->hasCollisionPoint = true;
                                memory.perceptedObjects[i]->mergingAsCP       = true;

                                int wIdx = pRoad->wpId2Index.indexOf( myEWPList[j] );
                                memory.perceptedObjects[i]->xCP = pRoad->wps[wIdx]->pos.x();
                                memory.perceptedObjects[i]->yCP = pRoad->wps[wIdx]->pos.y();
                                memory.perceptedObjects[i]->CPinNode = pRoad->wps[wIdx]->relatedNode;

                                memory.perceptedObjects[i]->myCPPathIndex  = myEWPList[j];
                                memory.perceptedObjects[i]->objCPPathIndex = objEWPList[k];
                                memory.perceptedObjects[i]->objPathCPChecked = memory.perceptedObjects[i]->objectPath;

                                break;
                            }
                        }
                        if( memory.perceptedObjects[i]->hasCollisionPoint == true ){
                            break;
                        }
                    }

                }


                // Update distance to CP
                if( memory.perceptedObjects[i]->mergingAsCP == false &&
                        memory.perceptedObjects[i]->myCPPathIndex >= 0 && memory.perceptedObjects[i]->objCPPathIndex >= 0 ){

                    int j = memory.perceptedObjects[i]->myCPPathIndex;
                    int k = memory.perceptedObjects[i]->objCPPathIndex;
                    int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
                    int objID = memory.perceptedObjects[i]->objectID;
                    int cpath = pRoad->paths[pIdx]->crossPoints[k]->crossPathID;

                    float myDist = 0.0;
                    for(int l=memory.currentTargetPathIndexInList;l>=0;l--){
                        if( l == j ){
                            myDist += pRoad->paths[pIdx]->crossPoints[k]->distFromStartWP;
                            break;
                        }
                        int tpIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[l] );
                        myDist += pRoad->paths[tpIdx]->pathLength;
                    }
                    myDist -= memory.distanceFromStartWPInCurrentPath;

                    memory.perceptedObjects[i]->myDistanceToCP = myDist;
                    memory.perceptedObjects[i]->myTimeToCP = myDist / (state.V + 0.5);

                    if( pAgent[objID]->memory_reference.currentTargetPathIndexInList >= 0 &&
                            pAgent[objID]->memory_reference.targetPathList.size() > 0 &&
                            pAgent[objID]->memory_reference.currentTargetPathIndexInList < pAgent[objID]->memory_reference.targetPathList.size() ){

                        float objDist = 0.0;

                        int ctpIndexList = pAgent[objID]->memory_reference.currentTargetPathIndexInList;
                        int mlIdx = pAgent[objID]->memory_reference.targetPathList.size()-1;
                        for(int l=ctpIndexList;l>=0;l--){
                            if( pAgent[objID]->memory_reference.targetPathList[l] == cpath ){
                                mlIdx = l;
                                break;
                            }
                        }

                        for(int l=ctpIndexList;l>=mlIdx;l--){
                            if( pAgent[objID]->memory_reference.targetPathList[l] == cpath ){
                                int tpIdx = pRoad->pathId2Index.indexOf( cpath );
                                for(int m=0;m<pRoad->paths[tpIdx]->crossPoints.size();++m){
                                    if( pRoad->paths[tpIdx]->crossPoints[m]->crossPathID == memory.targetPathList[j] ){
                                        objDist += pRoad->paths[tpIdx]->crossPoints[m]->distFromStartWP;
                                        break;
                                    }
                                }
                                break;
                            }
                            int tpath = pAgent[objID]->memory_reference.targetPathList[l];
                            int tpIdx = pRoad->pathId2Index.indexOf( tpath );
                            objDist += pRoad->paths[tpIdx]->pathLength;
                        }
                        objDist -= pAgent[objID]->memory_reference.distanceFromStartWPInCurrentPath;

                        memory.perceptedObjects[i]->objectDistanceToCP = objDist;
                        memory.perceptedObjects[i]->objectTimeToCP = objDist / ( memory.perceptedObjects[i]->V + 0.5);
                    }
                    else{
                        memory.perceptedObjects[i]->objectDistanceToCP = 0.0;
                        memory.perceptedObjects[i]->objectTimeToCP = 0.0;
                    }

                }
                else if( memory.perceptedObjects[i]->mergingAsCP == true &&
                        memory.perceptedObjects[i]->myCPPathIndex >= 0 && memory.perceptedObjects[i]->objCPPathIndex >= 0 ){

                    int objID = memory.perceptedObjects[i]->objectID;

                    float myDist = 0.0;
                    for(int l=memory.currentTargetPathIndexInList;l>=0;l--){
                        int tpIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[l] );
                        myDist += pRoad->paths[tpIdx]->pathLength;
                        if( pRoad->paths[tpIdx]->endWpId == memory.perceptedObjects[i]->myCPPathIndex ){
                            break;
                        }
                    }
                    myDist -= memory.distanceFromStartWPInCurrentPath;

                    memory.perceptedObjects[i]->myDistanceToCP = myDist;
                    memory.perceptedObjects[i]->myTimeToCP = myDist / (state.V + 0.5);

                    if( pAgent[objID]->memory_reference.currentTargetPathIndexInList >= 0 &&
                            pAgent[objID]->memory_reference.targetPathList.size() > 0 &&
                            pAgent[objID]->memory_reference.currentTargetPathIndexInList < pAgent[objID]->memory_reference.targetPathList.size() ){

                        float objDist = 0.0;
                        for(int l=pAgent[objID]->memory_reference.currentTargetPathIndexInList;l>=0;l--){
                            int tpIdx = pRoad->pathId2Index.indexOf( pAgent[objID]->memory_reference.targetPathList[l] );
                            objDist += pRoad->paths[tpIdx]->pathLength;
                            if( pRoad->paths[tpIdx]->endWpId == memory.perceptedObjects[i]->objCPPathIndex ){
                                break;
                            }
                        }
                        objDist -= pAgent[objID]->memory_reference.distanceFromStartWPInCurrentPath;

                        memory.perceptedObjects[i]->objectDistanceToCP = objDist;
                        memory.perceptedObjects[i]->objectTimeToCP = objDist / ( memory.perceptedObjects[i]->V + 0.5);
                    }
                    else{
                        memory.perceptedObjects[i]->objectDistanceToCP = 0.0;
                        memory.perceptedObjects[i]->objectTimeToCP = 0.0;
                    }

                }
            }


#ifdef _PERFORMANCE_CHECK_AGENT_HAZARD
            QueryPerformanceCounter(&end);
            calTime[0] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            calCount[0]++;
#endif


#ifdef _PERFORMANCE_CHECK_AGENT_HAZARD
            QueryPerformanceCounter(&start);
#endif

            // Distance to nearest CP
            memory.distToNearestCP = -1.0;
            memory.nearCPInNode = -1;

            bool existCPAhead = false;
            bool checkNextTurnNode = false;
            float distEvaled = 0.0;
            for(int j=memory.currentTargetPathIndexInList;j>=0;j--){
                int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
                if( memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING || memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
                    if( checkNextTurnNode == false && pRoad->paths[pIdx]->connectingNode == memory.nextTurnNode ){
                        checkNextTurnNode = true;
                    }
                    else if( checkNextTurnNode == true && pRoad->paths[pIdx]->connectingNode != memory.nextTurnNode ){
                        break;
                    }
                }
                if( pRoad->paths[pIdx]->crossPoints.size() > 0 ){
                    existCPAhead = true;
                    break;
                }
                distEvaled += pRoad->paths[pIdx]->pathLength;
                if( distEvaled > 200.0 ){
                    break;
                }
            }
            if( existCPAhead == true ){

                memory.distToNearestCP = 0.0;
                for(int j=memory.currentTargetPathIndexInList;j>=0;j--){
                    int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
                    if( pRoad->paths[pIdx]->crossPoints.size() == 0 ){
                        memory.distToNearestCP += pRoad->paths[pIdx]->pathLength;
                    }
                    else{
                        float dist = -1.0;
                        for(int k=0;k<pRoad->paths[pIdx]->crossPoints.size();++k){
                            if( dist < 0.0 || dist > pRoad->paths[pIdx]->crossPoints[k]->distFromStartWP ){
                                dist = pRoad->paths[pIdx]->crossPoints[k]->distFromStartWP;
                            }
                        }
                        memory.distToNearestCP += dist;
                        memory.distToNearestCP -= memory.distanceFromStartWPInCurrentPath;
                        memory.nearCPInNode = pRoad->paths[pIdx]->connectingNode;
                        break;
                    }
                }
            }
            else{

                if( memory.nextTurnNode == memory.currentTargetNode && memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING && pRoad->LeftOrRight == 0 ){

                    memory.distToNearestCP = 0.0;
                    for(int j=memory.currentTargetPathIndexInList;j>=0;j--){
                        int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
                        memory.distToNearestCP += pRoad->paths[pIdx]->pathLength;

                        int epIdx = pRoad->wpId2Index.indexOf( pRoad->paths[pIdx]->endWpId );
                        if( pRoad->wps[epIdx]->isNodeOutWP == true ){
                            memory.distToNearestCP -= memory.distanceFromStartWPInCurrentPath;
                            break;
                        }
                    }
                }
                else if( memory.nextTurnNode == memory.currentTargetNode && memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING && pRoad->LeftOrRight == 1 ){

                    memory.distToNearestCP = 0.0;
                    for(int j=memory.currentTargetPathIndexInList;j>=0;j--){
                        int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
                        memory.distToNearestCP += pRoad->paths[pIdx]->pathLength;

                        int epIdx = pRoad->wpId2Index.indexOf( pRoad->paths[pIdx]->endWpId );
                        if( pRoad->wps[epIdx]->isNodeOutWP == true ){
                            memory.distToNearestCP -= memory.distanceFromStartWPInCurrentPath;
                            break;
                        }
                    }
                }
            }

#ifdef _PERFORMANCE_CHECK_AGENT_HAZARD
            QueryPerformanceCounter(&end);
            calTime[1] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            calCount[1]++;
#endif

#ifdef _PERFORMANCE_CHECK_AGENT_HAZARD
            QueryPerformanceCounter(&start);
#endif

            // check to have to yeild the way
            memory.shouldYeild = false;
            memory.distToYeildStopLine = 0.0;
            distEvaled = 0.0;
            for(int i=memory.currentTargetPathIndexInList;i>=0;i--){

                int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[i] );
                if( pRoad->paths[pIdx]->stopPoints.size() == 0 ){
                    memory.distToYeildStopLine += pRoad->paths[pIdx]->pathLength;
                    distEvaled += pRoad->paths[pIdx]->pathLength;
                    if( distEvaled > 200.0 ){
                        break;
                    }
                    continue;
                }
                else{

                    int nIdx = pRoad->nodeId2Index.indexOf( pRoad->paths[pIdx]->connectingNode );
                    if( pRoad->nodes[nIdx]->hasTS == true ){
                        memory.distToYeildStopLine += pRoad->paths[pIdx]->pathLength;
                        distEvaled += pRoad->paths[pIdx]->pathLength;
                        if( distEvaled > 200.0 ){
                            break;
                        }
                        continue;
                    }
                    else{

                        float dist = -1.0;
                        for(int j=0;j<pRoad->paths[pIdx]->stopPoints.size();++j){
                            if( dist < 0.0 || dist > pRoad->paths[pIdx]->stopPoints[j]->distFromStartWP ){
                                dist = pRoad->paths[pIdx]->stopPoints[j]->distFromStartWP;
                            }
                        }

                        if( memory.distToYeildStopLine + dist - memory.distanceFromStartWPInCurrentPath > 200.0){
                            break;
                        }
                        else if( memory.distToYeildStopLine < -1.5 ){
                            break;
                        }

                        memory.shouldYeild = true;
                        memory.distToYeildStopLine += dist;
                        memory.distToYeildStopLine -= memory.distanceFromStartWPInCurrentPath;
                        break;
                    }
                }
            }


#ifdef _PERFORMANCE_CHECK_AGENT_HAZARD
            QueryPerformanceCounter(&end);
            calTime[2] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            calCount[2]++;
#endif
        }
    }


#ifdef _PERFORMANCE_CHECK_AGENT_HAZARD
    if( calCount[0] == 500 ){
        qDebug() << "Agent : ID = " << ID;
        for(int i=0;i<3;i++){
            calTime[i] /= calCount[i];
            qDebug() << "   Mean Time[" << i << "] = " << calTime[i];
            calCount[i] = 0;
        }
    }
#endif

}

