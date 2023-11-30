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

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION
#include <windows.h>
#endif



void Agent::Perception( Agent** pAgent, int maxAgent, Road* pRoad, QList<TrafficSignal*> trafficSignal )
{

    cognitionCount++;
    if( cognitionCount >= cognitionCountMax ){
        cognitionCount = 0;
    }
    else{
        return;
    }


    //
    //  Calculation of position on path
    //

    memory.lateralDeviationFromTargetPathAtPreviewPoint = 0.0;
    memory.previewPointPath = -1;


    if( agentKind < 100 && state.V > 0.1 ){

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION
        QueryPerformanceCounter(&start);
#endif

        memory.currentTargetPathIndexInList = -1;
        for(int i = memory.targetPathList.size()-1 ; i>=0 ; i--){
            if( memory.currentTargetPathIndexInList < 0 && memory.targetPathList[i] != memory.currentTargetPath ){
                continue;
            }
            memory.currentTargetPathIndexInList = i;
            break;
        }


        float D1 = state.V;
        float D2 = memory.actualTargetSpeed;
        float preview_dist = ( D1 < D2 ? D1: D2 );

        preview_dist *= memory.aimPointFactorForExternalControl;

        if( preview_dist < vHalfLength ){
            preview_dist = vHalfLength;
        }

        float preview_x = state.x + preview_dist * state.cosYaw;
        float preview_y = state.y + preview_dist * state.sinYaw;

        for(int i=memory.currentTargetPathIndexInList;i>=0;i--){

            float tdev,txt,tyt,txd,tyd,ts;

            if( i == memory.currentTargetPathIndexInList ){
                int chk = pRoad->GetDeviationFromPath( memory.targetPathList[i],
                                                       state.x, state.y, state.yaw,
                                                       tdev, txt, tyt, txd, tyd, ts );

                if( chk == memory.targetPathList[i] ){
                    memory.lateralDeviationFromTargetPath = tdev;
                    memory.distanceFromStartWPInCurrentPath = ts;

                    memory.relativeAttitudeToLane = txd * state.sinYaw - tyd * state.cosYaw;
                }
            }

            int chk = pRoad->GetDeviationFromPath( memory.targetPathList[i],
                                                   preview_x, preview_y, state.yaw,
                                                   tdev, txt, tyt, txd, tyd, ts, true );

    //        qDebug() << "i=" << i << " chk=" << chk << " dev=" << tdev;

            if( chk != memory.targetPathList[i] ){

                if( i == 0 && memory.currentTargetPathIndexInList == 0 ){

                    chk = pRoad->GetDeviationFromPathExtendEnd( memory.targetPathList[i],
                                                               preview_x, preview_y, state.yaw,
                                                               tdev, txt, tyt, txd, tyd, ts, true );

                    if( chk != memory.targetPathList[i] ){
                        memory.lateralDeviationFromTargetPathAtPreviewPoint = 0.0;
                        continue;
                    }
                }
                else{
                	continue;
            	}
            }
            if( isnan(tdev) == true ){
                memory.lateralDeviationFromTargetPathAtPreviewPoint = 0.0;
                break;
            }

            memory.lateralDeviationFromTargetPathAtPreviewPoint = tdev;
            memory.previewPointPath = memory.targetPathList[i];

            break;
        }


#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION
        QueryPerformanceCounter(&end);
        calTime[0] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
        calCount[0]++;
#endif



        memory.distanceToTurnNodeWPIn = -1.0;
        memory.distanceToNodeWPIn     = -1.0;

        memory.distanceToCurrentTargetNodeWPIn = -1.0;
        memory.distanceToCurrentTargetNodeWPOut = -1.0;

        memory.distanceToTurnNodeWPOut = -1.0;
        memory.distanceToNodeWPOut     = -1.0;
        memory.turnNodeOutWP           = -1;

        // only for agent with node list route data
        if( memory.myNodeList.size() > 1 ){

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION
            QueryPerformanceCounter(&start);
#endif

            if( memory.nextTurnNode >= 0 ){

                QList<int> destPathsIn;
                QList<int> destPathsOut;

                int inDir = memory.myInDirList.at( memory.nextTurnNodeIndexInNodeList );
                int outDir = memory.myOutDirList.at( memory.nextTurnNodeIndexInNodeList );

                int tnIdx = pRoad->nodeId2Index.indexOf( memory.nextTurnNode );
                for(int i=0;i<pRoad->nodes[tnIdx]->inBoundaryWPs.size();++i){
                    if( pRoad->nodes[tnIdx]->inBoundaryWPs[i]->relatedDirection == inDir ){
                        for(int j=0;j<pRoad->nodes[tnIdx]->inBoundaryWPs[i]->PathWithEWP.size();++j){
                            destPathsIn.append( pRoad->nodes[tnIdx]->inBoundaryWPs[i]->PathWithEWP[j] );
                        }
                    }
                }
                for(int i=0;i<pRoad->nodes[tnIdx]->outBoundaryWPs.size();++i){
                    if( pRoad->nodes[tnIdx]->outBoundaryWPs[i]->relatedDirection == outDir ){
                        for(int j=0;j<pRoad->nodes[tnIdx]->outBoundaryWPs[i]->PathWithEWP.size();++j){
                            destPathsOut.append( pRoad->nodes[tnIdx]->outBoundaryWPs[i]->PathWithEWP[j] );
                        }
                    }
                }

                float dist = 0.0;
                bool foundWPIn = false;
                for(int i=memory.currentTargetPathIndexInList;i>=0;i--){
                    int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList.at(i) );
                    dist += pRoad->paths[pIdx]->pathLength;
                    if( pRoad->paths[pIdx]->connectingNode == memory.nextTurnNode ){
                        if( destPathsIn.indexOf(memory.targetPathList.at(i)) >= 0 ){
                            foundWPIn = true;
                            memory.distanceToTurnNodeWPIn = dist - memory.distanceFromStartWPInCurrentPath;
                        }
                        if( destPathsOut.indexOf(memory.targetPathList.at(i)) >= 0 ){
                            memory.distanceToTurnNodeWPOut = dist - memory.distanceFromStartWPInCurrentPath;
                            int tpid = pRoad->pathId2Index.indexOf(memory.targetPathList.at(i));
                            memory.turnNodeOutWP = pRoad->paths[tpid]->endWpId;
                            break;
                        }
                    }
                }

                if( foundWPIn == false ){
                    dist = 0.0;
                    for(int i=memory.currentTargetPathIndexInList+1;i<memory.targetPathList.size();i++){
                        int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList.at(i) );
                        if( pRoad->paths[pIdx]->connectingNode == memory.nextTurnNode ){
                            if( destPathsIn.indexOf(memory.targetPathList.at(i)) >= 0 ){
                                dist -= memory.distanceFromStartWPInCurrentPath;
                                foundWPIn = true;
                                break;
                            }
                            else{
                                dist += pRoad->paths[pIdx]->pathLength;
                            }
                        }
                    }
                    if( foundWPIn == true ){
                        memory.distanceToTurnNodeWPIn = dist;
                    }
                }
            }


    #ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION
            QueryPerformanceCounter(&end);
            calTime[1] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            calCount[1]++;
    #endif



    #ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION
            QueryPerformanceCounter(&start);
    #endif

            if( memory.currentTargetNodeIndexInNodeList >= 0 &&
                    memory.currentTargetNodeIndexInNodeList < memory.myNodeList.size() &&
                    memory.currentTargetNodeIndexInNodeList < memory.myInDirList.size() &&
                    memory.currentTargetNodeIndexInNodeList < memory.myOutDirList.size() ){

                QList<int> destPathsIn;
                QList<int> destPathsOut;

                int inDir = memory.myInDirList.at( memory.currentTargetNodeIndexInNodeList );
                int outDir = memory.myOutDirList.at( memory.currentTargetNodeIndexInNodeList );

                int tnIdx = pRoad->nodeId2Index.indexOf( memory.currentTargetNode );
                for(int i=0;i<pRoad->nodes[tnIdx]->inBoundaryWPs.size();++i){
                    if( pRoad->nodes[tnIdx]->inBoundaryWPs[i]->relatedDirection == inDir ){
                        for(int j=0;j<pRoad->nodes[tnIdx]->inBoundaryWPs[i]->PathWithEWP.size();++j){
                            destPathsIn.append( pRoad->nodes[tnIdx]->inBoundaryWPs[i]->PathWithEWP[j] );
                        }
                    }
                }
                for(int i=0;i<pRoad->nodes[tnIdx]->outBoundaryWPs.size();++i){
                    if( pRoad->nodes[tnIdx]->outBoundaryWPs[i]->relatedDirection == outDir ){
                        for(int j=0;j<pRoad->nodes[tnIdx]->outBoundaryWPs[i]->PathWithEWP.size();++j){
                            destPathsOut.append( pRoad->nodes[tnIdx]->outBoundaryWPs[i]->PathWithEWP[j] );
                        }
                    }
                }

                float dist = 0.0;
                bool foundWPIn  = false;
                for(int i=memory.currentTargetPathIndexInList;i>=0;i--){
                    int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList.at(i) );
                    dist += pRoad->paths[pIdx]->pathLength;
                    if( pRoad->paths[pIdx]->connectingNode == memory.currentTargetNode ){
                        if( destPathsIn.indexOf(memory.targetPathList.at(i)) >= 0 ){
                            memory.distanceToCurrentTargetNodeWPIn = dist - memory.distanceFromStartWPInCurrentPath;
                            foundWPIn = true;
                            memory.currentWPInPos.setX( pRoad->paths[pIdx]->pos.last()->x() );
                            memory.currentWPInPos.setY( pRoad->paths[pIdx]->pos.last()->y() );
                        }
                        if( destPathsOut.indexOf(memory.targetPathList.at(i)) >= 0 ){
                            memory.distanceToCurrentTargetNodeWPOut = dist - memory.distanceFromStartWPInCurrentPath;
                            memory.currentWPOutPos.setX( pRoad->paths[pIdx]->pos.last()->x() );
                            memory.currentWPOutPos.setY( pRoad->paths[pIdx]->pos.last()->y() );
                            break;
                        }
                    }
                }
                if( foundWPIn == false ){
                    dist = 0.0;
                    for(int i=memory.currentTargetPathIndexInList+1;i<memory.targetPathList.size();i++){
                        int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList.at(i) );
                        if( pRoad->paths[pIdx]->connectingNode == memory.currentTargetNode ){
                            if( destPathsIn.indexOf(memory.targetPathList.at(i)) >= 0 ){
                                dist -= memory.distanceFromStartWPInCurrentPath;
                                foundWPIn = true;
                                memory.currentWPInPos.setX( pRoad->paths[pIdx]->pos.last()->x() );
                                memory.currentWPInPos.setY( pRoad->paths[pIdx]->pos.last()->y() );
                                break;
                            }
                            else{
                                dist -= pRoad->paths[pIdx]->pathLength;
                            }
                        }
                    }
                    if( foundWPIn == true ){
                        memory.distanceToCurrentTargetNodeWPIn = dist;
                    }
                }
            }

            if( memory.currentTargetNode != memory.nextTurnNode ){
                memory.distanceToNodeWPIn = memory.distanceToCurrentTargetNodeWPIn;
                memory.distanceToNodeWPOut = memory.distanceToCurrentTargetNodeWPOut;
            }
            else{
                memory.distanceToNodeWPIn  = memory.distanceToTurnNodeWPIn;
                memory.distanceToNodeWPOut = memory.distanceToTurnNodeWPOut;
            }

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION
            QueryPerformanceCounter(&end);
            calTime[2] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            calCount[2]++;
#endif


        }


        //
        // For Lane-Change
        if( memory.checkSideVehicleForLC == true && memory.laneChangeTargetPathList.size() > 0 ){

            float tdev,txt,tyt,txd,tyd,ts;
            int currentPath = pRoad->GetNearestPathFromList( memory.laneChangeTargetPathList,
                                                             state.x,
                                                             state.y,
                                                             state.z_path,
                                                             state.yaw,
                                                             tdev, txt, tyt, txd, tyd, ts);

            if( currentPath >= 0 ){
                memory.currentPathInLCTargetPathList = currentPath;
                memory.latDeviFromLCTargetPathList   = tdev;
                memory.distFromSWPLCTargetPathList   = ts;
            }
        }

    }
    else if( agentKind >= 100 && state.V > 0.01 ){

        float  dev = 0.0;
        float dist = 0.0;
        float xdir = 0.0;
        float ydir = 0.0;
        int secIndx = pRoad->GetDeviationFromPedestPathAllSection( memory.currentTargetPath,
                                                                   state.x,
                                                                   state.y,
                                                                   dev,
                                                                   dist,
                                                                   xdir,
                                                                   ydir);
        if( secIndx >= 0 ){
            memory.currentTargetPathIndexInList = secIndx;
            memory.lateralDeviationFromTargetPath = dev;
            memory.distanceFromStartWPInCurrentPath = dist;
        }
    }


    strForDebug = QString("");


    bool isNearIntersection = false;
    if( (memory.distanceToNodeWPIn < 30.0 ||
         memory.distanceToNodeWPIn < memory.requiredDistToStopFromTargetSpeed ||
         memory.distanceToNodeWPIn < 3.0 * state.V) &&
            memory.checkSideVehicleForLC == false ){
        isNearIntersection = true;
    }

    strForDebug += QString("isNearIntersection=%1\n").arg(isNearIntersection);

    onlyCheckPreceding = false;
    if( isNearIntersection == false ){

        // If not near intersection, reduce sampling rate to 5[Hz] to speed up calculation
        if( cognitionSubCount == 0 ){
            cognitionSubCount++;
            return;
        }
        cognitionSubCount = 0;

        if( state.V < 0.1 && memory.precedingVehicleID >= 0 && memory.speedPrecedingVehicle < 0.1 ){
            float D = -1.0;
            int PV = ID;
            for(int i=0;i<memory.perceptedObjects.size();++i){
                if( memory.perceptedObjects[i]->isValidData == false ){
                    continue;
                }
                if( memory.perceptedObjects[i]->objectID == memory.precedingVehicleID &&
                        memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::PRECEDING ){
                    D = memory.perceptedObjects[i]->distanceToObject;
                    PV = pAgent[memory.precedingVehicleID]->memory_reference.precedingVehicleID;
                    break;
                }
            }
            if( D > 0.0 && PV != ID ){
                onlyCheckPreceding = true;
            }
        }
    }

    strForDebug += QString("onlyCheckPreceding=%1\n").arg(onlyCheckPreceding);





    //
    //  Perception
    //
    float VD2 = param.visibleDistance * param.visibleDistance;


#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION
    QueryPerformanceCounter(&start);
#endif



    // Agents
    for(int j=0;j<memory.perceptedObjects.size();++j){
        if( memory.perceptedObjects[j]->isValidData == true ){

            // Set inView = false.
            // If the object is in view, inView will be set true later.
            memory.perceptedObjects[j]->inView = false;
            memory.perceptedObjects[j]->noUpdateCount++;
        }
    }


    int lastAgentIndex = 0;
    for(int i=maxAgent-1;i>=0;--i){
        if( pAgent[i]->agentStatus == 1 ){
            lastAgentIndex = i;
            break;
        }
    }


    bool lookingBack = false;
    if( memory.checkSideVehicleForLC == true ){
        lookingBack = true;
    }


    for(int i=0;i<=lastAgentIndex;++i){

        if( i == ID ){
            continue;
        }

            if( pAgent[i]->agentStatus == 0 ){
            continue;
        }

        bool thisIsPreceding = false;
        if( onlyCheckPreceding == true ){
            if( pAgent[i]->ID != memory.precedingVehicleID ){
                continue;
            }
            else{
                thisIsPreceding = true;
            }
        }


        // check the object is in agent view
        float rx = pAgent[i]->state.x - state.x;
        if( rx < -param.visibleDistance || rx > param.visibleDistance ){
            continue;
        }

        float ry = pAgent[i]->state.y - state.y;
        if( ry < -param.visibleDistance || ry > param.visibleDistance ){
            continue;
        }

        float shortestDistance = rx * rx + ry * ry;
        if( shortestDistance > VD2 ){
            continue;
        }

        shortestDistance = sqrt(shortestDistance);


        if( thisIsPreceding == false ){

            // If vehicle is not near intersection, no need to see far side vehicles
            if( agentKind < 100 && isNearIntersection == false ){

                float RY = rx * state.sinYaw - ry * state.cosYaw;
                if( fabs(RY) > 10.0 ){
                    continue;
                }
            }

            // If driver agent is looking forward, following vehicles will not be extracted
            if( agentKind < 100 && lookingBack == false ){
                float RX = rx * state.cosYaw + ry * state.sinYaw;
                if( RX < -20.0 ){
                    continue;
                }
            }


            float dz = pAgent[i]->state.z_path - state.z_path;
            if( fabs(dz) > 5.0 ){
                continue;
            }


            if( agentKind < 100 &&
                    pAgent[i]->isSInterfaceObject == false && pAgent[i]->agentKind < 100 &&
                    memory.currentTargetNodeIndexInNodeList >= 0 &&
                    memory.currentTargetNodeIndexInNodeList < memory.myNodeList.size() ){
                // Check route data to have cross node
                bool hasCrossNode = false;
                for(int j=memory.currentTargetNodeIndexInNodeList;j<memory.myNodeList.size();++j){
                    int tNd = memory.myNodeList[j];
                    if( pAgent[i]->memory_reference.myNodeList.indexOf( tNd ) >= pAgent[i]->memory_reference.currentTargetNodeIndexInNodeList ){
                        hasCrossNode = true;
                        break;
                    }
                }
                if( hasCrossNode == false ){
                    continue;
                }
            }
        }

        int alreadyPercepted = -1;
        for(int j=0;j<memory.perceptedObjects.size();++j){
            if( memory.perceptedObjects[j]->isValidData == false ){
                continue;
            }
            if( memory.perceptedObjects[j]->objectID == pAgent[i]->ID ){
                alreadyPercepted = j;
                break;
            }
        }

        if( alreadyPercepted >= 0 ){

            memory.perceptedObjects[alreadyPercepted]->inView = true;
            memory.perceptedObjects[alreadyPercepted]->noUpdateCount = 0;

            memory.perceptedObjects[alreadyPercepted]->objectType  = pAgent[i]->agentKind;
            memory.perceptedObjects[alreadyPercepted]->vHalfLength = pAgent[i]->vHalfLength;
            memory.perceptedObjects[alreadyPercepted]->vHalfWidth  = pAgent[i]->vHalfWidth;

            memory.perceptedObjects[alreadyPercepted]->x       = pAgent[i]->state.x;
            memory.perceptedObjects[alreadyPercepted]->y       = pAgent[i]->state.y;
            memory.perceptedObjects[alreadyPercepted]->yaw     = pAgent[i]->state.yaw;
            memory.perceptedObjects[alreadyPercepted]->cos_yaw = pAgent[i]->state.cosYaw;
            memory.perceptedObjects[alreadyPercepted]->sin_yaw = pAgent[i]->state.sinYaw;
            memory.perceptedObjects[alreadyPercepted]->V       = pAgent[i]->state.V;
            memory.perceptedObjects[alreadyPercepted]->Ax      = pAgent[i]->state.accel - pAgent[i]->state.brake;
            memory.perceptedObjects[alreadyPercepted]->winker  = pAgent[i]->vehicle.GetWinerState();


            for(int mm=9;mm>0;mm--){
                memory.perceptedObjects[alreadyPercepted]->pastV[mm] = memory.perceptedObjects[alreadyPercepted]->pastV[mm-1];
            }
            memory.perceptedObjects[alreadyPercepted]->pastV[0] = memory.perceptedObjects[alreadyPercepted]->V;
            memory.perceptedObjects[alreadyPercepted]->filteredV = 0.0;
            for(int mm=0;mm<10;++mm){
                memory.perceptedObjects[alreadyPercepted]->filteredV += memory.perceptedObjects[alreadyPercepted]->pastV[mm];
            }
            memory.perceptedObjects[alreadyPercepted]->filteredV *= 0.1;


            if( thisIsPreceding == false ){
                if( pAgent[i]->state.V < 0.1 && state.V < 0.1 ){
                    // If both vehicle is stopping, no need to update
                    continue;
                }
            }


            memory.perceptedObjects[alreadyPercepted]->objectPath              = pAgent[i]->memory_reference.currentTargetPath;
            memory.perceptedObjects[alreadyPercepted]->objectTargetNode        = pAgent[i]->memory_reference.currentTargetNode;
            memory.perceptedObjects[alreadyPercepted]->deviationFromObjectPath = pAgent[i]->memory_reference.lateralDeviationFromTargetPath;

//            memory.perceptedObjects[alreadyPercepted]->nearestTargetPath              = -1;
            memory.perceptedObjects[alreadyPercepted]->deviationFromNearestTargetPath = 0.0;
            memory.perceptedObjects[alreadyPercepted]->distanceToObject               = 0.0;

            memory.perceptedObjects[alreadyPercepted]->objPathInLCTargetPathList = -1;
            memory.perceptedObjects[alreadyPercepted]->latDevObjInLaneChangeTargetPathList = 0.0;
            memory.perceptedObjects[alreadyPercepted]->distToObjInLaneChangeTargetPathList = 0.0;

            memory.perceptedObjects[alreadyPercepted]->relPosEvaled = false;

            if( agentKind < 100 ){

                int objPathIdx = -1;
                bool checkForward = true;
                bool checkBackward = true;

                if( memory.perceptedObjects[alreadyPercepted]->objectType < 100 ){

//                    if( memory.perceptedObjects[alreadyPercepted]->nearestTargetPath >= 0 ){

//                        objPathIdx = memory.targetPathList.indexOf( memory.perceptedObjects[alreadyPercepted]->nearestTargetPath );

//                        if( objPathIdx >= 0 && objPathIdx > memory.currentTargetPathIndexInList ){
//                            objPathIdx = -1;
//                        }

//                        if( objPathIdx < 0 ){
                            memory.perceptedObjects[alreadyPercepted]->nearestTargetPath = -1;
//                        }
//                    }


                    if( memory.targetPathList.indexOf( memory.perceptedObjects[alreadyPercepted]->objectPath) >= 0 ){
                        objPathIdx = memory.targetPathList.indexOf( memory.perceptedObjects[alreadyPercepted]->objectPath);
                    }

                }
                else if( memory.perceptedObjects[alreadyPercepted]->objectType >= 100 ){
                    checkBackward = false;
                }


                if( objPathIdx >= 0 && objPathIdx <= memory.currentTargetPathIndexInList ){
                    checkBackward = false;
                }
                else if( objPathIdx >= 0 && objPathIdx > memory.currentTargetPathIndexInList ){
                    checkForward = false;
                }


                int latMinIndex = -1;
                float latMin = 0.0;
                float txtMin = 0.0;
                float tytMin = 0.0;
                float txdMin = 0.0;
                float tydMin = 0.0;
                float tsMin  = 0.0;

                float checkDistance = 0.0;

//                strForDebug += QString("V%1 objPathIdx=%2 checkForward=%3 checkBackward=%4 \n")
//                        .arg(memory.perceptedObjects[alreadyPercepted]->objectID)
//                        .arg(objPathIdx).arg(checkForward).arg(checkBackward);


                if( objPathIdx >= 0 ){

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                    QueryPerformanceCounter(&start);
#endif

                    // Check if the object remains the same path
                    float tdev,txt,tyt,txd,tyd,ts;
                    int chk = pRoad->GetDeviationFromPathWithZ( memory.targetPathList[objPathIdx],
                                                                pAgent[i]->state.x, pAgent[i]->state.y, pAgent[i]->state.z_path,
                                                                pAgent[i]->state.yaw,
                                                                tdev, txt, tyt, txd, tyd, ts,
                                                                true );

                    if( chk == memory.targetPathList[objPathIdx] ){

                        latMinIndex = objPathIdx;
                        latMin = tdev;
                        txtMin = txt;
                        tytMin = tyt;
                        txdMin = txd;
                        tydMin = tyd;
                        tsMin  = ts;

//                        checkForward  = false;
//                        checkBackward = false;
                    }
//                    else{
//                        checkForward  = true;
//                        checkBackward = true;
//                    }

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                    QueryPerformanceCounter(&end);
                    calTime[0] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                    calCount[0]++;
#endif
                }


                if( shortestDistance < 150.0 ){
                    checkForward  = true;
                    checkBackward = true;
                }



#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                QueryPerformanceCounter(&start);
#endif
                if( checkForward == true ){

                    int nEval = 0;
                    for(int k=memory.currentTargetPathIndexInList;k>=0;--k){   // Forward Check

                        if( k == objPathIdx ){
                            continue;
                        }

                        float tdev,txt,tyt,txd,tyd,ts;
                        int chk = pRoad->GetDeviationFromPathWithZ( memory.targetPathList[k],
                                                                    pAgent[i]->state.x, pAgent[i]->state.y, pAgent[i]->state.z_path,
                                                                    pAgent[i]->state.yaw,
                                                                    tdev, txt, tyt, txd, tyd, ts,
                                                                    true );

//                        strForDebug += QString("check %1 ret = %2\n")
//                                .arg(memory.targetPathList[k])
//                                .arg(chk);

                        if( chk != memory.targetPathList[k] ){

                            //checkDistance += memory.targetPathLength[k];
                            int cpIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[k] );
                            float dXpt = pRoad->paths[cpIdx]->pos.last()->x() - state.x;
                            float dYpt = pRoad->paths[cpIdx]->pos.last()->y() - state.y;
                            float distToPathTop = sqrt( dXpt * dXpt + dYpt * dYpt );

                            if( nEval > 0 && /*checkDistance > param.visibleDistance*/ distToPathTop > param.visibleDistance ){
                                break;
                            }
                            nEval++;
                            continue;
                        }

                        if( latMinIndex < 0 || fabs(latMin) > fabs(tdev) ){
                            latMinIndex = k;
                            latMin = tdev;
                            txtMin = txt;
                            tytMin = tyt;
                            txdMin = txd;
                            tydMin = tyd;
                            tsMin  = ts;

                            if( fabs(tdev) < 3.0 ){
                                checkBackward = false;
                            }
                        }

                        checkDistance += memory.targetPathLength[k];
                        if( checkDistance > param.visibleDistance ){
                            break;
                        }
                    }
                }

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                QueryPerformanceCounter(&end);
                calTime[1] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                calCount[1]++;
#endif

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                QueryPerformanceCounter(&start);
#endif

                if( checkBackward == true ){
                    checkDistance = 0.0;
                    int nEval = 0;
                    for(int k=memory.currentTargetPathIndexInList+1;k<memory.targetPathList.size();++k){   // Backward Check

                        if( k == objPathIdx ){
                            continue;
                        }

                        float tdev,txt,tyt,txd,tyd,ts;
                        int chk = pRoad->GetDeviationFromPathWithZ( memory.targetPathList[k],
                                                                    pAgent[i]->state.x, pAgent[i]->state.y, pAgent[i]->state.z_path,
                                                                    pAgent[i]->state.yaw,
                                                                    tdev, txt, tyt, txd, tyd, ts,
                                                                    true );

                        if( chk != memory.targetPathList[k] ){
                            checkDistance += memory.targetPathLength[k];
                            if( nEval > 0 && checkDistance > param.visibleDistance ){
                                break;
                            }
                            nEval++;
                            continue;
                        }

                        if( latMinIndex < 0 || fabs(latMin) > fabs(tdev) ){
                            latMinIndex = k;
                            latMin = tdev;
                            txtMin = txt;
                            tytMin = tyt;
                            txdMin = txd;
                            tydMin = tyd;
                            tsMin  = ts;
                        }

                        checkDistance += memory.targetPathLength[k];
                        if( checkDistance > param.visibleDistance ){
                            break;
                        }
                    }
                }

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                QueryPerformanceCounter(&end);
                calTime[2] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                calCount[2]++;
#endif


#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                QueryPerformanceCounter(&start);
#endif

                if( latMinIndex >= 0 ){

                    int k = latMinIndex;

                    AgentPerception *ap = memory.perceptedObjects[alreadyPercepted];
                    ap->nearestTargetPath              = memory.targetPathList[k];
                    ap->deviationFromNearestTargetPath = latMin;
                    ap->xOnTargetPath                  = txtMin;
                    ap->yOnTargetPath                  = tytMin;

                    ap->innerProductToNearestPathTangent = txdMin * (pAgent[i]->state.cosYaw) + tydMin * (pAgent[i]->state.sinYaw);
                    ap->innerProductToNearestPathNormal  = (-tydMin) * ap->cos_yaw + txdMin * ap->sin_yaw;
                    ap->effectiveHalfWidth = fabs(ap->innerProductToNearestPathTangent) * ap->vHalfWidth + fabs(ap->innerProductToNearestPathNormal) * ap->vHalfLength;


                    float dist = 0.0;

                    bool foundMe = false;
                    bool foundObj = false;
                    float distMe = 0.0;
                    float distObj = 0.0;

                    for(int l=memory.targetPathList.size()-1;l>=0;l--){

                        if( memory.targetPathList[l] == memory.currentTargetPath ){
                            foundMe = true;
                        }

                        if( l == k ){
                            foundObj = true;
                        }

                        if(foundMe == false && foundObj == false ){
                            continue;
                        }

                        float pLen = memory.targetPathLength[l];
                        if( foundMe == true ){
                            distMe += pLen;
                        }
                        if( foundObj == true ){
                            distObj += pLen;
                        }

                        if( foundMe == true && foundObj == true ){
                            break;
                        }
                    }

                    distMe -= memory.distanceFromStartWPInCurrentPath;
                    distObj -= tsMin;

                    dist = distMe - distObj;

                    memory.perceptedObjects[alreadyPercepted]->objDistFromSWPOfNearTargetPath = tsMin;
                    memory.perceptedObjects[alreadyPercepted]->distanceToObject = dist;

                    memory.perceptedObjects[alreadyPercepted]->relPosEvaled = true;

    //                qDebug() << "distanceToObject=" << memory.perceptedObjects[alreadyPercepted]->distanceToObject;
                }


                if( memory.checkSideVehicleForLC == true && memory.laneChangeTargetPathList.size() > 0 ){

                    float tdev,txt,tyt,txd,tyd,ts;
                    int objPathInLCPath = pRoad->GetNearestPathFromList( memory.laneChangeTargetPathList,
                                                                         pAgent[i]->state.x,
                                                                         pAgent[i]->state.y,
                                                                         pAgent[i]->state.z_path,
                                                                         pAgent[i]->state.yaw,
                                                                         tdev, txt, tyt, txd, tyd, ts);

                    if( objPathInLCPath >= 0 ){
                        memory.perceptedObjects[alreadyPercepted]->objPathInLCTargetPathList = objPathInLCPath;
                        memory.perceptedObjects[alreadyPercepted]->latDevObjInLaneChangeTargetPathList = tdev;

                        bool foundMe = false;
                        bool foundObj = false;
                        float distMe = 0.0;
                        float distObj = 0.0;
                        for(int l=memory.laneChangeTargetPathList.size()-1;l>=0;l--){

                            if( memory.laneChangeTargetPathList[l] == memory.currentPathInLCTargetPathList ){
                                foundMe = true;
                            }

                            if( memory.laneChangeTargetPathList[l] == objPathInLCPath  ){
                                foundObj = true;
                            }

                            if(foundMe == false && foundObj == false ){
                                continue;
                            }

                            float pLen = memory.laneChangePathLength[l];
                            if( foundMe == true ){
                                distMe += pLen;
                            }
                            if( foundObj == true ){
                                distObj += pLen;
                            }

                            if( foundMe == true && foundObj == true ){
                                break;
                            }
                        }

                        distMe -= memory.distFromSWPLCTargetPathList;
                        distObj -= ts;

                        memory.perceptedObjects[alreadyPercepted]->distToObjInLaneChangeTargetPathList = distMe - distObj;
                    }

                }


#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                QueryPerformanceCounter(&end);
                calTime[3] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                calCount[3]++;
#endif
            }
            else if( agentKind >= 100 ){

                float  dev = 0.0;
                float dist = 0.0;
                float xdir = 1.0;
                float ydir = 0.0;
                int secIndx = pRoad->GetDeviationFromPedestPathAllSection( memory.currentTargetPath,
                                                                           memory.perceptedObjects[alreadyPercepted]->x,
                                                                           memory.perceptedObjects[alreadyPercepted]->y,
                                                                           dev,
                                                                           dist,
                                                                           xdir,
                                                                           ydir);

                if( secIndx >= 0 ){

                    memory.perceptedObjects[alreadyPercepted]->nearestTargetPath = secIndx;
                    memory.perceptedObjects[alreadyPercepted]->deviationFromNearestTargetPath = dev;
                    memory.perceptedObjects[alreadyPercepted]->distanceToObject = dist - memory.distanceFromStartWPInCurrentPath;

                    memory.perceptedObjects[alreadyPercepted]->relPosEvaled = true;
                }

                memory.perceptedObjects[alreadyPercepted]->innerProductToNearestPathTangent = xdir * pAgent[i]->state.cosYaw + ydir * pAgent[i]->state.sinYaw;
                memory.perceptedObjects[alreadyPercepted]->innerProductToNearestPathNormal = xdir * (-pAgent[i]->state.sinYaw) + ydir * pAgent[i]->state.cosYaw;
                memory.perceptedObjects[alreadyPercepted]->effectiveHalfWidth = fabs(memory.perceptedObjects[alreadyPercepted]->innerProductToNearestPathNormal) * pAgent[i]->vHalfLength + fabs(memory.perceptedObjects[alreadyPercepted]->innerProductToNearestPathTangent) * pAgent[i]->vHalfWidth;

            }


        }
        else{


#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
            QueryPerformanceCounter(&start);
#endif

            int sIndex = -1;
            for(int j=0;j<memory.perceptedObjects.size();++j){
                if( memory.perceptedObjects[j]->isValidData == false ){
                    sIndex = j;
                    break;
                }
            }
            if( sIndex < 0 ){
                struct AgentPerception *ap = new struct AgentPerception;
                memory.perceptedObjects.append( ap );
                sIndex = memory.perceptedObjects.size() - 1;
            }

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
            QueryPerformanceCounter(&end);
            calTime[4] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            calCount[4]++;
#endif

            struct AgentPerception *ap = memory.perceptedObjects[sIndex];

            ap->isValidData = true;

            ap->inView = true;
            ap->noUpdateCount = 0;

            ap->objectID   = pAgent[i]->ID;
            ap->objectType = pAgent[i]->agentKind;

            ap->recognitionLabel = AGENT_RECOGNITION_LABEL::UNDEFINED_RECOGNITION_LABEL;
            if( ap->objectType >= 100 ){
                ap->recognitionLabel = AGENT_RECOGNITION_LABEL::PEDESTRIAN;
            }

            ap->vHalfLength  = pAgent[i]->vHalfLength;
            ap->vHalfWidth   = pAgent[i]->vHalfWidth;

            ap->x       = pAgent[i]->state.x;
            ap->y       = pAgent[i]->state.y;
            ap->yaw     = pAgent[i]->state.yaw;
            ap->cos_yaw = pAgent[i]->state.cosYaw;
            ap->sin_yaw = pAgent[i]->state.sinYaw;
            ap->V       = pAgent[i]->state.V;
            ap->Ax      = pAgent[i]->state.accel - pAgent[i]->state.brake;
            ap->winker  = pAgent[i]->vehicle.GetWinerState();


            for(int mm=9;mm>0;mm--){
                ap->pastV[mm] = 0.0;
            }
            ap->pastV[0] = ap->V;
            ap->filteredV = 0.0;
            for(int mm=0;mm<10;++mm){
                ap->filteredV += ap->pastV[mm];
            }
            ap->filteredV *= 0.1;


            ap->objectPath              = pAgent[i]->memory_reference.currentTargetPath;
            ap->objectTargetNode        = pAgent[i]->memory_reference.currentTargetNode;
            ap->deviationFromObjectPath = pAgent[i]->memory_reference.lateralDeviationFromTargetPath;

            ap->nearestTargetPath              = -1;
            ap->deviationFromNearestTargetPath = 0.0;
            ap->distanceToObject               = 0.0;

            ap->myCPPathIndex    = -1;
            ap->objCPPathIndex   = -1;
            ap->objPathCPChecked = -1;

            ap->objPathRecogLabelChecked = -1;
            ap->myPathRecogLabelChecked  = -1;

            ap->objPathInLCTargetPathList = -1;
            ap->latDevObjInLaneChangeTargetPathList = 0.0;
            ap->distToObjInLaneChangeTargetPathList = 0.0;

            ap->relPosEvaled = false;

            if( agentKind < 100 ){

                int objPathIdx = memory.targetPathList.indexOf( ap->objectPath );

                bool checkForward = true;
                bool checkBackward = true;

                if( objPathIdx >= 0 && objPathIdx <= memory.currentTargetPathIndexInList ){
                    checkBackward = false;
                }
                else if( objPathIdx >= 0 && objPathIdx > memory.currentTargetPathIndexInList ){
                    checkForward = false;
                }

                if( ap->objectType >= 100 ){
                    checkForward = false;
                }

                int latMinIndex = -1;
                float latMin = 0.0;
                float txtMin = 0.0;
                float tytMin = 0.0;
                float txdMin = 0.0;
                float tydMin = 0.0;
                float tsMin  = 0.0;

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                QueryPerformanceCounter(&start);
#endif

                float checkDistance = 0.0;
                if( checkForward == true ){

                    for(int k=memory.currentTargetPathIndexInList;k>=0;--k){

                        float tdev,txt,tyt,txd,tyd,ts;
                        int chk = pRoad->GetDeviationFromPathWithZ( memory.targetPathList[k],
                                                                    pAgent[i]->state.x, pAgent[i]->state.y, pAgent[i]->state.z_path,
                                                                    pAgent[i]->state.yaw,
                                                                    tdev, txt, tyt, txd, tyd, ts,
                                                                    true );

                        if( chk != memory.targetPathList[k] ){
                            checkDistance += memory.targetPathLength[k];
                            if( checkDistance > param.visibleDistance ){
                                break;
                            }
                            continue;
                        }

                        if( latMinIndex < 0 || fabs(latMin) > fabs(tdev) ){
                            latMinIndex = k;
                            latMin = tdev;
                            txtMin = txt;
                            tytMin = tyt;
                            txdMin = txd;
                            tydMin = tyd;
                            tsMin  = ts;

                            if( fabs(tdev) < 3.0 ){
                                checkBackward = false;
                            }
                        }

                        checkDistance += memory.targetPathLength[k];
                        if( checkDistance > param.visibleDistance ){
                            break;
                        }
                    }
                }

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                QueryPerformanceCounter(&end);
                calTime[5] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                calCount[5]++;
#endif

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                QueryPerformanceCounter(&start);
#endif
                if( checkBackward == true ){
                    checkDistance = 0.0;
                    for(int k=memory.currentTargetPathIndexInList+1;k<memory.targetPathList.size();++k){

                        float tdev,txt,tyt,txd,tyd,ts;
                        int chk = pRoad->GetDeviationFromPathWithZ( memory.targetPathList[k],
                                                                    pAgent[i]->state.x, pAgent[i]->state.y, pAgent[i]->state.z_path,
                                                                    pAgent[i]->state.yaw,
                                                                    tdev, txt, tyt, txd, tyd, ts,
                                                                    true );


                        if( chk != memory.targetPathList[k] ){
                            checkDistance += memory.targetPathLength[k];
                            if( checkDistance > param.visibleDistance ){
                                break;
                            }
                            continue;
                        }

                        if( latMinIndex < 0 || fabs(latMin) > fabs(tdev) ){
                            latMinIndex = k;
                            latMin = tdev;
                            txtMin = txt;
                            tytMin = tyt;
                            txdMin = txd;
                            tydMin = tyd;
                            tsMin  = ts;
                        }

                        checkDistance += memory.targetPathLength[k];
                        if( checkDistance > param.visibleDistance ){
                            break;
                        }
                    }
                }

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                QueryPerformanceCounter(&end);
                calTime[6] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                calCount[6]++;
#endif

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                QueryPerformanceCounter(&start);
#endif

                if( latMinIndex >= 0 ){

                    int k = latMinIndex;

                    ap->nearestTargetPath              = memory.targetPathList[k];
                    ap->deviationFromNearestTargetPath = latMin;
                    ap->xOnTargetPath                  = txtMin;
                    ap->yOnTargetPath                  = tytMin;
                    ap->innerProductToNearestPathTangent = txdMin * ap->cos_yaw + tydMin * ap->sin_yaw;
                    ap->innerProductToNearestPathNormal  = (-tydMin) * ap->cos_yaw + txdMin * ap->sin_yaw;
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
                            distObj += tsMin;
                        }
                        if( foundMe == true && foundObj == true ){
                            break;
                        }
                        else if(foundMe == false && foundObj == false ){
                            continue;
                        }
                        float pLen = memory.targetPathLength[l];
                        if( foundMe == false ){
                            distMe += pLen;
                        }
                        if( foundObj == false ){
                            distObj += pLen;
                        }
                    }
                    dist = distObj - distMe;

                    ap->objDistFromSWPOfNearTargetPath = tsMin;
                    ap->distanceToObject = dist;

                    ap->relPosEvaled = true;
                }


                if( memory.checkSideVehicleForLC == true && memory.laneChangeTargetPathList.size() > 0 ){

                    float tdev,txt,tyt,txd,tyd,ts;
                    int objPathInLCPath = pRoad->GetNearestPathFromList( memory.laneChangeTargetPathList,
                                                                         pAgent[i]->state.x,
                                                                         pAgent[i]->state.y,
                                                                         pAgent[i]->state.z_path,
                                                                         pAgent[i]->state.yaw,
                                                                         tdev, txt, tyt, txd, tyd, ts);

                    if( objPathInLCPath >= 0 ){
                        ap->objPathInLCTargetPathList = objPathInLCPath;
                        ap->latDevObjInLaneChangeTargetPathList = tdev;

                        bool foundMe = false;
                        bool foundObj = false;
                        float distMe = 0.0;
                        float distObj = 0.0;
                        for(int l=memory.laneChangeTargetPathList.size()-1;l>=0;l--){

                            if( memory.laneChangeTargetPathList[l] == memory.currentPathInLCTargetPathList ){
                                foundMe = true;
                            }

                            if( memory.laneChangeTargetPathList[l] == objPathInLCPath  ){
                                foundObj = true;
                            }

                            if(foundMe == false && foundObj == false ){
                                continue;
                            }

                            float pLen = memory.laneChangePathLength[l];
                            if( foundMe == true ){
                                distMe += pLen;
                            }
                            if( foundObj == true ){
                                distObj += pLen;
                            }

                            if( foundMe == true && foundObj == true ){
                                break;
                            }
                        }

                        distMe -= memory.distFromSWPLCTargetPathList;
                        distObj -= ts;

                        ap->distToObjInLaneChangeTargetPathList = distMe - distObj;
                    }

                }

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
                QueryPerformanceCounter(&end);
                calTime[7] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                calCount[7]++;
#endif
            }
            else{

                float  dev = 0.0;
                float dist = 0.0;
                float xdir = 1.0;
                float ydir = 0.0;
                int secIndx = pRoad->GetDeviationFromPedestPathAllSection( memory.currentTargetPath,
                                                                           ap->x,
                                                                           ap->y,
                                                                           dev,
                                                                           dist,
                                                                           xdir,
                                                                           ydir );

                if( secIndx >= 0 ){

                    ap->nearestTargetPath = secIndx;
                    ap->deviationFromNearestTargetPath = dev;
                    ap->distanceToObject = dist - memory.distanceFromStartWPInCurrentPath;
                    ap->effectiveHalfWidth = ap->vHalfWidth;
                    ap->relPosEvaled = true;
                }

                ap->innerProductToNearestPathTangent = xdir * pAgent[i]->state.cosYaw + ydir * pAgent[i]->state.sinYaw;
                ap->innerProductToNearestPathNormal = xdir * (-pAgent[i]->state.sinYaw) + ydir * pAgent[i]->state.cosYaw;
                ap->effectiveHalfWidth = fabs(ap->innerProductToNearestPathNormal) * pAgent[i]->vHalfLength + fabs(ap->innerProductToNearestPathTangent) * pAgent[i]->vHalfWidth;
            }
        }
    }

    for(int j=memory.perceptedObjects.size()-1;j>=0;--j){
        if( memory.perceptedObjects[j]->noUpdateCount > 3 ){
            memory.perceptedObjects[j]->isValidData       = false;
            memory.perceptedObjects[j]->noUpdateCount     = 0;
            memory.perceptedObjects[j]->inView            = false;
            memory.perceptedObjects[j]->myCPPathIndex     = -1;
            memory.perceptedObjects[j]->objCPPathIndex    = -1;
            memory.perceptedObjects[j]->objPathCPChecked  = -1;
            memory.perceptedObjects[j]->hasCollisionPoint = false;
            memory.perceptedObjects[j]->shouldEvalRisk    = false;
            memory.perceptedObjects[j]->objPathRecogLabelChecked = -1;
            memory.perceptedObjects[j]->myPathRecogLabelChecked  = -1;
            memory.perceptedObjects[j]->winker                   = 0;
        }
    }


#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION
    QueryPerformanceCounter(&end);
    calTime[3] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
    calCount[3]++;
#endif




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

#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION
    QueryPerformanceCounter(&start);
#endif


    for(int i=0;i<memory.perceptedSignals.size();++i){
        if( memory.perceptedSignals[i]->isValidData == true ){
            memory.perceptedSignals[i]->inView = false;
            memory.perceptedSignals[i]->noUpdateCount++;
        }
    }



    if( agentKind < 100 && memory.routeType == AGENT_ROUTE_TYPE::NODE_LIST && onlyCheckPreceding == false ){

        QList<int> checkedNode;

        float distChecked = -(memory.distanceFromStartWPInCurrentPath);

        int cIdx = memory.currentTargetPathIndexInList;
        for(int i = cIdx; i >= 0; i--){

            int tPath = memory.targetPathList[i];
            int tpIdx = pRoad->pathId2Index.indexOf( tPath );

            distChecked += pRoad->paths[tpIdx]->pathLength;

            int cNode = pRoad->paths[tpIdx]->connectingNode;
            if( checkedNode.indexOf(cNode) >= 0 ){
                if( distChecked > param.visibleDistance * 2.0 ){
                    break;
                }
                continue;
            }
            else{
                checkedNode.append(cNode);
            }

            int ndIdx = pRoad->nodeId2Index.indexOf( cNode );
            if( pRoad->nodes[ndIdx]->hasTS == false ){
                if( distChecked > param.visibleDistance * 2.0){
                    break;
                }
                continue;
            }

            float dx = pRoad->nodes[ndIdx]->xc - state.x;
            float dy = pRoad->nodes[ndIdx]->yc - state.y;
            float D = dx * dx + dy * dy;
            if( D > VD2 ){
                if( distChecked > param.visibleDistance * 2.0 ){
                    break;
                }
                continue;
            }

            // find ts
            int inDir = pRoad->paths[tpIdx]->connectingNodeInDir;

            strForDebug += QString("TS cNode=%1 inDir=%2 relatedVTSIndex.size=%3\n").arg(cNode).arg(inDir).arg(pRoad->nodes[ndIdx]->relatedVTSIndex.size());

            for(int n=0;n<pRoad->nodes[ndIdx]->relatedVTSIndex.size();++n){

                int j = pRoad->nodes[ndIdx]->relatedVTSIndex[n];

                if( trafficSignal[j]->controlDirection != inDir ){
                    continue;
                }

                int psIdx = -1;
                for(int k=0;k<memory.perceptedSignals.size();++k){
                    if( memory.perceptedSignals[k]->isValidData == false ){
                        continue;
                    }
                    if( memory.perceptedSignals[k]->objectID == trafficSignal[j]->id ){
                        psIdx = k;
                        break;
                    }
                }

                strForDebug += QString("[1]psIdx=%1\n").arg(psIdx);

                if( psIdx < 0 ){

                    for(int k=0;k<memory.perceptedSignals.size();++k){
                        if( memory.perceptedSignals[k]->isValidData == false ){
                            psIdx = k;
                            continue;
                        }
                    }

                    strForDebug += QString("[2]psIdx=%1\n").arg(psIdx);

                    if( psIdx < 0 ){
                        struct TrafficSignalPerception* ts = new struct TrafficSignalPerception;
                        memory.perceptedSignals.append( ts );

                        psIdx = memory.perceptedSignals.size() - 1;
                    }

                    strForDebug += QString("[3]psIdx=%1\n").arg(psIdx);


                    struct TrafficSignalPerception* ts = memory.perceptedSignals.at(psIdx);

                    ts->isValidData = true;
                    ts->objectID    = trafficSignal[j]->id;
                    ts->objectType  = 'v';
                    ts->x           = trafficSignal[j]->xTS;
                    ts->y           = trafficSignal[j]->yTS;
                    ts->yaw         = trafficSignal[j]->direction;
                    ts->relatedNode = cNode;
                    ts->SLonPathID  = -1;
                    ts->SLID        = -1;

                }
                else{

                    int SLonPathID = memory.perceptedSignals[psIdx]->SLonPathID;
                    if( memory.targetPathList.indexOf( SLonPathID ) < 0 ){
                        memory.perceptedSignals[psIdx]->SLonPathID = -1;
                    }

                }

                memory.perceptedSignals[psIdx]->signalDisplay = trafficSignal[j]->GetCurrentDisplayInfo();


                strForDebug += QString("type=%1\n").arg(memory.perceptedSignals[psIdx]->objectType);
                strForDebug += QString("signalDisplay=%1 SLonPathID=%2\n").arg(memory.perceptedSignals[psIdx]->signalDisplay).arg(memory.perceptedSignals[psIdx]->SLonPathID);

                if( memory.perceptedSignals[psIdx]->SLonPathID < 0 ){

                    memory.perceptedSignals[psIdx]->distToSL = (-1.0) * memory.distanceFromStartWPInCurrentPath;

                    for(int k = cIdx; k >= 0; k--){
                        tPath = memory.targetPathList[k];
                        tpIdx = pRoad->pathId2Index.indexOf( tPath );
                        bool foundTs = false;
                        for(int l=0;l<pRoad->paths[tpIdx]->stopPoints.size();++l){
                            if( pRoad->paths[tpIdx]->stopPoints[l]->relatedNode == cNode ){

                                memory.perceptedSignals[psIdx]->SLID      = pRoad->paths[tpIdx]->stopPoints[l]->stopPointID;
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

                        tPath = memory.targetPathList.at(k);
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

            if( distChecked > param.visibleDistance * 2.0 ){
                break;
            }
        }
    }
    else if( agentKind >= 100 ){

        int plIdx = pRoad->pedestPathID2Index.indexOf( memory.currentTargetPath );
        if( plIdx >= 0 ){
            int szSect = pRoad->pedestPaths[plIdx]->shape.size() - 1;
            for(int i=memory.currentTargetPathIndexInList;i<szSect;++i){

                if( pRoad->pedestPaths[plIdx]->shape[i]->isCrossWalk == true ){

                    int pedestTSID = pRoad->pedestPaths[plIdx]->shape[i]->controlPedestSignalID;
                    for(int j=0;j<trafficSignal.size();++j){

                        if( trafficSignal[j]->id != pedestTSID ){
                            continue;
                        }

                        int psIdx = -1;
                        for(int k=0;k<memory.perceptedSignals.size();++k){
                            if( memory.perceptedSignals[k]->isValidData == false ){
                                continue;
                            }
                            if( memory.perceptedSignals[k]->objectID == trafficSignal[j]->id ){
                                psIdx = k;
                                break;
                            }
                        }

                        if( psIdx < 0 ){

                            for(int k=0;k<memory.perceptedSignals.size();++k){
                                if( memory.perceptedSignals[k]->isValidData == false ){
                                    psIdx = k;
                                    continue;
                                }
                            }
                            if( psIdx < 0 ){
                                struct TrafficSignalPerception* ts = new struct TrafficSignalPerception;
                                memory.perceptedSignals.append( ts );

                                psIdx = memory.perceptedSignals.size() - 1;
                            }

                            struct TrafficSignalPerception* ts = memory.perceptedSignals[psIdx];

                            ts->isValidData = true;
                            ts->objectID    = trafficSignal[j]->id;
                            ts->objectType  = 'p';
                            ts->x           = trafficSignal[j]->xTS;
                            ts->y           = trafficSignal[j]->yTS;
                            ts->yaw         = trafficSignal[j]->direction;
                            ts->relatedNode = trafficSignal[j]->relatedNode;
                            ts->SLonPathID  = -1;
                            ts->SLID        = -1;

                        }

                        memory.perceptedSignals[psIdx]->signalDisplay = trafficSignal[j]->GetCurrentDisplayInfo();

                        memory.perceptedSignals[psIdx]->inView = true;
                        memory.perceptedSignals[psIdx]->noUpdateCount = 0;
                    }
                    break;
                }
            }
        }
    }


    for(int j=memory.perceptedSignals.size()-1;j>=0;--j){
        if( memory.perceptedSignals[j]->noUpdateCount > 3 ){
            memory.perceptedSignals[j]->isValidData = false;
        }
    }


#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION
    QueryPerformanceCounter(&end);
    calTime[4] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
    calCount[4]++;
#endif


//    qDebug() << "Percetion of Traffic Signals:";
//    for(int i=0;i<memory.perceptedSignals.size();++i){
//        qDebug() << " ID = " << memory.perceptedSignals[i]->objectID
//                 << " Color = " << memory.perceptedSignals[i]->signalDisplay
//                 << " Dist = " << memory.perceptedSignals[i]->distToSL;
//    }



#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION
    if( calCount[0] == 500 ){
        qDebug() << "Agent : ID = " << ID;
        for(int i=0;i<5;i++){
            calTime[i] /= calCount[i];
            qDebug() << "   Mean Time[" << i << "] = " << calTime[i];
            calCount[i] = 0;
        }
    }
#endif


#ifdef _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
    if( calCount[0] == 500 ){
        qDebug() << "Agent : ID = " << ID;
        for(int i=0;i<8;i++){
            calTime[i] /= calCount[i];
            qDebug() << "   Mean Time[" << i << "] = " << calTime[i] << "  Count=" << calCount[i];
            calCount[i] = 0;
        }
    }
#endif
}


