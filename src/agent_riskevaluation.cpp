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


void Agent::RiskEvaluation(Agent** pAgent, int maxAgent, Road* pRoad)
{

    if( decisionMalingCount != 0 ){
        return;
    }


    if( memory.controlMode == AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY ){

        if( memory.precedingVehicleIDByScenario >= 0 ){

            for(int i=0;i<memory.perceptedObjects.size();++i){

                if( memory.perceptedObjects[i]->isValidData == false ){
                    continue;
                }

                if( memory.perceptedObjects[i]->objectID == memory.precedingVehicleIDByScenario ){

                    memory.doHeadwayDistanceControl   = true;

                    memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->distanceToObject;

                    if( memory.distanceToPrecedingVehicle >= 0.0 ){
                        memory.distanceToPrecedingVehicle -= memory.perceptedObjects[i]->vHalfLength;
                        memory.distanceToPrecedingVehicle -= vHalfLength;
                    }
                    else{
                        memory.distanceToPrecedingVehicle += memory.perceptedObjects[i]->vHalfLength;
                        memory.distanceToPrecedingVehicle += vHalfLength;
                    }

                    memory.speedPrecedingVehicle = memory.perceptedObjects[i]->V * memory.perceptedObjects[i]->innerProductToNearestPathTangent;
                    memory.axPrecedingVehicle    = memory.perceptedObjects[i]->Ax;

//                    qDebug() << "[Agent" << ID << "]";
//                    qDebug() << "Preceding Vehicle ID = " << memory.precedingVehicleIDByScenario;
//                    qDebug() << " distanceToPrecedingVehicle = " << memory.distanceToPrecedingVehicle;
//                    qDebug() << " innerProductToNearestPathTangent = " << memory.perceptedObjects[i]->innerProductToNearestPathTangent;
//                    qDebug() << " speedPrecedingVehicle = " << memory.speedPrecedingVehicle;

                    break;
                }
            }

        }
        else{
            memory.doHeadwayDistanceControl = false;
        }

        return;
    }




    if(  memory.controlMode == AGENT_CONTROL_MODE::AGENT_LOGIC ){


        if( state.V < 1.0 ){
            memory.speedZeroCount++;
        }
        else{
            memory.speedZeroCount = 0;
        }

        memory.causeOfStopControl = QString("---");

        //
        // Risk evaluation for Traffic Signal
        if( memory.doStopControl == true ){
            memory.releaseStopCount++;

            // memory.releaseStopCount == 0 is added to move from temporal stop, yeilding or confirmation of safety,
            // without delay
            if( memory.releaseStopCount == 0 || memory.releaseStopCount * calInterval >=param.startRelay ){
                memory.doStopControl = false;
                memory.releaseStopCount = 0;
            }
        }

        int signalColor = 0;

        if( memory.perceptedSignals.size() > 0 ){

            if( agentKind < 100 ){

                bool ShouldStop = false;
                float minDist = 0.0;

                for(int i=0;i<memory.perceptedSignals.size();++i){

                    if( memory.perceptedSignals[i]->isValidData == false ){
                        continue;
                    }

//                    strForDebug += QString("Check Signal %1\n").arg( memory.perceptedSignals[i]->objectID );

                    bool alreadyPassed = false;
                    for(int j=0;j<memory.currentTargetNodeIndexInNodeList;j++){
                        if( memory.myNodeList[j] == memory.perceptedSignals[i]->relatedNode ){
                            alreadyPassed = true;
                            break;
                        }
                    }

//                    strForDebug += QString("alreadyPassed = %1\n").arg( alreadyPassed );

                    if( alreadyPassed == true ){
                        continue;
                    }

                    signalColor = memory.perceptedSignals[i]->signalDisplay;

                    if( memory.perceptedSignals[i]->signalDisplay == TRAFFICSIGNAL_YELLOW ){

                        if( memory.shouldStopAtSignalSL == true || memory.perceptedSignals[i]->distToSL > memory.distanceToZeroSpeed + 5.0 ){

                            if( ShouldStop == false || minDist > memory.perceptedSignals[i]->distToSL ){
                                ShouldStop = true;
                                minDist = memory.perceptedSignals[i]->distToSL;
                                signalColor = TRAFFICSIGNAL_YELLOW;

//                                strForDebug += QString("[Y]signalColor = %1\n").arg( signalColor );
                            }
                        }
                    }
                    else if( memory.perceptedSignals[i]->signalDisplay == TRAFFICSIGNAL_RED ){

                        // Not to stay turn vehicle inside intersection
                        if( pRoad->LeftOrRight == 0 && memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING &&
                                memory.distanceToTurnNodeWPIn < 0.0 ){
                            continue;
                        }
                        else if( pRoad->LeftOrRight == 1 && memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING &&
                                memory.distanceToTurnNodeWPIn < 0.0 ){
                            continue;
                        }

                        if( memory.shouldStopAtSignalSL == true || memory.distToNearestCP > memory.distanceToZeroSpeed ){

                            if( ShouldStop == false || minDist > memory.perceptedSignals[i]->distToSL ){
                                ShouldStop = true;
                                minDist = memory.perceptedSignals[i]->distToSL;
                                signalColor = TRAFFICSIGNAL_RED;

//                                strForDebug += QString("[R]signalColor = %1\n").arg( signalColor );
                            }
                        }
                    }
                }

                if( ShouldStop == true ){
                    memory.doStopControl = true;
                    memory.distanceToStopPoint = minDist - vHalfLength;
                    memory.releaseStopCount = 0;
                    memory.causeOfStopControl = QString("Signal");
                }
                else{
                    if( memory.doStopControl == true ){
                        // Check remaining cross vehicles inside intersection
                        for(int i=0;i<memory.perceptedObjects.size();++i){

                            if( memory.perceptedObjects[i]->isValidData == false ){
                                continue;
                            }

                            if( memory.currentTargetNode != memory.perceptedObjects[i]->objectTargetNode ){
                                continue;
                            }

                            int objID = memory.perceptedObjects[i]->objectID;
                            if( pAgent[objID]->memory_reference.distanceToNodeWPIn > 0.0 ){
                                continue;
                            }

                            if( memory.perceptedObjects[i]->recognitionLabel >= AGENT_RECOGNITION_LABEL::LEFT_CROSSING_STRAIGHT &&
                                    memory.perceptedObjects[i]->recognitionLabel <= AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_RIGHT ){

                                memory.releaseStopCount = 0;
                                memory.causeOfStopControl = QString("Remaining Cross Vehicle: ID=%1").arg( memory.perceptedObjects[i]->objectID );

                                ShouldStop = true;
                            }
                        }
                    }
                }

                memory.shouldStopAtSignalSL = ShouldStop;
            }
            else if( agentKind >= 100 ){

                int plIdx = pRoad->pedestPathID2Index.indexOf( memory.currentTargetPath );
                if( plIdx >= 0 ){

                    int szSect = pRoad->pedestPaths[plIdx]->shape.size() - 1;
                    int sIdx = memory.currentTargetPathIndexInList;


                    int currentDisplay = TRAFFICSIGNAL_BLUE;
                    for(int i=0;i<memory.perceptedSignals.size();++i){
                        if( memory.perceptedSignals[i]->isValidData == false ){
                            continue;
                        }
                        currentDisplay = memory.perceptedSignals[i]->signalDisplay;
                    }

//                    strForDebug += QString("sIdx = %1,  szSect = %2  , currentDisplay = %3\n").arg( sIdx ).arg( szSect ).arg( currentDisplay );


                    if( sIdx + 1 < szSect ){

                        if(pRoad->pedestPaths[plIdx]->shape[sIdx]->isCrossWalk == true  ){

                           float dx = state.x - pRoad->pedestPaths[plIdx]->shape[sIdx]->pos.x();
                           float dy = state.y - pRoad->pedestPaths[plIdx]->shape[sIdx]->pos.y();
                           float S = dx * (pRoad->pedestPaths[plIdx]->shape[sIdx]->cosA) + dy * (pRoad->pedestPaths[plIdx]->shape[sIdx]->sinA);

//                           strForDebug += QString("S = %1\n").arg( S );

                           if( S < 1.0 ){
                               if( currentDisplay == TRAFFICSIGNAL_RED ){
                                   memory.shouldStopAtSignalSL = true;
//                                   strForDebug += QString("[1] shouldStopAtSignalSL = true\n");
                               }
                               else{
                                   memory.shouldStopAtSignalSL = false;
//                                   strForDebug += QString("[2] shouldStopAtSignalSL = false\n");
                               }
                           }
                           else{
                               memory.shouldStopAtSignalSL = false;
//                               strForDebug += QString("[3] shouldStopAtSignalSL = false\n");
                           }
                        }
                        else{
                            int sigIdx = -1;
                            for(int i=sIdx+1;i<szSect;++i){
                                if( pRoad->pedestPaths[plIdx]->shape[i]->isCrossWalk == true ){
                                    sigIdx = i;
                                    break;
                                }
                            }

//                            strForDebug += QString("sigIdx = %1\n").arg( sigIdx );

                            if( sigIdx > 0 ){

                                float distToSig = 0.0;
                                for(int i=sigIdx-1;i>=sIdx;i--){
                                    if( i == sIdx ){
                                        float dx = state.x - pRoad->pedestPaths[plIdx]->shape[i]->pos.x();
                                        float dy = state.y - pRoad->pedestPaths[plIdx]->shape[i]->pos.y();
                                        float S = dx * (pRoad->pedestPaths[plIdx]->shape[i]->cosA) + dy * (pRoad->pedestPaths[plIdx]->shape[i]->sinA);

//                                        strForDebug += QString("distToSig = %1\n").arg( distToSig );
//                                        strForDebug += QString("distanceToNextPos = %1\n").arg( pRoad->pedestPaths[plIdx]->shape[i]->distanceToNextPos );
//                                        strForDebug += QString("S = %1\n").arg( S );

                                        if( distToSig + pRoad->pedestPaths[plIdx]->shape[i]->distanceToNextPos - S < 2.5 ){
                                            if( currentDisplay == TRAFFICSIGNAL_RED ){
                                                memory.shouldStopAtSignalSL = true;
//                                                strForDebug += QString("[4] shouldStopAtSignalSL = true\n");
                                            }
                                            else{
                                                memory.shouldStopAtSignalSL = false;
//                                                strForDebug += QString("[5] shouldStopAtSignalSL = false\n");
                                            }
                                        }
                                        else{
                                            memory.shouldStopAtSignalSL = false;
//                                            strForDebug += QString("[6] shouldStopAtSignalSL = false\n");
                                        }
                                    }
                                    else{
                                        distToSig += pRoad->pedestPaths[plIdx]->shape[i]->distanceToNextPos;
                                    }
                                }
                            }
                        }
                    }
                    else{
                        memory.shouldStopAtSignalSL = false;
//                        strForDebug += QString("[0]shouldStopAtSignalSL = false\n");
                    }
                }
            }
        }
        else{
            memory.shouldStopAtSignalSL = false;
        }



        //
        // Risk evaluation for preceding vehicle
        if( agentKind < 100 ){

            memory.doHeadwayDistanceControl = false;
            memory.precedingVehicleID = -1;
            memory.distanceToPrecedingVehicle = 0.0;
            memory.precedingObstacle = 0;

            if( memory.avoidTarget < 0 ){
                memory.lateralShiftTarget = 0.0;
            }
            else {
                int tmpAvoidTarget = memory.avoidTarget;
                memory.avoidTarget = -1;
                for(int i=0;i<memory.perceptedObjects.size();++i){

                    if( memory.perceptedObjects[i]->isValidData == false ){
                        continue;
                    }

                    if( memory.perceptedObjects[i]->relPosEvaled == false ){
                        continue;
                    }

                    if( memory.perceptedObjects[i]->objectID == tmpAvoidTarget ){
                        if( memory.perceptedObjects[i]->distanceToObject > 0.0 ){
                            memory.avoidTarget = tmpAvoidTarget;
                            break;
                        }
                    }
                }
            }


            float vLenH = 0.0;

            for(int i=0;i<memory.perceptedObjects.size();++i){

                if( memory.perceptedObjects[i]->isValidData == false ){
                    continue;
                }

                if( memory.perceptedObjects[i]->relPosEvaled == false ){
                    continue;
                }

                if( memory.perceptedObjects[i]->objectType >= 100 ){
                    continue;
                }


                if( memory.perceptedObjects[i]->recognitionLabel == PRECEDING ){

                    if( memory.precedingVehicleID < 0 ||  memory.distanceToPrecedingVehicle > memory.perceptedObjects[i]->distanceToObject ){

                        memory.precedingVehicleID = memory.perceptedObjects[i]->objectID;
                        memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->distanceToObject;
                        memory.speedPrecedingVehicle = memory.perceptedObjects[i]->V;
                        memory.axPrecedingVehicle    = memory.perceptedObjects[i]->Ax;
                        vLenH = memory.perceptedObjects[i]->vHalfLength;
                        memory.precedingObstacle = 0;
                    }
                }
                else{

                    // If a front vehicle such as cross-merging and oncoming-mergine comes into my lane, regard the vehile as preceding
                    if( fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath - memory.lateralShiftTarget) - memory.perceptedObjects[i]->effectiveHalfWidth - vHalfWidth < 0.5 &&
                            memory.perceptedObjects[i]->distanceToObject > 0.0 ){

                        if( memory.perceptedObjects[i]->V < 3.6 ){
                            if( fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath) - memory.perceptedObjects[i]->effectiveHalfWidth - vHalfWidth > 0.1 ){
                                continue;
                            }
                        }

                        if( memory.nextTurnNode == memory.currentTargetNode ){
                            if( memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING &&
                                    memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_LEFT ){
                                continue;
                            }
                            else if( memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING &&
                                    memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_RIGHT ){
                                continue;
                            }

                            if( (memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING || memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING ) &&
                                    memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_STRAIGHT ){
                                continue;
                            }
                        }


                        if( memory.precedingVehicleID < 0 ||  memory.distanceToPrecedingVehicle > memory.perceptedObjects[i]->distanceToObject ){

                            // Avoid turn-waiting stopping oncoming vehicle if necessary
                            if( memory.currentTargetNode != memory.nextTurnNode &&
                                    memory.currentTargetNode == memory.perceptedObjects[i]->objectTargetNode &&
                                    memory.perceptedObjects[i]->V < 1.0 ){

                                if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_LEFT ||
                                    memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_RIGHT ){

                                    memory.avoidTarget = memory.perceptedObjects[i]->objectID;
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 0.0){
                                        memory.lateralShiftTarget = memory.perceptedObjects[i]->deviationFromNearestTargetPath;
                                        memory.lateralShiftTarget -= memory.perceptedObjects[i]->effectiveHalfWidth;
                                        memory.lateralShiftTarget -= vHalfWidth;
                                        memory.lateralShiftTarget -= 0.5;
                                        if( memory.lateralShiftTarget > 0.0 ){
                                            memory.lateralShiftTarget = 0.0;
                                            memory.avoidTarget = -1;
                                        }
                                        else if( memory.lateralShiftTarget < -(1.5 - vHalfWidth)){
                                            memory.lateralShiftTarget = -(1.5 - vHalfWidth);
                                        }
                                    }
                                    else if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < 0.0){
                                        memory.lateralShiftTarget = memory.perceptedObjects[i]->deviationFromNearestTargetPath;
                                        memory.lateralShiftTarget += memory.perceptedObjects[i]->effectiveHalfWidth;
                                        memory.lateralShiftTarget += vHalfWidth;
                                        memory.lateralShiftTarget += 0.5;
                                        if( memory.lateralShiftTarget < 0.0 ){
                                            memory.lateralShiftTarget = 0.0;
                                            memory.avoidTarget = -1;
                                        }
                                        else if( memory.lateralShiftTarget > (1.5 - vHalfWidth)){
                                            memory.lateralShiftTarget = (1.5 - vHalfWidth);
                                        }
                                    }
                                }
                            }


                            memory.precedingVehicleID = memory.perceptedObjects[i]->objectID;
                            memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->distanceToObject;
                            memory.speedPrecedingVehicle = memory.perceptedObjects[i]->V * memory.perceptedObjects[i]->innerProductToNearestPathNormal;
                            if( memory.speedPrecedingVehicle < 0.0 ){
                                memory.speedPrecedingVehicle = 0.0;
                            }
                            memory.axPrecedingVehicle    = memory.perceptedObjects[i]->Ax;
                            vLenH = memory.perceptedObjects[i]->vHalfLength;
                            memory.precedingObstacle = 1;
                        }
                    }
                }
            }

            if( memory.precedingVehicleID >= 0 ){

                memory.doHeadwayDistanceControl = true;

                memory.targetHeadwayDistance = param.headwayTime * state.V;
                if( memory.targetHeadwayDistance < param.minimumHeadwayDistanceAtStop + vLenH + vHalfLength  ){
                    memory.targetHeadwayDistance = param.minimumHeadwayDistanceAtStop + vLenH + vHalfLength;
                }
            }
        }
        else if( agentKind >= 100 ){

            memory.doHeadwayDistanceControl = false;

            for(int i=0;i<memory.perceptedObjects.size();++i){
                if( memory.perceptedObjects[i]->objectType < 100 ){
                    continue;
                }

                if( memory.currentTargetPath != memory.perceptedObjects[i]->objectPath ){
                    continue;
                }


                float dx = memory.perceptedObjects[i]->x - state.x;
                float dy = memory.perceptedObjects[i]->y - state.y;
                float L = dx * state.cosYaw + dy * state.sinYaw;

                float minDist = 1.5 * state.V;
                if( minDist < 2.0 ){
                    minDist = 2.0;
                }

                if( L < 0.0 || L > minDist ){
                    continue;
                }
                float Y = memory.perceptedObjects[i]->deviationFromNearestTargetPath + memory.lateralShiftTarget;
                if( fabs(Y) > 0.6 ){
                    continue;
                }

                if( memory.perceptedObjects[i]->V < 0.1 ){
                    memory.doHeadwayDistanceControl = true;
                    break;
                }
            }

        }



        //
        // Risk evaluation of oncoming vehicle
        if( agentKind < 100 && memory.nextTurnNode >= 0 &&
                ((memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING && pRoad->LeftOrRight == 0) ||
                 (memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING && pRoad->LeftOrRight == 1)) ){

            memory.distToNearOncomingCP = 0.0;
            memory.distToFatOncomingCP  = 0.0;

            if( memory.nearOncomingWaitPathInfo >= 0 ){

                for(int j=memory.currentTargetPathIndexInList;j>=0;j--){

                    int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );

                    if( memory.targetPathList[j] == memory.nearOncomingWaitPathInfo ){
                        memory.distToNearOncomingCP += pRoad->paths[pIdx]->crossPoints[ memory.nearOncomingWaitCPInfo ]->distFromStartWP;
                        break;
                    }
                    memory.distToNearOncomingCP += pRoad->paths[pIdx]->pathLength;
                }
                memory.distToNearOncomingCP -= memory.distanceFromStartWPInCurrentPath;
            }

            if( memory.farOncomingWaitPathInfo >= 0 ){

                for(int j=memory.currentTargetPathIndexInList;j>=0;j--){

                    int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );

                    if( memory.targetPathList[j] == memory.farOncomingWaitPathInfo ){

                        memory.distToFatOncomingCP += pRoad->paths[pIdx]->crossPoints[ memory.farOncomingWaitCPInfo ]->distFromStartWP;
                        break;
                    }
                    memory.distToFatOncomingCP += pRoad->paths[pIdx]->pathLength;
                }
                memory.distToFatOncomingCP -= memory.distanceFromStartWPInCurrentPath;
            }

            // not pass wait point
            if( memory.distToNearOncomingCP > 0.0 ){

                bool shoudStop = false;

                float dStop = 0.0;
                if( memory.distToNearOncomingCP < 5.0 ){
                    dStop = memory.distanceToZeroSpeed;
                }
                if( memory.distToNearOncomingCP - dStop < vHalfLength ){

                    shoudStop = false;

                }
                else{
                    for(int i=0;i<memory.perceptedObjects.size();++i){

                        if( memory.perceptedObjects[i]->isValidData == false ){
                            continue;
                        }

                        if( memory.perceptedObjects[i]->relPosEvaled == false ){
                            continue;
                        }

                        if( memory.perceptedObjects[i]->objectType >= 100 ){
                            continue;
                        }

                        if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_STRAIGHT &&
                                memory.perceptedObjects[i]->hasCollisionPoint == true ){

                            // Checl collision point is in turn node
                            if( memory.perceptedObjects[i]->CPinNode != memory.nextTurnNode ){
                                continue;
                            }

                            // If the oncoming vehicle is around collision point, do not enter
                            if( memory.perceptedObjects[i]->objectTimeToCP >= 0.0 && memory.perceptedObjects[i]->objectTimeToCP < 3.0 ){
                                shoudStop = true;
                                break;
                            }

                            // The oncoming vehicle has already passed collision point
                            if( memory.perceptedObjects[i]->objectTimeToCP < 0.0 ){
                                continue;
                            }

                            // Far oncoming vehicle is neglected
                            if( memory.perceptedObjects[i]->objectTimeToCP > 6.0 && memory.perceptedObjects[i]->objectDistanceToCP > 30.0 ){
                                continue;
                            }


                            //�@I have already passed collision point
                            if( memory.perceptedObjects[i]->myTimeToCP < 0.0 ){
                                continue;
                            }

                            //�@I am far from collision point
                            if( memory.perceptedObjects[i]->myTimeToCP > 6.0 && memory.perceptedObjects[i]->myDistanceToCP > 60.0 ){
                                continue;
                            }

                            // If the signal intersection and the color is red, check if the oncoming stop

//                            strForDebug += QString("signalColor=%1\n").arg( signalColor );

                            if( signalColor == TRAFFICSIGNAL_YELLOW || signalColor == TRAFFICSIGNAL_RED ){
                                int objID = memory.perceptedObjects[i]->objectID;
                                float dSL = pAgent[objID]->memory_reference.distanceToStopPoint;
                                float dVZ = pAgent[objID]->memory_reference.distanceToZeroSpeed - pAgent[objID]->state.V - memory.perceptedObjects[i]->vHalfLength;

//                                strForDebug += QString("dSL=%1 dVZ=%2\n").arg( dSL ).arg( dVZ );

                                if( dSL + 5.0 > dVZ ){
                                    continue;
                                }
                            }

                            // Check if a vehicle exist in the same lane
                            bool noNeedToEval = false;
                            int objID = memory.perceptedObjects[i]->objectID;
                            for(int j=0;j<pAgent[objID]->memory_reference.perceptedObjects.size();++j){
                                if( pAgent[objID]->memory_reference.perceptedObjects[j]->isValidData == false ){
                                    continue;
                                }
                                if( pAgent[objID]->memory_reference.perceptedObjects[j]->recognitionLabel == AGENT_RECOGNITION_LABEL::PRECEDING ){
                                    int pID = pAgent[objID]->memory_reference.perceptedObjects[j]->objectID;
                                    for(int k=0;k<memory.perceptedObjects.size();++k){
                                        if( i == k ){
                                            break;
                                        }
                                        if( memory.perceptedObjects[k]->isValidData == false ){
                                            continue;
                                        }
                                        if( memory.perceptedObjects[k]->objectID == pID && memory.perceptedObjects[k]->V < 0.1 ){
                                            if( memory.perceptedObjects[k]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_STRAIGHT ||
                                                    memory.perceptedObjects[k]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_LEFT ||
                                                    memory.perceptedObjects[k]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_RIGHT ){
                                                if( memory.perceptedObjects[k]->distanceToObject > 0.0 ){
                                                    noNeedToEval = true;
                                                }
                                            }
                                            break;
                                        }
                                    }
                                }
                            }
                            if( noNeedToEval == true ){
                                continue;
                            }


                            // Prediction; The oncoming vehicle will pass collision when I reach the point
                            float preTime = 1.0;
                            if( memory.perceptedObjects[i]->myTimeToCP < preTime ){
                                preTime = memory.perceptedObjects[i]->myTimeToCP;
                            }
                            if( memory.perceptedObjects[i]->objectDistanceToCP - memory.perceptedObjects[i]->V * preTime  < -5.0){
                                continue;
                            }


                            float myTime = memory.perceptedObjects[i]->myDistanceToCP / 5.0;
                            if( myTime > memory.perceptedObjects[i]->myTimeToCP ){
                                myTime = memory.perceptedObjects[i]->myTimeToCP;
                            }

                            float crossTime = myTime - memory.perceptedObjects[i]->objectTimeToCP;
                            if( crossTime > param.crossTimeSafetyMargin ){  // oncoming vehicle will pass faster than me
                                continue;
                            }
                            else if( crossTime < (-1.0) * param.crossTimeSafetyMargin * 2.0 ){  // I can pass faster than oncoming vehicle
                                continue;
                            }

//                            strForDebug += QString("[1]Oncoming Risk by ID%1\n").arg(  memory.perceptedObjects[i]->objectID );

                            shoudStop = true;
                            break;
                        }
                        else if( ( (pRoad->LeftOrRight == 0 && memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_LEFT ) ||
                                   (pRoad->LeftOrRight == 1 && memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_RIGHT) ) &&
                                    memory.perceptedObjects[i]->hasCollisionPoint == true ){

                            // Checl collision point is in turn node
                            if( memory.perceptedObjects[i]->CPinNode != memory.nextTurnNode ){
                                continue;
                            }

                            // Prediction; The oncoming-merging vehicle will pass collision when I reach the point
                            float preTime = 1.0;
                            if( memory.perceptedObjects[i]->myTimeToCP < preTime ){
                                preTime = memory.perceptedObjects[i]->myTimeToCP;
                            }
                            if( memory.perceptedObjects[i]->objectDistanceToCP - memory.perceptedObjects[i]->V * preTime  < 0.0){
                                continue;
                            }

                            // If the oncoming-merging vehicle is around collision/merging point, do not enter
                            if( memory.perceptedObjects[i]->objectTimeToCP >= 0.0 && memory.perceptedObjects[i]->objectTimeToCP < 3.0 ){
                                shoudStop = true;
                                break;
                            }

                            // The oncoming vehicle has already passed collision point
                            if( memory.perceptedObjects[i]->objectTimeToCP < 0.0 ){
                                continue;
                            }

                            // If the signal intersection and the color is red, check if the oncoming stop
                            if( signalColor == TRAFFICSIGNAL_YELLOW || signalColor == TRAFFICSIGNAL_RED ){
                                int objID = memory.perceptedObjects[i]->objectID;
                                float dSL = pAgent[objID]->memory_reference.distanceToStopPoint;
                                float dVZ = pAgent[objID]->memory_reference.distanceToZeroSpeed - pAgent[objID]->state.V - memory.perceptedObjects[i]->vHalfLength;

//                                strForDebug += QString("dSL=%1 dVZ=%2").arg( dSL ).arg( dVZ );

                                if( dSL + 5.0 > dVZ ){
                                    continue;
                                }
                            }

                            float myTime = memory.perceptedObjects[i]->myDistanceToCP / 5.0;
                            if( myTime > memory.perceptedObjects[i]->myTimeToCP ){
                                myTime = memory.perceptedObjects[i]->myTimeToCP;
                            }

                            float crossTime = myTime - memory.perceptedObjects[i]->objectTimeToCP;
                            if( crossTime > param.crossTimeSafetyMargin ){  // oncoming-merging vehicle will pass faster than me
                                continue;
                            }
                            else if( crossTime < (-1.0) * param.crossTimeSafetyMargin * 2.0 ){  // I can pass faster than oncoming-merging vehicle
                                continue;
                            }

//                            strForDebug += QString("[2]Oncoming Risk by ID%1\n").arg(  memory.perceptedObjects[i]->objectID );

                            shoudStop = true;
                            break;
                        }
                    }
                }

                memory.shouldWaitOverCrossPoint = shoudStop;

                if( shoudStop == true ){

                    if( memory.doStopControl == true ){
                        float d = memory.distToNearOncomingCP - param.crossWaitPositionSafeyMargin - vHalfLength;;
                        if( memory.distanceToStopPoint > d ){
                            memory.distanceToStopPoint = d;
                            memory.releaseStopCount = -1;
                        }
                    }
                    else{
                        memory.doStopControl = true;
                        memory.distanceToStopPoint = memory.distToNearOncomingCP - param.crossWaitPositionSafeyMargin - vHalfLength;
                        memory.releaseStopCount = -1;
                        memory.causeOfStopControl = QString("Oncoming");
                    }
                }
            }
        }


        //
        // Check of pedestrian crossing
        if( agentKind < 100 ){

            for(int i=0;i<memory.perceptedObjects.size();++i){

                if( memory.perceptedObjects[i]->objectType < 100 ){
                    continue;
                }

                if( memory.perceptedObjects[i]->hasCollisionPoint == false ){
                    continue;
                }

                if( memory.perceptedObjects[i]->distanceToObject < 0.0 ){
                    continue;
                }

                if( fabs(memory.perceptedObjects[i]->objectDistanceToCP) > vHalfWidth &&
                        memory.perceptedObjects[i]->objectTimeToCP > 5.0 ){
                    continue;
                }

                if( memory.perceptedObjects[i]->objectTimeToCP < -1.5 ){
                    continue;
                }


                int turnDir = 0;
                int ntpIdx = pRoad->pathId2Index.indexOf( memory.perceptedObjects[i]->nearestTargetPath );
                if( ntpIdx >= 0 ){
                    int crossNode = pRoad->paths[ntpIdx]->connectingNode;
                    if( crossNode == memory.nextTurnNode ){
                        turnDir = memory.nextTurnDirection;
                    }
                }

                if( turnDir == 0 || turnDir == DIRECTION_LABEL::STRAIGHT ){
                    if( fabs(memory.perceptedObjects[i]->objectDistanceToCP) > vHalfWidth + 0.5 ){
                        continue;
                    }
                }


                if( (pRoad->LeftOrRight == 0 && turnDir == DIRECTION_LABEL::RIGHT_CROSSING) ||
                        (pRoad->LeftOrRight == 1 && turnDir == DIRECTION_LABEL::LEFT_CROSSING) ){

                    if( memory.doStopControl == true ){
                        float d = memory.distToNearOncomingCP - param.crossWaitPositionSafeyMargin - vHalfLength;;
                        if( memory.distanceToStopPoint > d ){
                            memory.distanceToStopPoint = d;
                            memory.releaseStopCount = -1;
                        }
                    }
                    else{
                        memory.doStopControl = true;
                        memory.distanceToStopPoint = memory.distToNearOncomingCP - param.crossWaitPositionSafeyMargin - vHalfLength;
                        memory.releaseStopCount = -1;
                        memory.causeOfStopControl = QString("Pedest-But-Wait-at-Oncoming");
                    }

                }
                else{

                    if( memory.doStopControl == true ){
                        float d = memory.perceptedObjects[i]->myDistanceToCP - param.pedestWaitPositionSafetyMargin - vHalfLength;
                        if( memory.distanceToStopPoint > d ){
                            memory.distanceToStopPoint = d;
                            memory.releaseStopCount = -1;
                        }
                    }
                    else{
                        memory.doStopControl = true;
                        memory.distanceToStopPoint = memory.perceptedObjects[i]->myDistanceToCP - param.pedestWaitPositionSafetyMargin - vHalfLength;
                        memory.releaseStopCount = -1;
                        memory.causeOfStopControl = QString("Pedest-Wait");
                    }

                }
            }

        }


        //
        // Check of vehicles when enter non-signalized intersection, should yeild the way
        if( memory.shouldYeild == true ){

            if( memory.distToYeildStopLine > 2.5 + vHalfLength ){
                if( memory.doStopControl == true ){
                    float d = memory.distToYeildStopLine - vHalfLength;;
                    if( memory.distanceToStopPoint > d ){
                        memory.distanceToStopPoint = d;
                        memory.releaseStopCount = -1;
                    }
                }
                else{
                    memory.doStopControl = true;
                    memory.distanceToStopPoint = memory.distToYeildStopLine - vHalfLength;
                    memory.releaseStopCount = -1;
                    memory.causeOfStopControl = QString("Yeild 1");
                }

                memory.leftCrossIsClear = false;
                memory.rightCrossIsClear = false;

                memory.leftCrossCheckCount = 0;
                memory.rightCrossCheckCount = 0;

                memory.safetyConfimed = false;
            }
            else{

                if( memory.doStopControl == true ){
                    float d = memory.distanceToTurnNodeWPIn - vHalfLength;
                    if( (pRoad->LeftOrRight == 0 && memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING) ||
                         (pRoad->LeftOrRight == 1 && memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING)   ){
                        d -= 5.0;
                    }

                    if( memory.distanceToStopPoint > d ){
                        memory.distanceToStopPoint = d;
                        memory.releaseStopCount = -1;
                    }
                }
                else{
                    memory.doStopControl = true;
                    memory.distanceToStopPoint = memory.distanceToTurnNodeWPIn - vHalfLength;
                    memory.releaseStopCount = -1;
                    memory.causeOfStopControl = QString("Yeild 2");
                }


                if( memory.distanceToStopPoint < 2.5 + vHalfLength && state.V >= 2.0 ){
                    if( memory.safetyConfimed == true ){
                        memory.doStopControl = false;
                    }
                }
                else if( memory.distanceToStopPoint < 2.5 + vHalfLength && state.V < 2.0 ){

                    if( memory.safetyConfimed == true ){

                        // If stay for a while after safety confirmed, the situation may be changed, so check again
                        if( memory.speedZeroCount * calInterval > param.safetyConfirmTime ){
                            memory.safetyConfimed = false;
                        }
                        else{
                            memory.doStopControl = false;
                        }

                    }
                    else{

                        // Check cross vehicles
                        int checkKind = 0;
                        if( pRoad->LeftOrRight == 0 ){

                            if( memory.rightCrossIsClear == false ){
                                checkKind = 2;
                                memory.rightCrossCheckCount = 0;
                            }
                            else if( memory.rightCrossIsClear == true ){
                                checkKind = 2;
                                memory.rightCrossCheckCount++;
                                if( memory.rightCrossCheckCount * calInterval > param.safetyConfirmTime ){
                                    checkKind = -1;
                                }

                                if( memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING ){

                                    // If taking much time to confirm left-cross, the situation of right-cross may be changed, check again
                                    if( memory.leftCrossCheckCount * calInterval > param.safetyConfirmTime * 2.5 ){
                                        checkKind = 2;
                                        memory.rightCrossCheckCount = 0;
                                    }
                                }
                            }

                        }
                        else if( pRoad->LeftOrRight == 1 ){

                            if( memory.leftCrossIsClear == false ){
                                checkKind = 1;
                                memory.leftCrossCheckCount = 0;
                            }
                            else if( memory.leftCrossIsClear == true ){
                                checkKind = 1;
                                memory.leftCrossCheckCount++;
                                if( memory.leftCrossCheckCount * calInterval > param.safetyConfirmTime ){
                                    checkKind = -1;
                                }

                                if( memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING ){
                                    // If taking much time to confirm right-cross, the situation of left-cross may be changed, check again
                                    if( memory.leftCrossCheckCount * calInterval > param.safetyConfirmTime * 2.5 ){
                                        checkKind = 1;
                                        memory.leftCrossCheckCount = 0;
                                    }
                                }
                            }
                        }

//                        strForDebug += QString("[1]checkKind=%1\n").arg(checkKind);

                        if( pRoad->LeftOrRight == 0 && memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING && checkKind == -1 ){

                            if( memory.leftCrossIsClear == false ){
                                checkKind = 1;
                                memory.leftCrossCheckCount = 0;
                            }
                            else if( memory.leftCrossIsClear == true ){
                                checkKind = 1;
                                memory.leftCrossCheckCount++;
                                if( memory.leftCrossCheckCount * calInterval > param.safetyConfirmTime ){
                                    checkKind = -1;
                                }
                            }
                        }
                        else if( pRoad->LeftOrRight == 1 && memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING && checkKind == -1 ){

                            if( memory.rightCrossIsClear == false ){
                                checkKind = 2;
                                memory.rightCrossCheckCount = 0;
                            }
                            else if( memory.rightCrossIsClear == true ){
                                checkKind = 2;
                                memory.rightCrossCheckCount++;
                                if( memory.rightCrossCheckCount * calInterval > param.safetyConfirmTime ){
                                    checkKind = -1;
                                }
                            }
                        }

//                        strForDebug += QString("[2]checkKind=%1\n").arg(checkKind);

                        if( checkKind == -1 ){
                            memory.safetyConfimed = true;
                            memory.doStopControl = false;
                            memory.speedZeroCount = 0;
                        }
                        else if( checkKind == 1 ){

                            memory.leftCrossIsClear = true;

                            for(int i=0;i<memory.perceptedObjects.size();++i){

                                if( memory.perceptedObjects[i]->isValidData == false ){
                                    continue;
                                }
                                if( memory.perceptedObjects[i]->objectType >= 100 ){
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->recognitionLabel != AGENT_RECOGNITION_LABEL::LEFT_CROSSING_STRAIGHT &&
                                        memory.perceptedObjects[i]->recognitionLabel != AGENT_RECOGNITION_LABEL::LEFT_CROSSING_LEFT &&
                                        memory.perceptedObjects[i]->recognitionLabel != AGENT_RECOGNITION_LABEL::LEFT_CROSSING_RIGHT ){
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->hasCollisionPoint == false ){
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->CPinNode != memory.nextTurnNode ){
                                    continue;
                                }

                                // Reject fat vehicle
                                if( memory.perceptedObjects[i]->objectTimeToCP > 8.0 && memory.perceptedObjects[i]->objectDistanceToCP > 40.0 ){
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->objectDistanceToCP > 0.0 &&
                                        memory.perceptedObjects[i]->objectDistanceToCP < 10.0 ){

                                    memory.leftCrossIsClear = false;
                                    memory.leftCrossCheckCount = 0;
                                    break;
                                }

                                float decelTime = memory.perceptedObjects[i]->V / param.accelControlGain;
                                float myTime = memory.perceptedObjects[i]->myDistanceToCP / 2.0;  // Assuem average speed of 2[m/s] to move to collision point
                                if( state.V > 2.0 ){
                                    myTime = memory.perceptedObjects[i]->myDistanceToCP / state.V;
                                }

                                float diffTime = myTime - memory.perceptedObjects[i]->objectTimeToCP;
                                if( diffTime > param.crossTimeSafetyMargin ){
                                    // I can after left-cross vehicle
                                    continue;
                                }
                                else if( diffTime < (decelTime + param.crossTimeSafetyMargin) * -1.0 ){
                                    // I can go faster
                                    continue;
                                }

//                                strForDebug += QString("[1]Crossing Risk by ID%1\n").arg(  memory.perceptedObjects[i]->objectID );

                                memory.leftCrossIsClear = false;
                                memory.leftCrossCheckCount = 0;
                                break;
                            }
                        }
                        else if( checkKind == 2 ){

                            memory.rightCrossIsClear = true;

                            for(int i=0;i<memory.perceptedObjects.size();++i){

                                if( memory.perceptedObjects[i]->isValidData == false ){
                                    continue;
                                }
                                if( memory.perceptedObjects[i]->objectType >= 100 ){
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->recognitionLabel != AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_STRAIGHT &&
                                        memory.perceptedObjects[i]->recognitionLabel != AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_LEFT &&
                                        memory.perceptedObjects[i]->recognitionLabel != AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_RIGHT ){
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->hasCollisionPoint == false ){
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->CPinNode != memory.nextTurnNode ){
                                    continue;
                                }

                                // Reject fat vehicle
                                if( memory.perceptedObjects[i]->objectTimeToCP > 8.0 && memory.perceptedObjects[i]->objectDistanceToCP > 40.0 ){
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->objectDistanceToCP > 0.0 &&
                                        memory.perceptedObjects[i]->objectDistanceToCP < 10.0 ){

                                    memory.rightCrossIsClear = false;
                                    memory.rightCrossCheckCount = 0;
                                    break;
                                }

                                float decelTime = memory.perceptedObjects[i]->V / param.accelControlGain;

                                float myTime = memory.perceptedObjects[i]->myDistanceToCP / 2.0;  // Assuem average speed of 2[m/s] to move to collision point
                                if( state.V > 2.0 ){
                                    myTime = memory.perceptedObjects[i]->myDistanceToCP / state.V;
                                }

                                float diffTime = myTime - memory.perceptedObjects[i]->objectTimeToCP;
                                if( diffTime > param.crossTimeSafetyMargin ){
                                    // I can after left-cross vehicle
                                    continue;
                                }
                                else if( diffTime < (decelTime + param.crossTimeSafetyMargin) * -1.0 ){
                                    // I can go faster
                                    continue;
                                }

//                                strForDebug += QString("[2]Crossing Risk by ID%1\n").arg(  memory.perceptedObjects[i]->objectID );

                                memory.rightCrossIsClear = false;
                                memory.rightCrossCheckCount = 0;
                                break;
                            }
                        }
                    }
                }
            }
        }

        return;
    }

}

