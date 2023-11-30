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


void Agent::RiskEvaluation(Agent** pAgent, int maxAgent, Road* pRoad,QList<TrafficSignal*> trafficSignal)
{

    if( decisionMakingCount != 0 ){
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

                    memory.halfLenPrecedingVehicle = memory.perceptedObjects[i]->vHalfLength;
                    memory.precedingVehicleIndex   = i;


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
    }




    if(  memory.controlMode == AGENT_CONTROL_MODE::AGENT_LOGIC ){

        if( state.V < 1.0 ){
            memory.speedZeroCount++;
        }
        else{
            memory.speedZeroCount = 0;
        }



        memory.ignoreCollisonAvoidanceCheckObject.clear();


        //
        // To reduce calculation
        strForDebugRiskEval += QString("onlyCheckPreceding = %1\n").arg( onlyCheckPreceding );

        if( onlyCheckPreceding == true && memory.precedingVehicleID >= 0 && memory.precedingVehicleID < maxAgent && memory.checkSideVehicleForLC == false ){

            // Check Preceding Vehicle Speed
            memory.speedPrecedingVehicle = pAgent[memory.precedingVehicleID]->state.V;

            for(int i=0;i<memory.perceptedObjects.size();++i){
                if( memory.perceptedObjects[i]->isValidData == false ){
                    continue;
                }
                if( memory.perceptedObjects[i]->objectID != memory.precedingVehicleID ){
                    continue;
                }

                memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->distanceToObject;

                if( memory.distanceToPrecedingVehicle >= 0.0 ){
                    memory.distanceToPrecedingVehicle -= memory.perceptedObjects[i]->vHalfLength;
                    memory.distanceToPrecedingVehicle -= vHalfLength;
                }

                memory.axPrecedingVehicle    = memory.perceptedObjects[i]->Ax;

                memory.halfLenPrecedingVehicle = memory.perceptedObjects[i]->vHalfLength;
                memory.precedingVehicleIndex   = i;

                break;
            }

            return;
        }

        memory.causeOfStopControl = QString("---");

        strForDebugRiskEval += QString("shouldStopAtSignalSL = %1\n").arg( memory.shouldStopAtSignalSL );
        strForDebugRiskEval += QString("releaseStopCount = %1\n").arg( memory.releaseStopCount );


        memory.distanceToStopPoint = memory.requiredDistToStopFromTargetSpeed;
        strForDebugRiskEval += QString("Reset: distanceToStopPoint= %1\n").arg( memory.distanceToStopPoint );


        //
        // Check to release stop flag
        if( memory.shouldStopAtSignalSL == true || (memory.doStopControl == true && memory.releaseStopCount == -1) ){

            memory.releaseStopCount++;

            strForDebugRiskEval += QString("releaseStopCount = %1\n").arg( memory.releaseStopCount );


            // memory.releaseStopCount == 0 is added to move from temporal stop, yeilding or confirmation of safety,
            // without delay
            if( memory.releaseStopCount == 0 || memory.releaseStopCount * calInterval * decisionMakingCountMax >=param.startRelay ){
                memory.shouldStopAtSignalSL = false;
                memory.releaseStopCount = 0;
                memory.doStopControl = false;
            }
        }


        int turnIntersectionTSValue = 0;

        //
        // Risk evaluation for Traffic Signal
        if( memory.perceptedSignals.size() > 0 ){

            if( agentKind < 100 ){


                float minDist = -1000.0;

                bool SignalIsBlue = false;

                int nearestTS = -1;
                float distToNearestTS = 0.0;
                for(int i=0;i<memory.perceptedSignals.size();++i){
                    if( memory.perceptedSignals[i]->isValidData == false ){
                        continue;
                    }
                    if( memory.perceptedSignals[i]->distToSL < -vHalfLength &&
                            memory.perceptedSignals[i]->relatedNode == memory.currentTargetNode ){
                        continue;
                    }
                    if( nearestTS < 0 || distToNearestTS > memory.perceptedSignals[i]->distToSL ){
                        nearestTS = i;
                        distToNearestTS = memory.perceptedSignals[i]->distToSL;
                    }
                }

                for(int i=0;i<memory.perceptedSignals.size();++i){

                    if( memory.perceptedSignals[i]->isValidData == false ){
                        continue;
                    }

                    strForDebugRiskEval += QString("Check Signal %1, dist=%2\n").arg( memory.perceptedSignals[i]->objectID ).arg(memory.perceptedSignals[i]->distToSL);

                    if( memory.nextTurnNode == memory.perceptedSignals[i]->relatedNode ){
                        turnIntersectionTSValue = memory.perceptedSignals[i]->signalDisplay;
                    }

                    bool alreadyPassed = false;
                    if( memory.nextTurnNode == memory.perceptedSignals[i]->relatedNode &&
                            ( memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING  ||
                              memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING ) ){

                        if( memory.perceptedSignals[i]->distToSL < 0.0 && memory.shouldStopAtSignalSL == false ){
                            // Get current signal color to be used later
                            alreadyPassed = true;
                        }
                    }

                    if( memory.perceptedSignals[i]->distToSL < 0.0 && memory.currentTargetNode != memory.perceptedSignals[i]->relatedNode ){
                        alreadyPassed = true;
                    }

                    if( memory.distToNearestCP < 0.0 && memory.perceptedSignals[i]->relatedNode == memory.nearCPInNode ){
                        alreadyPassed = true;
                    }

                    if( memory.perceptedSignals[i]->distToSL < 0.0 && memory.distanceToNodeWPOut < -(memory.perceptedSignals[i]->distToSL)
                            && memory.currentTargetNode == memory.perceptedSignals[i]->relatedNode ){
                        alreadyPassed = true;
                    }

                    strForDebugRiskEval += QString("alreadyPassed = %1\n").arg( alreadyPassed );

                    if( alreadyPassed == true ){
                        continue;
                    }

                    int relatedNode = memory.perceptedSignals[i]->relatedNode;

                    // get turn direction of relatedNode
                    int myTurnDir = DIRECTION_LABEL::STRAIGHT;
                    if( relatedNode == memory.nextTurnNode ){
                        if( memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
                            myTurnDir = DIRECTION_LABEL::RIGHT_CROSSING;
                        }
                        else if( memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING ){
                            myTurnDir = DIRECTION_LABEL::LEFT_CROSSING;
                        }
                    }


                    int signalColor = memory.perceptedSignals[i]->signalDisplay;

                    bool isGreen    = ((signalColor & 0x01) == 0x01);
                    bool isYellow   = ((signalColor & 0x02) == 0x02);
                    bool isRed      = ((signalColor & 0x04) == 0x04);
                    bool isLeft     = ((signalColor & 0x08) == 0x08);
                    bool isStraight = ((signalColor & 0x10) == 0x10);
                    bool isRight    = ((signalColor & 0x20) == 0x20);

                    bool canGo      = false;
                    bool checkStop  = false;
                    bool shouldStop = false;

                    if( isGreen == true ||
                            (isRed == true && myTurnDir == DIRECTION_LABEL::STRAIGHT && isStraight == true ) ||
                            (isRed == true && myTurnDir == DIRECTION_LABEL::LEFT_CROSSING && isLeft == true) ||
                            (isRed == true && myTurnDir == DIRECTION_LABEL::RIGHT_CROSSING && isRight == true) ){
                        canGo = true;
                    }

                    if( memory.shouldStopAtSignalSL == false && isYellow == true ){
                        checkStop = true;
                    }
                    else if( memory.shouldStopAtSignalSL == true && isYellow == true ){
                        shouldStop = true;
                    }

                    if( (isRed == true && myTurnDir == DIRECTION_LABEL::STRAIGHT && isStraight == false ) ||
                            (isRed == true && myTurnDir == DIRECTION_LABEL::LEFT_CROSSING && isLeft == false) ||
                            (isRed == true && myTurnDir == DIRECTION_LABEL::RIGHT_CROSSING && isRight == false) ){
                        shouldStop = true;
                    }


                    strForDebugRiskEval += QString("isGreen = %1\n").arg( isGreen );
                    strForDebugRiskEval += QString("isYellow = %1\n").arg( isYellow );
                    strForDebugRiskEval += QString("isRed = %1\n").arg( isRed );
                    strForDebugRiskEval += QString("isLeft = %1\n").arg( isLeft );
                    strForDebugRiskEval += QString("isStraight = %1\n").arg( isStraight );
                    strForDebugRiskEval += QString("isRight = %1\n").arg( isRight );
                    strForDebugRiskEval += QString("canGo = %1\n").arg( canGo );
                    strForDebugRiskEval += QString("checkStop = %1\n").arg( checkStop );
                    strForDebugRiskEval += QString("shouldStop = %1\n").arg( shouldStop );


                    if( checkStop == true ){

                        // Not to stay turn vehicle inside intersection
                        if( pRoad->LeftOrRight == 0 && myTurnDir == DIRECTION_LABEL::RIGHT_CROSSING &&
                                memory.shouldStopAtSignalSL == false && memory.perceptedSignals[i]->distToSL < 0.0 ){
                            continue;
                        }
                        else if( pRoad->LeftOrRight == 1 && myTurnDir == DIRECTION_LABEL::LEFT_CROSSING &&
                                memory.shouldStopAtSignalSL == false && memory.perceptedSignals[i]->distToSL < 0.0 ){
                            continue;
                        }

                        if( memory.shouldStopAtSignalSL == true || memory.perceptedSignals[i]->distToSL > memory.distanceToZeroSpeed + 5.0 ){

                            if( minDist < 0.0 || minDist > memory.perceptedSignals[i]->distToSL ){

                                minDist = memory.perceptedSignals[i]->distToSL;

                                strForDebugRiskEval += QString("[Y]signalColor = %1\n").arg( signalColor );

                                memory.shouldStopAtSignalSL = true;
                                memory.releaseStopCount = 0;
                            }
                        }
                    }
                    else if( shouldStop == true ){

                        // Not to stay turn vehicle inside intersection
                        if( pRoad->LeftOrRight == 0 && myTurnDir == DIRECTION_LABEL::RIGHT_CROSSING &&
                                memory.shouldStopAtSignalSL == false && memory.perceptedSignals[i]->distToSL < 0.0 ){
                            continue;
                        }
                        else if( pRoad->LeftOrRight == 1 && myTurnDir == DIRECTION_LABEL::LEFT_CROSSING &&
                                memory.shouldStopAtSignalSL == false && memory.perceptedSignals[i]->distToSL < 0.0 ){
                            continue;
                        }

                        if( memory.shouldStopAtSignalSL == true ||
                                memory.distToNearestCP > memory.distanceToZeroSpeedByMaxBrake ||
                                memory.perceptedSignals[i]->distToSL > memory.distanceToZeroSpeedByMaxBrake ){

                            if( minDist < 0.0 || minDist > memory.perceptedSignals[i]->distToSL ){

                                minDist = memory.perceptedSignals[i]->distToSL;

                                strForDebugRiskEval += QString("[R]signalColor = %1\n").arg( signalColor );

                                memory.shouldStopAtSignalSL = true;
                                memory.releaseStopCount = 0;
                            }
                        }
                    }
                    else if( canGo == true ){
                        if( nearestTS == i ){
                            SignalIsBlue = true;
                        }
                    }
                }


                if( memory.shouldStopAtSignalSL == true && minDist > -1000.0 ){
                    memory.doStopControl = true;
					memory.distanceToStopPoint = minDist - vHalfLength;
                    memory.causeOfStopControl = QString("Signal");
                }


                int cNDIdx = pRoad->nodeId2Index.indexOf(memory.currentTargetNode);
                int isTSNode = false;
                if( cNDIdx >= 0){
                    isTSNode = pRoad->nodes[ cNDIdx ]->hasTS;
                }

                bool ShouldStop = false;


                if( memory.shouldStopAtSignalSL == true && SignalIsBlue == true && isTSNode == true &&
                        memory.distToNearestCP > 1.5 + vHalfLength && memory.distanceToCurrentTargetNodeWPOut > memory.distanceToCurrentTargetNodeWPIn ){

                    // Check remaining cross vehicles inside intersection
                    for(int i=0;i<memory.perceptedObjects.size();++i){

                        if( memory.perceptedObjects[i]->isValidData == false ){
                            continue;
                        }

                        if( memory.currentTargetNode != memory.perceptedObjects[i]->objectTargetNode ){
                            continue;
                        }


                        int objID = memory.perceptedObjects[i]->objectID;
                        if( pAgent[objID]->memory_reference.distanceToCurrentTargetNodeWPIn > 0.0 ||
                                pAgent[objID]->memory_reference.distanceToStopPoint > 0.0 ){
                            continue;
                        }

                        if( memory.perceptedObjects[i]->distanceToObject < 0.0 ){
                            continue;
                        }

                        if( pRoad->LeftOrRight == 0 &&
                            (memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_RIGHT ||
                            memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_RIGHT ) ){

                            if( memory.nextTurnNode == memory.currentTargetNode && memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING ){
                                continue;
                            }

                            if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_RIGHT ){
                                if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < -1.5 - memory.perceptedObjects[i]->vHalfLength ){
                                    continue;
                                }
                                else if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 1.75 + memory.perceptedObjects[i]->vHalfLength && memory.perceptedObjects[i]->V < 0.1 ){
                                    continue;
                                }
                            }
                            else if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_RIGHT ){
                                if( memory.nextTurnNode == memory.currentTargetNode && memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 1.5 + memory.perceptedObjects[i]->vHalfLength ){
                                        continue;
                                    }
                                    else if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < -1.75 - memory.perceptedObjects[i]->vHalfLength && memory.perceptedObjects[i]->V < 0.1 ){
                                        continue;
                                    }
                                }
                                else{
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > -1.5 + memory.perceptedObjects[i]->vHalfLength ){
                                        continue;
                                    }
                                    else if( memory.perceptedObjects[i]->deviationFromNearestTargetPath <= -1.5 - memory.perceptedObjects[i]->vHalfLength && memory.perceptedObjects[i]->V < 0.1 ){
                                        continue;
                                    }
                                }
                            }

                            memory.releaseStopCount = 0;
                            memory.causeOfStopControl = QString("Remaining Cross LCR/RCR Vehicle: ID=%1").arg( memory.perceptedObjects[i]->objectID );
                            memory.hazardusObject = memory.perceptedObjects[i]->objectID;

                            ShouldStop = true;
                            break;
                        }

                        if( pRoad->LeftOrRight == 1 &&
                             (memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_LEFT ||
                              memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_LEFT ) ){

                            if( memory.nextTurnNode == memory.currentTargetNode && memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
                                continue;
                            }

                            if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_RIGHT ){
                                if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 1.5 + memory.perceptedObjects[i]->vHalfLength ){
                                    continue;
                                }
                            }
                            else if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_LEFT ){
                                if( memory.nextTurnNode == memory.currentTargetNode && memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING ){
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < -1.5 - memory.perceptedObjects[i]->vHalfLength ){
                                        continue;
                                    }
                                }
                                else{
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < +1.5 + memory.perceptedObjects[i]->vHalfLength ){
                                        continue;
                                    }
                                }
                            }

                            memory.releaseStopCount = 0;
                            memory.causeOfStopControl = QString("Remaining Cross RCL/LCL Vehicle: ID=%1").arg( memory.perceptedObjects[i]->objectID );
                            memory.hazardusObject = memory.perceptedObjects[i]->objectID;

                            ShouldStop = true;
                            break;
                        }

                        if( pRoad->LeftOrRight == 0 &&
                            (memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_STRAIGHT ||
                            memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_STRAIGHT) ){

                            if( memory.nextTurnNode == memory.currentTargetNode && memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING ){
                                if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_STRAIGHT ){
                                    continue;
                                }
                                else{
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 1.5 + memory.perceptedObjects[i]->vHalfLength ){
                                        continue;
                                    }
                                    else if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < -1.5 - memory.perceptedObjects[i]->vHalfLength && memory.perceptedObjects[i]->V < 0.1 ){
                                        continue;
                                    }
                                }
                            }
                            else if( memory.nextTurnNode == memory.currentTargetNode && memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
                                if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_STRAIGHT ){
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < 0.0 ){
                                        continue;
                                    }
                                    else if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 1.5 + memory.perceptedObjects[i]->vHalfLength && memory.perceptedObjects[i]->V < 0.1 ){
                                        continue;
                                    }
                                }
                                else{
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 1.5 + memory.perceptedObjects[i]->vHalfLength ){
                                        continue;
                                    }
                                    else if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < -1.5 - memory.perceptedObjects[i]->vHalfLength && memory.perceptedObjects[i]->V < 0.1 ){
                                        continue;
                                    }
                                }
                            }
                            else{
                                if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_STRAIGHT ){
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < -1.5 - memory.perceptedObjects[i]->vHalfLength ){
                                        continue;
                                    }
                                    else if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 1.5 + memory.perceptedObjects[i]->vHalfLength && memory.perceptedObjects[i]->V < 0.1 ){
                                        continue;
                                    }
                                }
                                else{
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 1.5 + memory.perceptedObjects[i]->vHalfLength ){
                                        continue;
                                    }
                                    else if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < -1.5 - memory.perceptedObjects[i]->vHalfLength && memory.perceptedObjects[i]->V < 0.1 ){
                                        continue;
                                    }
                                }
                            }

                            memory.releaseStopCount = 0;
                            memory.causeOfStopControl = QString("Remaining Cross LCS/RCS Vehicle: ID=%1").arg( memory.perceptedObjects[i]->objectID );
                            memory.hazardusObject = memory.perceptedObjects[i]->objectID;

                            ShouldStop = true;
                            break;
                        }

                        if( pRoad->LeftOrRight == 1 &&
                            (memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_STRAIGHT ||
                            memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_STRAIGHT) ){

                            if( memory.nextTurnNode == memory.currentTargetNode && memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
                                if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_STRAIGHT ){
                                    continue;
                                }
                                else{
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < -1.5 - memory.perceptedObjects[i]->vHalfLength ){
                                        continue;
                                    }
                                }
                            }
                            else if( memory.nextTurnNode == memory.currentTargetNode && memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING ){
                                if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_STRAIGHT ){
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 0.0 ){
                                        continue;
                                    }
                                }
                                else{
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < -1.5 - memory.perceptedObjects[i]->vHalfLength ){
                                        continue;
                                    }
                                }
                            }
                            else{
                                if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_STRAIGHT ){
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 1.5 + memory.perceptedObjects[i]->vHalfLength ){
                                        continue;
                                    }
                                }
                                else{
                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < -1.5 - memory.perceptedObjects[i]->vHalfLength ){
                                        continue;
                                    }
                                }
                            }

                            memory.releaseStopCount = 0;
                            memory.causeOfStopControl = QString("Remaining Cross LCS/RCS Vehicle: ID=%1").arg( memory.perceptedObjects[i]->objectID );
                            memory.hazardusObject = memory.perceptedObjects[i]->objectID;

                            ShouldStop = true;
                            break;
                        }


                        if( pRoad->LeftOrRight == 0 &&
                            (memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_LEFT ||
                            memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_LEFT ) ){

                            if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_LEFT ){
                                continue;
                            }

                            if( memory.nextTurnNode == memory.currentTargetNode &&
                                    ( memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING || memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING )  ){
                                continue;
                            }
                            else{
                                if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < 1.5 + memory.perceptedObjects[i]->vHalfLength ){
                                    continue;
                                }
                                else if( memory.perceptedObjects[i]->deviationFromNearestTargetPath >= 1.5 + memory.perceptedObjects[i]->vHalfLength && memory.perceptedObjects[i]->V < 0.1 ){
                                    continue;
                                }
                            }

                            memory.releaseStopCount = 0;
                            memory.causeOfStopControl = QString("Remaining Cross LCL/RCL Vehicle: ID=%1").arg( memory.perceptedObjects[i]->objectID );
                            memory.hazardusObject = memory.perceptedObjects[i]->objectID;

                            ShouldStop = true;
                            break;
                        }

                        if( pRoad->LeftOrRight == 1 &&
                            (memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_RIGHT ||
                            memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_RIGHT ) ){

                            if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_RIGHT ){
                                continue;
                            }

                            if( memory.nextTurnNode == memory.currentTargetNode &&
                                    ( memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING  || memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING ) ){
                                continue;
                            }
                            else{
                                if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > -1.5 - memory.perceptedObjects[i]->vHalfLength ){
                                    continue;
                                }
                            }

                            memory.releaseStopCount = 0;
                            memory.causeOfStopControl = QString("Remaining Cross LCR/RCR Vehicle: ID=%1").arg( memory.perceptedObjects[i]->objectID );
                            memory.hazardusObject = memory.perceptedObjects[i]->objectID;

                            ShouldStop = true;
                            break;
                        }
                    }

                    strForDebugRiskEval += QString("Remaining Cross Check: ShouldStop = %1\n").arg( ShouldStop );

                }

                if( memory.shouldStopAtSignalSL == true && SignalIsBlue == true && ShouldStop == false ){
                    memory.shouldStopAtSignalSL = true;
                    if( memory.doStopControl == true ){
                        memory.doStopControl  = false;
                    }
                }

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

//                    strForDebugRiskEval += QString("sIdx = %1,  szSect = %2  , currentDisplay = %3\n").arg( sIdx ).arg( szSect ).arg( currentDisplay );


                    if( sIdx + 1 < szSect ){

                        if(pRoad->pedestPaths[plIdx]->shape[sIdx]->isCrossWalk == true  ){

                           float dx = state.x - pRoad->pedestPaths[plIdx]->shape[sIdx]->pos.x();
                           float dy = state.y - pRoad->pedestPaths[plIdx]->shape[sIdx]->pos.y();
                           float S = dx * (pRoad->pedestPaths[plIdx]->shape[sIdx]->cosA) + dy * (pRoad->pedestPaths[plIdx]->shape[sIdx]->sinA);

//                           strForDebugRiskEval += QString("S = %1\n").arg( S );

                           if( S < 1.0 ){
                               if( currentDisplay == TRAFFICSIGNAL_RED ){
                                   memory.shouldStopAtSignalSL = true;
//                                   strForDebugRiskEval += QString("[1] shouldStopAtSignalSL = true\n");
                               }
                               else{
                                   memory.shouldStopAtSignalSL = false;
//                                   strForDebugRiskEval += QString("[2] shouldStopAtSignalSL = false\n");
                               }
                           }
                           else{
                               memory.shouldStopAtSignalSL = false;
//                               strForDebugRiskEval += QString("[3] shouldStopAtSignalSL = false\n");
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

//                            strForDebugRiskEval += QString("sigIdx = %1\n").arg( sigIdx );

                            if( sigIdx > 0 ){

                                float distToSig = 0.0;
                                for(int i=sigIdx-1;i>=sIdx;i--){
                                    if( i == sIdx ){
                                        float dx = state.x - pRoad->pedestPaths[plIdx]->shape[i]->pos.x();
                                        float dy = state.y - pRoad->pedestPaths[plIdx]->shape[i]->pos.y();
                                        float S = dx * (pRoad->pedestPaths[plIdx]->shape[i]->cosA) + dy * (pRoad->pedestPaths[plIdx]->shape[i]->sinA);

//                                        strForDebugRiskEval += QString("distToSig = %1\n").arg( distToSig );
//                                        strForDebugRiskEval += QString("distanceToNextPos = %1\n").arg( pRoad->pedestPaths[plIdx]->shape[i]->distanceToNextPos );
//                                        strForDebugRiskEval += QString("S = %1\n").arg( S );

                                        if( distToSig + pRoad->pedestPaths[plIdx]->shape[i]->distanceToNextPos - S < 2.5 ){
                                            if( currentDisplay == TRAFFICSIGNAL_RED ){
                                                memory.shouldStopAtSignalSL = true;
//                                                strForDebugRiskEval += QString("[4] shouldStopAtSignalSL = true\n");
                                            }
                                            else{
                                                memory.shouldStopAtSignalSL = false;
//                                                strForDebugRiskEval += QString("[5] shouldStopAtSignalSL = false\n");
                                            }
                                        }
                                        else{
                                            memory.shouldStopAtSignalSL = false;
//                                            strForDebugRiskEval += QString("[6] shouldStopAtSignalSL = false\n");
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
//                        strForDebugRiskEval += QString("[0]shouldStopAtSignalSL = false\n");
                    }
                }
            }
        }
        else{
            memory.shouldStopAtSignalSL = false;
        }


        bool setLateralShiftTarget = false;

        //
        // Risk evaluation for preceding vehicle
        if( agentKind < 100 ){

            memory.doHeadwayDistanceControl = false;
            memory.precedingVehicleID = -1;
            memory.distanceToPrecedingVehicle = 0.0;
            memory.precedingObstacle = 0;
            memory.halfLenPrecedingVehicle = vHalfLength;
            memory.precedingVehicleIndex = -1;

            if( memory.runOncomingLane == true ){
                memory.lateralShiftTarget = -2.5;
            }

            if( memory.avoidTarget >= 0) {
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


            // Check Labeled As Preceding
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

                    strForDebugRiskEval += QString("[P]V%1 by Label\n").arg( memory.perceptedObjects[i]->objectID );

                    if( memory.precedingVehicleID < 0 ||  memory.distanceToPrecedingVehicle > memory.perceptedObjects[i]->distanceToObject ){

                        memory.precedingVehicleID         = memory.perceptedObjects[i]->objectID;
                        memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->distanceToObject;
                        memory.speedPrecedingVehicle      = memory.perceptedObjects[i]->V;
                        memory.axPrecedingVehicle         = memory.perceptedObjects[i]->Ax;
                        memory.halfLenPrecedingVehicle    = memory.perceptedObjects[i]->vHalfLength;
                        memory.precedingVehicleIndex      = i;
                        memory.precedingObstacle          = 0;
                    }
                }
            }


            QList<float> distToSideLaneMergePoint;

            for(int j=0;j<memory.laneMerge.size();++j){

                strForDebugRiskEval += QString("laneMerge = %1 and %2\n").arg( memory.laneMerge[j].x() ).arg( memory.laneMerge[j].y() );

                int cIdx = memory.targetPathList.indexOf( memory.laneMerge[j].x() );
                if( cIdx < 0 ){
                    cIdx = memory.targetPathList.indexOf( memory.laneMerge[j].y() );
                }
                if( cIdx < 0 || cIdx > memory.currentTargetPathIndexInList ){
                    distToSideLaneMergePoint.append( -1.0 );
                    continue;
                }
                float length = 0.0;
                for(int k=cIdx;k<=memory.currentTargetPathIndexInList;k++){
                    int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[k] );
                    length += pRoad->paths[pIdx]->pathLength;
                }
                length -= memory.distanceFromStartWPInCurrentPath;
                if( length < memory.requiredDistToStopFromTargetSpeed || length < 50.0 || cIdx == memory.currentTargetPathIndexInList ){
                    distToSideLaneMergePoint.append( length );
                }
                else{
                    distToSideLaneMergePoint.append( -1.0 );
                }

                strForDebugRiskEval += QString("length = %1\n").arg( length );
            }


            // Not Labeled As Preceding
            for(int i=0;i<memory.perceptedObjects.size();++i){

                if( memory.perceptedObjects[i]->isValidData == false ){
                    continue;
                }

                if( memory.perceptedObjects[i]->relPosEvaled == false ){
                    continue;
                }

//                if( pAgent[memory.perceptedObjects[i]->objectID]->isSInterfaceObject == true ){
//                    continue;
//                }

                if( memory.perceptedObjects[i]->objectType >= 100 ){
                    continue;
                }

                if( memory.perceptedObjects[i]->objectID == memory.ignoreHazardusObject ){
                    continue;
                }

                if( memory.perceptedObjects[i]->distanceToObject < (vHalfLength + memory.perceptedObjects[i]->vHalfLength + param.minimumHeadwayDistanceAtStop) * (-1.0) ){
                    continue;
                }

                if( memory.perceptedObjects[i]->recognitionLabel != PRECEDING ){

                    QList<int> alreadyCheckedList;

                    // Check if the side vehicle will be merged
                    for(int j=0;j<distToSideLaneMergePoint.size();++j){

                        strForDebugRiskEval += QString("distToSideLaneMergePoint[%1]=%2\n").arg( j ).arg(distToSideLaneMergePoint[j]);

                        if( distToSideLaneMergePoint[j] < 0.0 ){
                            continue;
                        }

                        if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_SIDE ||
                                memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_SIDE ||
                                memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_SIDE_PRECEDING ||
                                memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_SIDE_PRECEDING ){

                            int objID = memory.perceptedObjects[i]->objectID;
                            if( pAgent[objID]->memory_reference.targetPathList.size() > 0 ){

                                int checkPath = memory.laneMerge[j].y();
                                int sIdx = pAgent[objID]->memory_reference.targetPathList.indexOf( checkPath );
                                if( sIdx < 0 ){
                                    checkPath = memory.laneMerge[j].x();
                                    sIdx = pAgent[objID]->memory_reference.targetPathList.indexOf( checkPath );
                                }

                                strForDebugRiskEval += QString("V%1: check path=%2, sIdx=%3\n").arg( objID ).arg( checkPath ).arg(sIdx);

                                if( sIdx < 0 || (pAgent[objID]->memory_reference.currentTargetPathIndexInList >= 0 && sIdx > pAgent[objID]->memory_reference.currentTargetPathIndexInList) ){

                                    alreadyCheckedList.append( objID );

                                    continue;
                                }


                                if( pAgent[objID]->memory_reference.currentTargetPathIndexInList >= 0 &&
                                        pAgent[objID]->memory_reference.targetPathList.size() > 0 &&
                                        pAgent[objID]->memory_reference.currentTargetPathIndexInList < pAgent[objID]->memory_reference.targetPathList.size() ){

                                    float length = 0.0;
                                    for(int k=sIdx;k<=pAgent[objID]->memory_reference.currentTargetPathIndexInList;k++){
                                        int pIdx = pRoad->pathId2Index.indexOf( pAgent[objID]->memory_reference.targetPathList[k] );
                                        length += pRoad->paths[pIdx]->pathLength;
                                    }
                                    length -= pAgent[objID]->memory_reference.distanceFromStartWPInCurrentPath;

                                    strForDebugRiskEval += QString("length=%1 RefLength=%2\n").arg( length )
                                            .arg( pAgent[objID]->memory_reference.requiredDistToStopFromTargetSpeed );

                                    if( fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath) > 10.0 &&
                                            length > pAgent[objID]->memory_reference.requiredDistToStopFromTargetSpeed ){

                                        alreadyCheckedList.append( objID );

                                        continue;
                                    }

                                    strForDebugRiskEval += QString("SideMerge: V=%1, path=%2 length=%3\n").arg( objID ).arg(memory.laneMerge[j].y()).arg(length);

                                    float myTTC = distToSideLaneMergePoint[j] / (state.V + 0.1 );
                                    float objTTC = length / ( memory.perceptedObjects[i]->V + 0.1 );

                                    strForDebugRiskEval += QString("SmyTTC=%1 objTTC=%2\n").arg( myTTC ).arg( objTTC );

                                    if( memory.perceptedObjects[i]->distanceToObject > 0.0 && (myTTC < 6.0 || fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath)< 5.0) && (objTTC < myTTC || distToSideLaneMergePoint[j] > length) ){

                                        bool regardAsPreceding = false;
                                        if( memory.precedingVehicleID < 0 ){
                                            regardAsPreceding = true;
                                        }
                                        else{

                                            float relVp = state.V - memory.speedPrecedingVehicle;
                                            if( relVp < 0.1 ){
                                                relVp = 0.1;
                                            }
                                            float TTCp = memory.distanceToPrecedingVehicle / relVp;

                                            float relVt = state.V - memory.perceptedObjects[i]->filteredV;
                                            if( relVt < 0.1 ){
                                                relVt = 0.1;
                                            }
                                            float TTCt = memory.perceptedObjects[i]->distanceToObject / relVt;
                                            if( TTCt < 0.0 ){
                                                continue;
                                            }

                                            if( TTCp > TTCt ){
                                                regardAsPreceding = true;
                                            }

                                            float H = memory.perceptedObjects[i]->distanceToObject;
                                            H -= vHalfLength;
                                            H -= memory.perceptedObjects[i]->vHalfLength;

                                            if( H < param.minimumHeadwayDistanceAtStop &&
                                                    fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath) - memory.perceptedObjects[i]->effectiveHalfWidth < 1.5 ){

                                                regardAsPreceding = true;
                                            }

                                            strForDebugRiskEval += QString("[S]V%1 TTCt=%2 TTCp=%3 : PV=%4\n")
                                                    .arg( memory.perceptedObjects[i]->objectID )
                                                    .arg( TTCt )
                                                    .arg( TTCp )
                                                    .arg( memory.precedingVehicleID );

                                        }

                                        strForDebugRiskEval += QString("V%1 regardAsPreceding=%2 : PV=%3[A]\n")
                                                .arg( memory.perceptedObjects[i]->objectID )
                                                .arg( regardAsPreceding )
                                                .arg( memory.precedingVehicleID );

                                        if( regardAsPreceding == true ){

                                            memory.precedingVehicleID         = memory.perceptedObjects[i]->objectID;
                                            memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->distanceToObject;
                                            memory.speedPrecedingVehicle      = memory.perceptedObjects[i]->V;
                                            memory.axPrecedingVehicle         = memory.perceptedObjects[i]->Ax;
                                            memory.halfLenPrecedingVehicle    = memory.perceptedObjects[i]->vHalfLength;
                                            memory.precedingVehicleIndex      = i;
                                            memory.precedingObstacle          = 0;
                                        }
                                    }
                                }
                            }
                        }
                    }


                    // Check Merging vehicle
                    if( pRoad->LeftOrRight == 0 ){

                        // Check if to regard merging from left as preceding
                        if( memory.perceptedObjects[i]->mergingAsCP == true ){

                            int cpNdIdx = pRoad->nodeId2Index.indexOf( memory.perceptedObjects[i]->CPinNode );

                            if( cpNdIdx >= 0 ){

                                if( pRoad->nodes[cpNdIdx]->isMergeNode == true ||
                                        (memory.perceptedObjects[i]->hasCollisionPoint == true &&
                                         memory.perceptedObjects[i]->myDistanceToCP > memory.perceptedObjects[i]->objectDistanceToCP &&
                                         fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath) < 3.0 ) ){

                                    // Check if to regard merging from left as preceding
                                    if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_STRAIGHT ||
                                            memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_LEFT ){

                                        alreadyCheckedList.append( memory.perceptedObjects[i]->objectID );

                                        if( (memory.perceptedObjects[i]->myTimeToCP < 8.0 || memory.perceptedObjects[i]->myDistanceToCP < memory.requiredDistToStopFromTargetSpeed) &&
                                                (memory.perceptedObjects[i]->myTimeToCP > memory.perceptedObjects[i]->objectTimeToCP ||
                                                 memory.perceptedObjects[i]->myDistanceToCP > memory.perceptedObjects[i]->objectDistanceToCP ) &&
                                                fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath) < 5.0 ){

                                            bool regardAsPreceding = false;
                                            if( memory.precedingVehicleID < 0 ){
                                                regardAsPreceding = true;
                                            }
                                            else{

                                                float relVp = state.V - memory.speedPrecedingVehicle;
                                                if( relVp < 0.1 ){
                                                    relVp = 0.1;
                                                }
                                                float TTCp = memory.distanceToPrecedingVehicle / relVp;

                                                float relVt = state.V - memory.perceptedObjects[i]->V;
                                                if( relVt < 0.1 ){
                                                    relVt = 0.1;
                                                }
                                                float TTCt = (memory.perceptedObjects[i]->myDistanceToCP - memory.perceptedObjects[i]->objectDistanceToCP) / relVt;
                                                if( TTCt < 0.0 ){
                                                    continue;
                                                }

                                                if( TTCp > TTCt ){
                                                    regardAsPreceding = true;
                                                }

                                                float H = memory.perceptedObjects[i]->myDistanceToCP - memory.perceptedObjects[i]->objectDistanceToCP;
                                                H -= vHalfLength;
                                                H -= memory.perceptedObjects[i]->vHalfLength;

                                                if( H < param.minimumHeadwayDistanceAtStop &&
                                                        fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath) - memory.perceptedObjects[i]->effectiveHalfWidth < 1.5 ){

                                                    regardAsPreceding = true;
                                                }

                                                strForDebugRiskEval += QString("[1]V%1 TTCt=%2 TTCp=%3 : PV=%4\n")
                                                        .arg( memory.perceptedObjects[i]->objectID )
                                                        .arg( TTCt )
                                                        .arg( TTCp )
                                                        .arg( memory.precedingVehicleID );

                                            }

                                            strForDebugRiskEval += QString("V%1 regardAsPreceding=%2 : PV=%3[B]\n")
                                                    .arg( memory.perceptedObjects[i]->objectID )
                                                    .arg( regardAsPreceding )
                                                    .arg( memory.precedingVehicleID );

                                            if( regardAsPreceding == true ){

                                                memory.precedingVehicleID         = memory.perceptedObjects[i]->objectID;
                                                memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->myDistanceToCP - memory.perceptedObjects[i]->objectDistanceToCP;
                                                memory.speedPrecedingVehicle      = memory.perceptedObjects[i]->V;
                                                memory.axPrecedingVehicle         = memory.perceptedObjects[i]->Ax;
                                                memory.halfLenPrecedingVehicle    = memory.perceptedObjects[i]->vHalfLength;
                                                memory.precedingVehicleIndex      = i;
                                                memory.precedingObstacle          = 0;
                                            }
                                        }
                                    }

                                    // Check if to regard main lane vehicle as preceding
                                    if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_STRAIGHT ){

                                        alreadyCheckedList.append( memory.perceptedObjects[i]->objectID );

                                        if( (memory.perceptedObjects[i]->myTimeToCP < 8.0 || memory.perceptedObjects[i]->myDistanceToCP < memory.requiredDistToStopFromTargetSpeed) &&
                                                (memory.perceptedObjects[i]->myTimeToCP > (memory.perceptedObjects[i]->objectTimeToCP - 1.0) ||
                                                 memory.perceptedObjects[i]->myDistanceToCP > memory.perceptedObjects[i]->objectDistanceToCP - param.minimumHeadwayDistanceAtStop) ){

                                            bool regardAsPreceding = false;
                                            if( memory.precedingVehicleID < 0 ){
                                                regardAsPreceding = true;
                                            }
                                            else{

                                                strForDebugRiskEval += QString("[2]V%1 dist=%2 : PV=%3 dist=%4\n")
                                                        .arg( memory.perceptedObjects[i]->objectID )
                                                        .arg( memory.perceptedObjects[i]->distanceToObject )
                                                        .arg( memory.precedingVehicleID )
                                                        .arg( memory.distanceToPrecedingVehicle );

                                                if( memory.distanceToPrecedingVehicle < memory.perceptedObjects[i]->distanceToObject ){
                                                    continue;
                                                }

                                                float relVp = state.V - memory.speedPrecedingVehicle;
                                                if( relVp < 1.0 ){
                                                    relVp = 1.0;
                                                }
                                                float TTCp = memory.distanceToPrecedingVehicle / relVp;

                                                float relVt = state.V - memory.perceptedObjects[i]->V;
                                                if( relVt < 1.0 ){
                                                    relVt = 1.0;
                                                }
                                                float TTCt = (memory.perceptedObjects[i]->myDistanceToCP - memory.perceptedObjects[i]->objectDistanceToCP) / relVt;
                                                if( TTCt < 0.0 ){
                                                    continue;
                                                }

                                                if( TTCp > TTCt ){
                                                    regardAsPreceding = true;
                                                }

                                                if( memory.speedPrecedingVehicle < 1.0 && memory.perceptedObjects[i]->V < 1.0 ){
                                                    if( memory.distanceToPrecedingVehicle > memory.perceptedObjects[i]->distanceToObject ){
                                                        regardAsPreceding = true;
                                                    }
                                                }

                                                strForDebugRiskEval += QString("[2]V%1 TTCt=%2 TTCp=%3 : PV=%4\n")
                                                        .arg( memory.perceptedObjects[i]->objectID )
                                                        .arg( TTCt )
                                                        .arg( TTCp )
                                                        .arg( memory.precedingVehicleID );

                                            }

                                            strForDebugRiskEval += QString("V%1 regardAsPreceding=%2 : PV=%3[C]\n")
                                                    .arg( memory.perceptedObjects[i]->objectID )
                                                    .arg( regardAsPreceding )
                                                    .arg( memory.precedingVehicleID );

                                            if( regardAsPreceding == true ){

                                                memory.precedingVehicleID         = memory.perceptedObjects[i]->objectID;
                                                memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->myDistanceToCP - memory.perceptedObjects[i]->objectDistanceToCP;
                                                memory.speedPrecedingVehicle      = memory.perceptedObjects[i]->V;
                                                memory.axPrecedingVehicle         = memory.perceptedObjects[i]->Ax;
                                                memory.halfLenPrecedingVehicle    = memory.perceptedObjects[i]->vHalfLength;
                                                memory.precedingVehicleIndex      = i;
                                                memory.precedingObstacle          = 0;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    else{

                        // Check if to regard merging from left as preceding
                        if( memory.perceptedObjects[i]->mergingAsCP == true ){

                            int cpNdIdx = pRoad->nodeId2Index.indexOf( memory.perceptedObjects[i]->CPinNode );
                            if( cpNdIdx >= 0 ){
                                if( pRoad->nodes[cpNdIdx]->isMergeNode == true ||
                                        (memory.perceptedObjects[i]->hasCollisionPoint == true &&
                                         memory.perceptedObjects[i]->myDistanceToCP > memory.perceptedObjects[i]->objectDistanceToCP &&
                                         fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath) < 3.0) ){

                                    // Check if to regard merging from left as preceding
                                    if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_STRAIGHT ||
                                            memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_RIGHT ){

                                        alreadyCheckedList.append( memory.perceptedObjects[i]->objectID );

                                        if( (memory.perceptedObjects[i]->myTimeToCP < 8.0 || memory.perceptedObjects[i]->myDistanceToCP < memory.requiredDistToStopFromTargetSpeed) &&
                                                memory.perceptedObjects[i]->myTimeToCP > memory.perceptedObjects[i]->objectTimeToCP ){

                                            bool regardAsPreceding = false;
                                            if( memory.precedingVehicleID < 0 ){
                                                regardAsPreceding = true;
                                            }
                                            else{

                                                float relVp = state.V - memory.speedPrecedingVehicle;
                                                if( relVp < 0.1 ){
                                                    relVp = 0.1;
                                                }
                                                float TTCp = memory.distanceToPrecedingVehicle / relVp;

                                                float relVt = state.V - memory.perceptedObjects[i]->V;
                                                if( relVt < 0.1 ){
                                                    relVt = 0.1;
                                                }
                                                float TTCt = (memory.perceptedObjects[i]->myDistanceToCP - memory.perceptedObjects[i]->objectDistanceToCP) / relVt;
                                                if( TTCt < 0.0 ){
                                                    continue;
                                                }

                                                if( TTCp > TTCt ){
                                                    regardAsPreceding = true;
                                                }

                                                float H = memory.perceptedObjects[i]->myDistanceToCP - memory.perceptedObjects[i]->objectDistanceToCP;
                                                H -= vHalfLength;
                                                H -= memory.perceptedObjects[i]->vHalfLength;

                                                if( H < param.minimumHeadwayDistanceAtStop &&
                                                        fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath) - memory.perceptedObjects[i]->effectiveHalfWidth < 1.5 ){

                                                    regardAsPreceding = true;
                                                }
                                            }

                                            strForDebugRiskEval += QString("V%1 regardAsPreceding=%2[A]\n")
                                                    .arg( memory.perceptedObjects[i]->objectID )
                                                    .arg( regardAsPreceding );

                                            if( regardAsPreceding == true ){

                                                memory.precedingVehicleID         = memory.perceptedObjects[i]->objectID;
                                                memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->myDistanceToCP - memory.perceptedObjects[i]->objectDistanceToCP;
                                                memory.speedPrecedingVehicle      = memory.perceptedObjects[i]->V;
                                                memory.axPrecedingVehicle         = memory.perceptedObjects[i]->Ax;
                                                memory.halfLenPrecedingVehicle    = memory.perceptedObjects[i]->vHalfLength;
                                                memory.precedingVehicleIndex      = i;
                                                memory.precedingObstacle          = 0;
                                            }
                                        }
                                    }

                                    // Check if to regard main lane vehicle as preceding
                                    if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::LEFT_CROSSING_STRAIGHT ){

                                        alreadyCheckedList.append( memory.perceptedObjects[i]->objectID );

                                        // 2.0 is to yeild
                                        if( (memory.perceptedObjects[i]->myTimeToCP < 8.0 || memory.perceptedObjects[i]->myDistanceToCP < memory.requiredDistToStopFromTargetSpeed) &&
                                                (memory.perceptedObjects[i]->myTimeToCP + 2.0 > memory.perceptedObjects[i]->objectTimeToCP ||
                                                 memory.perceptedObjects[i]->myDistanceToCP < memory.perceptedObjects[i]->objectDistanceToCP) ){

                                            bool regardAsPreceding = false;
                                            if( memory.precedingVehicleID < 0 ){
                                                regardAsPreceding = true;
                                            }
                                            else{

                                                float relVp = state.V - memory.speedPrecedingVehicle;
                                                if( relVp < 0.1 ){
                                                    relVp = 0.1;
                                                }
                                                float TTCp = memory.distanceToPrecedingVehicle / relVp;

                                                float relVt = state.V - memory.perceptedObjects[i]->V;
                                                if( relVt < 0.1 ){
                                                    relVt = 0.1;
                                                }
                                                float TTCt = (memory.perceptedObjects[i]->myDistanceToCP - memory.perceptedObjects[i]->objectDistanceToCP) / relVt;
                                                if( TTCt < 0.0 ){
                                                    continue;
                                                }

                                                if( TTCp > TTCt ){
                                                    regardAsPreceding = true;
                                                }

                                                if( memory.speedPrecedingVehicle < 1.0 && memory.perceptedObjects[i]->V < 1.0 ){
                                                    if( memory.distanceToPrecedingVehicle > memory.perceptedObjects[i]->distanceToObject ){
                                                        regardAsPreceding = true;
                                                    }
                                                }

                                            }

                                            strForDebugRiskEval += QString("V%1 regardAsPreceding=%2[B]\n")
                                                    .arg( memory.perceptedObjects[i]->objectID )
                                                    .arg( regardAsPreceding );

                                            if( regardAsPreceding == true ){

                                                memory.precedingVehicleID         = memory.perceptedObjects[i]->objectID;
                                                memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->myDistanceToCP - memory.perceptedObjects[i]->objectDistanceToCP;
                                                memory.speedPrecedingVehicle      = memory.perceptedObjects[i]->V;
                                                memory.axPrecedingVehicle         = memory.perceptedObjects[i]->Ax;
                                                memory.halfLenPrecedingVehicle    = memory.perceptedObjects[i]->vHalfLength;
                                                memory.precedingVehicleIndex      = i;
                                                memory.precedingObstacle          = 0;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // Lane-Change vehicle
                    if( memory.perceptedObjects[i]->distanceToObject > 2.0 * state.V &&
                            memory.perceptedObjects[i]->recognitionLabel < AGENT_RECOGNITION_LABEL::ONCOMING_STRAIGHT ){

                        bool regardAsPreceding = false;
                        if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 1.5 &&
                                memory.perceptedObjects[i]->deviationFromNearestTargetPath < 4.0 &&
                                memory.perceptedObjects[i]->winker == DIRECTION_LABEL::RIGHT_CROSSING ){
                            regardAsPreceding = true;
                        }
                        else if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < -1.5 &&
                                 memory.perceptedObjects[i]->deviationFromNearestTargetPath > -4.0 &&
                                 memory.perceptedObjects[i]->winker == DIRECTION_LABEL::LEFT_CROSSING ){
                            regardAsPreceding = true;
                        }
                        if( regardAsPreceding == true ){

                            alreadyCheckedList.append( memory.perceptedObjects[i]->objectID );

                            if( memory.precedingVehicleID >= 0 ){

                                float relVp = memory.speedPrecedingVehicle - state.V;
                                if( relVp > -0.1 ){
                                    relVp = -0.1;
                                }
                                float relVt = memory.perceptedObjects[i]->V - state.V;
                                if( relVt > -0.1 ){
                                    relVt = -0.1;
                                }

                                float relDp = memory.distanceToPrecedingVehicle;
                                float relDt = memory.perceptedObjects[i]->distanceToObject;

                                float TTCp = relDp / (-relVp);
                                float TTCt = relDt / (-relVt);

                                strForDebugRiskEval += QString("[LC-P]V%1 TTCp=%2 TTCt=%3 : P=%4\n").arg( memory.perceptedObjects[i]->objectID ).arg( TTCp ).arg( TTCt ).arg( memory.precedingVehicleID );

                                if( TTCp < TTCt ){
                                    regardAsPreceding = false;
                                }

                            }

                            if( regardAsPreceding == true ){

                                strForDebugRiskEval += QString("[LC-P]V%1 by LC-Check\n").arg( memory.perceptedObjects[i]->objectID );

                                memory.precedingVehicleID         = memory.perceptedObjects[i]->objectID;
                                memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->myDistanceToCP - memory.perceptedObjects[i]->objectDistanceToCP;
                                memory.speedPrecedingVehicle      = memory.perceptedObjects[i]->V;
                                memory.axPrecedingVehicle         = memory.perceptedObjects[i]->Ax;
                                memory.halfLenPrecedingVehicle    = memory.perceptedObjects[i]->vHalfLength;
                                memory.precedingVehicleIndex      = i;
                                memory.precedingObstacle          = 0;
                            }
                        }

                    }



                    // If a front vehicle such as cross-merging and oncoming-merging comes into my lane, regard the vehile as preceding
                    if( memory.runOncomingLane == false &&
                            fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath - memory.lateralShiftTarget) - memory.perceptedObjects[i]->effectiveHalfWidth - vHalfWidth < 0.5 &&
                            memory.perceptedObjects[i]->distanceToObject > 0.0 && memory.perceptedObjects[i]->distanceToObject < memory.distanceToNodeWPOut + memory.distanceToZeroSpeed ){

                        if( alreadyCheckedList.indexOf( memory.perceptedObjects[i]->objectID ) >= 0 ){
                            continue;
                        }

                        if( memory.perceptedObjects[i]->V < 0.1 && fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath - memory.lateralShiftTarget) - memory.perceptedObjects[i]->effectiveHalfWidth - vHalfWidth >= 0.0 ){
                            continue;
                        }

                        if( memory.perceptedObjects[i]->V - state.V > 0.0 ){
                            continue;
                        }

                        if( memory.perceptedObjects[i]->V < 3.6 ){
                            if( fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath - memory.lateralDeviationFromTargetPath) - memory.perceptedObjects[i]->effectiveHalfWidth - vHalfWidth > 0.1 ){
                                continue;
                            }
                        }

                        // If both vehicle is slow enoguh, drivers can steer well to avoid though the action is difficult to mimic in the simulation
                        if( memory.perceptedObjects[i]->V < 1.0 && state.V < 1.0 ){
                            continue;
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

                        bool regardAsPreceding = false;
                        if( memory.precedingVehicleID < 0 ){
                            regardAsPreceding = true;
                        }
                        else{
                            float relVp = memory.speedPrecedingVehicle - state.V;
                            if( relVp > -0.1 ){
                                relVp = -0.1;
                            }
                            float relVt = memory.perceptedObjects[i]->V - state.V;
                            if( relVt > -0.1 ){
                                relVt = -0.1;
                            }

                            float relDp = memory.distanceToPrecedingVehicle;
                            float relDt = memory.perceptedObjects[i]->distanceToObject;

                            float TTCp = relDp / (-relVp);
                            float TTCt = relDt / (-relVt);

                            strForDebugRiskEval += QString("V%1 TTCp=%2 TTCt=%3 : P=%4\n").arg( memory.perceptedObjects[i]->objectID ).arg( TTCp ).arg( TTCt ).arg( memory.precedingVehicleID );

                            if( TTCp > TTCt ){
                                regardAsPreceding = true;
                            }
                        }

                        if( regardAsPreceding == true ){


                            // Avoid turn-waiting stopping oncoming vehicle if necessary
                            if( memory.currentTargetNode != memory.nextTurnNode &&
                                    memory.currentTargetNode == memory.perceptedObjects[i]->objectTargetNode &&
                                    memory.perceptedObjects[i]->V < 1.0 ){

                                if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_LEFT ||
                                    memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_RIGHT ){

                                    memory.avoidTarget = memory.perceptedObjects[i]->objectID;

                                    float HalfLaneWidth = 1.5;
                                    if( memory.distanceToCurrentTargetNodeWPIn < 0.0 ){
                                        HalfLaneWidth = 2.5;
                                    }

                                    if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 0.0){
                                        memory.lateralShiftTarget = memory.perceptedObjects[i]->deviationFromNearestTargetPath;
                                        memory.lateralShiftTarget -= memory.perceptedObjects[i]->effectiveHalfWidth;
                                        memory.lateralShiftTarget -= vHalfWidth;
                                        memory.lateralShiftTarget -= 0.5;
                                        if( memory.lateralShiftTarget > 0.0 ){
                                            memory.lateralShiftTarget = 0.0;
                                            memory.avoidTarget = -1;
                                        }
                                        else if( memory.lateralShiftTarget < -(HalfLaneWidth - vHalfWidth)){
                                            memory.lateralShiftTarget = -(HalfLaneWidth - vHalfWidth);
                                        }

                                        setLateralShiftTarget = true;
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
                                        else if( memory.lateralShiftTarget > (HalfLaneWidth - vHalfWidth)){
                                            memory.lateralShiftTarget = (HalfLaneWidth - vHalfWidth);
                                        }

                                        setLateralShiftTarget = true;
                                    }
                                }
                            }


                            strForDebugRiskEval += QString("[P]V%1 by Obstacle-Check\n").arg( memory.perceptedObjects[i]->objectID );

                            memory.precedingVehicleID         = memory.perceptedObjects[i]->objectID;
                            memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->distanceToObject;
                            memory.speedPrecedingVehicle      = memory.perceptedObjects[i]->V * memory.perceptedObjects[i]->innerProductToNearestPathTangent;
                            if( memory.speedPrecedingVehicle < 0.0 ){
                                memory.speedPrecedingVehicle = 0.0;
                            }
                            memory.axPrecedingVehicle      = memory.perceptedObjects[i]->Ax;
                            memory.halfLenPrecedingVehicle = memory.perceptedObjects[i]->vHalfLength;
                            memory.precedingVehicleIndex   = i;
                            memory.precedingObstacle       = 1;

                            if( memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING && memory.perceptedObjects[i]->recognitionLabel == ONCOMING_LEFT ){
                                memory.precedingObstacle = 0;
                            }
                            else if( memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING && memory.perceptedObjects[i]->recognitionLabel == ONCOMING_RIGHT ){
                                memory.precedingObstacle = 0;
                            }
                        }
                    }
                }
            }

            if( memory.precedingVehicleID >= 0 ){

                memory.doHeadwayDistanceControl = true;

                memory.targetHeadwayDistance = param.headwayTime * state.V;
                if( memory.targetHeadwayDistance < param.minimumHeadwayDistanceAtStop  ){
                    memory.targetHeadwayDistance = param.minimumHeadwayDistanceAtStop;
                }

                memory.distanceToPrecedingVehicle -= memory.halfLenPrecedingVehicle;
                memory.distanceToPrecedingVehicle -= vHalfLength;
            }
        }
        else if( agentKind >= 100 ){

            memory.doHeadwayDistanceControl = false;

            int lastAvoidTarget = memory.avoidTarget;
            memory.avoidTarget = -20;
            memory.distToAvoidTarget = -1.0;

            float ppHW = pRoad->GetPedestPathWidth( memory.currentTargetPath, memory.currentTargetPathIndexInList) * 0.5;
            float ppHalfWidth = ppHW - vHalfWidth;
            if( ppHalfWidth < 0.05 ){
                ppHalfWidth = 0.05;
            }

            // Select Candidates
            QList<int> candidates;
            for(int i=0;i<memory.perceptedObjects.size();++i){

                if( memory.perceptedObjects[i]->isValidData == false ){
                    continue;
                }

                if( memory.perceptedObjects[i]->objectType < 100 ){
                    continue;
                }

                if( memory.perceptedObjects[i]->relPosEvaled == false ){
                    continue;
                }

                if( memory.perceptedObjects[i]->distanceToObject < 0.0 ){
                    continue;
                }

                float relY = memory.perceptedObjects[i]->deviationFromNearestTargetPath - memory.lateralDeviationFromTargetPath;
                if( fabs(relY) > vHalfWidth + memory.perceptedObjects[i]->vHalfWidth + 0.25 ){
                    continue;
                }

                float vRel = memory.perceptedObjects[i]->V;
                vRel -= state.V;
                if( vRel > 0.0 ){
                    continue;
                }

                float distCheck = vRel * (-3.0);
                if( distCheck < 5.0 ){
                    distCheck = 5.0;
                }
                if( memory.perceptedObjects[i]->distanceToObject - vHalfLength - memory.perceptedObjects[i]->vHalfLength > distCheck ){
                    continue;
                }

                candidates.append( i );
            }

            QList<float> tmpShifts;
            for(int n=0;n<candidates.size();++n ){

                int i = candidates[n];

                float tmplateralShiftTarget = 0.0;

                float tmplateralShiftTarget1 = memory.perceptedObjects[i]->deviationFromNearestTargetPath;
//                tmplateralShiftTarget1 -= vHalfWidth;
                tmplateralShiftTarget1 -= memory.perceptedObjects[i]->vHalfWidth;
                tmplateralShiftTarget1 -= 0.25;
                if( tmplateralShiftTarget1 < -ppHalfWidth ){
                    tmplateralShiftTarget1 = -ppHalfWidth;
                }

                float tmplateralShiftTarget2 = memory.perceptedObjects[i]->deviationFromNearestTargetPath;
//                tmplateralShiftTarget2 += vHalfWidth;
                tmplateralShiftTarget2 += memory.perceptedObjects[i]->vHalfWidth;
                tmplateralShiftTarget2 += 0.25;
                if( tmplateralShiftTarget2 > ppHalfWidth ){
                    tmplateralShiftTarget2 = ppHalfWidth;
                }

                float margin1 = tmplateralShiftTarget1 - memory.perceptedObjects[i]->deviationFromNearestTargetPath;
                float margin2 = tmplateralShiftTarget2 - memory.perceptedObjects[i]->deviationFromNearestTargetPath;
                if( fabs(margin1) > fabs(margin2) ){
                    tmplateralShiftTarget = tmplateralShiftTarget1;
                }
                else{
                    tmplateralShiftTarget = tmplateralShiftTarget2;
                }

                if( tmplateralShiftTarget > ppHalfWidth ){
                    tmplateralShiftTarget = ppHalfWidth;
                }
                else if( tmplateralShiftTarget < -ppHalfWidth ){
                    tmplateralShiftTarget = -ppHalfWidth;
                }

                tmpShifts.append( tmplateralShiftTarget );


                float relY = memory.perceptedObjects[i]->deviationFromNearestTargetPath - memory.lateralDeviationFromTargetPath;

                strForDebugRiskEval += QString("[1]P=%1: relY=%2 s1=%3 m1=%4 s2=%5 m2=%6 s=%7\n").arg( memory.perceptedObjects[i]->objectID ).arg(relY)
                        .arg(tmplateralShiftTarget1).arg(margin1).arg(tmplateralShiftTarget2).arg(margin2).arg(tmplateralShiftTarget);

            }

            QList<bool> isValidShift;
            QList<float> minMargin;
            for(int n=0;n<tmpShifts.size();++n ){

                // Check if other pedestrian exist near
                bool validLateralShift = true;

                int i = candidates[n];
                float tmpLateralShift = tmpShifts[n];
                float margin = ppHW * 2.0;
                for(int j=0;j<memory.perceptedObjects.size();++j){

                    if( memory.perceptedObjects[j]->isValidData == false ){
                        continue;
                    }

                    if( memory.perceptedObjects[j]->objectType < 100 ){
                        continue;
                    }

                    if( memory.perceptedObjects[j]->relPosEvaled == false ){
                        continue;
                    }

                    if( memory.perceptedObjects[j]->distanceToObject < 0.0 ){
                        continue;
                    }

                    if( memory.perceptedObjects[j]->distanceToObject > memory.perceptedObjects[i]->distanceToObject + 2.5 ){
                        continue;
                    }

                    float relY = memory.perceptedObjects[j]->deviationFromNearestTargetPath - tmpLateralShift;

                    if( margin > fabs(relY) ){
                        margin = fabs(relY);
                    }

                    strForDebugRiskEval += QString("[2]P=%1: relY=%2 margin=%3\n").arg( memory.perceptedObjects[i]->objectID ).arg(relY).arg(margin);

                    if( fabs(relY) > vHalfWidth + memory.perceptedObjects[i]->vHalfWidth ){
                        continue;
                    }

                    validLateralShift = false;
                }

                isValidShift.append( validLateralShift );
                minMargin.append( margin );

                strForDebugRiskEval += QString("tmpShift=%1 isValid=%2 margin=%3\n").arg(tmpLateralShift).arg(validLateralShift).arg(margin);
            }

            int selShiftIndex = -1;
            for(int n=0;n<isValidShift.size();++n ){

                if( isValidShift[n] == false ){
                    continue;
                }

                if( selShiftIndex < 0 ){
                    selShiftIndex = n;
                }
                else{
                    if( minMargin[selShiftIndex] < minMargin[n]){
                        selShiftIndex = n;
                    }
                }
            }

//            if( selShiftIndex < 0 ){
//                for(int n=0;n<isValidShift.size();++n ){
//                    if( selShiftIndex < 0 ){
//                        selShiftIndex = n;
//                    }
//                    else{
//                        if( minMargin[selShiftIndex] < minMargin[n]){
//                            selShiftIndex = n;
//                        }
//                    }
//                }
//            }

            strForDebugRiskEval += QString("selShiftIndex=%1\n").arg(selShiftIndex);

            if( selShiftIndex >= 0 ){
                memory.avoidTarget = memory.perceptedObjects[ candidates[selShiftIndex] ]->objectID;
                memory.lateralShiftTarget = tmpShifts[selShiftIndex];
                memory.distToAvoidTarget = memory.perceptedObjects[ candidates[selShiftIndex] ]->distanceToObject;
            }


            if( memory.avoidTarget < -1 ){

                if( lastAvoidTarget < -1 ){
                    memory.avoidTarget = lastAvoidTarget;
                }

                memory.avoidTarget++;
                if( memory.avoidTarget == -1 ){
                    memory.lateralShiftTarget = memory.lateralShiftTarget_backup;
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
                int stopTargetIdx = -1;

                float dStop = 0.0;
                if( memory.distanceToCurrentTargetNodeWPIn < 0.0 ){
                    dStop = memory.minimumDistanceToStop;
                }

                // Check if I can stop before Near Oncoming CP
                if( memory.distToNearOncomingCP < dStop + 2.5 + vHalfLength ){

                    shoudStop = false;

                }
                else{

                    bool oncomingVehicleCanGo = true;
                    if( ( ((turnIntersectionTSValue & 0x04) == 0x04) && ((turnIntersectionTSValue & 0x10) == 0x00) ) ||
                            ( (turnIntersectionTSValue & 0x02) == 0x02 ) ){
                        oncomingVehicleCanGo = false;
                    }

                    int ntNdIdx = pRoad->nodeId2Index.indexOf( memory.nextTurnNode );
                    if( pRoad->nodes[ntNdIdx]->hasTS == true ){
                        int ocDir = memory.nextTurnNodeOncomingDir;
                        for(int n=0;n<pRoad->nodes[ntNdIdx]->relatedVTSIndex.size();++n){
                            int j = pRoad->nodes[ntNdIdx]->relatedVTSIndex[n];
                            if( trafficSignal[j]->controlDirection != ocDir ){
                                continue;
                            }
                            int sig = trafficSignal[j]->GetCurrentDisplayInfo();
                            if( (((sig & 0x04) == 0x04) && ((sig & 0x10) == 0x00 )) || sig == 0x00 ){  // 0x00 : Signal Down
                                oncomingVehicleCanGo = false;
                            }
                            break;
                        }
                    }

                    strForDebugRiskEval += QString("oncomingVehicleCanGo = %1\n").arg( oncomingVehicleCanGo );

                    for(int i=0;i<memory.perceptedObjects.size();++i){

                        if( memory.perceptedObjects[i]->isValidData == false ){
                            continue;
                        }

                        if( memory.perceptedObjects[i]->relPosEvaled == false ){
                            continue;
                        }

//                        if( pAgent[memory.perceptedObjects[i]->objectID]->isSInterfaceObject == true ){
//                            continue;
//                        }

                        if( memory.perceptedObjects[i]->objectType >= 100 ){
                            continue;
                        }

                        if( memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_STRAIGHT &&
                                memory.perceptedObjects[i]->hasCollisionPoint == true ){

                            // Check collision point is in turn node
                            if( memory.perceptedObjects[i]->CPinNode != memory.nextTurnNode ){
                                continue;
                            }

                            // If the oncoming vehicle is around collision point, do not enter
                            if( memory.perceptedObjects[i]->objectTimeToCP >= 0.0 && memory.perceptedObjects[i]->objectTimeToCP < 3.0 ){

                                strForDebugRiskEval += QString("[3]Oncoming Risk by ID%1\n").arg(  memory.perceptedObjects[i]->objectID );

                                shoudStop = true;
                                stopTargetIdx = i;

                                break;
                            }

                            // Special Case: oncoming vehicle waiting for crossing


                            if( oncomingVehicleCanGo == true &&
                                    memory.perceptedObjects[i]->objectDistanceToCP >= 0.0 &&
                                    memory.perceptedObjects[i]->objectDistanceToCP - memory.perceptedObjects[i]->vHalfLength < 10.0 &&
                                    memory.perceptedObjects[i]->filteredV < 0.1 ){

                                if( memory.perceptedObjects[i]->myDistanceToCP - vHalfLength > 10.0 ){

                                    strForDebugRiskEval += QString("[4]Oncoming Risk by ID%1\n").arg(  memory.perceptedObjects[i]->objectID );

                                    shoudStop = true;
                                    stopTargetIdx = i;
                                    break;
                                }
                                else{

                                    strForDebugRiskEval += QString("Oncoming ID%1 will yeild\n").arg(  memory.perceptedObjects[i]->objectID );

                                    continue;
                                }
                            }


                            // The oncoming vehicle has already passed collision point
                            if( memory.perceptedObjects[i]->objectDistanceToCP + 1.5 + memory.perceptedObjects[i]->vHalfLength < 0.0  ){

                                strForDebugRiskEval += QString("Oncoming ID%1 passed CP\n").arg(  memory.perceptedObjects[i]->objectID );

                                continue;
                            }

                            // Far oncoming vehicle is neglected if I can reach CP before oncoming vehicle
                            if( memory.perceptedObjects[i]->objectTimeToCP > 6.0 && memory.perceptedObjects[i]->objectDistanceToCP > 30.0 &&
                                    memory.perceptedObjects[i]->objectTimeToCP > memory.perceptedObjects[i]->myTimeToCP + param.crossTimeSafetyMargin * 2.0 ){

                                strForDebugRiskEval += QString("Oncoming ID%1 far from CP\n").arg(  memory.perceptedObjects[i]->objectID );

                                continue;
                            }


                            // I have already passed collision point
                            if( memory.perceptedObjects[i]->myTimeToCP < 0.0 ){

                                strForDebugRiskEval += QString("I passed CP faster than Oncoming ID%1\n").arg(  memory.perceptedObjects[i]->objectID );
                                memory.ignoreCollisonAvoidanceCheckObject.append( memory.perceptedObjects[i]->objectID );

                                continue;
                            }

                            // I am far from collision point
                            if( memory.perceptedObjects[i]->myTimeToCP > 6.0 && memory.perceptedObjects[i]->myDistanceToCP > 60.0 ){

                                strForDebugRiskEval += QString("I am far from Oncoming ID%1\n").arg(  memory.perceptedObjects[i]->objectID );

                                continue;
                            }


                            // If the signal intersection and the color is red, check if the oncoming stop
                            if( oncomingVehicleCanGo == false ){
                                int objID = memory.perceptedObjects[i]->objectID;
                                float dSL = pAgent[objID]->memory_reference.distanceToStopPoint;
                                float dVZ = pAgent[objID]->memory_reference.distanceToZeroSpeed - pAgent[objID]->state.V - memory.perceptedObjects[i]->vHalfLength;

                                strForDebugRiskEval += QString("dSL=%1 dVZ=%2\n").arg( dSL ).arg( dVZ );

                                if( dSL > 0.0 && dSL + 5.0 > dVZ ){
                                    strForDebugRiskEval += QString("Oncoming ID%1 will stop\n").arg(  memory.perceptedObjects[i]->objectID );
                                    memory.ignoreCollisonAvoidanceCheckObject.append( memory.perceptedObjects[i]->objectID );

                                    continue;
                                }
                            }


                            // Prediction; The oncoming vehicle will pass collision when I reach the point
                            float preTime = 1.0;
                            if( memory.perceptedObjects[i]->myTimeToCP < preTime ){
                                preTime = memory.perceptedObjects[i]->myTimeToCP;
                            }
                            if( memory.perceptedObjects[i]->objectDistanceToCP - memory.perceptedObjects[i]->V * preTime  < -1.5 - memory.perceptedObjects[i]->vHalfLength ){

                                strForDebugRiskEval += QString("Oncoming ID%1 will pass CP when I reach CP\n").arg(  memory.perceptedObjects[i]->objectID );

                                continue;
                            }


                            float myTime1 = memory.perceptedObjects[i]->myDistanceToCP / 5.0;
                            float myTime2 = memory.perceptedObjects[i]->myTimeToCP;

                            float myTimeMin = (myTime1 < myTime2 ? myTime1 : myTime2 );
                            float myTimeMax = (myTime1 > myTime2 ? myTime1 : myTime2 );


                            float crossTimeP = myTimeMin - memory.perceptedObjects[i]->objectTimeToCP;
                            float crossTimeM = myTimeMax - memory.perceptedObjects[i]->objectTimeToCP;

                            if( crossTimeP > param.crossTimeSafetyMargin  ){  // oncoming vehicle will pass faster than me
                                continue;
                            }
                            else if( crossTimeM < (-1.0) * param.crossTimeSafetyMargin  ){  // I can pass faster than oncoming vehicle
                                continue;
                            }

                            strForDebugRiskEval += QString("[1]Oncoming Risk by ID%1\n").arg(  memory.perceptedObjects[i]->objectID );
                            strForDebugRiskEval += QString("  crossTimeP=%1 crossTimeM=%2\n").arg( crossTimeP ).arg( crossTimeM );

                            shoudStop = true;
                            stopTargetIdx = i;
                            break;
                        }
                        else if(  (pRoad->LeftOrRight == 0 && memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_RIGHT ) ||
                                  (pRoad->LeftOrRight == 1 && memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_LEFT) ){

                            if( memory.perceptedObjects[i]->winker == 0 ){

                                int objID = memory.perceptedObjects[i]->objectID;

                                // If the signal intersection and the color is red, check if the oncoming stop
                                if( oncomingVehicleCanGo == false ){
                                    int objID = memory.perceptedObjects[i]->objectID;
                                    float dSL = pAgent[objID]->memory_reference.distanceToStopPoint;
                                    float dVZ = pAgent[objID]->memory_reference.distanceToZeroSpeed - pAgent[objID]->state.V - memory.perceptedObjects[i]->vHalfLength;

                                    strForDebugRiskEval += QString("dSL=%1 dVZ=%2\n").arg( dSL ).arg( dVZ );

                                    if( dSL > 0.0 && dSL + 5.0 > dVZ ){

                                        strForDebugRiskEval += QString("Oncoming ID%1 will stop\n").arg(  memory.perceptedObjects[i]->objectID );

                                        continue;
                                    }
                                }

                                float TTC = pAgent[objID]->memory_reference.distanceToCurrentTargetNodeWPIn / ( memory.perceptedObjects[i]->V + 0.1 );
                                if( TTC < 6.0 ){
                                    shoudStop = true;
                                    stopTargetIdx = i;
                                    break;
                                }

                            }
                        }
                        else if( ( (pRoad->LeftOrRight == 0 && memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_LEFT ) ||
                                   (pRoad->LeftOrRight == 1 && memory.perceptedObjects[i]->recognitionLabel == AGENT_RECOGNITION_LABEL::ONCOMING_RIGHT) ) &&
                                    memory.perceptedObjects[i]->hasCollisionPoint == true ){

                            // Check collision point is in turn node
                            if( memory.perceptedObjects[i]->CPinNode != memory.nextTurnNode ){
                                continue;
                            }

                            // Far oncoming vehicle is neglected if I can reach CP before oncoming vehicle
                            if( memory.perceptedObjects[i]->objectTimeToCP > 6.0 && memory.perceptedObjects[i]->objectDistanceToCP > 30.0 &&
                                    memory.perceptedObjects[i]->objectTimeToCP > memory.perceptedObjects[i]->myTimeToCP + param.crossTimeSafetyMargin * 2.0 ){
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

                            // The oncoming vehicle has already passed collision point
                            if( memory.perceptedObjects[i]->objectTimeToCP < 0.0 ){
                                continue;
                            }


                            // If the mergine vehicle is assumed as preceding vehicle, ignore it
                            if( memory.perceptedObjects[i]->objectID == memory.precedingVehicleID ){
                                continue;
                            }


                            // If the signal intersection and the color is red, check if the oncoming stop
                            if( oncomingVehicleCanGo == false ){
                                int objID = memory.perceptedObjects[i]->objectID;
                                float dSL = pAgent[objID]->memory_reference.distanceToStopPoint;
                                float dVZ = pAgent[objID]->memory_reference.distanceToZeroSpeed - pAgent[objID]->state.V - memory.perceptedObjects[i]->vHalfLength;

                                strForDebugRiskEval += QString("dSL=%1 dVZ=%2").arg( dSL ).arg( dVZ );

                                if( (dSL > 0.0 && dSL + 5.0 > dVZ)|| memory.perceptedObjects[i]->V < 3.0 ){
                                    continue;
                                }
                            }

                            // If the oncoming-merging vehicle is around collision/merging point, do not enter
                            if( memory.perceptedObjects[i]->objectTimeToCP >= 0.0 && memory.perceptedObjects[i]->objectTimeToCP < 3.0 ){
                                shoudStop = true;
                                stopTargetIdx = i;
                                break;
                            }

                            float myTime1 = memory.perceptedObjects[i]->myDistanceToCP / 5.0;
                            float myTime2 = memory.perceptedObjects[i]->myTimeToCP;

                            float myTimeMin = (myTime1 < myTime2 ? myTime1 : myTime2 );
                            float myTimeMax = (myTime1 > myTime2 ? myTime1 : myTime2 );


                            float crossTimeP = myTimeMin - memory.perceptedObjects[i]->objectTimeToCP;
                            float crossTimeM = myTimeMax - memory.perceptedObjects[i]->objectTimeToCP;
                            if( crossTimeP > param.crossTimeSafetyMargin  ){  // oncoming-merging vehicle will pass faster than me
                                continue;
                            }
                            else if( crossTimeM < (-1.0) * param.crossTimeSafetyMargin  ){  // I can pass faster than oncoming-merging vehicle
                                continue;
                            }

                            strForDebugRiskEval += QString("[2]Oncoming Risk by ID%1\n").arg(  memory.perceptedObjects[i]->objectID );
                            strForDebugRiskEval += QString("  crossTimeP=%1 crossTimeM=%2\n").arg( crossTimeP ).arg( crossTimeM );

                            shoudStop = true;
                            stopTargetIdx = i;
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
                            memory.causeOfStopControl = QString("Oncoming");
                            if( stopTargetIdx >= 0 ){
                                memory.hazardusObject = memory.perceptedObjects[stopTargetIdx]->objectID;
                            }
                        }
                    }
                    else{
                        memory.doStopControl = true;
                        memory.distanceToStopPoint = memory.distToNearOncomingCP - param.crossWaitPositionSafeyMargin - vHalfLength;
                        memory.releaseStopCount = -1;
                        memory.causeOfStopControl = QString("Oncoming");
                        if( stopTargetIdx >= 0 ){
                            memory.hazardusObject = memory.perceptedObjects[stopTargetIdx]->objectID;
                        }
                    }
                }
            }
        }


        //
        // Check of pedestrian crossing
        if( agentKind < 100 ){

            for(int i=0;i<memory.perceptedObjects.size();++i){

                if( memory.perceptedObjects[i]->isValidData == false ){
                    continue;
                }

                if( memory.perceptedObjects[i]->objectType < 100 ){
                    continue;
                }

                if( memory.perceptedObjects[i]->relPosEvaled == false ){
                    continue;
                }

                if( memory.perceptedObjects[i]->distanceToObject < vHalfLength ){
                    continue;
                }
                else if( memory.perceptedObjects[i]->distanceToObject > memory.distanceToZeroSpeed + vHalfLength + 5.0){
                    continue;
                }

                float timeToPed = memory.perceptedObjects[i]->distanceToObject / ( state.V + 0.1 );
                float latpos = memory.perceptedObjects[i]->deviationFromNearestTargetPath - memory.lateralDeviationFromTargetPath;
                float relVn = memory.perceptedObjects[i]->innerProductToNearestPathNormal * memory.perceptedObjects[i]->filteredV;

                if( timeToPed > 10.0 ){
                    timeToPed = 10.0;
                }

                if( relVn > 0.0 && relVn < 0.1 ){
                    relVn = 0.1;
                }
                else if( relVn < 0.0 && relVn > -0.1 ){
                    relVn = -0.1;
                }

                float pedestMargin = 0.5;
                if( latpos > 0 && relVn < 0.0 ){
                    if( relVn < -1.0 ){
                        pedestMargin = 1.5;
                    }
                    else{
                        pedestMargin = 0.5 - relVn * 1.0;
                    }
                }
                else if( latpos < 0.0 && relVn > 0.0 ){
                    if( relVn > 1.0 ){
                        pedestMargin = 1.5;
                    }
                    else{
                        pedestMargin = 0.5 + relVn * 1.0;
                    }
                }
                if( fabs(latpos) < vHalfWidth ){
                    pedestMargin = 1.5;
                }

                if( state.V < 0.1 && fabs(relVn) < 0.25 ){
                    pedestMargin = 0.25;
                    if( fabs(latpos) > vHalfWidth + pedestMargin ){
                        continue;
                    }
                }

//                strForDebugRiskEval += QString("pedest=%1 timeToPed=%2 latpos=%3 relVn=%4 pedestMargin=%5\n").arg( memory.perceptedObjects[i]->objectID ).arg( timeToPed ).arg(latpos).arg(relVn).arg(pedestMargin);

                if( state.V > 3.0 && timeToPed > 6.0 ){
                    continue;
                }

                if( state.V < 3.0 && timeToPed > 1.0 ){
                    timeToPed = 1.0;
                }

                if( latpos > 0.0 && latpos < 1.5 && fabs(relVn) < 0.1 ){
                    memory.lateralShiftTarget = -1.0;

                    setLateralShiftTarget = true;
                }

                if( fabs(latpos + relVn * timeToPed ) > vHalfWidth + pedestMargin + memory.lateralShiftTarget ){
                    continue;
                }


//                if( fabs(memory.perceptedObjects[i]->objectDistanceToCP) > vHalfWidth &&
//                        memory.perceptedObjects[i]->objectTimeToCP > 5.0 ){
//                    continue;
//                }

//                if( memory.perceptedObjects[i]->objectTimeToCP < -1.5 ){
//                    continue;
//                }

                int turnDir = 0;
                int ntpIdx = pRoad->pathId2Index.indexOf( memory.perceptedObjects[i]->nearestTargetPath );
                if( ntpIdx >= 0 ){
                    int crossNode = pRoad->paths[ntpIdx]->connectingNode;
                    if( crossNode == memory.nextTurnNode ){
                        turnDir = memory.nextTurnDirection;
                    }
                }

                strForDebugRiskEval += QString("pedest=%1 turnDir=%2 pedestMargin=%3 relVn=%4\n").arg( memory.perceptedObjects[i]->objectID ).arg( turnDir ).arg( pedestMargin ).arg( relVn );

                if( turnDir == 0 || turnDir == DIRECTION_LABEL::STRAIGHT ){

                    if( memory.doStopControl == true ){
                        float d = memory.perceptedObjects[i]->distanceToObject - param.crossWaitPositionSafeyMargin - vHalfLength;;
                        if( memory.distanceToStopPoint > d ){
                            memory.distanceToStopPoint = d;
                            memory.releaseStopCount = -1;
                            memory.causeOfStopControl = QString("Pedest-Wait-front: P%1").arg( memory.perceptedObjects[i]->objectID );
                            memory.hazardusObject = memory.perceptedObjects[i]->objectID;
                        }
                    }
                    else{
                        memory.doStopControl = true;
                        memory.distanceToStopPoint = memory.perceptedObjects[i]->distanceToObject - param.crossWaitPositionSafeyMargin - vHalfLength;
                        memory.releaseStopCount = -1;
                        memory.causeOfStopControl = QString("Pedest-Wait-front: P%1").arg( memory.perceptedObjects[i]->objectID );
                        memory.hazardusObject = memory.perceptedObjects[i]->objectID;
                    }

                }
                else if( (pRoad->LeftOrRight == 0 && turnDir == DIRECTION_LABEL::RIGHT_CROSSING) ||
                        (pRoad->LeftOrRight == 1 && turnDir == DIRECTION_LABEL::LEFT_CROSSING) ){

                    if( memory.doStopControl == true ){
                        float d = memory.distToNearOncomingCP - param.crossWaitPositionSafeyMargin - vHalfLength;;
                        if( memory.distanceToStopPoint > d ){
                            memory.distanceToStopPoint = d;
                            memory.releaseStopCount = -1;
                            memory.causeOfStopControl = QString("Pedest-But-Wait-at-Oncoming: P%1").arg( memory.perceptedObjects[i]->objectID );
                            memory.hazardusObject = memory.perceptedObjects[i]->objectID;
                        }
                    }
                    else{
                        memory.doStopControl = true;
                        memory.distanceToStopPoint = memory.distToNearOncomingCP - param.crossWaitPositionSafeyMargin - vHalfLength;
                        memory.releaseStopCount = -1;
                        memory.causeOfStopControl = QString("Pedest-But-Wait-at-Oncoming: P%1").arg( memory.perceptedObjects[i]->objectID );
                        memory.hazardusObject = memory.perceptedObjects[i]->objectID;
                    }

                }
                else{

                    if( memory.doStopControl == true ){
                        float d = memory.perceptedObjects[i]->myDistanceToCP - param.pedestWaitPositionSafetyMargin - vHalfLength;
                        if( memory.distanceToStopPoint > d ){
                            memory.distanceToStopPoint = d;
                            memory.releaseStopCount = -1;
                            memory.causeOfStopControl = QString("Pedest-Wait: %1").arg( memory.perceptedObjects[i]->objectID );
                            memory.hazardusObject = memory.perceptedObjects[i]->objectID;
                        }
                    }
                    else{
                        memory.doStopControl = true;
                        memory.distanceToStopPoint = memory.perceptedObjects[i]->myDistanceToCP - param.pedestWaitPositionSafetyMargin - vHalfLength;
                        memory.releaseStopCount = -1;
                        memory.causeOfStopControl = QString("Pedest-Wait: %1").arg( memory.perceptedObjects[i]->objectID );
                        memory.hazardusObject = memory.perceptedObjects[i]->objectID;
                    }

                }
            }
        }


        if( agentKind < 100 && setLateralShiftTarget == false && state.V > 3.0 && fabs(memory.lateralShiftTarget) > 0.01 ){

            float decreaseLatShift = 0.035;

            if( memory.lateralShiftTarget > 0.0 ){
                memory.lateralShiftTarget -= decreaseLatShift;
                if( memory.lateralShiftTarget < 0.0 ){
                    memory.lateralShiftTarget = 0.0;
                }
            }
            else{
                memory.lateralShiftTarget += decreaseLatShift;
                if( memory.lateralShiftTarget > 0.0 ){
                    memory.lateralShiftTarget = 0.0;
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
                        memory.causeOfStopControl = QString("Yeild 1");
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

				float d = memory.distToYeildStopLine - vHalfLength;;
                if( memory.distanceToStopPoint > d ){
                    memory.distanceToStopPoint = d;
                    memory.causeOfStopControl = QString("Yeild 1");
                    if( memory.safetyConfimed == false ){
                        memory.doStopControl = true;
                    }
                }
                
                if( memory.nextTurnNode >= 0 && memory.nextTurnNode == memory.currentTargetNode ){
                    if( memory.doStopControl == true ){
                        float d = memory.distanceToTurnNodeWPIn - vHalfLength;
                        if( (pRoad->LeftOrRight == 0 && memory.currentTargetNode == memory.nextTurnNode && memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING) ||
                             (pRoad->LeftOrRight == 1 && memory.currentTargetNode == memory.nextTurnNode && memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING)   ){
                            d -= 5.0;
                        }

                        if( memory.distanceToStopPoint > d ){
                            memory.distanceToStopPoint = d;
                            memory.releaseStopCount = -1;
                            memory.causeOfStopControl = QString("Yeild 2-1");
                        }
                    }
                    else{
                        memory.doStopControl = true;
                        memory.distanceToStopPoint = memory.distanceToTurnNodeWPIn - vHalfLength;
                        memory.releaseStopCount = -1;
                        memory.causeOfStopControl = QString("Yeild 2-2");
                    }
                }

                if( memory.distanceToStopPoint < 2.5 + vHalfLength && state.V >= 2.0 ){
                    if( memory.safetyConfimed == true ){
                        memory.doStopControl = false;
                    }
                }
                else if( memory.distanceToStopPoint < 2.5 + vHalfLength && state.V < 2.0 ){

                    if( memory.safetyConfimed == true ){

                        // If stay for a while after safety confirmed, the situation may be changed, so check again
                        if( memory.speedZeroCount * calInterval * decisionMakingCountMax > param.safetyConfirmTime ){
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
                                if( memory.rightCrossCheckCount * calInterval * decisionMakingCountMax > param.safetyConfirmTime ){
                                    checkKind = -1;
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
                                if( memory.leftCrossCheckCount * calInterval * decisionMakingCountMax > param.safetyConfirmTime ){
                                    checkKind = -1;
                                }
                            }
                        }

                        strForDebugRiskEval += QString("[1]checkKind=%1\n").arg(checkKind);

                        if( pRoad->LeftOrRight == 0 &&
                                ( (memory.currentTargetNode == memory.nextTurnNode && memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING) ||
                                  (memory.currentTargetNode != memory.nextTurnNode) ) &&
                                checkKind == -1 ){

                            if( memory.leftCrossIsClear == false ){
                                checkKind = 1;
                                memory.leftCrossCheckCount = 0;
                            }
                            else if( memory.leftCrossIsClear == true ){
                                checkKind = 1;
                                memory.leftCrossCheckCount++;
                                if( memory.leftCrossCheckCount * calInterval * decisionMakingCountMax > param.safetyConfirmTime ){
                                    checkKind = -1;
                                }

                                if( memory.rightCrossCheckCount * calInterval * decisionMakingCountMax > param.safetyConfirmTime * 3.0 ){
                                    checkKind = 2;
                                    memory.rightCrossCheckCount = 0;
                                    memory.rightCrossIsClear = false;
                                }
                            }
                        }
                        else if( pRoad->LeftOrRight == 1 &&
                                 ( (memory.currentTargetNode == memory.nextTurnNode && memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING) ||
                                   (memory.currentTargetNode != memory.nextTurnNode) ) &&
                                 checkKind == -1 ){

                            if( memory.rightCrossIsClear == false ){
                                checkKind = 2;
                                memory.rightCrossCheckCount = 0;
                            }
                            else if( memory.rightCrossIsClear == true ){
                                checkKind = 2;
                                memory.rightCrossCheckCount++;
                                if( memory.rightCrossCheckCount * calInterval * decisionMakingCountMax > param.safetyConfirmTime ){
                                    checkKind = -1;
                                }

                                if( memory.leftCrossCheckCount * calInterval * decisionMakingCountMax > param.safetyConfirmTime * 3.0 ){
                                    checkKind = 1;
                                    memory.leftCrossCheckCount = 0;
                                    memory.leftCrossIsClear = false;
                                }
                            }
                        }

                        strForDebugRiskEval += QString("[2]checkKind=%1\n").arg(checkKind);

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

//                                if( pAgent[memory.perceptedObjects[i]->objectID]->isSInterfaceObject == true ){
//                                    continue;
//                                }

                                if( memory.perceptedObjects[i]->recognitionLabel != AGENT_RECOGNITION_LABEL::LEFT_CROSSING_STRAIGHT &&
                                        memory.perceptedObjects[i]->recognitionLabel != AGENT_RECOGNITION_LABEL::LEFT_CROSSING_LEFT &&
                                        memory.perceptedObjects[i]->recognitionLabel != AGENT_RECOGNITION_LABEL::LEFT_CROSSING_RIGHT ){
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->hasCollisionPoint == false ){
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->CPinNode != memory.currentTargetNode ){
                                    continue;
                                }

                                // Reject far vehicle
                                if( memory.perceptedObjects[i]->objectTimeToCP > 8.0 && memory.perceptedObjects[i]->objectDistanceToCP > 40.0 ){
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->objectDistanceToCP > 0.0 &&
                                        memory.perceptedObjects[i]->objectDistanceToCP < 10.0 ){

                                    int objID = memory.perceptedObjects[i]->objectID;

                                    if( memory.perceptedObjects[i]->V < 0.1 && memory.perceptedObjects[i]->objectDistanceToCP > 1.5 + memory.perceptedObjects[i]->vHalfLength
                                            && pAgent[objID]->memory.causeOfStopControl.contains(QString("Obj%1").arg(ID)) == true ){
                                        continue;
                                    }
                                    else{
	                                    memory.leftCrossIsClear = false;
	                                    memory.leftCrossCheckCount = 0;
	                                    break;
	                                }
                                }

                                float decelTime = memory.perceptedObjects[i]->V / param.accelControlGain;
                                float myTime = memory.perceptedObjects[i]->myDistanceToCP / 2.0;  // Assuem average speed of 2[m/s] to move to collision point
                                if( state.V > 2.0 ){
                                    myTime = memory.perceptedObjects[i]->myDistanceToCP / state.V;
                                }
                                if( myTime > 4.0 ){
                                    myTime = 4.0;
                                }

                                float diffTime = myTime - memory.perceptedObjects[i]->objectTimeToCP;

                                float accelEffect = 0.0;
                                if( pRoad->LeftOrRight == 0 ){
                                    accelEffect = 2.0;
                                }

                                if( diffTime - accelEffect > param.crossTimeSafetyMargin + decelTime ){
                                    if( memory.perceptedObjects[i]->myDistanceToCP > memory.perceptedObjects[i]->objectTimeToCP + vHalfLength + memory.perceptedObjects[i]->vHalfLength + param.minimumHeadwayDistanceAtStop ){
	                                    // I can after left-cross vehicle
	                                        memory.ignoreCollisonAvoidanceCheckObject.append( memory.perceptedObjects[i]->objectID );
	                                    continue;
	                                }
                                }
                                else if( diffTime < (decelTime + param.crossTimeSafetyMargin) * -1.0 ){
                                    // I can go faster
                                    memory.ignoreCollisonAvoidanceCheckObject.append( memory.perceptedObjects[i]->objectID );
                                    continue;
                                }

                                strForDebugRiskEval += QString("[1]Crossing Risk by ID%1\n").arg(  memory.perceptedObjects[i]->objectID );

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

//                                if( pAgent[memory.perceptedObjects[i]->objectID]->isSInterfaceObject == true ){
//                                    continue;
//                                }

                                if( memory.perceptedObjects[i]->recognitionLabel != AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_STRAIGHT &&
                                        memory.perceptedObjects[i]->recognitionLabel != AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_LEFT &&
                                        memory.perceptedObjects[i]->recognitionLabel != AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_RIGHT ){
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->hasCollisionPoint == false ){
                                    strForDebugRiskEval += QString("[2]ID%1 has no hasCollisionPoint\n").arg(  memory.perceptedObjects[i]->objectID );
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->CPinNode != memory.currentTargetNode ){
                                    strForDebugRiskEval += QString("[2]CPinNode of ID%1 is not currentTargetNode\n").arg(  memory.perceptedObjects[i]->objectID );
                                    continue;
                                }

                                // Reject fat vehicle
                                if( memory.perceptedObjects[i]->objectTimeToCP > 8.0 && memory.perceptedObjects[i]->objectDistanceToCP > 40.0 ){
                                    strForDebugRiskEval += QString("[2]Far, ID%1 is refected\n").arg(  memory.perceptedObjects[i]->objectID );
                                    continue;
                                }

                                if( memory.perceptedObjects[i]->objectDistanceToCP > 0.0 &&
                                        memory.perceptedObjects[i]->objectDistanceToCP < 10.0 ){


                                    int objID = memory.perceptedObjects[i]->objectID;

                                    if( memory.perceptedObjects[i]->V < 0.1 && memory.perceptedObjects[i]->objectDistanceToCP > 1.5 + memory.perceptedObjects[i]->vHalfLength
                                            && pAgent[objID]->memory.causeOfStopControl.contains(QString("Obj%1").arg(ID)) == true ){
                                        continue;
                                    }
                                    else{
	                                    memory.rightCrossIsClear = false;
	                                    memory.rightCrossCheckCount = 0;
	                                    break;
	                                }
                                }

                                float decelTime = memory.perceptedObjects[i]->V / param.accelControlGain;

                                float myTime = memory.perceptedObjects[i]->myDistanceToCP / 2.0;  // Assuem average speed of 2[m/s] to move to collision point
                                if( state.V > 2.0 ){
                                    myTime = memory.perceptedObjects[i]->myDistanceToCP / state.V;
                                }
                                if( myTime > 4.0 ){
                                    myTime = 4.0;
                                }

                                float diffTime = myTime - memory.perceptedObjects[i]->objectTimeToCP;

                                float accelEffect = 0.0;
                                if( pRoad->LeftOrRight == 1 ){
                                    accelEffect = 2.0;
                                }
                                if( diffTime - accelEffect > param.crossTimeSafetyMargin + decelTime ){
                                    if( memory.perceptedObjects[i]->myDistanceToCP > memory.perceptedObjects[i]->objectDistanceToCP + vHalfLength + memory.perceptedObjects[i]->vHalfLength + param.minimumDistanceToStopLine ){
                                        // I can go after right-cross vehicle
                                        memory.ignoreCollisonAvoidanceCheckObject.append( memory.perceptedObjects[i]->objectID );
	                                    strForDebugRiskEval += QString("[2]I can go after ID%1\n").arg(  memory.perceptedObjects[i]->objectID );
	                                    continue;
	                                }
                                }
                                else if( diffTime < (decelTime + param.crossTimeSafetyMargin) * -1.0 ){
                                    // I can go faster
                                    memory.ignoreCollisonAvoidanceCheckObject.append( memory.perceptedObjects[i]->objectID );
                                    strForDebugRiskEval += QString("[2]I can go faster than ID%1\n").arg(  memory.perceptedObjects[i]->objectID );
                                    continue;
                                }

                                strForDebugRiskEval += QString("[2]Crossing Risk by ID%1\n").arg(  memory.perceptedObjects[i]->objectID );

                                memory.rightCrossIsClear = false;
                                memory.rightCrossCheckCount = 0;
                                break;
                            }
                        }
                    }
                }
            }
        }


        // Check risk of side vehicles for Lane-Change
        if( agentKind < 100 && memory.checkSideVehicleForLC == true ){

            if( memory.LCCheckState == 1 ){
                memory.LCInfoGetCount++;

                // Decision Making: 5Hz -> 0.2[s]
                if( memory.LCInfoGetCount * 0.2 > param.LCInfoGetTime ){
                    memory.LCCheckState = 2;
                    //qDebug() << "set LCCheckState = 2";
                }
            }
            else if( memory.LCCheckState == 2 ){

                memory.sideVehicleRiskClear = true;

                for(int i=0;i<memory.perceptedObjects.size();++i){

                    if( memory.perceptedObjects[i]->isValidData == false ){
                        continue;
                    }

//                    if( pAgent[memory.perceptedObjects[i]->objectID]->isSInterfaceObject == true ){
//                        continue;
//                    }

                    if( memory.perceptedObjects[i]->objectType >= 100 ){
                        continue;
                    }

                    if( memory.perceptedObjects[i]->objPathInLCTargetPathList < 0 ){
                        continue;
                    }

                    if( memory.LCDirection == DIRECTION_LABEL::LEFT_CROSSING &&
                            (memory.perceptedObjects[i]->latDevObjInLaneChangeTargetPathList > 0.5 ||
                             memory.perceptedObjects[i]->latDevObjInLaneChangeTargetPathList < memory.latDeviFromLCTargetPathList + 0.5) ){
                        continue;
                    }

                    if( memory.LCDirection == DIRECTION_LABEL::RIGHT_CROSSING &&
                            (memory.perceptedObjects[i]->latDevObjInLaneChangeTargetPathList < -0.5 ||
                             memory.perceptedObjects[i]->latDevObjInLaneChangeTargetPathList > memory.latDeviFromLCTargetPathList - 0.5) ){
                        continue;
                    }


                    //qDebug() << "Agent[" << ID << "]: Check Side Risk :ID=" << memory.perceptedObjects[i]->objectID;
                    float vRel = memory.perceptedObjects[i]->V - state.V;
                    //qDebug() << "vRel = " << vRel;

                    strForDebugRiskEval += QString("Side Risk :ID%1 vRel=%2\n").arg(  memory.perceptedObjects[i]->objectID ).arg( vRel );


                    float Lmin = vHalfLength + memory.perceptedObjects[i]->vHalfLength;
                    if( vRel > 0.0 ){
                        Lmin += param.minimumHeadwayDistanceAtStop;
                    }
                    if( fabs(memory.perceptedObjects[i]->distToObjInLaneChangeTargetPathList) <= Lmin ){
                        memory.sideVehicleRiskClear = false;
                        strForDebugRiskEval += QString(" NG: Just Side\n");
                        //qDebug() << " NG: Just Side";
                        break;
                    }
                    else if( memory.perceptedObjects[i]->distToObjInLaneChangeTargetPathList > Lmin ){
                        if( vRel <= 1.0 ){
                            float Vs = memory.perceptedObjects[i]->V;
                            float L1 =  0.5 * Vs * Vs / param.accelControlGain + memory.perceptedObjects[i]->distToObjInLaneChangeTargetPathList;
                            if( L1 < memory.distanceToZeroSpeedByMaxBrake ){
                                memory.sideVehicleRiskClear = false;
                                strForDebugRiskEval += QString(" NG: Can't Stop L1=%1, D=%2\n").arg(L1).arg(memory.distanceToZeroSpeedByMaxBrake);
                                //qDebug() << " NG: Can't Stop L1" << L1 << " D=" << memory.distanceToZeroSpeedByMaxBrake;
                                break;
                            }
                        }
                    }
                    else if( memory.perceptedObjects[i]->distToObjInLaneChangeTargetPathList < -Lmin ){
                        if( vRel > 0.0 ){
                            float TTC = memory.perceptedObjects[i]->distToObjInLaneChangeTargetPathList * (-1.0) - Lmin;
                            TTC /= (vRel + 0.01);
                            //qDebug() << "TTC = " << TTC;
                            if( TTC < param.LCCutInAllowTTC ){
                                memory.sideVehicleRiskClear = false;
                                strForDebugRiskEval += QString(" NG: Smal TTC=%1, vRel=%2\n").arg(TTC).arg(vRel);
                                //qDebug() << " NG: Smal TTC TTC=" << TTC << " vRel=" << vRel;
                                break;
                            }
                        }
                    }

                    //qDebug() << " OK";
                }

                //qDebug() << "sideVehicleRiskClear = " << memory.sideVehicleRiskClear;

                if( memory.sideVehicleRiskClear == true ){
                    memory.LCCheckState = 3;
                    //qDebug() << "set LCCheckState = 3";
                }
            }
            else if( memory.LCCheckState == 3 ){

                memory.targetPathList.clear();
                memory.targetPathList = memory.laneChangeTargetPathList;

                //qDebug() << "targetPathList = " << memory.targetPathList;

                memory.targetPathLength.clear();
                for(int j=0;j<memory.targetPathList.size();++j){

                    float len = pRoad->GetPathLength( memory.targetPathList[j] );
                    memory.targetPathLength.append( len );

                    if( memory.targetPathList[j] == memory.currentTargetPath ){
                        memory.currentTargetPathIndexInList = j;
                    }
                }


                float tdev,txt,tyt,txd,tyd,ts;
                int currentPath = pRoad->GetNearestPathFromList( memory.targetPathList,
                                                                 state.x,
                                                                 state.y,
                                                                 state.z_path,
                                                                 state.yaw,
                                                                 tdev, txt, tyt, txd, tyd, ts );

                //qDebug() << "searched currentPath = " << currentPath;

                if( currentPath >= 0 ){

                    memory.currentTargetPath = currentPath;
                    memory.lateralDeviationFromTargetPath = tdev;

                    for(int j=0;j<memory.targetPathList.size();++j){
                        if( memory.targetPathList[j] == memory.currentTargetPath ){
                            memory.currentTargetPathIndexInList = j;
                            break;
                        }
                    }

                    int pIdx = pRoad->pathId2Index.indexOf( currentPath );
                    if( isScenarioObject == true ){
                        if( pRoad->paths[pIdx]->speed85pt == pRoad->paths[pIdx]->speedInfo ){
                            refSpeedMode = 1;
                        }
                        else{
                            refSpeedMode = 0;
                        }
                    }
                    SetTargetSpeedIndividual( pRoad->paths[pIdx]->speed85pt );

                    SetTargetNodeListByTargetPaths( pRoad );

                }
                else{
                    qDebug() << "Lane-Change : exchange targetPathList";
                    qDebug() << "Agent[" << ID << "]: Cannot determine nearest path from targetPathList;" << memory.targetPathList;
                }

                memory.laneMerge.clear();

                int i = memory.routeIndex;
                int selIdx = 0;
                for(int j=0;j<memory.LCSupportRouteLaneIndex;++j){
                    selIdx += pRoad->odRoute[i]->LCSupportLaneLists[j]->laneList.size();
                }
                selIdx += memory.routeLaneIndex;

                if( selIdx < pRoad->odRoute[i]->mergeLanesInfo.size() ){
                    for(int k=0;k<pRoad->odRoute[i]->mergeLanesInfo[selIdx].size();++k){

                        QPoint pairData;
                        pairData.setX( pRoad->odRoute[i]->mergeLanesInfo[selIdx][k].x() );
                        pairData.setY( pRoad->odRoute[i]->mergeLanesInfo[selIdx][k].y() );

                        memory.laneMerge.append( pairData );
                    }
                }
                else{
                    qDebug() << "Agent[" << ID << "]: Cannot determine mergeLanesInfo index";
                    qDebug() << "  selIdx = " << selIdx << " size = " << pRoad->odRoute[i]->mergeLanesInfo.size();
                }

                memory.LCCheckState = 4;
                memory.LCSteerMax = 0.0;
                memory.checkSideVehicleForLC = false;

            }
        }


        // Cehck Risk of Vehicles when Run-Out
        if( agentKind >= 100 && memory.exeRunOut == true && memory.runOutState == 1 ){

            float ex = memory.cosRunOutDir;
            float ey = memory.sinRunOutDir;

            int nearRightObj = -1;
            float nearRightDist = 0.0;
            int nearLeftObj = -1;
            int nearLeftDist = 0.0;
            for(int i=0;i<memory.perceptedObjects.size();++i){
                if( memory.perceptedObjects[i]->isValidData == false ){
                    continue;
                }
                if( memory.perceptedObjects[i]->objectType >= 100 ){
                    continue;
                }
                float rx = memory.perceptedObjects[i]->x - state.x;
                float ry = memory.perceptedObjects[i]->y - state.y;

                float Lx = rx * ex + ry * ey;
                if( Lx < 0.0 || Lx > 15.0 ){
                    continue;
                }

                float Ly = rx * (-ey) + ry * ex;
                if( Ly >= 0.0 && Ly - memory.perceptedObjects[i]->vHalfLength < 0.5 ){
                    nearLeftObj = i;
                    nearLeftDist = 0.0;
                    break;
                }
                if( Ly <= 0.0 && Ly + memory.perceptedObjects[i]->vHalfLength > -0.5 ){
                    nearRightObj = i;
                    nearRightDist = 0.0;
                    break;
                }

                if( Ly > 0.0 ){
                    if( nearLeftObj < 0 || nearLeftDist > Ly ){
                        nearLeftObj = i;
                        nearLeftDist = Ly;
                    }
                }
                else if( Ly < 0.0 ){
                    if( nearRightObj < 0 || nearRightDist > (-Ly) ){
                        nearRightObj = i;
                        nearRightDist = (-Ly);
                    }
                }
            }

            strForDebugRiskEval += QString("nearLeftObj = %1 nearLeftDist=%2\n").arg( nearLeftObj ).arg( nearLeftDist );
            strForDebugRiskEval += QString("nearRightObj = %1 nearRightDist=%2\n").arg( nearRightObj ).arg( nearRightDist );

            bool runOutCheck = true;
            if( nearRightObj >= 0 && nearRightDist < 0.5 ){
                runOutCheck = false;
            }
            if( nearLeftObj >= 0 && nearLeftDist < 0.5 ){
                runOutCheck = false;
            }

            strForDebugRiskEval += QString("runOutCheck = %1\n").arg( runOutCheck );

            if( runOutCheck == true ){
                memory.runOutState = 2;
                memory.runOutStartX = state.x;
                memory.runOutStartY = state.y;
            }
        }


 		//
        // Yield the way in case of traffic jam
        //
        bool checkEnterIS = true;
        if( memory.nextTurnDirection != DIRECTION_LABEL::STRAIGHT && memory.nextTurnNode == memory.currentTargetNode ){
            checkEnterIS = false;
        }
        if( checkEnterIS == true && state.V < 7.0 && memory.distanceToCurrentTargetNodeWPIn > -vHalfLength && memory.distanceToCurrentTargetNodeWPIn < memory.requiredDistToStopFromTargetSpeed ){

            float vPThr = 3.0;
            if( memory.precedingVehicleID >= 0 ){
                int ppID = pAgent[ memory.precedingVehicleID ]->memory.precedingVehicleID;
                if( ppID >= 0 ){
                    if( pAgent[ ppID ]->state.V <= pAgent[ memory.precedingVehicleID ]->state.V + 0.1 ){
                        vPThr = 7.0;
                    }
                }
            }

            if( memory.precedingVehicleID >= 0 && memory.speedPrecedingVehicle < vPThr && memory.axPrecedingVehicle <= 0.0 &&
                    memory.distanceToPrecedingVehicle < memory.distanceToNodeWPOut ){

                if( memory.doStopControl == true ){
                    float d = memory.distanceToCurrentTargetNodeWPIn - vHalfLength;
                    if( memory.distanceToStopPoint > d ){
                        memory.distanceToStopPoint = d;
                        memory.releaseStopCount = -1;
                        memory.causeOfStopControl = QString("Traffic Jam");
                        memory.hazardusObject = -1;
                    }
                }
                else{
                    memory.doStopControl = true;
                    memory.distanceToStopPoint = memory.distanceToCurrentTargetNodeWPIn - vHalfLength;
                    memory.releaseStopCount = -1;
                    memory.causeOfStopControl = QString("Traffic Jam");
                    memory.hazardusObject = -1;
                }
            }

        }
        
        //
        // Collisiton Avoidance
        //
        float eL =  vHalfWidth;
        float eR = -vHalfWidth;

        float eLrel = memory.lateralDeviationFromTargetPath + vHalfWidth;
        float eRrel = memory.lateralDeviationFromTargetPath - vHalfWidth;

        bool avoidByShift = false;

        memory.lastHazardusObject = memory.hazardusObject;
        memory.hazardusObject = -1;

        float ppWH = pRoad->GetPedestPathWidth( memory.currentTargetPath, memory.currentTargetPathIndexInList) * 0.5;
        float ppHalfWidth = ppWH - vHalfWidth;
        if( ppHalfWidth < 0.05 ){
            ppHalfWidth = 0.05;
        }
        else if( ppHalfWidth > ppWH ){
            ppHalfWidth = ppWH;
        }

        strForDebugRiskEval += QString("ppHalfWidth = %1\n").arg( ppHalfWidth );

        if( agentKind >= 100 ){
            if( memory.lateralShiftTarget > ppHalfWidth ){
                memory.lateralShiftTarget = ppHalfWidth;
            }
            else if( memory.lateralShiftTarget < -ppHalfWidth ){
                memory.lateralShiftTarget = -ppHalfWidth;
            }
        }


		bool ableToAvoid = true;
        if( memory.shouldYeild == true && memory.doStopControl == true &&
                memory.distToYeildStopLine > 0.0 ){
            ableToAvoid = false;
        }
        for(int i=0;i<memory.perceptedObjects.size();++i){

            if( memory.perceptedObjects[i]->isValidData == false ){
                continue;
            }

            if( memory.perceptedObjects[i]->objectID == memory.ignoreHazardusObject ){
                continue;
            }

            if( memory.ignoreCollisonAvoidanceCheckObject.contains( memory.perceptedObjects[i]->objectID ) == true ){
                continue;
            }

            if( agentKind >= 100 && memory.avoidTarget == memory.perceptedObjects[i]->objectID ){
                continue;
            }


            bool shouldStop = false;

            float fact = 1.0;

            if( memory.perceptedObjects[i]->relPosEvaled == true ){

                float tL = memory.perceptedObjects[i]->deviationFromNearestTargetPath + memory.perceptedObjects[i]->effectiveHalfWidth * fact;
                float tR = memory.perceptedObjects[i]->deviationFromNearestTargetPath - memory.perceptedObjects[i]->effectiveHalfWidth * fact;

                float distCheck = memory.perceptedObjects[i]->distanceToObject;
                if( agentKind < 100 && memory.perceptedObjects[i]->objectType < 100 && memory.perceptedObjects[i]->hasCollisionPoint == true ){

                    if(memory.perceptedObjects[i]->V < 0.1 && memory.perceptedObjects[i]->objectDistanceToCP > 4.2 ){
                        strForDebugRiskEval += QString("A%1; stop and far enought\n").arg( memory.perceptedObjects[i]->objectID );
                        continue;
                    }

                    if( memory.perceptedObjects[i]->myDistanceToCP > 10.0 + memory.distanceToZeroSpeed ){
                        strForDebugRiskEval += QString("A%1; myDistanceToCP=%2\n").arg( memory.perceptedObjects[i]->objectID )
                                .arg( memory.perceptedObjects[i]->myDistanceToCP );
                        continue;
                    }

                    if( memory.perceptedObjects[i]->objectDistanceToCP > memory.perceptedObjects[i]->myDistanceToCP ){
                        strForDebugRiskEval += QString("A%1; large objectDistanceToCP=%2\n").arg( memory.perceptedObjects[i]->objectID )
                                .arg( memory.perceptedObjects[i]->objectDistanceToCP );
                        continue;
                    }

                    if( memory.perceptedObjects[i]->objectDistanceToCP < -memory.perceptedObjects[i]->vHalfLength - 0.5 ){
                        strForDebugRiskEval += QString("A%1; small objectDistanceToCP=%2\n").arg( memory.perceptedObjects[i]->objectID )
                                .arg( memory.perceptedObjects[i]->objectDistanceToCP );
                        continue;
                    }

                    if( state.V > 1.0 && memory.perceptedObjects[i]->V > 1.0 && fabs(memory.perceptedObjects[i]->objectTimeToCP - memory.perceptedObjects[i]->myTimeToCP) > 3.0 ){
                        strForDebugRiskEval += QString("A%1; large ttc=%2\n").arg( memory.perceptedObjects[i]->objectID )
                                .arg( fabs(memory.perceptedObjects[i]->objectTimeToCP - memory.perceptedObjects[i]->myTimeToCP) );
                        continue;
                    }

                    shouldStop = true;

                    strForDebugRiskEval += QString("[CA] ObjID=%1, distToObj=%2 shouldStop=1\n").arg(memory.perceptedObjects[i]->objectID).arg(memory.perceptedObjects[i]->distanceToObject);

                }
                else{
                    if( distCheck < 0.0 ){

                        strForDebugRiskEval += QString("A%1; back\n").arg( memory.perceptedObjects[i]->objectID );

                        continue;
                    }

                    if( tR* fact > eLrel + 0.08 || eRrel - 0.08 > tL * fact ){

                        strForDebugRiskEval += QString("A%1; tR=%2 eLrel=%3 eRrel=%4 tL=%5\n").arg( memory.perceptedObjects[i]->objectID )
                                .arg( tR ).arg( eLrel ).arg( eRrel ).arg( tL ) ;

                        continue;
                    }

                    float relV = state.V;
                    relV -= memory.perceptedObjects[i]->innerProductToNearestPathTangent * memory.perceptedObjects[i]->V;
//                    if( relV < -0.1 ){

//                        strForDebugRiskEval += QString("A%1; relV=%2 \n").arg( memory.perceptedObjects[i]->objectID )
//                                .arg( relV ) ;

//                        continue;
//                    }


                    float L = memory.perceptedObjects[i]->distanceToObject - vHalfLength - memory.perceptedObjects[i]->vHalfLength;
                    float LL = param.minimumHeadwayDistanceAtStop;
                    if( memory.perceptedObjects[i]->recognitionLabel != PRECEDING &&
                        memory.perceptedObjects[i]->recognitionLabel != RIGHT_SIDE &&
                        memory.perceptedObjects[i]->recognitionLabel != LEFT_SIDE &&
                        memory.perceptedObjects[i]->recognitionLabel != RIGHT_SIDE_PRECEDING &&
                        memory.perceptedObjects[i]->recognitionLabel != LEFT_SIDE_PRECEDING ){
                        LL += memory.distanceToZeroSpeed;
                    }
                    else{
                        if( relV < 0.0 ){
                            strForDebugRiskEval += QString("A%1; relV=%2 departing\n").arg( memory.perceptedObjects[i]->objectID )
                                                            .arg( relV ) ;
                            continue;
                        }
                    }

                    if( relV < 0.0 ){
                        relV = 0.0;
                    }

                    if( L < LL ){
                        shouldStop = true;
                    }
                    else{
                        float ttc = memory.perceptedObjects[i]->distanceToObject / (relV + 0.1);
                        if( ttc > 2.5 || L > memory.distanceToZeroSpeed + param.minimumHeadwayDistanceAtStop ){

                            strForDebugRiskEval += QString("A%1; ttc=%2 L=%3\n").arg( memory.perceptedObjects[i]->objectID )
                                    .arg( ttc ).arg( L ) ;

                            continue;
                        }
                        shouldStop = true;
                    }

                    strForDebugRiskEval += QString("[CA] ObjID=%1, distToObj=%2 relV=%3 shouldStop=%4\n").arg(memory.perceptedObjects[i]->objectID).arg(memory.perceptedObjects[i]->distanceToObject).arg(relV).arg(shouldStop);
                }

                if( shouldStop == true ){

                    // To avoid dead-lock
                    if( agentKind >= 100 && memory.perceptedObjects[i]->objectType < 100 && memory.perceptedObjects[i]->V < 0.5 ){

                        float shift1 = tL + 1.5;
                        float shift2 = tR - 1.5;

                        memory.lateralShiftTarget = ( fabs(shift1) < fabs(shift2) ? shift1 : shift2 );
                        memory.distToAvoidTarget = memory.perceptedObjects[i]->distanceToObject;

                        memory.avoidTarget = memory.perceptedObjects[i]->objectID;

                        strForDebugRiskEval += QString("shift1=%1, shift2=%2 lateralShiftTarget=%3 distToAvoidTarget=%4\n")
                                .arg(shift1).arg(shift2)
                                .arg(memory.lateralShiftTarget).arg(memory.distToAvoidTarget);

                        if( ableToAvoid ==true ){
                        	avoidByShift = true;
                        }
                    }
                    else if( agentKind < 100 && memory.perceptedObjects[i]->objectType < 100 && memory.perceptedObjects[i]->V < 0.5 ){

                        strForDebugRiskEval += QString("eRrel=%1, tL=%2\n").arg(eRrel).arg(tL);

                        if( eRrel < tL && tL < 0.0 &&  (tL - eRrel) < 1.5 - vHalfWidth ){

                            if( ableToAvoid ==true ){
                            	avoidByShift = true;
                            }

                            memory.lateralShiftTarget += (tL - eRrel) + 0.25;

                            if( memory.lateralShiftTarget > 0.5 ){
                                memory.lateralShiftTarget = 0.5;
                            }
                            else if( memory.lateralShiftTarget < -0.5 ){
                                memory.lateralShiftTarget = -0.5;
                            }
                        }
                        else{

                            if( memory.doStopControl == true ){
                                float d = memory.perceptedObjects[i]->distanceToObject - param.minimumHeadwayDistanceAtStop - vHalfLength;
                                if( memory.distanceToStopPoint > d ){
                                    memory.distanceToStopPoint = d;
                                    memory.releaseStopCount = -1;
                                    memory.causeOfStopControl = QString("Avoid Colision: Obj%1").arg( memory.perceptedObjects[i]->objectID );
                                    memory.hazardusObject = memory.perceptedObjects[i]->objectID;
                                }
                            }
                            else{
                                memory.doStopControl = true;
                                memory.distanceToStopPoint = memory.perceptedObjects[i]->distanceToObject - param.minimumHeadwayDistanceAtStop - vHalfLength;
                                memory.releaseStopCount = -1;
                                memory.causeOfStopControl = QString("Avoid Colision: Obj%1").arg( memory.perceptedObjects[i]->objectID );
                                memory.hazardusObject = memory.perceptedObjects[i]->objectID;
                            }
                        }
                    }
                    else{

                        strForDebugRiskEval += QString("agentKind=%1, objectType=%2 V=%3\n")
                                .arg(agentKind).arg(memory.perceptedObjects[i]->objectType)
                                .arg(memory.perceptedObjects[i]->V);

                        if( memory.doStopControl == true ){
                            float d = memory.perceptedObjects[i]->distanceToObject - param.minimumHeadwayDistanceAtStop - vHalfLength;
                            if( memory.distanceToStopPoint > d ){
                                memory.distanceToStopPoint = d;
                                memory.releaseStopCount = -1;
                                memory.causeOfStopControl = QString("Avoid Colision: Obj%1").arg( memory.perceptedObjects[i]->objectID );
                                memory.hazardusObject = memory.perceptedObjects[i]->objectID;
                            }
                        }
                        else{
                            memory.doStopControl = true;
                            memory.distanceToStopPoint = memory.perceptedObjects[i]->distanceToObject - param.minimumHeadwayDistanceAtStop - vHalfLength;
                            memory.releaseStopCount = -1;
                            memory.causeOfStopControl = QString("Avoid Colision: Obj%1").arg( memory.perceptedObjects[i]->objectID );
                            memory.hazardusObject = memory.perceptedObjects[i]->objectID;
                        }

                        if( memory.avoidTarget < -1 && fabs(memory.lateralShiftTarget) > 0.0 && objTypeForUE4 == 2 ){
                            memory.lateralShiftTarget = -vHalfWidth;
                        }
                    }
                    continue;
                }
            }


            if( agentKind >= 100 || memory.perceptedObjects[i]->relPosEvaled == false ){

                float rx = memory.perceptedObjects[i]->x - state.x;
                float ry = memory.perceptedObjects[i]->y - state.y;
                float distToObj = rx * state.cosYaw + ry * state.sinYaw;
                float devObj = rx * (-state.sinYaw) + ry * state.cosYaw;

                if( distToObj < -vHalfLength - memory.perceptedObjects[i]->vHalfLength ){
                    continue;
                }

                float tV = state.cosYaw * memory.perceptedObjects[i]->cos_yaw + state.sinYaw * memory.perceptedObjects[i]->sin_yaw;
                float nV = (-state.sinYaw) * memory.perceptedObjects[i]->cos_yaw + state.cosYaw * memory.perceptedObjects[i]->sin_yaw;
                float eW = fabs(nV) * memory.perceptedObjects[i]->vHalfLength + fabs(tV) * memory.perceptedObjects[i]->vHalfWidth;

                float tL = devObj + eW * fact;
                float tR = devObj - eW * fact;
                if( tR* fact > eL + 0.20 || eR - 0.20 > tL * fact ){
                    continue;
                }

                float relV = state.V;
                if( agentKind >= 100 ){
                    if( memory.perceptedObjects[i]->objectType >= 100){
                        relV -= memory.perceptedObjects[i]->V;
                    }
                }
                else{
                    float Vt = memory.perceptedObjects[i]->innerProductToNearestPathTangent;
                    Vt *= memory.perceptedObjects[i]->V;
                    relV -= Vt;
                }
                if( relV < -0.1 ){
                    continue;
                }
                else if( relV < 0.0 ){
                    relV = 0.0;
                }

                float L = distToObj - vHalfLength - memory.perceptedObjects[i]->vHalfLength;
                if( L < param.minimumHeadwayDistanceAtStop ){
                    shouldStop = true;
                }
                else{
                    float ttc = distToObj / (relV + 0.1);
                    if( ttc > 1.5 || L > memory.distanceToZeroSpeed + param.minimumHeadwayDistanceAtStop ){
                        continue;
                    }
                    shouldStop = true;
                }

                strForDebugRiskEval += QString("[CA] ObjID=%1, distToObj=%2 relV=%3 shouldStop=%4\n").arg(memory.perceptedObjects[i]->objectID).arg(distToObj).arg(relV).arg(shouldStop);

                if( shouldStop == true ){

                    // To avoid dead-lock
                    if( agentKind >= 100 && memory.perceptedObjects[i]->objectType < 100 && memory.perceptedObjects[i]->V < 1.0 ){

                        float shift1 = tL + memory.lateralDeviationFromTargetPath + 0.5;
                        float shift2 = tR + memory.lateralDeviationFromTargetPath - 0.5;

                        memory.lateralShiftTarget = ( fabs(shift1) < fabs(shift2) ? shift1 : shift2 );
                        memory.distToAvoidTarget = distToObj;

                        strForDebugRiskEval += QString("shift1=%1, shift2=%2 lateralShiftTarget=%3 distToAvoidTarget=%4\n")
                                .arg(shift1).arg(shift2)
                                .arg(memory.lateralShiftTarget).arg(memory.distToAvoidTarget);

                        if( ableToAvoid ==true ){
                        	avoidByShift = true;
                        }
                    }
                    else if( agentKind < 100 && memory.perceptedObjects[i]->objectType < 100 && memory.perceptedObjects[i]->filteredV < 0.1 ){

                        strForDebugRiskEval += QString("eR=%1, tL=%2\n").arg(eR).arg(tL);

                        if( eR < tL && tL < 0.0 &&  (tL - eR) < 1.5 - vHalfWidth ){

                            if( ableToAvoid ==true ){
                            	avoidByShift = true;
                            }
                            memory.lateralShiftTarget += (tL - eR) + 0.25;

                            if( memory.lateralShiftTarget > 0.5 ){
                                memory.lateralShiftTarget = 0.5;
                            }
                            else if( memory.lateralShiftTarget < -0.5 ){
                                memory.lateralShiftTarget = -0.5;
                            }
                        }
                        else{

                            if( memory.doStopControl == true ){
                                float d = distToObj - param.minimumHeadwayDistanceAtStop - vHalfLength;
                                if( memory.distanceToStopPoint > d ){
                                    memory.distanceToStopPoint = d;
                                    memory.releaseStopCount = -1;
                                    memory.causeOfStopControl = QString("Avoid Colision: Obj%1").arg( memory.perceptedObjects[i]->objectID );
                                    memory.hazardusObject = memory.perceptedObjects[i]->objectID;
                                }
                            }
                            else{
                                memory.doStopControl = true;
                                memory.distanceToStopPoint = distToObj - param.minimumHeadwayDistanceAtStop - vHalfLength;
                                memory.releaseStopCount = -1;
                                memory.causeOfStopControl = QString("Avoid Colision: Obj%1").arg( memory.perceptedObjects[i]->objectID );
                                memory.hazardusObject = memory.perceptedObjects[i]->objectID;
                            }
                        }

                    }
                    else{
                        if( memory.doStopControl == true ){
                            float d = distToObj - param.minimumHeadwayDistanceAtStop - vHalfLength;
                            if( memory.distanceToStopPoint > d ){
                                memory.distanceToStopPoint = d;
                                memory.releaseStopCount = -1;
                                memory.causeOfStopControl = QString("Avoid Colision: Obj%1").arg( memory.perceptedObjects[i]->objectID );
                                memory.hazardusObject = memory.perceptedObjects[i]->objectID;  
                            }
                        }
                        else{
                            memory.doStopControl = true;
                            memory.distanceToStopPoint = distToObj - param.minimumHeadwayDistanceAtStop - vHalfLength;
                            memory.releaseStopCount = -1;
                            memory.causeOfStopControl = QString("Avoid Colision: Obj%1").arg( memory.perceptedObjects[i]->objectID );
                            memory.hazardusObject = memory.perceptedObjects[i]->objectID;
                        }
                    }
                }
            }
        }


//        if( agentKind >= 100 ){
//            if( memory.lateralShiftTarget > ppHalfWidth ){
//                memory.lateralShiftTarget = ppHalfWidth;
//            }
//            else if( memory.lateralShiftTarget < -ppHalfWidth ){
//                memory.lateralShiftTarget = -ppHalfWidth;
//            }
//        }


        // Dead-Lock Check
        if( state.V < 3.0 && memory.doStopControl == true && memory.hazardusObject >= 0 ){

            if( (pAgent[memory.hazardusObject]->memory.lastHazardusObject == ID || pAgent[memory.hazardusObject]->memory.causeOfStopControl.contains("Yeild 2") == true )
                  &&  pAgent[memory.hazardusObject]->state.V < 0.1 ){

                strForDebugRiskEval += QString("Detect Dead Lock: obj = %1\n").arg(memory.hazardusObject);

                if( memory.speedZeroCount > pAgent[memory.hazardusObject]->memory.speedZeroCount ||
                        ( memory.speedZeroCount == pAgent[memory.hazardusObject]->memory.speedZeroCount &&
                          ID < memory.hazardusObject) ||
                        pAgent[memory.hazardusObject]->memory.causeOfStopControl.contains("Yeild 2") == true ){

                    memory.doStopControl = false;
                    memory.ignoreHazardusObject = memory.hazardusObject;
                    memory.hazardusObject = -1;
                    memory.lastHazardusObject = -1;

                    strForDebugRiskEval += QString("Release Lock, given priority\n");
                }
            }
        }


        //
        //  Force to remove stacked objects
        //
        if( memory.distanceToCurrentTargetNodeWPIn < -(vHalfLength + 5.0) &&
                memory.speedZeroCount * calInterval * decisionMakingCountMax > 180.0 ){
            for(int i=0;i<memory.perceptedObjects.size();++i){
                if( memory.perceptedObjects[i]->isValidData == false ){
                    continue;
                }
                if( memory.perceptedObjects[i]->objectType >= 100 ){
                    continue;
                }
                if( memory.perceptedObjects[i]->V < 0.1 ){
                    float dx = memory.perceptedObjects[i]->x - state.x;
                    float dy = memory.perceptedObjects[i]->y - state.y;
                    float R2 = dx * dx + dy * dy;
                    if( R2 < 225 ){
                        int objID = memory.perceptedObjects[i]->objectID;
                        pAgent[objID]->agentStatus = 2;
                    }
                }
            }
            agentStatus = 2;

            qDebug() << "!!!!!";
            qDebug() << "Force to remove stacked vehicles ... (" << state.x << "," << state.y << ")";
            qDebug() << "!!!!!";
        }


        if( avoidByShift == true && memory.doStopControl == true && memory.causeOfStopControl.contains("Avoid Colision") == true ){

            memory.doStopControl = false;

        }

    }
    else{

        if( memory.LCCheckState == 1 ){
            memory.LCCheckState = 3;
        }

        if( memory.LCCheckState == 3 ){

            memory.targetPathList.clear();
            memory.targetPathList = memory.laneChangeTargetPathList;

            //qDebug() << "targetPathList = " << memory.targetPathList;

            memory.targetPathLength.clear();
            for(int j=0;j<memory.targetPathList.size();++j){

                float len = pRoad->GetPathLength( memory.targetPathList[j] );
                memory.targetPathLength.append( len );

                if( memory.targetPathList[j] == memory.currentTargetPath ){
                    memory.currentTargetPathIndexInList = j;
                }
            }


            float tdev,txt,tyt,txd,tyd,ts;
            int currentPath = pRoad->GetNearestPathFromList( memory.targetPathList,
                                                             state.x,
                                                             state.y,
                                                             state.z_path,
                                                             state.yaw,
                                                             tdev, txt, tyt, txd, tyd, ts );

            //qDebug() << "searched currentPath = " << currentPath;

            if( currentPath >= 0 ){

                memory.currentTargetPath = currentPath;
                memory.lateralDeviationFromTargetPath = tdev;

                int pIdx = pRoad->pathId2Index.indexOf( currentPath );
                if( isScenarioObject == true ){
                    if( pRoad->paths[pIdx]->speed85pt == pRoad->paths[pIdx]->speedInfo ){
                        refSpeedMode = 1;
                    }
                    else{
                        refSpeedMode = 0;
                    }
                }
                SetTargetSpeedIndividual( pRoad->paths[pIdx]->speed85pt );

                SetTargetNodeListByTargetPaths( pRoad );

            }
            else{
                qDebug() << "Lane-Change : exchange targetPathList";
                qDebug() << "Agent[" << ID << "]: Cannot determine nearest path from targetPathList;" << memory.targetPathList;
            }

            memory.laneMerge.clear();

            int i = memory.routeIndex;
            int selIdx = 0;
            for(int j=0;j<memory.LCSupportRouteLaneIndex;++j){
                selIdx += pRoad->odRoute[i]->LCSupportLaneLists[j]->laneList.size();
            }
            selIdx += memory.routeLaneIndex;

            if( selIdx < pRoad->odRoute[i]->mergeLanesInfo.size() ){
                for(int k=0;k<pRoad->odRoute[i]->mergeLanesInfo[selIdx].size();++k){

                    QPoint pairData;
                    pairData.setX( pRoad->odRoute[i]->mergeLanesInfo[selIdx][k].x() );
                    pairData.setY( pRoad->odRoute[i]->mergeLanesInfo[selIdx][k].y() );

                    memory.laneMerge.append( pairData );
                }
            }
            else{
                qDebug() << "Agent[" << ID << "]: Cannot determine mergeLanesInfo index";
                qDebug() << "  selIdx = " << selIdx << " size = " << pRoad->odRoute[i]->mergeLanesInfo.size();
            }

            memory.LCCheckState = 4;
            memory.LCSteerMax = 0.0;
            memory.checkSideVehicleForLC = false;
        }
    }

}


