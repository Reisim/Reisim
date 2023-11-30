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


void Agent::Control(Road* pRoad)
{
//    qDebug() << "agent: ID = " << ID << " kind=" << agentKind << " control-mode=" << memory.controlMode
//             << " x = " << state.x << " y=" << state.y;


    if( agentKind < 100 ){

        memory.timeToZeroSpeed = state.V / param.accelControlGain;
        memory.distanceToZeroSpeed = state.V * state.V * 0.5f / param.accelControlGain + vHalfLength;
        memory.distanceToZeroSpeedByMaxBrake = state.V * state.V * 0.5f / param.maxDeceleration + state.V + vHalfLength;
        memory.requiredDistToStopFromTargetSpeed = memory.targetSpeed * memory.targetSpeed * 0.5f / param.accelControlGain + vHalfLength;
        if( state.accel > 0.0 ){
            memory.distanceToZeroSpeed += state.V;
            memory.timeToZeroSpeed += 1.0;
            memory.requiredDistToStopFromTargetSpeed += memory.targetSpeed;
        }

        memory.minimumDistanceToStop = state.V * state.V * 0.5f / param.maxDeceleration;


        int cIdx = memory.currentTargetPathIndexInList;


        // Fluctuation in Speed Control
        int tpIdx = pRoad->pathId2Index.indexOf( memory.currentTargetPath );
        if( pRoad->paths[tpIdx]->setSpeedVariationParam == true && param.deadZoneSpeedControl > 0.5 ){

            param.refVforDev     = pRoad->paths[tpIdx]->refVforDev;
            param.vDevAllowPlus  = pRoad->paths[tpIdx]->vDevP;
            param.vDevAllowMinus = pRoad->paths[tpIdx]->vDevM;
            param.accelAtVDev    = pRoad->paths[tpIdx]->accelAtDev;
            param.decelAtVDev    = pRoad->paths[tpIdx]->decelAtDev;
            param.deadZoneSpeedControl = 0.0;

        }


        //
        // Longitudinal Control
        //
        if( memory.controlMode == AGENT_CONTROL_MODE::AGENT_LOGIC ){

            controlCount++;
            if( controlCount >= controlCountMax ){
                controlCount = 0;
            }
            else{
                if( memory.LCCheckState == 0 ){
                    return;
                }
            }

            memory.accel = 0.0;
            memory.brake = 0.0;

            if( isScenarioObject == true ){

                strForDebug += QString("setTargetSpeedByScenarioFlag = %1\n").arg(memory.setTargetSpeedByScenarioFlag);

                if( memory.setTargetSpeedByScenarioFlag == true ){
                    memory.actualTargetSpeed = memory.targetSpeedByScenario;
                    memory.distanceAdjustLowSpeed = 0.0;

                    strForDebug += QString("actualTargetSpeed = %1[A1]\n").arg(memory.actualTargetSpeed);

                    SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeedByScenario );

                    strForDebug += QString("actualTargetSpeed = %1[B1]\n").arg(memory.actualTargetSpeed);
                }
                else{
                    memory.actualTargetSpeed = memory.targetSpeed;
                    memory.distanceAdjustLowSpeed = 0.0;

                    strForDebug += QString("actualTargetSpeed = %1[A2]\n").arg(memory.actualTargetSpeed);

                    SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeed );

                    strForDebug += QString("actualTargetSpeed = %1[B2]\n").arg(memory.actualTargetSpeed);
                }
            }
            else{
                memory.actualTargetSpeed = memory.targetSpeed;
                memory.distanceAdjustLowSpeed = 0.0;

                strForDebug += QString("actualTargetSpeed = %1[A]\n").arg(memory.actualTargetSpeed);

                SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeed );

                strForDebug += QString("actualTargetSpeed = %1[B]\n").arg(memory.actualTargetSpeed);
            }


//            if( memory.doHeadwayDistanceControl == true ){
//                if( memory.distanceToPrecedingVehicle < 3.0 * state.V ){
//                    if( memory.speedPrecedingVehicle < memory.actualTargetSpeed ){
//                        memory.actualTargetSpeed = memory.speedPrecedingVehicle;
//                    }
//                }
//            }

            SpeedControl();

            float ax_com = memory.axSpeedControl;

            //qDebug() << "[ID=" << ID << "] ax_com = " << ax_com << " - V";

            if( memory.doHeadwayDistanceControl == true ){

                memory.actualTargetHeadwayDistance = memory.targetHeadwayDistance;

                HeadwayControlAgent();
                if( memory.axHeadwayControl < 0.0 && ax_com > memory.axHeadwayControl ){
                    ax_com = memory.axHeadwayControl;
                    //qDebug() << "[ID=" << ID << "] ax_com = " << ax_com << " - H";
                }

            }

            if( memory.doStopControl == true ){

                StopControl();
                if( memory.axStopControl < 0.0 && ax_com > memory.axStopControl ){
                    ax_com = memory.axStopControl;
                    //qDebug() << "[ID=" << ID << "] ax_com = " << ax_com << " - S";
                }

            }

            // Deceleration for Lane-Change
            if( memory.checkSideVehicleForLC == true && memory.LCCheckState == 2 && memory.sideVehicleRiskClear == false &&
                    state.V > memory.targetSpeed * 0.25 ){
                if( ax_com > param.accelControlGain * (-1.0) ){
                    ax_com = param.accelControlGain * (-1.0);
                }
            }

            if( ax_com >= 0.0f ){
                memory.accel = ax_com;
            }
            else{
                memory.brake = -ax_com;
            }
        }
        else if( memory.controlMode == AGENT_CONTROL_MODE::SPEED_PROFILE ){

            memory.accel = 0.0;
            memory.brake = 0.0;

            memory.speedProfileCount++;
            float currentProfTime = memory.speedProfileCount * calInterval;

            if( memory.profileTime.last() <= currentProfTime ){

                memory.actualTargetSpeed = memory.profileSpeed.last();
                memory.distanceAdjustLowSpeed = 0.0;
                memory.protectProfileData = false;

                float ax_com = 0.0;

                if( memory.disableSpeedAdjustForCurveByScenario == false ){
                    SpeedAdjustForCurve( pRoad, cIdx, memory.profileSpeed.last() );
                }

                memory.activeBrakeInVelocityControl = true;
                SpeedControl();
                memory.activeBrakeInVelocityControl = false;

                ax_com = memory.axSpeedControl;

                if( ax_com >= 0.0 ){
                    memory.accel = ax_com;
                }
                else{
                    memory.brake = -ax_com;
                }
            }
            else{
                for(int i=1;i<memory.profileTime.size();++i){
                    if( memory.profileTime[i-1] <= currentProfTime && currentProfTime < memory.profileTime[i] ){

                        float deltaT = memory.profileTime[i] - memory.profileTime[i-1];
                        float deltaV = memory.profileSpeed[i] - memory.profileSpeed[i-1];
                        float aReq = deltaV / deltaT;
                        float targetProfSpeed = memory.profileSpeed[i-1] + aReq * ( currentProfTime - memory.profileTime[i-1] );
                        float err = targetProfSpeed - state.V;

                        if( targetProfSpeed < 0.1 && state.V < 0.1 ){
                            aReq = (-1.0) * param.maxDeceleration;
                        }

                        float ax_com = aReq + err * param.accelControlGain * 1.5;
                        if( ax_com >= 0.0 ){
                            memory.accel = ax_com;
                        }
                        else{
                            memory.brake = -ax_com;
                        }

                        //qDebug() << "Vref = " << targetProfSpeed << " V = " <<  state.V << " ax = " << ax_com / 9.81;

                        break;
                    }
                }
            }
        }
        else if( memory.controlMode == AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY ){

//            qDebug() << " PV=" << memory.precedingVehicleIDByScenario
//                     << " D=" << memory.distanceToPrecedingVehicle
//                     << " do=" << memory.doHeadwayDistanceControl
//                     << " Vr=" << memory.targetSpeedByScenario
//                     << " HD=" << memory.targetHeadwayDistanceByScenario
//                     << " HT=" << memory.targetHeadwayTimeByScenario;

            memory.accel = 0.0;
            memory.brake = 0.0;

            float ax_com = 0.0;

            float isFollowing = 1.0;
            if( memory.distanceToPrecedingVehicle < 0.0f ){
                isFollowing = -1.0;
            }

            if( isFollowing < 0.0 ){

                memory.distanceAdjustLowSpeed = 0.0;

                if( memory.doHeadwayDistanceControl == true ){

                    float headwayDist = memory.targetHeadwayDistanceByScenario;
                    float devHeadwayDist = fabs(memory.distanceToPrecedingVehicle) - headwayDist;

//                    qDebug() << " HD=" << headwayDist
//                             << " dist=" << memory.distanceToPrecedingVehicle
//                             << " Dev=" << devHeadwayDist
//                             << " A=" << memory.allowableHeadwayDistDeviation;

                    if( fabs(devHeadwayDist) < memory.allowableHeadwayDistDeviation ){

                        memory.actualTargetSpeed = memory.targetSpeedByScenario;

                        if( memory.disableSpeedAdjustForCurveByScenario == false ){
                            SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeedByScenario );
                        }

                        SpeedControl();

                        ax_com = memory.axSpeedControl;

//                        qDebug() << " Normal Vel Con : ax=" << ax_com;
                    }
                    else{

                        if( devHeadwayDist > memory.allowableHeadwayDistDeviation ){
                            devHeadwayDist -= memory.allowableHeadwayDistDeviation;
                        }
                        else if( devHeadwayDist < (-1.0) * memory.allowableHeadwayDistDeviation ){
                            devHeadwayDist += memory.allowableHeadwayDistDeviation;
                        }

                        float speedDev = (-1.0) * devHeadwayDist / 2.0;    // 2.0 should be driver-characterisitc dependent gain
                        if( speedDev > 5.55 ){
                            speedDev = 5.55;
                        }
                        else if( speedDev < -5.55 ){
                            speedDev = -5.55;
                        }

                        memory.actualTargetSpeed = memory.speedPrecedingVehicle + speedDev;
                        if( memory.actualTargetSpeed < 0.0 ){
                            memory.actualTargetSpeed = 0.0;
                        }

//                        qDebug() << " speedDev =" << speedDev
//                                 << " actualTargetSpeed = " << memory.actualTargetSpeed;

                        if( memory.disableSpeedAdjustForCurveByScenario == false ){
                            SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeedByScenario );
                        }

                        memory.activeBrakeInVelocityControl = true;
                        SpeedControl();
                        memory.activeBrakeInVelocityControl = false;

                        ax_com = memory.axSpeedControl;

//                        qDebug() << " VMod Vel Con : ax=" << ax_com;
                    }
                }
                else{

                    memory.actualTargetSpeed = memory.targetSpeedByScenario;

                    if( memory.disableSpeedAdjustForCurveByScenario == false ){
                        SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeedByScenario );
                    }

                    SpeedControl();

                    ax_com = memory.axSpeedControl;
                }

                if( memory.actualTargetSpeed < 0.01 ){
                    memory.axSpeedControl = -1.0;
                    ax_com = -1.0;
                }
            }
            else{

                memory.actualTargetSpeed = memory.targetSpeedByScenario;
                memory.distanceAdjustLowSpeed = 0.0;

                if( memory.disableSpeedAdjustForCurveByScenario == false ){
                    SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeedByScenario );
                }

                memory.activeBrakeInVelocityControl = true;
                SpeedControl();
                memory.activeBrakeInVelocityControl = false;

                ax_com = memory.axSpeedControl;

                if( memory.doHeadwayDistanceControl == true ){

                    //
                    //  Determine target headway distance
                    //
                    if( fabs(memory.targetHeadwayDistanceByScenario) < 0.001f ){

                        if( fabs(memory.targetHeadwayTimeByScenario) < 0.001f ){
                            memory.actualTargetHeadwayDistanceByScenario = 0.0;
                        }
                        else{
                            float dist2 = memory.targetHeadwayTimeByScenario * state.V;
                            if( dist2 < param.minimumHeadwayDistanceAtStop ){
                                dist2 = param.minimumHeadwayDistanceAtStop;
                            }
                            memory.actualTargetHeadwayDistanceByScenario = dist2;
                        }
                    }
                    else if( fabs(memory.targetHeadwayTimeByScenario) < 0.001f ){

                        float dist1 = memory.targetHeadwayDistanceByScenario;
                        if( dist1 < param.minimumHeadwayDistanceAtStop ){
                            dist1 = param.minimumHeadwayDistanceAtStop;
                        }
                        memory.actualTargetHeadwayDistanceByScenario = dist1;
                    }
                    else{
                        float dist1 = memory.targetHeadwayDistanceByScenario;
                        float dist2 = memory.targetHeadwayTimeByScenario * state.V;
                        if( dist2 < param.minimumHeadwayDistanceAtStop ){
                            dist2 = param.minimumHeadwayDistanceAtStop;
                        }
                        memory.actualTargetHeadwayDistanceByScenario = dist1 > dist2 ? dist1 : dist2;
                    }

                    memory.actualTargetHeadwayDistance = memory.actualTargetHeadwayDistanceByScenario;

                    if( memory.actualTargetHeadwayDistance > 0.0f ){

                        HeadwayControl();

                        if( memory.doHeadwayDistanceControl == true ){
                                if( ax_com > memory.axHeadwayControl ){
                                ax_com = memory.axHeadwayControl;
                            }
                        }
                    }
                }

                if( memory.actualTargetSpeed < 0.01 ){
                    memory.axSpeedControl = -1.0;
                    ax_com = -1.0;
                }
            }

            if( ax_com >= 0.0f ){
                memory.accel = ax_com;
            }
            else{
                memory.brake = -ax_com;
            }
        }
        else if( memory.controlMode == AGENT_CONTROL_MODE::STOP_AT ){

            memory.accel = 0.0;
            memory.brake = 0.0;

            float ax_com = 0.0;

            memory.actualTargetSpeed = memory.targetSpeedByScenario;
            memory.distanceAdjustLowSpeed = 0.0;

            if( memory.disableSpeedAdjustForCurveByScenario == false ){
                SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeedByScenario );
            }

            memory.activeBrakeInVelocityControl = true;
            SpeedControl();
            memory.activeBrakeInVelocityControl = false;

            ax_com = memory.axSpeedControl;

            memory.doStopControl = true;
            memory.actualStopAtX = memory.targetStopAtXByScenario;
            memory.actualStopAtY = memory.targetStopAtYByScenario;
            memory.causeOfStopControl = QString("AGENT_CONTROL_MODE::STOP_AT");

            if( memory.actualStopOnPathID < 0 ){
                for(int k=0;k<memory.targetPathList.size();++k){

                    float tdev,txt,tyt,txd,tyd,ts;
                    int chk = pRoad->GetDeviationFromPath( memory.targetPathList[k],
                                                           memory.actualStopAtX, memory.actualStopAtY, 0.0,
                                                           tdev, txt, tyt, txd, tyd, ts,
                                                           true );
                    if( chk != memory.targetPathList[k] ){
                        continue;
                    }
                    else{
                        memory.actualStopOnPathID = chk;
                        memory.actualStopOnPathIndex = k;
                        memory.distToStopAtOnThePath = ts;

//                        qDebug() << "actualStopOnPathID = " << memory.actualStopOnPathID;
//                        qDebug() << "actualStopOnPathIndex = " << memory.actualStopOnPathIndex;
//                        qDebug() << "distToStopAtOnThePath = " << memory.distToStopAtOnThePath;

                        break;
                    }
                }
            }

            float dist = 0.0;
            if( memory.actualStopOnPathIndex < memory.currentTargetPathIndexInList ){
                for(int l=memory.currentTargetPathIndexInList; l>memory.actualStopOnPathIndex ; l-- ){
                    dist += pRoad->GetPathLength( memory.targetPathList[l] );
                }
                dist += memory.distToStopAtOnThePath;
                dist -= memory.distanceFromStartWPInCurrentPath;
            }
            else if( memory.actualStopOnPathIndex == memory.currentTargetPathIndexInList ){
                dist = memory.distToStopAtOnThePath - memory.distanceFromStartWPInCurrentPath;
            }
            else{
                for(int l=memory.currentTargetPathIndexInList+1; l<=memory.actualStopOnPathIndex ; l++ ){
                    dist -= pRoad->GetPathLength( memory.targetPathList[l] );
                }
                dist += memory.distToStopAtOnThePath;
                dist -= memory.distanceFromStartWPInCurrentPath;
            }

            memory.distanceToStopPoint = dist - vHalfLength;

//            qDebug() << "distanceToStopPoint = " << memory.distanceToStopPoint
//                     << " Index=" << memory.actualStopOnPathIndex
//                     << " Current=" << memory.currentTargetPathIndexInList
//                     << " S = " << memory.distanceFromStartWPInCurrentPath
//                     << " L = " << memory.distToStopAtOnThePath;


            StopControl();

            //qDebug() << "doStopControl = " << memory.doStopControl << " axStopControl = " << memory.axStopControl;

            if( memory.doStopControl == true && ax_com > memory.axStopControl ){

                //qDebug() << "ax_com = " << ax_com << " -> " << memory.axStopControl;
                ax_com = memory.axStopControl;
            }

            if( ax_com >= 0.0f ){
                memory.accel = ax_com;
            }
            else{
                memory.brake = -ax_com;
            }
        }
        else if( memory.controlMode == AGENT_CONTROL_MODE::INTERSECTION_TURN_CONTROL ){

            memory.accel = 0.0;
            memory.brake = 0.0;


            float ax_com = 0.0;

            if( cIdx == 0 ){

                memory.actualTargetSpeed = memory.targetSpeedByScenario;
                SpeedControl();
                ax_com = memory.axSpeedControl;
                memory.startDecel = 0;

                vehicle.SetWinker( 0 );
            }
            else if( cIdx == 1 ){

                memory.actualTargetSpeed = memory.targetSpeedInsideIntersectionTurnByScenario;
                SpeedControl();
                ax_com = memory.axSpeedControl;
                memory.startDecel = 0;

            }
            else if( cIdx == 2 ){

                memory.actualTargetSpeed = memory.targetSpeedByScenario;
                SpeedControl();
                ax_com = memory.axSpeedControl;

                float D = pRoad->GetPathLength( memory.targetPathList[cIdx] );
                D -= memory.distanceFromStartWPInCurrentPath;

                // Winker
                if( (D < 30.0 || D < state.V * 3.0) && vehicle.GetWinerState() == 0 ){

                    int curvePath = memory.targetPathList[1];
                    int cpidx = pRoad->pathId2Index.indexOf(curvePath);
                    float c1 = pRoad->paths[cpidx]->derivative.first()->x();
                    float s1 = pRoad->paths[cpidx]->derivative.first()->y();
                    float c2 = pRoad->paths[cpidx]->derivative.last()->x();
                    float s2 = pRoad->paths[cpidx]->derivative.last()->y();
                    float cp = c1 * (-s2) + s1 * c2;
                    if( cp > 0.0 ){
                        vehicle.SetWinker( 2 );  // right
                    }
                    else{
                        vehicle.SetWinker( 1 );  // left
                    }
                }


                if( memory.startDecel == 0 && state.V > memory.actualTargetSpeed - 1.0 ){

                    float vp = state.V + memory.targetSpeedInsideIntersectionTurnByScenario;
                    float vm = state.V - memory.targetSpeedInsideIntersectionTurnByScenario;
                    float L = vp * vm / (param.accelControlGain * 1.5) * 0.5;

//                    qDebug() << "D = " << D << " L = " << L;
                    if( D < L ){
                        memory.startDecel = 1;
                    }
                }

                if( memory.startDecel == 1 ){
                    ax_com = param.accelControlGain * (-1.5);
                }
                if( memory.startDecel == 1 && state.V < memory.targetSpeedInsideIntersectionTurnByScenario ){
                    memory.startDecel = 2;
                }

            }

//            qDebug() << "V=" << state.V*3.6 << " idx=" << cIdx << " ax_com=" << ax_com << " startDecel=" << memory.startDecel;

            if( ax_com >= 0.0f ){
                memory.accel = ax_com;
            }
            else{
                memory.brake = -ax_com;
            }
        }


        if( memory.overrideBrakeByScenario == true ){
            memory.accel = 0.0;
            memory.brake = memory.overrideAxControl * (-1.0f);
        }


        if( memory.ADDisturbFlag == true ){

            float lt = memory.ADDisturbCount * calInterval;
            if( lt > memory.ADDisturbTime.last() ){
                memory.ADDisturbFlag = false;
            }
            else{
                for(int np=0;np<memory.ADDisturbTime.size()-1;++np){
                    if( memory.ADDisturbTime[np] <= lt && lt <= memory.ADDisturbTime[np+1] ){

                        float tt = ( lt - memory.ADDisturbTime[np] ) / ( memory.ADDisturbTime[np+1] - memory.ADDisturbTime[np] );

                        float adval = memory.ADDisturb[np] + (memory.ADDisturb[np+1] - memory.ADDisturb[np]) * tt;  // [G]

                        adval *= 9.81;  // [m/s^2]

                        if( adval >= 0.0f ){
                            memory.accel = adval;
                            memory.brake = 0.0;
                        }
                        else{
                            memory.accel = 0.0;
                            memory.brake = adval * (-1.0f);
                        }

                        break;
                    }
                }

                memory.ADDisturbCount++;
            }
        }


        //
        // Lateral Control
        //
        if( isScenarioObject == true ){
            memory.lateralDeviationFromTargetPathAtPreviewPoint -= memory.targetLateralShiftByScenario;
        }

        if( memory.additionalShiftByScenarioEvent == true ){
            memory.lateralDeviationFromTargetPathAtPreviewPoint -= memory.additionalLateralShift;
    //        qDebug() << "devAP = " << memory.lateralDeviationFromTargetPathAtPreviewPoint
    //                 << " dev = " << memory.lateralDeviationFromTargetPath
    //                 << " shift = " << memory.additionalLateralShift;
        }

        float lowSpeedAdjustGain = 1.0;
        if( state.V < 8.0 ){
            lowSpeedAdjustGain = 1.5 - 0.5 * state.V / 8.0;
        }

        float highSpeedAdjustGain = 1.0;
        if( state.V > 22.22 ){
            highSpeedAdjustGain = 1.0 - (state.V - 22.22) / 11.11 * 0.85;
            if( highSpeedAdjustGain < 0.15 ){
                highSpeedAdjustGain = 0.15;
            }
        }

        float Y = memory.lateralDeviationFromTargetPathAtPreviewPoint - memory.lateralShiftTarget;

        float oldSteer = memory.steer;
        float nextSteer = (-1.0) * Y * param.steeringControlGain * lowSpeedAdjustGain * highSpeedAdjustGain;


        // Rate Limit
        int nDecination = 1;
        if( memory.controlMode == AGENT_CONTROL_MODE::AGENT_LOGIC && memory.LCCheckState == 0 ){
            nDecination = controlCountMax;
        }

        float maxRate = 1.57;
        if( state.V < 16.66 ){
            maxRate = 6.28 + 1.57;
        }
        float steerVel = (nextSteer - oldSteer) / (calInterval * nDecination);
        if( steerVel > maxRate ){
            steerVel = maxRate;
        }
        else if( steerVel < -maxRate ){
            steerVel = -maxRate;
        }
        float ratedSteer = oldSteer + steerVel * (calInterval * nDecination);

        if( nextSteer > 0.0 && ratedSteer > 0.0 ){
            if( nextSteer > ratedSteer ){
                memory.steer = ratedSteer;
            }
            else{
                memory.steer = nextSteer;
            }
        }
        else if( nextSteer < 0.0 && ratedSteer < 0.0 ){
            if( nextSteer < ratedSteer ){
                memory.steer = ratedSteer;
            }
            else{
                memory.steer = nextSteer;
            }
        }
        else{
            memory.steer = nextSteer;
        }


        // Lane-Change Lateral Speed Adjust
        if( memory.LCCheckState == 4 ){

            float latVtrue = fabs(memory.relativeAttitudeToLane * state.V);
            float latV = (memory.relativeAttitudeToLane + vehicle.state.yawRate * 0.1 )* state.V;
            if( fabs(latV) > param.maxLateralSpeedForLaneChange ){
                memory.steer = 0.0;
            }
//            if( memory.LCbyEventMode > 0 ){
//                qDebug() << " latV = " << latVtrue << " max = " << param.maxLateralSpeedForLaneChange << " steer=" << memory.steer;
//            }
        }


        // Value Limit
        float maxSteer = 6.8;

        if( memory.LCCheckState == 4 ){
            if( memory.LCSteerMax < 0.75 ){
                memory.LCSteerMax += 0.05;
            }
            maxSteer = memory.LCSteerMax;
        }


        if( memory.steer > maxSteer ){
            memory.steer = maxSteer;
        }
        else if( memory.steer < -maxSteer ){
            memory.steer = -maxSteer;
        }

        // Do not steer at stop, keep last steer
        if( state.V < 0.1 ){
            memory.steer = oldSteer;
        }

        memory.steer *= memory.steerControlGainForExternalControl;

        if( memory.overrideSteerByScenario == true ){
           memory.steer = memory.overrideSteerControl * 0.014752; // should be multiplied a gain varied depending on speed
        }


        if( memory.steerDisturbFlag == true ){

            float lt = memory.steerDisturbCount * calInterval;
            if( lt > memory.steerDisturbTime.last() ){
                memory.steerDisturbFlag = false;
            }
            else{
                for(int np=0;np<memory.steerDisturbTime.size()-1;++np){
                    if( memory.steerDisturbTime[np] <= lt && lt <= memory.steerDisturbTime[np+1] ){

                        float tt = ( lt - memory.steerDisturbTime[np] ) / ( memory.steerDisturbTime[np+1] - memory.steerDisturbTime[np] );

                        float sval = memory.steerDisturb[np] + (memory.steerDisturb[np+1] - memory.steerDisturb[np]) * tt;

                        memory.steer = memory.steerDisturbInit + sval * 0.017452 / 8.0;  // 8.0; assumed gear-ratio

                        break;
                    }
                }

                memory.steerDisturbCount++;
            }
        }

//        if( ID == 10 ){
//            qDebug() << "distanceToTurnNodeWPIn = " << memory.distanceToTurnNodeWPIn << " nextTurnNode=" << memory.nextTurnNode
//                     << " currentTargetNode=" << memory.currentTargetNode;
//        }


        // Winker Operation
        if( memory.controlMode <= AGENT_CONTROL_MODE::SPEED_PROFILE ){

            if( memory.nextTurnNode == memory.currentTargetNode ){

                if( memory.isMergeNode == -1 ){
                    memory.isMergeNode = 0;
                    // Special Case for merge
                    int ndIdx = pRoad->nodeId2Index.indexOf( memory.currentTargetNode );
                    if( pRoad->nodes[ndIdx]->legIDs.size() == 3 && pRoad->nodes[ndIdx]->hasTS == false ){
                        if( memory.currentTargetNodeInDirect == 1 ){
                            bool hasStopLine = false;
                            for(int k=memory.currentTargetPathIndexInList;k>=0;k--){
                                int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[k] );
                                if( pRoad->paths[pIdx]->connectingNode != memory.currentTargetNode ){
                                    break;
                                }
                                if( pRoad->paths[pIdx]->stopPoints.size() > 0 ){
                                    hasStopLine = true;
                                    break;
                                }
                            }
                            if( hasStopLine == false ){
                                memory.isMergeNode = 1;
                            }
                        }
                    }
                }

                if( memory.isMergeNode == 0 ){
                    if( memory.distanceToTurnNodeWPIn > 5.0 &&
                            (memory.distanceToTurnNodeWPIn <= 60.0 || memory.distanceToTurnNodeWPIn < 3.0 * state.V + 30.0) ){
                        if( memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING ){
                            vehicle.SetWinker( 1 );
                        }
                        else if( memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
                            vehicle.SetWinker( 2 );
                        }
                    }
                }
                else if( memory.isMergeNode == 1 ){
                    float Ls = 10.0;
                    if( memory.turnNodeOutWP >= 0 ){

                        strForDebug += QString("turnNodeOutWP = %1\n").arg(memory.turnNodeOutWP);

                        int wpidx = pRoad->wpId2Index.indexOf( memory.turnNodeOutWP );
                        if( wpidx >= 0 ){

                            float ipv = state.cosYaw * (pRoad->wps[wpidx]->cosDirect) + state.sinYaw * (pRoad->wps[wpidx]->sinDirect);
                            if( ipv > 0.707 ){
                                float dxv = state.x - pRoad->wps[wpidx]->pos.x();
                                float dyv = state.y - pRoad->wps[wpidx]->pos.y();

                                Ls = dxv * (-pRoad->wps[wpidx]->sinDirect) + dyv * (pRoad->wps[wpidx]->cosDirect);
                            }
//                            strForDebug += QString("x = %1 y = %2\n").arg(pRoad->wps[wpidx]->pos.x()).arg(pRoad->wps[wpidx]->pos.y());
//                            strForDebug += QString("c = %1 s = %2\n").arg(pRoad->wps[wpidx]->cosDirect).arg(pRoad->wps[wpidx]->sinDirect);
                        }
                    }


                    if( fabs(Ls) < 8.0 || memory.distanceToTurnNodeWPOut <= 30.0 || memory.distanceToTurnNodeWPOut < 3.0 * state.V ){
                        if( memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING ){
                            vehicle.SetWinker( 2 );
                        }
                        else if( memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
                            vehicle.SetWinker( 1 );
                        }
                    }
                    if( fabs(Ls) < 1.0 ){
                        vehicle.SetWinker( 0 );
                    }

                    strForDebug += QString("Ls = %1\n").arg(Ls);
                }
            }
            else{
                if( memory.distanceToTurnNodeWPIn > 5.0 ){
                    if( memory.distanceToTurnNodeWPIn <= 60.0 || memory.distanceToTurnNodeWPIn < 3.0 * state.V + 30.0 ){
                        if( memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING ){
                            vehicle.SetWinker( 1 );
                        }
                        else if( memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
                            vehicle.SetWinker( 2 );
                        }
                    }
                }
            }

        }


        if( memory.LCCheckState == 4 && fabs(memory.lateralDeviationFromTargetPath) < 0.5 ){
            if( vehicle.GetWinerState() > 0 ){
                vehicle.SetWinker( 0 );
            }
            memory.LCCheckState = 0;
            memory.LCbyEventMode = 0;
        }

    }
    else if( agentKind >= 100 ){

        memory.timeToZeroSpeed = state.V / param.accelControlGain;
        memory.distanceToZeroSpeed = state.V * state.V * 0.5f / param.accelControlGain + vHalfLength;
        memory.distanceToZeroSpeedByMaxBrake = state.V * state.V * 0.5f / param.maxDeceleration + state.V + vHalfLength;
        memory.requiredDistToStopFromTargetSpeed = memory.targetSpeed * memory.targetSpeed * 0.5f / param.accelControlGain + vHalfLength;
        memory.minimumDistanceToStop = state.V * state.V * 0.5f / param.maxDeceleration;


        controlCount++;
        if( controlCount >= controlCountMax ){
            controlCount = 0;
        }
        else{
            return;
        }


        if( memory.controlMode == AGENT_CONTROL_MODE::AGENT_LOGIC ){

            float tV = memory.targetSpeed;
            if( isScenarioObject == true ){
                tV = memory.targetSpeedByScenario;
            }

            if( memory.shouldStopAtSignalSL == true ){
                tV = 0.0;
            }

            if( memory.doHeadwayDistanceControl == true ){
                tV = 0.0;
            }

            if( memory.doStopControl == true ){
                tV = 0.0;
                for(int i=0;i<memory.perceptedObjects.size();++i){
                    if( memory.perceptedObjects[i]->objectID == memory.hazardusObject ){
                        tV = memory.perceptedObjects[i]->V;

                        if( memory.perceptedObjects[i]->distanceToObject < 1.5 + vHalfLength + memory.perceptedObjects[i]->vHalfLength ){
                            tV *= 0.9;
                        }

                        if( tV > memory.targetSpeed ){
                            tV = memory.targetSpeed;
                        }
                        break;
                    }
                }
            }

            if( objTypeForUE4 == 2 ){  // Only for bicycle

                float xp = state.x;
                float yp = state.y;

                float fact = pRoad->GetSpeedAdjustFactorPedestPath( memory.currentTargetPath,
                                                                    memory.currentTargetPathIndexInList,
                                                                    xp, yp, state.V );
                tV *= fact;

                strForDebug += QString("fact by path = %1\n").arg(fact);

                float latdev = memory.lateralShiftTarget - memory.lateralDeviationFromTargetPath;
                if( fabs(latdev) > 0.5 ){

                    fact = 0.75;
                    tV *= fact;

                    strForDebug += QString("fact by deviation = %1\n").arg(fact);

                }
            }

            if( memory.exeRunOut == true && memory.runOutState >= 1 ){
                tV = memory.targetSpeed;
            }


            float dv = (tV - state.V);
            float pedestSpeedGain = 1.0;
            if( tV > 0.1 ){
                pedestSpeedGain = 0.2 * fabs(dv);
            }

            dv *= pedestSpeedGain * calInterval * controlCountMax;

            float dvMax = 1.0 * calInterval * controlCountMax;  // Max 1[m/s/s] = 0.1[G]

            if( dv > dvMax ){
                dv = dvMax;
            }
            else if( dv < -dvMax ){
                dv = -dvMax;
            }
            state.V += dv;
            if( state.V < 0.0 ){
                state.V = 0.0;
            }
            else if( state.V > tV ){
                state.V = tV;
            }
        }
        else if( memory.controlMode == AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY ){

            state.V = memory.targetSpeedByScenario;

        }
        else if( memory.controlMode == AGENT_CONTROL_MODE::SPEED_PROFILE ){

            memory.speedProfileCount++;
            float currentProfTime = memory.speedProfileCount * controlCountMax * calInterval;

            if( memory.profileTime.last() <= currentProfTime ){

                state.V = memory.profileSpeed.last();

            }
            else{
                for(int i=1;i<memory.profileTime.size();++i){
                    if( memory.profileTime[i-1] <= currentProfTime && currentProfTime < memory.profileTime[i] ){
                        float deltaT = memory.profileTime[i] - memory.profileTime[i-1];
                        float deltaV = memory.profileSpeed[i] - memory.profileSpeed[i-1];
                        float aReq = deltaV / deltaT;
                        state.V = memory.profileSpeed[i-1] + aReq * ( currentProfTime - memory.profileTime[i-1] );
                        break;
                    }
                }
            }
            //qDebug() << "V = " << state.V;
        }


        if( memory.controlMode == AGENT_CONTROL_MODE::RUN_OUT && memory.steerDisturbFlag == true ){

            memory.steerDisturbCount++;
            if( memory.steerDisturbCount * controlCountMax * calInterval > 5.0){
                // Dispose
                agentStatus = 2;
            }

            state.yaw = memory.steerDisturbInit;
            state.cosYaw = cos( state.yaw );
            state.sinYaw = sin( state.yaw );
        }
        else{

            if( memory.exeRunOut == true && memory.runOutState >= 2 ){

                float xdir = memory.cosRunOutDir;
                float ydir = memory.sinRunOutDir;

                float mxdir = xdir * 0.15 + state.cosYaw * 0.85;
                float mydir = ydir * 0.15 + state.sinYaw * 0.85;

                state.cosYaw = mxdir;
                state.sinYaw = mydir;


                float dRx = state.x - memory.runOutStartX;
                float dRy = state.y - memory.runOutStartY;
                float distRunOut = dRx * dRx + dRy * dRy;

//                qDebug() << "A[" << ID << "]: distRunOut=" << distRunOut;

                if( distRunOut > 225 ){  // 15[m]
                    // Dispose
                    agentStatus = 2;
//                    qDebug() << "Dispose";
                }
                else{

                    // Reached other pedestrian path ?
                    for(int i=0;i<pRoad->pedestPaths.size();++i){

                        if( pRoad->pedestPaths[i]->id == memory.currentTargetPath ){
                            continue;
                        }

                        float  dev = 0.0;
                        float dist = 0.0;
                        float xdir = 0.0;
                        float ydir = 0.0;
                        int secIndx = pRoad->GetDeviationFromPedestPathAllSection( pRoad->pedestPaths[i]->id,
                                                                                   state.x,
                                                                                   state.y,
                                                                                   dev,
                                                                                   dist,
                                                                                   xdir,
                                                                                   ydir);


                        if( fabs(dev) < 1.5 && secIndx >= 0 ){

                            memory.currentTargetPath = pRoad->pedestPaths[i]->id;
                            memory.currentTargetPathIndexInList = secIndx;
                            memory.lateralDeviationFromTargetPath = dev;
                            memory.distanceFromStartWPInCurrentPath = dist;

                            memory.runOutState = -1;

//                            qDebug() << "Check PPath" << pRoad->pedestPaths[i]->id << " dev=" << dev << " secIndx = " << secIndx;
//                            qDebug() << "Changed";

                            break;
                        }
                    }
                }
            }
            else if( state.V > 0.2 ){
		
                float  dev = 0.0;
                float    z = 0.0;
                float xdir = 0.0;
                float ydir = 0.0;

                float xp = state.x;
                float yp = state.y;

                int ret = pRoad->GetDeviationFromPedestPath( memory.currentTargetPath,
                                                             memory.currentTargetPathIndexInList,
                                                             xp, yp, dev, z, xdir, ydir);

                float latdev = memory.lateralShiftTarget - dev;

                if( isScenarioObject == true ){
                    latdev += memory.targetLateralShiftByScenario;
                }

                if( ret == 1 ){
                    latdev = 0.0;
                }

                float dist = memory.distToAvoidTarget;
                if( dist < 0.5 ){
                    dist = 0.5;
                }

                if( memory.avoidTarget < 0 ){
                    dist = 5.0;
                }

                latdev /= dist; // Gain

                if( latdev > 10.0 ){
                    latdev = 10.0;
                }
                else if( latdev < -10.0 ){
                    latdev = -10.0;
                }


    //            strForDebug += QString("latdev=%1\n").arg(latdev);

                xdir -= latdev * state.sinYaw;
                ydir += latdev * state.cosYaw;

                float mxdir = xdir * 0.15 + state.cosYaw * 0.85;
                float mydir = ydir * 0.15 + state.sinYaw * 0.85;

                state.cosYaw = mxdir;
                state.sinYaw = mydir;
            }
        }

        state.z = 0.0;
    }
}




void Agent::SpeedControl()
{
    float err_speed = memory.actualTargetSpeed - state.V;    // [m/s]

//    qDebug() << "[" << ID << "] V = " << state.V << " actualTargetSpeed = " << memory.actualTargetSpeed << " err = " << err_speed;

    if( param.deadZoneSpeedControl > 0.5 ){

        float deadZone = param.deadZoneSpeedControl;
        float adFact = memory.actualTargetSpeed / 22.22;
        if( adFact > 1.0 ){
            adFact = 1.0;
        }
        deadZone *= adFact;

        if( isScenarioObject == true ){
            deadZone = 0.0;
        }

        float currentAccelGain = param.accelControlGain * (param.maxSpeedVehicle - state.V * 3.6) / param.maxSpeedVehicle;
        if( currentAccelGain < 0.0 ){
            currentAccelGain = 0.0;
        }

        if( err_speed - deadZone >= 0.0 ){  // should accelerate
            memory.speedControlState = 0;
            memory.axSpeedControl = currentAccelGain;
        }
        else if( fabs(err_speed) < deadZone ){
            if( memory.speedControlState == 0 ){
                memory.axSpeedControl = currentAccelGain * memory.speedControlVariation;
            }
            else if( memory.speedControlState == 1 ){
                if( refSpeedMode == 4 ){
                    memory.axSpeedControl = param.accelControlGain * (-1.0);
                }
                else{
                    memory.axSpeedControl = param.accelOffDeceleration * (-1.0) * memory.speedControlVariation;
                }
            }
        }
        else if( err_speed + deadZone <= 0.0 ){   // should decelerate

            if( memory.activeBrakeInVelocityControl == true ){
                if( memory.speedControlState == 0 && err_speed + deadZone > (-5.0) / 3.6 ){
                    memory.axSpeedControl = param.accelOffDeceleration * (-1.0);
                }
                else{
                    memory.axSpeedControl = param.accelControlGain * (-1.0);
                }
            }
            else{
                if( memory.speedControlState == 0 ){
                    memory.speedControlVariation = rndGen.GenUniform();
                }
                if( refSpeedMode == 4 ){
                    memory.axSpeedControl = param.accelControlGain * (-1.0);
                }
                else{
                    memory.axSpeedControl = param.accelOffDeceleration * (-1.0) * memory.speedControlVariation;
                }
                memory.speedControlState = 1;
            }


            // Deceleration to change speed within distance of memory.distanceAdjustLowSpeed
            if( memory.distanceAdjustLowSpeed > vHalfLength ){
                float v1 = state.V;
                float v2 = memory.actualTargetSpeed;
                memory.axSpeedControl = (v2 * v2 - v1 * v1) / memory.distanceAdjustLowSpeed * 0.5;

    //            qDebug() << " distanceAdjustLowSpeed = " << memory.distanceAdjustLowSpeed << " axSpeedControl = " <<  memory.axSpeedControl;

                if( memory.axSpeedControl < param.maxDeceleration * (-1.0) ){
                    memory.axSpeedControl = param.maxDeceleration * (-1.0);
                }
            }
        }

//        qDebug() << "[" << ID << "] deadZone = " << deadZone << " speedControlState = " << memory.speedControlState << " axSpeedControl = " << memory.axSpeedControl;
    }
    else{

        float vDevP = param.vDevAllowPlus;
        float vDevM = param.vDevAllowMinus;
        if( param.refVforDev > 1.0 && state.V < param.refVforDev ){
            float f = state.V / param.refVforDev;
            vDevP *= f;
            vDevM *= f;
        }

        if( state.V < memory.actualTargetSpeed - vDevM ){

            if( memory.speedControlState == 1 && state.V >= memory.actualTargetSpeed - vDevM - 2.0 ){
                memory.axSpeedControl = param.accelAtVDev;
            }
            else{
                memory.axSpeedControl = param.accelControlGain;
            }
            memory.speedControlState = 0;
        }
        else if( state.V >= memory.actualTargetSpeed - vDevM &&
                 state.V <= memory.actualTargetSpeed + vDevP ){

            if( memory.speedControlState == 0 ){
                memory.axSpeedControl = param.accelAtVDev;
            }
            else if( memory.speedControlState == 1 ){
                memory.axSpeedControl = param.decelAtVDev * (-1.0);
            }
        }
        else if( state.V > memory.actualTargetSpeed + vDevP ){

            if( memory.activeBrakeInVelocityControl == true ){
                if( memory.speedControlState == 0 && state.V < memory.actualTargetSpeed + vDevP + (5.0) / 3.6 ){
                    memory.axSpeedControl = param.decelAtVDev * (-1.0);
                }
                else{
                    memory.axSpeedControl = param.accelControlGain * (-1.0);
                }
            }
            else{
                memory.axSpeedControl = param.decelAtVDev * (-1.0);
                memory.speedControlState = 1;
            }

            // Deceleration to change speed within distance of memory.distanceAdjustLowSpeed
            if( memory.distanceAdjustLowSpeed > vHalfLength ){
                float v1 = state.V;
                float v2 = memory.actualTargetSpeed;
                memory.axSpeedControl = (v2 * v2 - v1 * v1) / memory.distanceAdjustLowSpeed * 0.5;

    //            qDebug() << " distanceAdjustLowSpeed = " << memory.distanceAdjustLowSpeed << " axSpeedControl = " <<  memory.axSpeedControl;

                if( memory.axSpeedControl < param.maxDeceleration * (-1.0) ){
                    memory.axSpeedControl = param.maxDeceleration * (-1.0);
                }
            }
        }

    }
}


void Agent::HeadwayControlAgent()
{
    float maxDecel = param.maxDeceleration;
    float accelOffDecel = param.accelOffDeceleration;

    float relV = (memory.speedPrecedingVehicle - state.V);

    strForDebug += QString("[HWC]relV=%1 headwayControlDecelState=%2\n").arg(relV).arg(memory.headwayControlDecelState);


//    qDebug() << "ID" << ID
//             << " V = " << state.V
//             << " Vt = " << memory.targetSpeed
//             << " relv = " << relV
//             << " X = " << memory.distanceToPrecedingVehicle
//             << " bstat = " << memory.headwayControlDecelState
//             << " H = " << memory.targetHeadwayDistance;

    // This will happen when merging
    if( memory.precedingVehicleIndex >= 0 ){

        if( fabs( memory.perceptedObjects[memory.precedingVehicleIndex]->deviationFromNearestTargetPath ) < 1.5 &&
            fabs(memory.distanceToPrecedingVehicle) < vHalfLength + memory.halfLenPrecedingVehicle ){
            if( state.V > 3.3 ){
                memory.axHeadwayControl = param.accelControlGain * (-1.0);
                memory.headwayControlDecelState = true;

                strForDebug += QString("[HWC]marge\n");
//                qDebug() << "[1] axHeadwayControl = " << memory.axHeadwayControl;
                return;
            }
        }
        else if( memory.perceptedObjects[memory.precedingVehicleIndex]->deviationFromNearestTargetPath < -2.5 &&
                 memory.distanceToPrecedingVehicle < vHalfLength + memory.halfLenPrecedingVehicle ){

            if( state.V > 3.3 ){
                memory.axHeadwayControl = param.accelControlGain * (-1.0);
                memory.headwayControlDecelState = true;

                strForDebug += QString("[HWC]marge\n");
//                qDebug() << "[2] axHeadwayControl = " << memory.axHeadwayControl;
                return;
            }
        }
    }


    // Should decelerate
    if( state.V < 0.5 && memory.speedPrecedingVehicle < 0.5 && memory.distanceToPrecedingVehicle < memory.targetHeadwayDistance + 5.0 ){
        memory.axHeadwayControl = param.accelControlGain * (-1.0);
        memory.headwayControlDecelState = true;

        strForDebug += QString("[HWC]stop\n");
//        qDebug() << "[3] axHeadwayControl = " << memory.axHeadwayControl;
        return;
    }


    // departing ; speed perception +-5[km/h] = 1.3[m/s]
    if( relV > 1.3f ){

        // Should decelerate
        if( memory.distanceToPrecedingVehicle < memory.actualTargetHeadwayDistance ){  // should brake
            memory.axHeadwayControl = accelOffDecel * (-1.0);
            memory.headwayControlDecelState = true;

            strForDebug += QString("[HWC]inside target distance\n");
//            qDebug() << "[11] axHeadwayControl = " << memory.axHeadwayControl;
            return;
        }

        // Vehoclity Control
        memory.axHeadwayControl = 0.0f;
        memory.headwayControlDecelState = false;

        strForDebug += QString("[HWC]departing\n");
//        qDebug() << "[4] axHeadwayControl = " << memory.axHeadwayControl;
        return;
    }



    float addMargin = 0.0;
    if( relV < 0.0 ){
        addMargin = relV * (-1.0) - param.minimumHeadwayDistanceAtStop;
    }
    if(addMargin < 0.0){
        addMargin = 0.0;
    }

    strForDebug += QString("addMargin=%1\n").arg(addMargin);

    if( memory.precedingObstacle == 1 ){
        addMargin = 10.0;
    }

    if( state.V > 0.1 && state.V < 1.0 ){
        addMargin = 3.0;
    }
    else if( state.V <= 0.1 ){
        addMargin = 0.35;
    }

    // Should decelerate
    if( memory.distanceToPrecedingVehicle < memory.targetHeadwayDistance + addMargin  ){
        if( memory.distanceToPrecedingVehicle < param.minimumHeadwayDistanceAtStop + addMargin &&
                fabs( memory.perceptedObjects[memory.precedingVehicleIndex]->deviationFromNearestTargetPath ) < 3.0 ){

            float ax = (-0.5) * state.V * state.V / (memory.distanceToPrecedingVehicle + 0.1);
            if( ax < maxDecel * (-1.0f) ){
                ax = maxDecel * (-1.0f);
            }
            else if( ax > param.accelControlGain * (-1.0) ){
                ax = param.accelControlGain * (-1.0);
            }

            memory.axHeadwayControl = ax;
            memory.headwayControlDecelState = true;

            strForDebug += QString("[HWC]near\n");
//            qDebug() << "[5] axHeadwayControl = " << memory.axHeadwayControl;
            return;

        }
        else{
            memory.axHeadwayControl = param.accelOffDeceleration * (-1.0);
            memory.headwayControlDecelState = true;

            strForDebug += QString("[HWC]accel off\n");
//            qDebug() << "[6] axHeadwayControl = " << memory.axHeadwayControl;
            return;
        }
    }


    // Should Decelerate
    if( memory.distanceToPrecedingVehicle > param.minimumHeadwayDistanceAtStop + memory.targetHeadwayDistance &&
            memory.distanceToPrecedingVehicle < param.minimumHeadwayDistanceAtStop + memory.targetHeadwayDistance + memory.distanceToZeroSpeed
            && relV < -1.3 ){

        float L = memory.distanceToPrecedingVehicle - memory.targetHeadwayDistance;
        if( L < 0.1 ){
            L = 0.1;
        }

        strForDebug += QString("L=%1\n").arg(L);

        float dv = memory.speedPrecedingVehicle - state.V;
        float ax = (-0.5) * dv * dv / L;
        if( ax < accelOffDecel * (-1.0) ){
            memory.axHeadwayControl = ax;
        }
        else{
            memory.axHeadwayControl = accelOffDecel * (-1.0);
        }

        if( memory.axHeadwayControl < maxDecel * (-1.0f) ){
            memory.axHeadwayControl = maxDecel * (-1.0f);
        }

        memory.headwayControlDecelState = true;

        strForDebug += QString("[HWC]should braking\n");
//        qDebug() << "[7] axHeadwayControl = " << memory.axHeadwayControl << " L = " << L << " dv = " << dv;
        return;
    }


    // Far enough
    if( memory.distanceToPrecedingVehicle > memory.actualTargetHeadwayDistance + 0.5 * state.V ){

        // To avoid chatering
        if( relV < -1.3 && memory.headwayControlDecelState == true ){

            float L = memory.distanceToPrecedingVehicle - param.minimumHeadwayDistanceAtStop - memory.targetHeadwayDistance;
            if( L < 0.1 ){
                L = 0.1;
            }

            float dv = memory.speedPrecedingVehicle - state.V;
            float ax = (-0.5) * dv * dv / L;
            if( ax < accelOffDecel * (-1.0) ){
                memory.axHeadwayControl = ax;
            }
            else{
                memory.axHeadwayControl = accelOffDecel * (-1.0);
            }

            memory.headwayControlDecelState = true;
            strForDebug += QString("[HWC]avoid chatering\n");

//            qDebug() << "[8] axHeadwayControl = " << memory.axHeadwayControl;
            return;
        }

        memory.axHeadwayControl = 0.0;
        memory.headwayControlDecelState = false;
        strForDebug += QString("[HWC]can approach\n");

//        qDebug() << "[9] axHeadwayControl = " << memory.axHeadwayControl;
        return;
    }


    // hysterisys
    if( memory.headwayControlDecelState == false ){
        memory.axHeadwayControl = 0.0;
    }
    else{
        memory.axHeadwayControl = accelOffDecel * (-1.0);
    }

    strForDebug += QString("[HWC]hysterisys\n");
//    qDebug() << "[10] axHeadwayControl = " << memory.axHeadwayControl;

    return;
}


void Agent::HeadwayControl()
{
    float maxDecel = param.maxDeceleration;
//    float accelOffDecel = param.accelOffDeceleration;

    float isFollowing = 1.0;
    if( memory.distanceToPrecedingVehicle < 0.0f ){
        isFollowing = -1.0;
    }

    float HDerr = memory.actualTargetHeadwayDistance - fabs(memory.distanceToPrecedingVehicle);
    float relV = (memory.speedPrecedingVehicle - state.V) * isFollowing;

//    qDebug() << "distanceToPrecedingVehicle = " << memory.distanceToPrecedingVehicle;
//    qDebug() << "HDerr = " << HDerr;
//    qDebug() << "speedPrecedingVehicle = " << memory.speedPrecedingVehicle;
//    qDebug() << "V = " << state.V;
//    qDebug() << "relV = " << relV;

    strForDebug += QString("HDerr=%1 relV=%2\n").arg(HDerr).arg(relV);

    if( relV > 0.5f ){

        if( isFollowing > 0.0f ){
            memory.axHeadwayControl = param.accelControlGain;
        }
        else{
            if( HDerr >= 0.0 ){
                memory.axHeadwayControl = param.accelControlGain * (-1.0f);
            }
            else{
                memory.axHeadwayControl = param.accelControlGain;
            }

//            qDebug() << "axHeadwayControl = " << memory.axHeadwayControl;
//            qDebug() << "axPrecedingVehicle = " << memory.axPrecedingVehicle;

            if( memory.axPrecedingVehicle < memory.axHeadwayControl ){
                memory.axHeadwayControl = memory.axPrecedingVehicle;
            }
        }
        return;
    }

    float addMargin = 0.0;
    if( relV < 0.0 ){
        addMargin = relV * 1.5;
    }

    strForDebug += QString("addMargin=%1\n").arg(addMargin);

    if( fabs(memory.distanceToPrecedingVehicle) + addMargin < param.minimumHeadwayDistanceAtStop ){
        if( isFollowing > 0.0f ){
            memory.axHeadwayControl = maxDecel * (-1.0f);
        }
        else{
            memory.axHeadwayControl = memory.axPrecedingVehicle;
        }
        return;
    }

    float S = HDerr - relV * param.headwayControlGain;
//    qDebug() << "S = " << S;

    strForDebug += QString("S=%1\n").arg(S);

    if( S <= 0.0f ){
        if( isFollowing > 0.0f ){
            memory.axHeadwayControl = param.accelControlGain;
        }
        else{
            memory.axHeadwayControl = memory.axPrecedingVehicle;
        }
        return;
    }
    else if( S > 0.0f ){

        float aReq = (-2.0f) * S / (param.headwayControlGain * param.headwayControlGain);

        //qDebug() << "aReq = " << aReq;

        if( isFollowing > 0.0f ){

            if( memory.speedPrecedingVehicle > 1.0 && memory.axPrecedingVehicle < param.minimumPerceptibleDecelerationOfPreceding * (-1.0) ){
                aReq += memory.axPrecedingVehicle;
            }

            memory.axHeadwayControl = aReq;

            if( memory.axHeadwayControl < maxDecel * (-1.0f) ){
                memory.axHeadwayControl = maxDecel * (-1.0f);
            }
        }
        else{
            aReq *= (-1.0f);
            aReq += memory.axPrecedingVehicle;

            memory.axHeadwayControl = aReq;
        }
        return;
    }
}



void Agent::StopControl()
{
    float maxDecel = param.maxDeceleration;
    float accelOffDecel = param.accelOffDeceleration;

    if( memory.distanceToStopPoint < param.minimumDistanceToStopLine ){
        memory.axStopControl = maxDecel * (-1.0);
        return;
    }

    //qDebug() << "  distanceToZeroSpeed = " << memory.distanceToZeroSpeed;

    if( memory.distanceToStopPoint > memory.distanceToZeroSpeed + param.minimumDistanceToStopLine ){
        memory.axStopControl = 0.0;
        return;
    }

    float Dist = memory.distanceToStopPoint;
    if( Dist < 1.0 ){
        Dist = 1.0;
    }

    float aReq = (-0.5) * state.V * state.V / Dist;
    if( aReq > accelOffDecel * (-1.0) ){
        memory.axStopControl = 0.0;
        return;
    }

    //qDebug() << "  aReq = " << aReq;

    memory.axStopControl = aReq;

    if( memory.axStopControl < maxDecel * (-1.0) ){
        memory.axStopControl = maxDecel * (-1.0);
    }
}


void Agent::SpeedAdjustForCurve(Road *pRoad,int cIdx,float targetSpeed)
{
    float distToLowSpeed = 0.0;
    float speedForCurve  = 0.0;
    bool  foundLowSpeed  = false;

//    strForDebug += QString("SpeedAdjustForCurve: targetSpeed=%1\n").arg(targetSpeed);


    for(int i=cIdx;i>=0;i--){

        int pidx = pRoad->pathId2Index.indexOf( memory.targetPathList[i] );

//        strForDebug += QString("path=%1: meanPathCurvature=%2\n").arg(memory.targetPathList[i]).arg(pRoad->paths[pidx]->meanPathCurvature);

        if( fabs(pRoad->paths[pidx]->meanPathCurvature) < 0.0040 ){  // over R250, not adjust speed

            //
            // To adjust Speed for Speed Change to make Bottle-neck by Scenario
            //
            float checkForLargeSpeedChange = pRoad->paths[pidx]->speed85pt;

            float vMean = 0.7692 * checkForLargeSpeedChange;  // Actual Speed = 65[km/h] when vMean = 50[km/h]
            float vStd  = checkForLargeSpeedChange - vMean;   // Actual Speed is assumed to be about 1-sigma for Normal Distribution

            float vDev = vStd * param.speedVariationFactor;

            checkForLargeSpeedChange += vDev;

            if( targetSpeed - checkForLargeSpeedChange > 8.33  ){

                if( checkForLargeSpeedChange < memory.actualTargetSpeed ){

                    float v2 = checkForLargeSpeedChange;
                    float v1 = state.V;
                    if( v2 < v1 ){
                        float distToThatPoint = distToLowSpeed -  memory.distanceFromStartWPInCurrentPath;
                        if( distToThatPoint < memory.requiredDistToStopFromTargetSpeed ){
                            float aReq = (v2 * v2 - v1 * v1) * 0.5 / distToThatPoint;
                            if( aReq < (-1.0) * param.accelControlGain * 0.7 ){

                                memory.actualTargetSpeed = checkForLargeSpeedChange;
                                memory.distanceAdjustLowSpeed = distToThatPoint;

                                foundLowSpeed = true;
                                break;
                            }
                        }
                    }
                }
            }

            distToLowSpeed += pRoad->paths[pidx]->pathLength;

//            strForDebug += QString("  distToLowSpeed=%1\n").arg(distToLowSpeed);

            continue;
        }

        float tempDistToLowSpeed = distToLowSpeed;

        for(int j=0;j<pRoad->paths[pidx]->curvature.size();++j){

//            strForDebug += QString("  curvature[%1]=%2\n").arg(j).arg(pRoad->paths[pidx]->curvature[j]);

            if( fabs(pRoad->paths[pidx]->curvature[j]) < 0.0040 ){  // over R250, not adjust speed
                distToLowSpeed += pRoad->paths[pidx]->length[j];
//                strForDebug += QString("  [1]distToLowSpeed=%1\n").arg(distToLowSpeed);
                continue;
            }
            if( i == cIdx && pRoad->paths[pidx]->length[j] < memory.distanceFromStartWPInCurrentPath + vHalfLength ){
                distToLowSpeed += pRoad->paths[pidx]->length[j];
//                strForDebug += QString("  [2]distToLowSpeed=%1\n").arg(distToLowSpeed);
                continue;
            }
            float distToThatPoint = distToLowSpeed -  memory.distanceFromStartWPInCurrentPath + pRoad->paths[pidx]->length[j];

//            strForDebug += QString("  distToThatPoint=%1 distanceToZeroSpeed=%2\n").arg(distToThatPoint).arg(memory.distanceToZeroSpeed);

            if( distToThatPoint > memory.distanceToZeroSpeed + 3.0 * state.V ){
                break;
            }

            speedForCurve = sqrt( param.latAccelAtTurn / fabs( (pRoad->paths[pidx]->curvature[j] + pRoad->paths[pidx]->meanPathCurvature) * 0.5) );

            if( speedForCurve < 1.5 ){
                speedForCurve = 1.5;
            }

//            strForDebug += QString("speedForCurve=%1\n").arg(speedForCurve);

            if( speedForCurve < targetSpeed ){
                if( speedForCurve < memory.actualTargetSpeed ){

                    memory.actualTargetSpeed = speedForCurve;

                    float v2 = speedForCurve;
                    float v1 = state.V;
                    float aReq = (v2 * v2 - v1 * v1) * 0.5 / distToThatPoint;

//                    strForDebug += QString("set actualTargetSpeed; aReq=%1\n").arg(aReq);

                    if( aReq < (-1.0) * param.accelOffDeceleration ){

                        distToLowSpeed += pRoad->paths[pidx]->length[j];
                        foundLowSpeed = true;
                        break;
                    }
                }
            }
        }


        if( foundLowSpeed == false ){
            distToLowSpeed = tempDistToLowSpeed;
            distToLowSpeed += pRoad->paths[pidx]->pathLength;
//            strForDebug += QString("  [3]distToLowSpeed=%1\n").arg(distToLowSpeed);
        }
        else{
            distToLowSpeed -= memory.distanceFromStartWPInCurrentPath;
            memory.distanceAdjustLowSpeed = distToLowSpeed;
            break;
        }
    }

//    strForDebug += QString("distanceAdjustLowSpeed=%1\n").arg(memory.distanceAdjustLowSpeed);


//    qDebug() << "actualTargetSpeed = " << memory.actualTargetSpeed
//             << " distanceAdjustLowSpeed = " << memory.distanceAdjustLowSpeed;
}


void Agent::SetTargetSpeedIndividual(float vTarget)
{
    //qDebug() << "[SetTargetSpeedIndividual] ID = " << ID << " vTarget = " << vTarget << " refSpeedMode = " << refSpeedMode;

    if( refSpeedMode >= 1 && refSpeedMode != 4 ){

        memory.targetSpeed = vTarget;

    }
    else if( refSpeedMode == 4 ){

        if( memory.setTargetSpeedByScenarioFlag == true ){
            memory.targetSpeed = memory.targetSpeedByScenario;
        }
        else{
            memory.targetSpeed = vTarget;
        }
    }
    else{

        float vMean = 0.7692 * vTarget;  // Actual Speed = 65[km/h] when vMean = 50[km/h]
        float vStd  = vTarget - vMean;   // Actual Speed is assumed to be about 1-sigma for Normal Distribution

        float vDev = vStd * param.speedVariationFactor;

        //qDebug() << "vMean = " << vMean << " vStd = " << vStd << " vDev = " << vDev;

        memory.targetSpeed = vTarget + vDev;
        if( memory.targetSpeed < 0.0 && vTarget > 0.0 ){
            memory.targetSpeed = vTarget;
        }

        //qDebug() << "targetSpeed = " << memory.targetSpeed;
    }
}
