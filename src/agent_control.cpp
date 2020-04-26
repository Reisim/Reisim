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
        memory.requiredDistToStopFromTargetSpeed = memory.targetSpeed * memory.targetSpeed * 0.5f / param.accelControlGain + vHalfLength;
        if( state.accel > 0.0 ){
            memory.distanceToZeroSpeed += state.V;
            memory.timeToZeroSpeed += 1.0;
            memory.requiredDistToStopFromTargetSpeed += memory.targetSpeed;
        }

        memory.minimumDistanceToStop = state.V * state.V * 0.5f / param.maxDeceleration;


        int cIdx = memory.currentTargetPathIndexInList;

        //
        // Longitudinal Control
        //
        if( memory.controlMode == AGENT_CONTROL_MODE::AGENT_LOGIC ){

            controlCount++;
            if( controlCount >= controlCountMax ){
                controlCount = 0;
            }
            else{
                return;
            }

            memory.accel = 0.0;
            memory.brake = 0.0;

            memory.actualTargetSpeed = memory.targetSpeed;

            memory.distanceAdjustLowSpeed = 0.0;
            SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeed );


            if( memory.doHeadwayDistanceControl == true ){
                if( memory.distanceToPrecedingVehicle < 3.0 * state.V ){
                    if( memory.speedPrecedingVehicle < memory.actualTargetSpeed ){
                        memory.actualTargetSpeed = memory.speedPrecedingVehicle;
                    }
                }
            }

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

                float ax_com = 0.0;

                SpeedAdjustForCurve( pRoad, cIdx, memory.profileSpeed.last() );

                SpeedControl();

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

                        float ax_com = aReq + err * param.accelControlGain;
                        if( ax_com >= 0.0 ){
                            memory.accel = ax_com;
                        }
                        else{
                            memory.brake = -ax_com;
                        }

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

                        SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeedByScenario );

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

                        SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeedByScenario );

                        memory.activeBrakeInVelocityControl = true;
                        SpeedControl();
                        memory.activeBrakeInVelocityControl = false;

                        ax_com = memory.axSpeedControl;

//                        qDebug() << " VMod Vel Con : ax=" << ax_com;
                    }
                }
                else{

                    SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeedByScenario );

                    SpeedControl();

                    ax_com = memory.axSpeedControl;
                }
            }
            else{

                memory.actualTargetSpeed = memory.targetSpeedByScenario;
                memory.distanceAdjustLowSpeed = 0.0;

                SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeedByScenario );

                SpeedControl();

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

            SpeedAdjustForCurve( pRoad, cIdx, memory.targetSpeedByScenario );

            SpeedControl();

            ax_com = memory.axSpeedControl;

            memory.doStopControl = true;
            memory.actualStopAtX = memory.targetStopAtXByScenario;
            memory.actualStopAtY = memory.targetStopAtYByScenario;

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

    //        qDebug() << "distanceToStopPoint = " << memory.distanceToStopPoint
    //                 << " Index=" << memory.actualStopOnPathIndex
    //                 << " Current=" << memory.currentTargetPathIndexInList
    //                 << " S = " << memory.distanceFromStartWPInCurrentPath
    //                 << " L = " << memory.distToStopAtOnThePath;


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

        float Y = memory.lateralDeviationFromTargetPathAtPreviewPoint - memory.lateralShiftTarget;
        memory.steer = (-1.0) * Y * param.steeringControlGain * lowSpeedAdjustGain;

        if( memory.steer > 4.2 ){
            memory.steer = 4.2;
            }
        else if( memory.steer < -4.2 ){
            memory.steer = -4.2;
        }

        // Do not steer at stop
        if( state.V < 0.1 ){
            memory.steer = 0.0;
        }

        if( memory.overrideSteerByScenario == true ){
           memory.steer = memory.overrideSteerControl * 0.014752; // should be multiplied a gain varied depending on speed
        }


        // Winker Operation
        if( memory.controlMode == AGENT_CONTROL_MODE::AGENT_LOGIC ){
            if( memory.distanceToTurnNodeWPIn >= 0 && vehicle.GetWinerState() == 0 ){
                if( memory.distanceToTurnNodeWPIn <= 30.0 || memory.distanceToTurnNodeWPIn < 3.0 * state.V ){
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
    else if( agentKind >= 100 ){


        controlCount++;
        if( controlCount >= controlCountMax ){
            controlCount = 0;
        }
        else{
            return;
        }


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

        float dv = (tV - state.V);
        float dvMax = 2.0 * calInterval;
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

        float  dev = 0.0;
        float    z = 0.0;
        float xdir = 0.0;
        float ydir = 0.0;
        pRoad->GetDeviationFromPedestPath( memory.currentTargetPath, memory.currentTargetPathIndexInList,
                                           state.x, state.y, dev, z, xdir, ydir,
                                           memory.lateralShiftTarget );

//        qDebug() << "currentTargetPathIndexInList = " << memory.currentTargetPathIndexInList << " xdir=" << xdir << " ydir=" << ydir;

        float mxdir = xdir * 0.2 + state.cosYaw * 0.8;
        float mydir = ydir * 0.2 + state.sinYaw * 0.8;

        state.yaw = atan2( mydir, mxdir );
        state.cosYaw = mxdir;
        state.sinYaw = mydir;

        state.z = 0.0;
    }
}




void Agent::SpeedControl()
{
    float err_speed = memory.actualTargetSpeed - state.V;    // [m/s]

//    if( ID == 2 ){
//        qDebug() << "V = " << state.V << " actualTargetSpeed = " << memory.actualTargetSpeed << " err = " << err_speed;
//    }

    float deadZone = param.deadZoneSpeedControl;
    if( isScenarioObject == true ){
        deadZone = 0.0;
    }
    if( err_speed - deadZone >= 0.0 ){
        memory.speedControlState = 0;
        memory.axSpeedControl = param.accelControlGain;
    }
    else if( fabs(err_speed) < deadZone ){
        if( memory.speedControlState == 0 ){
            memory.axSpeedControl = param.accelControlGain;
        }
        else if( memory.speedControlState == 1 ){
            memory.axSpeedControl = param.accelOffDeceleration * (-1.0);
        }
    }
    else if( err_speed + deadZone < 0.0 ){

        if( memory.activeBrakeInVelocityControl == true ){
            if( memory.speedControlState == 0 || err_speed + deadZone > (-5.0) / 3.6 ){
            memory.axSpeedControl = param.accelOffDeceleration * (-1.0);
        }
        else{
            memory.axSpeedControl = param.accelControlGain * (-1.0);
        }
        }
        else{
            memory.axSpeedControl = param.accelOffDeceleration * (-1.0);
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

//    if( ID == 2 ){
//        qDebug() << "speedControlState = " << memory.speedControlState << " axSpeedControl = " << memory.axSpeedControl;
//    }
}


void Agent::HeadwayControlAgent()
{
    float maxDecel = param.maxDeceleration;
    float accelOffDecel = param.accelOffDeceleration;

    float relV = (memory.speedPrecedingVehicle - state.V);
    if( relV > 0.5f ){
        memory.axHeadwayControl = 0.0f;
        return;
    }

    float addMargin = 0.0;
    if( relV < 0.0 ){
        addMargin = relV * 1.0;
    }

    if( memory.precedingObstacle == 1 ){
        addMargin += 10.0;
    }

    if( memory.distanceToPrecedingVehicle - addMargin < param.minimumHeadwayDistanceAtStop ){
        memory.axHeadwayControl = maxDecel * (-1.0f);
        return;
    }


    float distStopPreceding = 0.5 * memory.speedPrecedingVehicle * memory.speedPrecedingVehicle / param.accelControlGain;

    if( distStopPreceding + memory.distanceToPrecedingVehicle - (memory.distanceToZeroSpeed - state.V)  < param.minimumHeadwayDistanceAtStop
            && relV < -1.0 ){

        float L = distStopPreceding + memory.distanceToPrecedingVehicle - param.minimumHeadwayDistanceAtStop;
        if( L < 0.1 ){
            L = 0.1;
        }
        float ax = (-0.5) * state.V * state.V / L;

        memory.axHeadwayControl = ax;

        if( memory.axHeadwayControl < maxDecel * (-1.0f) ){
            memory.axHeadwayControl = maxDecel * (-1.0f);
        }

        return;
    }

    if( memory.actualTargetHeadwayDistance > memory.distanceToPrecedingVehicle ){
        memory.axHeadwayControl = accelOffDecel * (-1.0);
        return;
    }

    memory.axHeadwayControl = 0.0f;
    return;
}


void Agent::HeadwayControl()
{
    float maxDecel = param.maxDeceleration;
    float accelOffDecel = param.accelOffDeceleration;

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


    if( relV > 0.5f ){

        if( isFollowing > 0.0f ){
            memory.axHeadwayControl = 0.0f;
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

    for(int i=cIdx;i>=0;i--){

        int pidx = pRoad->pathId2Index.indexOf( memory.targetPathList[i] );

        for(int j=0;j<pRoad->paths[pidx]->curvature.size();++j){
            if( fabs(pRoad->paths[pidx]->curvature[j]) < 0.0025 ){  // over R400 is assumed as Straight
                continue;
            }
            if( i == cIdx && pRoad->paths[pidx]->length[j] < memory.distanceFromStartWPInCurrentPath + vHalfLength ){
                continue;
            }
            float distToThatPoint = distToLowSpeed -  memory.distanceFromStartWPInCurrentPath + pRoad->paths[pidx]->length[j];
            if( distToThatPoint > memory.distanceToZeroSpeed ){
                break;
            }

            speedForCurve = sqrt( param.latAccelAtTurn / fabs(pRoad->paths[pidx]->curvature[j]) );

            if( speedForCurve < 1.5 ){
                speedForCurve = 1.5;
            }

            if( speedForCurve < targetSpeed ){
                if( speedForCurve < memory.actualTargetSpeed ){

                    memory.actualTargetSpeed = speedForCurve;

                    float v2 = speedForCurve;
                    float v1 = state.V;
                    float aReq = (v2 * v2 - v1 * v1) * 0.5 / distToThatPoint;
                    if( aReq < (-1.0) * param.accelOffDeceleration ){

                        distToLowSpeed += pRoad->paths[pidx]->length[j];
                        foundLowSpeed = true;
                        break;
                    }
                }

            }
        }
        if( foundLowSpeed == false ){
            distToLowSpeed += pRoad->paths[pidx]->pathLength;
        }
        else{
            distToLowSpeed -= memory.distanceFromStartWPInCurrentPath;
            memory.distanceAdjustLowSpeed = distToLowSpeed;
            break;
        }
    }

//    qDebug() << "actualTargetSpeed = " << memory.actualTargetSpeed
//             << " distanceAdjustLowSpeed = " << memory.distanceAdjustLowSpeed;
}


void Agent::SetTargetSpeedIndividual(float vTarget)
{
    float vMean = 0.7692 * vTarget;  // Actual Speed = 65[km/h] when vMean = 50[km/h]
    float vStd  = vTarget - vMean;   // Actual Speed is assumed to be about 1-sigma for Normal Distribution

    float vDev = vStd * param.speedVariationFactor;

    memory.targetSpeed = vTarget + vDev;
}
