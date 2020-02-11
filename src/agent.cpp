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


Agent::Agent()
{
    agentStatus = 0;

    memset( &attri, 0, sizeof(attri) );
    memset( &param, 0, sizeof(param) );
    memset( &state, 0, sizeof(state) );

    vHalfWidth  = 0.8;
    vHalfLength = 3.0;

    skipSetControlInfo = false;

    memory.accel = 0.0;
    memory.brake = 0.0;
    memory.steer = 0.0;
    memory.controlMode = AGENT_CONTROL_MODE::AGENT_LOGIC;
    memory.startDecel = 0;
    memory.allowableHeadwayDistDeviation = 5.0;

    memory.speedControlState = 0;
    memory.distanceAdjustLowSpeed = 0.0;
    memory.speedProfileCount = 0;
    memory.doSteerControl = true;
    memory.activeBrakeInVelocityControl = false;
		
    memory.doHeadwayDistanceControl = false;

    memory.doStopControl = false;
    memory.releaseStopCount = 0;

    memory.targetPathList.clear();
    memory.targetPathListBackup.clear();
    memory.currentTargetPath = -1;
    memory.currentTargetPathIndexInList = -1;

    memory.precedingVehicleID = -1;
    memory.precedingVehicleIDByScenario = -1;
    memory.scenarioPathSelectID = -1;


    // Default parameters
    param.accelControlGain = 0.25 * 9.81;
    param.deadZoneSpeedControl = 5.0 / 3.6;
    param.maxDeceleration = 0.6 * 9.81;
    param.accelOffDeceleration = 0.06 * 9.81;
    param.steeringControlGain = 1.5;
    param.latAccelAtTurn = 0.32 * 9.81;
    param.headwayTime = 1.5;
    param.headwayControlGain = 1.2;
    param.minimumPerceptibleDecelerationOfPreceding = 0.3 * 9.81;
    param.minimumHeadwayDistanceAtStop = 2.5;
    param.minimumDistanceToStopLine = 0.0;
    param.visibleDistance = 400.0;
    param.startRelay = 1.0;
    param.crossTimeSafetyMargin = 4.0;
    param.crossWaitPositionSafeyMargin = 7.5;
    param.safetyConfirmTime = 1.5;


    isScenarioObject   = false;
    isSInterfaceObject = false;
    isBehaviorEmbeded  = false;
    justWarped = false;
    notAllowedAppear   = false;
}


void Agent::InitializeMemory()
{
    memory.accel = 0.0;
    memory.brake = 0.0;
    memory.steer = 0.0;
    memory.controlMode = AGENT_CONTROL_MODE::AGENT_LOGIC;

    memory.speedControlState = 0;
    memory.distanceAdjustLowSpeed = 0.0;
    memory.speedProfileCount = 0;

    if( memory.profileTime.size() > 0 ){
        memory.profileTime.clear();
    }
    if( memory.profileSpeed.size() > 0 ){
        memory.profileSpeed.clear();
    }

    if( memory.targetPathList.size() > 0 ){
        memory.targetPathList.clear();
    }
    if( memory.targetPathListBackup.size() > 0 ){
        memory.targetPathListBackup.clear();
    }
    memory.currentTargetPath = -1;
    memory.currentTargetPathIndexInList = -1;

    if( memory.perceptedObjects.size() > 0 ){
        for(int i=0;i<memory.perceptedObjects.size();++i){
            delete memory.perceptedObjects[i];
        }
        memory.perceptedObjects.clear();
    }

    if( memory.perceptedSignals.size() > 0 ){
        for(int i=0;i<memory.perceptedSignals.size();++i){
            delete memory.perceptedSignals[i];
        }
        memory.perceptedSignals.clear();
    }

    memory.doHeadwayDistanceControl = false;
    memory.axHeadwayControl = 0.0;

    memory.releaseStopCount = 0;
    memory.doStopControl = false;
    memory.distanceToStopPoint = 0.0;
    memory.requiredDistToStopFromTargetSpeed = 0.0;
    memory.axStopControl = 0.0;

    memory.shouldStopAtSignalSL = false;

    memory.overrideBrakeByScenario = false;
    memory.overrideAxControl = 0.0;

    memory.overrideSteerByScenario = false;
    memory.overrideSteerControl = 0.0;

    memory.additionalShiftByScenarioEvent = false;
    memory.additionalLateralShift = 0.0;

    memory.steeringControlGainMultiplier = 1.0;
    memory.speedZeroCount = 0;

    memory.leftCrossCheckCount = 0;
    memory.rightCrossCheckCount = 0;

    memory.lateralShiftTarget = 0.0;
    memory.avoidTarget = -1;

    memory.isChaningLane = false;
}

