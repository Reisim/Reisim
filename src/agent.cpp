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

    memory.aimPointFactorForExternalControl = 1.0;
    memory.steerControlGainForExternalControl = 1.0;

    memory.speedControlState = 0;
    memory.distanceAdjustLowSpeed = 0.0;
    memory.speedProfileCount = 0;
    memory.doSteerControl = true;
    memory.activeBrakeInVelocityControl = false;

    memory.setTargetSpeedByScenarioFlag = false;
    memory.targetSpeedByScenario = 0.0;

    memory.lateralDeviationFromTargetPath = 0.0;
    memory.lateralDeviationFromTargetPathAtPreviewPoint = 0.0;
		
    memory.doHeadwayDistanceControl = false;
    memory.disableSpeedAdjustForCurveByScenario = false;

    memory.doStopControl = false;
    memory.releaseStopCount = 0;

    memory.targetPathList.clear();
    memory.currentTargetPath = -1;
    memory.currentTargetPathIndexInList = -1;

    memory.precedingVehicleID = -1;
    memory.precedingVehicleIDByScenario = -1;
    memory.scenarioPathSelectID = -1;
    memory.precedingVehicleIndex = -1;

    memory.checkSideVehicleForLC = false;
    memory.LCCheckState = 0;
    memory.LCDirection = 0;
    memory.LCInfoGetCount = 0;
    memory.laneChangeTargetPathList.clear();
    memory.laneChangePathLength.clear();
    memory.LCbyEventMode = 0;

    memory.runOncomingLane = false;

    // Default parameters
    param.accelControlGain = 0.25 * 9.81;
    param.deadZoneSpeedControl = 2.5 / 3.6;
    param.maxDeceleration = 0.75 * 9.81;
    param.accelOffDeceleration = 0.06 * 9.81;
    param.steeringControlGain = 2.0;
    param.latAccelAtTurn = 0.20 * 9.81;

    param.headwayTime = rndGen.GetNormalDist(2.0,1.0);
    if( param.headwayTime < 1.0 ){
        param.headwayTime = 1.0;
    }
    else if( param.headwayTime > 3.0 ){
        param.headwayTime = 3.0;
    }

    param.headwayControlGain = 2.0;
    param.minimumPerceptibleDecelerationOfPreceding = 0.3 * 9.81;
    param.minimumHeadwayDistanceAtStop = 4.5;
    param.minimumDistanceToStopLine = 1.5;
    param.visibleDistance = 200.0;
    param.startRelay = 1.0;
    param.crossTimeSafetyMargin = 2.5;
    param.crossWaitPositionSafeyMargin = 7.5;
    param.pedestWaitPositionSafetyMargin = 5.0;
    param.safetyConfirmTime = 0.75;

    param.speedVariationFactor = 0.0;

    param.vDevAllowPlus = 1.388;
    param.vDevAllowMinus = 1.388;
    param.accelAtVDev = 0.05 * 9.81;
    param.decelAtVDev = 0.04 * 9.81;

    param.maxSpeedVehicle = 150.0;   // Max Speed when acceleration becomes 0

    param.LCInfoGetTime = 2.0;
    param.LCCutInAllowTTC = 2.0;
    param.maxLateralSpeedForLaneChange = 1.0;  // 3.0[s] for 3.0[m] shift

    refSpeedMode = 0;
    brakeLampOverride = -1;

    isScenarioObject   = false;
    isOldScenarioType  = false;
    isSInterfaceObject = false;
    isSInterObjDataSet = false;
    isBehaviorEmbeded  = false;
    justWarped = false;
    notAllowedAppear   = false;


    struct TrafficSignalPerception* vts = new struct TrafficSignalPerception;

    vts->isValidData = false;
    vts->objectID    = -1;
    vts->objectType  = 'v';
    vts->x           = 0.0;
    vts->y           = 0.0;
    vts->yaw         = 0.0;
    vts->relatedNode = -1;
    vts->SLonPathID  = -1;
    vts->SLID        = -1;

    memory.perceptedSignals.append( vts );

    objIDForUE4 = -1;
    for(int i=0;i<10;++i){
        UE4ObjectID[i] = -1;
        UE4ObjIDDelCount[i] = 0;
    }

    
#ifdef _PERFORMANCE_CHECK_AGENT
    QueryPerformanceFrequency(&freq);
    for(int i=0;i<10;++i){
        calTime[i] = 0.0;
        calCount[i] = 0;
    }
#endif
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

    memory.speedControlVariation = 1.0;

    refSpeedMode = 0;
    memory.setTargetSpeedByScenarioFlag = false;
    memory.targetSpeedByScenario = 0.0;

    if( memory.profileTime.size() > 0 ){
        memory.profileTime.clear();
    }
    if( memory.profileSpeed.size() > 0 ){
        memory.profileSpeed.clear();
    }
    memory.protectProfileData = false;

    if( memory.targetPathList.size() > 0 ){
        memory.targetPathList.clear();
    }

    memory.ADDisturbFlag = false;
    memory.ADDisturbCount = 0;

    if( memory.ADDisturbTime.size() > 0 ){
        memory.ADDisturbTime.clear();
    }
    if( memory.ADDisturb.size() > 0 ){
        memory.ADDisturb.clear();
    }

    memory.steerDisturbFlag = false;
    memory.steerDisturbCount = 0;
    memory.steerDisturbInit = 0.0;

    if( memory.steerDisturbTime.size() > 0 ){
        memory.steerDisturbTime.clear();
    }
    if( memory.steerDisturb.size() > 0 ){
        memory.steerDisturb.clear();
    }

    memory.currentTargetPath = -1;
    memory.currentTargetPathIndexInList = -1;

    for(int i=0;i<memory.perceptedObjects.size();++i){
        memory.perceptedObjects[i]->isValidData = false;
    }

    for(int i=0;i<memory.perceptedSignals.size();++i){
        memory.perceptedSignals[i]->isValidData = false;
    }

    memory.doHeadwayDistanceControl = false;
    memory.axHeadwayControl = 0.0;

    memory.precedingVehicleID = -1;
    memory.distanceToPrecedingVehicle = 0.0;
    memory.precedingVehicleIDByScenario = -1;
    memory.scenarioPathSelectID = -1;
    memory.precedingVehicleIndex = -1;
    memory.headwayControlDecelState = false;

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

    memory.lateralDeviationFromTargetPath = 0.0;
    memory.lateralDeviationFromTargetPathAtPreviewPoint = 0.0;
    memory.targetLateralShiftByScenario = 0.0;
    memory.relativeAttitudeToLane = 0.0;

    memory.shouldYeild = false;
    memory.leftCrossCheckCount = 0;
    memory.rightCrossCheckCount = 0;

    memory.lateralShiftTarget = 0.0;
    memory.lateralShiftTarget_backup = 0.0;
    memory.distToAvoidTarget = 0.0;

    memory.avoidTarget = -1;

    memory.isChaningLane = false;
    memory.alwaysMoveSide = false;
    memory.requestTemporalDeceleratrion = false;

    memory.checkSideVehicleForLC = false;
    memory.LCCheckState = 0;
    memory.LCDirection = 0;
    memory.LCInfoGetCount = 0;
    memory.laneChangeTargetPathList.clear();
    memory.laneChangePathLength.clear();
    memory.LCbyEventMode = 0;

    memory.currentPathInLCTargetPathList = -1;
    memory.latDeviFromLCTargetPathList = 0.0;
    memory.distFromSWPLCTargetPathList = 0.0;

    memory.runOutChecked = false;
    memory.exeRunOut = false;
    memory.runOutState = 0;

    memory.hazardusObject = -1;
    memory.lastHazardusObject = -1;
    memory.ignoreHazardusObject = -1;

    memory.runOncomingLane = false;


    strForDebug         = QString("");
    strForDebugRiskEval = QString("");

    objIDForUE4 = -1;

    for(int i=0;i<10;++i){
        UE4ObjectID[i] = -1;
        UE4ObjIDDelCount[i] = 0;
    }

    brakeLampOverride = -1;
}


void Agent::BackupMemory()
{
    // control output
//    memory_reference.accel = memory.accel;
//    memory_reference.brake = memory.brake;
//    memory_reference.steer = memory.steer;

//    memory_reference.overrideBrakeByScenario = memory.overrideBrakeByScenario;
//    memory_reference.overrideAxControl       = memory.overrideAxControl;

//    memory_reference.overrideSteerByScenario = memory.overrideSteerByScenario;
//    memory_reference.overrideSteerControl    = memory.overrideSteerControl;

//    memory_reference.additionalShiftByScenarioEvent = memory.additionalShiftByScenarioEvent;
//    memory_reference.additionalLateralShift         = memory.additionalLateralShift;

    // control reference
//    memory_reference.controlMode = memory.controlMode;

    memory_reference.distanceToZeroSpeed               = memory.distanceToZeroSpeed;
//    memory_reference.timeToZeroSpeed                   = memory.timeToZeroSpeed;
    memory_reference.requiredDistToStopFromTargetSpeed = memory.requiredDistToStopFromTargetSpeed;
//    memory_reference.minimumDistanceToStop             = memory.minimumDistanceToStop;

//    memory_reference.actualTargetSpeed                           = memory.actualTargetSpeed;
//    memory_reference.actualTargetHeadwayDistance                 = memory.actualTargetHeadwayDistance;
//    memory_reference.targetSpeed                                 = memory.targetSpeed;
//    memory_reference.targetHeadwayDistance                       = memory.targetHeadwayDistance;
//    memory_reference.targetSpeedByScenario                       = memory.targetSpeedByScenario;
//    memory_reference.actualTargetHeadwayDistanceByScenario       = memory.actualTargetHeadwayDistanceByScenario;
//    memory_reference.targetHeadwayDistanceByScenario             = memory.targetHeadwayDistanceByScenario;
//    memory_reference.allowableHeadwayDistDeviation               = memory.allowableHeadwayDistDeviation;
//    memory_reference.targetHeadwayTimeByScenario                 = memory.targetHeadwayTimeByScenario;
//    memory_reference.targetSpeedInsideIntersectionTurnByScenario = memory.targetSpeedInsideIntersectionTurnByScenario;
//    memory_reference.startDecel                                  = memory.startDecel;
//    memory_reference.activeBrakeInVelocityControl                = memory.activeBrakeInVelocityControl;

//    memory_reference.actualStopAtX           = memory.actualStopAtX;
//    memory_reference.actualStopAtY           = memory.actualStopAtY;
//    memory_reference.targetStopAtX           = memory.targetStopAtX;
//    memory_reference.targetStopAtY           = memory.targetStopAtY;
//    memory_reference.targetStopAtXByScenario = memory.targetStopAtXByScenario;
//    memory_reference.targetStopAtYByScenario = memory.targetStopAtYByScenario;
//    memory_reference.actualStopOnPathID      = memory.actualStopOnPathID;
//    memory_reference.actualStopOnPathIndex   = memory.actualStopOnPathIndex;
//    memory_reference.distToStopAtOnThePath   = memory.distToStopAtOnThePath;
    memory_reference.distanceToStopPoint     = memory.distanceToStopPoint;

//    memory_reference.speedControlState      = memory.speedControlState;
//    memory_reference.distanceAdjustLowSpeed = memory.distanceAdjustLowSpeed;
//    memory_reference.axSpeedControl         = memory.axSpeedControl;

//    memory_reference.speedProfileCount = memory.speedProfileCount;

//    memory_reference.profileTime.clear();
//    for(int i=0;i<memory.profileTime.size();++i){
//        memory_reference.profileTime.append( memory.profileTime.at(i) );
//    }

//    memory_reference.profileSpeed.clear();
//    for(int i=0;i<memory.profileSpeed.size();++i){
//        memory_reference.profileSpeed.append( memory.profileSpeed.at(i) );
//    }

//    memory_reference.doHeadwayDistanceControl = memory.doHeadwayDistanceControl;
//    memory_reference.axHeadwayControl         = memory.axHeadwayControl;

//    memory_reference.doStopControl      = memory.doStopControl;
//    memory_reference.causeOfStopControl = memory.causeOfStopControl;
//    memory_reference.axStopControl      = memory.axStopControl;
//    memory_reference.releaseStopCount   = memory.releaseStopCount;

//    memory_reference.doSteerControl                               = memory.doSteerControl;
//    memory_reference.lateralDeviationFromTargetPathAtPreviewPoint = memory.lateralDeviationFromTargetPathAtPreviewPoint;
//    memory_reference.previewPointPath                             = memory.previewPointPath;
    memory_reference.lateralDeviationFromTargetPath               = memory.lateralDeviationFromTargetPath;
//    memory_reference.steeringControlGainMultiplier                = memory.steeringControlGainMultiplier;
//    memory_reference.lateralShiftTarget                           = memory.lateralShiftTarget;
//    memory_reference.avoidTarget                                  = memory.avoidTarget;

//    memory_reference.isChaningLane = memory.isChaningLane;


    // hazard and risk valuation
    memory_reference.precedingVehicleID           = memory.precedingVehicleID;
//    memory_reference.precedingVehicleIDByScenario = memory.precedingVehicleIDByScenario;
//    memory_reference.distanceToPrecedingVehicle   = memory.distanceToPrecedingVehicle;
//    memory_reference.speedPrecedingVehicle        = memory.speedPrecedingVehicle;
//    memory_reference.axPrecedingVehicle           = memory.axPrecedingVehicle;
//    memory_reference.precedingObstacle            = memory.precedingObstacle;

//    memory_reference.targetLateralShift           = memory.targetLateralShift;
//    memory_reference.targetLateralShiftByScenario = memory.targetLateralShiftByScenario;

//    memory_reference.distToNearOncomingCP     = memory.distToNearOncomingCP;
//    memory_reference.distToFatOncomingCP      = memory.distToFatOncomingCP;
//    memory_reference.shouldWaitOverCrossPoint = memory.shouldWaitOverCrossPoint;

//    memory_reference.distToNearestCP      = memory.distToNearestCP;
//    memory_reference.shouldStopAtSignalSL = memory.shouldStopAtSignalSL;

//    memory_reference.distToYeildStopLine  = memory.distToYeildStopLine;
//    memory_reference.shouldYeild          = memory.shouldYeild;
//    memory_reference.leftCrossIsClear     = memory.leftCrossIsClear;
//    memory_reference.rightCrossIsClear    = memory.rightCrossIsClear;
//    memory_reference.leftCrossCheckCount  = memory.leftCrossCheckCount;
//    memory_reference.rightCrossCheckCount = memory.rightCrossCheckCount;
//    memory_reference.safetyConfimed       = memory.safetyConfimed;
//    memory_reference.speedZeroCount       = memory.speedZeroCount;


    // perception & recognition
//    int diffN = memory.perceptedObjects.size() - memory_reference.perceptedObjects.size();
//    if( diffN > 0 ){
//        for(int i=0;i<diffN;++i){
//            struct AgentPerception *ap = new struct AgentPerception;
//            memory_reference.perceptedObjects.append( ap );
//        }
//    }
//    for(int i=0;i<memory.perceptedObjects.size();++i){

//        memory_reference.perceptedObjects[i]->objectID    = memory.perceptedObjects[i]->objectID;
//        memory_reference.perceptedObjects[i]->objectType  = memory.perceptedObjects[i]->objectType;

//        memory_reference.perceptedObjects[i]->vHalfLength = memory.perceptedObjects[i]->vHalfLength;
//        memory_reference.perceptedObjects[i]->vHalfWidth  = memory.perceptedObjects[i]->vHalfWidth;

//        memory_reference.perceptedObjects[i]->x           = memory.perceptedObjects[i]->x;
//        memory_reference.perceptedObjects[i]->y           = memory.perceptedObjects[i]->y;
//        memory_reference.perceptedObjects[i]->yaw         = memory.perceptedObjects[i]->yaw;
//        memory_reference.perceptedObjects[i]->cos_yaw     = memory.perceptedObjects[i]->cos_yaw;
//        memory_reference.perceptedObjects[i]->sin_yaw     = memory.perceptedObjects[i]->sin_yaw;
//        memory_reference.perceptedObjects[i]->V           = memory.perceptedObjects[i]->V;
//        memory_reference.perceptedObjects[i]->Ax          = memory.perceptedObjects[i]->Ax;

//        memory_reference.perceptedObjects[i]->objectPath              = memory.perceptedObjects[i]->objectPath;
//        memory_reference.perceptedObjects[i]->objectTargetNode        = memory.perceptedObjects[i]->objectTargetNode;
//        memory_reference.perceptedObjects[i]->deviationFromObjectPath = memory.perceptedObjects[i]->deviationFromObjectPath;

//        memory_reference.perceptedObjects[i]->nearestTargetPath                = memory.perceptedObjects[i]->nearestTargetPath;
//        memory_reference.perceptedObjects[i]->deviationFromNearestTargetPath   = memory.perceptedObjects[i]->deviationFromNearestTargetPath;
//        memory_reference.perceptedObjects[i]->distanceToObject                 = memory.perceptedObjects[i]->distanceToObject;
//        memory_reference.perceptedObjects[i]->xOnTargetPath                    = memory.perceptedObjects[i]->xOnTargetPath;
//        memory_reference.perceptedObjects[i]->yOnTargetPath                    = memory.perceptedObjects[i]->yOnTargetPath;
//        memory_reference.perceptedObjects[i]->innerProductToNearestPathTangent = memory.perceptedObjects[i]->innerProductToNearestPathTangent;
//        memory_reference.perceptedObjects[i]->innerProductToNearestPathNormal  = memory.perceptedObjects[i]->innerProductToNearestPathNormal;
//        memory_reference.perceptedObjects[i]->effectiveHalfWidth               = memory.perceptedObjects[i]->effectiveHalfWidth;

//        memory_reference.perceptedObjects[i]->recognitionLabel = memory.perceptedObjects[i]->recognitionLabel;

//        memory_reference.perceptedObjects[i]->hasCollisionPoint  = memory.perceptedObjects[i]->hasCollisionPoint;
//        memory_reference.perceptedObjects[i]->xCP                = memory.perceptedObjects[i]->xCP;
//        memory_reference.perceptedObjects[i]->yCP                = memory.perceptedObjects[i]->yCP;
//        memory_reference.perceptedObjects[i]->myDistanceToCP     = memory.perceptedObjects[i]->myDistanceToCP;
//        memory_reference.perceptedObjects[i]->myTimeToCP         = memory.perceptedObjects[i]->myTimeToCP;
//        memory_reference.perceptedObjects[i]->objectDistanceToCP = memory.perceptedObjects[i]->objectDistanceToCP;
//        memory_reference.perceptedObjects[i]->objectTimeToCP     = memory.perceptedObjects[i]->objectTimeToCP;
//        memory_reference.perceptedObjects[i]->CPinNode           = memory.perceptedObjects[i]->CPinNode;

//        memory_reference.perceptedObjects[i]->myCPPathIndex      = memory.perceptedObjects[i]->myCPPathIndex;
//        memory_reference.perceptedObjects[i]->objCPPathIndex     = memory.perceptedObjects[i]->objCPPathIndex;
//        memory_reference.perceptedObjects[i]->objPathCPChecked   = memory.perceptedObjects[i]->objPathCPChecked;

//        memory_reference.perceptedObjects[i]->shouldEvalRisk = memory.perceptedObjects[i]->shouldEvalRisk;

//        memory_reference.perceptedObjects[i]->inView        = memory.perceptedObjects[i]->inView;
//        memory_reference.perceptedObjects[i]->noUpdateCount = memory.perceptedObjects[i]->noUpdateCount;
//        memory_reference.perceptedObjects[i]->isValidData   = memory.perceptedObjects[i]->isValidData;
//    }

//    diffN = memory.perceptedSignals.size() - memory_reference.perceptedSignals.size();
//    if( diffN > 0 ){
//        for(int i=0;i<diffN;++i){
//            struct TrafficSignalPerception *tsp = new struct TrafficSignalPerception;
//            memory_reference.perceptedSignals.append( tsp );
//        }
//    }
//    for(int i=0;i<memory.perceptedSignals.size();++i){

//        memory_reference.perceptedSignals[i]->objectID    = memory.perceptedSignals[i]->objectID;
//        memory_reference.perceptedSignals[i]->objectType  = memory.perceptedSignals[i]->objectType;

//        memory_reference.perceptedSignals[i]->x           = memory.perceptedSignals[i]->x;
//        memory_reference.perceptedSignals[i]->y           = memory.perceptedSignals[i]->y;
//        memory_reference.perceptedSignals[i]->yaw         = memory.perceptedSignals[i]->yaw;
//        memory_reference.perceptedSignals[i]->relatedNode = memory.perceptedSignals[i]->relatedNode;

//        memory_reference.perceptedSignals[i]->signalDisplay  = memory.perceptedSignals[i]->signalDisplay;

//        memory_reference.perceptedSignals[i]->stopLineX      = memory.perceptedSignals[i]->stopLineX;
//        memory_reference.perceptedSignals[i]->stopLineY      = memory.perceptedSignals[i]->stopLineY;
//        memory_reference.perceptedSignals[i]->distToSL       = memory.perceptedSignals[i]->distToSL;
//        memory_reference.perceptedSignals[i]->SLonPathID     = memory.perceptedSignals[i]->SLonPathID;
//        memory_reference.perceptedSignals[i]->stopPointIndex = memory.perceptedSignals[i]->stopPointIndex;

//        memory_reference.perceptedSignals[i]->inView        = memory.perceptedSignals[i]->inView;
//        memory_reference.perceptedSignals[i]->noUpdateCount = memory.perceptedSignals[i]->noUpdateCount;
//        memory_reference.perceptedSignals[i]->isValidData   = memory.perceptedSignals[i]->isValidData;
//    }


    // guidance
    memory_reference.targetPathList.clear();
    for(int i=0;i<memory.targetPathList.size();++i){
        memory_reference.targetPathList.append( memory.targetPathList.at(i) );
    }

//    memory_reference.targetPathLength.clear();
//    for(int i=0;i<memory.targetPathLength.size();++i){
//        memory_reference.targetPathLength.append( memory.targetPathLength.at(i) );
//    }


    memory_reference.currentTargetPath                = memory.currentTargetPath;
    memory_reference.currentTargetPathIndexInList     = memory.currentTargetPathIndexInList;
    memory_reference.distanceFromStartWPInCurrentPath = memory.distanceFromStartWPInCurrentPath;

//    memory_reference.scenarioPathSelectID    = memory.scenarioPathSelectID;
//    memory_reference.distanceToTurnNodeWPIn  = memory.distanceToTurnNodeWPIn;
    memory_reference.distanceToNodeWPIn      = memory.distanceToNodeWPIn;
//    memory_reference.distanceToTurnNodeWPOut = memory.distanceToTurnNodeWPOut;
//    memory_reference.distanceToNodeWPOut     = memory.distanceToNodeWPOut;

    memory_reference.myNodeList.clear();
    for(int i=0;i<memory.myNodeList.size();++i){
        memory_reference.myNodeList.append( memory.myNodeList.at(i) );
    }

    memory_reference.myInDirList.clear();
    for(int i=0;i<memory.myInDirList.size();++i){
        memory_reference.myInDirList.append(memory.myInDirList.at(i) );
    }

    memory_reference.myOutDirList.clear();
    for(int i=0;i<memory.myOutDirList.size();++i){
        memory_reference.myOutDirList.append( memory.myOutDirList.at(i) );
    }

//    memory_reference.myTurnDirectionList.clear();
//    for(int i=0;i<memory.myTurnDirectionList.size();++i){
//        memory_reference.myTurnDirectionList.append( memory.myTurnDirectionList.at(i) );
//    }

    memory_reference.currentTargetNode                = memory.currentTargetNode;
    memory_reference.currentTargetNodeIndexInNodeList = memory.currentTargetNodeIndexInNodeList;
//    memory_reference.nextTurnDirection                = memory.nextTurnDirection;
    memory_reference.nextTurnNode                     = memory.nextTurnNode;
//    memory_reference.nextTurnNodeIndexInNodeList      = memory.nextTurnNodeIndexInNodeList;

//    memory_reference.oncomingWaitPathList.clear();
//    for(int i=0;i<memory.oncomingWaitPathList.size();++i){
//        memory_reference.oncomingWaitPathList.append( memory.oncomingWaitPathList.at(i) );
//    }

//    memory_reference.oncomingWaitCPList.clear();
//    for(int i=0;i<memory.oncomingWaitCPList.size();++i){
//        memory_reference.oncomingWaitCPList.append( memory.oncomingWaitCPList.at(i) );
//    }

//    memory_reference.nextTurnNodeOncomingDir  = memory.nextTurnNodeOncomingDir;
//    memory_reference.nearOncomingWaitPathInfo = memory.nearOncomingWaitPathInfo;
//    memory_reference.nearOncomingWaitCPInfo   = memory.nearOncomingWaitCPInfo;
//    memory_reference.farOncomingWaitPathInfo  = memory.farOncomingWaitPathInfo;
//    memory_reference.farOncomingWaitCPInfo    = memory.farOncomingWaitCPInfo;



    // navigation
//    memory_reference.routeType      = memory.routeType;
//    memory_reference.routeIndex     = memory.routeIndex;
//    memory_reference.routeLaneIndex = memory.routeLaneIndex;
}
