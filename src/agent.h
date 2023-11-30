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


#ifndef AGENT_H
#define AGENT_H

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <QList>

#include "vehicle.h"
#include "road.h"
#include "trafficsignal.h"

#include "randomgenerator.h"

#ifdef _PERFORMANCE_CHECK_AGENT
#include <windows.h>
#endif


enum AGENT_CONTROL_MODE
{
    AGENT_LOGIC,
    CONSTANT_SPEED_HEADWAY,
    CONSTANT_SPEED_HEADWAY_KEEP_INITIAL_LATERAL_OFFSET,
    USE_TIME_SERIES_DATA,
    SPEED_PROFILE,
    STOP_AT,
    INTERSECTION_TURN_CONTROL,
    SET_STEER_CONTROL_FLAG,
    RUN_OUT,
    DIRECT_ACCEL_ASSIGN
};

enum AGENT_ROUTE_TYPE
{
    NODE_LIST,
    PATH_LIST
};

enum AGENT_RECOGNITION_LABEL
{
    PRECEDING,
    FOLLOWING,
    LEFT_SIDE,
    RIGHT_SIDE,
    LEFT_SIDE_PRECEDING,
    RIGHT_SIDE_PRECEDING,
    LEFT_SIDE_FOLLOWING,
    RIGHT_SIDE_FOLLOWING,
    ONCOMING_STRAIGHT,
    ONCOMING_LEFT,
    ONCOMING_RIGHT,
    LEFT_CROSSING_STRAIGHT,
    LEFT_CROSSING_LEFT,
    LEFT_CROSSING_RIGHT,
    RIGHT_CROSSING_STRAIGHT,
    RIGHT_CROSSING_LEFT,
    RIGHT_CROSSING_RIGHT,
    UNDEFINED_RECOGNITION_LABEL,
    PEDESTRIAN
};

struct AgentPerception
{
    int objectID;
    int objectType;

    float vHalfLength;
    float vHalfWidth;

    float x;
    float y;
    float yaw;
    float cos_yaw;
    float sin_yaw;
    float V;
    float Ax;

    float pastV[10];
    float filteredV;

    int objectPath;
    int objectTargetNode;
    float deviationFromObjectPath;

    bool relPosEvaled;
    int nearestTargetPath;
    float objDistFromSWPOfNearTargetPath;
    float deviationFromNearestTargetPath;
    float distanceToObject;
    float xOnTargetPath;
    float yOnTargetPath;
    float innerProductToNearestPathTangent;
    float innerProductToNearestPathNormal;
    float effectiveHalfWidth;

    int recognitionLabel;
    int objPathRecogLabelChecked;
    int myPathRecogLabelChecked;
    int winker;

    bool hasCollisionPoint;
    bool mergingAsCP;
    float xCP;
    float yCP;
    float myDistanceToCP;
    float myTimeToCP;
    float objectDistanceToCP;
    float objectTimeToCP;
    int CPinNode;

    int myCPPathIndex;
    int objCPPathIndex;
    int objPathCPChecked;

    float distToObjInLaneChangeTargetPathList;
    float latDevObjInLaneChangeTargetPathList;
    int objPathInLCTargetPathList;

    bool shouldEvalRisk;

    bool inView;
    int noUpdateCount;

    bool isValidData;
};

struct TrafficSignalPerception
{
    int objectID;
    char objectType;

    float x;
    float y;
    float yaw;
    int relatedNode;

    int signalDisplay;

    int SLID;
    float stopLineX;
    float stopLineY;
    float distToSL;
    int SLonPathID;
    int stopPointIndex;

    bool inView;
    int noUpdateCount;

    bool isValidData;
};

struct AgentMemory
{
    int flag;

    // control output
    float accel;
    float brake;
    float steer;

    bool overrideBrakeByScenario;
    float overrideAxControl;

    bool overrideSteerByScenario;
    float overrideSteerControl;

    bool additionalShiftByScenarioEvent;
    float additionalLateralShift;

    // control reference
    int controlMode;

    float distanceToZeroSpeed;
    float distanceToZeroSpeedByMaxBrake;
    float timeToZeroSpeed;
    float requiredDistToStopFromTargetSpeed;
    float minimumDistanceToStop;

    float aimPointFactorForExternalControl;
    float steerControlGainForExternalControl;

    float actualTargetSpeed;
    float actualTargetHeadwayDistance;
    float targetSpeed;
    float targetHeadwayDistance;
    bool setTargetSpeedByScenarioFlag;
    float targetSpeedByScenario;
    float actualTargetHeadwayDistanceByScenario;
    float targetHeadwayDistanceByScenario;
    bool disableSpeedAdjustForCurveByScenario;
    float allowableHeadwayDistDeviation;
    float targetHeadwayTimeByScenario;
    float targetSpeedInsideIntersectionTurnByScenario;
    int startDecel;
    bool activeBrakeInVelocityControl;
    bool headwayControlDecelState;

    float actualStopAtX;
    float actualStopAtY;
    float targetStopAtX;
    float targetStopAtY;
    float targetStopAtXByScenario;
    float targetStopAtYByScenario;
    int actualStopOnPathID;
    int actualStopOnPathIndex;
    float distToStopAtOnThePath;
    float distanceToStopPoint;

    int speedControlState;
    float distanceAdjustLowSpeed;
    float axSpeedControl;
    float speedControlVariation;

    int speedProfileCount;
    QList<float> profileTime;
    QList<float> profileSpeed;
    bool protectProfileData;

    bool ADDisturbFlag;
    int ADDisturbCount;
    QList<float> ADDisturbTime;
    QList<float> ADDisturb;

    bool steerDisturbFlag;
    int steerDisturbCount;
    QList<float> steerDisturbTime;
    QList<float> steerDisturb;
    float steerDisturbInit;

    bool doHeadwayDistanceControl;
    float axHeadwayControl;

    bool doStopControl;
    QString causeOfStopControl;
    float axStopControl;
    int releaseStopCount;

    bool doSteerControl;
    float lateralDeviationFromTargetPathAtPreviewPoint;
    int previewPointPath;
    float lateralDeviationFromTargetPath;
    float steeringControlGainMultiplier;
    float lateralShiftTarget;
    float lateralShiftTarget_backup;
    float distToAvoidTarget;
    int avoidTarget;
    int hazardusObject;
    int lastHazardusObject;
    int ignoreHazardusObject;
    QList<int> ignoreCollisonAvoidanceCheckObject;

    float relativeAttitudeToLane;

    bool isChaningLane;
    bool alwaysMoveSide;
    bool requestTemporalDeceleratrion;


    // hazard and risk valuation
    int precedingVehicleID;
    int precedingVehicleIDByScenario;
    float distanceToPrecedingVehicle;
    float speedPrecedingVehicle;
    float axPrecedingVehicle;
    float halfLenPrecedingVehicle;
    int precedingVehicleIndex;
    int precedingObstacle;

    float targetLateralShift;
    float targetLateralShiftByScenario;

    float distToNearOncomingCP;
    float distToFatOncomingCP;
    bool shouldWaitOverCrossPoint;

    float distToNearestCP;
    int nearCPInNode;
    bool shouldStopAtSignalSL;

    float distToYeildStopLine;
    bool shouldYeild;
    bool leftCrossIsClear;
    bool rightCrossIsClear;
    int leftCrossCheckCount;
    int rightCrossCheckCount;
    bool safetyConfimed;
    int speedZeroCount;


    // perception & recognition
    QList<struct AgentPerception *> perceptedObjects;
    QList<struct TrafficSignalPerception *> perceptedSignals;



    // guidance
    QList<int> targetPathList;
    QList<float> targetPathLength;
    QList<QPoint> laneMerge;

    int currentTargetPath;
    int currentTargetPathIndexInList;
    float distanceFromStartWPInCurrentPath;

    int scenarioPathSelectID;
    float distanceToTurnNodeWPIn;
    float distanceToNodeWPIn;
    float distanceToTurnNodeWPOut;
    float distanceToNodeWPOut;
    float distanceToCurrentTargetNodeWPIn;
    float distanceToCurrentTargetNodeWPOut;

    int turnNodeOutWP;

    QPoint currentWPInPos;
    QPoint currentWPOutPos;

    QList<int> myNodeList;
    QList<int> myInDirList;
    QList<int> myOutDirList;
    QList<int> myTurnDirectionList;
    int currentTargetNode;
    int currentTargetNodeIndexInNodeList;
    int currentTargetNodeInDirect;
    int currentTargetNodeOutDirect;
    int currentTargetNodeOncomingDirect;
    int nextTurnDirection;
    int nextTurnNode;
    int nextTurnNodeIndexInNodeList;
    int isMergeNode;
    QList<int> oncomingWaitPathList;
    QList<int> oncomingWaitCPList;
    int nextTurnNodeOncomingDir;
    int nearOncomingWaitPathInfo;
    int nearOncomingWaitCPInfo;
    int farOncomingWaitPathInfo;
    int farOncomingWaitCPInfo;



    // Lane-Change
    int LCDirection;
    bool checkSideVehicleForLC;
    int LCCheckState;
    int LCInfoGetCount;
    bool sideVehicleRiskClear;
    float LCSteerMax;
    int LCbyEventMode;

    QList<int> laneChangeTargetPathList;
    QList<float> laneChangePathLength;

    int currentPathInLCTargetPathList;
    float latDeviFromLCTargetPathList;
    float distFromSWPLCTargetPathList;


    // Run-Out of Pedestrian
    bool runOutChecked;
    bool exeRunOut;
    int runOutState;
    float runOutPosInPedestPath;
    float runOutDir;
    float cosRunOutDir;
    float sinRunOutDir;
    float runOutStartX;
    float runOutStartY;


    // navigation
    int routeType;
    int routeIndex;
    int routeLaneIndex;
    int LCSupportRouteLaneIndex;
    int LCStartRouteIndex;
    int destinationNode;


    // Abnormal Driving
    bool runOncomingLane;
};

struct AgentParam
{
    float accelControlGain;
    float maxSpeedVehicle;
    float deadZoneSpeedControl;
    float maxDeceleration;
    float accelOffDeceleration;
    float steeringControlGain;
    float latAccelAtTurn;
    float headwayTime;
    float headwayControlGain;
    float minimumPerceptibleDecelerationOfPreceding;
    float minimumHeadwayDistanceAtStop;
    float minimumDistanceToStopLine;
    float visibleDistance;
    float startRelay;
    float crossTimeSafetyMargin;
    float crossWaitPositionSafeyMargin;
    float pedestWaitPositionSafetyMargin;
    float safetyConfirmTime;
    float speedVariationFactor;
    float LCInfoGetTime;
    float LCCutInAllowTTC;
    float vDevAllowPlus;
    float vDevAllowMinus;
    float accelAtVDev;
    float decelAtVDev;
    float refVforDev;
    float maxLateralSpeedForLaneChange;
};

struct AgentState
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    float V;
    float cosYaw;
    float sinYaw;
    float accel;
    float brake;
    float steer;

    float accel_log;
    float brake_log;
    float steer_log;
    int  warpFlag;

    float z_path;
};

struct AgentAttiribute
{
    int age;
    int gender;
};


struct SInterObjInfo
{
    int objID;
    int objType;
    float lf;
    float lr;
    float tf;
    float tr;
    bool brakeLamp;
    bool winkerLeft;
    bool winkerRight;
    bool lowbeam;
    bool highbeam;
};


class Agent
{
public:
    Agent();

    float calInterval;
    void InitializeMemory();
    void BackupMemory();

    void Perception( Agent**, int, Road*, QList<TrafficSignal*> trafficSignal );
    void Recognition( Agent**, int, Road* ) ;
    void HazardIdentification( Agent**, int,Road* );
    void RiskEvaluation( Agent**, int, Road*, QList<TrafficSignal*> trafficSignal );
    void Control(Road*);
    void UpdateState(Road*);


    void SpeedControl();
    void SpeedAdjustForCurve(Road*,int,float);
    void HeadwayControl();
    void HeadwayControlAgent();
    void StopControl();
    void SetTargetSpeedIndividual(float vTarget);


    void CheckPathList(Road*);
    void SetTargetNodeListByTargetPaths(Road*);
    void ProcessLaneChangeRequest(Road*,int LCdir,int LCmode,float moveLatDist);


    int ID;
    int agentKind;        // 0 ~ 99 : vehicles(car, truck, motorcycle,etc)
                          // 100 ~ 199 : other traffic participants(pedestrian, bicycle, etc.0
    int agentStatus;
    bool isScenarioObject;
    bool isOldScenarioType;
    bool canAppearRepeatedly;   // This is only used for scenario vehicle, isScenarioObject = true
    bool isSInterfaceObject;
    bool isSInterObjDataSet;
    bool isBehaviorEmbeded;
    bool notAllowedAppear;
    float TimeOfAppear;

    int refSpeedMode;
    int brakeLampOverride;

    struct AgentAttiribute attri;
    struct AgentMemory     memory;
    struct AgentParam      param;
    struct AgentState      state;

    struct AgentMemory     memory_reference;


    QString strForDebug;
    QString strForDebugRiskEval;

    Vehicle vehicle;
    float vHalfLength;
    float vHalfWidth;
    int objTypeForUE4;
    int objNoForUE4;
    int objIDForUE4;

    bool justWarped;
    bool skipSetControlInfo;

    int cognitionCountMax;
    int cognitionCount;
    int cognitionSubCount;

    int decisionMakingCountMax;
    int decisionMakingCount;

    int controlCountMax;
    int controlCount;

    bool onlyCheckPreceding;

    RandomGenerator rndGen;
    double GenUniform(){ return rndGen.GenUniform(); }
    double GetNormalDist(double mean,double std){ return rndGen.GetNormalDist(mean, std); }
    double GetExponentialDist(double mean){ return rndGen.GetExponentialDist(mean); }


    int UE4ObjectID[10];
    int UE4ObjIDDelCount[10];

#ifdef _PERFORMANCE_CHECK_AGENT
    LARGE_INTEGER start, end;
    LARGE_INTEGER freq;
    double calTime[10];
    int calCount[10];
#endif
};

#endif // AGENT_H
