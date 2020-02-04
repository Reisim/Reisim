#ifndef AGENT_H
#define AGENT_H

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <QVector>

#include "vehicle.h"
#include "road.h"
#include "trafficsignal.h"


enum AGENT_CONTROL_MODE
{
    AGENT_LOGIC,
    CONSTANT_SPEED_HEADWAY,
    CONSTANT_SPEED_HEADWAY_KEEP_INITIAL_LATERAL_OFFSET,
    USE_TIME_SERIES_DATA,
    SPEED_PROFILE,
    STOP_AT,
    INTERSECTION_TURN_CONTROL,
    SET_STEER_CONTROL_FLAG
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
    UNDEFINED_RECOGNITION_LABEL
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

    int objectPath;
    float deviationFromObjectPath;

    int nearestTargetPath;
    float deviationFromNearestTargetPath;
    float distanceToObject;
    float xOnTargetPath;
    float yOnTargetPath;
    float innerProductToNearestPathTangent;
    float innerProductToNearestPathNormal;
    float effectiveHalfWidth;

    int recognitionLabel;

    bool hasCollisionPoint;
    float xCP;
    float yCP;
    float myDistanceToCP;
    float myTimeToCP;
    float objectDistanceToCP;
    float objectTimeToCP;

    bool inView;
    int noUpdateCount;
};

struct TrafficSignalPerception
{
    int objectID;
    char objectType;

    float x;
    float y;
    float yaw;

    int signalDisplay;

    float stopLineX;
    float stopLineY;
    float distToSL;
    int SLonPathID;
    int stopPointIndex;

    bool inView;
    int noUpdateCount;
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
    float requiredDistToStopFromTargetSpeed;

    float actualTargetSpeed;
    float actualTargetHeadwayDistance;
    float targetSpeed;
    float targetHeadwayDistance;
    float targetSpeedByScenario;
    float actualTargetHeadwayDistanceByScenario;
    float targetHeadwayDistanceByScenario;
    float allowableHeadwayDistDeviation;
    float targetHeadwayTimeByScenario;
    float targetSpeedInsideIntersectionTurnByScenario;
    int startDecel;
    bool activeBrakeInVelocityControl;

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

    int speedProfileCount;
    QVector<float> profileTime;
    QVector<float> profileSpeed;

    bool doHeadwayDistanceControl;
    float axHeadwayControl;

    bool doStopControl;
    float axStopControl;
    int releaseStopCount;

    bool doSteerControl;
    float lateralDeviationFromTargetPathAtPreviewPoint;
    int previewPointPath;
    float lateralDeviationFromTargetPath;
    float steeringControlGainMultiplier;


    // hazard and risk valuation
    int precedingVehicleID;
    int precedingVehicleIDByScenario;
    float distanceToPrecedingVehicle;
    float speedPrecedingVehicle;
    float axPrecedingVehicle;

    float targetLateralShift;
    float targetLateralShiftByScenario;


    // recognition




    // perception
    QList<struct AgentPerception *> perceptedObjects;
    QList<struct TrafficSignalPerception *> perceptedSignals;



    // guidance
    QVector<int> targetPathList;
    QVector<int> targetPathListBackup;
    int currentTargetPath;
    int currentTargetPathIndexInList;
    float distanceFromStartWPInCurrentPath;
    int currentTargetDirectionPedestPath;
    int scenarioPathSelectID;

    QList<int> myNodeList;
    QList<int> myInDirList;
    QList<int> myOutDirList;


    // navigation
    int routeType;
    int routeIndex;

};

struct AgentParam
{
    float accelControlGain;
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
};

struct AgentAttiribute
{
    int age;
    int gender;
};


struct SInterObjInfo
{
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

    void Perception( Agent**, int, Road*, QList<TrafficSignal*> trafficSignal );
    void Recognition( Agent**, int, Road* ) ;
    void HazardIdentification();
    void RiskEvaluation();
    void Control(Road*);
    void UpdateState();


    void SpeedControl();
    void SpeedAdjustForCurve(Road*,int,float);
    void HeadwayControl();
    void StopControl();


    void CheckPathList(Road*);
    void SetTargetNodeListByTargetPaths(Road*);


    int ID;
    int agentKind;        // 0 ~ 99 : vehicles(car, truck, motorcycle,etc)
                          // 100 ~ 199 : other traffic participants(pedestrian, bicycle, etc.0
    int agentStatus;
    bool isScenarioObject;
    bool isSInterfaceObject;
    bool isBehaviorEmbeded;
    bool notAllowedAppear;

    struct AgentAttiribute attri;
    struct AgentMemory     memory;
    struct AgentParam      param;
    struct AgentState      state;


    Vehicle vehicle;
    float vHalfLength;
    float vHalfWidth;

    bool justWarped;
    bool skipSetControlInfo;
};

#endif // AGENT_H