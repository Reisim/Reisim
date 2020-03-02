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


#ifndef SIMULATIONMANAGER_H
#define SIMULATIONMANAGER_H

#include <QList>
#include <QVector>
#include <QString>
#include <QStringList>
#include "randomgenerator.h"
#include "agent.h"
#include "road.h"
#include "trafficsignal.h"


enum TRIGGER_TYPE
{
    AT_ONCE,
    TIME_TRIGGER,
    POSITION_TRIGGER,
    POSITION_SPEED_TRIGGER,
    BY_FUNCTION_EXTENDER,
    POSITION_TIME_TRIGGER,
    BY_KEYBOARD_FUNC_KEY
};

enum ROUTE_TYPE
{
    NODE_LIST_TYPE,
    PATH_LIST_TYPE,
};

enum SCENARIO_EVENT_TYPE
{
    SYSTEM_EVENT,
    OBJECT_EVENT
};

enum SYSTEM_EVENT_KIND
{
    DO_NOTHING_AS_SYSTEM_EVENT,
    SET_OBJECT_POSITION,
    SEND_UDP_DATA_BY_SYSTEM_EVENT,
    CHANGE_ROAD_SPEEDLIMIT,
    CHANGE_TRAFFIC_SIGNAL
};

enum OBJECT_EVENT_KIND
{
    DO_NOTHING_AS_OBJECT_EVENT,
    SUDDEN_DECELERATION,
    LANE_CHANGE,
    LANE_DEPARTURE,
    TRAFFIC_SIGNAL_VIOLATION,
    CHANGE_CONTROL_MODE,
    SET_LATERAL_OFFSET,
    SEND_UDP_DATA_BY_OBJECT_EVENT,
    HEADLIGHT,
    VEHICLE_LAMPS
};


struct SimulationTime
{
    int day;
    int hour;
    int min;
    int sec;
    float msec;
    int msec_count;
    int exe_freq;
    float dt;
};


struct ObjectTriggerData
{
    int targetObjectID;
    float x;
    float y;
    float direction;  // [rad
    float speed;      // [m/s]
    float TTC;        // [sec]
    int targetEventID;
    float cosDirect;
    float sinDirect;
};


struct ScenarioTriggers
{
    int mode;
    int func_keys;
    struct SimulationTime* timeTrigger;
    float timeTriggerInSec;
    QVector<struct ObjectTriggerData*> objectTigger;
    bool ANDCondition;
    bool byExternalTriggerFlag;
};


struct ScenarioNodeRoute
{
    int node;
    int inDirect;
    int outDirect;
};


struct ScenarioWPRoute
{
    float x;
    float y;
    float z;
    float direct;    // [rad]
    float speedInfo; // [m/s]
    int wpID;
};

struct ScenarioPedestPathRoute
{
    float x1;
    float y1;
    float z1;
    float x2;
    float y2;
    float z2;
    bool isCrossWalk;
    float width;
    int roadSideInfo;
};


struct ScenarioObjectControlInfo
{
    int mode;
    float targetSpeed;
    float targetHeadwayDistance;
    float targetHeadwayTime;
    float targetLateralOffset;
    int targetObjectID;
    float stopAtX;
    float stopAtY;
    QList<float> profileAxis;
    QList<float> speedProfile;
    int routeType;
    QList<struct ScenarioNodeRoute*> nodeRoute;
    QList<struct ScenarioWPRoute*> wpRoute;
    QList<struct ScenarioPedestPathRoute*> ppRouteElem;
    QList<int> pedestPathRoute;
};


struct ScenarioEvents
{
    int eventID;

    int eventType;       // System Event or Object Event
    int targetObjectID;  // Target Object ID for Object Event

    int eventKind;       // What kind of Event
    QVector<int> eventIntData;
    QVector<float> eventFloatData;

    struct ScenarioTriggers* eventTrigger;   // Event Triggers

    int eventState;
    int eventTimeCount;
    int eventTimeCount_sub;
};

struct ScenarioItem
{
    char type;
    int objectID;
    int objectModelID;
    bool repeat;
    int status;
    QVector<float> initialState;  // (X,Y,Z,YawAngle[rad],Speed[m/s])
    struct ScenarioTriggers* appearTriggers;
    struct ScenarioTriggers* disappearTriggers;
    struct ScenarioObjectControlInfo* controlInfo;
};


struct ScenarioData
{
    int scenarioID;
    struct ScenarioTriggers* endTrigger;
    bool repeat;
    int status;
    QVector<struct ScenarioItem*> scenarioItems;
    QVector<struct ScenarioEvents*> scenarioEvents;
};



class SimulationManager
{
public:
    SimulationManager();

    void ResetSimulationTime();
    QString GetSimulationTimeStr();
    float GetSimulationTimeInSec();
    void UpdateSimulationTime();
    void SetFrequency(int);
    void SetDSMode() { DSMode = true; }

    void AppearAgents(Agent**,int,Road *);
    void DisappearAgents(Agent**,int);

    void RaiseEvent(Agent**,int,Road *);
    void ClearScenarioData();
    void AllocateScenario(int id);
    void SetScenarioEndTrigger(int sID, struct ScenarioTriggers* trigger);
    void SetScenarioRepeatFlag(int sID);
    void SetScenarioItem(int sID, struct ScenarioItem* item);
    void SetScenarioEvent(int sID, struct ScenarioEvents* event);

    int GetNumberScenarioData(){ return scenario.size(); }
    int GetNumberScenarioObject(int sId){ return scenario[sId]->scenarioItems.size(); }
    int GetScenarioObjectID(int sId,int itemId){ return scenario[sId]->scenarioItems[itemId]->objectID; }
    char GetScenarioObjectType(int sId,int itemId){ return scenario[sId]->scenarioItems[itemId]->type; }
    int GetNumberWPDataOfScenarioObject(int sId,int itemId){ return scenario[sId]->scenarioItems[itemId]->controlInfo->wpRoute.size(); }
    struct ScenarioWPRoute* GetWPDataOfScenarioObject(int sId,int itemId,int idx){ return scenario[sId]->scenarioItems[itemId]->controlInfo->wpRoute[idx]; }
    void SetAppearFlagByFE(int id);

    int GetScenarioObjectRouteType(int sId,int itemId){ return scenario[sId]->scenarioItems[itemId]->controlInfo->routeType; }
    int GetNumberPPElemOfScenarioObject(int sId,int itemId){ return scenario[sId]->scenarioItems[itemId]->controlInfo->ppRouteElem.size(); }
    struct ScenarioPedestPathRoute* GetPPElemOfScenarioObject(int sId,int itemId,int idx){ return scenario[sId]->scenarioItems[itemId]->controlInfo->ppRouteElem[idx]; }
    void SetPedestPathRouteToScenarioObject(int sId,int itemId,int ppID){
        if( scenario[sId]->scenarioItems[itemId]->controlInfo->pedestPathRoute.indexOf( ppID ) < 0 ){
            scenario[sId]->scenarioItems[itemId]->controlInfo->pedestPathRoute.append( ppID ) ;
        }
    }

    void CheckScenarioState();

    double GenUniform(){ return rndGen.GenUniform(); }
    double GetNormalDist(double mean,double std){ return rndGen.GetNormalDist(mean, std); }
    double GetExponentialDist(double mean){ return rndGen.GetExponentialDist(mean); }

    int GetVehicleShapeByWheelbase(float wl,Road *pRoad);
    void DumpScenarioData();
    void SetTargetPathListScenarioVehicle(Agent** pAgent,Road *pRoad,int aID);
    void CopyScenarioData(int fromAID,int toAID);


    RandomGenerator rndGen;


private:
    struct SimulationTime simTime;

    QVector<struct ScenarioData *> scenario;
    int currentScenarioID;

    bool DSMode;
    int IDAllowed;
};

#endif // SIMULATIONMANAGER_H
