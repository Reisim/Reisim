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


#ifndef ROAD_H
#define ROAD_H

#include <QList>
#include <QVector2D>
#include <QVector3D>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <math.h>
#include "randomgenerator.h"


#define MAX_POINT_DIV_PATH  (10)

#define   LEFT_HAND_TRAFFIC   (0)
#define   RIGHT_HAND_TRAFFIC  (1)



enum DIRECTION_LABEL
{
    ONCOMING,
    LEFT_CROSSING,
    RIGHT_CROSSING,
    STRAIGHT,
    UNDEFINED_DIRECTION
};


struct WP
{
    int id;
    QVector3D pos;
    float direct;
    int scenarioObjectID;
    int order;
    float speedInfo;   // [m/s]

    float cosDirect;
    float sinDirect;

    bool isNodeInWP;
    bool isNodeOutWP;
    int relatedNode;
    int relatedNodeLeg;
};


struct CrossPoint
{
    int crossPathID;
    QVector3D pos;
    QVector2D derivative;
    float distFromStartWP;
};

struct PedestCrossPoint
{
    int crossPathID;
    int sectionIndex;
    QVector3D pos;
    QVector2D derivative;
    float distFromStartWP;
};

struct StopPoint
{
    int stopPointID;
    int type;
    QVector3D pos;
    QVector3D derivative;
    float distFromStartWP;
    int relatedNode;
    int relatedDir;
};


struct Path
{
    int id;
    int startWpId;
    int endWpId;
    QList<int> forwardPaths;
    QList<int> followingPaths;

    int numDivPath;
    QList<QVector3D*> pos;
    QList<QVector2D*> derivative;

    QList<float> curvature;
    QList<float> length;

    float pathLength;
    float meanPathCurvature;
    float maxPathCurvature;

    float speedInfo;   // Speed Limit[m/s]
    float speed85pt;    // Actual Speed, 85th-percentile[m/s]

    float xmin;
    float ymin;
    float xmax;
    float ymax;

    int scenarioObjectID;

    QList<struct CrossPoint*> crossPoints;
    QList<struct StopPoint*> stopPoints;
    QList<struct PedestCrossPoint*> pedestCrossPoints;

    int connectingNode;
    int connectingNodeInDir;

    bool setSpeedVariationParam;
    float vDevP;
    float vDevM;
    float refVforDev;
    float accelAtDev;
    float decelAtDev;
};


struct PedestPathShapeInfo
{
    QVector3D pos;
    float angleToNextPos;  // in [rad]
    float cosA;
    float sinA;
    float width;
    float distanceToNextPos;
    bool isCrossWalk;
    int controlPedestSignalID;
    int runOutDirect;
    float runOutProb;
};

struct PedestPath
{
    int id;
    QList<PedestPathShapeInfo*> shape;
    QList<int> trafficVolume;
    int scenarioObjectID;

    int totalVolume;
    QList<float> pedestKindSelectProbability;
    float meanArrivalTime;
    float NextAppearTime;
};


struct NodeConnectInfo
{
    int outDirectionID;
    int connectedNode;
    int inDirectionID;
};


struct BoundaryWPs
{
    int relatedDirection;
    int wpId;
    int laneNo;
    int leftWpId;
    int rightWpId;
    QList<int> PathWithSWP;
    QList<int> PathWithEWP;
};


struct LanePathList
{
    int inDirect;
    int outDirect;
    QList<QList<int>> pathList;
};


struct DirectionMap
{
    int inDirect;
    int oncomingDirect;
    QList<int> leftDirect;
    QList<int> rightDirect;
};


struct Node
{
    int id;
    float xc;
    float yc;
    int nCross;
    QList<int> legIDs;
    QList<struct NodeConnectInfo*> nodeConnectInfo;
    QList<struct BoundaryWPs*> inBoundaryWPs;
    QList<struct BoundaryWPs*> outBoundaryWPs;
    QList<struct LanePathList*> pathLists;
    QList<struct DirectionMap*> directionMap;
    QList<int> nWPin;
    QList<int> nWPout;
    bool hasTS;
    QList<int> relatedVTSIndex;
    QList<int> relatedPTSIndex;
    bool isMergeNode;
    QList<struct StopPoint*> stopPoints;
};


struct RouteElem
{
    int inDir;
    int node;
    int outDir;
};


struct RouteLaneData
{
    int startNode;
    int goalNode;
    int sIndexInNodeList;
    int gIndexInNodeList;
    QList<QList<int>> laneList;
    int LCDirect;
};


struct ODRouteData
{
    int originNode;
    int destinationNode;
    QList<struct RouteElem*> routeToDestination;

    QList<QList<int>> laneListsToDestination;

    QList<struct RouteLaneData*> LCSupportLaneLists;

    QList<int> trafficVolumne;
    int totalVolumne;
    QList<float> vehicleKindSelectProbability;
    float meanArrivalTime;
    float NextAppearTime;
    QList<QList<QPoint>> mergeLanesInfo;

    bool onlyForScenarioVehicle;
    int relatedScenarioObjectID;

    bool allowAgentGeneration;
};


struct ObjectCategoryAndSize
{
    int id;
    QString category;
    QString subcategory;
    float length;
    float width;
    float height;
    int type;
    int No;
    int UE4ModelID;
    float meanSpeed;
    float stdDevSpeed;
    int ageInfo;
};


class Road
{
public:
    Road();

    void LoadRoadData(QString filename);
    void ClearRoadData();

    void SetPathShape(int id);
    void SetPathConnection();
    void SetPathRelatedNodes();

    void CalculatePathShape(struct Path *);
    void CalculatePathCurvature(struct Path *);
    float CalculateCurvatureFromThreePoints(float x1,float y1,float x2,float y2,float x3,float y3);

    float GetPathLength(int pathID);
    float GetPathMeanCurvature(int pathID);
    
    int GetTargetNodeOfPath(int pathID);

    int GetNearestPath(float xp, float yp,float yawAngle,
                       float &deviation,float &xt,float &yt,float &xderiv,float &yderiv,float &distFromStartWP,
                       bool negrectYawAngleInfo = false );
    int GetNearestPathFromList(QList<int> &pathList,float xp,float yp,float zp,float yawAngle,
                               float &deviation,float &xt,float &yt,float &xderiv,float &yderiv,float &distFromStartWP);
    int GetNearestPathFromListWithZ(QList<int> &pathList,float xp,float yp,float yawAngle,
                               float &deviation,float &xt,float &yt,float &zt,float &xderiv,float &yderiv,float &distFromStartWP);
    int GetDeviationFromPath(int pathID,
                             float xp,float yp,float yawAngle,
                             float &deviation,float &xt,float &yt,float &xderiv,float &yderiv,float &distFromStartWP,
                             bool negrectYawAngleInfo = false );
    int GetDeviationFromPathWithZ(int pathID,
                                 float xp,float yp,float zp,float yawAngle,
                                 float &deviation,float &xt,float &yt,float &xderiv,float &yderiv,float &distFromStartWP,
                                 bool negrectYawAngleInfo = false );
    int GetDeviationFromPathExtendEnd(int pathID,
                                     float xp,float yp,float yawAngle,
                                     float &deviation,float &xt,float &yt,float &xderiv,float &yderiv,float &distFromStartWP,
                                     bool negrectYawAngleInfo = false );

    int GetNearestPedestPathSectionIndex(float xp,float yp,float &dist,int &overEdge,int objectID=-1);
    bool GetNearestPedestPath(float xp,float yp,float psip,float &latDev,int &nearPathID,int &sectID,float &distInSect);

    int GetDeviationFromPedestPath(int pedestPathID,int sectIndex,float xp,float yp,
                                   float &dev,float &z,float &xdir,float &ydir);
    float GetSpeedAdjustFactorPedestPath(int pedestPathID,int sectIndex,float xp,float yp,float V);
    int GetDeviationFromPedestPathAllSection(int pedestPathID,float xp,float yp, float &dev,float &dist,float &xdir, float& ydir);


    void CheckSideBoundaryWPs(struct Node *);
    int GetDirectionLabel(int nodeID,int inDir,int checkDir);

    int CreateWPforScenarioObject(float x,float y,float z,float direct,int objectID,int order,float speedInfo);
    void CreatePathsforScenarioObject(int objectID);
    int CreatePedestPathsforScenarioObject(int objectID,
                                            float x1,float y1,float z1,
                                            float x2,float y2,float z2,
                                            float w,
                                            bool isCrossWalk,
                                            int roadSideInfo);
    float GetPedestPathWidth(int pedestPathID, int sectionID);
    void CheckPedestPathConnection();
    void SetPedestPathArrivalTimes();

    QList<int> GetPathList(int routeIndex,int currentPath,bool &needLC,int &nodeUntil, RandomGenerator*);
    int RandomSelect(int N,float rnd);

    QList<int> wpId2Index;
    QList<int> pathId2Index;
    QList<int> nodeId2Index;
    QList<int> pedestPathID2Index;


    QList<struct WP*> wps;
    QList<struct Path*> paths;
    QList<struct Node*> nodes;
    QList<struct PedestPath*> pedestPaths;

    QString currentRoadFileName;

    QList<struct ODRouteData*> odRoute;
    QList<struct ObjectCategoryAndSize*> vehicleKind;
    QList<struct ObjectCategoryAndSize*> pedestrianKind;
    int numActorForUE4Model;
    int maxActorInUE4;

    int LeftOrRight;
};

#endif // ROAD_H
