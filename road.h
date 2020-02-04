/****************************************************************************
**                                 Re:sim
**
**   Copyright (C) 2020 Misaki Design.LLC
**   Copyright (C) 2020 Jun Tajima <tajima@misaki-design.co.jp>
**
**   This file is part of the Re:sim simulation software
**
**   This file may be used under the terms of the GNU Lesser General Public
**   License version 3 as published by the Free Software Foundation.
**   For more detail, visit https://www.gnu.org/licenses/gpl-3.0.html
**
*************************************************************************** */


#ifndef ROAD_H
#define ROAD_H

#include <QVector>
#include <QVector2D>
#include <QVector3D>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <math.h>
#include "randomgenerator.h"


#define MAX_POINT_DIV_PATH  (10)


enum DIRECTION_LABEL
{
    ONCOMING,
    LEFT_CORSSING,
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
};


struct CrossPoint
{
    int crossPathID;
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
    QVector<int> forwardPaths;
    QVector<int> followingPaths;

    int numDivPath;
    QVector<QVector3D*> pos;
    QVector<QVector2D*> derivative;

    QVector<float> curvature;
    QVector<float> length;
    float pathLength;

    float speedInfo;   // [m/s]

    float xmin;
    float ymin;
    float xmax;
    float ymax;

    int scenarioObjectID;

    QVector<struct CrossPoint*> crossPoints;
    QVector<struct StopPoint*> stopPoints;

    int connectingNode;
    int connectingNodeInDir;
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
    bool hasTS;
};


struct PedestPath
{
    int id;

    float x1;
    float y1;
    float z1;
    QList<int> connectedPedestPath1;

    float x2;
    float y2;
    float z2;
    QList<int> connectedPedestPath2;

    float eX;
    float eY;
    float Length;

    bool isCrossWalk;
    float width;
    int roadSideDirection;  //  = 1 if right side of path 1->2 is road
                            //  = 2 if left side of path 1->2 is road

    int scenarioObjectID;
};


struct RouteElem
{
    int inDir;
    int node;
    int outDir;
};

struct ODRouteData
{
    int originNode;
    int destinationNode;
    QList<struct RouteElem*> routeToDestination;
    QList<int> trafficVolumne;
    int totalVolumne;
    QList<float> vehicleKindSelectProbability;
    float meanArrivalTime;
    float NextAppearTime;
};


struct ObjectCategoryAndSize
{
    QString category;
    QString subcategory;
    float length;
    float width;
    float height;
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
    int GetNearestPath(float xp, float yp,float yawAngle,float &dist);
    int GetNearestPathFromList(float xp,float yp,float yawAngle,float &dist,QVector<int> &pathList);
    int GetDeviationFromPath(int pathID,
                             float xp,float yp,float yawAngle,
                             float &deviation,float &xt,float &yt,float &xderiv,float &yderiv,float &distFromStartWP,
                             bool recursive = false, bool negrectYawAngleInfo = false );

    int GetDirectionByPedestPathLink(int pedestPathID, int connectedPedestPathID);
    int GetNearestPedestPath(float xp,float yp,float &dist,int &overEdge,int objectID=-1);
    int GetDeviationFromPedestPath(int pedestPathID,float xp,float yp,
                                   float &dev,float &z,
                                   float &xdir1,float &ydir1,
                                   float &xdir2,float &ydir2);

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
    void CheckPedestPathConnection();

    QVector<int> GetPathList(int routeIndex,int currentPath,bool &needLC,int &nodeUntil, RandomGenerator*);
    int RandomSelect(int N,float rnd);

    QVector<int> wpId2Index;
    QVector<int> pathId2Index;
    QVector<int> nodeId2Index;
    QVector<int> pedestPathID2Index;

    QVector<struct WP*> wps;
    QVector<struct Path*> paths;
    QVector<struct Node*> nodes;
    QVector<struct PedestPath*> pedestPaths;

    QString currentRoadFileName;

    QList<struct ODRouteData*> odRoute;
    QList<struct ObjectCategoryAndSize*> vehicleKind;
    QList<struct ObjectCategoryAndSize*> pedestrianKind;

};

#endif // ROAD_H
