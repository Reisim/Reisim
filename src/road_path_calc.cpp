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


#include "road.h"
#include <QDebug>

int Road::GetNearestPathFromList(float xp, float yp, float yawAngle, float &dist, QList<int> &pathList)
{
    int ret = -1;

    for(int n=0;n<pathList.size();++n){

        int i = pathId2Index.indexOf( pathList[n] );
        if( i < 0 ){
            continue;
        }

        if( paths[i]->xmin > xp || paths[i]->xmax < xp ){
            continue;
        }

        if( paths[i]->ymin > yp || paths[i]->ymax < yp ){
            continue;
        }

        float deviation,xt,yt,xd,yd,s;
        int id = GetDeviationFromPath( paths[i]->id,
                                       xp, yp, yawAngle,
                                       deviation, xt, yt, xd, yd, s );

        if( id == paths[i]->id ){
            if( ret < 0 ){
                ret = id;
                dist = deviation;
            }
            else{
                if( fabs(deviation) < dist ){
                    ret = id;
                    dist = deviation;
                }
            }
        }
    }
    return ret;
}


int Road::GetNearestPath(float xp, float yp,float yawAngle,float &dist)
{
    int ret = -1;

    for(int i=0;i<paths.size();++i){

        if( paths[i]->xmin > xp || paths[i]->xmax < xp ){
            continue;
        }

        if( paths[i]->ymin > yp || paths[i]->ymax < yp ){
            continue;
        }

        float deviation,xt,yt,xd,yd,s;
        int id = GetDeviationFromPath( paths[i]->id,
                                       xp, yp, yawAngle,
                                       deviation, xt, yt, xd, yd, s );

        if( id == paths[i]->id ){
            if( ret < 0 ){
                ret = id;
                dist = deviation;
            }
            else{
                if( fabs(deviation) < dist ){
                    ret = id;
                    dist = deviation;
                }
            }
        }
    }
    return ret;
}


//
//  Calculate deviation of the point (xp,yp) from the Path of pathID.
//  Yaw angle of the vehicle is used to eliminate opposite directional path.
//  If recursive flag is set true and the point (xp, yp) has no cross point
//  with the Path, connected paths are checked to have cross point.
//
//  Return value is the pathID of the path from which deviation is calcuated.
//  Calculated deviation and cross point (xt,yt), tangent-direction vector
//  (xderiv, yderiv) and distance from the StartWP of the path is stored in
//  the argument passed by reference.
//
int Road::GetDeviationFromPath(int pathID,
                               float xp,
                               float yp,
                               float yawAngle,
                               float &deviation,
                               float &xt,
                               float &yt,
                               float &xderiv,
                               float &yderiv,
                               float &distFromStartWP,
                               bool negrectYawAngleInfo )
{
    int index = pathId2Index.indexOf(pathID);
    if( index < 0 ){
        return -1;
    }

    float cya = cos( yawAngle );
    float sya = sin( yawAngle );

    int ret = -1;

    distFromStartWP = 0.0;

    int nDiv = paths[index]->pos.size();

    // Check
    {
        float rx1 = xp - paths[index]->pos.first()->x();
        float ry1 = yp - paths[index]->pos.first()->y();
        float dx1 = paths[index]->derivative.first()->x();
        float dy1 = paths[index]->derivative.first()->y();
        float ip1 = rx1 * dx1 + ry1 * dy1;

        float rx2 = xp - paths[index]->pos.last()->x();
        float ry2 = yp - paths[index]->pos.last()->y();
        float dx2 = paths[index]->derivative.last()->x();
        float dy2 = paths[index]->derivative.last()->y();
        float ip2 = rx2 * dx2 + ry2 * dy2;

        if( negrectYawAngleInfo == false ){
            float dxm = (dx1 + dx2) * 0.5;
            float dym = (dy1 + dy2) * 0.5;

            float ip = cya * dxm + sya * dym;
            if( ip < 0.0 ){
                return ret;
            }
        }

        if( ip1 * ip2 > 0.0 ){
            return ret;
        }
    }

    for(int i=1;i<nDiv;++i){

        float x = paths[index]->pos[i]->x();
        float y = paths[index]->pos[i]->y();

        float rx = xp - x;
        float ry = yp - y;
        float dx = paths[index]->derivative[i]->x();
        float dy = paths[index]->derivative[i]->y();

        float ip = rx * dx + ry * dy;
        if( ip > 0.0 ){
            continue;
        }

        float cp = rx * (-dy) + ry * dx;

        ret = pathID;
        deviation = cp;

        xt = x + ip * dx;
        yt = y + ip * dy;

        xderiv = dx;
        yderiv = dy;

        distFromStartWP = paths[index]->length[i] + ip;

        break;
    }

    return ret;
}


int Road::GetNearestPedestPathSectionIndex(float xp,float yp,float &dist,int &overEdge,int objectID)
{
    int ret = -1;
    float nearestDist = 0.0;
    for(int i=0;i<pedestPaths.size();++i){

        if( objectID >= 0 && pedestPaths[i]->scenarioObjectID != objectID ){
            continue;
        }

        for(int j=0;j<pedestPaths[i]->shape.size()-1;++j){

            float dx = xp - pedestPaths[i]->shape[j]->pos.x();
            float dy = yp - pedestPaths[i]->shape[j]->pos.y();

            float ct = pedestPaths[i]->shape[j]->cosA;
            float st = pedestPaths[i]->shape[j]->sinA;

            float ip = dx * ct + dy * st;
            float cp = dy * ct - dx * st;

            if( ret < 0 || fabs(nearestDist) > fabs(cp) ){
                ret = j;
                nearestDist = cp;

                if( j == 0 && ip < 0.0 ){
                    overEdge = 0;
                }
                else if( j == pedestPaths[i]->shape.size() - 2 && ip > pedestPaths[i]->shape[j]->distanceToNextPos ){
                    overEdge = 2;
                }
                else{
                    overEdge = 1;
                }
            }
        }
    }

    dist = nearestDist;

    return ret;
}


int Road::GetDeviationFromPedestPathAllSection(int pedestPathID, float xp, float yp, float &dev)
{
    int ret = -1;
    float nearestDist = 0.0;

    int plIdx = pedestPathID2Index.indexOf( pedestPathID );
    if( plIdx < 0 ){
        return ret;
    }


    for(int j=0;j<pedestPaths[plIdx]->shape.size()-1;++j){

        float dx = xp - pedestPaths[plIdx]->shape[j]->pos.x();
        float dy = yp - pedestPaths[plIdx]->shape[j]->pos.y();

        float ct = pedestPaths[plIdx]->shape[j]->cosA;
        float st = pedestPaths[plIdx]->shape[j]->sinA;

        float ip = dx * ct + dy * st;
        if( ip < -0.5 || ip > pedestPaths[plIdx]->shape[j]->distanceToNextPos + 0.5 ){
            continue;
        }

        float cp = dy * ct - dx * st;
        if( ret < 0 || fabs(nearestDist) > fabs(cp) ){
            ret = j;
            nearestDist = cp;
        }

    }

    dev = nearestDist;
    return ret;
}


int Road::GetDeviationFromPedestPath(int pedestPathID,int sectIndex,float xp,float yp,
                                     float &dev,float &z,float &xdir,float &ydir,float lateralShift)
{
    int idx = pedestPathID2Index.indexOf( pedestPathID );
    if( idx < 0 ){
        return -1;
    }

    if( sectIndex < 0 || sectIndex >= pedestPaths[idx]->shape.size() - 1 ){
        return -1;
    }

    float dx = xp - pedestPaths[idx]->shape[sectIndex]->pos.x();
    float dy = yp - pedestPaths[idx]->shape[sectIndex]->pos.y();

    float ip = dx * (pedestPaths[idx]->shape[sectIndex]->cosA) + dy * (pedestPaths[idx]->shape[sectIndex]->sinA);
    float cp = dy * (pedestPaths[idx]->shape[sectIndex]->cosA) - dx * (pedestPaths[idx]->shape[sectIndex]->sinA);

    dev = cp;
    z = pedestPaths[idx]->shape[sectIndex]->pos.z() + ip / (pedestPaths[idx]->shape[sectIndex]->distanceToNextPos) * ( pedestPaths[idx]->shape[sectIndex+1]->pos.z() - pedestPaths[idx]->shape[sectIndex]->pos.z() );

    xdir = pedestPaths[idx]->shape[sectIndex+1]->pos.x() - lateralShift * pedestPaths[idx]->shape[sectIndex]->sinA - xp;
    ydir = pedestPaths[idx]->shape[sectIndex+1]->pos.y() + lateralShift * pedestPaths[idx]->shape[sectIndex]->cosA - yp;
    float L = sqrt( xdir * xdir + ydir * ydir );
    if( L > 1.0 ){
        xdir /= L;
        ydir /= L;
    }

    return 0;
}


QList<int> Road::GetPathList(int routeIndex, int currentPath, bool &needLC, int &nodeUntil, RandomGenerator *rndGen)
{
    QList<int> ret;

    needLC = false;
    nodeUntil = -1;

    if( routeIndex < 0 || routeIndex >= odRoute.size() ){
        return ret;
    }

    int destNode = odRoute[routeIndex]->destinationNode;
    int dnIdx = nodeId2Index.indexOf( destNode );
    if( dnIdx < 0 ){
        return ret;
    }

    int destNodeOutDir = odRoute[routeIndex]->routeToDestination.last()->outDir;
    int destNodeInDir = odRoute[routeIndex]->routeToDestination.last()->inDir;

    if( nodes[dnIdx]->nCross > 1 && destNodeOutDir < 0){

        QList<int> outDirCandidates;
        for(int i=0;i<nodes[dnIdx]->pathLists.size();++i){
            if( nodes[dnIdx]->pathLists[i]->inDirect == destNodeInDir ){
                continue;
            }

            if( nodes[dnIdx]->pathLists[i]->outDirect >= 0 && outDirCandidates.indexOf(nodes[dnIdx]->pathLists[i]->outDirect) < 0 ){
                outDirCandidates.append( nodes[dnIdx]->pathLists[i]->outDirect );
            }
        }

        if( outDirCandidates.size() > 0 ){
            float rnd = rndGen->GenUniform();
            int selected = RandomSelect( outDirCandidates.size(), rnd );
            destNodeOutDir = outDirCandidates[selected];
        }
        else{
            qDebug() << "[Fatal] Road::GetPathList, routeIdx = " << routeIndex << ", Node " << destNode << " does not have any path-list for outDir = " << destNodeOutDir;
            return ret;
        }

    }

//    qDebug() << "destNode = " << destNode << " destNodeOutDir = " << destNodeOutDir << " destNodeInDir = " << destNodeInDir;



    // Check if the path list that lead to the destination from current-path without chaning lane exists, by back-propagation.
    QList<int> extractedList;

    for(int i=0;i<nodes[dnIdx]->pathLists.size();++i){
        if( nodes[dnIdx]->pathLists[i]->outDirect == destNodeOutDir &&
                nodes[dnIdx]->pathLists[i]->inDirect == destNodeInDir ){

            int Nlist = nodes[dnIdx]->pathLists[i]->pathList.size();
//            qDebug() << "Nlist = " << Nlist;

            if( Nlist < 1 ){
                qDebug() << "[Fatal] Road::GetPathList, routeIdx = " << routeIndex << ", Node " << destNode << " does not have any path-list for inDir = "
                         << destNodeInDir << " and outDir = " << destNodeOutDir;
                return ret;
            }
            int selected = 0;
            if( Nlist > 1 ){
                float rnd = rndGen->GenUniform();
                selected = RandomSelect( Nlist, rnd );
            }
            for(int j=0;j<nodes[dnIdx]->pathLists[i]->pathList[selected].size();++j){
                extractedList.append( nodes[dnIdx]->pathLists[i]->pathList[selected].at(j) );
            }
        }
    }

//    qDebug() << "[1]extractedList = " << extractedList;


    for(int n = odRoute[routeIndex]->routeToDestination.size()-2; n>0; n-- ){

        int onTheWayNode = odRoute[routeIndex]->routeToDestination[n]->node;
        int outDirOWNode = odRoute[routeIndex]->routeToDestination[n]->outDir;
        int inDirOWNode  = odRoute[routeIndex]->routeToDestination[n]->inDir;

        int owIdx = nodeId2Index.indexOf( onTheWayNode );
        if( owIdx < 0 ){
            qDebug() << "[Fatal] Road::GetPathList, routeIdx = " << routeIndex << ", Node " << onTheWayNode << " in Node-List Data has no corresponding node data.";
            break;
        }

//        qDebug() << "n=" << n << " onTheWayNode = " << onTheWayNode << " outDirOWNode = " << outDirOWNode << " inDirOWNode = " << inDirOWNode;


        for(int i=0;i<nodes[owIdx]->pathLists.size();++i){
            if( nodes[owIdx]->pathLists[i]->outDirect == outDirOWNode && nodes[owIdx]->pathLists[i]->inDirect == inDirOWNode ){

                int Nlist = nodes[owIdx]->pathLists[i]->pathList.size();

//                qDebug() << "Nlist = " << Nlist;

                QList<int> candiateListIndex;
                for(int j=0;j<Nlist;++j){

                    int topPath = nodes[owIdx]->pathLists[i]->pathList[j].first();

//                    qDebug() << "topPath = " << topPath;

                    int tpIdx = pathId2Index.indexOf( topPath );
                    if( tpIdx >= 0 ){

//                        qDebug() << "[1]contain = " << paths[tpIdx]->forwardPaths.contains( extractedList.last() );

                        if( paths[tpIdx]->forwardPaths.contains( extractedList.last() ) == true ){

//                            qDebug() << "[2]contain = " << nodes[owIdx]->pathLists[i]->pathList[j].contains( currentPath );

                            if( n == 1 && nodes[owIdx]->pathLists[i]->pathList[j].contains( currentPath ) == false ){
                                continue;
                            }
                            else{
                                candiateListIndex.append( j );
                            }
                        }
                    }
                    else{
                        qDebug() << "[Fatal] Road::GetPathList, find path whose pathId2Index = -1,  Path ID = " << topPath;
                        continue;
                    }
                }

                if( candiateListIndex.size() == 0 ){

                    needLC = true;
                    nodeUntil = onTheWayNode;

                    break;
                }

                int selected = candiateListIndex[0];
                if( candiateListIndex.size() > 1 ){
                    float rnd = rndGen->GenUniform();
                    selected = candiateListIndex.at( RandomSelect( Nlist, rnd ) );
                }

//                qDebug() << "selected = " << selected;

                for(int j=0;j<nodes[owIdx]->pathLists[i]->pathList[selected].size();++j){
                    extractedList.append( nodes[owIdx]->pathLists[i]->pathList[selected].at(j) );
                }

//                qDebug() << "[2]extractedList = " << extractedList;

                break;
            }
        }
    }

//    qDebug() << "[3]extractedList = " << extractedList;


    if( needLC == false ){
        for(int i=0;i<extractedList.size();++i){
            ret.append( extractedList[i] );
        }
        return ret;
    }


    // In case Lane-Change is needed to go to the destination, generate temporal path list to move, by forward-propagation.

    int badConnectNodeListPos = -1;

    for(int n=1;n<odRoute[routeIndex]->routeToDestination.size();++n){

        int onTheWayNode = odRoute[routeIndex]->routeToDestination[n]->node;
        int outDirOWNode = odRoute[routeIndex]->routeToDestination[n]->outDir;
        int inDirOWNode  = odRoute[routeIndex]->routeToDestination[n]->inDir;

        int owIdx = nodeId2Index.indexOf( onTheWayNode );
        if( owIdx < 0 ){
            qDebug() << "[Fatal] Road::GetPathList, routeIdx = " << routeIndex << ", Node " << onTheWayNode << " in Node-List Data has no corresponding node data.";
            return ret;
        }

        for(int i=0;i<nodes[owIdx]->pathLists.size();++i){
            if( nodes[owIdx]->pathLists[i]->outDirect == outDirOWNode &&
                    nodes[owIdx]->pathLists[i]->inDirect == inDirOWNode ){

                int Nlist = nodes[owIdx]->pathLists[i]->pathList.size();
                QList<int> candiateListIndex;
                for(int j=0;j<Nlist;++j){

                    if( n == 1 && nodes[owIdx]->pathLists[i]->pathList[j].contains( currentPath ) == false ){
                        continue;
                    }
                    else{
                        candiateListIndex.append( j );
                    }
                }

                if( candiateListIndex.size() == 0 ){

                    badConnectNodeListPos = n;
                    break;

                }
                else{
                    int selected = 0;
                    if( candiateListIndex.size() > 1 ){
                        float rnd = rndGen->GenUniform();
                        selected = RandomSelect( Nlist, rnd );
                    }
                    for(int j=nodes[owIdx]->pathLists[i]->pathList[selected].size()-1;j>=0;j--){
                        ret.prepend( nodes[owIdx]->pathLists[i]->pathList[selected].at(j) );
                    }
                }

                break;
            }
        }
    }

    if( badConnectNodeListPos >= 0 ){


    }

    return ret;
}




