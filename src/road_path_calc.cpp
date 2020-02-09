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

int Road::GetNearestPathFromList(float xp, float yp, float yawAngle, float &dist, QVector<int> &pathList)
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
                               bool recursive,
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

    for(int i=0;i<paths[index]->pos.size()-1;++i){

        float rx = xp - paths[index]->pos[i]->x();
        float ry = yp - paths[index]->pos[i]->y();
        float dx = paths[index]->derivative[i]->x();
        float dy = paths[index]->derivative[i]->y();

        float ip = rx * dx + ry * dy;
        if( ip < -0.08 ){
            continue;
        }

        float nrx = xp - paths[index]->pos[i+1]->x();
        float nry = yp - paths[index]->pos[i+1]->y();
        float ndx = paths[index]->derivative[i+1]->x();
        float ndy = paths[index]->derivative[i+1]->y();

        float nip = nrx * ndx + nry * ndy;
        if( nip > + 0.08 ){
            continue;
        }

        if( negrectYawAngleInfo == false ){
            float ip2 = cya * dx + sya * dy;
            if( ip2 < 0.0 ){
                continue;
            }
        }


        float cp = rx * (-dy) + ry * dx;
        float r = ip / paths[index]->length[i+1];
        if( r > 1.0){
            r = 1.0;
        }
        else if( r < 0.0 ){
            r = 0.0;
        }

        if( ret < 0 ){
            ret = pathID;
            deviation = cp;
            xt = paths[index]->pos[i]->x() + ip * dx;
            yt = paths[index]->pos[i]->y() + ip * dy;
            xderiv = dx + (ndx - dx) * r;
            yderiv = dy + (ndy - dy) * r;
            distFromStartWP = paths[index]->length[i] + ip;
        }
        else{
            if( fabs(deviation) > fabs(cp) ){
                deviation = cp;
                xt = paths[index]->pos[i]->x() + ip * dx;
                yt = paths[index]->pos[i]->y() + ip * dy;
                xderiv = dx + (ndx - dx) * r;
                yderiv = dy + (ndy - dy) * r;
                distFromStartWP = paths[index]->length[i] + ip;
            }
        }
    }

    if( ret < 0 && recursive == true ){

        float rx1 = xp - paths[index]->pos.first()->x();
        float ry1 = yp - paths[index]->pos.first()->y();
        float L1 = rx1 * rx1 + ry1 * ry1;

        float rx2 = xp - paths[index]->pos.last()->x();
        float ry2 = yp - paths[index]->pos.last()->y();
        float L2 = rx2 * rx2 + ry2 * ry2;

        if( L1 < L2 ){
            for(int i=0;i<paths[index]->followingPaths.size();++i){

                float tdev,txt,tyt,txd,tyd,ts;
                int chk = GetDeviationFromPath( paths[index]->followingPaths[i],
                                                xp, yp, yawAngle,
                                                tdev, txt, tyt, txd, tyd, ts );

                if( chk == paths[index]->followingPaths[i] ){
                    if( ret < 0 ){
                        ret = chk;
                        deviation = tdev;
                        xt = txt;
                        yt = tyt;
                        xderiv = txd;
                        yderiv = tyd;
                        distFromStartWP = ts;
                    }
                    else{
                        if( fabs(deviation) > fabs(tdev) ){
                            ret = chk;
                            deviation = tdev;
                            xt = txt;
                            yt = tyt;
                            xderiv = txd;
                            yderiv = tyd;
                            distFromStartWP = ts;
                        }
                    }
                }
            }
        }
        else{
            for(int i=0;i<paths[index]->forwardPaths.size();++i){

                float tdev,txt,tyt,txd,tyd,ts;
                int chk = GetDeviationFromPath( paths[index]->forwardPaths[i],
                                                xp, yp, yawAngle,
                                                tdev, txt, tyt, txd, tyd, ts );

                if( chk == paths[index]->forwardPaths[i] ){
                    if( ret < 0 ){
                        ret = chk;
                        deviation = tdev;
                        xt = txt;
                        yt = tyt;
                        xderiv = txd;
                        yderiv = tyd;
                        distFromStartWP = ts;
                    }
                    else{
                        if( fabs(deviation) > fabs(tdev) ){
                            ret = chk;
                            deviation = tdev;
                            xt = txt;
                            yt = tyt;
                            xderiv = txd;
                            yderiv = tyd;
                            distFromStartWP = ts;
                        }
                    }
                }
            }
        }
    }
    return ret;
}


int Road::GetDirectionByPedestPathLink(int pedestPathID, int connectedPedestPathID)
{
    int ret = 0;
    int idx = pedestPathID2Index.indexOf( pedestPathID );
    if( idx < 0 ){
        return ret;
    }
    for(int i=0;i<pedestPaths[idx]->connectedPedestPath1.size();++i){
        if( pedestPaths[idx]->connectedPedestPath1[i] == connectedPedestPathID ){
            ret = 1;
            break;
        }
    }
    if( ret == 0 ){
        for(int i=0;i<pedestPaths[idx]->connectedPedestPath2.size();++i){
            if( pedestPaths[idx]->connectedPedestPath2[i] == connectedPedestPathID ){
                ret = 2;
                break;
            }
        }
    }
    return ret;
}


int Road::GetNearestPedestPath(float xp,float yp,float &dist,int &overEdge,int objectID)
{
    int ret = -1;
    float nearestDist = 0.0;
    for(int i=0;i<pedestPaths.size();++i){

        if( objectID >= 0 && pedestPaths[i]->scenarioObjectID != objectID ){
            continue;
        }

        float dx = xp - pedestPaths[i]->x1;
        float dy = yp - pedestPaths[i]->y1;

        float ip = dx * pedestPaths[i]->eX + dy * pedestPaths[i]->eY;
        float cp = dy * pedestPaths[i]->eX - dx * pedestPaths[i]->eY;

        if( ret < 0 || fabs(nearestDist) > fabs(cp) ){
            ret = pedestPaths[i]->id;
            nearestDist = cp;
            if( ip > pedestPaths[i]->Length ){
                overEdge = 2;
            }
            else if( ip < 0.0 ){
                overEdge = 1;
            }
            else{
                overEdge = 0;
            }
        }
    }

    dist = nearestDist;

    return ret;
}


int Road::GetDeviationFromPedestPath(int pedestPathID,float xp,float yp,
                                     float &dev,float &z,
                                     float &xdir1,float &ydir1,
                                     float &xdir2,float &ydir2)
{
    int idx = pedestPathID2Index.indexOf( pedestPathID );
    if( idx < 0 ){
        return -1;
    }

    float dx = xp - pedestPaths[idx]->x1;
    float dy = yp - pedestPaths[idx]->y1;

    float ip = dx * pedestPaths[idx]->eX + dy * pedestPaths[idx]->eY;
    float cp = dy * pedestPaths[idx]->eX - dx * pedestPaths[idx]->eY;

    dev = cp;
    z = pedestPaths[idx]->z1 + ip / pedestPaths[idx]->Length * ( pedestPaths[idx]->z2 - pedestPaths[idx]->z1 );

    xdir1 = pedestPaths[idx]->x1 - xp;
    ydir1 = pedestPaths[idx]->y1 - yp;
    float L = sqrt( xdir1 * xdir1 + ydir1 * ydir1 );
    if( L > 1.0 ){
        xdir1 /= L;
        ydir1 /= L;
    }

    xdir2 = pedestPaths[idx]->x2 - xp;
    ydir2 = pedestPaths[idx]->y2 - yp;
    L = sqrt( xdir2 * xdir2 + ydir2 * ydir2 );
    if( L > 1.0 ){
        xdir2 /= L;
        ydir2 /= L;
    }

    return 0;
}


QVector<int> Road::GetPathList(int routeIndex, int currentPath, bool &needLC, int &nodeUntil, RandomGenerator *rndGen)
{
    QVector<int> ret;

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




