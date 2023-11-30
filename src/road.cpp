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

#include "networkdrivecheck.h"


Road::Road()
{
    LeftOrRight = 0;
}


void Road::ClearRoadData()
{
    if( wps.size() > 0 ){
        for(int i=0;i<wps.size();++i){
            delete wps[i];
        }
        wps.clear();
    }

    if( paths.size() > 0 ){
        for(int i=0;i<paths.size();++i){
            for(int j=0;j<paths[i]->pos.size();++j){
                delete paths[i]->pos[j];
            }
            for(int j=0;j<paths[i]->derivative.size();++j){
                delete paths[i]->derivative[j];
            }
            paths[i]->pos.clear();
            paths[i]->derivative.clear();
            paths[i]->length.clear();
            paths[i]->curvature.clear();
            paths[i]->forwardPaths.clear();
            paths[i]->followingPaths.clear();
            delete paths[i];
        }
        paths.clear();
    }

    if( pedestPaths.size() > 0 ){
        for(int i=0;i<pedestPaths.size();++i){
            for(int j=0;j<pedestPaths[i]->shape.size();++j){
                delete pedestPaths[i]->shape[j];
            }
            pedestPaths[i]->shape.clear();
            delete pedestPaths[i];
        }
        pedestPaths.clear();
    }

    if( nodes.size() > 0 ){
        for(int i=0;i<nodes.size();++i){
            if( nodes[i]->nodeConnectInfo.size() > 0 ){
                for(int j=0;j<nodes[i]->nodeConnectInfo.size();++j){
                    delete nodes[i]->nodeConnectInfo[j];
                }
                nodes[i]->nodeConnectInfo.clear();
            }

            if( nodes[i]->inBoundaryWPs.size() > 0 ){
                for(int j=0;j<nodes[i]->inBoundaryWPs.size();++j){
                    delete nodes[i]->inBoundaryWPs[j];
                }
                nodes[i]->inBoundaryWPs.clear();
            }

            if( nodes[i]->outBoundaryWPs.size() > 0 ){
                for(int j=0;j<nodes[i]->outBoundaryWPs.size();++j){
                    delete nodes[i]->outBoundaryWPs[j];
                }
                nodes[i]->outBoundaryWPs.clear();
            }

            if( nodes[i]->pathLists.size() > 0 ){
                for(int j=0;j<nodes[i]->pathLists.size();++j){
                    for(int k=0;k<nodes[i]->pathLists[j]->pathList.size();++k){
                        nodes[i]->pathLists[j]->pathList[k].clear();
                    }
                    nodes[i]->pathLists[j]->pathList.clear();

                    delete nodes[i]->pathLists[j];
                }
                nodes[i]->pathLists.clear();
            }

            delete nodes[i];
        }
        nodes.clear();
    }

    wpId2Index.clear();
    pathId2Index.clear();
    nodeId2Index.clear();
    pedestPathID2Index.clear();

    for(int i=0;i<odRoute.size();++i){
        for(int j=0;j<odRoute[i]->routeToDestination.size();++j){
            delete odRoute[i]->routeToDestination[j];
        }
        odRoute[i]->routeToDestination.clear();
        odRoute[i]->trafficVolumne.clear();
        odRoute[i]->vehicleKindSelectProbability.clear();

        delete odRoute[i];
    }
    odRoute.clear();

    for(int i=0;i<vehicleKind.size();++i){
        delete vehicleKind[i];
    }
    vehicleKind.clear();

    for(int i=0;i<pedestrianKind.size();++i){
        delete pedestrianKind[i];
    }
    pedestrianKind.clear();
}


void Road::LoadRoadData(QString filename)
{
    qDebug() << "[Road::LoadRoadData] filename = " << filename;

    if( filename.isNull() ){
        return;
    }

    QFile file( CheckNetworkDrive(filename) );
    if( !file.open(QIODevice::ReadOnly | QIODevice::Text) ){
        QMessageBox::warning(NULL,"Error","Cannot open file: " + filename);
        return;
    }

    QTextStream in(&file);
    QString line;

    line = in.readLine();
    line = in.readLine();
    if( line.contains("Re:sim Road Data File") == false ){
        QMessageBox::warning(NULL,"Error","The file is not Re:sim Road Data File");
        file.close();
        return;
    }
    line = in.readLine();


    ClearRoadData();


    numActorForUE4Model = 10;
    maxActorInUE4 = 1000;

    QList<struct StopPoint*> tmpStopPointsData;

    while( in.atEnd() == false ){

        line = in.readLine();
        if( line.startsWith("#") || line.isEmpty() || line.contains(";") == false ){
            continue;
        }

        //qDebug() << "line : " << line;

        QStringList divLine = line.split(";");
        QString tag = QString( divLine[0] ).trimmed();

        //qDebug() << "tag : " << tag;

        if( tag == QString("Left-Hand Or Right-Hand") ){

            if( QString( divLine[1]).trimmed().contains("RIGHT") == true ){
                LeftOrRight = 1;
            }
            else{
                LeftOrRight = 0;
            }

        }
        else if( tag == QString("WayPoint") ){

            struct WP* wp = new struct WP;

            QStringList elem = QString( divLine[1]).split(",");

            wp->id = QString( elem[0] ).trimmed().toInt();
            wp->pos.setX( QString(elem[1]).trimmed().toFloat() );
            wp->pos.setY( QString(elem[2]).trimmed().toFloat() );
            wp->pos.setZ( QString(elem[3]).trimmed().toFloat() );
            wp->direct = QString(elem[4]).trimmed().toFloat();

            wp->cosDirect = cos(wp->direct);
            wp->sinDirect = sin(wp->direct);

            wp->scenarioObjectID = -1;
            wp->order            = -1;
            wp->speedInfo        = 0.0;

            wp->isNodeInWP     = false;
            wp->isNodeOutWP    = false;
            wp->relatedNode    = -1;
            wp->relatedNodeLeg = -1;

            wps.append( wp );
        }
        else if( tag == QString("Path") ){

            struct Path* path = new struct Path;

            QStringList elem = QString( divLine[1]).split(",");

            path->id = QString( elem[0] ).trimmed().toInt();
            path->startWpId  = QString( elem[1] ).trimmed().toInt();
            path->endWpId    = QString( elem[2] ).trimmed().toInt();
            path->numDivPath = QString( elem[3] ).trimmed().toInt() - 1;
            path->speedInfo  = QString( elem[4] ).trimmed().toFloat() / 3.6;
            path->speed85pt  = QString( elem[5] ).trimmed().toFloat() / 3.6;

            path->scenarioObjectID = -1;

            path->setSpeedVariationParam = false;
            path->vDevP = 0.0;
            path->vDevM = 0.0;
            path->refVforDev = 0.0;
            path->accelAtDev = 0.0;
            path->decelAtDev = 0.0;

            paths.append( path );
        }
        else if( tag == QString("CrossPoint") ){

            QStringList elem = QString( divLine[1]).split(",");

            int pathID = QString( elem[0] ).trimmed().toInt();
            for(int i=0;i<paths.size();++i){
                if( paths[i]->id == pathID ){

                    for(int j=1;j<elem.size();++j){

                        QStringList cpData = QString( elem[j] ).trimmed().split("/");

                        if( cpData.size() == 7 ){

                            struct CrossPoint *cp = new struct CrossPoint;

                            cp->crossPathID = QString( cpData[0] ).trimmed().toInt();
                            cp->pos.setX( QString( cpData[1] ).trimmed().toFloat() );
                            cp->pos.setY( QString( cpData[2] ).trimmed().toFloat() );
                            cp->pos.setZ( QString( cpData[3] ).trimmed().toFloat() );
                            cp->derivative.setX( QString( cpData[4] ).trimmed().toFloat() );
                            cp->derivative.setY( QString( cpData[5] ).trimmed().toFloat() );
                            cp->distFromStartWP = QString( cpData[6] ).trimmed().toFloat();

                            paths[i]->crossPoints.append( cp );
                        }
                        else{
                            qDebug() << "Invalid Data : CrossPoint of Path " << pathID << ", size of cpData = " << cpData.size();
                            qDebug() << "  Data = " << QString( elem[j] );
                        }

                    }
                    break;
                }
            }
        }
        else if( tag == QString("PedestCrossPoint") ){

            QStringList elem = QString( divLine[1]).split(",");

            int pathID = QString( elem[0] ).trimmed().toInt();

            for(int i=0;i<paths.size();++i){
                if( paths[i]->id == pathID ){

                    for(int j=1;j<elem.size();++j){

                        QStringList cpData = QString( elem[j] ).trimmed().split("/");

                        if( cpData.size() == 8 ){

                            struct PedestCrossPoint *pcp = new struct PedestCrossPoint;

                            pcp->crossPathID = QString( cpData[0] ).trimmed().toInt();
                            pcp->sectionIndex = QString( cpData[1] ).trimmed().toInt();
                            pcp->pos.setX( QString( cpData[2] ).trimmed().toFloat() );
                            pcp->pos.setY( QString( cpData[3] ).trimmed().toFloat() );
                            pcp->pos.setZ( QString( cpData[4] ).trimmed().toFloat() );
                            pcp->derivative.setX( QString( cpData[5] ).trimmed().toFloat() );
                            pcp->derivative.setY( QString( cpData[6] ).trimmed().toFloat() );
                            pcp->distFromStartWP = QString( cpData[7] ).trimmed().toFloat();

                            paths[i]->pedestCrossPoints.append( pcp );
                        }
                        else{
                            qDebug() << "Invalid Data : PedestCrossPoint of Path " << pathID << ", size of cpData = " << cpData.size();
                            qDebug() << "  Data = " << QString( elem[j] );
                        }

                    }
                    break;
                }
            }
        }
        else if( tag == QString("StopPoint") ){

            QStringList elem = QString( divLine[1]).split(",");

            int pathID = QString( elem[0] ).trimmed().toInt();
            for(int i=0;i<paths.size();++i){
                if( paths[i]->id == pathID ){

                    for(int j=1;j<elem.size();++j){

                        QStringList spData = QString( elem[j] ).trimmed().split("/");

                        if( spData.size() == 10 ){

                            struct StopPoint *sp = new struct StopPoint;

                            sp->stopPointID = QString( spData[0] ).trimmed().toInt();
                            sp->type = QString( spData[1] ).trimmed().toInt();
                            sp->pos.setX( QString( spData[2] ).trimmed().toFloat() );
                            sp->pos.setY( QString( spData[3] ).trimmed().toFloat() );
                            sp->pos.setZ( QString( spData[4] ).trimmed().toFloat() );
                            sp->derivative.setX( QString( spData[5] ).trimmed().toFloat() );
                            sp->derivative.setY( QString( spData[6] ).trimmed().toFloat() );
                            sp->distFromStartWP = QString( spData[7] ).trimmed().toFloat();
                            sp->relatedNode = QString( spData[8] ).trimmed().toInt();
                            sp->relatedDir = QString( spData[9] ).trimmed().toInt();

                            paths[i]->stopPoints.append( sp );

                            tmpStopPointsData.append( sp );
                        }
                        else{
                            qDebug() << "Invalid Data : StopPoint of Path " << pathID << ", size of spData = " << spData.size();
                            qDebug() << "  Data = " << QString( elem[j] );
                        }

                    }
                    break;
                }
            }
        }
        else if( tag == QString("Pedest-Path") ){

            struct PedestPath* pedestpath = new struct PedestPath;

            pedestpath->id = QString( divLine[1] ).trimmed().toInt();
            pedestpath->scenarioObjectID = -1;

            pedestPaths.append( pedestpath );
        }
        else if ( tag == QString("Pedest-Path Shape") ) {

            struct PedestPathShapeInfo *si = new struct PedestPathShapeInfo;

            QStringList elem = QString( divLine[1]).split(",");

            si->pos.setX( QString( elem[0] ).trimmed().toFloat() );
            si->pos.setY( QString( elem[1] ).trimmed().toFloat() );
            si->pos.setZ( QString( elem[2] ).trimmed().toFloat() );
            si->width = QString( elem[3] ).trimmed().toFloat();
            si->distanceToNextPos = QString( elem[4] ).trimmed().toFloat();
            si->angleToNextPos = QString( elem[5] ).trimmed().toFloat();  // [rad]
            si->cosA = cos( si->angleToNextPos );
            si->sinA = sin( si->angleToNextPos );

            pedestPaths.last()->shape.append( si );
        }
        else if ( tag == QString("Pedest-Path Property") ) {

            QStringList elem = QString( divLine[1]).split(",");

            pedestPaths.last()->shape.last()->isCrossWalk = ( QString( elem[0] ).trimmed().toInt() == 1 ? true : false );
            pedestPaths.last()->shape.last()->controlPedestSignalID = QString( elem[1] ).trimmed().toInt();
            pedestPaths.last()->shape.last()->runOutProb = QString( elem[2] ).trimmed().toFloat();
            pedestPaths.last()->shape.last()->runOutDirect = QString( elem[3] ).trimmed().toInt();

        }
        else if ( tag == QString("Pedest-Path Traffic") ) {

            QStringList elem = QString( divLine[1]).split(",");

            for(int i=0;i<elem.size();++i){
                pedestPaths.last()->trafficVolume.append( QString( elem[i] ).trimmed().toInt() );
            }
        }
        else if( tag == QString("Node") ){

            struct Node* node = new struct Node;

            QStringList elem = QString( divLine[1]).split(",");

            node->id = QString( elem[0] ).trimmed().toInt();
            node->xc = QString( elem[1] ).trimmed().toFloat();
            node->yc = QString( elem[2] ).trimmed().toFloat();
            node->nCross = QString( elem[3] ).trimmed().toInt();

            QStringList legIDStr = QString( elem[4] ).trimmed().split("/");
            for(int i=0;i<legIDStr.size();++i){
                node->legIDs.append( QString(legIDStr[i]).trimmed().toInt() );
            }

            node->hasTS  = QString( elem[5] ).trimmed().toInt();

            for(int i=0;i<tmpStopPointsData.size();++i){
                if( tmpStopPointsData[i]->relatedNode == node->id ){
                    node->stopPoints.append( tmpStopPointsData[i] );
                }
            }

            nodes.append( node );
        }
        else if( tag == QString("Node Connection") ){

            QStringList elem = QString( divLine[1]).split(",");

            int id = QString( elem[0] ).trimmed().toInt();
            for(int i=0;i<nodes.size();++i){
                if( nodes[i]->id != id ){
                    continue;
                }

                for(int j=1;j<elem.size();++j){

                    QStringList connectInfo = QString(elem[j]).split("/");

                    struct NodeConnectInfo *nci = new struct NodeConnectInfo;

                    nci->inDirectionID  = QString( connectInfo[0] ).trimmed().toInt();
                    nci->connectedNode  = QString( connectInfo[1] ).trimmed().toInt();
                    nci->outDirectionID = QString( connectInfo[2] ).trimmed().toInt();

                    nodes[i]->nodeConnectInfo.append( nci );
                }
                break;
            }
        }
        else if ( tag == QString("Direction Map") ) {

            QStringList elem = QString( divLine[1]).split(",");

            int id = QString( elem[0] ).trimmed().toInt();

            for(int i=0;i<nodes.size();++i){
                if( nodes[i]->id != id ){
                    continue;
                }

                for(int j=1;j<elem.size();++j){

                    QStringList dirInfo = QString(elem[j]).split("/");

                    struct DirectionMap *dm = new struct DirectionMap;

                    dm->inDirect = QString( dirInfo[0] ).trimmed().toInt();
                    dm->oncomingDirect = QString( dirInfo[1] ).trimmed().toInt();

                    QString leftDirs = QString( dirInfo[2] );
                    if( leftDirs.isNull() == false && leftDirs.isEmpty() == false ){

                        QStringList leftDirsDiv = leftDirs.split("|");
                        for(int k=0;k<leftDirsDiv.size();++k){
                            dm->leftDirect.append( QString(leftDirsDiv[k]).trimmed().toInt() );
                        }
                    }

                    QString rightDirs = QString( dirInfo[3] );
                    if( rightDirs.isNull() == false && rightDirs.isEmpty() == false ){

                        QStringList rightDirsDiv = rightDirs.split("|");
                        for(int k=0;k<rightDirsDiv.size();++k){
                            dm->rightDirect.append( QString(rightDirsDiv[k]).trimmed().toInt() );
                        }
                    }

                    nodes[i]->directionMap.append( dm );
                }
                break;
            }

        }
        else if( tag == QString("In-boundary WPs") ){

            if( QString( divLine[1] ).contains(",") == false ){
                continue;
            }

            QStringList elem = QString( divLine[1] ).split(",");

            int id = QString( elem[0] ).trimmed().toInt();
            for(int i=0;i<nodes.size();++i){
                if( nodes[i]->id != id ){
                    continue;
                }
                for(int j=1;j<elem.size();++j){

                    QStringList bwpStr = QString(elem[j]).split("/");

                    int direct = QString( bwpStr[0] ).trimmed().toInt();
                    for(int k=1;k<bwpStr.size();++k){

                        int bwp = QString( bwpStr[k] ).trimmed().toInt();

                        struct BoundaryWPs *b = new struct BoundaryWPs;

                        b->wpId = bwp;
                        b->relatedDirection = direct;
                        b->laneNo = k - 1;
                        b->leftWpId = -1;
                        b->rightWpId = -1;

                        nodes[i]->inBoundaryWPs.append( b );
                    }
                }
                break;
            }
        }
        else if( tag == QString("Out-boundary WPs") ){

            if( QString( divLine[1] ).contains(",") == false ){
                continue;
            }

            QStringList elem = QString( divLine[1] ).split(",");

            int id = QString( elem[0] ).trimmed().toInt();
            for(int i=0;i<nodes.size();++i){
                if( nodes[i]->id != id ){
                    continue;
                }
                for(int j=1;j<elem.size();++j){

                    QStringList bwpStr = QString(elem[j]).split("/");

                    int direct = QString( bwpStr[0] ).trimmed().toInt();
                    for(int k=1;k<bwpStr.size();++k){

                        int bwp = QString( bwpStr[k] ).trimmed().toInt();

                        struct BoundaryWPs *b = new struct BoundaryWPs;

                        b->wpId = bwp;
                        b->relatedDirection = direct;
                        b->laneNo = j - 1;
                        b->leftWpId = -1;
                        b->rightWpId = -1;

                        nodes[i]->outBoundaryWPs.append( b );
                    }
                }
                break;
            }
        }
        else if( tag == QString("Lane List") ){

            QStringList elem = QString( divLine[1] ).split(",");

            if( elem.size() == 4 ){

                int nodeId = QString( elem[0] ).trimmed().toInt();
                int outDir = QString( elem[1] ).trimmed().toInt();
                int inDir  = QString( elem[2] ).trimmed().toInt();

                for(int i=0;i<nodes.size();++i){
                    if( nodes[i]->id != nodeId ){
                        continue;
                    }

                    int pIdx = -1;
                    for(int j=0;j<nodes[i]->pathLists.size();++j){
                        if( nodes[i]->pathLists[j]->outDirect == outDir && nodes[i]->pathLists[j]->inDirect == inDir ){
                            pIdx = j;
                            break;
                        }
                    }
                    if( pIdx < 0 ){
                        struct LanePathList *lp = new struct LanePathList;

                        lp->outDirect = outDir;
                        lp->inDirect  = inDir;

                        nodes[i]->pathLists.append( lp );

                        pIdx = nodes[i]->pathLists.size() - 1;
                    }

                    QList<int> tmpList;
                    QStringList listStr = QString( elem[3] ).split("<-");
                    for(int j=0;j<listStr.size();++j){
                        tmpList.append( QString(listStr[j]).trimmed().toInt() );
                    }

                    nodes[i]->pathLists[pIdx]->pathList.append( tmpList );
                }
            }

        }
        else if( tag == QString("Route Data") ){

            struct ODRouteData *odr = new ODRouteData;

            QStringList elem = QString( divLine[1] ).split(",");

            odr->originNode      = QString( elem[0] ).trimmed().toInt();
            odr->destinationNode = QString( elem[1] ).trimmed().toInt();

            QStringList nodeListStr = QString( elem[2] ).trimmed().split("/");
            for(int i=0;i<nodeListStr.size();++i){

                QStringList nodeListElem = QString( nodeListStr[i] ).trimmed().split("|");
                if( nodeListElem.size() == 3 ){

                    struct RouteElem *re = new struct RouteElem;

                    re->inDir  = QString( nodeListElem[0] ).trimmed().toInt();
                    re->node   = QString( nodeListElem[1] ).trimmed().toInt();
                    re->outDir = QString( nodeListElem[2] ).trimmed().toInt();

                    odr->routeToDestination.append( re );
                }
            }

            QStringList volumeStr = QString( elem[3] ).trimmed().split("/");
            for(int i=0;i<volumeStr.size();++i){

                QStringList volumeElem = QString( volumeStr[i] ).trimmed().split("|");
                if( volumeElem.size() == 2 ){

                    odr->trafficVolumne.append(  QString( volumeElem[1] ).trimmed().toInt() );
                }
            }

            odr->totalVolumne = 0;
            for(int i=0;i<odr->trafficVolumne.size();++i){
                odr->totalVolumne += odr->trafficVolumne[i];
            }

            for(int i=0;i<odr->trafficVolumne.size();++i){
                float p = 0.0;
                if( odr->totalVolumne > 0 ){
                    p = (float)(odr->trafficVolumne[i]) / (float)(odr->totalVolumne);
                }
                odr->vehicleKindSelectProbability.append( p );
            }

            if( odr->totalVolumne > 0 ){
                odr->meanArrivalTime = 3600.0 / (float)odr->totalVolumne;
            }
            else{
                odr->meanArrivalTime = -1.0;
            }

            odr->NextAppearTime = -1.0;

            odr->onlyForScenarioVehicle = false;
            odr->allowAgentGeneration = true;

            odRoute.append( odr );
        }
        else if( tag == QString("Route Multi-Lanes") ){

            QStringList elem = QString( divLine[1] ).split(",");

            static int currentODRouteIndex = -1;

            int com = QString( elem[0] ).trimmed().toInt();
            if( com == 1 ){

                currentODRouteIndex = -1;

                QStringList nodeList = elem;
                nodeList.removeAt(0);

//                qDebug() << "Route Multi-Lanes: nodeList = " << nodeList;

                for(int i=0;i<odRoute.size();++i){

                    if( nodeList.size() != odRoute[i]->routeToDestination.size() ){
                        continue;
                    }

                    bool matched = true;
                    for(int j=0;j<odRoute[i]->routeToDestination.size();++j){
                        if( odRoute[i]->routeToDestination[j]->node != QString(nodeList[j]).trimmed().toInt() ){
                            matched = false;
                            break;
                        }
                    }
                    if( matched == false ){
                        continue;
                    }

                    currentODRouteIndex = i;

//                    qDebug() << "currentODRouteIndex = " << currentODRouteIndex;
                }
            }
            else if( com == 2 ){

                if( currentODRouteIndex >= 0 && currentODRouteIndex < odRoute.size() && elem.size() >= 5 ){

                    struct RouteLaneData *rld = new struct RouteLaneData;

                    rld->startNode = QString( elem[1] ).trimmed().toInt();
                    rld->goalNode  = QString( elem[2] ).trimmed().toInt();
                    rld->sIndexInNodeList = QString( elem[3] ).trimmed().toInt();
                    rld->gIndexInNodeList = QString( elem[4] ).trimmed().toInt();

                    rld->LCDirect = DIRECTION_LABEL::STRAIGHT;

                    odRoute[currentODRouteIndex]->LCSupportLaneLists.append( rld );

//                    qDebug() << "RouteLaneData add to odRoute[" << currentODRouteIndex << "]->LCSupportLaneLists: size = "
//                             << odRoute[currentODRouteIndex]->LCSupportLaneLists.size();
//                    qDebug() << "    Start = " << rld->startNode << " Goal = " << rld->goalNode;
                }

            }
            else if( com == 3 ){

                if( currentODRouteIndex >= 0 && currentODRouteIndex < odRoute.size() ){

                    struct RouteLaneData *rld = odRoute[currentODRouteIndex]->LCSupportLaneLists.last();

                    QList<int> lanelist;
                    for(int i=1;i<elem.size();++i){

                        int lane = QString(elem[i]).trimmed().toInt();
                        lanelist.append( lane );

                    }

                    rld->laneList.append( lanelist );

//                    for(int i=0;i<odRoute[currentODRouteIndex]->LCSupportLaneLists.size();++i){
//                        qDebug() << "odRoute[" << currentODRouteIndex << "]->LCSupportLaneLists[" << i << "]:";
//                        for(int j=0;j<odRoute[currentODRouteIndex]->LCSupportLaneLists[i]->laneList.size();++j){
//                            qDebug() << "  Lane List[" << j << "] : " << odRoute[currentODRouteIndex]->LCSupportLaneLists[i]->laneList[j];
//                        }
//                    }
                }
            }
        }
        else if( tag == QString("Route Lanes") ){

            QStringList elem = QString( divLine[1] ).split("|");

            QStringList nodeList = QString( elem[0] ).trimmed().split(",");

//            qDebug() << "Route Lanes: nodeList = " << nodeList;

            for(int i=0;i<odRoute.size();++i){

                if( nodeList.size() != odRoute[i]->routeToDestination.size() ){
                    continue;
                }

                bool matched = true;
                for(int j=0;j<odRoute[i]->routeToDestination.size();++j){
                    if( odRoute[i]->routeToDestination[j]->node != QString(nodeList[j]).trimmed().toInt() ){
                        matched = false;
                        break;
                    }
                }
                if( matched == false ){
                    continue;
                }

                QStringList laneLists = QString( elem[1] ).trimmed().split("/");
                for(int j=0;j<laneLists.size();++j){

                    QList<int> laneList;
                    QStringList lanes = QString(laneLists[j]).trimmed().split(",");
                    for(int k=0;k<lanes.size();++k){
                        laneList.append( QString(lanes[k]).trimmed().toInt() );
                    }

                    odRoute[i]->laneListsToDestination.append( laneList );

                }

//                qDebug() << "Route Lanes: " << odRoute[i]->originNode << " -> " << odRoute[i]->destinationNode;
//                for(int j=0;j<odRoute[i]->laneListsToDestination.size();++j){
//                    qDebug() << "[" <<  j << "] : " << odRoute[i]->laneListsToDestination[j];
//                }

                break;
            }

        }
        else if( tag == QString("Vehicle Kind") ){

            QStringList elem = QString( divLine[1] ).split(",");
            if( elem.size() >= 5 ){

                struct ObjectCategoryAndSize *k = new ObjectCategoryAndSize;

                k->id = vehicleKind.size();

                k->category    = QString( elem[0] ).trimmed();
                k->subcategory = QString( elem[1] ).trimmed();

                k->length = QString( elem[2] ).trimmed().toFloat();
                k->width  = QString( elem[3] ).trimmed().toFloat();
                k->height = QString( elem[4] ).trimmed().toFloat();

                if( elem.size() >= 6 ){
                    if( QString( elem[5] ).isNull() == false && QString( elem[5] ).isEmpty() == false ){
                        k->UE4ModelID  = QString( elem[5] ).trimmed().toInt();
                    }
                    else{
                        k->UE4ModelID  = -1;
                    }
                }
                else{
                    k->UE4ModelID  = -1;
                }

                k->meanSpeed   = 0.0;
                k->stdDevSpeed = 0.0;
                k->ageInfo     = 0;

                k->type = 0;  // Vehicle
                k->No = vehicleKind.size();

                vehicleKind.append( k );
            }
        }
        else if( tag == QString("Pedestrian Kind") ){

            QStringList elem = QString( divLine[1] ).split(",");
            if( elem.size() >= 5 ){

                struct ObjectCategoryAndSize *k = new ObjectCategoryAndSize;

                k->id = pedestrianKind.size();

                k->category    = QString( elem[0] ).trimmed();
                k->subcategory = QString( elem[1] ).trimmed();

                k->length = QString( elem[2] ).trimmed().toFloat();
                k->width  = QString( elem[3] ).trimmed().toFloat();
                k->height = QString( elem[4] ).trimmed().toFloat();

                if( k->category.contains("Pedestrian") == true ){
                    k->type = 1;
                }
                else if( k->category.contains("Bicycle") == true ){
                    k->type = 2;
                }
                else{
                    k->type = 1;
                }

                if( elem.size() >= 9 ){
                    if( QString( elem[5] ).isNull() == false && QString( elem[5] ).isEmpty() == false ){
                        k->UE4ModelID  = QString( elem[5] ).trimmed().toInt();
                    }
                    else{
                        k->UE4ModelID  = -1;
                    }
                    if( QString( elem[6] ).isNull() == false && QString( elem[6] ).isEmpty() == false ){
                        k->meanSpeed   = QString( elem[6] ).trimmed().toFloat();
                    }
                    else{
                        k->meanSpeed = 0.0;
                    }
                    if( QString( elem[7] ).isNull() == false && QString( elem[7] ).isEmpty() == false ){
                        k->stdDevSpeed = QString( elem[7] ).trimmed().toFloat();
                    }
                    else{
                        k->stdDevSpeed = 0.0;
                    }
                    if( QString( elem[8] ).isNull() == false && QString( elem[8] ).isEmpty() == false ){
                        k->ageInfo     = QString( elem[8] ).trimmed().toFloat();
                    }
                    else{
                        k->ageInfo = 0;
                    }
                }
                else{
                    k->UE4ModelID  = -1;
                    k->meanSpeed   = 0.0;
                    k->stdDevSpeed = 0.0;
                    k->ageInfo = 0;
                }

                k->No = 0;
                for(int n=0;n<pedestrianKind.size();++n){
                    if( pedestrianKind[n]->type == k->type ){
                        k->No = pedestrianKind[n]->No + 1;
                    }
                }

                pedestrianKind.append( k );
            }
        }
        else if( tag == QString("Number of Actor for UE4 Model") ){

            numActorForUE4Model = QString( divLine[1]).trimmed().toInt();
        }
        else if( tag == QString("Max Number of Actors of UE4") ){

            maxActorInUE4 = QString( divLine[1]).trimmed().toInt();
        }

    }

    file.close();

    tmpStopPointsData.clear();


    // Set Dummy Vehicle Size Data if no data supplied
    if( vehicleKind.size() == 0 ){
        struct ObjectCategoryAndSize *k = new ObjectCategoryAndSize;

        k->id = vehicleKind.size();

        k->category    = QString( "Dummy Vehicle" );
        k->subcategory = QString( "Normal Sedan" );

        k->length = 4.7;
        k->width  = 1.72;
        k->height = 1.5;

        k->UE4ModelID = -1;
        k->meanSpeed = 0.0;
        k->stdDevSpeed = 0.0;
        k->ageInfo = 0;

        vehicleKind.append( k );
    }


    qDebug() << "size of wps ; " << wps.size();
    qDebug() << "size of paths ; " << paths.size();
    qDebug() << "size of pedestPaths ; " << pedestPaths.size();
    qDebug() << "size of nodes ; " << nodes.size();


    //
    //  Set ID to Index data
    //
    for(int i=0;i<wps.size();++i){
        wpId2Index.append( wps[i]->id );
    }

    for(int i=0;i<paths.size();++i){
        pathId2Index.append( paths[i]->id );
    }

    for(int i=0;i<pedestPaths.size();++i){
        pedestPathID2Index.append( pedestPaths[i]->id );
    }

    for(int i=0;i<nodes.size();++i){
        nodeId2Index.append( nodes[i]->id );
    }



    //
    //  Set Extra Path Data
    //
    for(int i=0;i<paths.size();++i){
        //qDebug() << " set path shape : id = " << paths[i]->id;
        SetPathShape( paths[i]->id );
    }

    for(int i=0;i<paths.size();++i){
        //qDebug() << " calculate path curvature : id = " << paths[i]->id;
        CalculatePathCurvature( paths[i] );
    }

    SetPathConnection();

    SetPathRelatedNodes();


    //
    // Check size of vectors
    //
    for(int i=0;i<paths.size();++i){
        if( paths[i]->pos.size() != paths[i]->derivative.size() ||
                paths[i]->pos.size() != paths[i]->curvature.size() ||
                paths[i]->pos.size() != paths[i]->length.size() ){

            qDebug() << "!!! check size of vectors.";
            qDebug() << "  Path ID : " << paths[i]->id;
            qDebug() << "  Number of Path Division : " << paths[i]->numDivPath;
            qDebug() << "  size of pos : " << paths[i]->pos.size();
            qDebug() << "  size of derivative : " << paths[i]->derivative.size();
            qDebug() << "  size of curvature : " << paths[i]->curvature.size();
            qDebug() << "  size of length : " << paths[i]->length.size();

        }
    }


    //
    //  Set Extra PedestPath Data
    //
    CheckPedestPathConnection();

    SetPedestPathArrivalTimes();



    //
    //  Set Extra Node Data
    //
    for(int i=0;i<nodes.size();++i){
        CheckSideBoundaryWPs( nodes[i] );

        nodes[i]->isMergeNode = false;

        nodes[i]->nWPin.clear();
        nodes[i]->nWPout.clear();;

        for(int j=0;j<nodes[i]->nCross;++j){

            int nWPin = 0;
            int nWPout = 0;
            for(int k=0;k<nodes[i]->inBoundaryWPs.size();++k){
                if( nodes[i]->inBoundaryWPs[k]->relatedDirection == j ){
                    nWPin++;
                }
            }
            for(int k=0;k<nodes[i]->outBoundaryWPs.size();++k){
                if( nodes[i]->outBoundaryWPs[k]->relatedDirection == j ){
                    nWPout++;
                }
            }

            nodes[i]->nWPin.append( nWPin );
            nodes[i]->nWPout.append( nWPout );
        }

        if( nodes[i]->nCross == 3 && nodes[i]->hasTS == false ){

            QList<int> outDirList;
            for(int k=0;k<nodes[i]->outBoundaryWPs.size();++k){
                int outDir = nodes[i]->outBoundaryWPs[k]->relatedDirection;
                if( outDirList.indexOf( outDir ) < 0 ){
                    outDirList.append( outDir );
                }
            }
            if( outDirList.size() == 1 ){
                bool hasSL = false;
                for(int j=0;j<paths.size();++j){
                    if( paths[j]->connectingNode == nodes[i]->id ){
                        if( paths[j]->stopPoints.size() > 0 ){
                            hasSL = true;
                            break;
                        }
                    }
                }
                if( hasSL == false ){
                    nodes[i]->isMergeNode = true;
                    qDebug() << "Node " << nodes[i]->id << " : Merge Node";
                }
            }
        }
    }



    //
    //   Set Extra Route Data
    //
    for(int i=0;i<odRoute.size();++i){

        for(int j=0;j<odRoute[i]->laneListsToDestination.size();++j){

            QList<QPoint> pairData;

            odRoute[i]->mergeLanesInfo.append( pairData );


            for(int k=0;k<odRoute[i]->laneListsToDestination[j].size()-1;++k){

                int pIdx = pathId2Index.indexOf( odRoute[i]->laneListsToDestination[j][k] );
                if( pIdx >= 0 ){
                    if( paths[pIdx]->followingPaths.size() == 2 ){

                        int p1 = paths[pIdx]->followingPaths[0];
                        int p2 = paths[pIdx]->followingPaths[1];

                        if( p2 == odRoute[i]->laneListsToDestination[j][k+1] ){
                            int t = p2;
                            p2 = p1;
                            p1 = t;
                        }

                        int p1Idx = pathId2Index.indexOf( p1 );
                        int p2Idx = pathId2Index.indexOf( p2 );
                        if( p1Idx >= 0 && p2Idx >= 0 ){

                            if( paths[p1Idx]->connectingNodeInDir == paths[p2Idx]->connectingNodeInDir ){

                                QPoint mergeLanePair;
                                mergeLanePair.setX( p1 );
                                mergeLanePair.setY( p2 );
                                if( odRoute[i]->mergeLanesInfo.last().contains( mergeLanePair ) == false ){
                                    odRoute[i]->mergeLanesInfo.last().append( mergeLanePair );
                                }
                            }
                        }
                    }
                }
                else{
                    qDebug() << "Invalid path found: id = " << odRoute[i]->laneListsToDestination[j][k] << " in laneListsToDestination";
                    qDebug() << "No index found.";
                    qDebug() << "odRoute: O = " << odRoute[i]->originNode << " D = " << odRoute[i]->destinationNode;
                }
            }
        }

        for(int j=0;j<odRoute[i]->LCSupportLaneLists.size();++j){

            for(int k=0;k<odRoute[i]->LCSupportLaneLists[j]->laneList.size();++k){

                QList<QPoint> pairData;

                odRoute[i]->mergeLanesInfo.append( pairData );

                for(int l=0;l<odRoute[i]->LCSupportLaneLists[j]->laneList[k].size()-1;++l){

                    int pIdx = pathId2Index.indexOf( odRoute[i]->LCSupportLaneLists[j]->laneList[k][l] );
                    if( pIdx >= 0 ){
                        if( paths[pIdx]->followingPaths.size() == 2 ){

                            int p1 = paths[pIdx]->followingPaths[0];
                            int p2 = paths[pIdx]->followingPaths[1];

                            if( p2 == odRoute[i]->LCSupportLaneLists[j]->laneList[k][l+1] ){
                                int t = p2;
                                p2 = p1;
                                p1 = t;
                            }

                            int p1Idx = pathId2Index.indexOf( p1 );
                            int p2Idx = pathId2Index.indexOf( p2 );
                            if( p1Idx >= 0 && p2Idx >= 0 ){

                                if( paths[p1Idx]->connectingNodeInDir == paths[p2Idx]->connectingNodeInDir ){

                                    QPoint mergeLanePair;
                                    mergeLanePair.setX( p1 );
                                    mergeLanePair.setY( p2 );
                                    if( odRoute[i]->mergeLanesInfo.last().contains( mergeLanePair ) == false ){
                                        odRoute[i]->mergeLanesInfo.last().append( mergeLanePair );
                                    }
                                }
                            }
                        }
                    }
                    else{
                        qDebug() << "Invalid path found: id = " << odRoute[i]->LCSupportLaneLists[j]->laneList[k][l] << " in LCSupportLaneLists[" << j << "]->laneList[" << k << "]";
                        qDebug() << "No index found.";
                        qDebug() << "odRoute: O = " << odRoute[i]->originNode << " D = " << odRoute[i]->destinationNode;
                    }
                }
            }

        }

        if( odRoute[i]->mergeLanesInfo.size() > 0 &&  odRoute[i]->mergeLanesInfo[0].isEmpty() == false ){
        	qDebug() << "mergeLanesInfo for odRoute[" << i << "]";
        	for(int j=0;j<odRoute[i]->mergeLanesInfo.size();++j){
            	qDebug() << odRoute[i]->mergeLanesInfo[j];
        	}
    	}
    }


    for(int i=0;i<odRoute.size();++i){

        if( odRoute[i]->LCSupportLaneLists.size() < 2 ){
            continue;
        }

        for(int j=1;j<odRoute[i]->LCSupportLaneLists.size();++j){

//            qDebug() << "odRoute[" << i << "]: need LC , LCSupportLaneLists[" << j << "] -> [" << j-1 << "]";

            struct RouteLaneData *rld = odRoute[i]->LCSupportLaneLists.at( j );

            int myWPin = -1;
            for(int k=0;k<rld->laneList[0].size();++k){

                int lIdx = pathId2Index.indexOf( rld->laneList[0][k] );
                if( paths[lIdx]->connectingNode != rld->goalNode ){
                    continue;
                }
                int twp = paths[lIdx]->endWpId;
                int twpIdx = wpId2Index.indexOf( twp );
                if( wps[twpIdx]->isNodeInWP == true ){
                    myWPin = twp;
                    break;
                }
            }
//            qDebug() << "    myWPin = " << myWPin;

            struct RouteLaneData *nextrld = odRoute[i]->LCSupportLaneLists.at( j-1 );

            int targetWPin = -1;
            for(int k=0;k<nextrld->laneList[0].size();++k){
                int lIdx = pathId2Index.indexOf( nextrld->laneList[0][k] );
                if( paths[lIdx]->connectingNode != rld->goalNode ){
                    continue;
                }
                int twp = paths[lIdx]->endWpId;
                int twpIdx = wpId2Index.indexOf( twp );
                if( wps[twpIdx]->isNodeInWP == true ){
                    targetWPin = twp;
                    break;
                }
            }

//            qDebug() << "    targetWPin = " << targetWPin;

            if( myWPin >= 0 && targetWPin >= 0 ){

                int laneNoDiff = 0;

                int ndIdx = nodeId2Index.indexOf( rld->goalNode );
                for(int k=0;k<nodes[ndIdx]->inBoundaryWPs.size();++k){
                    if( nodes[ndIdx]->inBoundaryWPs[k]->wpId == myWPin ){
                        laneNoDiff += nodes[ndIdx]->inBoundaryWPs[k]->laneNo;

//                        qDebug() << "    [m]laneNoDiff = " << laneNoDiff;

                    }
                    if( nodes[ndIdx]->inBoundaryWPs[k]->wpId == targetWPin ){
                        laneNoDiff -= nodes[ndIdx]->inBoundaryWPs[k]->laneNo;

//                        qDebug() << "    [t]laneNoDiff = " << laneNoDiff;
                    }
                }

                if( laneNoDiff > 0 ){
                    rld->LCDirect = DIRECTION_LABEL::LEFT_CROSSING;
                }
                else{
                    rld->LCDirect = DIRECTION_LABEL::RIGHT_CROSSING;
                }

            }

//            qDebug() << "    LCDirect = " << rld->LCDirect << " " << (rld->LCDirect == DIRECTION_LABEL::RIGHT_CROSSING ? "RIGHT" : (rld->LCDirect == DIRECTION_LABEL::LEFT_CROSSING ? "LEFT" : "STRAIGHT") );
        }
    }



    qDebug() << "[Road::LoadRoadData] data loaded.";
}


void Road::CalculatePathShape(struct Path *p)
{
    int swpIndex = wpId2Index.indexOf( p->startWpId );
    if( swpIndex < 0 ){
        qDebug() << "[CalculatePathShape:PathID=" << p->id << "]: cannot find startWP ID = " << p->startWpId << " in wpId2Index";
        return;
    }

    int ewpIndex = wpId2Index.indexOf( p->endWpId );
    if( ewpIndex < 0 ){
        qDebug() << "[CalculatePathShape:PathID=" << p->id << "]: cannot find endWpId ID = " << p->endWpId << " in wpId2Index";
        return;
    }

    float sX   = wps[swpIndex]->pos.x();
    float sY   = wps[swpIndex]->pos.y();
    float sZ   = wps[swpIndex]->pos.z();
    float sTht = wps[swpIndex]->direct;
    float sCt  = cos( sTht );
    float sSt  = sin( sTht );

    float eX   = wps[ewpIndex]->pos.x();
    float eY   = wps[ewpIndex]->pos.y();
    float eZ   = wps[ewpIndex]->pos.z();
    float eTht = wps[ewpIndex]->direct;
    float eCt  = cos( eTht );
    float eSt  = sin( eTht );

    float dx = eX - sX;
    float dy = eY - sY;
    float L = sqrt( dx * dx + dy * dy );
    if( L < 1.0 ){
        L = 1.0;
    }
    float D = 1.0 + 0.707 * ( L - 1.0 );

    float px[4],py[4];
    px[0] = -(eCt) * D;
    py[0] = -(eSt) * D;
    px[1] =  (sCt) * D;
    py[1] =  (sSt) * D;
    px[2] = -px[0];
    py[2] = -py[0];
    px[3] =  dx - px[1];
    py[3] =  dy - py[1];


    QVector3D *point = new QVector3D( sX, sY, sZ );
    p->pos.append( point );

    QVector2D *deriv = new QVector2D( sCt, sSt );
    p->derivative.append( deriv );

    p->length.append( 0.0 );
    p->pathLength = 0.0;

    if( p->numDivPath > 0 ){

        for(int i=1;i<=p->numDivPath;++i){

            float tht = 3.141592653 * 0.5 * (float)i / (float)(p->numDivPath + 1);
            float CDiv = cos(tht);
            float SDiv = sin(tht);
            float CDiv2 = CDiv * CDiv;
            float SDiv2 = SDiv * SDiv;

            float xp = sX + (px[0] * CDiv + px[1] * SDiv + px[2] * CDiv2 + px[3] * SDiv2);
            float yp = sY + (py[0] * CDiv + py[1] * SDiv + py[2] * CDiv2 + py[3] * SDiv2);
            float zp = sZ + ( eZ - sZ ) * (float)i / (float)(p->numDivPath + 1);

            float dXp = -px[0] * SDiv + px[1] * CDiv + 2.0 * (px[3] - px[2]) * CDiv * SDiv;
            float dYp = -py[0] * SDiv + py[1] * CDiv + 2.0 * (py[3] - py[2]) * CDiv * SDiv;

            float den = sqrt( dXp * dXp + dYp * dYp );
//            if( fabs(den) < 0.01 ){
//                den = 1.0;
//            }
            float Dp = 1.0 / (den + 0.0001);
            dXp *= Dp;
            dYp *= Dp;

            point = new QVector3D( xp, yp, zp );
            p->pos.append( point );

            deriv = new QVector2D( dXp, dYp );
            p->derivative.append( deriv );

            int idx = p->pos.size() - 1;

            dx = p->pos[idx]->x() - p->pos[idx-1]->x();
            dy = p->pos[idx]->y() - p->pos[idx-1]->y();
            float segLen = sqrt( dx * dx + dy * dy );
            p->pathLength += segLen;
            p->length.append( p->pathLength );
        }
    }

    point = new QVector3D( eX, eY, eZ );
    p->pos.append( point );

    deriv = new QVector2D( eCt, eSt );
    p->derivative.append( deriv );

    int idx = p->pos.size() - 1;
    dx = p->pos[idx]->x() - p->pos[idx-1]->x();
    dy = p->pos[idx]->y() - p->pos[idx-1]->y();
    float segLen = sqrt( dx * dx + dy * dy );

    p->pathLength += segLen;
    p->length.append( p->pathLength );


    //
    // set (xmin,ymin) and (xmax,ymax), used to search the path
    //
    p->xmin = p->pos[0]->x();
    p->xmax = p->xmin;
    p->ymin = p->pos[0]->y();
    p->ymax = p->ymin;
    for(int i=1;i<p->pos.size();++i){
        float xt = p->pos[i]->x();
        float yt = p->pos[i]->y();
        if( p->xmin > xt ){
            p->xmin = xt;
        }
        else if( p->xmax < xt ){
            p->xmax = xt;
        }
        if( p->ymin > yt ){
            p->ymin = yt;
        }
        else if(p->ymax < yt ){
            p->ymax = yt;
        }
    }

    p->xmin -= 10.0;
    p->ymin -= 10.0;
    p->xmax += 10.0;
    p->ymax += 10.0;
}


void Road::CalculatePathCurvature(Path *p)
{
    if( !p ){
        qDebug() << "[Road::CalculatePathCurvature] pointer p is null.";
        return;
    }

    int nFollowingPath = p->followingPaths.size();
    if( nFollowingPath == 0 ){
        p->curvature.append( 0.0 );
    }
    else{
        float x1 = 0.0;
        float y1 = 0.0;
        for(int i=0;i<nFollowingPath;++i){
            int index = pathId2Index.indexOf( p->followingPaths[i] );
            x1 += paths[index]->pos.last()->x();
            y1 += paths[index]->pos.last()->y();
        }
        x1 /= nFollowingPath;
        y1 /= nFollowingPath;

        float x2 = p->pos.first()->x();
        float y2 = p->pos.first()->y();
        float x3 = p->pos.last()->x();
        float y3 = p->pos.last()->y();

        float curvature = CalculateCurvatureFromThreePoints(x1,y1,x2,y2,x3,y3);

        p->curvature.append( curvature );
    }


    for(int i=1;i<=p->numDivPath;++i){

        float x1 = p->pos[i-1]->x();
        float y1 = p->pos[i-1]->y();
        float x2 = p->pos[i]->x();
        float y2 = p->pos[i]->y();
        float x3 = p->pos[i+1]->x();
        float y3 = p->pos[i+1]->y();

        float curvature = CalculateCurvatureFromThreePoints(x1,y1,x2,y2,x3,y3);

        p->curvature.append( curvature );
    }


    int nForwardPath = p->forwardPaths.size();
    if( nForwardPath == 0 ){
        p->curvature.append( 0.0 );
    }
    else{
        float x3 = 0.0;
        float y3 = 0.0;
        for(int i=0;i<nForwardPath;++i){
            int index = pathId2Index.indexOf( p->forwardPaths[i] );
            x3 += paths[index]->pos.first()->x();
            y3 += paths[index]->pos.first()->y();
        }
        x3 /= nForwardPath;
        y3 /= nForwardPath;

        float x2 = p->pos.last()->x();
        float y2 = p->pos.last()->y();
        float x1 = p->pos.first()->x();
        float y1 = p->pos.first()->y();

        float curvature = CalculateCurvatureFromThreePoints(x1,y1,x2,y2,x3,y3);

        p->curvature.append( curvature );
    }

    p->meanPathCurvature = 0.0;
    p->maxPathCurvature = 0.0;
    for(int i=0;i<p->curvature.size();++i){
        p->meanPathCurvature += p->curvature.at(i);
        if( p->maxPathCurvature < fabs(p->curvature[i]) ){
            p->maxPathCurvature = fabs(p->curvature[i]);
        }
    }
    p->meanPathCurvature /= p->curvature.size();
}


float Road::CalculateCurvatureFromThreePoints(float x1, float y1, float x2, float y2, float x3, float y3)
{
    float curvature = 0.0;

    float xm1 = (x2 + x1) * 0.5;
    float ym1 = (y2 + y1) * 0.5;
    float xm2 = (x3 + x2) * 0.5;
    float ym2 = (y3 + y2) * 0.5;

    float dx1 = (x2 - x1);
    float dy1 = (y2 - y1);
    float dx2 = (x3 - x2);
    float dy2 = (y3 - y2);

    float det = dx2 * dy1 - dx1 * dy2;
    if( fabs(det) < 0.01 ){
        return curvature;
    }

    float l = dx2 * (xm1 - xm2) + dy2 * (ym1 - ym2);
    l /= det;

    float xc = xm1 - dy1 * l;
    float yc = ym1 + dx1 * l;

    float rx = x2 - xc;
    float ry = y2 - yc;
    float R = sqrt( rx*rx + ry*ry );

    curvature = 1.0 / R;
    if( det > 0.0 ){
        curvature *= -1.0;
    }

    return curvature;
}


float Road::GetPathLength(int pathID)
{
    int index = pathId2Index.indexOf(pathID);
    if( index < 0 ){
        return 0.0;
    }

    return paths[index]->pathLength;
}


int Road::GetTargetNodeOfPath(int pathID)
{
    int index = pathId2Index.indexOf(pathID);
    if( index < 0 ){
        return -1;
    }
    return paths[index]->connectingNode;
}


float Road::GetPathMeanCurvature(int pathID)
{
    int index = pathId2Index.indexOf(pathID);
    if( index < 0 ){
        return 0.0;
    }

    return paths[index]->meanPathCurvature;
}


void Road::SetPathShape(int id)
{
    int index = pathId2Index.indexOf(id);
    if( index < 0 ){
        qDebug() << "[SetPathShape:PathID=" << id << "]: cannot find Path ID = " << id << " in pathId2Index";
        return;
    }

    CalculatePathShape( paths[index] );
}


void Road::SetPathConnection()
{
    qDebug() << "[Road::SetPathConnection]";

    for(int i=0;i<paths.size();++i){

        int sWP = paths[i]->startWpId;
        int eWP = paths[i]->endWpId;
        for(int j=0;j<paths.size();++j){
            if( i == j ){
                continue;
            }
            if( paths[j]->endWpId == sWP ){
                paths[i]->followingPaths.append( paths[j]->id );
            }
            if( paths[j]->startWpId == eWP ){
                paths[i]->forwardPaths.append( paths[j]->id );
            }
        }
    }

    qDebug() << "path connection checked.";
}


void Road::SetPathRelatedNodes()
{
    qDebug() << "[Road::SetPathRelatedNodes]";

    for(int i=0;i<paths.size();++i){
        paths[i]->connectingNode = -1;
    }

    for(int i=0;i<nodes.size();++i){
        for(int j=0;j<nodes[i]->pathLists.size();++j){
            for(int k=0;k<nodes[i]->pathLists[j]->pathList.size();++k){
                for(int l=0;l<nodes[i]->pathLists[j]->pathList[k].size();++l){
                    int tPath = nodes[i]->pathLists[j]->pathList[k].at(l);
                    int pIdx = pathId2Index.indexOf( tPath );
                    paths[pIdx]->connectingNode = nodes[i]->id;
                    paths[pIdx]->connectingNodeInDir = nodes[i]->pathLists[j]->inDirect;
                }
            }
        }
    }

    for(int i=0;i<paths.size();++i){
        if( paths[i]->connectingNode == -1 ){
            qDebug() << "!!! path: id = " << paths[i]->id << " , connectingNode is not set. Check SEdit Data.";
        }
    }
}


void Road::CheckSideBoundaryWPs(struct Node *n)
{
    if( !n ){
        qDebug() << "[Road::CheckSideBoundaryWPs] pointer n is null.";
        return;
    }

    //qDebug() << "[Road::CheckSideBoundaryWPs]";

    for(int i=0;i<n->nCross;++i){

        QList<int> checkWPs;
        QList<int> tIndex;
        for(int j=0;j<n->inBoundaryWPs.size();++j){
            if( n->inBoundaryWPs[j]->relatedDirection == n->legIDs[i] ){
                checkWPs.append( n->inBoundaryWPs[j]->wpId );
                tIndex.append(j);
            }
        }
        if( checkWPs.size() > 0 ){
            QList<float> dist;
            dist.append(0);
            int wpIdx1 = wpId2Index.indexOf( checkWPs[0] );
            for(int j=1;j<checkWPs.size();++j){
                int wpIdx2 = wpId2Index.indexOf( checkWPs[j] );
                QVector3D diff_pos = wps[wpIdx2]->pos - wps[wpIdx1]->pos;
                float tmpDist = diff_pos.x() * wps[wpIdx1]->sinDirect + diff_pos.y() * wps[wpIdx1]->cosDirect * (-1.0);
                dist.append( tmpDist );
            }

            int laneNo = 0;
            while( dist.size() > 0 ){
                float minDist = dist[0];
                int minIdx = 0;
                for(int j=1;j<dist.size();++j){
                    if( minDist > dist[j] ){
                        minDist = dist[j];
                        minIdx = j;
                    }
                }

                n->inBoundaryWPs[ tIndex[minIdx] ]->laneNo = laneNo;

                laneNo++;
                dist.removeAt( minIdx );
                tIndex.removeAt( minIdx );
            }

            for( laneNo = 0; laneNo < tIndex.size(); ++laneNo ){
                for(int j=0;j<tIndex.size();++j){
                    if( n->inBoundaryWPs[tIndex[j]]->laneNo == laneNo ){
                        for(int k=0;k<tIndex.size();++k){
                            if(j == k){
                                continue;
                            }
                            if( n->inBoundaryWPs[tIndex[k]]->laneNo == laneNo + 1 ){
                                n->inBoundaryWPs[tIndex[j]]->rightWpId = n->inBoundaryWPs[tIndex[k]]->wpId;
                            }
                            else if( n->inBoundaryWPs[tIndex[k]]->laneNo == laneNo - 1 ){
                                n->inBoundaryWPs[tIndex[j]]->leftWpId = n->inBoundaryWPs[tIndex[k]]->wpId;
                            }
                        }
                        break;
                    }
                }
            }
        }
    }

    //qDebug() << "inBoundaryWPs set.";

    for(int i=0;i<n->nCross;++i){

        QList<int> checkWPs;
        QList<int> tIndex;
        for(int j=0;j<n->outBoundaryWPs.size();++j){
            if( n->outBoundaryWPs[j]->relatedDirection == n->legIDs[i] ){
                checkWPs.append( n->outBoundaryWPs[j]->wpId );
                tIndex.append(j);
            }
        }
        if( checkWPs.size() > 0 ){
            QList<float> dist;
            dist.append(0);
            int wpIdx1 = wpId2Index.indexOf( checkWPs[0] );
            for(int j=1;j<checkWPs.size();++j){
                int wpIdx2 = wpId2Index.indexOf( checkWPs[j] );
                QVector3D diff_pos = wps[wpIdx2]->pos - wps[wpIdx1]->pos;
                float tmpDist = diff_pos.x() * wps[wpIdx1]->sinDirect + diff_pos.y() * wps[wpIdx1]->cosDirect * (-1.0);
                dist.append( tmpDist );
            }

            int laneNo = 0;
            while( dist.size() > 0 ){
                float minDist = dist[0];
                int minIdx = 0;
                for(int j=1;j<dist.size();++j){
                    if( minDist > dist[j] ){
                        minDist = dist[j];
                        minIdx = j;
                    }
                }

                n->outBoundaryWPs[ tIndex[minIdx] ]->laneNo = laneNo;

                laneNo++;
                dist.removeAt( minIdx );
                tIndex.removeAt( minIdx );
            }

            for( laneNo = 0; laneNo < tIndex.size(); ++laneNo ){
                for(int j=0;j<tIndex.size();++j){
                    if( n->outBoundaryWPs[tIndex[j]]->laneNo == laneNo ){
                        for(int k=0;k<tIndex.size();++k){
                            if(j == k){
                                continue;
                            }
                            if( n->outBoundaryWPs[tIndex[k]]->laneNo == laneNo + 1 ){
                                n->outBoundaryWPs[tIndex[j]]->rightWpId = n->outBoundaryWPs[tIndex[k]]->wpId;
                            }
                            else if( n->outBoundaryWPs[tIndex[k]]->laneNo == laneNo - 1 ){
                                n->outBoundaryWPs[tIndex[j]]->leftWpId = n->outBoundaryWPs[tIndex[k]]->wpId;
                            }
                        }
                        break;
                    }
                }
            }
        }
    }

    //qDebug() << "outBoundaryWPs set.";

    for(int i=0;i<n->inBoundaryWPs.size();++i){

        int wpId = n->inBoundaryWPs[i]->wpId;
        for(int j=0;j<paths.size();++j){
            if( paths[j]->startWpId == wpId ){
                n->inBoundaryWPs[i]->PathWithSWP.append( paths[j]->id );
            }
            if( paths[j]->endWpId == wpId ){
                n->inBoundaryWPs[i]->PathWithEWP.append( paths[j]->id );
            }
        }

        int wpIdx = wpId2Index.indexOf( wpId );
        wps[wpIdx]->isNodeInWP = true;
        wps[wpIdx]->relatedNode = n->id;
        wps[wpIdx]->relatedNodeLeg = n->inBoundaryWPs[i]->relatedDirection;
    }

    for(int i=0;i<n->outBoundaryWPs.size();++i){

        int wpId = n->outBoundaryWPs[i]->wpId;
        for(int j=0;j<paths.size();++j){
            if( paths[j]->startWpId == wpId ){
                n->outBoundaryWPs[i]->PathWithSWP.append( paths[j]->id );
            }
            if( paths[j]->endWpId == wpId ){
                n->outBoundaryWPs[i]->PathWithEWP.append( paths[j]->id );
            }
        }

        int wpIdx = wpId2Index.indexOf( wpId );
        wps[wpIdx]->isNodeOutWP = true;
        wps[wpIdx]->relatedNode = n->id;
        wps[wpIdx]->relatedNodeLeg = n->outBoundaryWPs[i]->relatedDirection;
    }

    //qDebug() << "related data set.";
}


int Road::GetDirectionLabel(int nodeID, int inDir, int checkDir)
{
    int ret = DIRECTION_LABEL::UNDEFINED_DIRECTION;

    int ndIdx = nodeId2Index.indexOf( nodeID );
    if( ndIdx < 0 ){
        return ret;
    }

    for(int i=0;i<nodes[ndIdx]->directionMap.size();++i){

        if( nodes[ndIdx]->directionMap[i]->inDirect == inDir ){

            if( nodes[ndIdx]->directionMap[i]->oncomingDirect == checkDir ){
                ret = DIRECTION_LABEL::ONCOMING;
            }
            else if( nodes[ndIdx]->directionMap[i]->leftDirect.size() > 0 && nodes[ndIdx]->directionMap[i]->leftDirect.indexOf(checkDir) >= 0 ){
                ret = DIRECTION_LABEL::LEFT_CROSSING;
            }
            else if( nodes[ndIdx]->directionMap[i]->rightDirect.size() > 0 && nodes[ndIdx]->directionMap[i]->rightDirect.indexOf(checkDir) >= 0 ){
                ret = DIRECTION_LABEL::RIGHT_CROSSING;
            }

            break;
        }
    }

    return ret;
}


int Road::CreateWPforScenarioObject(float x, float y, float z, float direct, int objectID, int order,float speedInfo)
{
//    qDebug() << "[Road::CreateWPforScenarioObject]";
//    qDebug() << "   P(" << x << "," << y << "," << z << ")  Tht=" << direct * 57.3
//             << ", ID = " << objectID << " order = " << order << " V = " << speedInfo * 3.6;

    struct WP* wp = new struct WP;

    int maxID = 0;
    for(int i=0;i<wps.size();++i){
        if( maxID <= wps[i]->id ){
            maxID = wps[i]->id + 1;
        }
    }
    wp->id = maxID;
    wp->pos.setX( x );
    wp->pos.setY( y );
    wp->pos.setZ( z );
    wp->direct = direct;

    wp->scenarioObjectID = objectID;
    wp->order            = order;
    wp->speedInfo        = speedInfo;

    wps.append( wp );

    wpId2Index.append( maxID );

//    qDebug() << " WP ID = " << maxID << "  index = " << wpId2Index.indexOf( maxID );

    return maxID;
}


void Road::CreatePathsforScenarioObject(int objectID)
{
//    qDebug() << "[Road::CreatePathsforScenarioObject]";
//    qDebug() << "    objectID = " << objectID;


    QList<int> wpList;
    for(int i=0;i<wps.size();++i){
        if( wps[i]->scenarioObjectID == objectID ){
            wpList.append( i );
        }
    }

//    for(int i=0;i<wpList.size();++i){
//        qDebug() << "  wpList[" << i << "] ,  ID = " << wps[wpList[i]]->id;
//    }

    int order = 0;
    for(int i=0;i<wpList.size()-1;++i){
        int sWP = -1;
        int eWP = -1;
        for(int j=0;j<wpList.size();++j){
            if( wps[wpList[j]]->order == order ){
                sWP = wpList[j];
            }
            if( wps[wpList[j]]->order == order + 1 ){
                eWP = wpList[j];
            }
        }
        if( sWP < 0 || eWP < 0 ){
            continue;
        }

//        qDebug() << "  create path : " << wps[sWP]->id << " -> " << wps[eWP]->id;

        struct Path* path = new struct Path;

        int maxID = 0;
        for(int j=0;j<paths.size();++j){
            if( maxID <= paths[j]->id ){
                maxID = paths[j]->id + 1;
            }
        }
        path->id = maxID;
        path->startWpId  = wps[sWP]->id;
        path->endWpId    = wps[eWP]->id;
        path->numDivPath = 9;
        path->speedInfo  = (wps[sWP]->speedInfo + wps[eWP]->speedInfo) * 0.5;
        path->speed85pt  = path->speedInfo;

        path->scenarioObjectID = objectID;

        path->setSpeedVariationParam = false;
        path->vDevP = 0.0;
        path->vDevM = 0.0;
        path->refVforDev = 0.0;
        path->accelAtDev = 0.0;
        path->decelAtDev = 0.0;

        paths.append( path );

        pathId2Index.append( maxID );

        SetPathShape( maxID );

        CalculatePathCurvature( path );

        order++;
    }
}


int Road::CreatePedestPathsforScenarioObject(int objectID,
                                              float x1,float y1,float z1,
                                              float x2,float y2,float z2,
                                              float w,
                                              bool isCrossWalk,
                                              int roadSideInfo)
{
    qDebug() << "[Road::CreatePedestPathsforScenarioObject]";
    qDebug() << "    objectID = " << objectID;

    int maxID = 0;

    bool createNew = true;
    for(int i=0;i<pedestPaths.size();++i){
        if( pedestPaths[i]->scenarioObjectID == objectID ){
            createNew = false;
            maxID = i;
            break;
        }
    }

    if( createNew == true ){

        struct PedestPath* ppath = new struct PedestPath;
        maxID = 0;
        for(int j=0;j<pedestPaths.size();++j){
            if( maxID <= pedestPaths[j]->id ){
                maxID = pedestPaths[j]->id + 1;
            }
        }
        ppath->id = maxID;
        ppath->scenarioObjectID = objectID;

        ppath->meanArrivalTime = -1.0;

        pedestPaths.append( ppath );
        pedestPathID2Index.append( maxID );

        struct PedestPathShapeInfo *si1 = new struct PedestPathShapeInfo;
        si1->pos.setX( x1 );
        si1->pos.setY( y1 );
        si1->pos.setZ( z1 );

        si1->width = w;
        si1->isCrossWalk = isCrossWalk;
        si1->runOutProb = 0.0;
        si1->runOutDirect = roadSideInfo;
        si1->angleToNextPos = 0.0;
        si1->distanceToNextPos = 0.0;

        ppath->shape.append( si1 );

        struct PedestPathShapeInfo *si2 = new struct PedestPathShapeInfo;
        si2->pos.setX( x2 );
        si2->pos.setY( y2 );
        si2->pos.setZ( z2 );

        si2->width = 0.0;
        si2->isCrossWalk = false;
        si2->runOutProb = 0.0;
        si2->runOutDirect = 0;
        si2->angleToNextPos = 0.0;
        si2->distanceToNextPos = 0.0;

        ppath->shape.append( si2 );

    }
    else{

        pedestPaths[maxID]->shape.last()->width = w;
        pedestPaths[maxID]->shape.last()->isCrossWalk = isCrossWalk;
        pedestPaths[maxID]->shape.last()->runOutProb = 0.0;
        pedestPaths[maxID]->shape.last()->runOutDirect = roadSideInfo;

        struct PedestPathShapeInfo *si2 = new struct PedestPathShapeInfo;
        si2->pos.setX( x2 );
        si2->pos.setY( y2 );
        si2->pos.setZ( z2 );

        si2->width = 0.0;
        si2->isCrossWalk = false;
        si2->runOutProb = 0.0;
        si2->runOutDirect = 0;
        si2->angleToNextPos = 0.0;
        si2->distanceToNextPos = 0.0;

        pedestPaths[maxID]->shape.append( si2 );

        maxID = pedestPaths[maxID]->id;
    }

    return maxID;
}


float Road::GetPedestPathWidth(int ppID,int sect)
{
    float w = 1.0;
    int ppIdx = pedestPathID2Index.indexOf( ppID );
    if( ppIdx < 0 ){
        return w;
    }
    if( sect >= 0 && sect < pedestPaths[ppIdx]->shape.size() ){
        w = pedestPaths[ppIdx]->shape[sect]->width;
    }
    return w;
}


void Road::CheckPedestPathConnection()
{
    qDebug() << "[Road::CheckPedestPathConnection]";
    qDebug() << "    nPedestPath = " << pedestPaths.size();

    for(int i=0;i<pedestPaths.size();++i){
        if( pedestPaths[i]->scenarioObjectID >= 0 ){
            continue;
        }

        int j = pedestPaths[i]->shape.size() - 1;
        pedestPaths[i]->shape[j]->angleToNextPos = pedestPaths[i]->shape[j-1]->angleToNextPos;
        pedestPaths[i]->shape[j]->cosA = pedestPaths[i]->shape[j-1]->cosA;
        pedestPaths[i]->shape[j]->sinA = pedestPaths[i]->shape[j-1]->sinA;
        pedestPaths[i]->shape[j]->distanceToNextPos = 100.0;
    }


    for(int i=0;i<pedestPaths.size();++i){
        if( pedestPaths[i]->scenarioObjectID < 0 ){
            continue;
        }

        for(int j=0;j<pedestPaths[i]->shape.size()-1;++j){

            float dx = pedestPaths[i]->shape[j+1]->pos.x() - pedestPaths[i]->shape[j]->pos.x();
            float dy = pedestPaths[i]->shape[j+1]->pos.y() - pedestPaths[i]->shape[j]->pos.y();

            pedestPaths[i]->shape[j]->angleToNextPos = atan2( dy, dx );
            pedestPaths[i]->shape[j]->cosA = cos( pedestPaths[i]->shape[j]->angleToNextPos );
            pedestPaths[i]->shape[j]->sinA = sin( pedestPaths[i]->shape[j]->angleToNextPos );

            pedestPaths[i]->shape[j]->distanceToNextPos = sqrt( dx * dx + dy * dy );

        }

        int j = pedestPaths[i]->shape.size() - 1;
        pedestPaths[i]->shape[j]->angleToNextPos = pedestPaths[i]->shape[j-1]->angleToNextPos;
        pedestPaths[i]->shape[j]->cosA = pedestPaths[i]->shape[j-1]->cosA;
        pedestPaths[i]->shape[j]->sinA = pedestPaths[i]->shape[j-1]->sinA;
        pedestPaths[i]->shape[j]->distanceToNextPos = 100.0;
    }
}


void Road::SetPedestPathArrivalTimes()
{
    qDebug() << "[Road::SetPedestPathArrivalTimes]";

    for(int i=0;i<pedestPaths.size();++i){
        if( pedestPaths[i]->scenarioObjectID >= 0 ){
            pedestPaths[i]->meanArrivalTime = -1.0;
            continue;
        }

        pedestPaths[i]->totalVolume = 0;
        for(int j=0;j<pedestPaths[i]->trafficVolume.size();++j){
            pedestPaths[i]->totalVolume += pedestPaths[i]->trafficVolume[j];
        }

        qDebug() << "pedestPath: id=" << pedestPaths[i]->id << " totalVolume = " << pedestPaths[i]->totalVolume;

        if( pedestPaths[i]->totalVolume > 0 ){

            pedestPaths[i]->pedestKindSelectProbability.clear();
            for(int j=0;j<pedestPaths[i]->trafficVolume.size();++j){
                float p = (float)pedestPaths[i]->trafficVolume[j] / (float)pedestPaths[i]->totalVolume;
                pedestPaths[i]->pedestKindSelectProbability.append( p );
            }

            pedestPaths[i]->meanArrivalTime = 3600.0 / pedestPaths[i]->totalVolume;
            pedestPaths[i]->NextAppearTime = -1.0;   // This value is updated at simulationmangaer.cpp

        }
        else {
            pedestPaths[i]->meanArrivalTime = -1.0;
        }
    }
}



int Road::RandomSelect(int N, float rnd)
{
    if( N <= 1 ){
        return 0;
    }
    float D = 1.0 / (float)N;
    int selected = 0;
    for(int i=0;i<N-1;++i){
        if( rnd >= i * D && rnd < (i+1) * D ){
            selected = i;
            break;
        }
    }
    return selected;
}

