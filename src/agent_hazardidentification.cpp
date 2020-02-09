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


void Agent::HazardIdentification( Agent** pAgent, int maxAgent, Road* pRoad )
{
    if( memory.controlMode == AGENT_CONTROL_MODE::AGENT_LOGIC ){

        // Collision or Mergining Point Check
        for(int i=0;i<memory.perceptedObjects.size();++i){

            if( memory.perceptedObjects[i]->recognitionLabel < AGENT_RECOGNITION_LABEL::ONCOMING_STRAIGHT ){
                memory.perceptedObjects[i]->hasCollisionPoint = false;
                continue;
            }

            memory.perceptedObjects[i]->hasCollisionPoint = false;

            int objID = memory.perceptedObjects[i]->objectID;
            for(int j=memory.currentTargetPathIndexInList;j>=0;j--){
                int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
                if( pRoad->paths[pIdx]->crossPoints.size() == 0 ){
                    continue;
                }
                for(int k=0;k<pRoad->paths[pIdx]->crossPoints.size();++k){
                    int cpath = pRoad->paths[pIdx]->crossPoints[k]->crossPathID;
                    if( pAgent[objID]->memory.targetPathList.indexOf( cpath ) >= 0 ){

                        memory.perceptedObjects[i]->hasCollisionPoint = true;
                        memory.perceptedObjects[i]->xCP = pRoad->paths[pIdx]->crossPoints[k]->pos.x();
                        memory.perceptedObjects[i]->yCP = pRoad->paths[pIdx]->crossPoints[k]->pos.y();
                        memory.perceptedObjects[i]->CPinNode = pRoad->paths[pIdx]->connectingNode;

                        float myDist = 0.0;
                        for(int l=memory.currentTargetPathIndexInList;l>=0;l--){
                            if( l == j ){
                                myDist += pRoad->paths[pIdx]->crossPoints[k]->distFromStartWP;
                                break;
                            }
                            int tpIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[l] );
                            myDist += pRoad->paths[tpIdx]->pathLength;
                        }
                        myDist -= memory.distanceFromStartWPInCurrentPath;

                        memory.perceptedObjects[i]->myDistanceToCP = myDist;
                        memory.perceptedObjects[i]->myTimeToCP = myDist / (state.V + 0.5);

                        float objDist = 0.0;
                        int mlIdx = pAgent[objID]->memory.targetPathList.size()-1;
                        for(int l=pAgent[objID]->memory.currentTargetPathIndexInList;l>=0;l--){
                            if( pAgent[objID]->memory.targetPathList[l] == cpath ){
                                mlIdx = l;
                                break;
                            }
                        }
                        for(int l=pAgent[objID]->memory.currentTargetPathIndexInList;l>=mlIdx;l--){
                            if( pAgent[objID]->memory.targetPathList[l] == cpath ){
                                int tpIdx = pRoad->pathId2Index.indexOf( cpath );
                                for(int m=0;m<pRoad->paths[tpIdx]->crossPoints.size();++m){
                                    if( pRoad->paths[tpIdx]->crossPoints[m]->crossPathID == memory.targetPathList[j] ){
                                        objDist += pRoad->paths[tpIdx]->crossPoints[m]->distFromStartWP;
                                        break;
                                    }
                                }
                                break;
                            }
                            int tpIdx = pRoad->pathId2Index.indexOf( pAgent[objID]->memory.targetPathList[l] );
                            objDist += pRoad->paths[tpIdx]->pathLength;
                        }
                        objDist -= pAgent[objID]->memory.distanceFromStartWPInCurrentPath;

                        memory.perceptedObjects[i]->objectDistanceToCP = objDist;
                        memory.perceptedObjects[i]->objectTimeToCP = objDist / ( memory.perceptedObjects[i]->V + 0.5);

                        break;
                    }
                }
                if( memory.perceptedObjects[i]->hasCollisionPoint == true ){
                    break;
                }
            }

            if( memory.perceptedObjects[i]->hasCollisionPoint == true ){
                continue;
            }

            QList<int> myEWPList;
            for(int j=memory.currentTargetPathIndexInList;j>=0;j--){
                int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
                myEWPList.append( pRoad->paths[pIdx]->endWpId );
            }

            QList<int> objEWPList;
            for(int j=pAgent[objID]->memory.currentTargetPathIndexInList;j>=0;j--){
                int pIdx = pRoad->pathId2Index.indexOf( pAgent[objID]->memory.targetPathList[j] );
                objEWPList.append( pRoad->paths[pIdx]->endWpId );
            }

            for(int j=0;j<myEWPList.size();++j){
                for(int k=0;k<objEWPList.size();++k){
                    if( myEWPList[j] == objEWPList[k] ){

                        memory.perceptedObjects[i]->hasCollisionPoint = true;

                        int wIdx = pRoad->wpId2Index.indexOf( myEWPList[j] );
                        memory.perceptedObjects[i]->xCP = pRoad->wps[wIdx]->pos.x();
                        memory.perceptedObjects[i]->yCP = pRoad->wps[wIdx]->pos.y();
                        memory.perceptedObjects[i]->CPinNode = pRoad->wps[wIdx]->relatedNode;

                        float myDist = 0.0;
                        for(int l=memory.currentTargetPathIndexInList;l>=0;l--){
                            int tpIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[l] );
                            myDist += pRoad->paths[tpIdx]->pathLength;
                            if( pRoad->paths[tpIdx]->endWpId == myEWPList[j] ){
                                break;
                            }
                        }
                        myDist -= memory.distanceFromStartWPInCurrentPath;

                        memory.perceptedObjects[i]->myDistanceToCP = myDist;
                        memory.perceptedObjects[i]->myTimeToCP = myDist / (state.V + 0.5);

                        float objDist = 0.0;
                        int mlIdx = pAgent[objID]->memory.targetPathList.size() - 1;
                        for(int l=pAgent[objID]->memory.currentTargetPathIndexInList;l>=0;l--){
                            int tpIdx = pRoad->pathId2Index.indexOf( pAgent[objID]->memory.targetPathList[l] );
                            if( pRoad->paths[tpIdx]->endWpId == objEWPList[k] ){
                                mlIdx = l;
                                break;
                            }
                        }
                        for(int l=pAgent[objID]->memory.currentTargetPathIndexInList;l>=mlIdx;l--){
                            int tpIdx = pRoad->pathId2Index.indexOf( pAgent[objID]->memory.targetPathList[l] );
                            objDist += pRoad->paths[tpIdx]->pathLength;
                            if( pRoad->paths[tpIdx]->endWpId == objEWPList[k] ){
                                break;
                            }
                        }
                        objDist -= pAgent[objID]->memory.distanceFromStartWPInCurrentPath;

                        memory.perceptedObjects[i]->objectDistanceToCP = objDist;
                        memory.perceptedObjects[i]->objectTimeToCP = objDist / ( memory.perceptedObjects[i]->V + 0.5);

                        break;
                    }
                }
                if( memory.perceptedObjects[i]->hasCollisionPoint == true ){
                    break;
                }
            }
        }


        // Distance to nearest CP
        memory.distToNearestCP = -1.0;
        bool existCPAhead = false;
        bool checkNextTurnNode = false;
        for(int j=memory.currentTargetPathIndexInList;j>=0;j--){
            int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
            if( memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING || memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
                if( checkNextTurnNode == false && pRoad->paths[pIdx]->connectingNode == memory.nextTurnNode ){
                    checkNextTurnNode = true;
                }
                else if( checkNextTurnNode == true && pRoad->paths[pIdx]->connectingNode != memory.nextTurnNode ){
                    break;
                }
            }
            if( pRoad->paths[pIdx]->crossPoints.size() > 0 ){
                existCPAhead = true;
                break;
            }
        }
        if( existCPAhead == true ){

            memory.distToNearestCP = 0.0;
            for(int j=memory.currentTargetPathIndexInList;j>=0;j--){
                int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
                if( pRoad->paths[pIdx]->crossPoints.size() == 0 ){
                    memory.distToNearestCP += pRoad->paths[pIdx]->pathLength;
                }
                else{
                    float dist = -1.0;
                    for(int k=0;k<pRoad->paths[pIdx]->crossPoints.size();++k){
                        if( dist < 0.0 || dist > pRoad->paths[pIdx]->crossPoints[k]->distFromStartWP ){
                            dist = pRoad->paths[pIdx]->crossPoints[k]->distFromStartWP;
                        }
                    }
                    memory.distToNearestCP += dist;
                    memory.distToNearestCP -= memory.distanceFromStartWPInCurrentPath;
                    break;
                }
            }
        }
        else{
            if( memory.nextTurnNode == memory.currentTargetNode && memory.nextTurnDirection == DIRECTION_LABEL::LEFT_CROSSING && pRoad->LeftOrRight == 0 ){

                memory.distToNearestCP = 0.0;
                for(int j=memory.currentTargetPathIndexInList;j>=0;j--){
                    int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
                    memory.distToNearestCP += pRoad->paths[pIdx]->pathLength;

                    int epIdx = pRoad->wpId2Index.indexOf( pRoad->paths[pIdx]->endWpId );
                    if( pRoad->wps[epIdx]->isNodeOutWP == true ){
                        memory.distToNearestCP -= memory.distanceFromStartWPInCurrentPath;
                        break;
                    }
                }
            }
            else if( memory.nextTurnNode == memory.currentTargetNode && memory.nextTurnDirection == DIRECTION_LABEL::RIGHT_CROSSING && pRoad->LeftOrRight == 1 ){

                memory.distToNearestCP = 0.0;
                for(int j=memory.currentTargetPathIndexInList;j>=0;j--){
                    int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[j] );
                    memory.distToNearestCP += pRoad->paths[pIdx]->pathLength;

                    int epIdx = pRoad->wpId2Index.indexOf( pRoad->paths[pIdx]->endWpId );
                    if( pRoad->wps[epIdx]->isNodeOutWP == true ){
                        memory.distToNearestCP -= memory.distanceFromStartWPInCurrentPath;
                        break;
                    }
                }
            }
        }

        // check to have to yeild the way
        memory.shouldYeild = false;
        memory.distToYeildStopLine = 0.0;

        for(int i=memory.currentTargetPathIndexInList;i>=0;i--){

            int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[i] );
            if( pRoad->paths[pIdx]->stopPoints.size() == 0 ){
                memory.distToYeildStopLine += pRoad->paths[pIdx]->pathLength;
                continue;
            }
            else{

                int nIdx = pRoad->nodeId2Index.indexOf( pRoad->paths[pIdx]->connectingNode );
                if( pRoad->nodes[nIdx]->hasTS == true ){
                    memory.distToYeildStopLine += pRoad->paths[pIdx]->pathLength;
                    continue;
                }

                memory.shouldYeild = true;

                float dist = -1.0;
                for(int j=0;j<pRoad->paths[pIdx]->stopPoints.size();++j){
                    if( dist < 0.0 || dist > pRoad->paths[pIdx]->stopPoints[j]->distFromStartWP ){
                        dist = pRoad->paths[pIdx]->stopPoints[j]->distFromStartWP;
                    }
                }
                memory.distToYeildStopLine += dist;
                memory.distToYeildStopLine -= memory.distanceFromStartWPInCurrentPath;

                break;
            }
        }

        float tmpDist = memory.distanceFromStartWPInCurrentPath * (-1.0);
        bool backCheck = false;
        for(int i=memory.currentTargetPathIndexInList+1;i<memory.targetPathList.size();i++){

            int pIdx = pRoad->pathId2Index.indexOf( memory.targetPathList[i] );
            if( pRoad->paths[pIdx]->stopPoints.size() == 0 ){
                tmpDist -= pRoad->paths[pIdx]->pathLength;
                continue;
            }
            else{
                int nIdx = pRoad->nodeId2Index.indexOf( pRoad->paths[pIdx]->connectingNode );
                if( pRoad->nodes[nIdx]->hasTS == true ){
                    tmpDist -= pRoad->paths[pIdx]->pathLength;
                    continue;
                }

                if( pRoad->paths[pIdx]->connectingNode == memory.currentTargetNode ){
                    backCheck = true;
                    tmpDist -= pRoad->paths[pIdx]->pathLength;

                    float dist = -1.0;
                    for(int j=0;j<pRoad->paths[pIdx]->stopPoints.size();++j){
                        if( dist < pRoad->paths[pIdx]->stopPoints[j]->distFromStartWP ){
                            dist = pRoad->paths[pIdx]->stopPoints[j]->distFromStartWP;
                        }
                    }
                    tmpDist += dist;

                    break;
                }
            }
        }
        if( backCheck == true ){
            memory.shouldYeild = true;
            memory.distToYeildStopLine = tmpDist;
        }
    }


}

