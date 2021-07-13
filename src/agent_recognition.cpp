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

#include <QDebug>

#ifdef _PERFORMANCE_CHECK_AGENT_RECOGNITION
#include <windows.h>
#endif


void Agent::Recognition( Agent** pAgent, int maxAgent, Road* pRoad )
{

    if( cognitionCount != 0 ){
        return;
    }


    if( memory.controlMode == AGENT_CONTROL_MODE::AGENT_LOGIC && memory.myNodeList.size() > 0 ){

#ifdef _PERFORMANCE_CHECK_AGENT_RECOGNITION
        LARGE_INTEGER start4;
        QueryPerformanceCounter(&start4);
#endif


        if( onlyCheckPreceding == true ){
            return;
        }


        for(int i=0;i<memory.perceptedObjects.size();++i){

            if( memory.perceptedObjects[i]->isValidData == false ){
                continue;
            }

            if( memory.perceptedObjects[i]->objectType >= 100 ){
                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::PEDESTRIAN;
                continue;
            }
            else if( agentKind >= 100 && memory.perceptedObjects[i]->objectType < 100 ){
                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::UNDEFINED_RECOGNITION_LABEL;
                continue;
            }

            if( memory.perceptedObjects[i]->relPosEvaled == false ){
                continue;
            }

            if( memory.perceptedObjects[i]->filteredV < 0.1 && state.V < 0.1 &&
                    memory.perceptedObjects[i]->recognitionLabel != AGENT_RECOGNITION_LABEL::PRECEDING ){
                continue;
            }

            memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::UNDEFINED_RECOGNITION_LABEL;

            if( agentKind >= 100 ){
                continue;
            }


            int objPath = memory.perceptedObjects[i]->objectPath;
            if( objPath < 0 ){
                continue;
            }

            bool isSameLane = false;
            int sameLaneIndex = memory.targetPathList.indexOf( objPath );
            if( sameLaneIndex >= 0 ){
                isSameLane = true;
            }


            // S-Interface vehicle outside road-lane is ignored
            if( isSameLane == true &&
                    memory.perceptedObjects[i]->objectID >= 0 &&
                    memory.perceptedObjects[i]->objectID < maxAgent &&
                    pAgent[memory.perceptedObjects[i]->objectID]->isSInterfaceObject == true &&
                    fabs( memory.perceptedObjects[i]->deviationFromNearestTargetPath ) > 3.0 ){
                continue;
            }


            if( isSameLane == true ){

                if( sameLaneIndex == memory.currentTargetPathIndexInList ){
                    if( memory.perceptedObjects[i]->distanceToObject > vHalfLength ){
                        memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::PRECEDING;
                        memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                        memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                        strForDebug += QString("V%1-P1\n").arg(memory.perceptedObjects[i]->objectID);
                        continue;
                    }
                    else{
                        memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::FOLLOWING;
                        memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                        memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                        continue;
                    }
                }
                else if( sameLaneIndex < memory.currentTargetPathIndexInList && memory.perceptedObjects[i]->distanceToObject > vHalfLength ){
                    memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::PRECEDING;
                    memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                    memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                    strForDebug += QString("V%1-P2\n").arg(memory.perceptedObjects[i]->objectID);
                    continue;
                }
                else{
                    memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::FOLLOWING;
                    memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                    memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                    continue;
                }

            }
            else{

                int opIdx         = pRoad->pathId2Index.indexOf( objPath );
                int objTargetNode = pRoad->paths[opIdx]->connectingNode;
                int objInDirect   = pRoad->paths[opIdx]->connectingNodeInDir;

                bool sameInDirect = false;
                int myNLIdx = memory.myNodeList.indexOf( objTargetNode );
                if( myNLIdx >= 0 && memory.myInDirList.at(myNLIdx) == objInDirect ){
                    sameInDirect = true;
                }

                if( sameInDirect == true ){

#ifdef _PERFORMANCE_CHECK_AGENT_RECOGNITION
                    QueryPerformanceCounter(&start);
#endif

                    // SIDE Vehicles
                    if( fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath - memory.lateralDeviationFromTargetPath) >=
                            memory.perceptedObjects[i]->effectiveHalfWidth + vHalfWidth + 0.20 ){

                        if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 0.0 ){
                            if( memory.perceptedObjects[i]->distanceToObject > vHalfLength + memory.perceptedObjects[i]->vHalfLength ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_SIDE_PRECEDING;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                            else if( memory.perceptedObjects[i]->distanceToObject < -vHalfLength - memory.perceptedObjects[i]->vHalfLength ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_SIDE_FOLLOWING;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                            else{
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_SIDE;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                        }
                        else if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < 0.0 ){
                            if( memory.perceptedObjects[i]->distanceToObject > vHalfLength + memory.perceptedObjects[i]->vHalfLength ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_SIDE_PRECEDING;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;

                            }
                            else if( memory.perceptedObjects[i]->distanceToObject < -vHalfLength - memory.perceptedObjects[i]->vHalfLength ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_SIDE_FOLLOWING;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                            else{
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_SIDE;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                        }
                    }
                    else{

                        float relV = memory.perceptedObjects[i]->V - state.V;
                        if( relV < 0.0 || (relV < 1.0 && state.V < 1.0) ){
                            if( memory.perceptedObjects[i]->distanceToObject > vHalfLength  ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::PRECEDING;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                strForDebug += QString("V%1-P3\n").arg(memory.perceptedObjects[i]->objectID);
                                continue;
                            }
                            else{

                                if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 0.0 ){
                                    memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_SIDE;
                                    memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                    memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                    continue;
                                }
                                else{
                                    memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_SIDE;
                                    memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                    memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                    continue;
                                }
                            }
                        }
                        else{
                            if( memory.perceptedObjects[i]->distanceToObject < -vHalfLength ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::FOLLOWING;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                            else{

                                if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 0.0 ){
                                    memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_SIDE;
                                    memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                    memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                    continue;
                                }
                                else{
                                    memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_SIDE;
                                    memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                    memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                    continue;
                                }
                            }
                        }

                    }

#ifdef _PERFORMANCE_CHECK_AGENT_RECOGNITION
                    QueryPerformanceCounter(&end);
                    calTime[0] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                    calCount[0]++;
#endif

                }
                else{

                    // Check if the object have the same target node
                    int aID                = memory.perceptedObjects[i]->objectID;
                    int crossNode          = -1;
                    int crossNodeMyInDir   = -1;
                    int crossNodeObjInDir  = -1;
                    int crossNodeObjOutDir = -1;


#ifdef _PERFORMANCE_CHECK_AGENT_RECOGNITION
                    QueryPerformanceCounter(&start);
#endif

                    if( memory.currentTargetNode == pAgent[aID]->memory_reference.currentTargetNode &&
                            memory.currentTargetNodeIndexInNodeList >= 0 &&
                            memory.currentTargetNodeIndexInNodeList < memory.myNodeList.size() ){

//                        strForDebug += QString("[1][Recog] V=%1\n").arg( aID );
//                        strForDebug += QString(" crossNode=%1\n").arg( memory.currentTargetNode );


                        crossNode          = memory.currentTargetNode;
                        crossNodeMyInDir   = memory.myInDirList.at( memory.currentTargetNodeIndexInNodeList );

                        int cTNIndex = pAgent[aID]->memory_reference.currentTargetNodeIndexInNodeList;


//                        strForDebug += QString(" currentTargetNodeIndexInNodeList=%1\n").arg( cTNIndex );


                        if( cTNIndex >= 0 && cTNIndex < pAgent[aID]->memory_reference.myInDirList.size() ){

                            crossNodeObjInDir  = pAgent[aID]->memory_reference.myInDirList.at( cTNIndex );

//                            strForDebug += QString(" crossNodeObjInDir=%1\n").arg( crossNodeObjInDir );

                        }
                        if( cTNIndex >= 0 && cTNIndex < pAgent[aID]->memory_reference.myOutDirList.size() ){

                            crossNodeObjOutDir = pAgent[aID]->memory_reference.myOutDirList.at( cTNIndex );

//                            strForDebug += QString(" crossNodeObjOutDir=%1\n").arg( crossNodeObjOutDir );
                        }

                    }
                    else{


                        int cnIdx = memory.myNodeList.indexOf( pAgent[aID]->memory_reference.currentTargetNode );
                        if( memory.currentTargetNodeIndexInNodeList >= 0 &&
                                cnIdx >= memory.currentTargetNodeIndexInNodeList &&
                                cnIdx < memory.myNodeList.size() &&
                                cnIdx < memory.myInDirList.size() ){

                            crossNode          = memory.myNodeList.at(cnIdx);

//                            strForDebug += QString("[2][Recog] V=%1\n").arg( memory.perceptedObjects[i]->objectID );
//                            strForDebug += QString(" crossNode=%1\n").arg( crossNode );

                            crossNodeMyInDir   = memory.myInDirList.at(cnIdx);

//                            strForDebug += QString(" currentTargetNodeIndexInNodeList=%1\n").arg( pAgent[aID]->memory_reference.currentTargetNodeIndexInNodeList );

                            if( pAgent[aID]->memory_reference.currentTargetNodeIndexInNodeList >= 0 &&
                                    pAgent[aID]->memory_reference.currentTargetNodeIndexInNodeList < pAgent[aID]->memory_reference.myInDirList.size() ){

                                crossNodeObjInDir  = pAgent[aID]->memory_reference.myInDirList.at( pAgent[aID]->memory_reference.currentTargetNodeIndexInNodeList );

//                                strForDebug += QString(" crossNodeObjInDir=%1\n").arg( crossNodeObjInDir );
                            }
                            if( pAgent[aID]->memory_reference.currentTargetNodeIndexInNodeList >= 0 &&
                                    pAgent[aID]->memory_reference.currentTargetNodeIndexInNodeList < pAgent[aID]->memory_reference.myOutDirList.size() ){

                                crossNodeObjOutDir = pAgent[aID]->memory_reference.myOutDirList.at( pAgent[aID]->memory_reference.currentTargetNodeIndexInNodeList );

//                                strForDebug += QString(" crossNodeObjOutDir=%1\n").arg( crossNodeObjOutDir );
                            }
                        }
                        else {

                            if( memory.currentTargetNodeIndexInNodeList >= 0 &&
                                    memory.currentTargetNodeIndexInNodeList < memory.myNodeList.size() &&
                                    pAgent[aID]->memory_reference.currentTargetNodeIndexInNodeList >= 0 &&
                                    pAgent[aID]->memory_reference.currentTargetNodeIndexInNodeList < pAgent[aID]->memory_reference.myNodeList.size() ){

                                int nCheckObj = 0;
                                for(int j=pAgent[aID]->memory_reference.currentTargetNodeIndexInNodeList;j<pAgent[aID]->memory_reference.myNodeList.size();++j){
                                    nCheckObj++;
                                    if( nCheckObj > 3){
                                        break;
                                    }
                                    int nCheck = 0;
                                    for(int k=memory.currentTargetNodeIndexInNodeList;k<memory.myNodeList.size();++k){
                                        nCheck++;
                                        if( nCheck > 3 ){
                                            break;
                                        }
                                        if( pAgent[aID]->memory_reference.myNodeList.at(j) == memory.myNodeList.at(k) ){

                                            crossNode          = memory.myNodeList.at(k);

//                                            strForDebug += QString("[3][Recog] V=%1\n").arg( memory.perceptedObjects[i]->objectID );
//                                            strForDebug += QString(" crossNode=%1\n").arg( crossNode );

                                            crossNodeMyInDir   = memory.myInDirList.at(k);

//                                            strForDebug += QString(" j=%1\n").arg( j );

                                            crossNodeObjInDir  = pAgent[aID]->memory_reference.myInDirList.at(j);
                                            crossNodeObjOutDir = pAgent[aID]->memory_reference.myOutDirList.at(j);

//                                            strForDebug += QString(" crossNodeObjInDir=%1\n").arg( crossNodeObjInDir );
//                                            strForDebug += QString(" crossNodeObjOutDir=%1\n").arg( crossNodeObjOutDir );

                                            nCheckObj = 3;

                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }

#ifdef _PERFORMANCE_CHECK_AGENT_RECOGNITION
                    QueryPerformanceCounter(&end);
                    calTime[1] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                    calCount[1]++;
#endif


                    if( crossNode >= 0 ){

#ifdef _PERFORMANCE_CHECK_AGENT_RECOGNITION
                        QueryPerformanceCounter(&start);
#endif

                        int dirLabel = DIRECTION_LABEL::STRAIGHT;
                        if( crossNodeObjOutDir >= 0 ){
                            dirLabel = pRoad->GetDirectionLabel( crossNode, crossNodeObjInDir, crossNodeObjOutDir );
                        }

                        int dir = pRoad->GetDirectionLabel( crossNode, crossNodeMyInDir, crossNodeObjInDir );

//                        strForDebug += QString("[Recog] V=%1\n").arg( memory.perceptedObjects[i]->objectID );
//                        strForDebug += QString(" crossNode=%1\n").arg( crossNode );
//                        strForDebug += QString(" crossNodeMyInDir=%1\n").arg( crossNodeMyInDir );
//                        strForDebug += QString(" crossNodeObjInDir=%1\n").arg( crossNodeObjInDir );
//                        strForDebug += QString(" dir=%1\n").arg( dir );

                        if( dir == DIRECTION_LABEL::ONCOMING ){
                            if( dirLabel == DIRECTION_LABEL::RIGHT_CROSSING  ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::ONCOMING_RIGHT;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                            else if( dirLabel == DIRECTION_LABEL::LEFT_CROSSING ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::ONCOMING_LEFT;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                            else{
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::ONCOMING_STRAIGHT;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                        }
                        else if( dir == DIRECTION_LABEL::LEFT_CROSSING ){
                            if( dirLabel == DIRECTION_LABEL::RIGHT_CROSSING  ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_CROSSING_RIGHT;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                            else if( dirLabel == DIRECTION_LABEL::LEFT_CROSSING ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_CROSSING_LEFT;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                            else{
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_CROSSING_STRAIGHT;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                        }
                        else if( dir == DIRECTION_LABEL::RIGHT_CROSSING ){
                            if( dirLabel == DIRECTION_LABEL::RIGHT_CROSSING  ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_RIGHT;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                            else if( dirLabel == DIRECTION_LABEL::LEFT_CROSSING ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_LEFT;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                            else{
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_STRAIGHT;
                                memory.perceptedObjects[i]->objPathRecogLabelChecked = objPath;
                                memory.perceptedObjects[i]->myPathRecogLabelChecked  = memory.currentTargetPath;
                                continue;
                            }
                        }

#ifdef _PERFORMANCE_CHECK_AGENT_RECOGNITION
                        QueryPerformanceCounter(&end);
                        calTime[2] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                        calCount[2]++;
#endif
                    }
                }
            }
        }

#ifdef _PERFORMANCE_CHECK_AGENT_RECOGNITION
        QueryPerformanceCounter(&end);
        calTime[4] += static_cast<double>(end.QuadPart - start4.QuadPart) * 1000.0 / freq.QuadPart;
        calCount[4]++;
#endif



#ifdef _PERFORMANCE_CHECK_AGENT_RECOGNITION

        for(int i=0;i<4;i++){
            if( calCount[i] > 500 ){
                calTime[i] /= calCount[i];
                qDebug() << " [Recgnition] Mean Time[" << i << "] = " << calTime[i];
                calCount[i] = 0;
            }
        }
#endif

        return;
    }

}

