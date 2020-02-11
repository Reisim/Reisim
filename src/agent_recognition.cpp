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


void Agent::Recognition( Agent** pAgent, int maxAgent, Road* pRoad )
{

    if( memory.controlMode == AGENT_CONTROL_MODE::AGENT_LOGIC ){

        for(int i=0;i<memory.perceptedObjects.size();++i){

            memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::UNDEFINED_RECOGNITION_LABEL;

            if( memory.perceptedObjects[i]->objectType >= 100 ){
                continue;
            }

            int objPath = memory.perceptedObjects[i]->objectPath;
            bool isSameLane = false;
            if( memory.targetPathList.indexOf( objPath ) >= 0 ){
                isSameLane = true;
            }

            if( isSameLane == true ){

                if( memory.perceptedObjects[i]->distanceToObject > vHalfLength  ){
                    memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::PRECEDING;

                }
                else if( memory.perceptedObjects[i]->distanceToObject < -vHalfLength ){
                    memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::FOLLOWING;
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

                    // SIDE Vehicles
                    if( fabs(memory.perceptedObjects[i]->deviationFromNearestTargetPath - memory.lateralDeviationFromTargetPath) >=
                            memory.perceptedObjects[i]->effectiveHalfWidth + vHalfWidth + 0.5 ){

                        if( memory.perceptedObjects[i]->deviationFromNearestTargetPath > 0.0 ){
                            if( memory.perceptedObjects[i]->distanceToObject > vHalfLength + memory.perceptedObjects[i]->vHalfLength ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_SIDE_PRECEDING;

                            }
                            else if( memory.perceptedObjects[i]->distanceToObject < -vHalfLength - memory.perceptedObjects[i]->vHalfLength ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_SIDE_FOLLOWING;
                            }
                            else{
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_SIDE;
                            }
                        }
                        else if( memory.perceptedObjects[i]->deviationFromNearestTargetPath < 0.0 ){
                            if( memory.perceptedObjects[i]->distanceToObject > vHalfLength + memory.perceptedObjects[i]->vHalfLength ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_SIDE_PRECEDING;

                            }
                            else if( memory.perceptedObjects[i]->distanceToObject < -vHalfLength - memory.perceptedObjects[i]->vHalfLength ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_SIDE_FOLLOWING;
                            }
                            else{
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_SIDE;
                            }
                        }
                    }
                    else{

                        if( memory.perceptedObjects[i]->distanceToObject > vHalfLength  ){
                            memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::PRECEDING;

                        }
                        else if( memory.perceptedObjects[i]->distanceToObject < -vHalfLength ){
                            memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::FOLLOWING;
                        }
                    }

                }
                else{

                    // Check if the object have the same target node
                    int aID                = memory.perceptedObjects[i]->objectID;
                    int crossNode          = -1;
                    int crossNodeMyInDir   = -1;
                    int crossNodeObjInDir  = -1;
                    int crossNodeObjOutDir = -1;

                    if( memory.currentTargetNode == pAgent[aID]->memory.currentTargetNode ){
                        crossNode          = memory.currentTargetNode;
                        crossNodeMyInDir   = memory.myInDirList.at( memory.currentTargetNodeIndexInNodeList );
                        crossNodeObjInDir  = pAgent[aID]->memory.myInDirList.at( pAgent[aID]->memory.currentTargetNodeIndexInNodeList );
                        crossNodeObjOutDir = pAgent[aID]->memory.myOutDirList.at( pAgent[aID]->memory.currentTargetNodeIndexInNodeList );
                    }
                    else{

                        int cnIdx = memory.myNodeList.indexOf( pAgent[aID]->memory.currentTargetNode );
                        if( cnIdx >= memory.currentTargetNodeIndexInNodeList ){
                            crossNode          = memory.myNodeList.at(cnIdx);
                            crossNodeMyInDir   = memory.myInDirList.at(cnIdx);
                            crossNodeObjInDir  = pAgent[aID]->memory.myInDirList.at( pAgent[aID]->memory.currentTargetNodeIndexInNodeList );
                            crossNodeObjOutDir = pAgent[aID]->memory.myOutDirList.at( pAgent[aID]->memory.currentTargetNodeIndexInNodeList );
                        }
                        else {
                            for(int j=pAgent[aID]->memory.currentTargetNodeIndexInNodeList;j<pAgent[aID]->memory.myNodeList.size();++j){
                                int cnIdx = memory.myNodeList.indexOf( pAgent[aID]->memory.myNodeList.at(j) );
                                if( cnIdx >= memory.currentTargetNodeIndexInNodeList ){
                                    crossNode          = memory.myNodeList.at(cnIdx);
                                    crossNodeMyInDir   = memory.myInDirList.at(cnIdx);
                                    crossNodeObjInDir  = pAgent[aID]->memory.myInDirList.at(j);
                                    crossNodeObjOutDir = pAgent[aID]->memory.myOutDirList.at(j);
                                    break;
                                }
                            }
                        }
                    }


                    if( crossNode >= 0 ){

                        int dirLabel = DIRECTION_LABEL::STRAIGHT;
                        if( crossNodeObjOutDir >= 0 ){
                            dirLabel = pRoad->GetDirectionLabel( crossNode, crossNodeObjInDir, crossNodeObjOutDir );
                        }

                        int dir = pRoad->GetDirectionLabel( crossNode, crossNodeMyInDir, crossNodeObjInDir );
                        if( dir == DIRECTION_LABEL::ONCOMING ){
                            if( dirLabel == DIRECTION_LABEL::RIGHT_CROSSING  ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::ONCOMING_RIGHT;
                            }
                            else if( dirLabel == DIRECTION_LABEL::LEFT_CROSSING ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::ONCOMING_LEFT;
                            }
                            else{
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::ONCOMING_STRAIGHT;
                            }
                        }
                        else if( dir == DIRECTION_LABEL::LEFT_CROSSING ){
                            if( dirLabel == DIRECTION_LABEL::RIGHT_CROSSING  ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_CROSSING_RIGHT;
                            }
                            else if( dirLabel == DIRECTION_LABEL::LEFT_CROSSING ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_CROSSING_LEFT;
                            }
                            else{
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::LEFT_CROSSING_STRAIGHT;
                            }
                        }
                        else if( dir == DIRECTION_LABEL::RIGHT_CROSSING ){
                            if( dirLabel == DIRECTION_LABEL::RIGHT_CROSSING  ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_RIGHT;
                            }
                            else if( dirLabel == DIRECTION_LABEL::LEFT_CROSSING ){
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_LEFT;
                            }
                            else{
                                memory.perceptedObjects[i]->recognitionLabel = AGENT_RECOGNITION_LABEL::RIGHT_CROSSING_STRAIGHT;
                            }
                        }
                    }
                }
            }
        }
        return;
    }

}

