#include "agent.h"
#include <QDebug>


void Agent::RiskEvaluation()
{

    if( memory.controlMode == AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY ){

        if( memory.precedingVehicleIDByScenario >= 0 ){

            for(int i=0;i<memory.perceptedObjects.size();++i){
                if( memory.perceptedObjects[i]->objectID == memory.precedingVehicleIDByScenario ){

                    memory.doHeadwayDistanceControl   = true;

                    memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->distanceToObject;

                    if( memory.distanceToPrecedingVehicle >= 0.0 ){
                        memory.distanceToPrecedingVehicle -= memory.perceptedObjects[i]->vHalfLength;
                        memory.distanceToPrecedingVehicle -= vHalfLength;
                    }
                    else{
                        memory.distanceToPrecedingVehicle += memory.perceptedObjects[i]->vHalfLength;
                        memory.distanceToPrecedingVehicle += vHalfLength;
                    }

                    memory.speedPrecedingVehicle = memory.perceptedObjects[i]->V * memory.perceptedObjects[i]->innerProductToNearestPathTangent;
                    memory.axPrecedingVehicle    = memory.perceptedObjects[i]->Ax;

//                    qDebug() << "[Agent" << ID << "]";
//                    qDebug() << "Preceding Vehicle ID = " << memory.precedingVehicleIDByScenario;
//                    qDebug() << " distanceToPrecedingVehicle = " << memory.distanceToPrecedingVehicle;
//                    qDebug() << " innerProductToNearestPathTangent = " << memory.perceptedObjects[i]->innerProductToNearestPathTangent;
//                    qDebug() << " speedPrecedingVehicle = " << memory.speedPrecedingVehicle;

                    break;
                }
            }

        }
        else{
            memory.doHeadwayDistanceControl = false;
        }

        return;
    }




    if(  memory.controlMode == AGENT_CONTROL_MODE::AGENT_LOGIC ){

        //
        // Risk evaluation for Traffic Signal
        if( memory.doStopControl == true ){
            memory.releaseStopCount++;
            if( memory.releaseStopCount * calInterval >=param.startRelay ){
                memory.doStopControl = false;
                memory.releaseStopCount = 0;
            }
        }

        if( memory.perceptedSignals.size() >= 0 ){

            bool ShouldStop = false;
            float minDist = 0.0;

            for(int i=0;i<memory.perceptedSignals.size();++i){

                if( memory.perceptedSignals[i]->signalDisplay == TRAFFICSIGNAL_YELLOW ){

                    if( memory.perceptedSignals[i]->distToSL > memory.distanceToZeroSpeed + 5.0 ){

                        if( ShouldStop == false || minDist > memory.perceptedSignals[i]->distToSL ){
                            ShouldStop = true;
                            minDist = memory.perceptedSignals[i]->distToSL;
                        }
                    }
                }
                else if( memory.perceptedSignals[i]->signalDisplay == TRAFFICSIGNAL_RED ){

                    if( memory.perceptedSignals[i]->distToSL > -5.0 &&
                            memory.perceptedSignals[i]->distToSL < memory.distanceToZeroSpeed + 5.0 ){

                        if( ShouldStop == false || minDist > memory.perceptedSignals[i]->distToSL ){
                            ShouldStop = true;
                            minDist = memory.perceptedSignals[i]->distToSL;
                        }
                    }
                }
            }

            if( ShouldStop == true ){
                memory.doStopControl = true;
                memory.distanceToStopPoint = minDist;
            }
        }



        //
        // Risk evaluation for preceding vehicle
        memory.doHeadwayDistanceControl = false;
        memory.precedingVehicleID = -1;
        memory.distanceToPrecedingVehicle = 0.0;

        float vLenH = 0.0;

        for(int i=0;i<memory.perceptedObjects.size();++i){
            if( memory.perceptedObjects[i]->recognitionLabel == PRECEDING ){
                if( memory.precedingVehicleID < 0 ||  memory.distanceToPrecedingVehicle > memory.perceptedObjects[i]->distanceToObject ){
                    memory.precedingVehicleID = memory.perceptedObjects[i]->objectID;
                    memory.distanceToPrecedingVehicle = memory.perceptedObjects[i]->distanceToObject;
                    memory.speedPrecedingVehicle = memory.perceptedObjects[i]->V;
                    memory.axPrecedingVehicle    = memory.perceptedObjects[i]->Ax;
                    vLenH = memory.perceptedObjects[i]->vHalfLength;
                }
            }
        }

        if( memory.precedingVehicleID >= 0 ){

            memory.doHeadwayDistanceControl = true;

            memory.targetHeadwayDistance = param.headwayTime * state.V;
            if( memory.targetHeadwayDistance < param.minimumHeadwayDistanceAtStop + vLenH + vHalfLength  ){
                memory.targetHeadwayDistance = param.minimumHeadwayDistanceAtStop + vLenH + vHalfLength;
            }
        }

        return;
    }

}

