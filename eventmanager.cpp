#include "simulationmanager.h"
#include <windows.h>
#include <QDebug>


void SimulationManager::RaiseEvent(Agent** pAgent,int maxAgentNumber, Road *pRoad)
{
    //qDebug() << "[RaiseEvent]";

    //qDebug() << "currentScenarioID = " << currentScenarioID;
    if( currentScenarioID < 0 || currentScenarioID >= scenario.size() ){
        return;
    }

    struct ScenarioData* currentScenario = scenario[currentScenarioID];
    int nEvent = currentScenario->scenarioEvents.size();
    //qDebug() << "nEvent = " << nEvent;
    if( nEvent == 0 ){
        return;
    }

    float simTimeSec = simTime.msec + simTime.sec + simTime.min * 60.0 + simTime.hour * 3600.0 + simTime.day * 86400.0;
    //qDebug() << "simTimeSec = " << simTimeSec;

    for(int i=0;i<nEvent;++i){

        struct ScenarioEvents* currentEvent = currentScenario->scenarioEvents[i];

        //qDebug() << "Event[" << i << "]" << " eventState = " << currentEvent->eventState << " Type = " << currentEvent->eventType;
        if( currentEvent->eventState == 2 ){   // Already finished
            continue;
        }


        // To prevent continuously the event is triggerred for keyboard operation
        if( currentEvent->eventState < 0 ){
            currentEvent->eventState++;
            continue;
        }


        if( currentEvent->eventState == 0 ){

            //
            // Check Event Trigger Condition
            struct ScenarioTriggers* trigger = currentEvent->eventTrigger;

            //qDebug() << " trigger mode = " << trigger->mode;
            if( trigger->mode == TRIGGER_TYPE::TIME_TRIGGER ||
                trigger->mode == TRIGGER_TYPE::POSITION_TIME_TRIGGER ){

                if( trigger->timeTriggerInSec > simTimeSec ){
                    continue;
                }
            }

            if( trigger->mode == TRIGGER_TYPE::POSITION_TRIGGER ||
                    trigger->mode == TRIGGER_TYPE::POSITION_SPEED_TRIGGER ||
                    trigger->mode == TRIGGER_TYPE::POSITION_TIME_TRIGGER ){

                int nCondPass = 0;
                for(int j=0;j<trigger->objectTigger.size();++j){
                    int targetObjectID = trigger->objectTigger[j]->targetObjectID;
                    //qDebug() << "targetObjectID = " << targetObjectID;
                    if( targetObjectID < 0 || targetObjectID >= maxAgentNumber ){
                        continue;
                    }
                    if( pAgent[targetObjectID]->agentStatus != 1 ){
                        continue;
                    }

                    float dx = pAgent[targetObjectID]->state.x - trigger->objectTigger[j]->x;
                    float dy = pAgent[targetObjectID]->state.y - trigger->objectTigger[j]->y;
                    float ip = dx * trigger->objectTigger[j]->cosDirect + dy * trigger->objectTigger[j]->sinDirect;
                    //qDebug() << "ip=" << ip;
                    if( ip < 0.0 ){
                        continue;
                    }

                    float cp = dy * trigger->objectTigger[j]->cosDirect - dx * trigger->objectTigger[j]->sinDirect;
                    //qDebug() << "cp = " << cp;
                    if( fabs(cp) > 2.5 ){
                        continue;
                    }

                    float ip2 = pAgent[targetObjectID]->state.cosYaw * trigger->objectTigger[j]->cosDirect
                            + pAgent[targetObjectID]->state.sinYaw * trigger->objectTigger[j]->sinDirect;
                    //qDebug() << "ip2 = " << ip2;
                    if( ip2 < 0.0 ){
                        continue;
                    }

                    if( trigger->mode == TRIGGER_TYPE::POSITION_SPEED_TRIGGER ){
                        if( pAgent[targetObjectID]->state.V < trigger->objectTigger[j]->speed ){
                            continue;
                        }
                    }

                    nCondPass++;
                }
                if( trigger->ANDCondition == 0 && nCondPass == 0 ){
                    continue;
                }
                else if( trigger->ANDCondition == 1 && nCondPass < trigger->objectTigger.size() ){
                    continue;
                }
            }

            if( trigger->mode == TRIGGER_TYPE::BY_FUNCTION_EXTENDER ){
                if( trigger->byExternalTriggerFlag == 0 ){
                    continue;
                }
            }

            if( trigger->mode == TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
                int keyHit = 0;
                switch( trigger->func_keys ){
                case 1: keyHit = GetAsyncKeyState(VK_F1); break;
                case 2: keyHit = GetAsyncKeyState(VK_F2); break;
                case 3: keyHit = GetAsyncKeyState(VK_F3); break;
                case 4: keyHit = GetAsyncKeyState(VK_F4); break;
                case 5: keyHit = GetAsyncKeyState(VK_F5); break;
                case 6: keyHit = GetAsyncKeyState(VK_F6); break;
                case 7: keyHit = GetAsyncKeyState(VK_F7); break;
                case 8: keyHit = GetAsyncKeyState(VK_F8); break;
                case 9: keyHit = GetAsyncKeyState(VK_F9); break;
                case 10: keyHit = GetAsyncKeyState(VK_F10); break;
                case 11: keyHit = GetAsyncKeyState(VK_F11); break;
                case 12: keyHit = GetAsyncKeyState(VK_F12); break;
                }
                if( keyHit == 0 ){
                    continue;
                }
            }

            //
            // Event triggerred
            qDebug() << "Event[" << currentEvent->eventID << "] triggerred.";

            currentEvent->eventState = 1;
            currentEvent->eventTimeCount = 0;
            currentEvent->eventTimeCount_sub = 0;
        }


        if( currentEvent->eventType == SCENARIO_EVENT_TYPE::SYSTEM_EVENT ){

            if( currentEvent->eventKind == SYSTEM_EVENT_KIND::SET_OBJECT_POSITION ){

                qDebug() << "Set Object Position Event:";
                qDebug() << "  Target Object ID: " << currentEvent->eventIntData[0];
                qDebug() << "  X: " << currentEvent->eventFloatData[0];
                qDebug() << "  Y: " << currentEvent->eventFloatData[1];
                qDebug() << "  Z: " << currentEvent->eventFloatData[2];
                qDebug() << "  Yaw: " << currentEvent->eventFloatData[3];

                int targetObjectID = currentEvent->eventIntData[0];
                if( targetObjectID >= 0 && targetObjectID < maxAgentNumber ){

                    pAgent[targetObjectID]->state.x = currentEvent->eventFloatData[0];
                    pAgent[targetObjectID]->state.y = currentEvent->eventFloatData[1];
                    pAgent[targetObjectID]->state.z = currentEvent->eventFloatData[2];

                    float yawAngle = currentEvent->eventFloatData[3] * 0.017452;
                    pAgent[targetObjectID]->state.yaw = yawAngle;
                    pAgent[targetObjectID]->state.cosYaw = cos( yawAngle );
                    pAgent[targetObjectID]->state.sinYaw = sin( yawAngle );

                    pAgent[targetObjectID]->vehicle.SetInitialState( pAgent[targetObjectID]->state.V,
                                                                     pAgent[targetObjectID]->state.x,
                                                                     pAgent[targetObjectID]->state.y,
                                                                     pAgent[targetObjectID]->state.z,
                                                                     pAgent[targetObjectID]->state.yaw);

                    // get current path
                    if( pAgent[targetObjectID]->memory.routeType == ROUTE_TYPE::PATH_LIST_TYPE ){

                        float dist = 0;
                        int currentPath = pRoad->GetNearestPathFromList( pAgent[targetObjectID]->state.x,
                                                                         pAgent[targetObjectID]->state.y,
                                                                         yawAngle, dist,
                                                                         pAgent[targetObjectID]->memory.targetPathList );
                        if( currentPath < 0 ){
                            qDebug() << "[Warning]----------------------------------";
                            qDebug() << " Scenario Vehicle ID = " << targetObjectID << " cannot determin nearest path from assigned list.";
                            qDebug() << "   Assigned Path List : ";
                            for(int j=0;j<pAgent[targetObjectID]->memory.targetPathList.size();++j){
                                qDebug() << "           Path " << pAgent[targetObjectID]->memory.targetPathList[j];
                            }
                            continue;
                        }

                        pAgent[targetObjectID]->memory.currentTargetPath = currentPath;
                    }
                    else if( pAgent[targetObjectID]->memory.routeType == ROUTE_TYPE::NODE_LIST_TYPE ){



                    }

                }

                currentEvent->eventState = 2;

                int repeatFlag = currentEvent->eventIntData[1];
                if( repeatFlag == 1 ){
                    currentEvent->eventState = 0;
                }
            }
            else if( currentEvent->eventKind == SYSTEM_EVENT_KIND::SEND_UDP_DATA_BY_SYSTEM_EVENT ){


                currentEvent->eventState = 2;

            }
            else if( currentEvent->eventKind == SYSTEM_EVENT_KIND::CHANGE_ROAD_SPEEDLIMIT ){

                currentEvent->eventState = 2;

            }
            else if( currentEvent->eventKind == SYSTEM_EVENT_KIND::CHANGE_TRAFFIC_SIGNAL ){

                currentEvent->eventState = 2;

            }

        }
        else if( currentEvent->eventType == SCENARIO_EVENT_TYPE::OBJECT_EVENT ){

            int targetObjectID = currentEvent->targetObjectID;
            if( targetObjectID < 0 || targetObjectID >= maxAgentNumber ){
                continue;
            }

            if( currentEvent->eventKind == OBJECT_EVENT_KIND::SUDDEN_DECELERATION ){

                //qDebug() << "[OBJECT_EVENT_KIND::SUDDEN_DECELERATION]";
                //qDebug() << "  eventFloatData.size = " << currentEvent->eventFloatData.size();
                //qDebug() << "  targetObjectID = " << targetObjectID;

                pAgent[targetObjectID]->memory.overrideBrakeByScenario = true;

                float eventTime = currentEvent->eventTimeCount * simTime.dt;
                currentEvent->eventTimeCount++;

                int nAccelPoint = (int)currentEvent->eventFloatData[0];
                //qDebug() << "  nAccelPoint = " << nAccelPoint;

                if( nAccelPoint > 0 && nAccelPoint <= 8 ){
                    for(int k=1;k<=nAccelPoint;++k){
                        if( k == 1 ){
                            pAgent[targetObjectID]->memory.overrideAxControl = (-1.0) * fabs( currentEvent->eventFloatData[2] * eventTime / (currentEvent->eventFloatData[1]) )  * 9.81;
                        }
                        else{
                            float t1 = currentEvent->eventFloatData[2*(k-1)-1];
                            float t2 = currentEvent->eventFloatData[2*k-1];
                            float a1 = fabs(currentEvent->eventFloatData[2*(k-1)]) * 9.81;
                            float a2 = fabs(currentEvent->eventFloatData[2*k]) * 9.81;
                            //qDebug() << "k=" << k << " t1=" << t1 << " t2=" << t2 << " a1=" << a1 << " a2=" << a2;

                            if( t1 <= eventTime && eventTime < t2 ){
                                float d  = eventTime - t1;
                                float dt = t2 - t1;
                                float da = a2 - a1;
                                pAgent[targetObjectID]->memory.overrideAxControl = (-1.0) * ( a1 + fabs(da) / dt * d );
                                //qDebug() << "[1]overrideAxControl = " << pAgent[targetObjectID]->memory.overrideAxControl;
                                break;
                            }
                            else if( t2 <= eventTime && k == nAccelPoint ){
                                pAgent[targetObjectID]->memory.overrideAxControl = (-1.0) * a2;
                                //qDebug() << "[2]overrideAxControl = " << pAgent[targetObjectID]->memory.overrideAxControl;
                            }
                        }
                    }
                }

                if( fabs( currentEvent->eventFloatData[19] ) > 0.01 ){
                    pAgent[targetObjectID]->memory.overrideSteerByScenario = true;
                    pAgent[targetObjectID]->memory.overrideSteerControl = currentEvent->eventFloatData[19];
                }

                int completelyStop = (int)currentEvent->eventFloatData[18];
                if( completelyStop == 0 ){
                    float endTime = currentEvent->eventFloatData[2*nAccelPoint-1];
                    if( endTime < eventTime ){
                        pAgent[targetObjectID]->memory.overrideBrakeByScenario = false;
                        pAgent[targetObjectID]->memory.overrideAxControl = 0.0;
                        pAgent[targetObjectID]->memory.overrideSteerByScenario = false;
                        pAgent[targetObjectID]->memory.overrideSteerControl = 0.0;
                        currentEvent->eventTimeCount     = 0;
                        currentEvent->eventTimeCount_sub = 0;
                        currentEvent->eventState = 2;
                        if( currentEvent->eventTrigger->mode == TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
                            currentEvent->eventState = (-1) * simTime.exe_freq;
                        }
                        continue;
                    }
                }
                else{
                    if( pAgent[targetObjectID]->state.V  < 0.05 ){

                        float stopTime = currentEvent->eventTimeCount_sub * simTime.dt;
                        currentEvent->eventTimeCount_sub++;

                        float targetStopTime = currentEvent->eventFloatData[17];
                        if( targetStopTime <= stopTime ){
                            pAgent[targetObjectID]->memory.overrideBrakeByScenario = false;
                            pAgent[targetObjectID]->memory.overrideAxControl = 0.0;
                            pAgent[targetObjectID]->memory.overrideSteerByScenario = false;
                            pAgent[targetObjectID]->memory.overrideSteerControl = 0.0;
                            currentEvent->eventTimeCount     = 0;
                            currentEvent->eventTimeCount_sub = 0;
                            currentEvent->eventState = 2;
                            if( currentEvent->eventTrigger->mode == TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
                                currentEvent->eventState = (-1) * simTime.exe_freq;
                            }
                            continue;
                        }
                    }
                }
            }
            else if( currentEvent->eventKind == OBJECT_EVENT_KIND::LANE_CHANGE ){

                currentEvent->eventState = 2;

            }
            else if( currentEvent->eventKind == OBJECT_EVENT_KIND::LANE_DEPARTURE ){

                currentEvent->eventState = 2;

            }
            else if( currentEvent->eventKind == OBJECT_EVENT_KIND::TRAFFIC_SIGNAL_VIOLATION ){


                currentEvent->eventState = 2;

            }
            else if( currentEvent->eventKind == OBJECT_EVENT_KIND::CHANGE_CONTROL_MODE ){

                int tmpControlMode = (int)currentEvent->eventFloatData[0];

                pAgent[targetObjectID]->memory.controlMode = tmpControlMode;

                if( tmpControlMode == AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY
                        || tmpControlMode == AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY_KEEP_INITIAL_LATERAL_OFFSET ){


                    pAgent[targetObjectID]->memory.precedingVehicleIDByScenario = (int)currentEvent->eventFloatData[20];

                    pAgent[targetObjectID]->memory.targetSpeedByScenario = currentEvent->eventFloatData[1] / 3.6;

                    pAgent[targetObjectID]->memory.targetHeadwayDistanceByScenario = currentEvent->eventFloatData[2];
                    pAgent[targetObjectID]->memory.targetHeadwayTimeByScenario     = currentEvent->eventFloatData[3];

                    pAgent[targetObjectID]->memory.doHeadwayDistanceControl = false;  // This flag is set when detected preceding vehicle
                    pAgent[targetObjectID]->memory.doStopControl = false;

                }
                else if( tmpControlMode == AGENT_CONTROL_MODE::SPEED_PROFILE ){

                    pAgent[targetObjectID]->memory.speedProfileCount = 0;

                    pAgent[targetObjectID]->memory.profileTime.clear();
                    pAgent[targetObjectID]->memory.profileSpeed.clear();

                    int nData = (int)currentEvent->eventFloatData[4];
                    for(int k=0;k<nData;++k){
                        pAgent[targetObjectID]->memory.profileTime.append( currentEvent->eventFloatData[6+2*k] );
                        pAgent[targetObjectID]->memory.profileSpeed.append( currentEvent->eventFloatData[6+2*k+1] / 3.6 );
                    }

                    pAgent[targetObjectID]->vehicle.SetInitialSpeed( pAgent[targetObjectID]->memory.profileSpeed[0] );
                    pAgent[targetObjectID]->memory.profileTime[0] = 0.0f;

                    pAgent[targetObjectID]->state.V = pAgent[targetObjectID]->memory.profileSpeed[0];
                }
                else if( tmpControlMode == AGENT_CONTROL_MODE::USE_TIME_SERIES_DATA ){


                }
                else if( tmpControlMode == AGENT_CONTROL_MODE::STOP_AT ){

                    pAgent[targetObjectID]->memory.targetSpeedByScenario = currentEvent->eventFloatData[1] / 3.6;

                    pAgent[targetObjectID]->memory.targetStopAtXByScenario = currentEvent->eventFloatData[2];
                    pAgent[targetObjectID]->memory.targetStopAtYByScenario = currentEvent->eventFloatData[3];

                    pAgent[targetObjectID]->memory.actualStopOnPathID = -1;

                    pAgent[targetObjectID]->memory.doStopControl = true;
                }
                else if( tmpControlMode == AGENT_CONTROL_MODE::SET_STEER_CONTROL_FLAG ){
                    if( currentEvent->eventFloatData[1] < 0.5 ){
                        pAgent[targetObjectID]->memory.doSteerControl = false;
                    }
                    else if( currentEvent->eventFloatData[1] > 0.5 ){
                        pAgent[targetObjectID]->memory.doSteerControl = true;
                    }
                }

                currentEvent->eventState = 2;
                if( currentEvent->eventTrigger->mode == TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
                    currentEvent->eventState = (-1) * simTime.exe_freq;
                }

            }
            else if( currentEvent->eventKind == OBJECT_EVENT_KIND::SET_LATERAL_OFFSET ){

                pAgent[targetObjectID]->memory.additionalShiftByScenarioEvent = true;

                float eventTime = currentEvent->eventTimeCount * simTime.dt;
                currentEvent->eventTimeCount++;

                float riseTime  = currentEvent->eventFloatData[1];
                float holdTime  = currentEvent->eventFloatData[2];
                float decayTime = currentEvent->eventFloatData[3];

                float t1 = riseTime;
                float t2 = t1 + holdTime;
                float t3 = t2 + decayTime;

                float maxLateralShift = currentEvent->eventFloatData[0];

                if( eventTime < t1 ){
                    pAgent[targetObjectID]->memory.additionalLateralShift = maxLateralShift * eventTime / t1;
                }
                else if( eventTime < t2 ){
                    pAgent[targetObjectID]->memory.additionalLateralShift = maxLateralShift;
                }
                else if( eventTime < t3 ){
                    pAgent[targetObjectID]->memory.additionalLateralShift = maxLateralShift * (1.0 - (eventTime-t2)/(t3-t2));
                }
                else{
                    pAgent[targetObjectID]->memory.additionalShiftByScenarioEvent = false;
                    pAgent[targetObjectID]->memory.additionalLateralShift = 0.0;
                    currentEvent->eventTimeCount     = 0;
                    currentEvent->eventTimeCount_sub = 0;
                    currentEvent->eventState = 2;
                    if( currentEvent->eventTrigger->mode == TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
                        currentEvent->eventState = (-1) * simTime.exe_freq;
                    }
                }
            }
            else if( currentEvent->eventKind == OBJECT_EVENT_KIND::SEND_UDP_DATA_BY_OBJECT_EVENT ){


                currentEvent->eventState = 2;

            }
            else if( currentEvent->eventKind == OBJECT_EVENT_KIND::HEADLIGHT ){

                currentEvent->eventState = 2;

            }
            else if( currentEvent->eventKind == OBJECT_EVENT_KIND::VEHICLE_LAMPS ){



                currentEvent->eventState = 2;
            }
        }
    }

}
