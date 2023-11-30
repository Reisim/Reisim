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

        // Except vehicle or pedestrian appear event
        if( currentEvent->eventType == SCENARIO_EVENT_TYPE::OBJECT_EVENT &&
                (currentEvent->eventKind == OBJECT_SCENARIO_ACTION_KIND::APPEAR_VEHICLE ||
                 currentEvent->eventKind == OBJECT_SCENARIO_ACTION_KIND::APPEAR_PEDESTRIAN) ){
            continue;
        }

        // To prevent continuously the event is triggerred for keyboard operation
        if( currentEvent->eventState < 0 ){
            currentEvent->eventState++;
            continue;
        }


        bool posTrigFired = false;

        if( currentEvent->eventState == 0 ){

            //
            // Check Event Trigger Condition
            struct ScenarioTriggers* trigger = currentEvent->eventTrigger;

            //qDebug() << " trigger mode = " << trigger->mode << " combination=" << trigger->combination;

            if( trigger->combination >= 0 ){

                int nTriggered = 0;

                bool afterCheckAfterTimeOrPosTrigger = false;
                bool hasTimeTrigger = false;
                if( trigger->combination == 0 && trigger->objectTigger.size() > 0 ){
                    bool hasTimeOrPosTrigger = false;
                    bool hasVelOrTTCTrigger = false;
                    for(int j=0;j<trigger->objectTigger.size();++j){
                        if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ){
                            hasTimeTrigger = true;
                        }
                        if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ||
                                trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::POSITION_TRIGGER ){
                            hasTimeOrPosTrigger = true;
                        }
                        if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::VELOCITY_TRIGGER ||
                                trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TTC_TRIGGER ){
                            hasVelOrTTCTrigger = true;
                        }
                    }
                    if( hasTimeOrPosTrigger == true && hasVelOrTTCTrigger == true ){
                        afterCheckAfterTimeOrPosTrigger = true;
                    }
                }

                for(int j=0;j<trigger->objectTigger.size();++j){

                    if( trigger->objectTigger[j]->isTriggered == true ){
                        nTriggered++;
                        continue;
                    }

                    if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::AT_ONCE ){
                        trigger->objectTigger[j]->isTriggered = true;
                        nTriggered++;
                        continue;
                    }
                    else if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ){

                        if( currentEvent->eventType ==  SCENARIO_EVENT_TYPE::OBJECT_EVENT && trigger->objectTigger[j]->timeFromAppear == true ){

                            int objID = currentEvent->targetObjectID;
                            if( objID >= 0 && objID < maxAgentNumber && pAgent[objID]->agentStatus == 1 && pAgent[objID]->TimeOfAppear >= 0.0 ){
                                float triggerTime = pAgent[objID]->TimeOfAppear + trigger->objectTigger[j]->timeTriggerInSec;
                                if( triggerTime <= simTimeSec ){
                                    trigger->objectTigger[j]->isTriggered = true;
                                    nTriggered++;
                                    continue;
                                }
                            }
                        }
                        else{
                            if( trigger->objectTigger[j]->timeTriggerInSec <= simTimeSec ){
                                trigger->objectTigger[j]->isTriggered = true;
                                nTriggered++;
                                continue;
                            }
                        }
                    }
                    else if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::POSITION_TRIGGER ){

                        bool evalPosTrig = true;
                        if( hasTimeTrigger == true ){
                            bool evaluedTimeTrig = false;
                            for(int j=0;j<trigger->objectTigger.size();++j){
                                if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ){
                                    if( trigger->objectTigger[j]->isTriggered == true ){
                                        evaluedTimeTrig = true;
                                        break;
                                    }
                                }
                            }
                            if( evaluedTimeTrig == false ){
                                evalPosTrig = false;
                            }
                        }

                        if( evalPosTrig == true ){

                            int objID = trigger->objectTigger[j]->targetObjectID;
                            if( objID < 0 && currentEvent->eventType ==  SCENARIO_EVENT_TYPE::OBJECT_EVENT && currentEvent->targetObjectID >= 0 ){
                                objID = currentEvent->targetObjectID;
                            }

                            if( objID >= 0 && objID < maxAgentNumber && pAgent[objID]->agentStatus == 1 ){

                                float rx = pAgent[objID]->state.x -  trigger->objectTigger[j]->x;
                                float ry = pAgent[objID]->state.y -  trigger->objectTigger[j]->y;
                                float ip = rx * trigger->objectTigger[j]->cosDirect + ry * trigger->objectTigger[j]->sinDirect;
                                float e = rx * trigger->objectTigger[j]->sinDirect * (-1.0) + ry * trigger->objectTigger[j]->cosDirect;

                                //qDebug() << "ip = " << ip << " e = " << e << " passCheckFlag = " << trigger->objectTigger[j]->passCheckFlag;

                                float w = trigger->objectTigger[j]->widthHalf;

                                if( trigger->objectTigger[j]->passCheckFlag == 0 && fabs(e) < w && ip < 0.0 && ip > -30.0 ){
                                    trigger->objectTigger[j]->passCheckFlag = 1;
                                    qDebug() << "passCheckFlag = 1";
                                }
                                else if( trigger->objectTigger[j]->passCheckFlag == 1 && fabs(e) < w && ip >= 0.0 ){
                                    trigger->objectTigger[j]->passCheckFlag = 2;
                                    trigger->objectTigger[j]->isTriggered = true;

                                    posTrigFired = true;

                                    nTriggered++;
                                    qDebug() << "passCheckFlag = 2";
                                    continue;
                                }
                            }
                        }
                    }
                    else if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::VELOCITY_TRIGGER ){

                        bool evalVelTrig = true;
                        if( afterCheckAfterTimeOrPosTrigger == true ){
                            bool evaluedTimeOrPosTrig = false;
                            for(int j=0;j<trigger->objectTigger.size();++j){
                                if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ||
                                        trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::POSITION_TRIGGER ){
                                    if( trigger->objectTigger[j]->isTriggered == true ){
                                        evaluedTimeOrPosTrig = true;
                                        break;
                                    }
                                }
                            }
                            if( evaluedTimeOrPosTrig == false ){
                                evalVelTrig = false;
                            }
                        }

                        if( evalVelTrig == true ){
                            int objID = trigger->objectTigger[j]->targetObjectID;
                            //qDebug() << "[VEL_TRIG] objID = " << objID;

                            if( objID >= 0 && objID < maxAgentNumber && pAgent[objID]->agentStatus == 1 ){

                                //qDebug() << "[VEL_TRIG] objID = " << objID
                                //         << " isSInterfaceObject = " << pAgent[objID]->isSInterfaceObject
                                //         << " isSInterObjDataSet = " << pAgent[objID]->isSInterObjDataSet;

                                if( pAgent[objID]->isSInterfaceObject == true && pAgent[objID]->isSInterObjDataSet == false ){
                                    continue;
                                }

                                float objV = pAgent[objID]->state.V;

                                //qDebug() << "[VEL_TRIG] triggerParam = " << trigger->objectTigger[j]->triggerParam
                                //         << " objV = " << objV << " speed = " << trigger->objectTigger[j]->speed;

                                if( trigger->objectTigger[j]->triggerParam == 1 && objV < trigger->objectTigger[j]->speed ){
                                    trigger->objectTigger[j]->isTriggered = true;
                                    nTriggered++;

                                    //qDebug() << "[VEL_TRIG] Triggered; slower";

                                    continue;
                                }
                                else if( trigger->objectTigger[j]->triggerParam == 0 && objV > trigger->objectTigger[j]->speed ){
                                    trigger->objectTigger[j]->isTriggered = true;
                                    nTriggered++;

                                    //qDebug() << "[VEL_TRIG] Triggered; faster";

                                    continue;
                                }
                            }
                        }
                    }
                    else if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TTC_TRIGGER ){

                        bool evalTTCTrig = true;
                        if( afterCheckAfterTimeOrPosTrigger == true ){
                            bool evaluedTimeOrPosTrig = false;
                            for(int j=0;j<trigger->objectTigger.size();++j){
                                if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::TIME_TRIGGER ||
                                        trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::POSITION_TRIGGER ){
                                    if( trigger->objectTigger[j]->isTriggered == true ){
                                        evaluedTimeOrPosTrig = true;
                                        break;
                                    }
                                }
                            }
                            if( evaluedTimeOrPosTrig == false ){
                                evalTTCTrig = false;
                            }
                        }

                        if( evalTTCTrig == true ){
                            int objID = trigger->objectTigger[j]->targetObjectID;
                            if( objID < 0 && currentEvent->eventType ==  SCENARIO_EVENT_TYPE::OBJECT_EVENT && currentEvent->targetObjectID >= 0 ){
                                objID = currentEvent->targetObjectID;
                            }

    //                        qDebug() << "objID = " << objID << " triggerParam = " << trigger->objectTigger[j]->triggerParam;

                            if( trigger->objectTigger[j]->triggerParam == 1 ){    // calculate TTC to object

                                int calID = trigger->objectTigger[j]->triggerParam2;

                                if( objID >= 0 && objID < maxAgentNumber && pAgent[objID]->agentStatus == 1 &&
                                       calID >= 0 && calID < maxAgentNumber && pAgent[calID]->agentStatus == 1 ){

                                    float rx = pAgent[calID]->state.x - pAgent[objID]->state.x;
                                    float ry = pAgent[calID]->state.y - pAgent[objID]->state.y;
                                    float ip = rx * pAgent[objID]->state.cosYaw + ry * pAgent[objID]->state.sinYaw;
                                    float e = rx * pAgent[objID]->state.sinYaw * (-1.0) + ry * pAgent[objID]->state.cosYaw;

                                    float f = pAgent[calID]->state.cosYaw * pAgent[objID]->state.cosYaw + pAgent[calID]->state.sinYaw * pAgent[objID]->state.sinYaw;
                                    float relV = pAgent[objID]->state.V - f * pAgent[calID]->state.V;

                                    if( ip >= 0.0 && relV > 0.1 && fabs(e) < ip * 0.577 ){  // FOV = 60[deg]
                                        float ttc = ip / relV;
                                        if( ttc < trigger->objectTigger[j]->TTC ){
                                            trigger->objectTigger[j]->isTriggered = true;
                                            nTriggered++;
                                            continue;
                                        }
                                    }
                                }
                            }
                            else if( trigger->objectTigger[j]->triggerParam == 0 ){   // calculate TTC to point

                                float rx = trigger->objectTigger[j]->x - pAgent[objID]->state.x;
                                float ry = trigger->objectTigger[j]->y - pAgent[objID]->state.y;
                                float ip = rx * pAgent[objID]->state.cosYaw + ry * pAgent[objID]->state.sinYaw;
                                float e = rx * pAgent[objID]->state.sinYaw * (-1.0) + ry * pAgent[objID]->state.cosYaw;

    //                            qDebug() << "tx = " << trigger->objectTigger[j]->x << " ty = " << trigger->objectTigger[j]->y
    //                                     << " vx=" << pAgent[objID]->state.x << " vy = " << pAgent[objID]->state.y;
    //                            qDebug() << "rx = " << rx << " ry = " << ry << " ip=" << ip << " e = " << e;

                                if( ip >= 0.0 && pAgent[objID]->state.V > 0.1 && fabs(e) < ip * 0.577 ){  // FOV = 60[deg]
                                    float ttc = ip / pAgent[objID]->state.V;

    //                                qDebug() << "ttc = " << ttc << " TTC = " << trigger->objectTigger[j]->TTC;

                                    if( ttc < trigger->objectTigger[j]->TTC ){
                                        trigger->objectTigger[j]->isTriggered = true;
                                        nTriggered++;
                                        continue;
                                    }
                                }

                            }
                        }
                    }
                    else if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::BY_FUNCTION_EXTENDER ){

                        //
                        //  isTrigger is set true when
                        //
                        if( trigger->objectTigger[j]->isTriggered == true ){
                            nTriggered++;
                        }
                    }
                }

                bool exTrig = false;

                if( trigger->byKeyTriggerFlag == true ){
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
                    case 13: keyHit = GetAsyncKeyState(VK_NUMPAD0); break;
                    case 14: keyHit = GetAsyncKeyState(VK_NUMPAD1); break;
                    case 15: keyHit = GetAsyncKeyState(VK_NUMPAD2); break;
                    case 16: keyHit = GetAsyncKeyState(VK_NUMPAD3); break;
                    case 17: keyHit = GetAsyncKeyState(VK_NUMPAD4); break;
                    case 18: keyHit = GetAsyncKeyState(VK_NUMPAD5); break;
                    case 19: keyHit = GetAsyncKeyState(VK_NUMPAD6); break;
                    case 20: keyHit = GetAsyncKeyState(VK_NUMPAD7); break;
                    case 21: keyHit = GetAsyncKeyState(VK_NUMPAD8); break;
                    case 22: keyHit = GetAsyncKeyState(VK_NUMPAD9); break;
                    }
                    if( keyHit != 0 ){
                        exTrig = true;
                    }
                }

                if( trigger->byExternalTriggerFlag == true ){
                    if( trigger->extTriggerFlagState == true ){
                        exTrig = true;
                    }
                }


                //qDebug() << "nTriggered = " << nTriggered << " size = " << trigger->objectTigger.size();

                if( exTrig == false ){
                    if( trigger->combination == 0 && (nTriggered < trigger->objectTigger.size() || trigger->objectTigger.size() == 0) ){
                        continue;
                    }
                    else if( trigger->combination == 1 && nTriggered == 0 ){
                        continue;
                    }
                }

            }
            else{
                qDebug() << "trigeer->combination is negative. data format of scenario data will be old. Should be updated. [EventID="
                         << currentEvent->eventID << "]";
                qDebug() << "Change to combination is 0";

                trigger->combination = 0;

                continue;
            }


            //
            // Event triggerred
            qDebug() << "Event[" << currentEvent->eventID << "] triggerred.";

            currentEvent->eventState = 1;
            currentEvent->eventTimeCount = 0;
            currentEvent->eventTimeCount_sub = 0;
        }


        if( currentEvent->eventTrigger->combination >= 0 ){

            if( currentEvent->eventType == SCENARIO_EVENT_TYPE::SYSTEM_EVENT ){

                if( currentEvent->eventKind == SYSTEM_SCENARIO_ACTION_KIND::WARP ){


                    qDebug() << "Set Object Position Event:";
                    qDebug() << "  Target Object ID: " << currentEvent->eventIntData[0];
//                    qDebug() << "  X: " << currentEvent->eventFloatData[0];
//                    qDebug() << "  Y: " << currentEvent->eventFloatData[1];
//                    qDebug() << "  Yaw: " << currentEvent->eventFloatData[2];

                    int targetObjectID = currentEvent->eventIntData[0];
                    if( targetObjectID >= 0 && targetObjectID < maxAgentNumber ){

                        // Preserve Lateral Offset
                        bool addOffset = false;
                        float offset = 0.0;
                        float rot_angle = 0.0;

                        if( currentEvent->eventBooleanData.size() >= 3 &&
                                currentEvent->eventBooleanData[2] == false ){  // eventBooleanData[2]: clear lateral offset

//                            qDebug() << " - Keep Lateral Offset";

                            int pathID = pAgent[targetObjectID]->memory.currentTargetPath;

                            float tdev,txt,tyt,txd,tyd,ts;

                            if( posTrigFired == true ){

                                float xt=0.0, yt=0.0, psit=0.0;
                                for(int j=0;j<currentEvent->eventTrigger->objectTigger.size();++j){
                                    if( currentEvent->eventTrigger->objectTigger[j]->triggerType == TRIGGER_TYPE::POSITION_TRIGGER ){
                                        xt = currentEvent->eventTrigger->objectTigger[j]->x;
                                        yt = currentEvent->eventTrigger->objectTigger[j]->y;
                                        psit = currentEvent->eventTrigger->objectTigger[j]->direction;
                                        break;
                                    }
                                }

                                pathID = pRoad->GetNearestPath( xt, yt, psit,
                                                                tdev, txt, tyt, txd, tyd, ts );


                                if( pathID < 0 ){
                                    pathID = pAgent[targetObjectID]->memory.currentTargetPath;
                                }
                            }
                            else{
                                if( currentEvent->targetPathList.size() == 1 ){
                                    pathID = currentEvent->targetPathList[0];
                                    if( pathID < 0 ){
                                        pathID = pAgent[targetObjectID]->memory.currentTargetPath;
                                    }
                                }
                            }

//                            qDebug() << "   pathID = " << pathID;

                            if( pathID >= 0 ){

                                int chk = pRoad->GetDeviationFromPath( pathID,
                                                                       pAgent[targetObjectID]->state.x,
                                                                       pAgent[targetObjectID]->state.y,
                                                                       pAgent[targetObjectID]->state.yaw,
                                                                       tdev, txt, tyt, txd, tyd, ts );

                                if( chk < 0 && pathID != pAgent[targetObjectID]->memory.currentTargetPath ){

                                    pathID = pAgent[targetObjectID]->memory.currentTargetPath;

                                    chk = pRoad->GetDeviationFromPath( pathID,
                                                                       pAgent[targetObjectID]->state.x,
                                                                       pAgent[targetObjectID]->state.y,
                                                                       pAgent[targetObjectID]->state.yaw,
                                                                       tdev, txt, tyt, txd, tyd, ts );
                                }

//                                qDebug() << "   chk = " << chk;

                                if( chk == pathID ){

                                    addOffset = true;
                                    offset = tdev;
                                    float ip = pAgent[targetObjectID]->state.cosYaw * txd + pAgent[targetObjectID]->state.sinYaw * tyd;
                                    float s = pAgent[targetObjectID]->state.cosYaw * tyd * (-1.0) + pAgent[targetObjectID]->state.sinYaw * txd;
                                    if( ip > 1.0 ){
                                        ip = 1.0;
                                    }
                                    else if( ip < -1.0 ){
                                        ip = -1.0;
                                    }
                                    rot_angle = acos( ip );
                                    if( s < 0.0 ){
                                        rot_angle *= -1.0;
                                    }

//                                    qDebug() << "   addOffset; dev = " << offset << " rot = " << rot_angle;

                                }
//                                else{
//                                    qDebug() << "   Cannot calculate Lateral Offset.";
//                                }
                            }
                        }

                        float x = currentEvent->eventFloatData[0];
                        float y = currentEvent->eventFloatData[1];
                        float a = currentEvent->eventFloatData[2];

                        if( addOffset == true && currentEvent->eventIntData.size() >= 2 ){

                            int warpPathID = currentEvent->eventIntData[1];
//                            qDebug() << "   warpPathID = " << warpPathID;

                            float tdev,txt,tyt,txd,tyd,ts;
                            int chk = pRoad->GetDeviationFromPath( warpPathID,
                                                                   x,
                                                                   y,
                                                                   a,
                                                                   tdev, txt, tyt, txd, tyd, ts );

//                            qDebug() << "   chk = " << chk;

                            if( chk == warpPathID ){

//                                qDebug() << "   x = " << x << " y = " << y;
//                                qDebug() << "   offset = " << offset;

                                x = txt + offset * (-tyd);
                                y = tyt + offset *   txd;
                                a = ( atan2( tyd, txd ) + rot_angle ) * 57.3;
                                if( a > 180.0 ){
                                    a -= 360.0;
                                }
                                else if( a < -180.0 ){
                                    a += 360.0;
                                }

//                                qDebug() << "   ---> x = " << x << " y = " << y;
                            }
                        }


                        float deltaMoveX = x - pAgent[targetObjectID]->state.x;
                        float deltaMoveY = y - pAgent[targetObjectID]->state.y;

                        float deltaMovePsi = a * 0.017452 - pAgent[targetObjectID]->state.yaw;  // [rad]

                        float origX = pAgent[targetObjectID]->state.x;
                        float origY = pAgent[targetObjectID]->state.y;

                        float moveToX = x;
                        float moveToY = y;

                        float extractAreaSize = 500.0;

                        // Move Surrunding Object together
                        if( currentEvent->eventBooleanData.size() > 0 &&
                                currentEvent->eventBooleanData[0] == true ){

                            QList<int> removeList;
                            QList<int> warpList;
                            // Remove object around MoveTo Area
                            for(int j=0;j<maxAgentNumber;++j){

                                if( pAgent[j]->agentStatus != 1 ){
                                    continue;
                                }

                                float rx = pAgent[j]->state.x - moveToX;
                                float ry = pAgent[j]->state.y - moveToY;
                                if( fabs(rx) < extractAreaSize && fabs(ry) < extractAreaSize ){
                                    removeList.append( j );
                                }

                                rx = pAgent[j]->state.x - origX;
                                ry = pAgent[j]->state.y - origY;
                                if( fabs(rx) < extractAreaSize && fabs(ry) < extractAreaSize ){
                                    warpList.append( j );
                                }
                            }

                            for(int j=0;j<removeList.size();++j){
                                if( warpList.contains( removeList[j] ) == true ){
                                    continue;
                                }
                                pAgent[ removeList[j] ]->agentStatus = 2;
                            }

                            for(int j=0;j<warpList.size();++j){

                                int wIdx = warpList[j];

                                //qDebug() << "Warp Agent " << pAgent[wIdx]->ID;
                                //qDebug() << "From " << pAgent[wIdx]->state.x << "," << pAgent[wIdx]->state.y;


                                pAgent[wIdx]->state.x += deltaMoveX;
                                pAgent[wIdx]->state.y += deltaMoveY;

                                //qDebug() << "To " << pAgent[wIdx]->state.x << "," << pAgent[wIdx]->state.y;

                                pAgent[wIdx]->state.yaw += deltaMovePsi;

                                pAgent[wIdx]->state.cosYaw = cos( pAgent[wIdx]->state.yaw );
                                pAgent[wIdx]->state.sinYaw = sin( pAgent[wIdx]->state.yaw );


                                // rotation around target vehicle
                                float relx = pAgent[wIdx]->state.x - x;
                                float rely = pAgent[wIdx]->state.y - y;

                                float cosDeltaPsi = cos(deltaMovePsi);
                                float sinDeltaPsi = sin(deltaMovePsi);

                                float rotx = relx * cosDeltaPsi - rely * sinDeltaPsi;
                                float roty = relx * sinDeltaPsi + rely * cosDeltaPsi;

                                pAgent[wIdx]->state.x = x + rotx;
                                pAgent[wIdx]->state.y = y + roty;


                                pAgent[wIdx]->vehicle.SetInitialState( pAgent[wIdx]->state.V,
                                                                       pAgent[wIdx]->state.x,
                                                                       pAgent[wIdx]->state.y,
                                                                       pAgent[wIdx]->state.z,
                                                                       pAgent[wIdx]->state.yaw);


                                if( pAgent[wIdx]->vehicle.yawFiltered4CG != NULL ){
                                    pAgent[wIdx]->vehicle.yawFiltered4CG->SetInitialValue( pAgent[wIdx]->state.yaw );
                                }

                                if( pAgent[wIdx]->agentKind < 100 ){

                                    // Check Route
                                    float deviation,xt,yt,xd,yd,s;
                                    int cpath = pRoad->GetNearestPath( pAgent[wIdx]->state.x, pAgent[wIdx]->state.y, pAgent[wIdx]->state.yaw , deviation, xt, yt, xd, yd, s );
                                    //qDebug() << "cpath = " << cpath;

                                    if( pAgent[wIdx]->memory.targetPathList.indexOf( cpath ) >= 0 ){

                                        pAgent[wIdx]->memory.currentTargetPath = cpath;

                                        pAgent[wIdx]->SetTargetSpeedIndividual( pRoad->paths[ pRoad->pathId2Index.indexOf(cpath) ]->speed85pt );
                                        pAgent[wIdx]->SetTargetNodeListByTargetPaths( pRoad );

                                    }
                                    else{

                                        int dest = pAgent[wIdx]->memory.myNodeList.last();

                                        // Check if the route for same destination
                                        for(int chk=0;chk<2;chk++){

                                            QStringList validRoutes;

                                            for( int k=0;k<pRoad->odRoute.size();++k){

                                                if( chk == 0 ){
                                                    if( pRoad->odRoute[k]->destinationNode != dest ){
                                                        continue;
                                                    }
                                                }

                                                if( pRoad->odRoute[k]->laneListsToDestination.size() > 0 ){
                                                    for(int l=0;l<pRoad->odRoute[k]->laneListsToDestination.size();++l ){
                                                        if( pRoad->odRoute[k]->laneListsToDestination[l].indexOf(cpath) >= 0 ){
                                                            QString vR = QString("0,%1,%2").arg(k).arg(l);
                                                            validRoutes.append( vR );
                                                            break;
                                                        }
                                                    }
                                                }
                                                else if( pRoad->odRoute[k]->LCSupportLaneLists.size() > 0 ){
                                                    bool isSet = false;
                                                    for(int l=0;l<pRoad->odRoute[k]->LCSupportLaneLists.size();++l){
                                                        for(int m=0;m<pRoad->odRoute[k]->LCSupportLaneLists[l]->laneList.size();++m){
                                                            if( pRoad->odRoute[k]->LCSupportLaneLists[l]->laneList[m].indexOf( cpath ) >= 0 ){
                                                                QString vR = QString("1,%1,%2,%3").arg(k).arg(l).arg(m);
                                                                validRoutes.append( vR );
                                                                isSet = true;
                                                            }
                                                        }
                                                        if( isSet == true ){
                                                            break;
                                                        }
                                                    }
                                                }
                                            }

                                            int n = validRoutes.size();

                                            if( n == 0 && chk == 1 ){
                                                // No Route found - dispose
                                                pAgent[wIdx]->agentStatus = 2;
                                            }

                                            if( n > 0 ){

                                                int selIdx = (int)(rndGen.GenUniform() * n);
                                                if( selIdx >= n ){
                                                    selIdx = n - 1;
                                                }

                                                QStringList sel = QString( validRoutes[selIdx] ).split(",");
                                                int type = QString( sel[0] ).trimmed().toInt();
                                                int k = QString( sel[1] ).trimmed().toInt();
                                                int l = QString( sel[2] ).trimmed().toInt();

                                                pAgent[wIdx]->memory.routeIndex = k;

                                                if( type == 0 ){

                                                    pAgent[wIdx]->memory.LCStartRouteIndex = -1;
                                                    pAgent[wIdx]->memory.LCSupportRouteLaneIndex = -1;
                                                    pAgent[wIdx]->memory.routeLaneIndex = l;
                                                    pAgent[wIdx]->memory.targetPathList = pRoad->odRoute[k]->laneListsToDestination[l];
                                                    pAgent[wIdx]->memory.currentTargetPath = cpath;

                                                    pAgent[wIdx]->memory.currentTargetPathIndexInList = -1;
                                                    for(int n = pAgent[wIdx]->memory.targetPathList.size()-1 ; n>=0 ; n--){
                                                        if( pAgent[wIdx]->memory.currentTargetPathIndexInList < 0 &&
                                                                pAgent[wIdx]->memory.targetPathList[n] != pAgent[wIdx]->memory.currentTargetPath ){
                                                            continue;
                                                        }
                                                        pAgent[wIdx]->memory.currentTargetPathIndexInList = n;
                                                        break;
                                                    }

                                                    pAgent[wIdx]->memory.laneMerge.clear();

                                                    for(int m=0;m<pRoad->odRoute[k]->mergeLanesInfo[l].size();++m){

                                                        QPoint pairData;
                                                        pairData.setX( pRoad->odRoute[k]->mergeLanesInfo[l][m].x() );
                                                        pairData.setY( pRoad->odRoute[k]->mergeLanesInfo[l][m].y() );

                                                        pAgent[wIdx]->memory.laneMerge.append( pairData );
                                                    }

                                                    pAgent[wIdx]->memory.targetPathLength.clear();
                                                    for(int n=0;n<pAgent[wIdx]->memory.targetPathList.size();++n){
                                                        float len = pRoad->GetPathLength( pAgent[wIdx]->memory.targetPathList[n] );
                                                        pAgent[wIdx]->memory.targetPathLength.append( len );
                                                    }

                                                    pAgent[wIdx]->SetTargetSpeedIndividual( pRoad->paths[ pRoad->pathId2Index.indexOf(cpath) ]->speed85pt );
                                                    pAgent[wIdx]->SetTargetNodeListByTargetPaths( pRoad );

                                                    pAgent[wIdx]->memory.destinationNode = pAgent[wIdx]->memory.myNodeList.last();

                                                    break;
                                                }
                                                else if( type == 1 ){

                                                    int m = QString( sel[3] ).trimmed().toInt();

                                                    if( pRoad->odRoute[k]->LCSupportLaneLists.size() > 1 && l > 0 ){
                                                        pAgent[wIdx]->memory.LCStartRouteIndex = pRoad->odRoute[k]->LCSupportLaneLists[l]->gIndexInNodeList;
                                                    }
                                                    else{
                                                        pAgent[wIdx]->memory.LCStartRouteIndex = -1;
                                                    }

                                                    pAgent[wIdx]->memory.LCSupportRouteLaneIndex = l;
                                                    pAgent[wIdx]->memory.routeLaneIndex = m;
                                                    pAgent[wIdx]->memory.targetPathList = pRoad->odRoute[k]->LCSupportLaneLists[l]->laneList[m];
                                                    pAgent[wIdx]->memory.currentTargetPath = cpath;

                                                    pAgent[wIdx]->memory.currentTargetPathIndexInList = -1;
                                                    for(int n = pAgent[wIdx]->memory.targetPathList.size()-1 ; n>=0 ; n--){
                                                        if( pAgent[wIdx]->memory.currentTargetPathIndexInList < 0 &&
                                                                pAgent[wIdx]->memory.targetPathList[n] != pAgent[wIdx]->memory.currentTargetPath ){
                                                            continue;
                                                        }
                                                        pAgent[wIdx]->memory.currentTargetPathIndexInList = n;
                                                        break;
                                                    }

                                                    pAgent[wIdx]->memory.laneMerge.clear();

                                                    int MLIidx = 0;
                                                    for(int n=0;n<l;++n){
                                                        MLIidx += pRoad->odRoute[k]->LCSupportLaneLists[n]->laneList.size();
                                                    }
                                                    MLIidx += m;

                                                    for(int n=0;n<pRoad->odRoute[k]->mergeLanesInfo[MLIidx].size();++n){

                                                        QPoint pairData;
                                                        pairData.setX( pRoad->odRoute[k]->mergeLanesInfo[MLIidx][n].x() );
                                                        pairData.setY( pRoad->odRoute[k]->mergeLanesInfo[MLIidx][n].y() );

                                                        pAgent[wIdx]->memory.laneMerge.append( pairData );
                                                    }

                                                    pAgent[wIdx]->memory.targetPathLength.clear();
                                                    for(int n=0;n<pAgent[wIdx]->memory.targetPathList.size();++n){
                                                        float len = pRoad->GetPathLength( pAgent[wIdx]->memory.targetPathList[n] );
                                                        pAgent[wIdx]->memory.targetPathLength.append( len );
                                                    }

                                                    pAgent[wIdx]->SetTargetSpeedIndividual( pRoad->paths[ pRoad->pathId2Index.indexOf(cpath) ]->speed85pt );
                                                    pAgent[wIdx]->SetTargetNodeListByTargetPaths( pRoad );

                                                    pAgent[wIdx]->memory.destinationNode = pAgent[wIdx]->memory.myNodeList.last();


                                                    // Check Lane-change
                                                    if( l > 0 &&
                                                            pAgent[wIdx]->memory.LCStartRouteIndex >= 0 &&
                                                            pAgent[wIdx]->memory.LCStartRouteIndex < pRoad->odRoute[k]->routeToDestination.size() &&
                                                            pAgent[wIdx]->memory.currentTargetNode == pRoad->odRoute[k]->routeToDestination[pAgent[wIdx]->memory.LCStartRouteIndex]->node){


                                                        pAgent[wIdx]->memory.laneChangeTargetPathList.clear();

                                                        int z = l - 1;
                                                        int numLaneLists = pRoad->odRoute[k]->LCSupportLaneLists[z]->laneList.size();

                                                        selIdx = 0;
                                                        if( numLaneLists > 1 ){
                                                            float rnd = rndGen.GenUniform();
                                                            float H = 1.0 / (float)numLaneLists;
                                                            for(int n=0;n<numLaneLists;++n){
                                                                if( rnd >= n * H && rnd < (n+1) * H ){
                                                                    selIdx = n;
                                                                    break;
                                                                }
                                                            }
                                                        }

                                                        if( z > 0 ){
                                                            pAgent[wIdx]->memory.LCStartRouteIndex = pRoad->odRoute[k]->LCSupportLaneLists[z]->gIndexInNodeList;
                                                        }
                                                        else{
                                                            pAgent[wIdx]->memory.LCStartRouteIndex = -1;
                                                        }


                                                        pAgent[wIdx]->memory.routeLaneIndex = selIdx;
                                                        pAgent[wIdx]->memory.laneChangeTargetPathList = pRoad->odRoute[k]->LCSupportLaneLists[z]->laneList[selIdx];


                                                        pAgent[wIdx]->memory.laneChangePathLength.clear();
                                                        for(int n=0;n<pAgent[wIdx]->memory.laneChangeTargetPathList.size();++n){
                                                            float len = pRoad->GetPathLength( pAgent[wIdx]->memory.laneChangeTargetPathList[n] );
                                                            pAgent[wIdx]->memory.laneChangePathLength.append( len );
                                                        }


                                                        pAgent[wIdx]->memory.checkSideVehicleForLC = true;
                                                        pAgent[wIdx]->memory.LCCheckState          = 1;
                                                        pAgent[wIdx]->memory.LCInfoGetCount        = 0;

                                                        // Determine LC Direction
                                                        if( l >= 0 && l < pRoad->odRoute[k]->LCSupportLaneLists.size() ){
                                                            pAgent[wIdx]->memory.LCDirection = pRoad->odRoute[k]->LCSupportLaneLists[l]->LCDirect;
                                                        }

                                                        if( pAgent[wIdx]->vehicle.GetWinerState() == 0 ){
                                                            if( pAgent[wIdx]->memory.LCDirection == DIRECTION_LABEL::LEFT_CROSSING ){
                                                                pAgent[wIdx]->vehicle.SetWinker( 1 );
                                                            }
                                                            else if( pAgent[wIdx]->memory.LCDirection == DIRECTION_LABEL::RIGHT_CROSSING ){
                                                                pAgent[wIdx]->vehicle.SetWinker( 2 );
                                                            }
                                                        }

                                                        pAgent[wIdx]->memory.LCSupportRouteLaneIndex--;
                                                    }

                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                                else{

                                    pAgent[wIdx]->memory.currentTargetPath = -1;
                                    if( pAgent[wIdx]->isScenarioObject == true ){

                                        for(int k=0;k<pRoad->pedestPaths.size();++k){
                                            if( pRoad->pedestPaths[k]->scenarioObjectID == pAgent[wIdx]->ID ){
                                                pAgent[wIdx]->memory.currentTargetPath = pRoad->pedestPaths[k]->id;
                                                break;
                                            }
                                        }

                                        pAgent[wIdx]->memory.currentTargetPathIndexInList = 0;
                                        int overEdge = 0;
                                        float dist = 0.0;
                                        int nearPPSectIndex = pRoad->GetNearestPedestPathSectionIndex(pAgent[wIdx]->state.x,
                                                                                                      pAgent[wIdx]->state.y,
                                                                                                      dist,overEdge,
                                                                                                      pAgent[wIdx]->ID);
                                        if( nearPPSectIndex >= 0 ){
                                            pAgent[wIdx]->memory.currentTargetPathIndexInList = nearPPSectIndex;
                                        }
                                    }
                                    else{

                                        int onPPath = -1;
                                        int ppathSect = -1;
                                        float deviation = 0.0;
                                        float distInSect = 0.0;
                                        bool ret = pRoad->GetNearestPedestPath(pAgent[wIdx]->state.x,
                                                                               pAgent[wIdx]->state.x,
                                                                               pAgent[wIdx]->state.yaw,
                                                                               deviation,
                                                                               onPPath,
                                                                               ppathSect,
                                                                               distInSect);
                                        if( ret == true ){
                                            pAgent[wIdx]->memory.currentTargetPathIndexInList = ppathSect;
                                            pAgent[wIdx]->memory.currentTargetPath = onPPath;
                                        }
                                        else{
                                            qDebug() << "[Warp Event::move with Target Object] can not determine nearest pedestrian path for agent " << pAgent[wIdx]->ID;
                                        }
                                    }

                                    if( pAgent[wIdx]->memory.currentTargetPath >= 0 ){
                                        pAgent[wIdx]->memory.targetPathList.clear();
                                        pAgent[wIdx]->memory.targetPathList.append( pAgent[wIdx]->memory.currentTargetPath );
                                    }
                                }
                            }
                        }

                        if( pAgent[targetObjectID]->isSInterfaceObject == true ){

                            if( udpthread ){

                                // Re:sim coordinate to UE4 coordinate
                                y *= -1.0;
                                a *= -1.0;

                                qDebug() << "Call SendDSMoveCommand: x = " << x << " y = " << y << " a = " << a;
                                udpthread->SendDSMoveCommand( targetObjectID, x, y, a );
                            }

                        }
                        else{

                            pAgent[targetObjectID]->state.x = x;
                            pAgent[targetObjectID]->state.y = y;

                            float yawAngle = a * 0.017452;
                            pAgent[targetObjectID]->state.yaw = yawAngle;
                            pAgent[targetObjectID]->state.cosYaw = cos( yawAngle );
                            pAgent[targetObjectID]->state.sinYaw = sin( yawAngle );

                            pAgent[targetObjectID]->vehicle.SetInitialState( pAgent[targetObjectID]->state.V,
                                                                             pAgent[targetObjectID]->state.x,
                                                                             pAgent[targetObjectID]->state.y,
                                                                             pAgent[targetObjectID]->state.z,
                                                                             pAgent[targetObjectID]->state.yaw);

                            if( pAgent[targetObjectID]->vehicle.yawFiltered4CG != NULL ){
                                pAgent[targetObjectID]->vehicle.yawFiltered4CG->SetInitialValue( pAgent[targetObjectID]->state.yaw );
                            }

                            // get current path
                            if( pAgent[targetObjectID]->memory.routeType == ROUTE_TYPE::PATH_LIST_TYPE ){

                                float tdev,txt,tyt,txd,tyd,ts;
                                int currentPath = pRoad->GetNearestPathFromList( pAgent[targetObjectID]->memory.targetPathList,
                                                                                 pAgent[targetObjectID]->state.x,
                                                                                 pAgent[targetObjectID]->state.y,
                                                                                 pAgent[targetObjectID]->state.z_path,
                                                                                 yawAngle,
                                                                                 tdev,txt,tyt,txd,tyd,ts );
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
                        }
                    }

                    currentEvent->eventState = 2;

                    bool NotRepeatFlag = currentEvent->eventBooleanData[1];
                    if( NotRepeatFlag == true && currentEvent->repeatByFE == true ){
                        NotRepeatFlag = false;
                    }
                    if( NotRepeatFlag == false ){

                        currentEvent->eventState = 0;
                        currentEvent->repeatByFE = false;

                        qDebug() << "Reset event to repeat";

                        for(int j=0;j<currentEvent->eventTrigger->objectTigger.size();++j){
                            currentEvent->eventTrigger->objectTigger[j]->isTriggered = false;
                            currentEvent->eventTrigger->objectTigger[j]->passCheckFlag = 0;
                        }
                    }

                }
                else if( currentEvent->eventKind == SYSTEM_SCENARIO_ACTION_KIND::CHANGE_TRAFFIC_SIGNAL_DISPLAY ){

                    if( currentEvent->eventIntData.size() == 2 ){

                        qDebug() << "Change Traffic Signal:";
                        qDebug() << "  target TS = " << currentEvent->eventIntData[0];
                        qDebug() << "  display index = " << currentEvent->eventIntData[1];

                        QPoint data;
                        data.setX( currentEvent->eventIntData[0] );
                        data.setY( currentEvent->eventIntData[1] );

                        if( currentEvent->eventBooleanData.size() >= 2 ){
                            if( currentEvent->eventBooleanData[0] == true ){
                                data.setY( -1 );
                            }
                            if( currentEvent->eventBooleanData[1] == true ){
                                data.setX( -1 );
                            }
                        }

                        changeTSDisplayInfo.append( data );
                    }

                    currentEvent->eventState = 2;
                }
                else if( currentEvent->eventKind == SYSTEM_SCENARIO_ACTION_KIND::CHANGE_SPEED_INFO ){

                    if( currentEvent->eventFloatData.size() > 0 &&
                            currentEvent->eventIntData.size() > 0 &&
                            currentEvent->eventBooleanData.size() == 2 ){

                        qDebug() << "Change Lane Speed Info:";
                        qDebug() << "  change to : " << currentEvent->eventFloatData[0] << "[km/h]";
                        qDebug() << "  Path : ";
                        for(int j=0;j<currentEvent->eventIntData.size();++j){
                            qDebug() << "  [" << j << "] " << currentEvent->eventIntData[j];
                        }
                        qDebug() << "  Speed Limit: " << currentEvent->eventBooleanData[0];
                        qDebug() << "  Actual Speed: " << currentEvent->eventBooleanData[1];

                        float val = currentEvent->eventFloatData[0] / 3.6;
                        for(int j=0;j<currentEvent->eventIntData.size();++j){

                            int pathID = currentEvent->eventIntData[j];
                            int pIdx = pRoad->pathId2Index.indexOf( pathID );
                            if( pIdx >= 0 ){
                                if( currentEvent->eventBooleanData[0] == true ){
                                    pRoad->paths[pIdx]->speedInfo = val;
                                }
                                if( currentEvent->eventBooleanData[1] == true ){
                                    pRoad->paths[pIdx]->speed85pt = val;
                                }
                            }
                        }
                    }

                    currentEvent->eventState = 2;
                }
                else if( currentEvent->eventKind == SYSTEM_SCENARIO_ACTION_KIND::SEND_UDP_SYSTEM ){

                    QString ipAddr = QString("%1.%2.%3.%4")
                            .arg( currentEvent->eventIntData[0] )
                            .arg( currentEvent->eventIntData[1] )
                            .arg( currentEvent->eventIntData[2] )
                            .arg( currentEvent->eventIntData[3] );

                    int port = currentEvent->eventIntData[4];
                    int nData = currentEvent->eventFloatData.size() / 2;
                    int dataSize = 4 + (nData * 2 + 1) * 4;

                    char *sendData = new char [dataSize + 1];
                    memset( sendData, 0, dataSize + 1 );

                    sendData[0] = 'F';
                    sendData[1] = 'U';
                    sendData[2] = 'm';
                    sendData[3] = 'D';
                    int idx = 4;
                    memcpy( &(sendData[idx]), &nData, sizeof(int) );
                    idx += sizeof(int);

                    qDebug() << "Send UDP to " << ipAddr << ", port = " << port;
                    qDebug() << "nData = " << nData;

                    for(int j=0;j<nData;++j){

                        int ival = (int)(currentEvent->eventFloatData[ 2*j ]);
                        memcpy( &(sendData[idx]), &ival, sizeof(int) );
                        idx += sizeof(int);

                        float fval = currentEvent->eventFloatData[ 2*j+1 ];
                        memcpy( &(sendData[idx]), &fval, sizeof(float) );
                        idx += sizeof(float);

                        qDebug() << " ival = " << ival << " fval = " << fval;
                    }

                    QUdpSocket sock;
                    sock.writeDatagram( sendData, dataSize, QHostAddress(ipAddr), port);
                    sock.flush();
                    sock.writeDatagram( sendData, dataSize, QHostAddress(ipAddr), port);  // For in case the data lost
                    sock.flush();

                    currentEvent->eventState++;    // The data will send again twice to ensure arrival of data

                    if( currentEvent->eventState == 2 ){

                        if( currentEvent->eventBooleanData.size() > 0 ){
                            bool repeatFlag = currentEvent->eventBooleanData[0];
                            if( repeatFlag == false && currentEvent->repeatByFE == true ){
                                repeatFlag = true;
                            }
                            if( repeatFlag == true ){

                                currentEvent->eventState = 0;
                                currentEvent->repeatByFE = false;

                                qDebug() << "Reset event to repeat";

                                for(int j=0;j<currentEvent->eventTrigger->objectTigger.size();++j){
                                    currentEvent->eventTrigger->objectTigger[j]->isTriggered = false;
                                    currentEvent->eventTrigger->objectTigger[j]->passCheckFlag = 0;
                                }
                            }
                        }
                    }

                    delete [] sendData;
                }

            }
            else if( currentEvent->eventType == SCENARIO_EVENT_TYPE::OBJECT_EVENT ){

                if( currentEvent->eventKind == OBJECT_SCENARIO_ACTION_KIND::CONTROL_VEHICLE ){

                    if( currentEvent->eventBooleanData.size() >= 3 ){

                        int iParaIdx = 0;
                        int fParaIdx = 0;

                        if( currentEvent->eventBooleanData[0] == true ){ // Change Control Mode

                            int ctrlMd = currentEvent->eventIntData[iParaIdx++];
                            int ctrlTarget = currentEvent->eventIntData[iParaIdx++];

                            float refSpeed = currentEvent->eventFloatData[fParaIdx++];
                            float refHWTime = currentEvent->eventFloatData[fParaIdx++];
                            float refHWDist = currentEvent->eventFloatData[fParaIdx++];
                            float refStopX = currentEvent->eventFloatData[fParaIdx++];
                            float refStopY = currentEvent->eventFloatData[fParaIdx++];

                            int targetObjectID = currentEvent->targetObjectID;
                            if( targetObjectID >= 0 && targetObjectID < maxAgentNumber ){

                                switch(ctrlMd){
                                case 0: pAgent[targetObjectID]->memory.controlMode = AGENT_CONTROL_MODE::AGENT_LOGIC; break;
                                case 1: pAgent[targetObjectID]->memory.controlMode = AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY; break;
                                case 2: pAgent[targetObjectID]->memory.controlMode = AGENT_CONTROL_MODE::SPEED_PROFILE; break;
                                case 3: pAgent[targetObjectID]->memory.controlMode = AGENT_CONTROL_MODE::STOP_AT; break;
                                default: pAgent[targetObjectID]->memory.controlMode = AGENT_CONTROL_MODE::AGENT_LOGIC; break;
                                }

                                pAgent[targetObjectID]->memory.precedingVehicleIDByScenario = ctrlTarget;
                                pAgent[targetObjectID]->memory.targetSpeedByScenario = refSpeed / 3.6;
                                pAgent[targetObjectID]->memory.setTargetSpeedByScenarioFlag = true;
                                pAgent[targetObjectID]->memory.targetHeadwayTimeByScenario = refHWTime;
                                pAgent[targetObjectID]->memory.targetHeadwayDistanceByScenario = refHWDist;
                                pAgent[targetObjectID]->memory.targetStopAtX = refStopX;
                                pAgent[targetObjectID]->memory.targetStopAtY = refStopY;
                                pAgent[targetObjectID]->memory.actualStopOnPathID = -1;

                                int nPoint = currentEvent->eventIntData[iParaIdx++];

                                if( pAgent[targetObjectID]->memory.protectProfileData == false ){
                                    pAgent[targetObjectID]->memory.profileTime.clear();
                                    pAgent[targetObjectID]->memory.profileSpeed.clear();
                                }
                                pAgent[targetObjectID]->memory.speedProfileCount = 0;

                                if( pAgent[targetObjectID]->memory.controlMode == AGENT_CONTROL_MODE::SPEED_PROFILE && nPoint > 0 && pAgent[targetObjectID]->memory.protectProfileData == false ){

                                    for(int k=0;k<nPoint;++k){

                                        pAgent[targetObjectID]->memory.profileTime.append( currentEvent->eventFloatData[fParaIdx++] );
                                        pAgent[targetObjectID]->memory.profileSpeed.append( currentEvent->eventFloatData[fParaIdx++] / 3.6 );

                                    }
                                }
                            }
                        }

                        if( currentEvent->eventBooleanData[1] == true ){ // Accel/Brake Control

                            int nPoint = currentEvent->eventIntData[iParaIdx++];
                            int targetObjectID = currentEvent->targetObjectID;
                            if( targetObjectID >= 0 && targetObjectID < maxAgentNumber ){

                                pAgent[targetObjectID]->memory.ADDisturbFlag = true;
                                pAgent[targetObjectID]->memory.ADDisturbCount = 0;

                                for(int k=0;k<nPoint;++k){

                                    pAgent[targetObjectID]->memory.ADDisturbTime.append( currentEvent->eventFloatData[fParaIdx++] );
                                    pAgent[targetObjectID]->memory.ADDisturb.append( currentEvent->eventFloatData[fParaIdx++] );

                                }
                            }
                        }

                        if( currentEvent->eventBooleanData[2] == true ){ // Steering Control

                            int nPoint = currentEvent->eventIntData[iParaIdx++];
                            int targetObjectID = currentEvent->targetObjectID;
                            if( targetObjectID >= 0 && targetObjectID < maxAgentNumber ){

                                pAgent[targetObjectID]->memory.steerDisturbFlag = true;
                                pAgent[targetObjectID]->memory.steerDisturbCount = 0;
                                pAgent[targetObjectID]->memory.steerDisturbInit = pAgent[targetObjectID]->memory.steer;

                                for(int k=0;k<nPoint;++k){

                                    pAgent[targetObjectID]->memory.steerDisturbTime.append( currentEvent->eventFloatData[fParaIdx++] );
                                    pAgent[targetObjectID]->memory.steerDisturb.append( currentEvent->eventFloatData[fParaIdx++] );

                                }
                            }
                        }
                    }

                    currentEvent->eventState = 2;

                    if( currentEvent->eventBooleanData.size() >= 4 ){
                        bool repeatFlag = currentEvent->eventBooleanData[3];
                        if( repeatFlag == false && currentEvent->repeatByFE == true ){
                            repeatFlag = true;
                        }
                        if( repeatFlag == true ){

                            currentEvent->eventState = 0;
                            currentEvent->repeatByFE = false;

                            qDebug() << "Reset event to repeat";

                            for(int j=0;j<currentEvent->eventTrigger->objectTigger.size();++j){
                                currentEvent->eventTrigger->objectTigger[j]->isTriggered = false;
                                currentEvent->eventTrigger->objectTigger[j]->passCheckFlag = 0;
                            }
                        }
                    }
                }
                else if( currentEvent->eventKind == OBJECT_SCENARIO_ACTION_KIND::CONTROL_PEDESTRIAN ){

                    if( currentEvent->eventIntData.size() >= 2 ){

                        int ctrlMd = currentEvent->eventIntData[0];
                        int nPoint = currentEvent->eventIntData[1];

                        if( currentEvent->eventFloatData.size() >= 2 + 2 * nPoint ){

                            int fParaIdx = 0;
                            float refSpeed = currentEvent->eventFloatData[fParaIdx++];
                            float runOutDir = currentEvent->eventFloatData[fParaIdx++];

                            int targetObjectID = currentEvent->targetObjectID;
                            if( targetObjectID >= 0 && targetObjectID < maxAgentNumber ){

                                switch(ctrlMd){
                                case 0: pAgent[targetObjectID]->memory.controlMode = AGENT_CONTROL_MODE::AGENT_LOGIC; break;
                                case 1: pAgent[targetObjectID]->memory.controlMode = AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY; break;
                                case 2: pAgent[targetObjectID]->memory.controlMode = AGENT_CONTROL_MODE::SPEED_PROFILE; break;
                                case 3: pAgent[targetObjectID]->memory.controlMode = AGENT_CONTROL_MODE::RUN_OUT; break;
                                default: pAgent[targetObjectID]->memory.controlMode = AGENT_CONTROL_MODE::AGENT_LOGIC; break;
                                }

                                pAgent[targetObjectID]->memory.targetSpeedByScenario = refSpeed;

                                pAgent[targetObjectID]->memory.profileTime.clear();
                                pAgent[targetObjectID]->memory.profileSpeed.clear();
                                pAgent[targetObjectID]->memory.speedProfileCount = 0;

                                if( pAgent[targetObjectID]->memory.controlMode == AGENT_CONTROL_MODE::SPEED_PROFILE && nPoint > 0 ){

                                    for(int k=0;k<nPoint;++k){
                                        pAgent[targetObjectID]->memory.profileTime.append( currentEvent->eventFloatData[fParaIdx++] );
                                        pAgent[targetObjectID]->memory.profileSpeed.append( currentEvent->eventFloatData[fParaIdx++] );
                                    }
                                }

                                pAgent[targetObjectID]->memory.steerDisturbFlag = false;
                                pAgent[targetObjectID]->memory.steerDisturbCount = 0;
                                pAgent[targetObjectID]->memory.steerDisturbInit = 0.0;

                                if( pAgent[targetObjectID]->memory.controlMode == AGENT_CONTROL_MODE::RUN_OUT ){

                                    pAgent[targetObjectID]->memory.steerDisturbFlag = true;
                                    pAgent[targetObjectID]->memory.steerDisturbInit = pAgent[targetObjectID]->state.yaw;

                                    pAgent[targetObjectID]->memory.steerDisturbInit += runOutDir * 0.017452;
                                }
                            }
                        }

                    }

                    currentEvent->eventState = 2;

                    if( currentEvent->eventBooleanData.size() > 0 ){
                        bool repeatFlag = currentEvent->eventBooleanData[0];
                        if( repeatFlag == false && currentEvent->repeatByFE == true ){
                            repeatFlag = true;
                        }
                        if( repeatFlag == true ){

                            currentEvent->eventState = 0;
                            currentEvent->repeatByFE = false;

                            qDebug() << "Reset event to repeat";

                            for(int j=0;j<currentEvent->eventTrigger->objectTigger.size();++j){
                                currentEvent->eventTrigger->objectTigger[j]->isTriggered = false;
                                currentEvent->eventTrigger->objectTigger[j]->passCheckFlag = 0;
                            }
                        }
                    }
                }
                else if( currentEvent->eventKind == OBJECT_SCENARIO_ACTION_KIND::SEND_UDP_OBJECT ){

                    QString ipAddr = QString("%1.%2.%3.%4")
                            .arg( currentEvent->eventIntData[0] )
                            .arg( currentEvent->eventIntData[1] )
                            .arg( currentEvent->eventIntData[2] )
                            .arg( currentEvent->eventIntData[3] );

                    int port = currentEvent->eventIntData[4];
                    int nData = currentEvent->eventFloatData.size() / 2;
                    int dataSize = 4 + (nData * 2 + 1) * 4;

                    char *sendData = new char [dataSize + 1];
                    memset( sendData, 0, dataSize + 1 );

                    sendData[0] = 'F';
                    sendData[1] = 'U';
                    sendData[2] = 'm';
                    sendData[3] = 'D';
                    int idx = 4;
                    memcpy( &(sendData[idx]), &nData, sizeof(int) );
                    idx += sizeof(int);

                    for(int j=0;j<nData;++j){

                        int ival = (int)(currentEvent->eventFloatData[ 2*j ]);
                        memcpy( &(sendData[idx]), &ival, sizeof(int) );
                        idx += sizeof(int);

                        float fval = currentEvent->eventFloatData[ 2*j+1 ];
                        memcpy( &(sendData[idx]), &fval, sizeof(float) );
                        idx += sizeof(float);

                    }

                    QUdpSocket sock;
                    sock.writeDatagram( sendData, dataSize, QHostAddress(ipAddr), port);
                    sock.flush();
                    sock.writeDatagram( sendData, dataSize, QHostAddress(ipAddr), port);  // For in case the data lost
                    sock.flush();

                    currentEvent->eventState++;  // The data will send again twice to ensure arrival of data

                    if( currentEvent->eventState == 2 ){

                        if( currentEvent->eventBooleanData.size() > 0 ){
                            bool repeatFlag = currentEvent->eventBooleanData[0];
                            if( repeatFlag == false && currentEvent->repeatByFE == true ){
                                repeatFlag = true;
                            }
                            if( repeatFlag == true ){

                                currentEvent->eventState = 0;
                                currentEvent->repeatByFE = false;

                                qDebug() << "Reset event to repeat";

                                for(int j=0;j<currentEvent->eventTrigger->objectTigger.size();++j){
                                    currentEvent->eventTrigger->objectTigger[j]->isTriggered = false;
                                    currentEvent->eventTrigger->objectTigger[j]->passCheckFlag = 0;
                                }
                            }
                        }
                    }

                    delete [] sendData;
                }
                else if( currentEvent->eventKind == OBJECT_SCENARIO_ACTION_KIND::DISAPPEAR ){

                    int targetObjectID = currentEvent->targetObjectID;
                    if( targetObjectID >= 0 && targetObjectID < maxAgentNumber ){

                        pAgent[targetObjectID]->agentStatus = 2;

                        currentEvent->eventState = 2;

                        if( currentEvent->eventBooleanData.size() > 0 ){

                            bool repeatFlag = currentEvent->eventBooleanData[0];
                            if( repeatFlag == false && currentEvent->repeatByFE == true ){
                                repeatFlag = true;
                            }
                            if( repeatFlag == true ){

                                currentEvent->eventState = 0;
                                currentEvent->repeatByFE = false;

                                qDebug() << "Reset event to repeat";

                                for(int j=0;j<currentEvent->eventTrigger->objectTigger.size();++j){
                                    currentEvent->eventTrigger->objectTigger[j]->isTriggered = false;
                                    currentEvent->eventTrigger->objectTigger[j]->passCheckFlag = 0;
                                }
                            }
                        }
                    }
                }
            }

        }
//        else if( currentEvent->eventTrigger->combination < 0 ){  // Old ver. To Be Deleted.

//            if( currentEvent->eventType == SCENARIO_EVENT_TYPE::SYSTEM_EVENT ){

//                if( currentEvent->eventKind == SYSTEM_EVENT_KIND::SET_OBJECT_POSITION ){

//                    qDebug() << "Set Object Position Event:";
//                    qDebug() << "  Target Object ID: " << currentEvent->eventIntData[0];
//                    qDebug() << "  X: " << currentEvent->eventFloatData[0];
//                    qDebug() << "  Y: " << currentEvent->eventFloatData[1];
//                    qDebug() << "  Z: " << currentEvent->eventFloatData[2];
//                    qDebug() << "  Yaw: " << currentEvent->eventFloatData[3];

//                    int targetObjectID = currentEvent->eventIntData[0];
//                    if( targetObjectID >= 0 && targetObjectID < maxAgentNumber ){

//                        pAgent[targetObjectID]->state.x = currentEvent->eventFloatData[0];
//                        pAgent[targetObjectID]->state.y = currentEvent->eventFloatData[1];
//                        pAgent[targetObjectID]->state.z = currentEvent->eventFloatData[2];

//                        float yawAngle = currentEvent->eventFloatData[3] * 0.017452;
//                        pAgent[targetObjectID]->state.yaw = yawAngle;
//                        pAgent[targetObjectID]->state.cosYaw = cos( yawAngle );
//                        pAgent[targetObjectID]->state.sinYaw = sin( yawAngle );

//                        pAgent[targetObjectID]->vehicle.SetInitialState( pAgent[targetObjectID]->state.V,
//                                                                         pAgent[targetObjectID]->state.x,
//                                                                         pAgent[targetObjectID]->state.y,
//                                                                         pAgent[targetObjectID]->state.z,
//                                                                         pAgent[targetObjectID]->state.yaw);

//                        // get current path
//                        if( pAgent[targetObjectID]->memory.routeType == ROUTE_TYPE::PATH_LIST_TYPE ){

//                            float tdev,txt,tyt,txd,tyd,ts;
//                            int currentPath = pRoad->GetNearestPathFromList( pAgent[targetObjectID]->memory.targetPathList,
//                                                                             pAgent[targetObjectID]->state.x,
//                                                                             pAgent[targetObjectID]->state.y,
//                                                                             pAgent[targetObjectID]->state.z_path,
//                                                                             yawAngle,
//                                                                             tdev,txt,tyt,txd,tyd,ts );
//                            if( currentPath < 0 ){
//                                qDebug() << "[Warning]----------------------------------";
//                                qDebug() << " Scenario Vehicle ID = " << targetObjectID << " cannot determin nearest path from assigned list.";
//                                qDebug() << "   Assigned Path List : ";
//                                for(int j=0;j<pAgent[targetObjectID]->memory.targetPathList.size();++j){
//                                    qDebug() << "           Path " << pAgent[targetObjectID]->memory.targetPathList[j];
//                                }
//                                continue;
//                            }

//                            pAgent[targetObjectID]->memory.currentTargetPath = currentPath;
//                        }
//                        else if( pAgent[targetObjectID]->memory.routeType == ROUTE_TYPE::NODE_LIST_TYPE ){



//                        }

//                    }

//                    currentEvent->eventState = 2;

//                    int repeatFlag = currentEvent->eventIntData[1];
//                    if( repeatFlag == 1 ){
//                        currentEvent->eventState = 0;
//                    }
//                }
//                else if( currentEvent->eventKind == SYSTEM_EVENT_KIND::SEND_UDP_DATA_BY_SYSTEM_EVENT ){


//                    currentEvent->eventState = 2;

//                }
//                else if( currentEvent->eventKind == SYSTEM_EVENT_KIND::CHANGE_ROAD_SPEEDLIMIT ){

//                    currentEvent->eventState = 2;

//                }
//                else if( currentEvent->eventKind == SYSTEM_EVENT_KIND::CHANGE_TRAFFIC_SIGNAL ){

//                    currentEvent->eventState = 2;

//                }

//            }
//            else if( currentEvent->eventType == SCENARIO_EVENT_TYPE::OBJECT_EVENT ){

//                int targetObjectID = currentEvent->targetObjectID;
//                if( targetObjectID < 0 || targetObjectID >= maxAgentNumber ){
//                    continue;
//                }

//                if( currentEvent->eventKind == OBJECT_EVENT_KIND::SUDDEN_DECELERATION ){

//                    //qDebug() << "[OBJECT_EVENT_KIND::SUDDEN_DECELERATION]";
//                    //qDebug() << "  eventFloatData.size = " << currentEvent->eventFloatData.size();
//                    //qDebug() << "  targetObjectID = " << targetObjectID;

//                    pAgent[targetObjectID]->memory.overrideBrakeByScenario = true;

//                    float eventTime = currentEvent->eventTimeCount * simTime.dt;
//                    currentEvent->eventTimeCount++;

//                    int nAccelPoint = (int)currentEvent->eventFloatData[0];
//                    //qDebug() << "  nAccelPoint = " << nAccelPoint;

//                    if( nAccelPoint > 0 && nAccelPoint <= 8 ){
//                        for(int k=1;k<=nAccelPoint;++k){
//                            if( k == 1 ){
//                                pAgent[targetObjectID]->memory.overrideAxControl = (-1.0) * fabs( currentEvent->eventFloatData[2] * eventTime / (currentEvent->eventFloatData[1]) )  * 9.81;
//                            }
//                            else{
//                                float t1 = currentEvent->eventFloatData[2*(k-1)-1];
//                                float t2 = currentEvent->eventFloatData[2*k-1];
//                                float a1 = fabs(currentEvent->eventFloatData[2*(k-1)]) * 9.81;
//                                float a2 = fabs(currentEvent->eventFloatData[2*k]) * 9.81;
//                                //qDebug() << "k=" << k << " t1=" << t1 << " t2=" << t2 << " a1=" << a1 << " a2=" << a2;

//                                if( t1 <= eventTime && eventTime < t2 ){
//                                    float d  = eventTime - t1;
//                                    float dt = t2 - t1;
//                                    float da = a2 - a1;
//                                    pAgent[targetObjectID]->memory.overrideAxControl = (-1.0) * ( a1 + fabs(da) / dt * d );
//                                    //qDebug() << "[1]overrideAxControl = " << pAgent[targetObjectID]->memory.overrideAxControl;
//                                    break;
//                                }
//                                else if( t2 <= eventTime && k == nAccelPoint ){
//                                    pAgent[targetObjectID]->memory.overrideAxControl = (-1.0) * a2;
//                                    //qDebug() << "[2]overrideAxControl = " << pAgent[targetObjectID]->memory.overrideAxControl;
//                                }
//                            }
//                        }
//                    }

//                    if( fabs( currentEvent->eventFloatData[19] ) > 0.01 ){
//                        pAgent[targetObjectID]->memory.overrideSteerByScenario = true;
//                        pAgent[targetObjectID]->memory.overrideSteerControl = currentEvent->eventFloatData[19];
//                    }

//                    int completelyStop = (int)currentEvent->eventFloatData[18];
//                    if( completelyStop == 0 ){
//                        float endTime = currentEvent->eventFloatData[2*nAccelPoint-1];
//                        if( endTime < eventTime ){
//                            pAgent[targetObjectID]->memory.overrideBrakeByScenario = false;
//                            pAgent[targetObjectID]->memory.overrideAxControl = 0.0;
//                            pAgent[targetObjectID]->memory.overrideSteerByScenario = false;
//                            pAgent[targetObjectID]->memory.overrideSteerControl = 0.0;
//                            currentEvent->eventTimeCount     = 0;
//                            currentEvent->eventTimeCount_sub = 0;
//                            currentEvent->eventState = 2;
//                            if( currentEvent->eventTrigger->mode == TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
//                                currentEvent->eventState = (-1) * simTime.exe_freq;
//                            }
//                            continue;
//                        }
//                    }
//                    else{
//                        if( pAgent[targetObjectID]->state.V  < 0.05 ){

//                            float stopTime = currentEvent->eventTimeCount_sub * simTime.dt;
//                            currentEvent->eventTimeCount_sub++;

//                            float targetStopTime = currentEvent->eventFloatData[17];
//                            if( targetStopTime <= stopTime ){
//                                pAgent[targetObjectID]->memory.overrideBrakeByScenario = false;
//                                pAgent[targetObjectID]->memory.overrideAxControl = 0.0;
//                                pAgent[targetObjectID]->memory.overrideSteerByScenario = false;
//                                pAgent[targetObjectID]->memory.overrideSteerControl = 0.0;
//                                currentEvent->eventTimeCount     = 0;
//                                currentEvent->eventTimeCount_sub = 0;
//                                currentEvent->eventState = 2;
//                                if( currentEvent->eventTrigger->mode == TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
//                                    currentEvent->eventState = (-1) * simTime.exe_freq;
//                                }
//                                continue;
//                            }
//                        }
//                    }
//                }
//                else if( currentEvent->eventKind == OBJECT_EVENT_KIND::LANE_CHANGE ){

//                    currentEvent->eventState = 2;

//                }
//                else if( currentEvent->eventKind == OBJECT_EVENT_KIND::LANE_DEPARTURE ){

//                    currentEvent->eventState = 2;

//                }
//                else if( currentEvent->eventKind == OBJECT_EVENT_KIND::TRAFFIC_SIGNAL_VIOLATION ){


//                    currentEvent->eventState = 2;

//                }
//                else if( currentEvent->eventKind == OBJECT_EVENT_KIND::CHANGE_CONTROL_MODE ){

//                    int tmpControlMode = (int)currentEvent->eventFloatData[0];

//                    pAgent[targetObjectID]->memory.controlMode = tmpControlMode;

//                    if( tmpControlMode == AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY
//                            || tmpControlMode == AGENT_CONTROL_MODE::CONSTANT_SPEED_HEADWAY_KEEP_INITIAL_LATERAL_OFFSET ){


//                        pAgent[targetObjectID]->memory.precedingVehicleIDByScenario = (int)currentEvent->eventFloatData[20];

//                        pAgent[targetObjectID]->memory.targetSpeedByScenario = currentEvent->eventFloatData[1] / 3.6;

//                        pAgent[targetObjectID]->memory.targetHeadwayDistanceByScenario = currentEvent->eventFloatData[2];
//                        pAgent[targetObjectID]->memory.targetHeadwayTimeByScenario     = currentEvent->eventFloatData[3];

//                        pAgent[targetObjectID]->memory.doHeadwayDistanceControl = false;  // This flag is set when detected preceding vehicle
//                        pAgent[targetObjectID]->memory.doStopControl = false;

//                    }
//                    else if( tmpControlMode == AGENT_CONTROL_MODE::SPEED_PROFILE ){

//                        pAgent[targetObjectID]->memory.speedProfileCount = 0;

//                        pAgent[targetObjectID]->memory.profileTime.clear();
//                        pAgent[targetObjectID]->memory.profileSpeed.clear();

//                        int nData = (int)currentEvent->eventFloatData[4];
//                        for(int k=0;k<nData;++k){
//                            pAgent[targetObjectID]->memory.profileTime.append( currentEvent->eventFloatData[6+2*k] );
//                            pAgent[targetObjectID]->memory.profileSpeed.append( currentEvent->eventFloatData[6+2*k+1] / 3.6 );
//                        }

//                        pAgent[targetObjectID]->vehicle.SetInitialSpeed( pAgent[targetObjectID]->memory.profileSpeed[0] );
//                        pAgent[targetObjectID]->memory.profileTime[0] = 0.0f;

//                        pAgent[targetObjectID]->state.V = pAgent[targetObjectID]->memory.profileSpeed[0];
//                    }
//                    else if( tmpControlMode == AGENT_CONTROL_MODE::USE_TIME_SERIES_DATA ){


//                    }
//                    else if( tmpControlMode == AGENT_CONTROL_MODE::STOP_AT ){

//                        pAgent[targetObjectID]->memory.targetSpeedByScenario = currentEvent->eventFloatData[1] / 3.6;

//                        pAgent[targetObjectID]->memory.targetStopAtXByScenario = currentEvent->eventFloatData[2];
//                        pAgent[targetObjectID]->memory.targetStopAtYByScenario = currentEvent->eventFloatData[3];

//                        pAgent[targetObjectID]->memory.actualStopOnPathID = -1;

//                        pAgent[targetObjectID]->memory.doStopControl = true;
//                    }
//                    else if( tmpControlMode == AGENT_CONTROL_MODE::SET_STEER_CONTROL_FLAG ){
//                        if( currentEvent->eventFloatData[1] < 0.5 ){
//                            pAgent[targetObjectID]->memory.doSteerControl = false;
//                        }
//                        else if( currentEvent->eventFloatData[1] > 0.5 ){
//                            pAgent[targetObjectID]->memory.doSteerControl = true;
//                        }
//                    }

//                    currentEvent->eventState = 2;
//                    if( currentEvent->eventTrigger->mode == TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
//                        currentEvent->eventState = (-1) * simTime.exe_freq;
//                    }

//                }
//                else if( currentEvent->eventKind == OBJECT_EVENT_KIND::SET_LATERAL_OFFSET ){

//                    pAgent[targetObjectID]->memory.additionalShiftByScenarioEvent = true;

//                    float eventTime = currentEvent->eventTimeCount * simTime.dt;
//                    currentEvent->eventTimeCount++;

//                    float riseTime  = currentEvent->eventFloatData[1];
//                    float holdTime  = currentEvent->eventFloatData[2];
//                    float decayTime = currentEvent->eventFloatData[3];

//                    float t1 = riseTime;
//                    float t2 = t1 + holdTime;
//                    float t3 = t2 + decayTime;

//                    float maxLateralShift = currentEvent->eventFloatData[0];

//                    if( eventTime < t1 ){
//                        pAgent[targetObjectID]->memory.additionalLateralShift = maxLateralShift * eventTime / t1;
//                    }
//                    else if( eventTime < t2 ){
//                        pAgent[targetObjectID]->memory.additionalLateralShift = maxLateralShift;
//                    }
//                    else if( eventTime < t3 ){
//                        pAgent[targetObjectID]->memory.additionalLateralShift = maxLateralShift * (1.0 - (eventTime-t2)/(t3-t2));
//                    }
//                    else{
//                        pAgent[targetObjectID]->memory.additionalShiftByScenarioEvent = false;
//                        pAgent[targetObjectID]->memory.additionalLateralShift = 0.0;
//                        currentEvent->eventTimeCount     = 0;
//                        currentEvent->eventTimeCount_sub = 0;
//                        currentEvent->eventState = 2;
//                        if( currentEvent->eventTrigger->mode == TRIGGER_TYPE::BY_KEYBOARD_FUNC_KEY ){
//                            currentEvent->eventState = (-1) * simTime.exe_freq;
//                        }
//                    }
//                }
//                else if( currentEvent->eventKind == OBJECT_EVENT_KIND::SEND_UDP_DATA_BY_OBJECT_EVENT ){


//                    currentEvent->eventState = 2;

//                }
//                else if( currentEvent->eventKind == OBJECT_EVENT_KIND::HEADLIGHT ){

//                    currentEvent->eventState = 2;

//                }
//                else if( currentEvent->eventKind == OBJECT_EVENT_KIND::VEHICLE_LAMPS ){

//                    currentEvent->eventState = 2;
//                }
//            }
//        }
    }
}


void SimulationManager::SetScenarioTriggeredByFuncExtend(int eventType,int id,int idx,int option)
{
    if( currentScenarioID < 0 || currentScenarioID >= scenario.size() ){
        return;
    }

    struct ScenarioData* currentScenario = scenario[currentScenarioID];
    int nEvent = currentScenario->scenarioEvents.size();
    if( nEvent == 0 ){
        return;
    }

    qDebug() << "[SetScenarioTriggeredByFuncExtend]";

    int slot = 0;

    for(int i=0;i<nEvent;++i){

        struct ScenarioEvents* currentEvent = currentScenario->scenarioEvents[i];

        qDebug() << " Event Data[" << i << "] eventState = "
                 << currentEvent->eventState
                 << " eventType = " << currentEvent->eventType
                 << " eventID = " << currentEvent->eventID;

        if( currentEvent->eventState == 2 ){   // Already finished
            qDebug() << "Already Finished.";
            continue;
        }

        // Except vehicle or pedestrian appear event
        if( eventType == 0 && currentEvent->eventType != SCENARIO_EVENT_TYPE::SYSTEM_EVENT ){
            qDebug() << "not system event";
            continue;
        }
        if( eventType > 0 && currentEvent->eventType == SCENARIO_EVENT_TYPE::SYSTEM_EVENT ){
            qDebug() << "not object event";
            continue;
        }

        if( eventType == 0 && currentEvent->eventID == id ){
            struct ScenarioTriggers* trigger = currentEvent->eventTrigger;
            if( trigger->combination >= 0 ){

                qDebug() << "Check Trigger condition";

                for(int j=0;j<trigger->objectTigger.size();++j){
                    if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::BY_FUNCTION_EXTENDER ){

                        qDebug() << "Event Trigger set to 'true', System Event, eventID = " << id;

                        trigger->objectTigger[j]->isTriggered = true;
                        currentEvent->repeatByFE = true;

                        if( currentEvent->eventKind == SYSTEM_SCENARIO_ACTION_KIND::WARP ){
                            if( currentEvent->eventBooleanData.size() >= 3 && option == -2 ){
                                currentEvent->eventBooleanData[2] = true;
                            }
                            else{
                                currentEvent->eventBooleanData[2] = false;
                            }

                            currentEvent->targetPathList.clear();

                            if( option >= 0 ){
                                currentEvent->targetPathList.append( option );
                            }
                        }
                        return;
                    }
                }
            }
        }
        else if( eventType > 0 ){

            if( currentEvent->targetObjectID != id ){
                continue;
            }

            if( slot != idx ){
                slot++;
                continue;
            }

            struct ScenarioTriggers* trigger = currentEvent->eventTrigger;
            if( trigger->combination >= 0 ){
                for(int j=0;j<trigger->objectTigger.size();++j){
                    if( trigger->objectTigger[j]->triggerType == TRIGGER_TYPE::BY_FUNCTION_EXTENDER ){

                        qDebug() << "Event Trigger set to 'true', Object Event, targetObjectID = " << id;

                        trigger->objectTigger[j]->isTriggered = true;
                        currentEvent->repeatByFE = true;

                        return;
                    }
                }
            }
        }
    }
}


void SimulationManager::SetScenarioVehicleInitState(int sID,int aID,float x,float y,float z,float psi,float v)
{
    for(int i=0;i<scenario[sID]->scenarioEvents.size();++i){
        if( scenario[sID]->scenarioEvents[i]->targetObjectID != aID ||
                scenario[sID]->scenarioEvents[i]->eventType != SCENARIO_EVENT_TYPE::OBJECT_EVENT ||
                scenario[sID]->scenarioEvents[i]->eventKind != OBJECT_SCENARIO_ACTION_KIND::APPEAR_VEHICLE ){
            continue;
        }

        qDebug() << "SetScenarioVehicleInitState - aID = " << aID;

        scenario[sID]->scenarioEvents[i]->eventFloatData[0] = x;
        scenario[sID]->scenarioEvents[i]->eventFloatData[1] = y;
        scenario[sID]->scenarioEvents[i]->eventFloatData[2] = psi;
        scenario[sID]->scenarioEvents[i]->eventFloatData[3] = v;

        if( scenario[sID]->scenarioEvents[i]->eventTrigger ){
            if( scenario[sID]->scenarioEvents[i]->eventTrigger->objectTigger.size() > 0 ){
                scenario[sID]->scenarioEvents[i]->eventTrigger->objectTigger[0]->triggerType = TRIGGER_TYPE::AT_ONCE;

                qDebug() << "Change trigger type to AT_ONCE";
            }
        }
        break;
    }
}
