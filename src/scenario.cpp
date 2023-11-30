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

#include <QFile>
#include <QTextStream>
#include <QDebug>


void SimulationManager::AllocateScenario(int id)
{
    struct ScenarioData* s = new struct ScenarioData;
    s->scenarioID = id;
    s->status = 0;
    s->repeat = false;
    scenario.append( s );
}


void SimulationManager::SetScenarioEndTrigger(int sID, ScenarioTriggers *trigger)
{
    for(int i=0;i<scenario.size();++i){
        if( scenario[i]->scenarioID == sID ){
            scenario[i]->endTrigger = trigger;
            break;
        }
    }
}


void SimulationManager::SetScenarioRepeatFlag(int sID)
{
    for(int i=0;i<scenario.size();++i){
        if( scenario[i]->scenarioID == sID ){
            scenario[i]->repeat = true;
            break;
        }
    }
}


void SimulationManager::SetScenarioItem(int sID, ScenarioItem *item)
{
    for(int i=0;i<scenario.size();++i){
        if( scenario[i]->scenarioID == sID ){
            scenario[i]->scenarioItems.append( item );
            break;
        }
    }
}


void SimulationManager::SetScenarioEvent(int sID, ScenarioEvents *event)
{
    for(int i=0;i<scenario.size();++i){
        if( scenario[i]->scenarioID == sID ){
            scenario[i]->scenarioEvents.append( event );
            break;
        }
    }
}


struct ScenarioEvents* SimulationManager::GetScenarioEvent(int sID,int targetObjectID)
{
    struct ScenarioEvents* p = NULL;
    for(int i=0;i<scenario.size();++i){
        if( scenario[i]->scenarioID == sID ){
            for(int j=0;j<scenario[i]->scenarioEvents.size();++j){
                if( scenario[i]->scenarioEvents[j]->targetObjectID == targetObjectID ){
                    p = scenario[i]->scenarioEvents[j];
                    break;
                }
            }
        }
    }
    return p;
}


struct ScenarioEvents* SimulationManager::GetScenarioEventAppearVehicle(int sID, int targetObjectID)
{
    struct ScenarioEvents* p = NULL;
    for(int i=0;i<scenario.size();++i){
        if( scenario[i]->scenarioID == sID ){
            for(int j=0;j<scenario[i]->scenarioEvents.size();++j){

                if( scenario[i]->scenarioEvents[j]->eventType != SCENARIO_EVENT_TYPE::OBJECT_EVENT ){
                    continue;
                }
                if( scenario[i]->scenarioEvents[j]->eventKind != OBJECT_SCENARIO_ACTION_KIND::APPEAR_VEHICLE ){
                    continue;
                }

                if( scenario[i]->scenarioEvents[j]->targetObjectID == targetObjectID ){
                    p = scenario[i]->scenarioEvents[j];
                    break;
                }
            }
        }
    }
    return p;
}


void SimulationManager::DumpScenarioData()
{
    qDebug() << "----------- Scenario Data Dump ------------";
    qDebug() << "Number of Scenario Data : " << scenario.size();
    for(int i=0;i<scenario.size();++i){
        qDebug() << "+--- Scenario " << (i+1);

        qDebug() << "  Number of Item : " << scenario[i]->scenarioItems.size();
        qDebug() << "  Number of Event : " << scenario[i]->scenarioEvents.size();


        for(int j=0;j<scenario[i]->scenarioItems.size();++j){
            qDebug() << "    +----- Item " << (j+1);
            qDebug() << "       Type : " << scenario[i]->scenarioItems[j]->type;
            qDebug() << "       ObjectID : " << scenario[i]->scenarioItems[j]->objectID;
            qDebug() << "       Initial Position : " << scenario[i]->scenarioItems[j]->initialState[0] << ","
                     << scenario[i]->scenarioItems[j]->initialState[1] << ","
                     << scenario[i]->scenarioItems[j]->initialState[2];
            qDebug() << "       Initial Heading : " << scenario[i]->scenarioItems[j]->initialState[3] * 57.3;
            qDebug() << "       Initial Speed   : " << scenario[i]->scenarioItems[j]->initialState[4] * 3.6;
            qDebug() << "       Trigger Type    : " << scenario[i]->scenarioItems[j]->appearTriggers->mode;
            qDebug() << "       Trigger Time    : " << scenario[i]->scenarioItems[j]->appearTriggers->timeTrigger->hour
                     << "h" << scenario[i]->scenarioItems[j]->appearTriggers->timeTrigger->min << "m"
                     << scenario[i]->scenarioItems[j]->appearTriggers->timeTrigger->sec << "s"
                     << scenario[i]->scenarioItems[j]->appearTriggers->timeTrigger->msec;
            for(int k=0;k<scenario[i]->scenarioItems[j]->appearTriggers->objectTigger.size();++k){
                qDebug() << "       Trigger Condition " << (k+1);
                qDebug() << "          Target ID : " << scenario[i]->scenarioItems[j]->appearTriggers->objectTigger[k]->targetObjectID;
                qDebug() << "          Position  : "
                         << scenario[i]->scenarioItems[j]->appearTriggers->objectTigger[k]->x << ","
                         << scenario[i]->scenarioItems[j]->appearTriggers->objectTigger[k]->y;
                qDebug() << "          Direction : " << scenario[i]->scenarioItems[j]->appearTriggers->objectTigger[k]->direction * 57.3;
                qDebug() << "          Width     : " << scenario[i]->scenarioItems[j]->appearTriggers->objectTigger[k]->widthHalf;
                qDebug() << "          Speed     : " << scenario[i]->scenarioItems[j]->appearTriggers->objectTigger[k]->speed * 3.6;
                qDebug() << "          TTC       : " << scenario[i]->scenarioItems[j]->appearTriggers->objectTigger[k]->TTC;
            }
            qDebug() << "       Control Mode    : " << scenario[i]->scenarioItems[j]->controlInfo->mode;
            qDebug() << "       Target Speed    : " << scenario[i]->scenarioItems[j]->controlInfo->targetSpeed;
            qDebug() << "       Target HeadwayDistance  : " << scenario[i]->scenarioItems[j]->controlInfo->targetHeadwayDistance;
            qDebug() << "       Target HeadwayTime      : " << scenario[i]->scenarioItems[j]->controlInfo->targetHeadwayTime;
            qDebug() << "       Target Lateral Offset   : " << scenario[i]->scenarioItems[j]->controlInfo->targetLateralOffset;
            qDebug() << "       Target Object ID        : " << scenario[i]->scenarioItems[j]->controlInfo->targetObjectID;
            qDebug() << "       Stop At                 : ( " << scenario[i]->scenarioItems[j]->controlInfo->stopAtX
                     << " , " << scenario[i]->scenarioItems[j]->controlInfo->stopAtY << " )";
            qDebug() << "       Route Type      : " << scenario[i]->scenarioItems[j]->controlInfo->routeType;
            if( scenario[i]->scenarioItems[j]->type == 'v' ){
                if( scenario[i]->scenarioItems[j]->controlInfo->routeType == 0 ){
                    for(int k=0;k<scenario[i]->scenarioItems[j]->controlInfo->nodeRoute.size();++k){
                        qDebug() << "        Node List[" << k << "] :  ("
                                 << scenario[i]->scenarioItems[j]->controlInfo->nodeRoute[k]->inDirect
                                 << ") "
                                 << scenario[i]->scenarioItems[j]->controlInfo->nodeRoute[k]->node
                                 << " ("
                                 << scenario[i]->scenarioItems[j]->controlInfo->nodeRoute[k]->outDirect
                                 << ")";
                    }
                }
                else if( scenario[i]->scenarioItems[j]->controlInfo->routeType == 1 ){
                    for(int k=0;k<scenario[i]->scenarioItems[j]->controlInfo->wpRoute.size();++k){
                        qDebug() << "        WP List[" << k << "] :  "
                                 << scenario[i]->scenarioItems[j]->controlInfo->wpRoute[k]->x << ","
                                 << scenario[i]->scenarioItems[j]->controlInfo->wpRoute[k]->y << ","
                                 << scenario[i]->scenarioItems[j]->controlInfo->wpRoute[k]->z << "  Tht = "
                                 << scenario[i]->scenarioItems[j]->controlInfo->wpRoute[k]->direct * 57.3 << ", V = "
                                 << scenario[i]->scenarioItems[j]->controlInfo->wpRoute[k]->speedInfo * 3.6;
                    }
                }
            }
            else if( scenario[i]->scenarioItems[j]->type == 'p' ){
                for(int k=0;k<scenario[i]->scenarioItems[j]->controlInfo->pedestPathRoute.size();++k){
                    qDebug() << "        PedestPath List[" << k << "] :  "
                             << scenario[i]->scenarioItems[j]->controlInfo->pedestPathRoute[k];
                }
            }
        }

        qDebug() << "+--- Event";
        for(int j=0;j<scenario[i]->scenarioEvents.size();++j){
            qDebug() << "    +----- ID " << scenario[i]->scenarioEvents[j]->eventID;
            qDebug() << "        Type " << (scenario[i]->scenarioEvents[j]->eventType == SCENARIO_EVENT_TYPE::SYSTEM_EVENT ? ": System Event" : ": Object Event");
            if( scenario[i]->scenarioEvents[j]->eventType == SCENARIO_EVENT_TYPE::SYSTEM_EVENT ){
                if( scenario[i]->scenarioEvents[j]->eventKind == 0 ){
                    qDebug() << "        Kind : Warp" ;
                }
                else if( scenario[i]->scenarioEvents[j]->eventKind == 1 ){
                    qDebug() << "        Kind : Change Traffic Signal";
                }
                else if( scenario[i]->scenarioEvents[j]->eventKind == 2 ){
                    qDebug() << "        Kind : Change Speed Info";
                }
                else if( scenario[i]->scenarioEvents[j]->eventKind == 3 ){
                    qDebug() << "        Kind : Send UDP Data";
                }
            }
            else{
                if( scenario[i]->scenarioEvents[j]->eventKind == 0 ){
                    qDebug() << "        Kind : Appear" ;
                }
                else if( scenario[i]->scenarioEvents[j]->eventKind == 1 ){
                    qDebug() << "        Kind : Control";
                }
                else if( scenario[i]->scenarioEvents[j]->eventKind == 2 ){
                    qDebug() << "        Kind : Send UDP Data";
                }
                else if( scenario[i]->scenarioEvents[j]->eventKind == 3 ){
                    qDebug() << "        Kind : Disappear";
                }
            }

            if( scenario[i]->scenarioEvents[j]->eventTrigger->combination >= 0 ){
                if( scenario[i]->scenarioEvents[j]->eventTrigger->mode == 0 ){
                    qDebug() << "        Trigger Mode : Internal";
                }
                else{
                    qDebug() << "        Trigger Mode : External";
                    qDebug() << "        Trigger FuncKey : " << scenario[i]->scenarioEvents[j]->eventTrigger->func_keys;
                }

                if( scenario[i]->scenarioEvents[j]->eventTrigger->combination >= 0 ){
                    for(int k=0;k<scenario[i]->scenarioEvents[j]->eventTrigger->objectTigger.size();++k){
                        qDebug() << "           Trigger[" << k << "]: type = "
                                 << scenario[i]->scenarioEvents[j]->eventTrigger->objectTigger[k]->triggerType;
                    }
                }
            }
            else{
                qDebug() << "        Trigger Mode " << scenario[i]->scenarioEvents[j]->eventTrigger->mode;
                qDebug() << "        Trigger FuncKey " << scenario[i]->scenarioEvents[j]->eventTrigger->func_keys;
                qDebug() << "        Trigger Combination " << scenario[i]->scenarioEvents[j]->eventTrigger->combination;
            }
        }
    }
}

void SimulationManager::CheckScenarioState()
{

}


void SimulationManager::ClearScenarioData()
{
    qDebug() << "[SimulationManager::ClearScenarioData]";

    for(int i=0;i<scenario.size();++i){

        delete scenario[i]->endTrigger;

        //-------

        for(int j=0;j<scenario[i]->scenarioItems.size();++j){

            scenario[i]->scenarioItems[j]->initialState.clear();

            if( scenario[i]->scenarioItems[j]->appearTriggers ){
                for(int k=0;k<scenario[i]->scenarioItems[j]->appearTriggers->objectTigger.size();++k){
                    delete scenario[i]->scenarioItems[j]->appearTriggers->objectTigger[k];
                }
                scenario[i]->scenarioItems[j]->appearTriggers->objectTigger.clear();
                delete scenario[i]->scenarioItems[j]->appearTriggers->timeTrigger;
                delete scenario[i]->scenarioItems[j]->appearTriggers;
            }

            if( scenario[i]->scenarioItems[j]->disappearTriggers ){
                for(int k=0;k<scenario[i]->scenarioItems[j]->disappearTriggers->objectTigger.size();++k){
                    delete scenario[i]->scenarioItems[j]->disappearTriggers->objectTigger[k];
                }
                scenario[i]->scenarioItems[j]->disappearTriggers->objectTigger.clear();
                delete scenario[i]->scenarioItems[j]->disappearTriggers->timeTrigger;
                delete scenario[i]->scenarioItems[j]->disappearTriggers;
            }

            for(int k=0;k<scenario[i]->scenarioItems[j]->controlInfo->nodeRoute.size();++k){
                delete scenario[i]->scenarioItems[j]->controlInfo->nodeRoute[k];
            }
            scenario[i]->scenarioItems[j]->controlInfo->nodeRoute.clear();
            for(int k=0;k<scenario[i]->scenarioItems[j]->controlInfo->wpRoute.size();++k){
                delete scenario[i]->scenarioItems[j]->controlInfo->wpRoute[k];
            }
            scenario[i]->scenarioItems[j]->controlInfo->wpRoute.clear();
            for(int k=0;k<scenario[i]->scenarioItems[j]->controlInfo->ppRouteElem.size();++k){
                delete scenario[i]->scenarioItems[j]->controlInfo->ppRouteElem[k];
            }
            scenario[i]->scenarioItems[j]->controlInfo->ppRouteElem.clear();
            scenario[i]->scenarioItems[j]->controlInfo->pedestPathRoute.clear();
            scenario[i]->scenarioItems[j]->controlInfo->profileAxis.clear();
            scenario[i]->scenarioItems[j]->controlInfo->speedProfile.clear();
            delete scenario[i]->scenarioItems[j]->controlInfo;

            delete scenario[i]->scenarioItems[j];
        }
        scenario[i]->scenarioItems.clear();

        //-------

        for(int j=0;j<scenario[i]->scenarioEvents.size();++j){

            delete scenario[i]->scenarioEvents[j]->eventTrigger->timeTrigger;

            for(int k=0;k<scenario[i]->scenarioEvents[j]->eventTrigger->objectTigger.size();++k){
                delete scenario[i]->scenarioEvents[j]->eventTrigger->objectTigger[k];
            }
            scenario[i]->scenarioEvents[j]->eventTrigger->objectTigger.clear();

            delete scenario[i]->scenarioEvents[j]->eventTrigger;

            scenario[i]->scenarioEvents[j]->eventIntData.clear();
            scenario[i]->scenarioEvents[j]->eventFloatData.clear();
            scenario[i]->scenarioEvents[j]->eventBooleanData.clear();



            if( scenario[i]->scenarioEvents[j]->wpRoute.size() > 0 ){
                for(int k=0;k<scenario[i]->scenarioEvents[j]->wpRoute.size();++k){
                    delete scenario[i]->scenarioEvents[j]->wpRoute[k];
                }
                scenario[i]->scenarioEvents[j]->wpRoute.clear();
            }

            delete scenario[i]->scenarioEvents[j];
        }
        scenario[i]->scenarioEvents.clear();

        //-------

        delete scenario[i];
    }
    scenario.clear();

    qDebug() << "Scenario Data All Cleared.";
}


