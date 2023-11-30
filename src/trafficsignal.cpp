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


#include "trafficsignal.h"
#include <QDebug>


TrafficSignal::TrafficSignal()
{
    xTS = 0.0;
    yTS = 0.0;
    zTS = 0.0;
    direction = 0.0;

    relatedNode      = -1;
    controlDirection = -1;
    controlLane      = -1;
    controlCrossWalk = -1;

    currentDisplayIndex = -1;
    nextUpdateTimeFVal = 0.0;
    lastUpdateTimeFVal = 0.0;
    currentSimTime     = 0.0;

    remainingTimeToNextDisplay = 0.0;
    elapsedTimeCurrentDisplay  = 0.0;

    flushState = 0;
    flushTimeCheck = 0.0;

    displayOff = false;

    isSensorType = 0;
    timeChangeBySensor = 0;
    sensorState = 0;
    changeToCount = 0.0;
    setSensorPos = false;
}


void TrafficSignal::ClearSignalData()
{
    if( displayPattern.size() > 0 ){
        for(int i=0;i<displayPattern.size();++i){
            delete displayPattern[i];
        }
        displayPattern.clear();
    }

    currentDisplayIndex = -1;

    nextUpdateTimeFVal = 0.0;
    lastUpdateTimeFVal = 0.0;

    remainingTimeToNextDisplay = 0.0;
    elapsedTimeCurrentDisplay  = 0.0;
}


void TrafficSignal::AddSignalDisplayPattern(SignalDisplayPattern *dp)
{
    displayPattern.append( dp );
}


void TrafficSignal::SetLocationInfo(float x, float y, float dir)
{
    xTS       = x;
    yTS       = y;
    direction = dir;
}


void TrafficSignal::SetControlInfo(int nd, int dir, int lane, int cwalk,float z)
{
    relatedNode      = nd;
    controlDirection = dir;
    controlLane      = lane;
    controlCrossWalk = cwalk;

    zTS = z;
}


int TrafficSignal::GetCurrentDisplayInfo()
{
    int ret = -1;
    if( currentDisplayIndex >= 0 && currentDisplayIndex < displayPattern.size() ){
        ret = displayPattern[currentDisplayIndex]->displayInfo;
        if( ret == 64 || ret == 128 ){
            if( flushState == 1 ){
                ret = 255;
            }
        }

        if( type == 'p' && ret == 2 ){ // Yellow for Pedestrian Signal -> blue flush
            if( flushState == 1 ){
                ret = 255;
            }
            else{
                ret = 1;
            }
        }
    }

    if( displayOff == true ){
        ret = 0;
    }

    return ret;
}


void TrafficSignal::CheckDisplayInfoUpdate(float cSimTimFVal, float dt)
{
    currentSimTime = cSimTimFVal;

    if( isSensorType > 0 && sensorState != 2 ){
        if( sensorState < 0 ){
            sensorState++;
            remainingTimeToNextDisplay = timeChangeBySensor - sensorState * dt;
        }
        else if( sensorState == 1 ){  // detect vehicle stopping at stop line
            changeToCount += dt;

            //qDebug() << "changeToCount = " << changeToCount;

            if( changeToCount + dt >= timeChangeBySensor ){
                sensorState = 2;
                //qDebug() << "sensorState = " << sensorState;
            }
            remainingTimeToNextDisplay = timeChangeBySensor - changeToCount;
        }
        else{
            remainingTimeToNextDisplay = timeChangeBySensor;
        }
        elapsedTimeCurrentDisplay  = cSimTimFVal - lastUpdateTimeFVal;
        nextUpdateTimeFVal = currentSimTime + dt;
        return;
    }

    if( currentDisplayIndex >= 0 && currentDisplayIndex < displayPattern.size() ){
        if( displayPattern[currentDisplayIndex]->duration < 0.0 ){  // if duration is negative, no update
            return;
        }
    }

    if( cSimTimFVal >= nextUpdateTimeFVal ){
        currentDisplayIndex++;
        if( currentDisplayIndex >= displayPattern.size() ){
            currentDisplayIndex = 0;

            if( isSensorType > 0 ){
                sensorState = -(displayPattern[currentDisplayIndex]->duration) / dt;
                changeToCount = 0.0;
            }
        }
        lastUpdateTimeFVal = nextUpdateTimeFVal;
        nextUpdateTimeFVal += displayPattern[currentDisplayIndex]->duration;

//        qDebug() << "currentDisplayIndex = " << currentDisplayIndex << " size = " << displayPattern.size();
//        qDebug() << " at Time = " << cSimTimFVal << "[sec]";
    }

    remainingTimeToNextDisplay = nextUpdateTimeFVal - cSimTimFVal;
    elapsedTimeCurrentDisplay  = cSimTimFVal - lastUpdateTimeFVal;


    if( cSimTimFVal - flushTimeCheck >= 0.5 ){
        flushState = ( flushState + 1 ) % 2;
        flushTimeCheck = cSimTimFVal;

//        qDebug() << "flushState = " << flushState << " time = " << flushTimeCheck;
    }
}


void TrafficSignal::SetSignalStartTime(float simTimeFval)
{
    //qDebug() << "[TrafficSignal::SetSignalStartTime] : id = " << id << " simTimeFval = " << simTimeFval;

    currentSimTime = simTimeFval;

    int duration = 0;
    currentDisplayIndex = 0;
    while( 1 ){

        duration += displayPattern[currentDisplayIndex]->duration;

        if( duration > simTimeFval ){
            break;
        }
        else{
            currentDisplayIndex++;
            if( currentDisplayIndex >= displayPattern.size() ){
                currentDisplayIndex = 0;
            }
        }
    }

    nextUpdateTimeFVal = duration - simTimeFval;

    elapsedTimeCurrentDisplay  = 0.0;
    flushTimeCheck = simTimeFval;

    //qDebug() << " currentDisplayIndex = " << currentDisplayIndex << " nextUpdateTimeFVal = " << nextUpdateTimeFVal;
}


void TrafficSignal::ForceChangeDisplayTo(float simTimeFval,int targetIndex)
{
    if( targetIndex >= 0 && targetIndex < displayPattern.size()  ){
        currentDisplayIndex = targetIndex;
    }
    else{
        currentDisplayIndex++;
        if( currentDisplayIndex >= displayPattern.size() ){
            currentDisplayIndex = 0;
        }
    }

    lastUpdateTimeFVal = simTimeFval;
    nextUpdateTimeFVal = simTimeFval;

    if( currentDisplayIndex >= 0 && currentDisplayIndex < displayPattern.size() ){
        remainingTimeToNextDisplay = displayPattern[currentDisplayIndex]->duration;
        nextUpdateTimeFVal += remainingTimeToNextDisplay;
    }

    elapsedTimeCurrentDisplay  = 0.0;
}


int TrafficSignal::GetTimeToDisplay(int signalDisplayIndex)
{
    int t = remainingTimeToNextDisplay;

    for(int i=1;i<=displayPattern.size();++i ){

        int idx = (i + currentDisplayIndex) % displayPattern.size();

        if( idx == signalDisplayIndex ){
            break;
        }
        else{
            t += displayPattern[idx]->duration;
        }
    }
    return t;
}


void TrafficSignal::ProceedTime(int hasteTime)
{
    if( hasteTime > remainingTimeToNextDisplay ){
        hasteTime -= remainingTimeToNextDisplay;
        currentDisplayIndex++;
        if( currentDisplayIndex >= displayPattern.size() ){
            currentDisplayIndex = 0;
        }
    }
    else{
        remainingTimeToNextDisplay -= hasteTime;
        nextUpdateTimeFVal = currentSimTime + remainingTimeToNextDisplay;
        lastUpdateTimeFVal = nextUpdateTimeFVal - displayPattern[currentDisplayIndex]->duration;
        return;
    }

    while(1){
        if( hasteTime <= displayPattern[currentDisplayIndex]->duration  ){
            break;
        }
        else{
            hasteTime -= displayPattern[currentDisplayIndex]->duration;
            currentDisplayIndex++;
            if( currentDisplayIndex >= displayPattern.size() ){
                currentDisplayIndex = 0;
            }
        }
    }

    remainingTimeToNextDisplay = displayPattern[currentDisplayIndex]->duration - hasteTime;
    nextUpdateTimeFVal = currentSimTime + remainingTimeToNextDisplay;
    lastUpdateTimeFVal = nextUpdateTimeFVal - displayPattern[currentDisplayIndex]->duration;
}
