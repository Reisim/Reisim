/****************************************************************************
**                                 Re:sim
**
**   Copyright (C) 2020 Misaki Design.LLC
**   Copyright (C) 2020 Jun Tajima <tajima@misaki-design.co.jp>
**
**   This file is part of the Re:sim simulation software
**
**   This file may be used under the terms of the GNU Lesser General Public
**   License version 3 as published by the Free Software Foundation.
**   For more detail, visit https://www.gnu.org/licenses/gpl-3.0.html
**
*************************************************************************** */


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

    remainingTimeToNextDisplay = 0.0;
    elapsedTimeCurrentDisplay  = 0.0;

    flushState = 0;
    flushTimeCheck = 0.0;
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
    return ret;
}


void TrafficSignal::CheckDisplayInfoUpdate(float cSimTimFVal)
{
    if( currentDisplayIndex >= 0 && currentDisplayIndex < displayPattern.size() ){
        if( displayPattern[currentDisplayIndex]->duration < 0.0 ){  // if duration is negative, no update
            return;
        }
    }

    if( cSimTimFVal >= nextUpdateTimeFVal ){
        currentDisplayIndex++;
        if( currentDisplayIndex >= displayPattern.size() ){
            currentDisplayIndex = 0;
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

