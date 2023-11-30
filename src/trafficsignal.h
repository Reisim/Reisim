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


#ifndef TRAFFICSIGNAL_H
#define TRAFFICSIGNAL_H

#include <QList>

//
//  displayInfo
//  Bit:     8          7            6           5             4       3     2    1
//       red-flush yellow-flush right-arrow straight-arrow left-arrow red yellow blue
//

#define  TRAFFICSIGNAL_BLUE                 1
#define  TRAFFICSIGNAL_YELLOW               2
#define  TRAFFICSIGNAL_RED                  4
#define  TRAFFICSIGNAL_LEFT_RED             12
#define  TRAFFICSIGNAL_STRAIGHT_RED         20
#define  TRAFFICSIGNAL_STRAIGHT_LEFT_RED    28
#define  TRAFFICSIGNAL_RIGHT_RED            36
#define  TRAFFICSIGNAL_STRAIGHT_RIGHT_RED   52
#define  TRAFFICSIGNAL_YELLOWFLUSH          64
#define  TRAFFICSIGNAL_REDFLUSH             128

struct SignalDisplayPattern
{
    int displayInfo;
    int duration;
};


class TrafficSignal
{
public:
    TrafficSignal();

    void ClearSignalData();

    void AddSignalDisplayPattern(struct SignalDisplayPattern *);
    void SetLocationInfo(float x,float y,float dir);  // z-axis value is set at that of related node
                                                      // when setting relatedNode calling SetControlInfo
    void SetControlInfo(int nd,int dir,int lane,int cwalk,float z=0.0);

    int GetCurrentDisplayInfo();
    void CheckDisplayInfoUpdate(float simTimeFval, float dt);
    void SetSignalStartTime(float simTimeFval);
    void ForceChangeDisplayTo(float simTimeFval,int targetIndex);

    int GetTimeToDisplay(int signalDisplayIndex);
    void ProceedTime(int hasteTime);
    void SetDisplayOff(){ displayOff = true; }

    int id;

    char type;   // 'v' for vehicles
                 // 'p' for pedestrians

    float xTS;
    float yTS;
    float zTS;
    float direction;

    int relatedNode;
    int controlDirection;
    int controlLane;
    int controlCrossWalk;

    int currentDisplayIndex;
    float nextUpdateTimeFVal;
    float lastUpdateTimeFVal;
    float remainingTimeToNextDisplay;
    float elapsedTimeCurrentDisplay;
    float currentSimTime;

    int startOffset;

    QList<struct SignalDisplayPattern *> displayPattern;

    int flushState;
    float flushTimeCheck;

    bool displayOff;

    int isSensorType;
    int timeChangeBySensor;
    int sensorState;
    float changeToCount;

    bool setSensorPos;
    float sensorX;
    float sensorY;
};

#endif // TRAFFICSIGNAL_H
