#ifndef TRAFFICSIGNAL_H
#define TRAFFICSIGNAL_H

#include <QList>

//
//  displayInfo
//  Bit:     8          7            6           5             4       3     2    1
//       red-flush yellow-flush left-arrow straight-arrow right-arrow red yellow blue
//

#define  TRAFFICSIGNAL_BLUE                 1
#define  TRAFFICSIGNAL_YELLOW               2
#define  TRAFFICSIGNAL_RED                  4
#define  TRAFFICSIGNAL_RIGHT_RED            12
#define  TRAFFICSIGNAL_STRAIGHT_RED         20
#define  TRAFFICSIGNAL_STRAIGHT_RIGHT_RED   28
#define  TRAFFICSIGNAL_LEFT_RED             36
#define  TRAFFICSIGNAL_STRAIGHT_LEFT_RED    52
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
    void CheckDisplayInfoUpdate(float simTimeFval);
    void SetSignalStartTime(float simTimeFval);
    void ForceChangeDisplayTo(float simTimeFval,int targetIndex);

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

    int startOffset;

    QList<struct SignalDisplayPattern *> displayPattern;

    int flushState;
    float flushTimeCheck;
};

#endif // TRAFFICSIGNAL_H
