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


#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_STATE  4


enum VEHICLE_STATE_VARIABLES
{
    VELOCITY,
    YAW_RATE,
    YAW_ANGLE,
    X_COORD,
    Y_COORD,
    Z_COORD,
    ACCELERATION
};

struct VehicleParam
{
    float length;
    float width;
    float height;
    float wheelBase;
    float distRearAxleToRearEdge;
    float FRWeightRatio;
    float Lf;
    float Lr;
    float distFrontEdgeFromCG;
};


struct VehicleState
{
    float ax;
    float vx;
    float yawRate;
    float yawAngle;
    float X;
    float Y;
    float Z;

    float tireRotAngle;  // for DSMode ; drawing CG
    float relZBody;      // for DSMode
    float relYBody;      // for DSMode
    float relPitchBody;  // for DSMode
    float relRollBody;   // for DSMode
    float pitch;         // for DSMode
    float roll;          // for DSMode
    float XRear;         // for DSMode
    float YRear;         // for DSMode
    float steer;         // for DSMode
};


struct VehicleInput
{
    float accel;
    float brake;
    float steer;
};


class LowPassFilter
{
public:
    LowPassFilter(int dim,float dt,float cutoffFrq,float dampingCoef);
    void SetInput(float u1){ u[0] = u1; }
    void SetInitialValue(float y0){ y[0] = y0; y[1] = y0; y[2] = y0; u[0] = y0; u[1] = y0; u[2] = y0; }
    void SetParam(int dim,float dt,float cutoffFrq,float dampingCoef);
    float GetOutput(){ return y[0]; }
    float GetInput(){ return u[0]; }
    void Update();
    void Reset();

private:
    float u[3];
    float y[3];
    float a[3];
    float b[3];
};


class Vehicle
{
public:
    Vehicle();

    void SetVehicleID(int id){ ID = id; }
    void SetVehicleModelID(int id){ vehicleModelID = id; }
    int  GetVehicleModelID(){ return vehicleModelID; }
    int  GetVehicleType(){ return vehicleType; }

    float GetVehicleLength(){ return param.length; }
    float GetVehicleWidth(){ return param.width; }

    void SetInitialState(float Vinit,
                         float Xinit,
                         float Yinit,
                         float Zinit,
                         float Yawinit);

    void SetInitialSpeed(float Vinit);

    void SetVehicleParam(float Length,
                         float Width,
                         float Height,
                         float WheelBase,
                         float DistRearAxletoRearEdge,
                         float FRWeightRatio);

    void SetVehicleInput(float accel,float brake,float steer);

    void UpdateState( float dt );
    void StateEquation(float *,float *);
    void UpdateStateForDSCG( float dt );

    float GetVehicleState( enum VEHICLE_STATE_VARIABLES sv);

    void SetWinker(int winker);
    int GetWinkerIsBlink();
    int GetWinerState();
    void SetBrakeLampState(int bl);
    int GetBrakeLampState();


    int ID;
    int vehicleModelID;
    int vehicleType;
    struct VehicleInput input;
    struct VehicleParam param;
    struct VehicleState state;

    int winker_count;
    int winker_state;
    int brakelamp;
    int brakelamp_count;
    int headlight;
    float calInterval;

    LowPassFilter *steerFiltered4CG;
    LowPassFilter *axFiltered4CG;
    LowPassFilter *ayFiltered4CG;
    LowPassFilter *suspentionFL4CG;
    LowPassFilter *suspentionFR4CG;
    LowPassFilter *suspentionRL4CG;
    LowPassFilter *suspentionRR4CG;
    LowPassFilter *tireFL4CG;
    LowPassFilter *tireFR4CG;
    LowPassFilter *tireRL4CG;
    LowPassFilter *tireRR4CG;
    LowPassFilter *yawFiltered4CG;

    float axBuf4BrakeLamp[3];
    int axBufIndex;
};

#endif // VEHICLE_H
