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


#include "vehicle.h"
#include <QDebug>


Vehicle::Vehicle()
{
    memset( &input, 0, sizeof(input) );
    memset( &state, 0, sizeof(state) );

    param.length = 4.0;
    param.width  = 1.695;
    param.height = 1.55;
    param.wheelBase = 2.53;
    param.FRWeightRatio = 1.778;
    param.distRearAxleToRearEdge = 0.68;

    param.Lf = 1.0f / (1.0f + param.FRWeightRatio) * param.wheelBase;
    param.Lr = param.wheelBase - param.Lf;
    param.distFrontEdgeFromCG = param.length - param.distRearAxleToRearEdge - param.Lr;

    winker_count = 0;
    winker_state = 0;
    brakelamp    = 0;
    brakelamp_count = 0;
    headlight    = 0;
    calInterval = 0.02;

    vehicleModelID = -1;

    steerFiltered4CG = NULL;
    axFiltered4CG    = NULL;
    ayFiltered4CG    = NULL;
    suspentionFL4CG  = NULL;
    suspentionFR4CG  = NULL;
    suspentionRL4CG  = NULL;
    suspentionRR4CG  = NULL;
    tireFL4CG  = NULL;
    tireFR4CG  = NULL;
    tireRL4CG  = NULL;
    tireRR4CG  = NULL;
    yawFiltered4CG = NULL;

    axBuf4BrakeLamp[0] = 0.0;
    axBuf4BrakeLamp[1] = 0.0;
    axBuf4BrakeLamp[2] = 0.0;
    axBufIndex = 0;
}


void Vehicle::SetVehicleInput(float accel, float brake, float steer)
{
    input.accel = accel;
    input.brake = brake;
    input.steer = steer;
}


void Vehicle::SetWinker(int winker)
{
    if( winker_state != winker ){
        winker_state = winker;
        winker_count = 0;
    }
}


void Vehicle::SetInitialState(float Vinit,
                              float Xinit,
                              float Yinit,
                              float Zinit,
                              float Yawinit)   // in [rad]
{
    state.vx = Vinit;
    state.yawAngle = Yawinit;
    state.X = Xinit;
    state.Y = Yinit;
    state.Z = Zinit;
}


void Vehicle::SetInitialSpeed(float Vinit)
{
    state.vx = Vinit;
}


void Vehicle::SetVehicleParam(float Length,
                              float Width,
                              float Height,
                              float WheelBase,
                              float DistRearAxletoRearEdge,
                              float FRWeightRatio)
{
    param.length                 = Length;
    param.width                  = Width;
    param.height                 = Height;
    param.wheelBase              = WheelBase;
    param.distRearAxleToRearEdge = DistRearAxletoRearEdge;
    param.FRWeightRatio          = FRWeightRatio;

    param.Lf = 1.0f / (1.0f + param.FRWeightRatio) * param.wheelBase;
    param.Lr = param.wheelBase - param.Lf;
    param.distFrontEdgeFromCG = param.length - param.distRearAxleToRearEdge - param.Lr;
}


float Vehicle::GetVehicleState(VEHICLE_STATE_VARIABLES sv)
{
    float ret = 0.0;

    switch(sv){
    case VEHICLE_STATE_VARIABLES::VELOCITY : ret = state.vx; break;
    case VEHICLE_STATE_VARIABLES::YAW_RATE : ret = state.yawRate; break;
    case VEHICLE_STATE_VARIABLES::YAW_ANGLE : ret = state.yawAngle; break;
    case VEHICLE_STATE_VARIABLES::X_COORD : ret = state.X; break;
    case VEHICLE_STATE_VARIABLES::Y_COORD : ret = state.Y; break;
    case VEHICLE_STATE_VARIABLES::Z_COORD: ret = state.Z; break;
    case VEHICLE_STATE_VARIABLES::ACCELERATION : ret = state.ax; break;
    }

    return ret;
}


void Vehicle::UpdateState(float dt)
{
    float hh = 0.0f,h6 = 0.0f;
    float dym[MAX_STATE];
    float dyt[MAX_STATE];
    float yt[MAX_STATE];
    float y[MAX_STATE];
    float dydx[MAX_STATE];


    calInterval = dt;


    //
    //  Input
    state.ax       = input.accel - input.brake;
    state.yawRate  = input.steer * 0.3;


    if( state.vx > 0.01 || state.ax > 0.0 ){

        //
        //  Copy state variables
        y[0] = state.vx;
        y[1] = state.yawAngle;
        y[2] = state.X;
        y[3] = state.Y;


        hh = dt * 0.5f;
        h6 = dt * 0.16666667f;

        //
        StateEquation(y,dydx);


        //
        for(int i=0;i<MAX_STATE;++i){
            yt[i] = y[i] + hh * dydx[i];
        }
        StateEquation(yt,dyt);

        //
        for(int i=0;i<MAX_STATE;++i){
            yt[i] = y[i] + hh * dyt[i];
        }
        StateEquation(yt,dym);

        //
        for(int i=0;i<MAX_STATE;++i){
            yt[i] = y[i] + dt * dym[i];
            dym[i] += dyt[i];
        }
        StateEquation(yt,dyt);

        //
        //  Final Stop
        for(int i=0;i<MAX_STATE;++i){
            y[i] += h6*(dydx[i]+dyt[i]+dym[i]+dym[i]);
        }

        // Vehicle speed should not be negative
        if( y[0] < 0.0 ){
            y[0] = 0.0;
        }


        state.vx       = y[0];
        state.yawAngle = y[1];
        state.X        = y[2];
        state.Y        = y[3];

    }


    winker_count++;
    if( winker_count * calInterval >= 1.0 ){
        winker_count = 0;
    }


    axBuf4BrakeLamp[axBufIndex] = state.ax;
    axBufIndex++;
    if(axBufIndex == 3){
        axBufIndex = 0;
    }

    float maxAx = axBuf4BrakeLamp[0];
    float minAx = axBuf4BrakeLamp[0];
    float aveAx = axBuf4BrakeLamp[0];
    for(int i=1;i<3;++i){
        aveAx += axBuf4BrakeLamp[i];
        if( maxAx < axBuf4BrakeLamp[i] ){
            maxAx = axBuf4BrakeLamp[i];
        }
        if( minAx > axBuf4BrakeLamp[i] ){
            minAx = axBuf4BrakeLamp[i];
        }
    }
    aveAx -= maxAx;
    aveAx -= minAx;


    brakelamp = 0;
    if( aveAx < (-0.08) * 9.81 ){
        brakelamp = 1;
        brakelamp_count = (int)(0.2 / calInterval);
    }
    if( state.vx < 0.001 || ( aveAx < 0.0 && state.vx < 1.2 ) ){
        brakelamp = 1;
        brakelamp_count = (int)(0.2 / calInterval);
    }

    if( brakelamp_count > 0 ){
        brakelamp = 1;
        brakelamp_count--;
    }
}


void Vehicle::UpdateStateForDSCG(float dt)
{

    //
    //  Position of Rear Center
    //
    state.XRear = state.X - cos(state.yawAngle) * param.Lr;
    state.YRear = state.Y - sin(state.yawAngle) * param.Lr;


    //
    //  Filtering steering angle for smoothing
    //
    if( steerFiltered4CG != NULL ){
        steerFiltered4CG->SetInput( input.steer );
        steerFiltered4CG->Update();
        state.steer = steerFiltered4CG->GetOutput();
    }


    //
    //  Tire Rotation
    //
    float deltaAngle = state.vx / 0.270 * dt * 57.3;
    state.tireRotAngle += deltaAngle;
    while(1){
        if( state.tireRotAngle > 180.0 ){
            state.tireRotAngle -= 360.0;
        }
        else if( state.tireRotAngle < -180.0 ){
            state.tireRotAngle += 360.0;
        }
        else{
            break;
        }
    }


    //
    //  Suspention, Relative motion of Body to Tire
    //
    tireFL4CG->Update();
    tireFR4CG->Update();
    tireRL4CG->Update();
    tireRR4CG->Update();

    suspentionFL4CG->Update();
    suspentionFR4CG->Update();
    suspentionRL4CG->Update();
    suspentionRR4CG->Update();

    float zFL = tireFL4CG->GetOutput();
    float zFR = tireFR4CG->GetOutput();
    float zRL = tireRL4CG->GetOutput();
    float zRR = tireRR4CG->GetOutput();

    float zf = (zFL + zFR) * 0.5;
    float zr = (zRL + zRR) * 0.5;

    float lf = param.Lf;
    float lr = param.Lr;
    float L = lf + lr;
    float z = (lr * zf + lf * zr) / L;

    state.Z = (tireRL4CG->GetInput() + tireRR4CG->GetInput()) * 0.5;
    state.pitch = (zf - zr) / L;

    float theta_r_f = (zFL - zFR) / 1.8f;
    float theta_r_r = (zRL - zRR) / 1.8f;
    float theta_r = (theta_r_f + theta_r_r) * 0.5;
    state.roll = theta_r;


    float zFLFilRel = suspentionFL4CG->GetOutput();
    float zFRFilRel = suspentionFR4CG->GetOutput();
    float zRLFilRel = suspentionRL4CG->GetOutput();
    float zRRFilRel = suspentionRR4CG->GetOutput();

    zf = (zFLFilRel + zFRFilRel) * 0.5;
    zr = (zRLFilRel + zRRFilRel) * 0.5;
    float zs = (lr * zf + lf * zr) / L;
    state.relPitchBody = (zf - zr) / L * 0.25 - state.pitch;
    state.relZBody = zs - z;

    theta_r_f = (zFLFilRel - zFRFilRel) / 1.8f;
    theta_r_r = (zRLFilRel - zRRFilRel) / 1.8f;
    theta_r = (theta_r_f + theta_r_r) * 0.5;
    state.relRollBody = theta_r - state.roll;


    //
    //  Filtering ax and ay and Calculate Pitch and Roll motion for smoothing
    //
    float ax = state.ax;
    if( axFiltered4CG != NULL ){
        axFiltered4CG->SetInput( ax );
        axFiltered4CG->Update();
        ax = axFiltered4CG->GetOutput();
    }

    if( ax < -0.08 * 9.81 ){
        float addPitch = state.ax;
        if( addPitch < -0.5 * 9.81 ){
            addPitch = -0.5 * 9.81;
        }
        addPitch /= (0.5 * 9.81);
        addPitch *= 0.0;
        addPitch *= 0.017452;

        state.relPitchBody += addPitch;
    }

    float ay = state.yawRate;
    if( ayFiltered4CG != NULL ){
        ayFiltered4CG->SetInput( ay );
        ayFiltered4CG->Update();
        ay = ayFiltered4CG->GetOutput();
    }

    float addRoll = ay;
    addRoll /= 0.3;
    if( addRoll > 1.0 ){
        addRoll = 1.0;
    }
    else if( addRoll < -1.0 ){
        addRoll = -1.0;
    }
    addRoll *= 0.0;
    addRoll *= 0.017452;

    state.relRollBody -= addRoll;

    // take roll center on rear axis
    state.relYBody = 0.57 * addRoll / 57.3 * (-0.7);

}


int Vehicle::GetWinkerIsBlink()
{
    int ret = 0;
    if( winker_count * calInterval < 0.6 ){
        if( winker_state > 0 ){
            ret = winker_state;
        }
    }
    return ret;
}


void Vehicle::SetBrakeLampState(int bl)
{
    brakelamp = bl;
}


int Vehicle::GetBrakeLampState()
{
    return brakelamp;
}


int Vehicle::GetWinerState()
{
    return winker_state;
}


void Vehicle::StateEquation(float* y, float* dxdy)
{
    float V   = y[0];
    float Psi = y[1];

    float cp = cos(Psi);
    float sp = sin(Psi);

    dxdy[0] = state.ax;
    if( V <= 0.0 && dxdy[0] < 0.0 ){
        dxdy[0] = 0.0;
    }
    dxdy[1] = state.yawRate;
    dxdy[2] = V * cp;
    dxdy[3] = V * sp;
}


LowPassFilter::LowPassFilter(int dim,float dt, float cutoffFrq,float dampingCoef)
{
    for(int i=0;i<3;++i){
        u[i] = 0.0;
        y[i] = 0.0;
        a[i] = 0.0;
        b[i] = 0.0;
    }

    SetParam(dim,dt,cutoffFrq,dampingCoef);
}


void LowPassFilter::SetParam(int dim, float dt, float cutoffFrq, float dampingCoef)
{
    if( dim < 0 || dim > 2 ){
        return;
    }

    float wt = dt * cutoffFrq;

    if( dim == 1 ){
        a[1] = (2.0 - wt) / (2.0 + wt);
        b[0] = wt / (2.0 + wt);
        b[1] = b[0];
    }
    else if( dim == 2 ){
        float den = 4.0 + 4.0 * dampingCoef * wt + wt * wt;
        a[1] = (8.0 - 2.0 * wt * wt) / den;
        a[2] = -(4.0 - 4.0 * dampingCoef * wt + wt * wt) / den;
        b[0] = wt * wt / den;
        b[1] = 2.0 * b[0];
        b[2] = b[0];
    }
}


void LowPassFilter::Update()
{
    y[0] = a[1] * y[1] + a[2] * y[2] + b[0] * u[0] + b[1] * u[1] + b[2] * u[2];

    u[2] = u[1];
    u[1] = u[0];

    y[2] = y[1];
    y[1] = y[0];
}


void LowPassFilter::Reset()
{
    for(int i=0;i<3;++i){
        u[i] = 0.0;
        y[i] = 0.0;
    }
}

