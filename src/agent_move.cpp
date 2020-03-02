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


#include "agent.h"
#include <QDebug>

void Agent::UpdateState(Road* pRoad)
{
    if( agentKind < 100 ){

        vehicle.SetVehicleInput( memory.accel, memory.brake, memory.steer );
        vehicle.UpdateState( calInterval );

        if( isSInterfaceObject == false ){
            state.accel = memory.accel;
            state.brake = memory.brake;
            state.steer = memory.steer * 57.3 * 8.0;   // Assume steering gear-ratio 8.0
                                                       // The vehicle model is so simple that only as reference

            state.accel_log = state.accel;
            state.brake_log = state.brake;
            state.steer_log = state.steer;

            if( state.warpFlag == 0 ){
                float dx = state.x - vehicle.state.X;
                float dy = state.y - vehicle.state.Y;
                float L = dx * dx + dy * dy;
                if( L > 400.0 ){
                    state.warpFlag = 1;
                }
            }
        }

        state.x   = vehicle.state.X;
        state.y   = vehicle.state.Y;
        state.z   = vehicle.state.Z;

        if( memory.currentTargetPath >= 0 ){
            int pIdx = pRoad->pathId2Index.indexOf( memory.currentTargetPath );
            if( pIdx >= 0 ){
                float z = pRoad->paths[pIdx]->pos.first()->z();
                float dz = (pRoad->paths[pIdx]->pos.last()->z() - z) * memory.distanceFromStartWPInCurrentPath / pRoad->paths[pIdx]->pathLength;
                z += dz;
                state.z += z;
            }
        }


        state.V   = vehicle.state.vx;
        state.yaw = vehicle.state.yawAngle;
        state.pitch = 0.0;
        state.roll  = 0.0;

        state.cosYaw = cos( state.yaw );
        state.sinYaw = sin( state.yaw );


        if( vehicle.suspentionFL4CG != NULL &&
            vehicle.suspentionFR4CG != NULL &&
            vehicle.suspentionRL4CG != NULL &&
            vehicle.suspentionRR4CG != NULL ){

            vehicle.UpdateStateForDSCG( calInterval );

            state.pitch = vehicle.state.pitch;
            state.roll  = vehicle.state.roll;

            state.z   = vehicle.state.Z;
        }
    }
    else if( agentKind >= 100){

        //
        //  state.V and state.yaw is directly changed at control part
        //
        if( vehicle.axFiltered4CG != NULL && vehicle.ayFiltered4CG != NULL ){

//            float c = cos( state.yaw );
//            float s = sin( state.yaw );

//            vehicle.axFiltered4CG->SetInput( c );
//            vehicle.axFiltered4CG->Update();
//            c = vehicle.axFiltered4CG->GetOutput();

//            vehicle.ayFiltered4CG->SetInput( s );
//            vehicle.ayFiltered4CG->Update();
//            s = vehicle.ayFiltered4CG->GetOutput();

//            state.yaw = atan2( s, c );

            state.z   = vehicle.state.Z;
        }

        state.cosYaw = cos( state.yaw );
        state.sinYaw = sin( state.yaw );

        float ds = state.V * calInterval;

        state.x += ds * state.cosYaw;
        state.y += ds * state.sinYaw;

        state.pitch = 0.0;
        state.roll  = 0.0;
    }
}


