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

        if( isSInterfaceObject == true ){
            return;
        }


        vehicle.SetVehicleInput( memory.accel, memory.brake, memory.steer );
        vehicle.UpdateState( calInterval );

        if( brakeLampOverride >= 0 ){
            vehicle.SetBrakeLampState( brakeLampOverride );
        }

        state.accel = memory.accel;
        state.brake = memory.brake;
        state.steer = memory.steer * 57.3 * 8.0;   // Assume steering gear-ratio 8.0
                                                   // The vehicle model is so simple that only as reference

        state.accel_log = state.accel;
        state.brake_log = state.brake;
        state.steer_log = state.steer;    // The unit of state.steer is [deg]

        if( state.warpFlag == 0 ){
            float dx = state.x - vehicle.state.X;
            float dy = state.y - vehicle.state.Y;
            float L = dx * dx + dy * dy;
            if( L > 400.0 ){
                state.warpFlag = 1;
            }
        }

        state.x   = vehicle.state.X;
        state.y   = vehicle.state.Y;
        state.z   = vehicle.state.Z;

        state.V   = vehicle.state.vx;
        state.yaw = vehicle.state.yawAngle;
        state.pitch = 0.0;
        state.roll  = 0.0;

        state.cosYaw = cos( state.yaw );
        state.sinYaw = sin( state.yaw );


        state.z_path = state.z;
        if( memory.currentTargetPath >= 0 ){
            int pIdx = pRoad->pathId2Index.indexOf( memory.currentTargetPath );
            if( pIdx >= 0 ){

                float z = pRoad->paths[pIdx]->pos.first()->z();
                if( memory.distanceFromStartWPInCurrentPath >= 0 && memory.distanceFromStartWPInCurrentPath <= pRoad->paths[pIdx]->pathLength ){
                    float dz = (pRoad->paths[pIdx]->pos.last()->z() - z) * memory.distanceFromStartWPInCurrentPath / pRoad->paths[pIdx]->pathLength;
                    z += dz;
                }

                state.z_path = z;
            }
        }


        if( vehicle.yawFiltered4CG != NULL ){
            vehicle.yawFiltered4CG->SetInput( state.yaw );
            vehicle.yawFiltered4CG->Update();
        }

        if( vehicle.suspentionFL4CG != NULL &&
            vehicle.suspentionFR4CG != NULL &&
            vehicle.suspentionRL4CG != NULL &&
            vehicle.suspentionRR4CG != NULL ){

            vehicle.UpdateStateForDSCG( calInterval );

            state.pitch = vehicle.state.pitch;
            state.roll  = vehicle.state.roll;

            state.z   = vehicle.state.Z;
            //qDebug() << "id = " << ID << " z = " << state.z;
        }
        else{
            state.z = state.z_path;
        }
    }
    else if( agentKind >= 100){

        //
        //  state.V and state.yaw is directly changed at control part
        //
//        if( vehicle.yawFiltered4CG != NULL ){
//            vehicle.yawFiltered4CG->SetInput( state.yaw );
//            vehicle.yawFiltered4CG->Update();
//        }

//        if( vehicle.axFiltered4CG != NULL ){
//            vehicle.axFiltered4CG->SetInput( state.cosYaw );
//            vehicle.axFiltered4CG->Update();
//            state.cosYaw = vehicle.axFiltered4CG->GetOutput();
//        }

//        if( vehicle.ayFiltered4CG != NULL ){
//            vehicle.ayFiltered4CG->SetInput( state.sinYaw );
//            vehicle.ayFiltered4CG->Update();
//            state.sinYaw = vehicle.ayFiltered4CG->GetOutput();
//        }


        state.yaw = atan2( state.sinYaw, state.cosYaw );

//        state.cosYaw = cos( state.yaw );
//        state.sinYaw = sin( state.yaw );

        float ds = state.V * calInterval;

        state.x += ds * state.cosYaw;
        state.y += ds * state.sinYaw;
        state.z  = vehicle.state.Z;

        state.z_path = state.z;

        state.pitch = 0.0;
        state.roll  = 0.0;
    }
}


