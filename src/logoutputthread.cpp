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


#include "logoutputthread.h"
#include <QDebug>

extern QMutex *mutex_log;
extern QWaitCondition *cond_log;

volatile int logThreadOpeFlag = 0;


LogOutputThread::LogOutputThread(QObject *parent) :
    QThread(parent)
{
    logFileName = QString();
    logOutputFlag = false;

    logInterval = 1;
    logIntervalCount = 0;

    expID = QString();
}

LogOutputThread::~LogOutputThread()
{
    if( logOutputFlag ){
        if( fileLog.isOpen() ){
            qDebug() << "[LogOutputThread::~LogOutputThread] file to log output closed.";
            fileLog.close();
        }
    }
}


void LogOutputThread::SetLogFileName(QString filename)
{
    logFileName = filename;
    if( logFileName.isNull() == false && logFileName.isEmpty() == false && logFileName.contains(".") == true ){
        fileLog.setFileName( filename );
        if( fileLog.open( QIODevice::WriteOnly | QIODevice::Text ) == false ){
            QMessageBox::warning(NULL,"Error","Cannot open Log Output File. filename = " + filename );
        }
        else{
            logOutputFlag = true;

            QTextStream logOut(&fileLog);

            if( expID.isNull() == false ){
                logOut << expID
                       << ","
                       << "Time[sec],FE1,FE2,FE3,FE4";

                expID = QString(",");
            }
            else{
                logOut << "Time[sec],FE1,FE2,FE3,FE4";
            }

            for(int i=0;i<numTS;++i){
                logOut << QString(",TS%1").arg( ts[i]->id );
            }
            for(int i=0;i<maxAgent;++i){
                logOut << ",ID,Status,Kind,Accel,Brake,Steer[deg],X[m],Y[m],Yaw[deg],V[km/h],WK,BL,HL,Collison,Warp/LC";
            }
            logOut << "\n";
        }
    }
}


void LogOutputThread::CloseFile()
{
    if( logOutputFlag ){
        if( fileLog.isOpen() ){
            qDebug() << "[LogOutputThread::CloseFile] file to log output closed.";
            fileLog.close();
        }
    }
}


void LogOutputThread::run()
{
    if( logOutputFlag == false ){
        return;
    }

    stopFlag = false;

    while( stopFlag == false ){

        {
            mutex_log->lock();
            if( logThreadOpeFlag == 0 ){
                cond_log->wait(mutex_log);
            }
            logThreadOpeFlag = 0;
            mutex_log->unlock();

        }

        logIntervalCount++;
        if( logIntervalCount < logInterval ){
            continue;
        }
        logIntervalCount = 0;

        QTextStream logOut(&fileLog);

        float simTimeSec = simManage->GetSimulationTimeInSec();

        logOut << expID;
        logOut << simTimeSec;

        for(int i=0;i<4;++i){
            logOut << "," << FEData[i];
        }

        for(int i=0;i<numTS;++i){
            logOut << "," << ts[i]->GetCurrentDisplayInfo();
        }

        for(int i=0;i<maxAgent;++i){
            if( agent[i]->agentStatus != 1 ){
                logOut << "," << agent[i]->ID
                       << "," << agent[i]->agentStatus
                       << ",,,,,,,,,,,,,";
            }
            else{
                logOut << "," << agent[i]->ID
                       << "," << agent[i]->agentStatus
                       << "," << agent[i]->agentKind
                       << "," << agent[i]->state.accel_log
                       << "," << agent[i]->state.brake_log
                       << "," << agent[i]->state.steer_log
                       << "," << agent[i]->state.x
                       << "," << agent[i]->state.y
                       << "," << (agent[i]->state.yaw * 57.3)
                       << "," << (agent[i]->state.V * 3.6)
                       << "," << agent[i]->vehicle.winker_state
                       << "," << agent[i]->vehicle.brakelamp
                       << "," << agent[i]->vehicle.headlight;

                // check collision
                int collision = -1;
                for(int j=0;j<maxAgent;++j){
                    if( j == i ){
                        continue;
                    }
                    if( agent[i]->agentStatus != 1 ){
                        continue;
                    }
                    float dx = agent[j]->state.x - agent[i]->state.x;
                    if( dx > 5.0 || dx < -5.0 ){
                        continue;
                    }
                    float dy = agent[j]->state.y - agent[i]->state.y;
                    if( dy > 5.0 || dy < -5.0 ){
                        continue;
                    }
                    float R = dx * dx + dy * dy;
                    if( R < 1.5 ){
                        collision = agent[j]->ID;
                        break;
                    }
                }
                logOut << "," << collision;
                logOut << "," << (agent[i]->state.warpFlag + agent[i]->memory.LCCheckState);
                if( agent[i]->state.warpFlag == 1 ){
                    agent[i]->state.warpFlag = 0;
                }
            }
        }
        logOut << "\n";
    }
}


void LogOutputThread::SetFuncExtenderLogData(int index, QString data){
    if( index < 0 || index >= 4 ){
        return;
    }
    FEData[index] = data;
}


