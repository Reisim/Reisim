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


#ifndef SYSTEMTHREAD_H
#define SYSTEMTHREAD_H

#include <QObject>
#include <QThread>
#include <QFile>
#include <QTextStream>
#include <QString>
#include <QStringList>
#include <QFileDialog>
#include <QMessageBox>
#include <QList>

#include "udpthread.h"
#include "simulationmanager.h"
#include "logoutputthread.h"


enum SIMULATION_STATE
{
    INITIATED,
    READY_TO_START,
    STARTED,
    PAUSED,
};

class SystemThread : public QThread
{
    Q_OBJECT
public:
    explicit SystemThread(QObject *parent = 0);

    void run();

    void SetSysFile(QString filename);
    void SetConfFile(QString filename){ confFile = filename; }
    void LoadScenarioFile();

    QList<QList<int> *> evalIDs;
    int *pWorkingMode;

signals:
    void UpdateSimulationTimeDisplay(QString);
    void SetAgentPointer(Agent**);
    void SetTrafficSignalList(QList<TrafficSignal*>);
    void SetTrafficSignalPointer(TrafficSignal*);
    void SetMaxNumberAgent(int);
    void SetNumberTrafficSignal(int);
    void RedrawRequest();
    void SetRoadPointer(Road*);
    void TmpStopSimulation();
    void ExitProgram();



public slots:
    void Stop();
    void SetScenarioFile(QString filename);
    void SetSupplementFile(QString filename);
    void LoadRoadData(QString roadDataFile);
    void LoadTrafficSignalData(QString signalFile);
    void SetDSmode();
    void SetStartTrigger(int);
    void SetLogOutputFolder(QString);
    void SetLogFileName(QString);
    void SetLogOutputInterval(int);
    void SimulationStart();
    void SimulationStop();

    void SimulationPause();
    void SimulationResume();
    void SetSpeedAdjustVal(int);
    void wrapExitProgram();
    void SetSendData(char *,int,int *);
    void SetSimulationFrequency(int);
    void SetTireHeight(int,float,float,float,float);

    void SetSInterObjData( char, int, struct AgentState *,struct SInterObjInfo * );
    void ForceChangeTSColor(int,int,float);

    void WarpVehicle(int,float,float,float);
    void DisposeAgent(int);
    void AppearAgent(int);
    void EmbedAgentBehavior(int,float*,int);
    void ChangeReferenceSpeed(int,float);
    void CopyPathData(int,int);
    void ChangeControlModeStopAt(int,float,float);
    void ChangeControlModeHeadwayControl(int,float,float,float,float,int);
    void ChangeControlModeAgent(int);
    void ChangeControlModeIntersectionTurn(int,float,float);
    void CopyScenarioData(int,int);
    void ChangeVehicleWinkers(int,int);
    void SetStopGraphicUpdate(bool);
    void SetLateralShift(int,float);
    void SetLateralGainMultiplier(int,float);
    void SetAgentGenerationNotAllowFlag(int, bool);
    void ForceChangeSpeed(int,float);

    void ShowAgentData(float x,float y);
    void SetTmpStopTime(int hour,int min,int sec);


private:
    volatile bool stopped;
    int maxAgent;
    int numTrafficSignals;

    SimulationManager *simManage;
    Road *road;
    Agent** agent;

    QList<TrafficSignal*> trafficSignal;
    QList<int> tsId2Index;

    LogOutputThread *logThread;



    bool DSMode;
    int calHz;
    int intervalMSec;

    int  startTrigger;
    int  simulationState;
    int  speedAdjustVal;
    bool stopGraphicUpdate;

    UDPThread *udpThread;
    QString sysFile;
    QString confFile;
    QString scenarioFile;
    QString roadDataFile;
    QString signalDataFile;
    QString suppleFileName;
    QString logOutputFolder;
    QString logFileName;
    int logOutputInterval;

    int tmpStopHour;
    int tmpStopMin;
    int tmpStopSecond;
};

#endif // SYSTEMTHREAD_H
