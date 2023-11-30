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


#ifndef UDPTHREAD_H
#define UDPTHREAD_H

#include <QObject>
#include <QThread>
#include <QUdpSocket>
#include <QList>
#include <QString>
#include <QStringList>
#include <QFile>
#include <QTextStream>

#include "agent.h"

#include <windows.h>


struct SendUDPSocks
{
    QUdpSocket sock;
    QString ipAddress;
    int to_port;
};

struct RecvUDPSocks
{
    QUdpSocket sock;
    int wait_port;
};

struct AppInfo
{
    QString appName;
    QString ipAddress;
};

struct SystemNetwork
{
    QString sender;
    QString receiver;
    int receive_port;
};


class UDPThread : public QThread
{
    Q_OBJECT
public:
    explicit UDPThread(QObject *parent = 0);

    void run();
    void destroySocks();
    void createSendSock(QString ipAddres, int to_port, QString name);
    void createRecvSock(int wait_port, QString name);
    void loadSysFile(QString filename);
    void setupSockets();
    int  getNumberTrafficSignal(){ return numberTrafficSignal; }

    void SendDSMoveCommand(int targetID,float x,float y,float psi);
    void SendScenarioData(QString ipAddr, int port, char *sendBuf, int dataSize);

    void SetAgentCalFinished(){ agentCalFinished = true; }


    int GetMaxAgentDataSend(){ return maxAgentDataSend; }
    int GetMaxAgentDataSendToFE(){ return maxAgentDataSendToFE; }
    int GetMaxTSDataSend(){ return maxTSDataSend; }
    int GetSizeSInterfaceObjIDs(){ return SInterfaceObjIDs.size(); }
    int GetSInterfaceObjID(int idx){ return SInterfaceObjIDs[idx]; }
    bool GetHasFuncExtender(){ return HasFuncExtender; }

    void SendToUE4(int SInterObjID, char *buf, int size);
    void SendToFE(char *buf, int size);
    void SendToSCore();


    struct AgentState asv;
    struct SInterObjInfo sov;


signals:
    void SimulationStart();
    void SimulationStop();
    void ExitProgram();
    void RedrawRequest();
    void ReceiveContinueCommand();
    void SetSimulationFrequency(int);
    void ReceiveTireHeight(int,int,float,float,float,float);
    void ReceiveSInterInitState(char,int,float,float,float,float,float);
    void ReceiveSInterObjData( char, int, struct AgentState *,struct SInterObjInfo *);
    void ReceiveTSColorChange(int,int,float);
    void WarpVehicle(int,float,float,float);
    void WarpVehicleAdjustPosToLane(int,float,float,float);
    void DisposeAgent(int);
    void AppearAgent(int);
    void EmbedBehavior(int,float*,int);
    void ChangeReferenceSpeed(int,float);
    void CopyPathData(int,int);
    void RequestLaneChange(int,int,int);
    void RequestAssignedLaneChange(int,int,int,float);
    void ChangeControlModeStopAt(int,float,float);
    void ChangeControlModeHeadwayControl(int,float,float,float,float,int,bool);
    void FEDataReceived(int, QString);
    void ChangeControlModeAgent(int);
    void ChangeControlModeIntersectionTurn(int,float,float);
    void CopyScenarioData(int,int);
    void ChangeVehicleWinkers(int,int);
    void SetLateralShift(int,float);
    void SetLateralGainMultiplier(int,float);
    void SetAgentGenerationNotAllowFlag(int, bool);
    void ForceChangeSpeed(int,float);
    void SetRestartData();
    void ChangeSpeedProfile(int, QList<float>, QList<float>);
    void DirectAssignAcceleration(int,float);
    void SetTargetSpeedMode(int,int);
    void ChangeRoadLaneSpeedLimit(QList<int>, QList<float>);
    void ChangeVelocityControlParameters(float,float,float,float,float,int,QList<int>);
    void ChangeLaneAssignedVelocityControlParameters(float,float,float,float,float,QList<int>);
    void ChangeMoveSpeedPedestrian(QList<float>);
    void OverwriteAgentParameter(int,int,float);
    void ChangeOptionalImageParams(QList<float>);
    void SetEventTriggerByFuncExtend(int,int,int,int);
    void SetBrakeLampOverride(int,int);
    void SetScenarioVehicleInitStates(QList<int>,QList<float>,QList<float>,QList<float>,QList<float>,QList<float>);
    void SetAgentGenerationNotAllowFlagForNodes(QList<int>, bool);
    void AppearStoppingVehicle(int,float,float,float);
    void SetAgentExternalControlParams(int,float,float);
    void ChangePedestPathForScenarioPedestrian(int,QList<float>,QList<float>);


public slots:
    void Stop();
    void ReadUDPData();
    void SetMaxAgentNumber(int);
    void SetNumberTrafficSignal(int);


private:
    volatile bool stopped;

    int maxAgentDataSend;
    int maxAgentDataSendToFE;
    int maxTSDataSend;

    int maxAgent;
    int numberTrafficSignal;

    QList<struct AppInfo *> apps;
    QList<struct SystemNetwork *> sysNet;

    QList<struct SendUDPSocks*> sendSocks;
    QList<struct RecvUDPSocks*> recvSocks;
    QList<int> correspodingSockIndex;
    int scoreSendSockIndex;

    int syncSigCount;

    QList<int> SInterfaceObjIDs;
    QList<bool> recvFromSinterfaceObjFlag;

    bool HasFuncExtender;

    bool agentCalFinished;
    bool SInterfaceDataEmbeded;

    int simState;
};

#endif // UDPTHREAD_H
