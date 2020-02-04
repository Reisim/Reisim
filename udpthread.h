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

signals:
    void SimulationStart();
    void SimulationStop();
    void ExitProgram();
    void RequestSetSendData(char*,int,int *);
    void ReceiveContinueCommand();
    void SetSimulationFrequency(int);
    void ReceiveTireHeight(int,float,float,float,float);
    void ReceiveSInterObjData( char, int, struct AgentState *,struct SInterObjInfo *);
    void ReceiveTSColorChange(int,int,float);
    void WarpVehicle(int,float,float,float);
    void DisposeAgent(int);
    void AppearAgent(int);
    void EmbedBehavior(int,float*,int);
    void ChangeReferenceSpeed(int,float);
    void CopyPathData(int,int);
    void ChangeControlModeStopAt(int,float,float);
    void ChangeControlModeHeadwayControl(int,float,float,float,float,int);
    void FEDataReceived(int, QString);
    void ChangeControlModeAgent(int);
    void ChangeControlModeIntersectionTurn(int,float,float);
    void CopyScenarioData(int,int);
    void ChangeVehicleWinkers(int,int);
    void SetLateralShift(int,float);
    void SetLateralGainMultiplier(int,float);
    void SetAgentGenerationNotAllowFlag(int, bool);
    void ForceChangeSpeed(int,float);


public slots:
    void Stop();
    void ReadUDPData();
    void SetMaxAgentNumber(int);
    void SetNumberTrafficSignal(int);


private:
    volatile bool stopped;

    int maxAgent;
    int numberTrafficSignal;

    QList<struct AppInfo *> apps;
    QList<struct SystemNetwork *> sysNet;

    QList<struct SendUDPSocks*> sendSocks;
    QList<struct RecvUDPSocks*> recvSocks;
    QList<int> correspodingSockIndex;
    int scoreSendSockIndex;

    int syncSigCount;
};

#endif // UDPTHREAD_H
