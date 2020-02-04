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

signals:
    void UpdateSimulationTimeDisplay(QString);
    void SetAgentPointer(Agent**);
    void SetTrafficSignalPointer(QList<TrafficSignal*>);
    void SetMaxNumberAgent(int);
    void SetNumberTrafficSignal(int);
    void RedrawRequest();
    void SetRoadDataToCanvas(Road*);
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
    void WrapSetVehicleParameter(int ID,int Type,
                                 float length,
                                 float width,
                                 float height,
                                 float wheelBase,
                                 float distRR2RE,
                                 float FRWeightRatio);
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
};

#endif // SYSTEMTHREAD_H
