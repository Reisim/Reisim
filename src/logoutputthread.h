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


#ifndef LOGOUTPUTTHREAD_H
#define LOGOUTPUTTHREAD_H

#include <QObject>
#include <QThread>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>

#include <QMutex>
#include <QWaitCondition>

#include "simulationmanager.h"
#include "agent.h"
#include "trafficsignal.h"


class LogOutputThread : public QThread
{
    Q_OBJECT
public:
    explicit LogOutputThread(QObject *parent = 0);
    ~LogOutputThread();

    void SetDataReference(SimulationManager* s, Agent** a,int ma,QList<TrafficSignal*> t,int nts)
    {
        simManage = s;
        agent     = a;
        maxAgent  = ma;
        ts        = t;
        numTS     = nts;
    }


    void run();
    void SetLogFileName(QString filename);
    void SetLogOutputInterval(int interval){ logInterval = interval; }
    void CloseFile();
    void SetStopFlag(){ stopFlag = true; }
    void SetExpID(QString t){ expID = t; }


public slots:
    void SetFuncExtenderLogData(int index, QString data);


private:
    volatile bool stopFlag;

    QString logFileName;
    QFile fileLog;
    bool logOutputFlag;

    SimulationManager* simManage;
    Agent** agent;
    int maxAgent;
    QList<TrafficSignal *> ts;
    int numTS;

    int logInterval;
    int logIntervalCount;

    QString FEData[4];

    QString expID;
};

#endif // LOGOUTPUTTHREAD_H
