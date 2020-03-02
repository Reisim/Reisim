#ifndef CALDISTRIBUTETHREAD_H
#define CALDISTRIBUTETHREAD_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QList>

#include "agent.h"


class CalDistributeThread : public QThread
{
    Q_OBJECT
public:
    explicit CalDistributeThread(QObject *parent = 0);

    void run();
    void stop();

    void setThreadID(int id) { threadId = id; }

    QList<int> *evalIDs;
    int *pWorkingMode;

public slots:
    void SetAgentPointer(Agent**);
    void SetMaxNumberAgent(int n){ maxAgent = n; }
    void SetRoadPointer(Road* pr){ road = pr; }
    void SetTrafficSignalPointer(TrafficSignal* pt){ trafficSignal.append( pt ); }


private:
    int threadId;
    volatile bool stopped;

    int maxAgent;
    Agent** agent;

    QList<TrafficSignal*> trafficSignal;

    Road *road;
};

#endif // CALDISTRIBUTETHREAD_H
