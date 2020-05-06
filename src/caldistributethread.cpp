#include "caldistributethread.h"

#include <QDebug>

#ifdef _PERFORMANCE_CHECK
#include <windows.h>
#endif


extern int maxWorkerThread;
extern QList<QWaitCondition *> workerCond;
extern QList<int> statusWorkerThread;
extern QMutex *workerMutex;
extern bool allWorkerThreadFinshed;
extern QWaitCondition *condSimMain;



CalDistributeThread::CalDistributeThread(QObject *parent) : QThread(parent)
{
    stopped = false;
}


void CalDistributeThread::SetAgentPointer(Agent **pa)
{
    agent = pa;
}


void CalDistributeThread::stop()
{
    stopped = true;
}


void CalDistributeThread::run()
{
    qDebug() << "Start Worker Thread: thread_id = " << threadId;

#ifdef _PERFORMANCE_CHECK
    LARGE_INTEGER start, end;

    LARGE_INTEGER freq;
    QueryPerformanceFrequency(&freq);

    double calTime[10];
    int calCount[10];
    for(int i=0;i<10;++i){
        calTime[i] = 0.0;
        calCount[i] = 0;
    }
#endif


    while( stopped == false ){

        workerMutex->lock();
        if( statusWorkerThread[threadId] == 0 )
            workerCond[threadId]->wait(workerMutex);
        workerMutex->unlock();


        // To finish thread
        if( stopped == true ){
            break;
        }

        if( *pWorkingMode == 1 ){

//            workerMutex->lock();
//            qDebug() << "[Thread " << threadId << "] backup memory";
//            workerMutex->unlock();

            for(int n=0;n<evalIDs->size();++n){

                int i = evalIDs->at(n);

                agent[i]->BackupMemory();
            }

        }
        else if( *pWorkingMode == 2 ){

//            workerMutex->lock();
//            qDebug() << "[Thread " << threadId << "] agent's main logic";
//            workerMutex->unlock();


            //
            // Control
            for(int n=0;n<evalIDs->size();++n){

                int i = evalIDs->at(n);

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&start);
#endif

                agent[i]->Perception( agent, maxAgent, road, trafficSignal );

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&end);
                calTime[1] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                calCount[1]++;
#endif

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&start);
#endif

                agent[i]->Recognition( agent, maxAgent, road );

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&end);
                calTime[2] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                calCount[2]++;
#endif

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&start);
#endif

                agent[i]->HazardIdentification( agent, maxAgent, road );

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&end);
                calTime[3] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                calCount[3]++;
#endif

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&start);
#endif

                agent[i]->RiskEvaluation( agent, maxAgent, road, trafficSignal );

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&end);
                calTime[4] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                calCount[4]++;
#endif

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&start);
#endif

                agent[i]->Control( road );

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&end);
                calTime[5] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                calCount[5]++;
#endif

            }

#ifdef _PERFORMANCE_CHECK
            if( calCount[1] >= 5000 ){
                for(int i=1;i<5;i++){
                    calTime[i] /= calCount[i];
                    qDebug() << "[ThreadID=" << threadId << "]: Mean Time[" << i << "] = " << calTime[i];
                    calCount[i] = 0;
                }
            }
#endif

        }
        else if( *pWorkingMode == 3 ){

            //
            // Control
            for(int n=0;n<evalIDs->size();++n){

                int i = evalIDs->at(n);

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&start);
#endif

                agent[i]->UpdateState( road );

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&end);
                calTime[6] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                calCount[6]++;
#endif

            }

#ifdef _PERFORMANCE_CHECK
            if( calCount[6] >= 5000 ){
                calTime[6] /= calCount[6];
                qDebug() << "[ThreadID=" << threadId << "]: Mean Time[6] = " << calTime[6];
                calCount[6] = 0;
            }
#endif

        }
        else if( *pWorkingMode == 4 ){

            //
            // Check vehicles that reach to end of route
            for(int n=0;n<evalIDs->size();++n){

                int i = evalIDs->at(n);

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&start);
#endif

                agent[i]->CheckPathList( road );

#ifdef _PERFORMANCE_CHECK
                QueryPerformanceCounter(&end);
                calTime[7] += static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
                calCount[7]++;
#endif
            }

#ifdef _PERFORMANCE_CHECK
            if( calCount[7] >= 5000 ){
                calTime[7] /= calCount[7];
                qDebug() << "[ThreadID=" << threadId << "]: Mean Time[7] = " << calTime[7];
                calCount[7] = 0;
            }
#endif
        }



        workerMutex->lock();
        statusWorkerThread[threadId] = 0;
        bool allfinished = true;
        for(int i=0;i<maxWorkerThread;++i){
            if( statusWorkerThread[i] == 1 ){
                allfinished = false;
                break;
            }
        }
        if( allfinished == true ){
            allWorkerThreadFinshed = true;
            condSimMain->wakeAll();
        }
        workerMutex->unlock();

    }

    qDebug() << "Leave worker thread: thread id = " << threadId;
}

