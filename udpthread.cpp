/****************************************************************************
**                                 Re:sim
**
**   Copyright (C) 2020 Misaki Design.LLC
**   Copyright (C) 2020 Jun Tajima <tajima@misaki-design.co.jp>
**
**   This file is part of the Re:sim simulation software
**
**   This file may be used under the terms of the GNU Lesser General Public
**   License version 3 as published by the Free Software Foundation.
**   For more detail, visit https://www.gnu.org/licenses/gpl-3.0.html
**
*************************************************************************** */


#include "udpthread.h"
#include <QDebug>

#include <QMutex>
#include <QWaitCondition>
#include <QVector>

#include "networkdrivecheck.h"

extern QMutex *mutex;
extern QWaitCondition *cond;
extern int g_DSTimingFlag;

extern QMutex *mutex_sync;
extern QWaitCondition *cond_sync;
extern int allowDataGetForDS;


LARGE_INTEGER start_time, end_time;
LARGE_INTEGER freq;

QVector<double> time_record;


UDPThread::UDPThread(QObject *parent) :
    QThread(parent)
{
    stopped = false;
    maxAgent = 100;
    numberTrafficSignal = 0;
    syncSigCount = 0;

    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&start_time);
}


void UDPThread::Stop()
{
    stopped = true;
}


void UDPThread::run()
{
    qDebug() << "[UDPThread]Start thread";

    stopped = false;
    while( stopped == false ){



    }

    qDebug() << "[UDPThread]Now leave thread";
}


void UDPThread::destroySocks()
{
    if( sendSocks.size() > 0 ){
        for(int i=0;i<sendSocks.size();++i){
            sendSocks[i]->sock.close();
            delete sendSocks[i];
        }
        sendSocks.clear();
    }

    if( recvSocks.size() > 0 ){
        for(int i=0;i<recvSocks.size();++i){
            recvSocks[i]->sock.close();
            delete recvSocks[i];
        }
        recvSocks.clear();
    }
}


void UDPThread::createSendSock(QString ipAddres, int to_port, QString name)
{
    struct SendUDPSocks *s = new struct SendUDPSocks;
    s->ipAddress = ipAddres;
    s->to_port = to_port;
    s->sock.setObjectName( name );
    s->sock.setProperty("index", sendSocks.size());
    sendSocks.append( s );
}


void UDPThread::createRecvSock(int wait_port, QString name)
{
    struct RecvUDPSocks *s = new struct RecvUDPSocks;
    s->wait_port = wait_port;
    s->sock.setObjectName( name );
    if( s->sock.bind( QHostAddress(QHostAddress::Any), wait_port ) == false ){
        qDebug() << "[createRecvSock] Failed to bind to port: " << wait_port;
    }
    s->sock.setProperty("index",recvSocks.size());

    connect( &(s->sock), SIGNAL(readyRead()), this, SLOT(ReadUDPData()) );

    recvSocks.append( s );
}


void UDPThread::loadSysFile(QString filename)
{
    if( filename == QString() ){
        return;
    }

    //
    // Read Sconf file
    QFile file( CheckNetworkDrive(filename) );
    if( !file.open(QIODevice::ReadOnly | QIODevice::Text) ){
        qDebug() << "Error: Cannot open file: " << filename;
        return;
    }

    QTextStream in(&file);
    QString line;

    line = in.readLine();
    line = in.readLine();
    if( line.contains("Sirius System Config File") == false ){
        qDebug() << "Error: The file is not Sirius System Config File";
        file.close();
        return;
    }
    line = in.readLine();


    //
    // Clear data if exist
    if( apps.size() > 0 ){
        for(int i=0;i<apps.size();++i){
            delete apps[i];
        }
        apps.clear();
    }

    if( sysNet.size() > 0 ){
        for(int i=0;i<sysNet.size();++i){
            delete sysNet[i];
        }
        sysNet.clear();
    }


    struct AppInfo* as = NULL;

    while( in.atEnd() == false ){

        line = in.readLine();
        if( line.startsWith("#") || line.isEmpty() || line.contains(";") == false ){
            continue;
        }

        QStringList divLine = line.split(";");
        QString tag = QString( divLine[0] ).trimmed();

        if( tag == QString("SCore") ){

            as = new struct AppInfo;
            if( as ){
                as->appName = QString("SCore");
                as->ipAddress = QString(divLine[1]).trimmed();
                apps.append( as );
            }

        }
        else if( tag == QString("Name") ){

            as = new struct AppInfo;
            if( as ){
                as->appName = QString(divLine[1]).trimmed();
            }

        }
        else if( tag == QString("Opts") ){

            if( as ){

                QString optStr = QString(divLine[1]).trimmed();
                if( as->appName == QString("UE4App") ){
                    QStringList divOptStr = optStr.split("|*|");

                    for(int i=0;i<divOptStr.size();++i){
                        if( QString(divOptStr[i]).contains("-object=") == true ){
                            QString valStr = QString( divOptStr[i] ).remove("-object=");
                            as->appName += QString("[") + valStr + QString(",");
                            break;
                        }
                    }

                    for(int i=0;i<divOptStr.size();++i){
                        if( QString(divOptStr[i]).contains("-screen=") == true ){
                            QString valStr = QString( divOptStr[i] ).remove("-screen=");
                            as->appName += valStr + QString("]");
                            break;
                        }
                    }

                }
                else if( as->appName == QString("S-Interface") ){
                    QStringList divOptStr = optStr.split("|*|");

                    for(int i=0;i<divOptStr.size();++i){
                        if( QString(divOptStr[i]).contains("-id=") == true ){
                            QString valStr = QString( divOptStr[i] ).remove("-id=");
                            as->appName += QString("[") + valStr + QString("]");
                        }
                    }
                }
                else if( as->appName == QString("Function Extender") ){
                    QStringList divOptStr = optStr.split("|*|");

                    for(int i=0;i<divOptStr.size();++i){
                        if( QString(divOptStr[i]).contains("-ID=") == true ){
                            QString valStr = QString( divOptStr[i] ).remove("-ID=");
                            as->appName += QString("[") + valStr + QString("]");
                        }
                    }
                }
            }
        }
        else if( tag == QString("IPAd") ){
            if( as ){
                as->ipAddress = QString(divLine[1]).trimmed();

                apps.append( as );
            }
        }
        else if( tag == QString("Defs") ){

            struct SystemNetwork *sns = new SystemNetwork;

            sns->sender = QString(divLine[1]).trimmed();
            sns->receiver = QString(divLine[2]).trimmed();
            sns->receive_port = QString(divLine[3]).trimmed().toInt();

            sysNet.append( sns );
        }
    }

    file.close();


    qDebug() << " AppInfo:";
    for(int i=0;i<apps.size();++i){
        qDebug() << "     " << apps[i]->appName << " (" << apps[i]->ipAddress << ")";
    }

    qDebug() << " System Network:";
    for(int i=0;i<sysNet.size();++i){
        qDebug() << "     " << sysNet[i]->sender << " -> " << sysNet[i]->receiver << "(" << sysNet[i]->receive_port << ")";
    }

    setupSockets();
}


void UDPThread::setupSockets()
{
    //
    // Set My App Name
    QString myAppName = QString("Re:sim");


    //
    // Clear sockets if exist
    if( sendSocks.size() > 0 ){
        for(int i=0;i<sendSocks.size();++i){
            sendSocks[i]->sock.close();
            delete sendSocks[i];
        }
        sendSocks.clear();
    }

    if( recvSocks.size() > 0 ){
        for(int i=0;i<recvSocks.size();++i){
            recvSocks[i]->sock.close();
            delete recvSocks[i];
        }
        recvSocks.clear();
    }

    correspodingSockIndex.clear();
    scoreSendSockIndex = -1;


    //
    // Prepare sockets
    for(int i=0;i<sysNet.size();++i){

        if( sysNet[i]->sender == myAppName ){
            for(int j=0;j<apps.size();++j){
                if( sysNet[i]->receiver == apps[j]->appName ){
                    createSendSock( apps[j]->ipAddress, sysNet[i]->receive_port, sysNet[i]->receiver );
                    break;
                }
            }

            if( sysNet[i]->receiver == QString("SCore") ){
                scoreSendSockIndex = sendSocks.size() - 1;
            }
        }
        else if( sysNet[i]->receiver == myAppName ){
            for(int j=0;j<apps.size();++j){
                if( sysNet[i]->sender == apps[j]->appName ){
                    createRecvSock( sysNet[i]->receive_port, sysNet[i]->sender );
                    break;
                }
            }
        }
    }

    for(int i=0;i<recvSocks.size();++i){
        QString sender = recvSocks[i]->sock.objectName();
        for(int j=0;j<sendSocks.size();++j){
            if( sendSocks[j]->sock.objectName() == sender ){
                correspodingSockIndex.append(j);
                break;
            }
        }
    }

    qDebug() << "size of sendSocks = " << sendSocks.size();
    for(int i=0;i<sendSocks.size();++i){
        qDebug() << "   send to: " << sendSocks[i]->sock.objectName()
                 << " IP: " << sendSocks[i]->ipAddress
                 << " Port: " << sendSocks[i]->to_port;
    }

    qDebug() << "size of recvSocks = " << recvSocks.size();
    for(int i=0;i<recvSocks.size();++i){
        qDebug() << "   receive from : " << recvSocks[i]->sock.objectName()
                 << " Port: " << recvSocks[i]->wait_port;
    }

    qDebug() << "size of correspodingSockIndex = " << correspodingSockIndex.size();
    for(int i=0;i<correspodingSockIndex.size();++i){
        qDebug() << "correspodingSockIndex[" << i << "] = " << correspodingSockIndex[i];
    }
}



void UDPThread::ReadUDPData()
{
    int idx = sender()->property("index").toInt();
//    qDebug() << "[receiveUPDData] idx = " << idx;
    if( idx < 0 || idx >= recvSocks.size() ){
        return;
    }

    while ( recvSocks[idx]->sock.hasPendingDatagrams() ) {

//        qDebug() << "[receiveDataFromSCore]";

        QByteArray datagram;
        datagram.resize(recvSocks[idx]->sock.pendingDatagramSize());

        QHostAddress sender;
        recvSocks[idx]->sock.readDatagram( datagram.data(), datagram.size(), &sender );

        char *com = datagram.data();
        int comSize = datagram.size();

//        QueryPerformanceCounter(&end_time);
//        double cal_time = static_cast<double>(end_time.QuadPart - start_time.QuadPart) * 1000.0 / freq.QuadPart;
//        start_time = end_time;
//        qDebug() << cal_time << " " << com[0] << com[1] << " " << comSize;
//        time_record.append(cal_time);


        int index = 0;
//        while( index < comSize ){

            if( com[index+0] == 'C' ){  // Command from SCore

                if( com[index+1] == 'C' ){  // Check network connection

                    char res[255];
                    memset(res,0,255);
                    res[0] = 'C';
                    res[1] = 'C';
                    res[2] = 'R';
                    res[2] = 'S';

                    if( scoreSendSockIndex >= 0 ){
                        sendSocks[scoreSendSockIndex]->sock.writeDatagram( res,
                                                                           4,
                                                                           QHostAddress( sendSocks[scoreSendSockIndex]->ipAddress ),
                                                                           sendSocks[scoreSendSockIndex]->to_port );
                        sendSocks[scoreSendSockIndex]->sock.flush();
                        qDebug() << " send response data to " << sendSocks[scoreSendSockIndex]->sock.objectName();
                    }

                    index += 2;
                }
                else if( com[index+1] == 'I' ){  // Initialize model

                    int calHz = 0;
                    memcpy( &calHz, &(com[index+2]), sizeof(int) );
                    qDebug() << " calHz = " << calHz;

                    emit SetSimulationFrequency( calHz );

                    char res[255];
                    memset(res,0,255);
                    res[0] = 'C';
                    res[1] = 'I';
                    res[2] = 'E';

                    if( scoreSendSockIndex >= 0 && scoreSendSockIndex < sendSocks.size() ){
                        sendSocks[scoreSendSockIndex]->sock.writeDatagram( res,
                                                                           3,
                                                                           QHostAddress( sendSocks[scoreSendSockIndex]->ipAddress ),
                                                                           sendSocks[scoreSendSockIndex]->to_port );
                        sendSocks[scoreSendSockIndex]->sock.flush();
                        qDebug() << " send success data to " << sendSocks[scoreSendSockIndex]->sock.objectName();
                    }
                    else{
                        qDebug() << " invalid scoreSendSockIndex = " << scoreSendSockIndex;
                    }

                    index += 6;
                }
                else if( com[index+1] == 'S' ){  // Start Simulation

                    //
                    // Add code here if necessary
                    //
                    qDebug() << "Received Start Simulation Command";
                    emit SimulationStart();

                    index += 2;

                }
                else if( com[index+1] == 'T' ){  // Termination

                    qDebug() << "Received Stop Simulation Command";
                    qDebug() << "recved sync signal count = " << syncSigCount;

//                    for(int i=0;i<time_record.size();++i){
//                        qDebug() << time_record[i];
//                    }
//                    getchar();

                    emit SimulationStop();
                    emit ReceiveContinueCommand();

                    msleep(200);

                    qDebug() << "Now Exit Program...";
                    emit ExitProgram();

                    index += 2;

                }
                else if( com[index+1] == 'R' ){  // Check network connection with UE4, FuncExtend

                    char res[255];
                    memset(res,0,255);
                    res[0] = 'R';
                    res[1] = 'c';

                    for(int i=0;i<sendSocks.size();++i){

                        QString sendToApp = sendSocks[i]->sock.objectName();
                        if( sendToApp.contains("SCore") == true ||
                                sendToApp.contains("S-Interface") == true ||
                                sendToApp.contains("Re:sim") == true ){
                            continue;
                        }

                        sendSocks[i]->sock.writeDatagram( res,
                                                           2,
                                                           QHostAddress( sendSocks[i]->ipAddress ),
                                                           sendSocks[i]->to_port );
                        sendSocks[i]->sock.flush();
                        qDebug() << " send network check data to " << sendSocks[i]->sock.objectName();
                        qDebug() << "   data = " << res;
                        qDebug() << "   IP = " << sendSocks[i]->ipAddress << " : port = " << sendSocks[i]->to_port;
                    }

                    index += 2;
                }
            }
            else if( com[index+0] == 'S' ){  // Data from S-Interface

                if( com[index+1] == 'I' ){

                    if( com[index+2] == 'c' ){

                        char res[255];
                        memset(res,0,255);
                        res[0] = 'U';
                        res[1] = 'E';
                        res[2] = 'c';

                        for(int i=0;i<sendSocks.size();++i){
                            if( sendSocks[i]->sock.objectName() == recvSocks[idx]->sock.objectName() ){
                                sendSocks[i]->sock.writeDatagram( res,
                                                                  3,
                                                                  QHostAddress( sendSocks[i]->ipAddress ),
                                                                  sendSocks[i]->to_port );
                                sendSocks[i]->sock.flush();
                                qDebug() << " send response data to " << sendSocks[i]->sock.objectName();

                                break;
                            }
                        }

                        index += 3;
                    }
                    else if( com[index+2] == 'd' ){

                        if( comSize >= 72 ){

                            char SInterObjType = com[index+3];
                            int  SInterObjID = -1;
                            int pos = 4;
                            memcpy( &SInterObjID, &(com[index+pos]), sizeof(int) );
                            pos += sizeof(int);

                            if( SInterObjID >= 0 && SInterObjID < maxAgent ){

                                float xVal;
                                memcpy( &xVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float yVal;
                                memcpy( &yVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float zVal;
                                memcpy( &zVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float rollVal;
                                memcpy( &rollVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float pitchVal;
                                memcpy( &pitchVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float yawVal;
                                memcpy( &yawVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float vVal;
                                memcpy( &vVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float axVal;
                                memcpy( &axVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float yrVal;
                                memcpy( &yrVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                pos += sizeof(float);  // ay
                                pos += sizeof(float);  // vy
                                pos += sizeof(char);   // engineKeyState
                                pos += sizeof(char);   // gearPosition
                                pos += sizeof(int);    // Tachometer

                                char lightFlag1;
                                memcpy( &lightFlag1, &(com[index+pos]), sizeof(char) );
                                pos += sizeof(char);

                                pos += sizeof(char);   // lightFlag2

                                float lfVal;
                                memcpy( &lfVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float lrVal;
                                memcpy( &lrVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float tfVal;
                                memcpy( &tfVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float trVal;
                                memcpy( &trVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                pos += sizeof(float);  // tireFLRotateAngle
                                pos += sizeof(float);  // tireFLSteerAngle
                                pos += sizeof(float);  // tireFRRotateAngle
                                pos += sizeof(float);  // tireFRSteerAngle
                                pos += sizeof(float);  // tireRLRotateAngle
                                pos += sizeof(float);  // tireRLSteerAngle
                                pos += sizeof(float);  // tireRRRotateAngle
                                pos += sizeof(float);  // tireRRSteerAngle
                                pos += sizeof(float);  // relZBody
                                pos += sizeof(float);  // relRollBody
                                pos += sizeof(float);  // relPitchBody
                                pos += sizeof(float);  // dummy
                                pos += sizeof(char);   // optionalFlag1
                                pos += sizeof(char);   // optionalFlag2
                                pos += sizeof(char);   // optionalFlag3
                                pos += sizeof(char);   // optionalFlag4

                                float accel = 0.0;
                                memcpy( &accel, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float brake = 0.0;
                                memcpy( &brake, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float steer = 0.0;
                                memcpy( &steer, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);


                                struct AgentState as;

                                as.x = xVal;
                                as.y = yVal * (-1.0);
                                as.z = zVal;

                                as.roll = rollVal;
                                as.pitch = pitchVal;
                                as.yaw = yawVal * (-1.0);

                                as.V = vVal;
                                as.cosYaw = cos( as.yaw );
                                as.sinYaw = sin( as.yaw );

    //                            as.accel = accel;
    //                            as.brake = brake;

                                as.accel_log = accel;   // 0 ~ 1
                                as.brake_log = brake;   // 0 ~ 1

                                if( axVal >= 0.0 ){
                                    as.accel = axVal;
                                    as.brake = 0.0;
                                }
                                else if( axVal < 0.0 ){
                                    as.accel = 0.0;
                                    as.brake = axVal * (-1.0);
                                }

                                as.steer = steer;       // [rad]
                                as.steer_log = steer;   // [rad]

                                struct SInterObjInfo so;

                                so.lf = lfVal;
                                so.lr = lrVal;
                                so.tf = tfVal;
                                so.tr = trVal;

                                so.brakeLamp = false;
                                so.winkerLeft = false;
                                so.winkerRight = false;

                                if( (lightFlag1 & 0x01) == 0x01 ){
                                    so.brakeLamp = true;
                                }

                                if( (lightFlag1 & 0x02) == 0x02 ){
                                    so.winkerLeft = true;
                                }

                                if( (lightFlag1 & 0x04) == 0x04 ){
                                    so.winkerRight = true;
                                }

                                if( (lightFlag1 & 0x08) == 0x08 ){
                                    so.lowbeam = true;
                                }

                                if( (lightFlag1 & 0x10) == 0x10 ){
                                    so.highbeam = true;
                                }

                                emit ReceiveSInterObjData( SInterObjType, SInterObjID, &as, &so );
                            }
                        }

                        index += 72;
                    }
                }
            }
            else if( com[index+0] == 'U' ){  // Data from UE4App

                if( com[index+1] == 'E' ){   // Response against network connection check

                    if( com[index+2] == 'c' ){

                        qDebug() << " received data " << com << " from " << sendSocks[idx]->sock.objectName();

                        char res[255];
                        memset(res,0,255);
                        res[0] = 'R';
                        res[1] = 'C';
                        res[2] = 'V';
                        res[3] = '|';

                        QString responseAppName = sendSocks[idx]->sock.objectName();
                        memcpy( &(res[4]), responseAppName.toLocal8Bit().data(), responseAppName.length() );

                        if( scoreSendSockIndex >= 0 ){
                            sendSocks[scoreSendSockIndex]->sock.writeDatagram( res,
                                                                               strlen(res),
                                                                               QHostAddress( sendSocks[scoreSendSockIndex]->ipAddress ),
                                                                               sendSocks[scoreSendSockIndex]->to_port );
                            sendSocks[scoreSendSockIndex]->sock.flush();
                            qDebug() << " send response data to " << sendSocks[scoreSendSockIndex]->sock.objectName() << " , data = " << res;
                        }

                        index += 3;

                    }
                }
                else if( com[index+1] == 'D' && comSize >= 22){  // Tire hight data

                    int targetID = -1;
                    int pos = 2;
                    memcpy( &targetID, &(com[index+pos]), sizeof(int) );
                    pos += sizeof(int);

                    float zFL = 0.0;
                    memcpy( &zFL, &(com[index+pos]), sizeof(float) );
                    pos += sizeof(float);

                    float zFR = 0.0;
                    memcpy( &zFR, &(com[index+pos]), sizeof(float) );
                    pos += sizeof(float);

                    float zRL = 0.0;
                    memcpy( &zRL, &(com[index+pos]), sizeof(float) );
                    pos += sizeof(float);

                    float zRR = 0.0;
                    memcpy( &zRR, &(com[index+pos]), sizeof(float) );
                    pos += sizeof(float);

                    //qDebug() << "targetID = " << targetID << " z = " << zFL << " " << zFR << " " << zRL << " " << zRR;

                    emit ReceiveTireHeight( targetID, zFL, zFR, zRL, zRR );

                    index += 22;
                }
            }
            else if( com[index+0] == 'D' ){  // Update command from SCore

                if( com[index+1] == 'U' ){

                    syncSigCount++;

//                    QueryPerformanceCounter(&end_time);
//                    double cal_time = static_cast<double>(end_time.QuadPart - start_time.QuadPart) * 1000.0 / freq.QuadPart;
//                    start_time = end_time;
//                    qDebug() << cal_time;
//                    time_record.append(cal_time);

                    int sendDataMaxSize = 125 * maxAgent + 13 * numberTrafficSignal + 20;
                    if( sendDataMaxSize > 65536 ){
                        qDebug() << "!!! Buffer size smaller than required [emit RequestSetSendData]";
                        qDebug() << "maxAgent = " << maxAgent << " numberTrafficSignal = " << numberTrafficSignal << " sendDataMaxSize = " << sendDataMaxSize;
                    }
                    else{


//                        // should wait until last agent calculation finished
//                        mutex_sync->lock();
//                        if( allowDataGetForDS == 0 ){
//                            cond_sync->wait(mutex_sync);
//                        }
//                        allowDataGetForDS = 0;
//                        mutex_sync->unlock();


                        // set agent and TS data
                        static char sendData[65536];
                        memset( sendData, 0, 65536 );

                        sendData[0] = 'R';
                        sendData[1] = 'S';
                        sendData[2] = 'd';

                        //qDebug() << "emit RequestSetSendData";
                        int sendSize = 3;
                        emit RequestSetSendData(sendData, sendDataMaxSize, &sendSize);


                        //
                        //  Send data to Apps except S-Interface
                        //
                        for(int i=0;i<sendSocks.size();++i){

                            if( sendSocks[i]->sock.objectName().contains("S-Interface") == true ){
                                continue;
                            }

                            //qDebug() << "send data to " << sendSocks[i]->sock.objectName();

                            sendSocks[i]->sock.writeDatagram( sendData,
                                                              sendSize,
                                                              QHostAddress(sendSocks[i]->ipAddress),
                                                              sendSocks[i]->to_port);

                            sendSocks[i]->sock.flush();
                        }


//                        QueryPerformanceCounter(&end_time);
//                        double cal_time = static_cast<double>(end_time.QuadPart - start_time.QuadPart) * 1000.0 / freq.QuadPart;
//                        start_time = end_time;
//                        qDebug() << cal_time;
                    }


                    //
                    //  Start agent calculation
                    mutex->lock();
                    g_DSTimingFlag = 1;
                    cond->wakeAll();
                    mutex->unlock();

                    index += 2;
                }

            }
            else if( com[index+0] == 'F' ){   // Data from Function Extender

                if( com[index+1] == 'T' && com[index+2] == 'c' && comSize >= 15 ){  // Change Traffic Signal Color

                    int pos = 3;
                    int targetTSID = -1;
                    memcpy( &targetTSID, &(com[index+pos]), sizeof(int) );
                    pos += sizeof(int);

                    int targetColor = 0;
                    memcpy( &targetColor, &(com[index+pos]), sizeof(int) );
                    pos += sizeof(int);

                    float duration = 0.0;
                    memcpy( &duration, &(com[index+pos]), sizeof(float) );
                    pos += sizeof(float);

                    //qDebug() << "emit ReceiveTSColorChange("
                    //         << targetTSID << ","
                    //         << targetColor << ","
                    //         << duration << ")";

                    emit ReceiveTSColorChange( targetTSID, targetColor, duration );

                    index += 15;
                }
                else if( com[index+1] == 'L' ){    // Log data from UE4to output from Re:sim

                    int pos = 2;
                    int nData = 0;
                    memcpy( &nData, &(com[index+pos]), sizeof(int) );
                    pos += sizeof(int);

                    //qDebug() << "Log Data from FE: size = " << nData;
                    if( nData > 0 && nData <= 4 && comSize >=  nData * 4 + 6 ){

                        for(int i=0;i<nData;++i){
                            char feval[10];
                            memset( feval, 0, 10 );
                            memcpy( feval, &(com[index+pos]), sizeof(char) * 4 );
                            pos += 4;

                            //qDebug() << "feval" << (i+1) << " = " << feval;

                            emit FEDataReceived( i, QString(feval).trimmed() );
                        }
                    }

                    index += nData * 4 + 6;
                }
                else if( com[index+1] == 'W' ){    // Warp Scenario Vehicles

                    int pos = 2;

                    int targetVID = -1;
                    memcpy( &targetVID, &(com[index+pos]), sizeof(int) );
                    pos += sizeof(int);

                    float moveX = 0.0;
                    memcpy( &moveX, &(com[index+pos]), sizeof(float) );
                    pos += sizeof(float);

                    float moveY = 0.0;
                    memcpy( &moveY, &(com[index+pos]), sizeof(float) );
                    pos += sizeof(float);

                    float moveDir = 0;
                    memcpy( &moveDir, &(com[index+pos]), sizeof(float) );
                    pos += sizeof(float);

                    qDebug() << "x=" << moveX << " y=" << moveY << " dir=" << moveDir;

                    // UE4 coordinate -> Re:sim coordinate
                    moveY *= (-1.0);
                    moveDir *= (-1.0);

                    if( targetVID >= 0 && targetVID < maxAgent ){
                        emit WarpVehicle( targetVID, moveX, moveY, moveDir );
                    }

                    index += 18;
                }
                else if( com[index+1] == 'A' && com[index+2] == 'd' ){   // Set Dispose flag to Agent instance

                    int aID = -1;
                    memcpy( &aID, &(com[index+3]), sizeof(int) );
                    if( aID >= 0 && aID < maxAgent ){
                        qDebug() << "emit DisposeAgent(" << aID << ")";
                        emit DisposeAgent( aID );
                    }

                    index += 7;

                }
                else if( com[index+1] == 'A' && com[index+2] == 'a' ){   // Set Flag in the trigger to appear

                    int aID = -1;
                    memcpy( &aID, &(com[index+3]), sizeof(int) );
                    if( aID >= 0 && aID < maxAgent ){
                        qDebug() << "emit AppearAgent(" << aID << ")";
                        emit AppearAgent( aID );
                    }

                    index += 7;

                }
                else if( com[index+1] == 'A' && com[index+2] == 'e' && com[index+3] == 'b' ){   // Embed behavior data from FE

                    int aID = -1;
                    int pos = 4;
                    memcpy( &aID, &(com[index+pos]), sizeof(int) );
                    pos += sizeof(int);

                    if( aID >= 0 && aID < maxAgent ){

                        float embedData[9];

                        float X = 0.0;
                        memcpy( &X, &(com[index+pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Y = 0.0;
                        memcpy( &Y, &(com[index+pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Z = 0.0;
                        memcpy( &Z, &(com[index+pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Roll = 0.0;
                        memcpy( &Roll, &(com[index+pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Pitch = 0.0;
                        memcpy( &Pitch, &(com[index+pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Yaw = 0;
                        memcpy( &Yaw, &(com[index+pos]), sizeof(float) );
                        pos += sizeof(float);

                        float V = 0;
                        memcpy( &V, &(com[index+pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Steer = 0;
                        memcpy( &Steer, &(com[index+pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Brake = 0;
                        memcpy( &Brake, &(com[index+pos]), sizeof(float) );
                        pos += sizeof(float);

                        // UE4 coordinate -> Re:sim coordinate
                        Y *= (-1.0);
                        Yaw *= (-1.0);
                        Roll *= (-1.0);

                        Roll  *= 0.017452;  // [deg] -> [rad]
                        Pitch *= 0.017452;  // [deg] -> [rad]
                        Yaw   *= 0.017452;  // [deg] -> [rad]

                        V *= 0.277777;    // [km/h] -> [m/s]

                        embedData[0] = X;
                        embedData[1] = Y;
                        embedData[2] = Z;
                        embedData[3] = Roll;
                        embedData[4] = Pitch;
                        embedData[5] = Yaw;
                        embedData[6] = V;
                        embedData[7] = Steer;
                        embedData[8] = Brake;

                        emit EmbedBehavior(aID, embedData, 9 );
                    }

                    index += pos;
                }
                else if( com[index+1] == 'C' && com[index+2] == 'R' && com[index+3] == 'S'){   // Change Reference Speed

                    int aID = -1;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );
                    if( aID >= 0 && aID < maxAgent ){

                        float refSpeed = 0.0;
                        memcpy( &refSpeed, &(com[index+8]), sizeof(float) );

                        emit ChangeReferenceSpeed( aID, refSpeed );
                    }

                    index += 12;

                }
                else if( com[index+1] == 'C' && com[index+2] == 'P' ){   // Copy Path Data

                    int fromAID = -1;
                    memcpy( &fromAID, &(com[index+3]), sizeof(int) );

                    int toAID = -1;
                    memcpy( &toAID, &(com[index+7]), sizeof(int) );

                    if( fromAID >= 0 && fromAID < maxAgent && toAID >= 0 && toAID < maxAgent ){
                        emit CopyPathData( fromAID, toAID );
                    }

                    index += 11;

                }
                else if( com[index+1] == 'C' && com[index+2] == 'C' && com[index+3] == 'S' ){   // Change Control-mode to Stop-at

                    int aID = -1;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );
                    if( aID >= 0 && aID < maxAgent ){

                        float stopAtX = 0.0;
                        memcpy( &stopAtX, &(com[index+8]), sizeof(float) );

                        float stopAtY = 0.0;
                        memcpy( &stopAtY, &(com[index+12]), sizeof(float) );

                        stopAtY *= (-1.0);  // from UER4 -> Re:sim

                        emit ChangeControlModeStopAt( aID, stopAtX, stopAtY );
                    }

                    index += 16;
                }
                else if( com[index+1] == 'C' && com[index+2] == 'C' && com[index+3] == 'H' ){   // Change Control-mode to Headway Control

                    int aID = -1;
                    int addSize = 24;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );
                    if( aID >= 0 && aID < maxAgent ){

                        float Speed = 0.0;
                        memcpy( &Speed, &(com[index+8]), sizeof(float) );
                        Speed /= 3.6;

                        float HeadwayDist = 0.0;
                        memcpy( &HeadwayDist, &(com[index+12]), sizeof(float) );

                        float HeadwayTime = 0.0;
                        memcpy( &HeadwayTime, &(com[index+16]), sizeof(float) );

                        int targetID = 0;
                        memcpy( &targetID, &(com[index+20]), sizeof(int) );

                        float AllowHDDev = 0.0;
                        if( index+28 <= comSize){
                            memcpy( &AllowHDDev, &(com[index+24]), sizeof(float) );
                            addSize = 28;
                        }

                        qDebug() << "targetID=" << targetID;
                        emit ChangeControlModeHeadwayControl( aID, Speed, HeadwayDist, AllowHDDev, HeadwayTime, targetID );

                    }

                    index += addSize;
                }
                else if( com[index+1] == 'C' && com[index+2] == 'C' && com[index+3] == 'A' ){   // Change Control-mode to Agent Control

                    int aID = -1;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );
                    if( aID >= 0 && aID < maxAgent ){
                        emit ChangeControlModeAgent( aID );
                    }

                    index += 8;
                }
                else if( com[index+1] == 'C' && com[index+2] == 'C' && com[index+3] == 'I' ){   // Change Control mode to Intersection Turn

                    int aID = -1;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );
                    if( aID >= 0 && aID < maxAgent ){

                        float Speed = 0.0;
                        memcpy( &Speed, &(com[index+8]), sizeof(float) );
                        Speed /= 3.6;

                        float InsideSpeed = 0.0;
                        memcpy( &InsideSpeed, &(com[index+12]), sizeof(float) );
                        InsideSpeed /= 3.6;

                        emit ChangeControlModeIntersectionTurn( aID, Speed, InsideSpeed );
                    }

                    index += 16;
                }
                else if( com[index+1] == 'C' && com[index+2] == 'S' && com[index+3] == 'D' ){   // Copy Scenario Data

                    int fromAID = -1;
                    memcpy( &fromAID, &(com[index+4]), sizeof(int) );

                    int toAID = -1;
                    memcpy( &toAID, &(com[index+8]), sizeof(int) );

                    if( fromAID >= 0 && fromAID < maxAgent && toAID >= 0 && toAID < maxAgent ){
                        emit CopyScenarioData( fromAID, toAID );
                    }

                    index += 12;
                }
                else if( com[index+1] == 'V' && com[index+2] == 'W'  ){    // Control vehicle winkers/hazard

                    int aID = -1;
                    memcpy( &aID, &(com[index+3]), sizeof(int) );

                    int wState =0;
                    memcpy( &wState, &(com[index+7]), sizeof(int) );

                    if( aID >= 0 && aID < maxAgent ){
                        emit ChangeVehicleWinkers( aID, wState );
                        qDebug() << "winker control: targetID=" << aID << " state=" << wState;
                    }

                    index += 11;
                }
                else if( com[index+1] == 'S' && com[index+2] == 'L' && com[index+3] == 'S'  ){    // Set Lateral Shift

                    int aID = -1;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );

                    float lateralShift = 0;
                    memcpy( &lateralShift, &(com[index+8]), sizeof(float) );

                    if( aID >= 0 && aID < maxAgent ){
                        emit SetLateralShift( aID, lateralShift );
                        qDebug() << "lateral shift: targetID=" << aID << " lateralShift=" << lateralShift;
                    }

                    index += 12;
                }
                else if( com[index+1] == 'S' && com[index+2] == 'G' && com[index+3] == 'M'  ){    // Set Lateral Gain Multiplier

                    int aID = -1;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );

                    float gainMultiplier = 0;
                    memcpy( &gainMultiplier, &(com[index+8]), sizeof(float) );

                    if( aID >= 0 && aID < maxAgent ){
                        emit SetLateralGainMultiplier( aID, gainMultiplier );
                        qDebug() << "lateral gain multiplier: targetID=" << aID << " gainMultiplier=" << gainMultiplier;
                    }

                    index += 12;
                }
                else if( com[index+1] == 'A' && com[index+2] == 'p' && com[index+3] == 'a' ){  // Stop Agent Generation Temporally

                    int nData = -1;
                    memcpy( &nData, &(com[index+4]), sizeof(int) );

                    int pos = index + 8;
                    for(int i=0;i<nData;++i){
                        int aID = -1;
                        memcpy( &aID, &(com[pos]), sizeof(int) );
                        pos += sizeof(int);

                        if( aID >= 0 && aID < maxAgent ){
                            emit SetAgentGenerationNotAllowFlag( aID, true );
                        }
                    }

                    index += nData * 4 + 8;
                }
                else if( com[index+1] == 'A' && com[index+2] == 'r' && com[index+3] == 'a' ){  // Stop Agent Generation Temporally

                    int nData = -1;
                    memcpy( &nData, &(com[index+4]), sizeof(int) );

                    int pos = index + 8;
                    for(int i=0;i<nData;++i){
                        int aID = -1;
                        memcpy( &aID, &(com[pos]), sizeof(int) );
                        pos += sizeof(int);

                        if( aID >= 0 && aID < maxAgent ){
                            emit SetAgentGenerationNotAllowFlag( aID, false );
                        }
                    }

                    index += nData * 4 + 8;
                }
                else if( com[index+1] == 'F' && com[index+2] == 'C' && com[index+3] == 'S'){   // Force Change Speed

                    int aID = -1;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );
                    if( aID >= 0 && aID < maxAgent ){

                        float Speed = 0.0;
                        memcpy( &Speed, &(com[index+8]), sizeof(float) );
                        Speed /= 3.6;

                        emit ForceChangeSpeed( aID, Speed );

                        qDebug() << "Force Change Speed: targetID=" << aID << " speed=" << (Speed * 3.6);
                    }

                    index += 12;

                }
            }
//            else{
//                index++;
//            }
//        }
    }
}


void UDPThread::SetMaxAgentNumber(int maNum)
{
    maxAgent = maNum;
}


void UDPThread::SetNumberTrafficSignal(int n)
{
    numberTrafficSignal = n;
    qDebug() << "[UDPThread::SetNumberTrafficSignal]  numberTrafficSignal = " << numberTrafficSignal;
}



