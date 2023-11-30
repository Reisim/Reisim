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


#include "udpthread.h"
#include <QDebug>

#include <QMutex>
#include <QWaitCondition>
#include <QVector>

#include "networkdrivecheck.h"

extern QMutex *mutex;
extern QWaitCondition *cond;
extern int g_DSTimingFlag;
extern int g_SInterEmbedFlag;
extern int g_simulationCount;

extern QMutex *mutex_sync;
extern QWaitCondition *cond_sync;
extern int allowDataGetForDS;

extern bool invalidXPivotData;


LARGE_INTEGER start_time, end_time;
LARGE_INTEGER freq;

QVector<double> time_record;


char *sendDataBuf = NULL;



UDPThread::UDPThread(QObject *parent) :
    QThread(parent)
{
    stopped = false;
    maxAgent = 100;
    numberTrafficSignal = 0;
    syncSigCount = 0;

    maxAgentDataSend = 200;
    maxAgentDataSendToFE = 60;
    maxTSDataSend = 40;

    HasFuncExtender = false;

    agentCalFinished = false;
    SInterfaceDataEmbeded = false;

    simState = 0;

    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&start_time);

    memset( &asv, 0, sizeof(asv) );
    memset( &sov, 0, sizeof(sov) );
}


void UDPThread::Stop()
{
    stopped = true;
}


void UDPThread::run()
{
    //qDebug() << "[UDPThread]Start thread";

    stopped = false;
    while( stopped == false ){



    }

    //qDebug() << "[UDPThread]Now leave thread";
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

    if( sendDataBuf ){
        delete [] sendDataBuf;
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

                            SInterfaceObjIDs.append( valStr.toInt() );
                            recvFromSinterfaceObjFlag.append( false );
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

                    HasFuncExtender = true;
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

    qDebug() << " SInterfaceObjIDs = " << SInterfaceObjIDs;

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

//        qDebug() << "[ReadUDPData] comSize = " << comSize;

//        QueryPerformanceCounter(&end_time);
//        double cal_time = static_cast<double>(end_time.QuadPart - start_time.QuadPart) * 1000.0 / freq.QuadPart;
//        start_time = end_time;
//        qDebug() << cal_time << " " << com[0] << com[1] << " " << comSize;
//        time_record.append(cal_time);


        int index = 0;
//        while( index < comSize ){

//            qDebug() << "index=" << index << " comSize = " << comSize << " data=" << com[index];

            if( com[index+0] == 'C' ){  // Command from SCore

                if( com[index+1] == 'C' ){  // Check network connection

                    char res[255];
                    memset(res,0,255);
                    res[0] = 'C';
                    res[1] = 'C';
                    res[2] = 'R';
                    res[3] = 'S';

                    if( scoreSendSockIndex >= 0 ){
                        sendSocks[scoreSendSockIndex]->sock.writeDatagram( res,
                                                                           4,
                                                                           QHostAddress( sendSocks[scoreSendSockIndex]->ipAddress ),
                                                                           sendSocks[scoreSendSockIndex]->to_port );
                        sendSocks[scoreSendSockIndex]->sock.flush();
                        qDebug() << " send response data to " << sendSocks[scoreSendSockIndex]->sock.objectName();
                    }

                    index += 2;

                    qDebug() << "[ReadUDPData] Data = CC";
                }
                else if( com[index+1] == 'I' ){  // Initialize model

                    int calHz = 0;
                    memcpy( &calHz, &(com[index+2]), sizeof(int) );
                    qDebug() << " calHz = " << calHz;

                    emit SetSimulationFrequency( calHz );
                    emit SetRestartData();

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

                    qDebug() << "[ReadUDPData] Data = CI";

                    index += 6;

                    emit RedrawRequest();

                    simState = 1;
                }
                else if( com[index+1] == 'S' ){  // Start Simulation

                    qDebug() << "[ReadUDPData] Data = CS";

                    //
                    // Add code here if necessary
                    //
                    qDebug() << "Received Start Simulation Command";
                    emit SimulationStart();

                    index += 2;

                    simState = 2;
                }
                else if( com[index+1] == 'T' ){  // Termination

                    qDebug() << "[ReadUDPData] Data = CT";

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

                    simState = 3;

                }
                else if( com[index+1] == 'R' ){  // Check network connection with UE4, FuncExtend

                    qDebug() << "[ReadUDPData] Data = CR";

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
                else{
                    index++;
                }
            }
            else if( com[index+0] == 'S' ){  // Data from S-Interface

                if( com[index+1] == 'I' ){

                    if( com[index+2] == 'c' ){

                        qDebug() << "[ReadUDPData] Data = SIc";

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
                    else if( com[index+2] == 'i' ){

                        int pos = 4;
                        char SInterObjType = com[index+3];

                        int  SInterObjID = -1;

                        memcpy( &SInterObjID, &(com[index+pos]), sizeof(int) );
                        pos += sizeof(int);

                        if( SInterObjID >= 0 && SInterObjID < maxAgent ){

                            float xVal = 0.0;
                            memcpy( &xVal, &(com[index+pos]), sizeof(float) );
                            pos += sizeof(float);

                            float yVal = 0.0;
                            memcpy( &yVal, &(com[index+pos]), sizeof(float) );
                            pos += sizeof(float);

                            float zVal = 0.0;
                            memcpy( &zVal, &(com[index+pos]), sizeof(float) );
                            pos += sizeof(float);

                            float yawVal;
                            memcpy( &yawVal, &(com[index+pos]), sizeof(float) );
                            pos += sizeof(float);

                            float wheelbase;
                            memcpy( &wheelbase, &(com[index+pos]), sizeof(float) );
                            pos += sizeof(float);

                            // UE4 coordinate to Re:sim coordinate
                            yVal *= -1.0;
                            yawVal *= -1.0;

                            qDebug() << "Received S-Interface Initial State Data:";
                            qDebug() << "Type = " << SInterObjType;
                            qDebug() << "ID = " << SInterObjID;
                            qDebug() << "X = " << xVal;
                            qDebug() << "Y = " << yVal;
                            qDebug() << "Z = " << zVal;
                            qDebug() << "Yaw = " << yawVal;
                            qDebug() << "Wheelbase = " << wheelbase;

                            emit ReceiveSInterInitState( SInterObjType, SInterObjID, xVal, yVal, zVal, yawVal, wheelbase );

                            qDebug() << "Redraw requested.";
                            emit RedrawRequest();
                        }

                    }
                    else if( com[index+2] == 'd' ){

                        if( invalidXPivotData == true ){
                            qDebug() << "[ReadUDPData] Data = SId; comsize = " << comSize;
                        }

                        int pos = 4;

                        if( comSize >= 72 ){

                            char SInterObjType = com[index+3];

                            int  SInterObjID = -1;

                            memcpy( &SInterObjID, &(com[index+pos]), sizeof(int) );
                            pos += sizeof(int);


                            if( invalidXPivotData == true || simState < 2 ){
                                qDebug() << "SInterObjID = " << SInterObjID;
                            }

                            if( SInterObjID >= 0 && SInterObjID < maxAgent ){

                                float xVal = 0.0;
                                memcpy( &xVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                float yVal = 0.0;
                                memcpy( &yVal, &(com[index+pos]), sizeof(float) );
                                pos += sizeof(float);

                                if( invalidXPivotData == true ){
                                    qDebug() << "xVal = " << xVal << " yVal = " << yVal;
                                }

                                float zVal = 0.0;
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


                                memset( &asv, 0, sizeof(asv) );
                                memset( &sov, 0, sizeof(sov) );

                                asv.x = xVal;
                                asv.y = yVal * (-1.0);
                                asv.z = zVal;

                                asv.roll = rollVal;
                                asv.pitch = pitchVal;
                                asv.yaw = yawVal * (-1.0);

                                asv.V = vVal;
                                asv.cosYaw = cos( asv.yaw );
                                asv.sinYaw = sin( asv.yaw );

    //                            asv.accel = accel;
    //                            asv.brake = brake;

                                asv.accel_log = accel;   // 0 ~ 1
                                asv.brake_log = brake;   // 0 ~ 1

                                if( axVal >= 0.0 ){
                                    asv.accel = axVal;
                                    asv.brake = 0.0;
                                }
                                else if( axVal < 0.0 ){
                                    asv.accel = 0.0;
                                    asv.brake = axVal * (-1.0);
                                }

                                asv.steer = steer;       // [rad]
                                asv.steer_log = steer;   // [rad]


                                sov.lf = lfVal;
                                sov.lr = lrVal;
                                sov.tf = tfVal;
                                sov.tr = trVal;

                                sov.brakeLamp = false;
                                sov.winkerLeft = false;
                                sov.winkerRight = false;

                                if( (lightFlag1 & 0x01) == 0x01 ){
                                    sov.brakeLamp = true;
                                }

                                if( (lightFlag1 & 0x02) == 0x02 ){
                                    sov.winkerLeft = true;
                                }

                                if( (lightFlag1 & 0x04) == 0x04 ){
                                    sov.winkerRight = true;
                                }

                                if( (lightFlag1 & 0x08) == 0x08 ){
                                    sov.lowbeam = true;
                                }

                                if( (lightFlag1 & 0x10) == 0x10 ){
                                    sov.highbeam = true;
                                }


                                sov.objID = SInterObjID;
                                if( SInterObjType == 'v' || SInterObjType == 'm' ){
                                    sov.objType = 0;
                                }
                                else if( SInterObjType == 'p' || SInterObjType == 'b' ){
                                    sov.objType = 100;
                                }


                                // Copy data to Buffer
                                emit ReceiveSInterObjData( SInterObjType, SInterObjID, &asv, &sov );


                                int sobjIdx = SInterfaceObjIDs.indexOf( SInterObjID );
                                if( sobjIdx >= 0 ){

                                    recvFromSinterfaceObjFlag[sobjIdx] = true;

                                    SInterfaceDataEmbeded = true;
                                    for(int ns=0;ns<SInterfaceObjIDs.size();++ns){
                                        if( recvFromSinterfaceObjFlag[ns] == false ){
                                            SInterfaceDataEmbeded = false;
                                            break;
                                        }
                                    }

                                    if( SInterfaceDataEmbeded == true ){
                                        //
                                        //  Allow to embed SInterface Object
                                        mutex->lock();
                                        g_SInterEmbedFlag = 1;
                                        cond->wakeAll();
                                        mutex->unlock();

                                    }
                                }
                                else{
                                    qDebug() << "Cannot find SInterObjID = " << SInterObjID << " from SInterfaceObjIDs :" << SInterfaceObjIDs;
                                }
                            }
                        }

                        index += pos;
                    }
                    else{
                        index += 2;
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

                        index += strlen(com);

                    }
                    else{
                        index += 2;
                    }
                }
                else if( com[index+1] == 'D' && comSize >= 22){  // Tire hight data

//                    qDebug() << "[ReadUDPData] Data = UD";

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


                    QString sockName = sendSocks[idx]->sock.objectName();
                    QStringList divSockName = sockName.split(",");
                    int siObjectID = QString(divSockName[0]).remove("UE4App[").trimmed().toInt();

                    //qDebug() << "siObjectID=" << siObjectID << " targetID=" << targetID << " z = " << zFL << " " << zFR << " " << zRL << " " << zRR;

                    emit ReceiveTireHeight( siObjectID, targetID, zFL, zFR, zRL, zRR );

                    index += pos;
                }
                else{
                    index++;
                }
            }
            else if( com[index+0] == 'D' ){  // Update command from SCore

                if( com[index+1] == 'U' ){

//                    qDebug() << "[ReadUDPData] Data = DU";

                    syncSigCount++;

                    float simTimeFromScore = 0.0;
                    memcpy( &simTimeFromScore, &(com[index+2]), sizeof(float) );

                    int simCount = 0;
                    if( comSize >= index+10 ){
                        memcpy( &simCount, &(com[index+6]), sizeof(int) );
                    }
                    g_simulationCount = simCount;

                    mutex->lock();
                    g_DSTimingFlag = 1;
                    cond->wakeAll();
                    mutex->unlock();

//                    QueryPerformanceCounter(&end_time);
//                    double cal_time = static_cast<double>(end_time.QuadPart - start_time.QuadPart) * 1000.0 / freq.QuadPart;
//                    start_time = end_time;
//                    qDebug() << cal_time;
//                    time_record.append(cal_time);

                    index += 2;
                }
                else{
                    index++;
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

                    index += pos;
                }
                else if( com[index+1] == 'W' ){    // Warp Scenario Vehicles

                    if( com[index+2] == 'a' ){

                        int pos = 3;

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
                            emit WarpVehicleAdjustPosToLane( targetVID, moveX, moveY, moveDir );
                        }

                        index += pos;
                    }
                    else{
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

                        index += pos;
                    }
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
                    int pos = index + 4;
                    memcpy( &aID, &(com[pos]), sizeof(int) );
                    pos += sizeof(int);

                    if( aID >= 0 && aID < maxAgent ){

                        float embedData[9];

                        float X = 0.0;
                        memcpy( &X, &(com[pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Y = 0.0;
                        memcpy( &Y, &(com[pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Z = 0.0;
                        memcpy( &Z, &(com[pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Roll = 0.0;
                        memcpy( &Roll, &(com[pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Pitch = 0.0;
                        memcpy( &Pitch, &(com[pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Yaw = 0;
                        memcpy( &Yaw, &(com[pos]), sizeof(float) );
                        pos += sizeof(float);

                        float V = 0;
                        memcpy( &V, &(com[pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Steer = 0;
                        memcpy( &Steer, &(com[pos]), sizeof(float) );
                        pos += sizeof(float);

                        float Brake = 0;
                        memcpy( &Brake, &(com[pos]), sizeof(float) );
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

                    index = pos;
                }
                else if( com[index+1] == 'A' && com[index+2] == 'S' && com[index+3] == 'M' ){   // Reference Speed Determine Mode for Agents

                    int spmode = -1;
                    memcpy( &spmode, &(com[index+4]), sizeof(int) );

                    if( spmode == 0 || spmode == 1 || spmode == 4 ){
                        int aID = -1;
                        memcpy( &aID, &(com[index+8]), sizeof(int) );

                        emit SetTargetSpeedMode( spmode, aID );

                        index += 12;
                    }
                    else if( spmode == 2 || spmode == 3 ){
                        int nTarget = -1;
                        memcpy( &nTarget, &(com[index+8]), sizeof(int) );

                        index += 12;
                        for(int n=0;n<nTarget;++n){

                            int aID = -1;
                            memcpy( &aID, &(com[index]), sizeof(int) );

                            emit SetTargetSpeedMode( spmode, aID );

                            index += 4;
                        }
                    }

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
                else if( com[index+1] == 'm' && com[index+2] == 'C' && com[index+3] == 'R' && com[index+4] == 'S'){   // Change Reference Speed(multi)

                    int nV = -1;
                    memcpy( &nV, &(com[index+5]), sizeof(int) );

                    float refSpeed = 0.0;
                    memcpy( &refSpeed, &(com[index+9]), sizeof(float) );

                    int pos = index + 13;

                    for(int k=0;k<nV;++k){

                        int aID = -1;
                        memcpy( &aID, &(com[pos]), sizeof(int) );
                        pos += sizeof(int);

                        if( aID >= 0 && aID < maxAgent ){
                            emit ChangeReferenceSpeed( aID, refSpeed );
                        }
                    }

                    index = pos;
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
                else if( com[index+1] == 'C' && com[index+2] == 'L' ){   // Change Lane command

                    int aID = -1;
                    memcpy( &aID, &(com[index+3]), sizeof(int) );

                    int dir = -1;
                    memcpy( &dir, &(com[index+7]), sizeof(int) );

                    int mode = -1;
                    memcpy( &mode, &(com[index+11]), sizeof(int) );

                    if( aID >= 0 && aID < maxAgent ){

                        qDebug() << "Received Lane Change Command: aID = " << aID
                                 << " dir = " << dir << " mode = " << mode;

                        emit RequestLaneChange( aID, dir, mode );
                    }

                    index += 15;

                }
                else if( com[index+1] == 'C' && com[index+2] == 'A' && com[index+3] == 'L' ){   // Change to Assigned Lane command

                    int aID = -1;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );

                    int dir = -1;
                    memcpy( &dir, &(com[index+8]), sizeof(int) );

                    int mode = -1;
                    memcpy( &mode, &(com[index+12]), sizeof(int) );

                    float moveLatDist = -1.0;
                    memcpy( &moveLatDist, &(com[index+16]), sizeof(float) );

                    if( aID >= 0 && aID < maxAgent ){

                        qDebug() << "Received Assigned Lane Change Command: aID = " << aID
                                 << " dir = " << dir << " mode = " << mode << " moveLatDist = " << moveLatDist;

                        emit RequestAssignedLaneChange( aID, dir, mode, moveLatDist );
                    }

                    index += 20;

                }
                else if( com[index+1] == 'O' && com[index+2] == 'A' && com[index+3] == 'P'){   // Overwrite Agent Parameter
                    int aID = -1;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );

                    int paramID = 0;
                    memcpy( &paramID, &(com[index+8]), sizeof(int) );

                    float val = 0.0;
                    memcpy( &val, &(com[index+12]), sizeof(float) );

                    if( aID >= 0 && aID < maxAgent ){

                        qDebug() << "Received Overwrite Agent Parameter Command: aID = " << aID
                                 << " paramID = " << paramID << " val = " << val;

                        emit OverwriteAgentParameter( aID, paramID, val );
                    }

                    index += 16;

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
                    if( aID >= -1 && aID < maxAgent ){

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
                        if( index+28 <= comSize ){
                            memcpy( &AllowHDDev, &(com[index+24]), sizeof(float) );
                            addSize = 28;
                        }

                        bool disableSpeedAdjustForCurve = false;
                        if( index+29 <= comSize ){
                            if( com[index+28] == 'y' ){
                                disableSpeedAdjustForCurve = true;
                            }
                            addSize = 29;
                        }

                        //qDebug() << "targetID=" << targetID;
                        emit ChangeControlModeHeadwayControl( aID, Speed, HeadwayDist, AllowHDDev, HeadwayTime, targetID, disableSpeedAdjustForCurve);

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
                else if( com[index+1] == 'C' && com[index+2] == 'C' && com[index+3] == 'D' ){

                    int aID = -1;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );
                    if( aID >= 0 && aID < maxAgent ){

                        float a_com = 0.0;
                        memcpy( &a_com, &(com[index+8]), sizeof(float) );

                        emit DirectAssignAcceleration( aID, a_com );
                    }

                    index += 12;
                }
                else if( com[index+1] == 'V' && com[index+2] == 'C' && com[index+3] == 'P' ){

                    float refVforDev = 0.0;
                    memcpy( &refVforDev, &(com[index+4]), sizeof(float) );

                    float vDevP = 0.0;
                    memcpy( &vDevP, &(com[index+8]), sizeof(float) );

                    float vDevM = 0.0;
                    memcpy( &vDevM, &(com[index+12]), sizeof(float) );

                    float accel = 0.0;
                    memcpy( &accel, &(com[index+16]), sizeof(float) );

                    float decel = 0.0;
                    memcpy( &decel, &(com[index+20]), sizeof(float) );

                    int selMode = -1;
                    memcpy( &selMode, &(com[index+24]), sizeof(int) );

                    int nTarget = -1;
                    memcpy( &nTarget, &(com[index+28]), sizeof(int) );

                    index += 32;

                    QList<int> aIDs;

                    for(int k=0;k<nTarget;++k){

                        int aid = -1;
                        memcpy( &aid, &(com[index]), sizeof(int) );
                        index += sizeof(int);

                        aIDs.append( aid );
                    }

                    refVforDev /= 3.6;
                    vDevP /= 3.6;
                    vDevM = fabs(vDevM) / 3.6;
                    decel = fabs(decel);

                    qDebug() << "Receive FVCP command: ";
                    qDebug() << "  refVforDev = " << refVforDev << "[m/s]";
                    qDebug() << "  vDevP = " << vDevP << "[m/s]";
                    qDebug() << "  vDevM = " << vDevM << "[m/s]";
                    qDebug() << "  accel = " << accel << "[m/s^2]";
                    qDebug() << "  decel = " << decel << "[m/s^2]";
                    qDebug() << "  select = " << selMode;
                    qDebug() << "  nTarget = " << nTarget << "  size of aIDs = " << aIDs.size();

                    emit ChangeVelocityControlParameters(refVforDev,vDevP,vDevM,accel,decel,selMode,aIDs);
                }
                else if( com[index+1] == 'L' && com[index+2] == 'S' && com[index+3] == 'V' ){

                    float refVforDev = 0.0;
                    memcpy( &refVforDev, &(com[index+4]), sizeof(float) );

                    float vDevP = 0.0;
                    memcpy( &vDevP, &(com[index+8]), sizeof(float) );

                    float vDevM = 0.0;
                    memcpy( &vDevM, &(com[index+12]), sizeof(float) );

                    float accel = 0.0;
                    memcpy( &accel, &(com[index+16]), sizeof(float) );

                    float decel = 0.0;
                    memcpy( &decel, &(com[index+20]), sizeof(float) );

                    int nTarget = -1;
                    memcpy( &nTarget, &(com[index+24]), sizeof(int) );

                    index += 28;

                    QList<int> laneIDs;

                    for(int k=0;k<nTarget;++k){

                        int lid = -1;
                        memcpy( &lid, &(com[index]), sizeof(int) );
                        index += sizeof(int);

                        laneIDs.append( lid );
                    }

                    refVforDev /= 3.6;
                    vDevP /= 3.6;
                    vDevM = fabs(vDevM) / 3.6;
                    decel = fabs(decel);

                    qDebug() << "Receive FLSV command: ";
                    qDebug() << "  refVforDev = " << refVforDev << "[m/s]";
                    qDebug() << "  vDevP = " << vDevP << "[m/s]";
                    qDebug() << "  vDevM = " << vDevM << "[m/s]";
                    qDebug() << "  accel = " << accel << "[m/s^2]";
                    qDebug() << "  decel = " << decel << "[m/s^2]";
                    qDebug() << "  nTarget = " << nTarget << "  size of laneIDs = " << laneIDs.size();

                    emit ChangeLaneAssignedVelocityControlParameters(refVforDev,vDevP,vDevM,accel,decel,laneIDs);
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

                    index = pos;
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

                    index = pos;
                }
                else if( com[index+1] == 'A' && com[index+2] == 'r' && com[index+3] == 'a' ){  // Resume Agent Generation

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

                    index = pos;
                }
                else if( com[index+1] == 'F' && com[index+2] == 'C' && com[index+3] == 'S'){   // Force Change Speed

                    int aID = -2;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );
                    if( aID >= -1 && aID < maxAgent ){

                        float Speed = 0.0;
                        memcpy( &Speed, &(com[index+8]), sizeof(float) );
                        Speed /= 3.6;

                        emit ForceChangeSpeed( aID, Speed );

                        //qDebug() << "Force Change Speed: targetID=" << aID << " speed=" << (Speed * 3.6);
                    }

                    index += 12;
                }
                else if( com[index+1] == 'S' && com[index+2] == 'C' && com[index+3] == 'P' ){  // Change Scenario Speed Profile Data

                    int aID = -1;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );

                    int nPoint = 0;
                    memcpy( &nPoint, &(com[index+8]), sizeof(int) );

                    int pos = index + 12;

                    if( aID >= 0 && aID < maxAgent && nPoint > 0 ){

                        QList<float> td;
                        QList<float> vd;

                        for(int k=0;k<nPoint;++k){

                            float t = 0.0;
                            memcpy( &t, &(com[pos]), sizeof(float) );
                            pos += sizeof(float);

                            td.append( t );

                            float v = 0.0;
                            memcpy( &v, &(com[pos]), sizeof(float) );
                            pos += sizeof(float);

                            v /= 3.6;

                            vd.append( v );
                        }

                        qDebug() << "Change Scenario Speed Profile: id= " << aID;
                        qDebug() << " t = " << td;
                        qDebug() << " v = " << vd;

                        emit ChangeSpeedProfile( aID, td, vd );
                    }

                    index += pos;
                }
                else if( com[index+1] == 'R' && com[index+2] == 'L' && com[index+3] == 'S' ){  // Change Road Lane SpeedLimit Data

                    int nLane = 0;
                    memcpy( &nLane, &(com[index+4]), sizeof(int) );

                    int pos = index + 8;

                    if( nLane > 0 ){

                        QList<int> td;
                        QList<float> vd;

                        for(int k=0;k<nLane;++k){

                            int t = -1;
                            memcpy( &t, &(com[pos]), sizeof(int) );
                            pos += sizeof(int);

                            td.append( t );

                            float v = 0.0;
                            memcpy( &v, &(com[pos]), sizeof(float) );
                            pos += sizeof(float);

                            v /= 3.6;

                            vd.append( v );
                        }

                        emit ChangeRoadLaneSpeedLimit( td, vd );
                    }

                    index = pos;
                }
                else if( com[index+1] == 'P' && com[index+2] == 'M' && com[index+3] == 'S' ){  // Change Move Speed of Pedestrian

                    float vChild = 0;
                    memcpy( &vChild, &(com[index+4]), sizeof(float) );

                    float vAdult = 0;
                    memcpy( &vAdult, &(com[index+8]), sizeof(float) );

                    float vAged = 0;
                    memcpy( &vAged, &(com[index+12]), sizeof(float) );

                    QList<float> vPedest;
                    vPedest.append( vChild );
                    vPedest.append( vAdult );
                    vPedest.append( vAged );

                    emit ChangeMoveSpeedPedestrian( vPedest );

                    index += 16;
                }
                else if( com[index+1] == 'O' && com[index+2] == 'I' && com[index+3] == 'P' ){   // Set Optional Image Paramter

                    int idf = 0;
                    memcpy( &idf, &(com[index+4]), sizeof(int) );

                    int statf = 0;
                    memcpy( &statf, &(com[index+8]), sizeof(int) );

                    int pos = index + 12;

                    QList<float> p;
                    p.append( idf );
                    p.append( statf );

                    float x = 0.0;
                    memcpy( &x, &(com[pos]), sizeof(float) );
                    pos += sizeof(float);
                    p.append( x );

                    float y = 0.0;
                    memcpy( &y, &(com[pos]), sizeof(float) );
                    pos += sizeof(float);
                    p.append( y );

                    float z = 0.0;
                    memcpy( &z, &(com[pos]), sizeof(float) );
                    pos += sizeof(float);
                    p.append( z );

                    float rot = 0.0;
                    memcpy( &rot, &(com[pos]), sizeof(float) );
                    pos += sizeof(float);
                    p.append( rot );

                    float scale = 0.0;
                    memcpy( &scale, &(com[pos]), sizeof(float) );
                    pos += sizeof(float);
                    p.append( scale );

                    emit ChangeOptionalImageParams( p );

                    index = pos;
                }
                else if( com[index+1] == 'S' && com[index+2] == 'E' && com[index+3] == 'T' ){   // Set Event Trigger by FuncExtend

                    int eventType = 0;
                    memcpy( &eventType, &(com[index+4]), sizeof(int) );

                    int id = 0;
                    memcpy( &id, &(com[index+8]), sizeof(int) );

                    int idx = 0;
                    memcpy( &idx, &(com[index+12]), sizeof(int) );

                    int option = 0;
                    memcpy( &option, &(com[index+16]), sizeof(int) );

                    qDebug() << "Receive Event Trigger Command from FE: Type = " << eventType
                             << " ID = " << id
                             << " Index = " << idx
                             << " Option = " << option;

                    emit SetEventTriggerByFuncExtend( eventType, id, idx, option );

                    index += 20;
                }
                else if( com[index+1] == 'V' && com[index+2] == 'B' && com[index+3] == 'S' ){   // Set Brake Lamp Override by FuncExtend

                    int aID = 0;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );

                    int state = 0;
                    memcpy( &state, &(com[index+8]), sizeof(int) );

                    emit SetBrakeLampOverride( aID, state );

                    index += 12;
                }
                else if( com[index+1] == 'I' && com[index+2] == 'S' && com[index+3] == 'S' ){  // Set Scenario Vehicle Init States



                    int pos = index + 4;

                    int nSV = 0;
                    memcpy( &nSV, &(com[pos]), sizeof(int) );
                    pos += sizeof(int);

                    qDebug() << "Received FISS command: nSV = " << nSV;

                    if( nSV > 0 ){

                        QList<int> aIDs;
                        QList<float> Xs;
                        QList<float> Ys;
                        QList<float> Zs;
                        QList<float> Psis;
                        QList<float> Vs;

                        for(int k=0;k<nSV;++k){

                            int aID = 0;
                            memcpy( &aID, &(com[pos]), sizeof(int) );
                            pos += sizeof(int);

                            float X = 0.0;
                            memcpy( &X, &(com[pos]), sizeof(float) );
                            pos += sizeof(float);

                            float Y = 0.0;
                            memcpy( &Y, &(com[pos]), sizeof(float) );
                            pos += sizeof(float);

                            float Z = 0.0;
                            memcpy( &Z, &(com[pos]), sizeof(float) );
                            pos += sizeof(float);

                            float Psi = 0.0;
                            memcpy( &Psi, &(com[pos]), sizeof(float) );
                            pos += sizeof(float);

                            float V = 0.0;
                            memcpy( &V, &(com[pos]), sizeof(float) );
                            pos += sizeof(float);

                            aIDs.append( aID );
                            Xs.append( X );
                            Ys.append( Y );
                            Zs.append( Z );
                            Psis.append( Psi );
                            Vs.append( V );
                        }

                        emit SetScenarioVehicleInitStates(aIDs,Xs,Ys,Zs,Psis,Vs);
                    }

                    index = pos;
                }
                else if( com[index+1] == 'N' && com[index+2] == 'p' && com[index+3] == 'a' ){  // Stop Agent Generation Temporally for Node

                    int nData = -1;
                    memcpy( &nData, &(com[index+4]), sizeof(int) );

                    QList<int> nodeList;

                    int pos = index + 8;
                    for(int i=0;i<nData;++i){
                        int nID = -1;
                        memcpy( &nID, &(com[pos]), sizeof(int) );
                        pos += sizeof(int);

                        nodeList.append( nID );
                    }

                    emit SetAgentGenerationNotAllowFlagForNodes( nodeList, false );
                    index = pos;
                }
                else if( com[index+1] == 'N' && com[index+2] == 'r' && com[index+3] == 'a' ){  // Resume Agent Generation for Node

                    int nData = -1;
                    memcpy( &nData, &(com[index+4]), sizeof(int) );

                    QList<int> nodeList;

                    int pos = index + 8;
                    for(int i=0;i<nData;++i){
                        int nID = -1;
                        memcpy( &nID, &(com[pos]), sizeof(int) );
                        pos += sizeof(int);

                        nodeList.append( nID );
                    }

                    emit SetAgentGenerationNotAllowFlagForNodes( nodeList, true );
                    index = pos;
                }
                else if( com[index+1] == 'A' && com[index+2] == 'S' && com[index+3] == 'V' ){  // Appear Stopping Vehicle

                    int aID = -1;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );

                    float x = 0.0;
                    memcpy( &x, &(com[index+8]), sizeof(float) );

                    float y = 0.0;
                    memcpy( &y, &(com[index+12]), sizeof(float) );

                    float psi = 0.0;
                    memcpy( &psi, &(com[index+16]), sizeof(float) );

                    qDebug() << "Receive Appear Stopping Vehicle Command: aID = " << aID
                             << " x = " << x
                             << " y = " << y
                             << " psi = " << psi;

                    emit AppearStoppingVehicle( aID, x, y, psi );

                    index += 20;
                }
                else if( com[index+1] == 'E' && com[index+2] == 'C' && com[index+3] == 'F' ){  // Set External Control Factor

                    int aID = 0;
                    memcpy( &aID, &(com[index+4]), sizeof(int) );

                    float aimPointFactor = 0;
                    memcpy( &aimPointFactor, &(com[index+8]), sizeof(float) );

                    float steerGain = 0;
                    memcpy( &steerGain, &(com[index+12]), sizeof(float) );

                    emit SetAgentExternalControlParams( aID, aimPointFactor, steerGain );

                    index += 16;
                }
                else if( com[index+1] == 'P' && com[index+2] == 'P' && com[index+3] == 'C' ){  // Change Pedestrian Path for Scenario Pedestrian

                    QList<float> xPos;
                    QList<float> yPos;

                    index += 4;

                    int aID = -1;
                    memcpy( &aID, &(com[index]), sizeof(int) );
                    index += sizeof(int);

                    int nData = -1;
                    memcpy( &nData, &(com[index]), sizeof(int) );
                    index += sizeof(int);

                    for(int i=0;i<nData;++i){
                        float tx = 0.0;
                        memcpy( &tx, &(com[index]), sizeof(float) );
                        index += sizeof(float);

                        float ty = 0.0;
                        memcpy( &ty, &(com[index]), sizeof(float) );
                        index += sizeof(float);

                        xPos.append( tx );
                        yPos.append( ty );
                    }

                    qDebug() << "Receive Change Pedestrian Path Command: aID = " << aID
                             << " nData = " << nData;
                    for(int n=0;n<nData;++n){
                        qDebug() << " [" << n << "] : x = " << xPos[n] << " , y = " << yPos[n];
                    }

                    emit ChangePedestPathForScenarioPedestrian( aID, xPos, yPos );
                }
                else{
                    index++;
                }
            }
            else{
                index++;
            }
//        }
    }
}


void UDPThread::SetMaxAgentNumber(int maNum)
{
    maxAgent = maNum;

    qDebug() << "[UDPThread::SetMaxAgentNumber] maxAgent = " << maxAgent;
}


void UDPThread::SetNumberTrafficSignal(int n)
{
    numberTrafficSignal = n;

    qDebug() << "[UDPThread::SetNumberTrafficSignal]  numberTrafficSignal = " << numberTrafficSignal;
}



void UDPThread::SendDSMoveCommand(int targetID, float x, float y, float psi)
{
    if( SInterfaceObjIDs.contains( targetID ) == false ){
        return;
    }

    char sendDataBuf[255];

    memset( sendDataBuf, 0, 255 );

    sendDataBuf[0] = 'F';
    sendDataBuf[1] = 'W';

    memcpy( &(sendDataBuf[2]), &x, sizeof(float) );
    memcpy( &(sendDataBuf[6]), &y, sizeof(float) );
    memcpy( &(sendDataBuf[10]), &psi, sizeof(float) );

    sendDataBuf[14] = 'y';

    int sendSize = 15;

    QString targetSockName = QString("S-Interface[%1]").arg( targetID );

    for(int i=0;i<sendSocks.size();++i){

        if( sendSocks[i]->sock.objectName().contains(targetSockName) == false ){
            continue;
        }

        sendSocks[i]->sock.writeDatagram( sendDataBuf,
                                          sendSize,
                                          QHostAddress(sendSocks[i]->ipAddress),
                                          sendSocks[i]->to_port);

        sendSocks[i]->sock.flush();
        break;
    }
}


void UDPThread::SendScenarioData(QString ipAddr, int port, char *sendBuf, int dataSize)
{
    QUdpSocket *sock = new QUdpSocket();
    sock->writeDatagram( sendBuf, dataSize, QHostAddress(ipAddr), port);
    sock->flush();
}



void UDPThread::SendToFE(char *buf, int size)
{
    //
    //  Send data to Function Extender
    //   * This is because the data send to function extender should contain s-interface object
    //     data to transmit lateral deviation from path to function extender.
    //     UE4 can not accept s-interface object data from Re:sim, cause the data is send to UE4
    //     from S-interface itself.
    //
    for(int i=0;i<sendSocks.size();++i){

        if( sendSocks[i]->sock.objectName().contains("Function Extender") == false ){
            continue;
        }

        sendSocks[i]->sock.writeDatagram( buf,
                                          size,
                                          QHostAddress(sendSocks[i]->ipAddress),
                                          sendSocks[i]->to_port);

        sendSocks[i]->sock.flush();
    }
}



void UDPThread::SendToUE4(int SInterObjID, char *buf, int size)
{
    QString UE4SockName = QString("UE4App[%1,").arg( SInterObjID );

    for(int j=0;j<sendSocks.size();++j){

        if( sendSocks[j]->sock.objectName().contains( UE4SockName ) == true ){

            sendSocks[j]->sock.writeDatagram( buf,
                                              size,
                                              QHostAddress(sendSocks[j]->ipAddress),
                                              sendSocks[j]->to_port);

            sendSocks[j]->sock.flush();
        }
    }
}


void UDPThread::SendToSCore()
{
    char sendDataBuf[5];

    memset( sendDataBuf, 0, 5 );

    sendDataBuf[0] = 'R';
    sendDataBuf[1] = 'S';
    sendDataBuf[2] = 'd';

    int sendSize = 3;

    if( scoreSendSockIndex >= 0 && scoreSendSockIndex < sendSocks.size() ){
        sendSocks[scoreSendSockIndex]->sock.writeDatagram( sendDataBuf,
                                                           sendSize,
                                                           QHostAddress( sendSocks[scoreSendSockIndex]->ipAddress ),
                                                           sendSocks[scoreSendSockIndex]->to_port );
        sendSocks[scoreSendSockIndex]->sock.flush();
    }

    for(int ns=0;ns<SInterfaceObjIDs.size();++ns){
        recvFromSinterfaceObjFlag[ns] = false;
    }
}
