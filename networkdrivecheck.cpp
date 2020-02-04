#include <QString>
#include <windows.h>
#include <winnetwk.h>
#include <QList>
#include <QString>
#include <QDebug>


void GetNetworkDrive();
QString CheckNetworkDrive(QString filename);
QString GetNetworkDrivePair(QString fullpath);
void ReleaseNetworkDriveInfo();


struct NetworkDriveInfo
{
    QString driveName;
    QString path;
};


QList<struct NetworkDriveInfo *> netDrive;


void GetNetworkDrive()
{
    system("net use");

    QString tmpDrive = QString("Z:");

    wchar_t driveNameWT[255];
    tmpDrive.toWCharArray( driveNameWT );

    DWORD dwResult,dwLength=255;
    wchar_t szRemoteName[255];
    dwResult = WNetGetConnection(driveNameWT, szRemoteName, &dwLength);

    QString remoteName = QString::fromWCharArray(szRemoteName).replace("\\","/");
    qDebug() << " remoteName for Z: = " << remoteName;

    if( remoteName.startsWith("//") ){

        struct NetworkDriveInfo *ndi = new struct NetworkDriveInfo;

        ndi->driveName = tmpDrive;
        ndi->path = remoteName;

        netDrive.append( ndi );

//        qDebug() << "Network Drive;  " << ndi->driveName << " " << ndi->path;
    }
}



QString CheckNetworkDrive(QString filename)
{
    if( netDrive.size() == 0 ){
        return filename;
    }

    if( filename.contains(":") == true && filename.indexOf(":") == 1 ){
        QString driveName = filename.mid(0,2);
        for(int i=0;i<netDrive.size();++i){
            if( netDrive[i]->driveName == driveName ){
                QString remoteName = netDrive[i]->path;
                filename = filename.replace( driveName, remoteName );
                break;
            }
        }
    }

    return filename;
}


QString GetNetworkDrivePair(QString fullpath)
{
    QString ret = QString();

    for(int i=0;i<netDrive.size();++i){
        if( fullpath.contains( netDrive[i]->driveName ) == true
                || fullpath.contains( netDrive[i]->path ) == true  ){
            ret = netDrive[i]->driveName + QString(" ") + netDrive[i]->path;
            break;
        }
    }

    return ret;
}


void ReleaseNetworkDriveInfo()
{
    for(int i=0;i<netDrive.size();++i){
        delete [] netDrive[i];
    }
    netDrive.clear();
}

