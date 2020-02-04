#ifndef NETWORKDRIVECHECK_H
#define NETWORKDRIVECHECK_H

#include <QString>

extern void GetNetworkDrive();
extern QString CheckNetworkDrive(QString filename);
extern QString GetNetworkDrivePair(QString fullpath);
extern void ReleaseNetworkDriveInfo();

#endif // NETWORKDRIVECHECK_H
