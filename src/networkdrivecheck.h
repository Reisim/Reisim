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


#ifndef NETWORKDRIVECHECK_H
#define NETWORKDRIVECHECK_H

#include <QString>

extern void GetNetworkDrive();
extern QString CheckNetworkDrive(QString filename);
extern QString GetNetworkDrivePair(QString fullpath);
extern void ReleaseNetworkDriveInfo();

#endif // NETWORKDRIVECHECK_H
