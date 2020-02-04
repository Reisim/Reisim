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

#ifndef NETWORKDRIVECHECK_H
#define NETWORKDRIVECHECK_H

#include <QString>

extern void GetNetworkDrive();
extern QString CheckNetworkDrive(QString filename);
extern QString GetNetworkDrivePair(QString fullpath);
extern void ReleaseNetworkDriveInfo();

#endif // NETWORKDRIVECHECK_H
