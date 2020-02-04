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

#ifndef CONFIGWINDOW_H
#define CONFIGWINDOW_H

#include <QWidget>
#include <QAction>
#include <QMenu>
#include <QLabel>
#include <QCheckBox>
#include <QString>
#include <QStringList>
#include <QToolBar>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QSize>
#include <QMessageBox>


class ConfigWindow : public QWidget
{
    Q_OBJECT
public:
    explicit ConfigWindow(QWidget *parent = nullptr);

signals:

public slots:
    void NewConfig();
    void OpenConfig();
    void SaveConfig();
    void SaveAsConfig();

    void SelectScenario();
    void SelectLogOutputFolder();
    void SelectLogFileName();

private:
    QToolBar *settingToolBar;

    QAction *newSetting;
    QAction *openSetting;
    QAction *saveSetting;
    QAction *saveAsSetting;
    QAction *closeSetting;

    QCheckBox *cbDSMode;
    QCheckBox *cbOutputLogFile;

    QLabel *logSaveFolder;
    QLabel *logFileName;
    QLabel *scenarioFilename;


    QString currentConfigFilename;
};

#endif // CONFIGWINDOW_H
