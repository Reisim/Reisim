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
#include <QDoubleSpinBox>


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
    void SelectBaseMap();
    void SelectLogOutputFolder();
    void SelectLogFileName();
    void SelectRestartFile();


private:
    QToolBar *settingToolBar;

    QAction *newSetting;
    QAction *openSetting;
    QAction *saveSetting;
    QAction *saveAsSetting;
    QAction *closeSetting;

    QCheckBox *cbDSMode;
    QCheckBox *cbOutputLogFile;
    QCheckBox *cbOnlyFilename;

    QLabel *logSaveFolder;
    QLabel *logFileName;
    QLabel *scenarioFilename;
    QLabel *baseMapFilename;

    QLabel *restartFilename;
    QDoubleSpinBox *calTimeStep;


    QString currentConfigFilename;
};

#endif // CONFIGWINDOW_H
