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


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QAction>
#include <QToolBar>
#include <QPushButton>
#include <QFile>
#include <QTextStream>
#include <QFileDialog>
#include <QMessageBox>
#include <QLabel>
#include <QSlider>
#include <QCheckBox>
#include <QSpinBox>

#include "graphiccanvas.h"
#include "configwindow.h"


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void LoadSettingFile(QString);
    GraphicCanvas *GetPointerGraphicCanvas() { return canvas; }

protected:
    void keyPressEvent(QKeyEvent *);

signals:
    void SetScenraioFile(QString);
    void SetSupplementFile(QString);
    void SetDSMode();
    void SetStartTrigger(int);
    void SetLogOutputFolder(QString);
    void SetLogFileName(QString);
    void SetLogOutputInterval(int);
    void SimulationStart();
    void SimulationStop();
    void SimulationPause();
    void SimulationResume();
    void SetSpeedAdjustVal(int);
    void SetStopGraphicUpdate(bool);
    void SetSimulationFrequency(int);
    void SetTmpStpoTime(int,int,int);
    void OutputRestartData(QString);
    void SetRestartFile(QString);
    void SetDSMoveTarget(int);


public slots:
    void OpenSettingFile();
    void StartSimulation();
    void StopSimulation();
    void PauseSimulation();
    void ResumeSimulation();
    void UpdateSimulationTimeDisplay(QString);
    void SetAgentPointerToCanvas(Agent**);
    void SetTrafficSignalPointerToCanvas(QList<TrafficSignal *>);
    void SetMaxNumberAgentToCanvas(int);
    void SetNumberTrafficSignalToCanvas(int);
    void RedrawRequest();
    void SetRoadDataToCanvas(Road*);
    void ShowConfigWindow();
    void ResetSpeedAdjuster();
    void SpeedAdjustMoved(int);
    void ShowVIDChanged(bool);
    void ShowPathIDChanged(bool);
    void ShowTSIDChanged(bool);
    void ExitProgram();
    void StopGraphicUpdateChanged(bool);
    void FixCameraToObjChanged(bool);
    void SetFontScale(int);
    void OutputRestartData();


private:
    GraphicCanvas *canvas;
    QLabel *simulationTimeDisplay;


    QAction *openSimSetting;
    QAction *quitProgram;
    QAction *editConfig;

    QToolBar *actionToolBar;

    QPushButton *startBtn;
    QPushButton *stopBtn;
    QPushButton *pauseBtn;
    QPushButton *resumeBtn;
    QSpinBox *animeSpeedAdjusterByValue;
    QSlider *animeSpeedAdjuster;
    QPushButton *resetSpeedAdjuster;
    QCheckBox *cbShowVID;
    QCheckBox *cbShowPathID;
    QCheckBox *cbShowTSID;
    QCheckBox *cbStopGraphicUpdate;
    QSlider *fontScaler;
    QPushButton *snapshotBtn;

    QCheckBox *cbFixCameraToObj;
    QSpinBox *cameraFixToObjID;

    ConfigWindow *confWin;

    bool DSMode;
    int simState;
};

#endif // MAINWINDOW_H
