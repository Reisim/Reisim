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
#include <QMap>
#include <QThread>

#include "graphiccanvas.h"
#include "configwindow.h"


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void LoadSettingFile(QString,bool withArg = true);
    GraphicCanvas *GetPointerGraphicCanvas() { return canvas; }
    bool GetStopGraphicUpdate(){ return cbStopGraphicUpdate->isChecked(); }
    bool GetFixCameraToObj(){ return cbFixCameraToObj->isChecked(); }
    void ReadPreferenceSetting();
    void WritePreferenceSetting();


protected:
    void keyPressEvent(QKeyEvent *);

signals:
    void SetScenraioFile(QString);
    void SetSupplementFile(QString);
    void SetBasemapFile(QString,QString);
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
    void ResetZeye();
    void SpeedAdjustMoved(int);
    void ShowPathCGChanged(bool);
    void ShowTSCGChanged(bool);
    void ShowBaseMapChanged(bool);
    void ShowVIDChanged(bool);
    void ShowPathIDChanged(bool);
    void ShowTSIDChanged(bool);
    void ExitProgram();
    void StopGraphicUpdateChanged(bool);
    void FixCameraToObjChanged(bool);
    void SetFontScale(int);
    void OutputRestartData();
    void ShowOptionalImage(int,float,float,float,float,float);
    void HideOptionalImage(int);

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
    QPushButton *resetZeye;

    QCheckBox *cbShowPathCG;
    QCheckBox *cbShowBaseMap;
    QCheckBox *cbShowTSCG;

    QCheckBox *cbShowVID;
    QCheckBox *cbShowPathID;
    QCheckBox *cbShowTSID;
    QCheckBox *cbStopGraphicUpdate;
    QSlider *fontScaler;
    QPushButton *snapshotBtn;

    QLabel *SInterfaceScalerLabel;
    QSlider *SInterfaceScaler;

    QCheckBox *cbFixCameraToObj;
    QSpinBox *cameraFixToObjID;

    ConfigWindow *confWin;

    bool DSMode;
    int simState;

    QMap<QString,int> preferenceSetting;
};

#endif // MAINWINDOW_H
