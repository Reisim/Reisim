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


#include "configwindow.h"
#include "networkdrivecheck.h"
#include <QDebug>


ConfigWindow::ConfigWindow(QWidget *parent) : QWidget(parent)
{
    newSetting = new QAction();
    newSetting->setStatusTip( tr("New") );
    newSetting->setIcon(QIcon(":/images/new.png"));
    connect( newSetting, SIGNAL(triggered()), this, SLOT(NewConfig()));

    openSetting = new QAction( tr("&Open"), this );
    openSetting->setStatusTip( tr("Open") );
    openSetting->setIcon(QIcon(":/images/load_document.png"));
    connect( openSetting, SIGNAL(triggered()), this, SLOT(OpenConfig()));

    saveSetting = new QAction( tr("&Save"), this );
    saveSetting->setStatusTip( tr("Save") );
    saveSetting->setIcon(QIcon(":/images/save.png"));
    connect( saveSetting, SIGNAL(triggered()), this, SLOT(SaveConfig()));

    saveAsSetting = new QAction( tr("&SaveAs"), this );
    saveAsSetting->setStatusTip( tr("SaveAs") );
    saveAsSetting->setIcon(QIcon(":/images/saveas.png"));
    connect( saveAsSetting, SIGNAL(triggered()), this, SLOT(SaveAsConfig()));

    settingToolBar = new QToolBar();
    settingToolBar->addAction( newSetting );
    settingToolBar->addAction( openSetting );
    settingToolBar->addAction( saveSetting );
    settingToolBar->addAction( saveAsSetting );

    //----------------------

    QGridLayout *gridLay = new QGridLayout();

    gridLay->addWidget( new QLabel("Scenario File"), 0, 0, 1, 1 );
    gridLay->addWidget( new QLabel("Only Filename"), 1, 0, 1, 1 );
    gridLay->addWidget( new QLabel("Output Log File"), 2, 0, 1, 1 );
    gridLay->addWidget( new QLabel("    Log Output Folder"), 3, 0, 1, 1 );
    gridLay->addWidget( new QLabel("    Log File Name"), 4, 0, 1, 1 );
    gridLay->addWidget( new QLabel("Work with Driving Simulator"), 5, 0, 1, 1 );
    gridLay->addWidget( new QLabel("Restart File"), 6, 0, 1, 1 );
    gridLay->addWidget( new QLabel("Calculation Time Step"), 7, 0, 1, 1 );
    gridLay->addWidget( new QLabel("Base-Map File"), 8, 0, 1, 1 );


    QPushButton *selectScenario = new QPushButton("Select");
    selectScenario->setIcon( QIcon(":/images/select.png") );
    selectScenario->setFixedWidth(60);
    connect(selectScenario,SIGNAL(clicked(bool)),this,SLOT(SelectScenario()));
    gridLay->addWidget( selectScenario, 0, 1, 1, 1 );

    cbOnlyFilename = new QCheckBox();
    gridLay->addWidget( cbOnlyFilename, 1, 1, 1, 1 );

    cbOutputLogFile = new QCheckBox();
    gridLay->addWidget( cbOutputLogFile, 2, 1, 1, 1 );

    QPushButton *selectLogOutputFolder = new QPushButton("Select");
    selectLogOutputFolder->setIcon( QIcon(":/images/select.png") );
    selectLogOutputFolder->setFixedWidth(60);
    connect(selectLogOutputFolder,SIGNAL(clicked(bool)),this,SLOT(SelectLogOutputFolder()));
    gridLay->addWidget( selectLogOutputFolder, 3, 1, 1, 1 );

    QPushButton *selectLogFileName = new QPushButton("Select");
    selectLogFileName->setIcon( QIcon(":/images/select.png") );
    selectLogFileName->setFixedWidth(60);
    connect(selectLogFileName,SIGNAL(clicked(bool)),this,SLOT(SelectLogFileName()));
    gridLay->addWidget( selectLogFileName, 4, 1, 1, 1 );

    cbDSMode = new QCheckBox();
    gridLay->addWidget( cbDSMode, 5, 1, 1, 1 );

    QPushButton *selectRestartFile = new QPushButton("Select");
    selectRestartFile->setIcon( QIcon(":/images/select.png") );
    selectRestartFile->setFixedWidth(60);
    connect(selectRestartFile,SIGNAL(clicked(bool)),this,SLOT(SelectRestartFile()));
    gridLay->addWidget( selectRestartFile, 6, 1, 1, 1 );

    scenarioFilename = new QLabel("Not selected.");
    scenarioFilename->setFixedWidth( 600 );
    scenarioFilename->setWordWrap(true);
    gridLay->addWidget( scenarioFilename, 0, 2, 1, 1 );

    logSaveFolder = new QLabel("Not selected.");
    logSaveFolder->setFixedWidth( 600 );
    logSaveFolder->setWordWrap(true);
    gridLay->addWidget( logSaveFolder, 3, 2, 1, 1 );

    logFileName = new QLabel("Not selected.");
    logFileName->setFixedWidth( 600 );
    logFileName->setWordWrap(true);
    gridLay->addWidget( logFileName, 4, 2, 1, 1 );

    restartFilename = new QLabel("Not selected.");
    restartFilename->setFixedWidth( 600 );
    restartFilename->setWordWrap(true);
    gridLay->addWidget( restartFilename, 6, 2, 1, 1 );

    calTimeStep = new QDoubleSpinBox();
    calTimeStep->setMinimum( 0.001 );
    calTimeStep->setMaximum( 0.1 );
    calTimeStep->setValue( 0.02 );
    calTimeStep->setSingleStep(0.02);
    calTimeStep->setSuffix("[s]");
    gridLay->addWidget( calTimeStep, 7, 1, 1, 1, Qt::AlignLeft );

    QPushButton *selectBaseMap = new QPushButton("Select");
    selectBaseMap->setIcon( QIcon(":/images/select.png") );
    selectBaseMap->setFixedWidth(60);
    connect(selectBaseMap,SIGNAL(clicked(bool)),this,SLOT(SelectBaseMap()));
    gridLay->addWidget( selectBaseMap, 8, 1, 1, 1 );

    baseMapFilename = new QLabel("Not selected.");
    baseMapFilename->setFixedWidth( 600 );
    baseMapFilename->setWordWrap(true);
    gridLay->addWidget( baseMapFilename, 8, 2, 1, 1 );


    //----------------------

    QVBoxLayout *mainLayout = new QVBoxLayout();

    mainLayout->addWidget( settingToolBar );
    mainLayout->addLayout( gridLay );

    mainLayout->addStretch();

    setLayout( mainLayout );

    setWindowTitle( QString("Configulation: no filename[*]") );

    setFixedSize( sizeHint() );
}


void ConfigWindow::NewConfig()
{
    if( isWindowModified() ){
        int ret = QMessageBox::warning(this,"Clear config","Sure to continue?", QMessageBox::Ok, QMessageBox::No);
        if( ret == QMessageBox::No ){
            return;
        }
    }

    scenarioFilename->setText( "Not selected." );
    logSaveFolder->setText( "Not selected." );
    logFileName->setText( "Not selected." );
    restartFilename->setText( "Not selected." );
    baseMapFilename->setText( "Not selected." );

    cbOutputLogFile->setCheckState( Qt::Unchecked );
    cbDSMode->setCheckState( Qt::Unchecked );

    calTimeStep->setValue( 0.02 );

    currentConfigFilename = QString();

    setWindowModified(false);
    setWindowTitle( QString("Configulation: no filename[*]") );
}


void ConfigWindow::OpenConfig()
{
    qDebug() << "[OpenConfig]";

    if( isWindowModified() ){
        int ret = QMessageBox::warning(this,"Clear config","Sure to continue?", QMessageBox::Ok, QMessageBox::No);
        if( ret == QMessageBox::No ){
            return;
        }
    }

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Choose File"),
                                                    ".",
                                                    tr("re:sim config file(*.rc.txt)"));

    if( fileName.isNull() == false ){
        qDebug() << "   filename = " << fileName;


        QFile file( CheckNetworkDrive(fileName) );
        if( !file.open(QIODevice::ReadOnly | QIODevice::Text) ){
            QMessageBox::warning(this,"Error","Cannot open file: " + fileName);
            return;
        }


        scenarioFilename->setText( "Not selected." );
        logSaveFolder->setText( "Not selected." );
        logFileName->setText( "Not selected." );
        restartFilename->setText( "Not selected." );
        baseMapFilename->setText( "Not selected." );


        QTextStream in(&file);
        QString line;

        line = in.readLine();
        line = in.readLine();
        if( line.contains("Re:sim Configulation File") == false ){
            QMessageBox::warning(this,"Error","The file is not Re:sim Configulation File");
            file.close();
            return;
        }
        line = in.readLine();

        currentConfigFilename = fileName;

        while( in.atEnd() == false ){

            line = in.readLine();
            if( line.isNull() || line.isEmpty() || line.startsWith("#") ){
                continue;
            }

            QStringList divLine = line.split(";");

            QString tag = QString( divLine[0] ).trimmed();

            if( tag == QString("DS Mode") ){

                QString val = QString(divLine[1]).trimmed();
                if( val == QString("yes") ){
                    cbDSMode->setCheckState( Qt::Checked );
                }
                else{
                    cbDSMode->setCheckState( Qt::Unchecked );
                }

            }
            else if( tag == QString("Output Log File") ){

                QString val = QString(divLine[1]).trimmed();
                if( val == QString("yes") ){
                    cbOutputLogFile->setCheckState( Qt::Checked );
                }
                else{
                    cbOutputLogFile->setCheckState( Qt::Unchecked );
                }

            }
            else if( tag == QString("Scenario File") ){

                scenarioFilename->setText( QString(divLine[1]).trimmed() );

            }
            else if( tag == QString("Base-Map Files") ){

                baseMapFilename->setText( QString(divLine[1]).trimmed() );

            }
            else if( tag == QString("Log Output Folder") ){

                logSaveFolder->setText( QString(divLine[1]).trimmed() );

            }
            else if( tag == QString("CSV Output File") ){

                logFileName->setText( QString(divLine[1]).trimmed() );

            }
            else if( tag == QString("Restart File") ){

                if( QString(divLine[1]).trimmed().isEmpty() ){
                    restartFilename->setText( "Not selected." );
                }
                else{
                    restartFilename->setText( QString(divLine[1]).trimmed() );
                }
            }
            else if( tag == QString("Calculation Time Step") ){

                calTimeStep->setValue( QString(divLine[1]).trimmed().toDouble() );
            }
        }

        file.close();

        setWindowModified(false);
        setWindowTitle( QString("Configulation: %1[*]").arg(currentConfigFilename) );
    }
    else{
        qDebug() << "   filename is null";
    }

    setFixedSize( sizeHint() );
    update();
}


void ConfigWindow::SaveAsConfig()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Choose File"),
                                                    ".",
                                                    tr("re:sim config file(*.rc.txt)"));

    if( fileName.isNull() == false ){

        if( fileName.endsWith(".rc.txt") == false ){
            fileName += QString(".rc.txt");
        }

        qDebug() << "filename = " << fileName;

    }
    else{
        qDebug() << "filename is null";
        return;
    }

    currentConfigFilename = fileName;
    SaveConfig();
}


void ConfigWindow::SaveConfig()
{
    if( currentConfigFilename.isNull() ){
        SaveAsConfig();
        return;
    }


    QFile file( CheckNetworkDrive(currentConfigFilename) );
    if( file.open(QIODevice::WriteOnly | QIODevice::Text) ){

        QTextStream out(&file);
        out << "=============================================\n";
        out << "         Re:sim Configulation File\n";
        out << "=============================================\n";
        out << "\n";
        out << "# 'DS Mode' should be ahead of 'Scenario File'\n";
        out << "DS Mode ; ";
        if( cbDSMode->isChecked() ){
            out << "yes\n";
        }
        else{
            out << "no\n";
        }
        out << "\n";

        if( cbOnlyFilename->isChecked() == true ){
            QString tmpStr = scenarioFilename->text();
            QStringList tmpStrDiv = tmpStr.replace("\\","/").split("/");
            out << "Scenario File ; " <<  QString(tmpStrDiv.last()) << "\n";
        }
        else{
            out << "Scenario File ; " << scenarioFilename->text() << "\n";
        }

        if( baseMapFilename->text().contains("Not selected") == false ){
            out << "Base-Map Files ; " << baseMapFilename->text() << "\n";
        }

        out << "Calculation Time Step ; " << calTimeStep->value() << "\n";
        out << "\n";
        out << "Output Log File ; ";
        if( cbOutputLogFile->isChecked() ){
            out << "yes\n";
        }
        else{
            out << "no\n";
        }
        out << "Log Output Folder ; " << logSaveFolder->text() << "\n";
        out << "CSV Output File ; " << logFileName->text() << "\n";

        if( restartFilename->text().contains("Not selected") == false ){
            out << "Restart File ; " << restartFilename->text() << "\n";
        }


        file.close();

    }

    setWindowModified(false);
    setWindowTitle( QString("Configulation: %1[*]").arg(currentConfigFilename) );

    qDebug() << "Configulation File saved.";
}


void ConfigWindow::SelectScenario()
{
    qDebug() << "[SelectScenario]";

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Choose Scenario File"),
                                                    ".",
                                                    tr("re:sim scenario file(*.rs.txt)"));

    if( fileName.isNull() == false ){
        qDebug() << "   filename = " << fileName;
        scenarioFilename->setText( fileName );
        setWindowModified(true);
    }
    else{
        qDebug() << "   filename is null";
    }

    setFixedSize( sizeHint() );
}


void ConfigWindow::SelectBaseMap()
{
    qDebug() << "[SelectBaseMap]";

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Choose Base-Map Image File"),
                                                    ".",
                                                    tr("base-map image file(*.txt)"));

    if( fileName.isNull() == false ){
        qDebug() << "   filename = " << fileName;
        baseMapFilename->setText( fileName );
        setWindowModified(true);
    }
    else{
        qDebug() << "   filename is null";
        baseMapFilename->setText( "Not selected." );
        setWindowModified(true);
    }

    setFixedSize( sizeHint() );
}


void ConfigWindow::SelectLogOutputFolder()
{
    qDebug() << "[SelectLogOutputFolder]";

    QString dirName = QFileDialog::getExistingDirectory(this,
                                                        tr("Choose Output Folder"),
                                                        ".");

    if( dirName.isNull() == false ){
        qDebug() << "   dirName = " << dirName;
        logSaveFolder->setText( dirName );
        setWindowModified(true);
    }
    else{
        qDebug() << "   dirName is null";
    }

    setFixedSize( sizeHint() );
}


void ConfigWindow::SelectLogFileName()
{
    qDebug() << "[SelectLogFileName]";

    QString tmpLFName = QFileDialog::getSaveFileName(this,
                                                     tr("Choose File"),
                                                     ".",
                                                     tr("re:sim log file(*.vbf.csv)"));

    if( tmpLFName.isNull() == false ){

        QStringList divStr = tmpLFName.split("/");

        tmpLFName = QString(divStr.last());
        if( tmpLFName.endsWith(".vbf.csv") == false ){
            tmpLFName += QString(".vbf.csv");
        }

        qDebug() << "   logFileName = " << tmpLFName;
        logFileName->setText( tmpLFName );
        setWindowModified(true);
    }
    else{
        qDebug() << "   logFileName is null";
    }

    setFixedSize( sizeHint() );
}


void ConfigWindow::SelectRestartFile()
{
    qDebug() << "[SelectRestartFile]";

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Choose Restart File"),
                                                    ".",
                                                    tr("re:sim snapshot file(*.ss.txt)"));

    if( fileName.isNull() == false ){
        qDebug() << "   filename = " << fileName;
        restartFilename->setText( fileName );
        setWindowModified(true);
    }
    else{
        qDebug() << "   filename is null; clear restart file";
        restartFilename->setText( "Not selected." );
        setWindowModified(true);
    }

    setFixedSize( sizeHint() );
}

