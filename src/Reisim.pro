#-------------------------------------------------
#
# Project created by QtCreator 2018-12-15T23:35:51
#
#-------------------------------------------------

QT += core gui

QT += opengl
QT += network

CONFIG += console
CONFIG += c++11

DEFINES += SFMT_MEXP="19937"
#DEFINES += _PERFORMANCE_CHECK
#DEFINES += _PERFORMANCE_CHECK_AGENT
#DEFINES += _PERFORMANCE_CHECK_AGENT_PERCEPTION
#DEFINES += _PERFORMANCE_CHECK_AGENT_PERCEPTION_CORE
#DEFINES += _PERFORMANCE_CHECK_AGENT_RECOGNITION
#DEFINES += _PERFORMANCE_CHECK_AGENT_HAZARD

#DEFINES += _SHOW_AGENT_NUM_APPEAR


LIBS += C:\Qt\Tools\mingw730_64\x86_64-w64-mingw32\lib\libws2_32.a
LIBS += C:\Qt\Tools\mingw730_64\x86_64-w64-mingw32\lib\libmpr.a


#
#  Freetype2
#
INCLUDEPATH += "..\Reisim\libs\freetype\include\freetype2"

LIBS += "..\Reisim\libs\freetype\lib\libfreetype.a"
LIBS += "..\Reisim\libs\libpng\lib\libpng.a"
LIBS += "..\Reisim\libs\zlib\lib\libzlibstatic.a"
LIBS += "..\Reisim\libs\bzip2\lib\libbz2.a"



greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Reisim
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
    caldistributethread.cpp \
        main.cpp \
        mainwindow.cpp \
    graphiccanvas.cpp \
    networkdrivecheck.cpp \
    road.cpp \
    system_snapshot.cpp \
    systemthread.cpp \
    udpthread.cpp \
    simulationmanager.cpp \
    agent.cpp \
    vehicle.cpp \
    randomgenerator.cpp \
    agent_perception.cpp \
    agent_recognition.cpp \
    agent_hazardidentification.cpp \
    agent_riskevaluation.cpp \
    agent_control.cpp \
    agent_move.cpp \
    gltransform3d.cpp \
    road_path_calc.cpp \
    scenario.cpp \
    agent_route_management.cpp \
    logoutputthread.cpp \
    configwindow.cpp \
    eventmanager.cpp \
    trafficsignal.cpp

HEADERS += \
    caldistributethread.h \
        mainwindow.h \
    graphiccanvas.h \
    agent.h \
    networkdrivecheck.h \
    road.h \
    systemthread.h \
    udpthread.h \
    simulationmanager.h \
    vehicle.h \
    randomgenerator.h \
    gltransform3d.h \
    logoutputthread.h \
    configwindow.h \
    trafficsignal.h

RESOURCES += \
    resim_resource.qrc

RC_FILE = resim.rc
