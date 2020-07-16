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


#ifndef GRAPHICCANVAS_H
#define GRAPHICCANVAS_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QVector3D>
#include <QMatrix4x4>
#include <QString>
#include <QStringList>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QMap>
#include <QList>
#include "gltransform3d.h"

#include "agent.h"
#include "road.h"
#include "trafficsignal.h"


struct SimplePolygonVehicle
{
    QVector<GLfloat> vehiclePolygon;
    QOpenGLBuffer *vehiclePolygonBuffer;
    QOpenGLVertexArrayObject vehiclePolygonArray;
    bool isAllSet;
};

struct VehicleModel
{
    int modelID;
    int Type;
    float length;
    float width;
    float height;
    float wheelBase;
    float FRWeightRatio;
    float distRA2RE;
    float lf;
    float lr;
    float distCG2FE;
    float distCG2RE;

    bool objFileDataSet;
    QString polygonFileName;
    // Object Format Data
    //
    struct SimplePolygonVehicle simplePoly;
};

struct PersonModel
{
    int id;
    QVector<GLfloat> personPolygon;
    QOpenGLBuffer *personPolygonBuffer;
    QOpenGLVertexArrayObject personPolygonArray;
};

struct PathPolygon
{
    QVector<GLfloat> pathPolygonData;
    QOpenGLBuffer *pathPolygonsBuffer;
    QOpenGLVertexArrayObject pathPolygonsArray;
};

struct TrafficSignalPolygon
{
    QVector<GLfloat> TSPolygons;
    QOpenGLBuffer *TSPolygonsBuffer;
    QOpenGLVertexArrayObject TSPolygonsArray;
};


struct Character {
    GLuint TextureID;
    QSize Size;
    QSize Bearing;
    GLuint Advance;
};




class GraphicCanvas : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit GraphicCanvas(QOpenGLWidget *parent = nullptr);
    ~GraphicCanvas();

    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);

    void SetVehiclePolygon(int index,float lf,float lr,float width,float height);
    void SetPersonPolygon(int index,float width,float height,float depth);

    void SetTSData();
    void SetRoadData();
    void SetTrafficParticipantsData();


    void SetVIDFlag(bool v){ showVID = v; }
    void SetPathIDFlag(bool v){ showPathID = v; }
    void SetTSIDFlag(bool v){ showTSID = v; }
    void SetFontScale(int s){ fontScale = s; }

    int Get3DPhysCoordFromPickPoint(int xp,int yp, float &x,float &y);

    void LocateAtAgent(int id);

    float GetXeye(){ return X_eye; }
    float GetYeye(){ return Y_eye; }
    float GetZeye(){ return Z_eye; }


    Agent **agent;
    int maxAgent;

    QList<TrafficSignal *> trafficSignal;
    int numTrafficSignal;

    Road *road;

    bool trackingMode;
    int trackingObjID;
    void PushEyeCoord(){
        last_X_eye = X_eye;
        last_Y_eye = Y_eye;
        last_Z_eye = Z_eye;
        last_cameraYaw = cameraYaw;
        last_cameraPitch = cameraPitch;
    }
    void PopEyeCoord(){
        X_eye = last_X_eye;
        Y_eye = last_Y_eye;
        Z_eye = last_Z_eye;
        cameraYaw = last_cameraYaw;
        cameraPitch = last_cameraPitch;
        cameraQuat = QQuaternion(cos(cameraPitch*0.5), sin(cameraPitch*0.5) , 0.0 , 0.0 ) * QQuaternion(cos(cameraYaw*0.5), 0.0 , 0.0 , sin(cameraYaw*0.5));
    }


signals:
    void ShowAgentData(float x,float Y);
    void ChangeFontScale(int);
    void DSMove(float x,float Y);


protected:
    void mousePressEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void wheelEvent(QWheelEvent *e);


private:

    int currentWidth;
    int currentHeight;

    float X_eye;
    float Y_eye;
    float Z_eye;

    float X_trans;
    float Y_trans;
    float Z_trans;

    float last_X_eye;
    float last_Y_eye;
    float last_Z_eye;


    QVector2D mousePressPosition;
    float sx;
    float sy;

    // OpenGL State
    QOpenGLShaderProgram *program;

    int u_modelToWorld;
    int u_worldToView;
    QMatrix4x4 projection;
    GLTransform3D model2World;
    QQuaternion cameraQuat;

    float cameraYaw;
    float cameraPitch;

    float last_cameraYaw;
    float last_cameraPitch;

    QVector<struct VehicleModel *> vehicleModels;
    QVector<struct PersonModel *> personModels;

    QVector<struct TrafficSignalPolygon *> TSPolygons;

    struct PathPolygon *pathPolygons;


    // Shader Files
    QString vertexShaderProgramFile;
    QString fragmentShaderProgramFile;


    // Font
    QMap<GLchar, Character*> Characters;

    QOpenGLBuffer *VBOText_TS;
    QOpenGLVertexArrayObject VAOText_TS;

    QOpenGLBuffer *VBOText_Path;
    QOpenGLVertexArrayObject VAOText_Path;

    QOpenGLBuffer *VBOText;
    QOpenGLVertexArrayObject VAOText;


    bool showVID;
    bool showPathID;
    bool showTSID;

    int fontScale;
};

#endif // GRAPHICCANVAS_H
