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


#include "graphiccanvas.h"
#include <QApplication>

#include "ft2build.h"
#include FT_FREETYPE_H    // This macro is defined as <freetype/freetype.h>


#include "networkdrivecheck.h"

#define FONT_SCALE  0.1


GraphicCanvas::GraphicCanvas(QOpenGLWidget *parent) : QOpenGLWidget(parent)
{
    maxAgent = 0;
    agent = NULL;
    numTrafficSignal = 0;
    road  = NULL;
    pathPolygons = NULL;


    QString pathTo = QApplication::applicationDirPath();

    QFile file( CheckNetworkDrive( pathTo + QString("/resim_shader_def.txt") ) );
    if( file.open(QIODevice::ReadOnly | QIODevice::Text) ){
        QTextStream in(&file);

        QString line;
        QStringList divLine;

        line = in.readLine();
        divLine = line.split(";");

        vertexShaderProgramFile = pathTo + QString("/") + QString( divLine[1] ).trimmed();

        qDebug() << "   vertexShaderProgramFile   = " << vertexShaderProgramFile;

        line = in.readLine();
        divLine = line.split(";");

        fragmentShaderProgramFile = pathTo + QString("/") + QString( divLine[1] ).trimmed();

        qDebug() << "   fragmentShaderProgramFile = " << fragmentShaderProgramFile;

        file.close();
    }
    else{
        QMessageBox::warning(this,"Error","Cannot read Shader Setting File: resim_shader_def.txt");
    }

    X_eye = 0.0;
    Y_eye = 0.0;
    Z_eye = -50.0;

    xmin = 0.0;
    xmax = 0.0;
    ymin = 0.0;
    ymax = 0.0;

    X_trans = X_eye;
    Y_trans = Y_eye;
    Z_trans = Z_eye;

    cameraYaw   = 0.0;
    cameraPitch = 0.0;
    cameraQuat = QQuaternion(1.0,0.0,0.0,0.0);

    showVID = true;
    showPathID = false;
    showTSID = false;

    showPathCG = true;
    showTSCG = true;
    showMapImage = true;
    backMapImage = false;

    fontScale = 14;
    sInterfaceObjScale = 0;

    currentWidth = 800;
    currentHeight = 800;

    trackingMode = false;
    trackingObjID = -1;

    last_X_eye = X_eye;
    last_Y_eye = Y_eye;
    last_Z_eye = Z_eye;
}


GraphicCanvas::~GraphicCanvas()
{
    qDebug() << "Clean up";
    makeCurrent();
    program->release();
    delete program;
    doneCurrent();
}


void GraphicCanvas::initializeGL()
{
    qDebug() << "[initializeGL]";

    initializeOpenGLFunctions();

    glClearColor(0.6f, 0.6f, 0.6f, 0.6f);

    bool ret;

    program = new QOpenGLShaderProgram();

    ret = program->addShaderFromSourceFile(QOpenGLShader::Vertex, vertexShaderProgramFile);
    if( !ret ){
        qDebug() << "   addShaderFromSourceFile(Vertex) failed.";
    }

    ret = program->addShaderFromSourceFile(QOpenGLShader::Fragment, fragmentShaderProgramFile);
    if( !ret ){
        qDebug() << "   addShaderFromSourceFile(Fragment) failed.";
    }

    ret = program->link();
    if( !ret ){
        qDebug() << "   program->link failed.";
    }

    ret = program->bind();
    if( !ret ){
        qDebug() << "   program->bind failed.";
    }

    u_modelToWorld = program->uniformLocation("modelToWorld");
    u_worldToView  = program->uniformLocation("worldToView");
    //qDebug() << "u_modelToWorld = " << u_modelToWorld;
    //qDebug() << "u_worldToView  = " << u_worldToView;


    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    // Font
    Characters.clear();

    FT_Library ft;
    if (FT_Init_FreeType(&ft)){
        qDebug() << "ERROR::FREETYPE: Could not init FreeType Library";
    }

    FT_Face face;

    QString pathTo = QApplication::applicationDirPath();
    pathTo += QString("/togoshi-mono.TTF");

    if (FT_New_Face(ft, pathTo.toLocal8Bit().data() , 0, &face)){
        qDebug() << "ERROR::FREETYPE: Failed to load font";
    }

    FT_Set_Pixel_Sizes(face, 24, 24);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    for (GLubyte c = 0; c < 128; c++){
        // Load character glyph
        if (FT_Load_Char(face, c, FT_LOAD_RENDER))
        {
            qDebug() << "ERROR::FREETYTPE: Failed to load Glyph";
            continue;
        }
        // Generate texture
        GLuint texture;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RED,
            face->glyph->bitmap.width,
            face->glyph->bitmap.rows,
            0,
            GL_RED,
            GL_UNSIGNED_BYTE,
            face->glyph->bitmap.buffer
            );
        // Set texture options
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Now store character for later use
        Character *character = new Character;
        character->TextureID = texture;
        character->Size.setWidth(face->glyph->bitmap.width );
        character->Size.setHeight( face->glyph->bitmap.rows );
        character->Bearing.setWidth( face->glyph->bitmap_left );
        character->Bearing.setHeight( face->glyph->bitmap_top );
        character->Advance = face->glyph->advance.x * 1.2;

        Characters.insert(c, character);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    // Destroy FreeType once we're finished
    FT_Done_Face(face);
    FT_Done_FreeType(ft);


    // prepare for Letters
    Character* ch = Characters[ 'A' ];
    GLfloat w = ch->Size.width();
    GLfloat h = ch->Size.height();

    float scale = FONT_SCALE;
    w *= scale;
    h *= scale;

    QVector<GLfloat> fontPoly;
    fontPoly << 0.0 <<  h   << 2.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0;
    fontPoly << 0.0 << 0.0  << 2.0 << 0.0 << 1.0 << 0.0 << 0.0 << 0.0;
    fontPoly <<  w  << 0.0  << 2.0 << 1.0 << 1.0 << 0.0 << 0.0 << 0.0;
    fontPoly <<  w  <<  h   << 2.0 << 1.0 << 0.0 << 0.0 << 0.0 << 0.0;


    // for vehicle ID
    VAOText.create();
    VAOText.bind();

    VBOText = new QOpenGLBuffer();
    VBOText->create();
    VBOText->bind();

    VBOText->setUsagePattern( QOpenGLBuffer::StaticDraw );
    VBOText->allocate( fontPoly.constData(),
                       fontPoly.size() * sizeof(GLfloat) );

    program->enableAttributeArray( 0 );
    program->setAttributeBuffer( 0, GL_FLOAT, 0, 3, 8 * sizeof(GLfloat) );

    program->enableAttributeArray( 1 );
    program->setAttributeBuffer( 1, GL_FLOAT, 3 * sizeof(GLfloat) , 2, 8 * sizeof(GLfloat) );

    program->enableAttributeArray( 2 );
    program->setAttributeBuffer( 2, GL_FLOAT, 5 * sizeof(GLfloat) , 3, 8 * sizeof(GLfloat) );

    VBOText->release();
    VAOText.release();


    // for Path ID
    VAOText_Path.create();
    VAOText_Path.bind();

    VBOText_Path = new QOpenGLBuffer();

    VBOText_Path->create();
    VBOText_Path->bind();

    VBOText_Path->setUsagePattern( QOpenGLBuffer::StaticDraw );
    VBOText_Path->allocate( fontPoly.constData(),
                            fontPoly.size() * sizeof(GLfloat) );

    program->enableAttributeArray( 0 );
    program->setAttributeBuffer( 0, GL_FLOAT, 0, 3, 8 * sizeof(GLfloat) );

    program->enableAttributeArray( 1 );
    program->setAttributeBuffer( 1, GL_FLOAT, 3 * sizeof(GLfloat) , 2, 8 * sizeof(GLfloat) );

    program->enableAttributeArray( 2 );
    program->setAttributeBuffer( 2, GL_FLOAT, 5 * sizeof(GLfloat) , 3, 8 * sizeof(GLfloat) );

    VBOText_Path->release();
    VAOText_Path.release();


    // for TS ID
    VAOText_TS.create();
    VAOText_TS.bind();

    VBOText_TS = new QOpenGLBuffer();

    VBOText_TS->create();
    VBOText_TS->bind();

    VBOText_TS->setUsagePattern( QOpenGLBuffer::StaticDraw );
    VBOText_TS->allocate( fontPoly.constData(),
                          fontPoly.size() * sizeof(GLfloat) );

    program->enableAttributeArray( 0 );
    program->setAttributeBuffer( 0, GL_FLOAT, 0, 3, 8 * sizeof(GLfloat) );

    program->enableAttributeArray( 1 );
    program->setAttributeBuffer( 1, GL_FLOAT, 3 * sizeof(GLfloat) , 2, 8 * sizeof(GLfloat) );

    program->enableAttributeArray( 2 );
    program->setAttributeBuffer( 2, GL_FLOAT, 5 * sizeof(GLfloat) , 3, 8 * sizeof(GLfloat) );

    VBOText_TS->release();
    VAOText_TS.release();


    // Load Optinal Image
    LoadOptionalImage();
}


void GraphicCanvas::paintGL()
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );


    QMatrix4x4 w2c;
    w2c.setToIdentity();

    float xE = X_eye;
    float yE = Y_eye;

    float cpc = cos( cameraPitch );
    float cps = sin( cameraPitch );

    float cyc = cos( cameraYaw );
    float cys = sin( cameraYaw );

    if( trackingMode == true ){

        if( trackingObjID >= 0 && trackingObjID < maxAgent ){

            xE = agent[trackingObjID]->state.x * (-1.0);
            yE = agent[trackingObjID]->state.y * (-1.0);

            cpc = cos( cameraPitch );
            cps = sin( cameraPitch );

            cameraYaw = agent[trackingObjID]->state.yaw * (-1.0) + 3.141592 * 0.5;
            cyc = cos( cameraYaw );
            cys = sin( cameraYaw );

            cameraQuat = QQuaternion(cos(cameraPitch*0.5), sin(cameraPitch*0.5) , 0.0 , 0.0 ) * QQuaternion(cos(cameraYaw*0.5), 0.0 , 0.0 , sin(cameraYaw*0.5));
        }
    }


    X_trans = xE * cyc - yE * cys;
    Y_trans = xE * cys + yE * cyc;

    yE = Y_trans;
    Y_trans = yE * cpc;
    Z_trans = yE * cps;

    Z_trans += Z_eye;

    w2c.translate( QVector3D(X_trans,Y_trans, Z_trans) );

    w2c.rotate( cameraQuat );


    // Base map
    if( rectPoly.isValid == true && showMapImage == true ){

        glLineWidth(1.0);

        for(int i=0;i<baseMapImages.size();++i){

            if( baseMapImages[i]->isValid == false ){
                continue;
            }

            rectPoly.array.bind();

            float mapZ = -1.0;
            if( backMapImage == true ){
                mapZ = -25.0;
            }
            model2World.setTranslation( QVector3D( baseMapImages[i]->x,
                                                   baseMapImages[i]->y,
                                                   mapZ) );

            float angle = baseMapImages[i]->rotate * 0.017452;
            model2World.setRotation( QQuaternion( cos(angle*0.5), 0.0, 0.0, sin(angle*0.5) ) );

            float s = baseMapImages[i]->scale;
            model2World.setScale( QVector3D(s * baseMapImages[i]->halfWidth, s * baseMapImages[i]->halfHeight, 1.0) );

            program->setUniformValue( u_modelToWorld,  w2c * model2World.getWorldMatrix() );

            int useTex  = program->uniformLocation("useTex");
            program->setUniformValue( useTex, 1 );

            int isText  = program->uniformLocation("isText");
            program->setUniformValue( isText, 0 );

            glActiveTexture( GL_TEXTURE0 );

            glBindTexture(GL_TEXTURE_2D, baseMapImages[i]->textureID);

            glDrawArrays(GL_QUADS, 0, 4 * sizeof(GLfloat) );

            rectPoly.array.release();
        }
    }



    // Path
    if( pathPolygons && showPathCG ){

        program->setUniformValue( u_worldToView, projection );

        pathPolygons->pathPolygonsArray.bind();

        program->setUniformValue( u_modelToWorld,  w2c );

        int colorPos  = program->uniformLocation("vColor");
        program->setUniformValue( colorPos, QVector4D(0.7, 0.2, 0.3, 1.0) );

        int useTex  = program->uniformLocation("useTex");
        program->setUniformValue( useTex, 0 );

        int isText  = program->uniformLocation("isText");
        program->setUniformValue( isText, 0 );

        int offsetPos = program->uniformLocation("offsetPos");
        program->setUniformValue( offsetPos, QVector3D(0.0,0.0,0.0) );

        glDrawArrays(GL_QUADS, 0, pathPolygons->pathPolygonData.size() / 8 );

        pathPolygons->pathPolygonsArray.release();
    }


    int nAppearAgent = 0;

    for(int i=0;i<maxAgent;++i){

        if( agent[i]->agentStatus == 0 ){
            continue;
        }

        nAppearAgent++;

        if( agent[i]->agentKind < 100 ){

            int vehicleShapeID = agent[i]->vehicle.GetVehicleModelID();
            int brakeLame      = agent[i]->vehicle.GetBrakeLampState();
            int winkerState    = agent[i]->vehicle.GetWinkerIsBlink();

//

            //
            //
            //
            program->setUniformValue( u_worldToView, projection );


            if( vehicleModels[vehicleShapeID]->objFileDataSet == true ){


            }
            else{

//                qDebug() << "agent[" << i << "] vehicleShapeID = " << vehicleShapeID;

                vehicleModels[vehicleShapeID]->simplePoly.vehiclePolygonArray.bind();

                model2World.setTranslation(QVector3D( agent[i]->state.x ,
                                                      agent[i]->state.y ,
                                                      agent[i]->state.z_path  ));

                float half_psi = agent[i]->state.yaw * 0.5;
                model2World.setRotation( QQuaternion( cos(half_psi), 0.0, 0.0, sin(half_psi) ) );

                if( agent[i]->isSInterfaceObject == true ){
                    model2World.setScale( QVector3D( 1.0, 1.0, 1.0 ) * (1.0 + sInterfaceObjScale * 0.05) );
                }
                else{
                    model2World.setScale( QVector3D( 1.0, 1.0, 1.0 ) );
                }

                program->setUniformValue( u_modelToWorld,  w2c * model2World.getWorldMatrix() );


                int colorPos  = program->uniformLocation("vColor");
                if( agent[i]->isSInterfaceObject == true ){
                    program->setUniformValue( colorPos, QVector4D(0.998, 0.1, 0.05, 1.0) );
                }
                else if( agent[i]->isScenarioObject == true ){
                    program->setUniformValue( colorPos, QVector4D(0.12, 0.7, 0.7, 1.0) );
                }
                else{
                    program->setUniformValue( colorPos, QVector4D(0.2, 0.2, 0.7, 1.0) );
                }

                int useTex  = program->uniformLocation("useTex");

                int lampCom = 10;
                if( brakeLame == 1 && winkerState == 0 ){
                    lampCom = 2;
                }
                else if( brakeLame == 0 && winkerState == 1 ){
                    lampCom = 3;
                }
                else if( brakeLame == 0 && winkerState == 2 ){
                    lampCom = 4;
                }
                else if( brakeLame == 1 && winkerState == 1 ){
                    lampCom = 5;
                }
                else if( brakeLame == 1 && winkerState == 2 ){
                    lampCom = 6;
                }

                program->setUniformValue( useTex, lampCom );    // set 2 for brake lamp
                                                                //     3 for left winker
                                                                //     4 for right winker
                                                                //     5 for brake lamp + left winker
                                                                //     6 for brake lamp + right winker
                                                                //     10 for drawing outline
                                                                // set 12 for brake lamp with texture
                                                                //     13 for left winker with texture
                                                                //     14 for right winker with texture
                                                                //     15 for brake lamp + left winker with texture
                                                                //     16 for brake lamp + right winker with texture


                int isText  = program->uniformLocation("isText");
                program->setUniformValue( isText, 0 );


                int offsetPos = program->uniformLocation("offsetPos");
                program->setUniformValue( offsetPos, QVector3D(vehicleModels[vehicleShapeID]->distCG2RE * (-0.9),  // brake
                                                               vehicleModels[vehicleShapeID]->distCG2FE * (0.9),   // winker
                                                               0.4) );                                             // winker

                glDrawArrays(GL_QUADS, 0, vehicleModels[vehicleShapeID]->simplePoly.vehiclePolygon.size() / 8 );

                program->setUniformValue( useTex, 10 );
                program->setUniformValue( colorPos, QVector4D(1.0, 1.0, 1.0, 1.0) );

                glLineWidth(1);
                glDrawArrays(GL_LINE_STRIP, 0, vehicleModels[vehicleShapeID]->simplePoly.vehiclePolygon.size() / 8 );

                vehicleModels[vehicleShapeID]->simplePoly.vehiclePolygonArray.release();
            }
        }
        else if( agent[i]->agentKind >= 100 ){

            int personShapeID = agent[i]->vehicle.GetVehicleModelID();

            program->setUniformValue( u_worldToView, projection );

            personModels[personShapeID]->personPolygonArray.bind();

            model2World.setTranslation(QVector3D( agent[i]->state.x ,
                                                  agent[i]->state.y ,
                                                  agent[i]->state.z  ));

            float half_psi = agent[i]->state.yaw * 0.5;
            model2World.setRotation( QQuaternion( cos(half_psi), 0.0, 0.0, sin(half_psi) ) );

            if( agent[i]->isSInterfaceObject == true ){

                float w = personModels[personShapeID]->width;
                float f = 0.5 / w;

                model2World.setScale( QVector3D( 1.0, 1.0, 1.0 ) * (1.0 + sInterfaceObjScale * 0.05) * f );
            }
            else{
                model2World.setScale( QVector3D( 1.0, 1.0, 1.0 ) );
            }

            program->setUniformValue( u_modelToWorld,  w2c * model2World.getWorldMatrix() );


            int colorPos  = program->uniformLocation("vColor");
            if( agent[i]->isSInterfaceObject == true ){
                program->setUniformValue( colorPos, QVector4D(0.998, 0.1, 0.05, 1.0) );
            }
            else if( agent[i]->isScenarioObject == true ){
                program->setUniformValue( colorPos, QVector4D(0.12, 0.7, 0.7, 1.0) );
            }
            else{
                program->setUniformValue( colorPos, QVector4D(0.2, 0.2, 0.7, 1.0) );
            }


            int useTex  = program->uniformLocation("useTex");
            program->setUniformValue( useTex, 10 );

            int isText  = program->uniformLocation("isText");
            program->setUniformValue( isText, 0 );

            int offsetPos = program->uniformLocation("offsetPos");
            program->setUniformValue( offsetPos, QVector3D(0.0,0.0,0.0) );

            glDrawArrays(GL_QUADS, 0, personModels[personShapeID]->personPolygon.size() / 8 );

            program->setUniformValue( useTex, 10 );
            program->setUniformValue( colorPos, QVector4D(1.0, 1.0, 1.0, 1.0) );

            glLineWidth(1);
            glDrawArrays(GL_LINE_STRIP, 0, personModels[personShapeID]->personPolygon.size() / 8 );

            personModels[personShapeID]->personPolygonArray.release();
        }

    }


    if( showTSCG ){

    //qDebug() << "trafficSignal.size = " << trafficSignal.size();

        for(int i=0;i<trafficSignal.size();++i){

            program->setUniformValue( u_worldToView, projection );

            TSPolygons[i]->TSPolygonsArray.bind();

            model2World.setTranslation(QVector3D( trafficSignal[i]->xTS ,
                                                  trafficSignal[i]->yTS ,
                                                  trafficSignal[i]->zTS + 5.0  ));

            float half_psi = (trafficSignal[i]->direction + 3.141592 * 0.5) * 0.5;
            model2World.setRotation( QQuaternion( cos(half_psi), 0.0, 0.0, sin(half_psi) ) );

            model2World.setScale( QVector3D(1.0, 1.0, 1.0) );

            program->setUniformValue( u_modelToWorld,  w2c * model2World.getWorldMatrix() );


            int useTex  = program->uniformLocation("useTex");
            int currentDisplay = trafficSignal[i]->GetCurrentDisplayInfo();
    //        qDebug() << "currentDisplay = " << currentDisplay;

            if( currentDisplay == 1 ){
                program->setUniformValue( useTex, 101 );
            }
            else if( currentDisplay == 2 ){
                program->setUniformValue( useTex, 102 );
            }
            else if( currentDisplay == 4 ){
                program->setUniformValue( useTex, 103 );
            }
            else if( currentDisplay == 12 ){
                program->setUniformValue( useTex, 104 );
            }
            else if( currentDisplay == 20 ){
                program->setUniformValue( useTex, 105 );
            }
            else if( currentDisplay == 28 ){
                program->setUniformValue( useTex, 106 );
            }
            else if( currentDisplay == 36 ){
                program->setUniformValue( useTex, 107 );
            }
            else if( currentDisplay == 52 ){
                program->setUniformValue( useTex, 108 );
            }
            else if( currentDisplay == 64 ){
                program->setUniformValue( useTex, 102 );
            }
            else if( currentDisplay == 128 ){
                program->setUniformValue( useTex, 103 );
            }
            else if( currentDisplay == 255 ){
                program->setUniformValue( useTex, 109 );
            }
            else{
                program->setUniformValue( useTex, 109 );
            }

            int isText  = program->uniformLocation("isText");
            program->setUniformValue( isText, 0 );

            int offsetPos = program->uniformLocation("offsetPos");
            program->setUniformValue( offsetPos, QVector3D(0.0,0.0,0.0) );

            int colorPos  = program->uniformLocation("vColor");
            program->setUniformValue( colorPos, QVector4D(0.2, 0.2, 0.7, 1.0) );

            glDrawArrays(GL_QUADS, 0, TSPolygons[i]->TSPolygons.size() / 8 );

            TSPolygons[i]->TSPolygonsArray.release();
        }
    }



    // Fonts
    if( showPathCG == true && showPathID == true && road && road->paths.size() > 0 ){
        // road is added in the if condition because the condition would be evaluated before road is allocated.

        VAOText_Path.bind();

        int colorPos  = program->uniformLocation("vColor");
        program->setUniformValue( colorPos, QVector4D(1.0, 1.0, 1.0, 1.0) );

        int useTex  = program->uniformLocation("useTex");
        program->setUniformValue( useTex, 100 );

        int isText  = program->uniformLocation("isText");
        program->setUniformValue( isText, 100 );

        int letterPos  = program->uniformLocation("letterPos");

        for(int i=0;i<road->paths.size();++i){

            int idx = road->paths[i]->pos.size() / 2;
            model2World.setTranslation(QVector3D( road->paths[i]->pos[idx]->x() ,
                                                  road->paths[i]->pos[idx]->y() ,
                                                  road->paths[i]->pos[idx]->z() + 2.0  ));

            QQuaternion letterQuat = cameraQuat.conjugated();
            model2World.setRotation( letterQuat );

            program->setUniformValue( u_modelToWorld,  w2c * model2World.getWorldMatrix() );

            glActiveTexture( GL_TEXTURE0 );

            float x = 0.0;
            float y = 0.0;
            float scale = FONT_SCALE;

            char str[25];
            sprintf(str,"Path%d",road->paths[i]->id);

            for(unsigned int c=0;c<strlen(str);++c ){

                Character* ch = Characters[ str[c] ];
                GLfloat xpos = x + ch->Bearing.width() * scale;
                GLfloat ypos = y + (Characters['H']->Bearing.height() - ch->Bearing.height()) * scale;
                program->setUniformValue( letterPos, QVector3D(xpos, ypos, 0.0) );

                glBindTexture( GL_TEXTURE_2D, ch->TextureID );

                glDrawArrays(GL_QUADS, 0, 4 );

                x += ( ch->Advance >> 6 ) * scale;
            }
        }

        VAOText_Path.release();
    }

    if( showTSCG == true && showTSID == true && trafficSignal.size() > 0 ){

        VAOText_TS.bind();

        int colorPos  = program->uniformLocation("vColor");
        program->setUniformValue( colorPos, QVector4D(1.0, 1.0, 1.0, 1.0) );

        int useTex  = program->uniformLocation("useTex");
        program->setUniformValue( useTex, 100 );

        int isText  = program->uniformLocation("isText");
        program->setUniformValue( isText, 100 );

        int letterPos  = program->uniformLocation("letterPos");

        for(int i=0;i<trafficSignal.size();++i){

            model2World.setTranslation(QVector3D( trafficSignal[i]->xTS ,
                                                  trafficSignal[i]->yTS ,
                                                  trafficSignal[i]->zTS + 2.0  ));

            QQuaternion letterQuat = cameraQuat.conjugated();
            model2World.setRotation( letterQuat );

            program->setUniformValue( u_modelToWorld,  w2c * model2World.getWorldMatrix() );

            glActiveTexture( GL_TEXTURE0 );

            float x = 0.0;
            float y = 0.0;
            float scale = FONT_SCALE;

            char str[25];
            sprintf(str,"TS%d", trafficSignal[i]->id);

            for(unsigned int c=0;c<strlen(str);++c ){

                Character* ch = Characters[ str[c] ];
                GLfloat xpos = x + ch->Bearing.width() * scale;
                GLfloat ypos = y + (Characters['H']->Bearing.height() - ch->Bearing.height()) * scale;
                program->setUniformValue( letterPos, QVector3D(xpos, ypos, 0.0) );

                glBindTexture( GL_TEXTURE_2D, ch->TextureID );

                glDrawArrays(GL_QUADS, 0, 4 );

                x += ( ch->Advance >> 6 ) * scale;
            }
        }

        VAOText_TS.release();
    }

    if( showVID == true && nAppearAgent > 0 ){

        VAOText.bind();

        int colorPos  = program->uniformLocation("vColor");

        int useTex  = program->uniformLocation("useTex");
        program->setUniformValue( useTex, 100 );

        int isText  = program->uniformLocation("isText");
        program->setUniformValue( isText, 100 );

        int letterPos  = program->uniformLocation("letterPos");

        for(int i=0;i<maxAgent;++i){

            if( agent[i]->agentStatus == 0 ){
                continue;
            }


            model2World.setTranslation(QVector3D( agent[i]->state.x ,
                                                  agent[i]->state.y ,
                                                  agent[i]->state.z + 2.0  ));

            QQuaternion letterQuat = cameraQuat.conjugated();
            model2World.setRotation( letterQuat );

            model2World.setScale( QVector3D(fontScale * 0.1, fontScale* 0.1, fontScale* 0.1) );

            program->setUniformValue( u_modelToWorld,  w2c * model2World.getWorldMatrix() );

            glActiveTexture( GL_TEXTURE0 );

            float x = 0.0;
            float y = 0.0;
            float scale = FONT_SCALE;

            char str[25];
            if( agent[i]->agentKind < 100 ){
                if( agent[i]->isSInterfaceObject == true ){
                    sprintf(str,"[DS]V%d",agent[i]->ID);
                    program->setUniformValue( colorPos, QVector4D(0.998, 0.1, 0.05, 1.0) );
                }
                else if( agent[i]->isScenarioObject == true ){
                    sprintf(str,"[snr]V%d",agent[i]->ID);
                    program->setUniformValue( colorPos, QVector4D(0.067, 1.0, 0.067, 1.0) );
                }
                else{
                    sprintf(str,"V%d",agent[i]->ID);
                    program->setUniformValue( colorPos, QVector4D(0.067, 1.0, 0.067, 1.0) );
                }
            }
            else if( agent[i]->agentKind >= 100 ){
                if( agent[i]->isSInterfaceObject == true ){
                    sprintf(str,"[PS]P%d",agent[i]->ID);
                    program->setUniformValue( colorPos, QVector4D(0.998, 0.1, 0.05, 1.0) );
                }
                else if( agent[i]->isScenarioObject == true ){
                    sprintf(str,"[snr]P%d",agent[i]->ID);
                    program->setUniformValue( colorPos, QVector4D(0.998, 0.78, 0.00, 1.0) );
                }
                else{
                    sprintf(str,"P%d",agent[i]->ID);
                    program->setUniformValue( colorPos, QVector4D(0.998, 0.78, 0.00, 1.0) );
                }
            }

            for(unsigned int c=0;c<strlen(str);++c ){

                Character* ch = Characters[ str[c] ];

                GLfloat xpos = x + ch->Bearing.width() * scale;
                GLfloat ypos = y - Characters['H']->Bearing.height() * scale;
                program->setUniformValue( letterPos, QVector3D(xpos, ypos, 0.0) );

                glBindTexture( GL_TEXTURE_2D, ch->TextureID );

                glDrawArrays(GL_QUADS, 0, 4 );

                x += ( ch->Advance >> 6 ) * scale;
            }
        }

        VAOText.release();
    }

    model2World.setScale( QVector3D(1.0, 1.0, 1.0) );


    if( rectPoly.isValid == true && optionalImages.size() > 0 ){

        glLineWidth(1.0);

//        qDebug() << "Draw OptinalImage";

        for(int i=0;i<optionalImages.size();++i){

            if( optionalImages[i]->isValid == false || optionalImages[i]->showImage == false ){
                continue;
            }

            rectPoly.array.bind();
            model2World.setTranslation( QVector3D( optionalImages[i]->x,
                                                   optionalImages[i]->y,
                                                   optionalImages[i]->z) );

            qDebug() << "x = " << optionalImages[i]->x << " y = " << optionalImages[i]->y << " z = " << optionalImages[i]->z
                     << " rot = " << optionalImages[i]->rotate << " s = " << optionalImages[i]->scale;


            float angle = optionalImages[i]->rotate * 0.017452;
            model2World.setRotation( QQuaternion( cos(angle*0.5), 0.0, 0.0, sin(angle*0.5) ) );

            float s = optionalImages[i]->scale;
            model2World.setScale( QVector3D(s * optionalImages[i]->halfWidth, s * optionalImages[i]->halfHeight, 1.0) );

            program->setUniformValue( u_modelToWorld,  w2c * model2World.getWorldMatrix() );

            int useTex  = program->uniformLocation("useTex");
            program->setUniformValue( useTex, 1 );

            int isText  = program->uniformLocation("isText");
            program->setUniformValue( isText, 0 );

            glActiveTexture( GL_TEXTURE0 );

            glBindTexture(GL_TEXTURE_2D, optionalImages[i]->textureID);

            glDrawArrays(GL_QUADS, 0, 4 * sizeof(GLfloat) );

            rectPoly.array.release();
        }
    }
}


void GraphicCanvas::resizeGL(int w, int h)
{
//    qDebug() << "[resizeGL] w = " << w << " h = " << h;

    projection.setToIdentity();
    projection.perspective( 45.0, float(w) / float(h), 0.1, 100000.0 );

    sx = 1.0 / float(w);
    sy = 1.0 / float(h);

    currentWidth = w;
    currentHeight = h;
}


void GraphicCanvas::mousePressEvent(QMouseEvent *e)
{
    mousePressPosition = QVector2D(e->localPos());

    if( e->modifiers() & Qt::ControlModifier || e->modifiers() & Qt::AltModifier ){
        float x,y;
        int ret = Get3DPhysCoordFromPickPoint( e->x(), e->y(), x, y );
        if( ret == 1 ){
            if( e->modifiers() & Qt::ControlModifier ){
                emit ShowAgentData(x,y);
            }
            else if( e->modifiers() & Qt::AltModifier ){
                qDebug() << "Emit DSMove";
                emit DSMove(x,y);
            }

        }
//        qDebug() << "ret = " << ret << " x = " << x << " y = " << y;
    }
}


void GraphicCanvas::mouseMoveEvent(QMouseEvent *e)
{

    QVector2D diff = QVector2D(e->localPos()) - mousePressPosition;

    if( e->buttons() & Qt::RightButton ){

        diff.setX( diff.x() * sx );
        diff.setY( diff.y() * sy );
        float a = diff.length();
        if( a > 0.0 ){

            cameraYaw += diff.x() * 0.5;
            cameraPitch += diff.y() * 0.5;

            cameraQuat = QQuaternion(cos(cameraPitch*0.5), sin(cameraPitch*0.5) , 0.0 , 0.0 ) * QQuaternion(cos(cameraYaw*0.5), 0.0 , 0.0 , sin(cameraYaw*0.5));
        }
    }
    else if( e->buttons() & Qt::LeftButton ){

        float s = Z_eye / 50.0 * (-0.072);
        if( e->modifiers() & Qt::ControlModifier ){
            s *= 5.0;
        }

        float xMove = diff.x() * s;
        float yMove = diff.y() * (-s);

        float cyc = cos( cameraYaw );
        float cys = sin( cameraYaw );

        float xMm = xMove * cyc + cys * yMove;
        float yMm = -xMove * cys + cyc * yMove;

        X_eye += xMm;
        Y_eye += yMm;
    }

    QOpenGLWidget::update();
    mousePressPosition = QVector2D(e->localPos());
}


void GraphicCanvas::wheelEvent(QWheelEvent *e)
{
    float s = 1.0;
    if( e->modifiers() & Qt::ControlModifier ){
        s *= 2.2;
    }
    if( e->delta() > 0.0 ){
        if( Z_eye < -0.2 ){
            Z_eye /= (1.05 * s);
        }
    }
    else if( e->delta() < 0.0 ){
        Z_eye *= 1.05 * s;
    }

    int fs = (int)(-0.0838 * Z_eye) + 10;
    emit ChangeFontScale(fs);

    QOpenGLWidget::update();

//    qDebug() << "X=" << X_eye << " Y=" << Y_eye << " Z=" << Z_eye;
}


void GraphicCanvas::SetVehiclePolygon(int index,float lf,float lr,float width,float height)
{
    if( index < 0 || index >= vehicleModels.size() ){
        return;
    }

    makeCurrent();

    vehicleModels[index]->simplePoly.isAllSet = false;

    bool ret;
    ret = vehicleModels[index]->simplePoly.vehiclePolygonArray.create();
    if( !ret ){
        qDebug() << "vehicleModels[" << index << "]->simplePoly.vehiclePolygonArray.craete failed.";
        doneCurrent();
        return;
    }

    vehicleModels[index]->simplePoly.vehiclePolygonArray.bind();

    vehicleModels[index]->simplePoly.vehiclePolygonBuffer = new QOpenGLBuffer();


    ret = vehicleModels[index]->simplePoly.vehiclePolygonBuffer->create();
    if( !ret ){
        qDebug() << "vehicleModels[" << index << "]->simplePoly.vehiclePolygonBuffer.craete() failed.";
        doneCurrent();
        return;
    }

    ret = vehicleModels[index]->simplePoly.vehiclePolygonBuffer->bind();
    if( !ret ){
        qDebug() << "vehicleModels[" << index << "]->simplePoly.vehiclePolygonBuffer.bind() failed.";
        doneCurrent();
        return;
    }


    float x[8];
    float y[8];
    float z[8];

    float w = width * 0.5;

    x[0] =  lf;         y[0] =  w * 0.9;  z[0] = 0.0;
    x[1] = -lr;         y[1] =  w;        z[1] = 0.0;
    x[2] = -lr * 0.9;   y[2] =  w * 0.7;  z[2] = height;
    x[3] =  lf * 0.8;   y[3] =  w * 0.5;  z[3] = height * 2.0 / 3.0;
    x[4] =  lf;         y[4] = -w * 0.9;  z[4] = 0.0;
    x[5] = -lr;         y[5] = -w;        z[5] = 0.0;
    x[6] = -lr * 0.9;   y[6] = -w * 0.7;  z[6] = height;
    x[7] =  lf * 0.8;   y[7] = -w * 0.5;  z[7] = height * 2.0 / 3.0;


    //left
    vehicleModels[index]->simplePoly.vehiclePolygon << x[3] << y[3] << z[3] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[0] << y[0] << z[0] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[1] << y[1] << z[1] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[2] << y[2] << z[2] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;


    // top
    vehicleModels[index]->simplePoly.vehiclePolygon << x[6] << y[6] << z[6] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[7] << y[7] << z[7] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[3] << y[3] << z[3] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[2] << y[2] << z[2] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;


    //right
    vehicleModels[index]->simplePoly.vehiclePolygon << x[6] << y[6] << z[6] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[5] << y[5] << z[5] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[4] << y[4] << z[4] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[7] << y[7] << z[7] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;


    // front
    vehicleModels[index]->simplePoly.vehiclePolygon << x[3] << y[3] << z[3] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[7] << y[7] << z[7] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[4] << y[4] << z[4] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[0] << y[0] << z[0] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;


    // back
    vehicleModels[index]->simplePoly.vehiclePolygon << x[1] << y[1] << z[1] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[5] << y[5] << z[5] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[6] << y[6] << z[6] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    vehicleModels[index]->simplePoly.vehiclePolygon << x[2] << y[2] << z[2] << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;


    vehicleModels[index]->simplePoly.vehiclePolygonBuffer->setUsagePattern( QOpenGLBuffer::StaticDraw );

    vehicleModels[index]->simplePoly.vehiclePolygonBuffer->allocate( vehicleModels[index]->simplePoly.vehiclePolygon.constData(),
                                                                     vehicleModels[index]->simplePoly.vehiclePolygon.size() * sizeof(GLfloat) );


    program->enableAttributeArray( 0 );
    program->setAttributeBuffer( 0, GL_FLOAT, 0, 3, 8 * sizeof(GLfloat) );


    program->enableAttributeArray( 1 );
    program->setAttributeBuffer( 1, GL_FLOAT, 3 * sizeof(GLfloat) , 2, 8 * sizeof(GLfloat) );


    program->enableAttributeArray( 2 );
    program->setAttributeBuffer( 2, GL_FLOAT, 5 * sizeof(GLfloat) , 3, 8 * sizeof(GLfloat) );


    vehicleModels[index]->simplePoly.vehiclePolygonBuffer->release();
    vehicleModels[index]->simplePoly.vehiclePolygonArray.release();

    vehicleModels[index]->simplePoly.isAllSet = true;

    doneCurrent();

    qDebug() << "[SetVehiclePolygon] index=" << index << " lf=" << lf << " lr=" << lr << " w=" << w << " h=" << height;
}


void GraphicCanvas::SetPersonPolygon(int index,float width,float height,float depth)
{
    if( index < 0 || index >= personModels.size() ){
        return;
    }

    personModels[index]->width = width;

    makeCurrent();

    bool ret;
    ret = personModels[index]->personPolygonArray.create();
    if( !ret ){
        qDebug() << "personModels[" << index << "]->personPolygonArray.craete failed.";
        doneCurrent();
        return;
    }

    personModels[index]->personPolygonArray.bind();

    personModels[index]->personPolygonBuffer = new QOpenGLBuffer();

    ret = personModels[index]->personPolygonBuffer->create();
    if( !ret ){
        qDebug() << "personModels[" << index << "]->personPolygonBuffer.craete() failed.";
        doneCurrent();
        return;
    }

    ret = personModels[index]->personPolygonBuffer->bind();
    if( !ret ){
        qDebug() << "personModels[" << index << "]->personPolygonBuffer.bind() failed.";
        doneCurrent();
        return;
    }

    float hd = depth * 0.5;
    float hw = width * 0.5;
    float hh = height;


    //left
    personModels[index]->personPolygon <<  hd << hw <<  hh << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon <<  hd << hw << 0.0 << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon << -hd << hw << 0.0 << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon << -hd << hw <<  hh << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;

    // top
    personModels[index]->personPolygon <<  hd <<  hw << hh << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon << -hd <<  hw << hh << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon << -hd << -hw << hh << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon <<  hd << -hw << hh << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;

    //right
    personModels[index]->personPolygon << -hd << -hw <<  hh << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon << -hd << -hw << 0.0 << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon <<  hd << -hw << 0.0 << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon <<  hd << -hw <<  hh << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;

    // front
    personModels[index]->personPolygon << hd << -hw <<  hh << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon << hd << -hw << 0.0 << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon << hd <<  hw << 0.0 << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon << hd <<  hw <<  hh << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;

    // back
    personModels[index]->personPolygon << -hd <<  hw <<  hh << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon << -hd <<  hw << 0.0 << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon << -hd << -hw << 0.0 << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;
    personModels[index]->personPolygon << -hd << -hw <<  hh << 0.0f << 0.0f << 0.2f << 0.2f << 0.7f;

    personModels[index]->personPolygonBuffer->setUsagePattern( QOpenGLBuffer::StaticDraw );

    personModels[index]->personPolygonBuffer->allocate( personModels[index]->personPolygon.constData(),
                                                        personModels[index]->personPolygon.size() * sizeof(GLfloat) );


    program->enableAttributeArray( 0 );
    program->setAttributeBuffer( 0, GL_FLOAT, 0, 3, 8 * sizeof(GLfloat) );


    program->enableAttributeArray( 1 );
    program->setAttributeBuffer( 1, GL_FLOAT, 3 * sizeof(GLfloat) , 2, 8 * sizeof(GLfloat) );


    program->enableAttributeArray( 2 );
    program->setAttributeBuffer( 2, GL_FLOAT, 5 * sizeof(GLfloat) , 3, 8 * sizeof(GLfloat) );


    personModels[index]->personPolygonBuffer->release();
    personModels[index]->personPolygonArray.release();

    doneCurrent();
}


void GraphicCanvas::SetTSData()
{
    if( trafficSignal.size() == 0 ){
        return;
    }

    qDebug() << "[GraphicCanvas::SetTSData]";
    qDebug() << "number of Traffic Signal : " << trafficSignal.size();

    makeCurrent();

    if( TSPolygons.size() > 0 ){
        for(int i=0;i<trafficSignal.size();++i){

            TSPolygons[i]->TSPolygons.clear();
            TSPolygons[i]->TSPolygonsBuffer->release();
            TSPolygons[i]->TSPolygonsBuffer->destroy();
            TSPolygons[i]->TSPolygonsArray.release();

            delete TSPolygons[i];
        }
        TSPolygons.clear();
    }

    for(int i=0;i<trafficSignal.size();++i){

        struct TrafficSignalPolygon *tsp = new struct TrafficSignalPolygon;

        bool ret;
        ret = tsp->TSPolygonsArray.create();
        if( !ret ){
            qDebug() << "TrafficSignalPolygon->TSPolygonsArray.craete() failed.";
            doneCurrent();
            return;
        }
        //qDebug() << "TSPolygonsArray created.";

        tsp->TSPolygonsArray.bind();
        //qDebug() << "TSPolygonsArray bind.";

        tsp->TSPolygonsBuffer = new QOpenGLBuffer();
        //qDebug() << "TSPolygonsBuffer allocated.";

        ret = tsp->TSPolygonsBuffer->create();
        if( !ret ){
            qDebug() << "TrafficSignalPolygon->TSPolygonsBuffer->craete() failed.";
            doneCurrent();
            return;
        }
        //qDebug() << "TSPolygonsBuffer created.";

        ret = tsp->TSPolygonsBuffer->bind();
        if( !ret ){
            qDebug() << "TrafficSignalPolygon->TSPolygonsBuffer->bind() failed.";
            doneCurrent();
            return;
        }
        //qDebug() << "TSPolygonsBuffer binded.";
        //qDebug() << "trafficSignal type = " << trafficSignal[i]->type;

        if( trafficSignal[i]->type == 'v' ){

            int nVertex = 44 * 4;
            tsp->TSPolygons.reserve( nVertex * 8 );

            float R = 0.5;
            float hs = 4.0 * R;
            float ws = 3.0 * R;

            // plate 1
            tsp->TSPolygons << -ws << hs << 0.0 << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;
            tsp->TSPolygons << -ws << 0.0 << 0.0 << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;
            tsp->TSPolygons <<  ws << 0.0 << 0.0 << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;
            tsp->TSPolygons <<  ws << hs << 0.0 << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;

            // plate 2
            tsp->TSPolygons << -ws << 0.0 << 0.0 << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;
            tsp->TSPolygons << -ws << 0.0 << -hs << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;
            tsp->TSPolygons <<  ws << 0.0 << -hs << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;
            tsp->TSPolygons <<  ws << 0.0 << 0.0 << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;

            // blue
            float xcb = (-2.0) / 3.0 * ws;
            float ycb = (0.75) * hs;
            for(int j=0;j<4;++j){
                float angb = 90.0 * j * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << ycb + R * cos(angb) << 0.05 << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
                angb += 45 * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << ycb + R * cos(angb) << 0.05 << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
                angb += 45 * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << ycb + R * cos(angb) << 0.05 << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
                tsp->TSPolygons << xcb                 << ycb                 << 0.05 << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
            }

            // Yellow
            xcb = 0.0;
            for(int j=0;j<4;++j){
                float angb = 90.0 * j * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << ycb + R * cos(angb) << 0.05 << 0.2f << 0.2f << 1.0f << 0.643f << 0.0f;
                angb += 45 * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << ycb + R * cos(angb) << 0.05 << 0.2f << 0.2f << 1.0f << 0.643f << 0.0f;
                angb += 45 * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << ycb + R * cos(angb) << 0.05 << 0.2f << 0.2f << 1.0f << 0.643f << 0.0f;
                tsp->TSPolygons << xcb                 << ycb                 << 0.05 << 0.2f << 0.2f << 1.0f << 0.643f << 0.0f;
            }

            // Red
            xcb = 2.0 / 3.0 * ws;
            for(int j=0;j<4;++j){
                float angb = 90.0 * j * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << ycb + R * cos(angb) << 0.05 << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
                angb += 45 * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << ycb + R * cos(angb) << 0.05 << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
                angb += 45 * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << ycb + R * cos(angb) << 0.05 << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
                tsp->TSPolygons << xcb                 << ycb                 << 0.05 << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
            }

            // Left Arrow
            float xe1 = ws * (-1.0);
            float xe2 = xe1 + 2.0 * R;
            float yt = 0.25 * hs + R * 0.2;
            float yb = 0.25 * hs - R * 0.2;
            tsp->TSPolygons << xe1 << yt << 0.05 << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 << yb << 0.05 << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << yb << 0.05 << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << yt << 0.05 << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;

            xe2 = xe1 + R / 2.0;
            yb = yt;
            yt = 0.5 * hs;
            tsp->TSPolygons << xe2           << yt << 0.05 << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1           << yb << 0.05 << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 + R * 0.2 << yb << 0.05 << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 + R * 0.2 << yt << 0.05 << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;

            yb = 0;
            yt = 0.25 * hs - R * 0.2;
            tsp->TSPolygons << xe1           << yt << 0.05 << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2           << yb << 0.05 << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 + R * 0.2 << yb << 0.05 << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 + R * 0.2 << yt << 0.05 << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;

            // Straight Arrow
            xe1 = R * (-0.2);
            xe2 = R * 0.2;
            yt = 0.5 * hs;
            yb = 0.0;
            tsp->TSPolygons << xe1 << yt << 0.05 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 << yb << 0.05 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << yb << 0.05 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << yt << 0.05 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;

            xe2 = xe1;
            xe1 = R * (-1.0);
            yb = 0.25 * hs;
            tsp->TSPolygons << xe1 << yb           << 0.05 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 << yb - R * 0.2 << 0.05 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << yt - R * 0.2 << 0.05 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << yt           << 0.05 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;

            xe1 = R * 0.2;
            xe2 = R;
            tsp->TSPolygons << xe1 << yt           << 0.05 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 << yt - R * 0.2 << 0.05 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << yb - R * 0.2 << 0.05 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << yb           << 0.05 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;

            // Right Arrow
            xe2 = ws * (1.0);
            xe1 = xe2 - 2.0 * R;
            yt = 0.25 * hs + R * 0.2;
            yb = 0.25 * hs - R * 0.2;
            tsp->TSPolygons << xe1 << yt << 0.05 << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 << yb << 0.05 << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << yb << 0.05 << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << yt << 0.05 << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;

            xe1 = xe2 - R / 2.0;
            yb = yt;
            yt = 0.5 * hs;
            tsp->TSPolygons << xe1           << yt << 0.05 << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 - R * 0.2 << yt << 0.05 << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 - R * 0.2 << yb << 0.05 << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2           << yb << 0.05 << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;

            yb = 0;
            yt = 0.25 * hs - R * 0.2;
            tsp->TSPolygons << xe1           << yb << 0.05 << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2           << yt << 0.05 << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 - R * 0.2 << yt << 0.05 << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 - R * 0.2 << yb << 0.05 << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;


            // blue
            xcb = (-2.0) / 3.0 * ws;
            float zcb = (-0.25) * hs;
            for(int j=0;j<4;++j){
                float angb = 90.0 * j * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << -0.05 << zcb + R * cos(angb) << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
                angb += 45 * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << -0.05 << zcb + R * cos(angb) << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
                angb += 45 * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << -0.05 << zcb + R * cos(angb) << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
                tsp->TSPolygons << xcb                 << -0.05 << zcb                 << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
            }

            // Yellow
            xcb = 0.0;
            for(int j=0;j<4;++j){
                float angb = 90.0 * j * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << -0.05 << zcb + R * cos(angb) << 0.2f << 0.2f << 1.0f << 0.643f << 0.0f;
                angb += 45 * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << -0.05 << zcb + R * cos(angb) << 0.2f << 0.2f << 1.0f << 0.643f << 0.0f;
                angb += 45 * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << -0.05 << zcb + R * cos(angb) << 0.2f << 0.2f << 1.0f << 0.643f << 0.0f;
                tsp->TSPolygons << xcb                 << -0.05 << zcb                 << 0.2f << 0.2f << 1.0f << 0.643f << 0.0f;
            }

            // Red
            xcb = 2.0 / 3.0 * ws;
            for(int j=0;j<4;++j){
                float angb = 90.0 * j * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << -0.05 << zcb + R * cos(angb) << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
                angb += 45 * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << -0.05 << zcb + R * cos(angb) << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
                angb += 45 * 0.017452;
                tsp->TSPolygons << xcb - R * sin(angb) << -0.05 << zcb + R * cos(angb) << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
                tsp->TSPolygons << xcb                 << -0.05 << zcb                 << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
            }

            // Left Arrow
            xe1 = ws * (-1.0);
            xe2 = xe1 + 2.0 * R;
            float zt = (-0.75) * hs + R * 0.2;
            float zb = (-0.75) * hs - R * 0.2;
            tsp->TSPolygons << xe1 << -0.05 << zt << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 << -0.05 << zb << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << -0.05 << zb << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << -0.05 << zt << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;

            xe2 = xe1 + R / 2.0;
            zb = zt;
            zt = (-0.5) * hs;
            tsp->TSPolygons << xe2           << -0.05 << zt << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1           << -0.05 << zb << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 + R * 0.2 << -0.05 << zb << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 + R * 0.2 << -0.05 << zt << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;

            zb = hs * (-1.0);
            zt = (-0.75) * hs - R * 0.2;
            tsp->TSPolygons << xe1           << -0.05 << zt << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2           << -0.05 << zb << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 + R * 0.2 << -0.05 << zb << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 + R * 0.2 << -0.05 << zt << 0.4f << 0.4f << 0.443137f << 1.0f << 0.729412f;


            // Straight Arrow
            xe1 = R * (-0.2);
            xe2 = R * 0.2;
            zt = hs * (-0.5);
            zb = hs * (-1.0);
            tsp->TSPolygons << xe1 << -0.05 << zt << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 << -0.05 << zb << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << -0.05 << zb << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << -0.05 << zt << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;

            xe2 = xe1;
            xe1 = R * (-1.0);
            zb = hs * (-0.75);
            tsp->TSPolygons << xe1 << -0.05 << zb           << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 << -0.05 << zb - R * 0.2 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << -0.05 << zt - R * 0.2 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << -0.05 << zt           << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;

            xe1 = R * 0.2;
            xe2 = R;
            tsp->TSPolygons << xe1 << -0.05 << zt           << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 << -0.05 << zt - R * 0.2 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << -0.05 << zb - R * 0.2 << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << -0.05 << zb           << 0.5f << 0.5f << 0.443137f << 1.0f << 0.729412f;

            // Right Arrow
            xe2 = ws * (1.0);
            xe1 = xe2 - 2.0 * R;
            zt = (-0.75) * hs + R * 0.2;
            zb = (-0.75) * hs - R * 0.2;
            tsp->TSPolygons << xe1 << -0.05 << zt << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 << -0.05 << zb << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << -0.05 << zb << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 << -0.05 << zt << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;

            xe1 = xe2 - R / 2.0;
            zb = yt;
            zt = (-0.5) * hs;
            tsp->TSPolygons << xe1           << -0.05 << zt << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 - R * 0.2 << -0.05 << zt << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 - R * 0.2 << -0.05 << zb << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2           << -0.05 << zb << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;

            zb = (-1.0) * hs;
            zt = (-0.75) * hs - R * 0.2;
            tsp->TSPolygons << xe1           << -0.05 << zb << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2           << -0.05 << zt << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe2 - R * 0.2 << -0.05 << zt << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << xe1 - R * 0.2<< -0.05  << zb << 0.6f << 0.6f << 0.443137f << 1.0f << 0.729412f;

        }
        else if( trafficSignal[i]->type == 'p' ){

            int nVertex = 6 * 4;
            tsp->TSPolygons.reserve( nVertex * 8 );

            float L = 1.0;
            float ws = L * 0.5;
            float hs = 2 * L;

            // plate 1
            tsp->TSPolygons << -ws << hs << 0.0 << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;
            tsp->TSPolygons << -ws << 0.0 << 0.0 << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;
            tsp->TSPolygons <<  ws << 0.0 << 0.0 << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;
            tsp->TSPolygons <<  ws << hs << 0.0 << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;

            // plate 2
            tsp->TSPolygons << -ws << 0.0 << 0.0 << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;
            tsp->TSPolygons << -ws << 0.0 << -hs << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;
            tsp->TSPolygons <<  ws << 0.0 << -hs << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;
            tsp->TSPolygons <<  ws << 0.0 << 0.0 << 0.0f << 0.0f << 0.97f << 0.97f << 0.97f;

            // blue
            tsp->TSPolygons << -ws << hs << 0.05 << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << -ws << L  << 0.05 << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons <<  ws << L  << 0.05 << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons <<  ws << hs << 0.05 << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;

            // blue
            tsp->TSPolygons << -ws << -0.05 << 0.0 << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons << -ws << -0.05 << -L  << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons <<  ws << -0.05 << -L  << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;
            tsp->TSPolygons <<  ws << -0.05 << 0.0 << 0.1f << 0.1f << 0.443137f << 1.0f << 0.729412f;

            // red
            tsp->TSPolygons << -ws << L   << 0.05 << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
            tsp->TSPolygons << -ws << 0.0 << 0.05 << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
            tsp->TSPolygons <<  ws << 0.0 << 0.05 << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
            tsp->TSPolygons <<  ws << L   << 0.05 << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;

            // red
            tsp->TSPolygons << -ws << -0.05 << -L  << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
            tsp->TSPolygons << -ws << -0.05 << -hs << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
            tsp->TSPolygons <<  ws << -0.05 << -hs << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
            tsp->TSPolygons <<  ws << -0.05 << -L  << 0.3f << 0.3f << 1.0f << 0.141176f << 0.02745f;
        }


        tsp->TSPolygonsBuffer->setUsagePattern( QOpenGLBuffer::StaticDraw );

        tsp->TSPolygonsBuffer->allocate( tsp->TSPolygons.constData(),
                                         tsp->TSPolygons.size() * sizeof(GLfloat) );

        program->enableAttributeArray( 0 );
        program->setAttributeBuffer( 0, GL_FLOAT, 0, 3, 8 * sizeof(GLfloat) );


        program->enableAttributeArray( 1 );
        program->setAttributeBuffer( 1, GL_FLOAT, 3 * sizeof(GLfloat) , 2, 8 * sizeof(GLfloat) );


        program->enableAttributeArray( 2 );
        program->setAttributeBuffer( 2, GL_FLOAT, 5 * sizeof(GLfloat) , 3, 8 * sizeof(GLfloat) );


        tsp->TSPolygonsBuffer->release();
        tsp->TSPolygonsArray.release();

        TSPolygons.append( tsp );
    }

    doneCurrent();
}


void GraphicCanvas::SetRoadData()
{
    if( !road ){
        return;
    }

    qDebug() << "[GraphicCanvas::SetRoadData]";
    qDebug() << "number of path data : " << road->paths.size();

    makeCurrent();


    int nPoly = 0;
    for(int i=0;i<road->paths.size();++i){
        nPoly += road->paths[i]->numDivPath + 1;
    }
    for(int i=0;i<road->pedestPaths.size();++i){
        nPoly += road->pedestPaths[i]->shape.size() - 1;
    }
    int nVertex = nPoly * 4;
    qDebug() << " nVertex = " << nVertex;


    xmin = -50.0;
    xmax = 50.0;
    ymin = -50.0;
    ymax = 50.0;

    bool setEyePos = true;

    if( nVertex > 0 ){

        if( pathPolygons ){
            qDebug() << "Clear old road data.";

            pathPolygons->pathPolygonData.clear();
            pathPolygons->pathPolygonsBuffer->release();
            pathPolygons->pathPolygonsBuffer->destroy();
            pathPolygons->pathPolygonsArray.release();

            delete pathPolygons;

            setEyePos = false;
        }

        //qDebug() << "Create struct PathPolygon";

        pathPolygons = new struct PathPolygon;

        bool ret;
        ret = pathPolygons->pathPolygonsArray.create();
        if( !ret ){
            qDebug() << "pathPolygons->pathPolygonsArray.craete() failed.";
            doneCurrent();
            return;
        }
        //qDebug() << "pathPolygonsArray created.";

        pathPolygons->pathPolygonsArray.bind();
        //qDebug() << "pathPolygonsArray bind.";

        pathPolygons->pathPolygonsBuffer = new QOpenGLBuffer();
        //qDebug() << "pathPolygonsBuffer allocated.";

        ret = pathPolygons->pathPolygonsBuffer->create();
        if( !ret ){
            qDebug() << "pathPolygons->pathPolygonsBuffer->craete() failed.";
            doneCurrent();
            return;
        }
        //qDebug() << "pathPolygonsBuffer created.";

        ret = pathPolygons->pathPolygonsBuffer->bind();
        if( !ret ){
            qDebug() << "pathPolygons->pathPolygonsBuffer->bind() failed.";
            doneCurrent();
            return;
        }
        //qDebug() << "pathPolygonsBuffer binded.";

        pathPolygons->pathPolygonData.reserve( nVertex * 5 );

        float maxZ = 0.0;
        float minZ = 0.0;
        for(int i=0;i<road->paths.size();++i){
            for(int j=1;j<road->paths[i]->pos.size();++j){
                float zs = road->paths[i]->pos[j-1]->z();
                float ze = road->paths[i]->pos[j]->z();
                if( i == 0 && j == 1 ){
                    maxZ = zs;
                    minZ = zs;
                }
                if( maxZ < zs ){
                    maxZ = zs;
                }
                if( minZ > zs ){
                    minZ = zs;
                }
                if( maxZ < ze ){
                    maxZ = ze;
                }
                if( minZ > ze ){
                    minZ = ze;
                }
            }
        }
        if( fabs(maxZ - minZ) < 0.1 ){
            maxZ += 0.01;
        }

        for(int i=0;i<road->paths.size();++i){

            for(int j=1;j<road->paths[i]->pos.size();++j){

                float half_width = 1.5;
                float xs = road->paths[i]->pos[j-1]->x();
                float ys = road->paths[i]->pos[j-1]->y();
                float zs = road->paths[i]->pos[j-1]->z();
                float cs = road->paths[i]->derivative[j-1]->x();
                float ss = road->paths[i]->derivative[j-1]->y();
                float x1 = xs - ss * half_width;
                float y1 = ys + cs * half_width;
                float x2 = xs + ss * half_width;
                float y2 = ys - cs * half_width;

                float xe = road->paths[i]->pos[j]->x();
                float ye = road->paths[i]->pos[j]->y();
                float ze = road->paths[i]->pos[j]->z();
                float ce = road->paths[i]->derivative[j]->x();
                float se = road->paths[i]->derivative[j]->y();
                float x4 = xe - se * half_width;
                float y4 = ye + ce * half_width;
                float x3 = xe + se * half_width;
                float y3 = ye - ce * half_width;

                if( road->paths[i]->scenarioObjectID >= 0 ){
                    zs += 0.1;
                    ze += 0.1;
                }

                float c1 = (zs - minZ + 0.01) / (maxZ - minZ) * 0.7 + 0.2;
                float c2 = (ze - minZ + 0.01) / (maxZ - minZ) * 0.7 + 0.2;

                if( road->paths[i]->scenarioObjectID >= 0 ){
                    pathPolygons->pathPolygonData << x1 << y1 << zs << 0.0f << 0.0f << 0.2f << 0.3f << c1;
                    pathPolygons->pathPolygonData << x2 << y2 << zs << 0.0f << 0.0f << 0.2f << 0.3f << c1;
                    pathPolygons->pathPolygonData << x3 << y3 << ze << 0.0f << 0.0f << 0.2f << 0.3f << c2;
                    pathPolygons->pathPolygonData << x4 << y4 << ze << 0.0f << 0.0f << 0.2f << 0.3f << c2;
                }
                else{
                    pathPolygons->pathPolygonData << x1 << y1 << zs << 0.0f << 0.0f << c1 << 1.0-c1 << 0.3f;
                    pathPolygons->pathPolygonData << x2 << y2 << zs << 0.0f << 0.0f << c1 << 1.0-c1 << 0.3f;
                    pathPolygons->pathPolygonData << x3 << y3 << ze << 0.0f << 0.0f << c2 << 1.0-c2 << 0.3f;
                    pathPolygons->pathPolygonData << x4 << y4 << ze << 0.0f << 0.0f << c2 << 1.0-c2 << 0.3f;
                }



                if( xs < xmin ){
                    xmin = xs;
                }
                if( xe < xmin ){
                    xmin = xe;
                }

                if( xs > xmax ){
                    xmax = xs;
                }
                if( xe > xmax ){
                    xmax = xe;
                }

                if( ys < ymin ){
                    ymin = ys;
                }
                if( ye < ymin ){
                    ymin = ye;
                }

                if( ys > ymax ){
                    ymax = ys;
                }
                if( ye > ymax ){
                    ymax = ye;
                }
            }
        }

        for(int i=0;i<road->pedestPaths.size();++i){

            for(int j=0;j<road->pedestPaths[i]->shape.size()-1;++j){

                float half_width = road->pedestPaths[i]->shape[j]->width * 0.5;
                float xs = road->pedestPaths[i]->shape[j]->pos.x();
                float ys = road->pedestPaths[i]->shape[j]->pos.y();
                float zs = road->pedestPaths[i]->shape[j]->pos.z() + 0.1;
                float cs = road->pedestPaths[i]->shape[j]->cosA;
                float ss = road->pedestPaths[i]->shape[j]->sinA;
                float x1 = xs - ss * half_width;
                float y1 = ys + cs * half_width;
                float x2 = xs + ss * half_width;
                float y2 = ys - cs * half_width;

                float xe = road->pedestPaths[i]->shape[j+1]->pos.x();
                float ye = road->pedestPaths[i]->shape[j+1]->pos.y();
                float ze = road->pedestPaths[i]->shape[j+1]->pos.z() + 0.1;
                float ce = road->pedestPaths[i]->shape[j]->cosA;
                float se = road->pedestPaths[i]->shape[j]->sinA;
                float x4 = xe - se * half_width;
                float y4 = ye + ce * half_width;
                float x3 = xe + se * half_width;
                float y3 = ye - ce * half_width;

                if( road->pedestPaths[i]->scenarioObjectID >= 0 ){
                    zs += 0.1;
                    ze += 0.1;
                }

                if( road->pedestPaths[i]->scenarioObjectID >= 0 ){
                    pathPolygons->pathPolygonData << x1 << y1 << zs << 0.0f << 0.0f << 0.2f << 0.3f << 0.7f;
                    pathPolygons->pathPolygonData << x2 << y2 << zs << 0.0f << 0.0f << 0.2f << 0.3f << 0.7f;
                    pathPolygons->pathPolygonData << x3 << y3 << ze << 0.0f << 0.0f << 0.2f << 0.3f << 0.7f;
                    pathPolygons->pathPolygonData << x4 << y4 << ze << 0.0f << 0.0f << 0.2f << 0.3f << 0.7f;
                }
                else{
                    pathPolygons->pathPolygonData << x1 << y1 << zs << 0.0f << 0.0f << 0.1f << 0.8f << 0.4f;
                    pathPolygons->pathPolygonData << x2 << y2 << zs << 0.0f << 0.0f << 0.1f << 0.8f << 0.4f;
                    pathPolygons->pathPolygonData << x3 << y3 << ze << 0.0f << 0.0f << 0.1f << 0.8f << 0.4f;
                    pathPolygons->pathPolygonData << x4 << y4 << ze << 0.0f << 0.0f << 0.1f << 0.8f << 0.4f;
                }

                if( xs < xmin ){
                    xmin = xs;
                }
                if( xe < xmin ){
                    xmin = xe;
                }

                if( xs > xmax ){
                    xmax = xs;
                }
                if( xe > xmax ){
                    xmax = xe;
                }

                if( ys < ymin ){
                    ymin = ys;
                }
                if( ye < ymin ){
                    ymin = ye;
                }

                if( ys > ymax ){
                    ymax = ys;
                }
                if( ye > ymax ){
                    ymax = ye;
                }
            }
        }

        //qDebug() << "pathPolygonData set.";

        pathPolygons->pathPolygonsBuffer->setUsagePattern( QOpenGLBuffer::StaticDraw );

        pathPolygons->pathPolygonsBuffer->allocate( pathPolygons->pathPolygonData.constData(),
                                                    pathPolygons->pathPolygonData.size() * sizeof(GLfloat) );

        program->enableAttributeArray( 0 );
        program->setAttributeBuffer( 0, GL_FLOAT, 0, 3, 8 * sizeof(GLfloat) );


        program->enableAttributeArray( 1 );
        program->setAttributeBuffer( 1, GL_FLOAT, 3 * sizeof(GLfloat) , 2, 8 * sizeof(GLfloat) );


        program->enableAttributeArray( 2 );
        program->setAttributeBuffer( 2, GL_FLOAT, 5 * sizeof(GLfloat) , 3, 8 * sizeof(GLfloat) );

        pathPolygons->pathPolygonsBuffer->release();
        pathPolygons->pathPolygonsArray.release();
    }

    //
    // Rectangle
    qDebug() << "Create Rectangle Polygon";

    rectPoly.isValid = false;
    int ret = rectPoly.array.create();
    if( !ret ){
        qDebug() << "   rectPoly.array.create failed.";
    }
    rectPoly.array.bind();
    rectPoly.buffer = new QOpenGLBuffer();
    ret =  rectPoly.buffer->create();
    if( !ret ){
        qDebug() << "   rectPoly.buffer.create failed.";
    }
    else{
        ret = rectPoly.buffer->bind();
        if( !ret ){
            qDebug() << "   rectPoly.buffer.bind failed.";
        }
        else{

            rectPoly.vertex << -1.0 << 1.0 << 0.0 << 0.0 << 0.0 << 1.0 << 1.0 << 1.0;
            rectPoly.vertex << -1.0 << -1.0 << 0.0 << 1.0 << 0.0 << 1.0 << 1.0 << 1.0;
            rectPoly.vertex <<  1.0 << -1.0 << 0.0 << 1.0 << 1.0 << 1.0 << 1.0 << 1.0;
            rectPoly.vertex <<  1.0 <<  1.0 << 0.0 << 0.0 << 1.0 << 1.0 << 1.0 << 1.0;

            rectPoly.buffer->setUsagePattern( QOpenGLBuffer::StaticDraw );
            rectPoly.buffer->allocate( rectPoly.vertex.constData(), rectPoly.vertex.size() * sizeof(GLfloat) );

            program->enableAttributeArray( 0 );
            program->setAttributeBuffer( 0, GL_FLOAT, 0, 3, 8 * sizeof(GLfloat) );

            program->enableAttributeArray( 1 );
            program->setAttributeBuffer( 1, GL_FLOAT, 3 * sizeof(GLfloat) , 2, 8 * sizeof(GLfloat) );

            program->enableAttributeArray( 2 );
            program->setAttributeBuffer( 2, GL_FLOAT, 5 * sizeof(GLfloat) , 3, 8 * sizeof(GLfloat) );

            rectPoly.buffer->release();
            rectPoly.array.release();

            rectPoly.isValid = true;
        }
    }


    if( setEyePos ){
        qDebug() << "ResetView";
        ResetView();
    }

    qDebug() << "X_eye = " << X_eye << " Y_eye = " << Y_eye << " Z_eye = " << Z_eye;

    doneCurrent();
}


void GraphicCanvas::ResetView()
{
    X_eye = ( xmax + xmin ) / 2.0;
    Y_eye = ( ymax + ymin ) / 2.0;

    X_eye *= (-1.0);
    Y_eye *= (-1.0);

    float xlen = (xmax - xmin) / 2.0;
    float ylen = (ymax - ymin) / 2.0;
    if( Z_eye == -50.0 && xlen > 0 && ylen > 0 ){
        Z_eye = xlen > ylen ? xlen : ylen;
        Z_eye *= (-1.0) / 0.5578;
    }

    qDebug() << "xmin = " << xmin << " xmax = " << xmax;
    qDebug() << "ymin = " << ymin << " ymax = " << ymax;
    qDebug() << "xlen = " << xlen << " ylen = " << ylen;

    int fs = (int)(-0.0838 * Z_eye) + 10;
    emit ChangeFontScale( fs );
}

void GraphicCanvas::SetTrafficParticipantsData()
{
    makeCurrent();

    for(int i=0;i<road->vehicleKind.size();++i){

        struct VehicleModel *vm = new struct VehicleModel;

        vm->modelID = road->vehicleKind[i]->id;

        vm->wheelBase = road->vehicleKind[i]->length * 0.8;
        vm->lf        = road->vehicleKind[i]->length * 0.4;
        vm->lr        = road->vehicleKind[i]->length * 0.4;
        vm->distCG2FE = road->vehicleKind[i]->length * 0.5;
        vm->distCG2RE = road->vehicleKind[i]->length * 0.5;
        vm->distRA2RE = road->vehicleKind[i]->length * 0.1;
        vm->FRWeightRatio = 0.5;

        vm->objFileDataSet = false;

        vehicleModels.append( vm );
        int idx = vehicleModels.size() - 1;

        SetVehiclePolygon(idx,
                          road->vehicleKind[i]->length * 0.5,
                          road->vehicleKind[i]->length * 0.5,
                          road->vehicleKind[i]->width,
                          road->vehicleKind[i]->height );
    }

    for(int i=0;i<road->pedestrianKind.size();++i){

        struct PersonModel *pm = new struct PersonModel;

        pm->id = road->pedestrianKind[i]->id;

        personModels.append( pm );

        int idx = personModels.size() - 1;
        SetPersonPolygon( idx,
                          road->pedestrianKind[i]->width,
                          road->pedestrianKind[i]->height,
                          road->pedestrianKind[i]->length );
    }


    qDebug() << "[GraphicCanvas::SetTrafficParticipantsData]";
    qDebug() << " vehicleModel : n = " << vehicleModels.size();
    qDebug() << " personModels : n = " << personModels.size();

    doneCurrent();
}


int GraphicCanvas::Get3DPhysCoordFromPickPoint(int xp,int yp, float &x,float &y)
{
    //
    // View coordinate -1 < sx,sy < 1
    float lsx = 2.0 / currentWidth * xp - 1.0;
    float lsy = 1.0 - 2.0 / currentHeight * yp;

    float fieldOfView = 45.0f;
    float aspectRatio = float(currentWidth) / float(currentHeight);

    float Kx = -lsx * aspectRatio * tan( fieldOfView * 0.5 * 0.017452 );
    float Ky = -lsy * tan( fieldOfView * 0.5 * 0.017452 );

    float qx, qy, qz, qw;
    cameraQuat.getAxisAndAngle( &qx, &qy, &qz, &qw );

    float cqw = cos( qw * 0.017452 );
    float sqw = sin( qw * 0.017452 );

    //
    // Rodrigues' rotation formula
    float q11 = qx * qx * (1.0 - cqw) + cqw;
    float q12 = qy * qx * (1.0 - cqw) - qz * sqw;
    float q21 = qx * qy * (1.0 - cqw) + qz * sqw;
    float q22 = qy * qy * (1.0 - cqw) + cqw;
    float q31 = qx * qz * (1.0 - cqw) - qy * sqw;
    float q32 = qy * qz * (1.0 - cqw) + qx * sqw;

    //
    // By assuming z = 0, world coordinate (x, y, 0) can be calculated by
    // solving following Simultaneous equations:
    //      A11 * x + A12 * y = B11
    //      A21 * x + A22 * y = B21
    float A11 = q11 - Kx * q31;
    float A12 = q12 - Kx * q32;
    float A21 = q21 - Ky * q31;
    float A22 = q22 - Ky * q32;

    float xE = X_eye;
    float yE = Y_eye;

    float cpc = cos( cameraPitch );
    float cps = sin( cameraPitch );

    float cyc = cos( cameraYaw );
    float cys = sin( cameraYaw );

    X_trans = xE * cyc - yE * cys;
    Y_trans = xE * cys + yE * cyc;

    yE = Y_trans;
    Y_trans = yE * cpc;
    Z_trans = yE * cps;

    Z_trans += Z_eye;

    float B11 = Kx * Z_trans - X_trans;
    float B21 = Ky * Z_trans - Y_trans;

    float det = A11 * A22 - A12 * A21;
    if( fabs(det) > 1.0e-5 ){

        x = (A22 * B11 - A12 * B21) / det;
        y = (A11 * B21 - A21 * B11) / det;

        return 1;
    }
    else{
        x = 0.0;
        y = 0.0;

        return -1;
    }
}


void GraphicCanvas::LocateAtAgent(int id)
{
    if( id < 0 || id >= maxAgent ){
        return;
    }

    for(int i=0;i<maxAgent;++i){

        if( agent[i]->agentStatus == 0 ){
            continue;
        }

        if( agent[i]->ID == id ){

            X_eye = agent[i]->state.x;
            Y_eye = agent[i]->state.y;

            X_eye *= (-1.0);
            Y_eye *= (-1.0);

            update();

            break;
        }
    }
}


void GraphicCanvas::SetBasemapFile(QString filename,QString settingFileFolder)
{
    qDebug() << "[GraphicCanvas::SetBasemapFile] filename = " << filename;

    // Check supplied filename is absolute or relative
    if( filename.contains(":") == false ){
        filename = settingFileFolder + filename;  // filename is relative from settingFileFolder
    }

    QFile file( filename );
    if( file.open(QIODevice::ReadOnly | QIODevice::Text) == false ){
        qDebug() << "[GraphicCanvas::SetBasemapFile] Cannot open file ; " << filename;
        return;
    }

    QTextStream in(&file);
    QString strLines = in.readAll();
    file.close();

    QStringList strLinesDiv = strLines.split("\n");
    for(int i=0;i<strLinesDiv.size();++i){
        if( QString(strLinesDiv[i]).contains("BASE MAP") == false ){
            continue;
        }

        QStringList datDiv = QString(strLinesDiv[i]).split(",");
        if( datDiv.size() < 6 ){
            continue;
        }

        struct baseMapImage *bmi = new struct baseMapImage;

        bmi->path = QString( datDiv[0] ).remove("BASE MAP :").trimmed();
        bmi->filename = QString( datDiv[1] ).trimmed();
        bmi->x = QString( datDiv[2] ).trimmed().toFloat();
        bmi->y = QString( datDiv[3] ).trimmed().toFloat();
        bmi->scale = QString( datDiv[4] ).trimmed().toFloat();
        bmi->rotate = QString( datDiv[5] ).trimmed().toFloat();

//        qDebug() << "Try to Load: " << bmi->path << bmi->filename;
//        qDebug() << "  x = " << bmi->x << " y = " << bmi->y << " scale = " << bmi->scale;

        LoadMapImage(bmi);

        if( bmi->isValid == true ){
            baseMapImages.append( bmi );

            if( (i+1) % 10 == 0 ){
                qDebug() << "Processed " << (i+1) << "/" << strLinesDiv.size();
            }
        }
        else{
            delete bmi;
        }
    }

    update();
}


void GraphicCanvas::LoadMapImage(struct baseMapImage* bmi)
{
    makeCurrent();

    // Load Image
    QImage map;

    bmi->isValid = false;

    QString PathToFile = bmi->path;
    if( PathToFile.endsWith("/") == false ){
        PathToFile += QString("/");
    }

    QString testname = PathToFile + bmi->filename;

//    qDebug() << "Loading " << testname;

    if( map.load(testname) == false ){
        qDebug() << "Loading " << testname;
        qDebug() << "Failed.";
        return;
    }

    glGenTextures(1, &(bmi->textureID));
    glBindTexture(GL_TEXTURE_2D, bmi->textureID);

    int wi = map.width();
    int hi = map.height();

    bmi->halfWidth = wi * 0.5;
    bmi->halfHeight = hi * 0.5;

    int wi2 = 2;
    while( wi2 <= wi )
        wi2 *= 2;

    if( wi2 > 2048 )
        wi2 = 2048;

    int hi2 = 2;
    while( hi2 <= hi )
            hi2 *= 2;

    if( hi2 > 2048 )
        hi2 = 2048;

    GLubyte *bits;
    int tsz = wi2 * hi2 * 4;
    bits = new GLubyte [tsz];

    tsz = 0;
    for(int i=0;i<wi2;++i){
        int x = (int)((i / (float)wi2) * (float)wi);
        for(int j=0;j<hi2;++j){
            int y = (int)((j / (float)hi2) * (float)hi);
            QRgb pix = map.pixel(x,y);
            bits[tsz++] = (GLubyte)qRed(pix);
            bits[tsz++] = (GLubyte)qGreen(pix);
            bits[tsz++] = (GLubyte)qBlue(pix);
            bits[tsz++] = 0xff;
        }
    }

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D( GL_TEXTURE_2D , 0 , GL_RGBA , wi2 , hi2,  0 , GL_RGBA , GL_UNSIGNED_BYTE , bits );

    delete [] bits;

    glBindTexture(GL_TEXTURE_2D, 0);

    bmi->isValid = true;

    doneCurrent();

//    qDebug() << "Image loaded successfully.";
}

void GraphicCanvas::SetScaleSInterface(int v)
{
    sInterfaceObjScale = v;
}


void GraphicCanvas::LoadOptionalImage()
{
    qDebug() << "+--- LoadOptionalImage";

    QString pathToApp = QApplication::applicationDirPath();
    QString pathToImageList = pathToApp + QString("/optionalImages/image_list.txt");

    QFile file(pathToImageList);
    if( file.open(QIODevice::ReadOnly | QIODevice::Text) == false ){
        qDebug() << "  image_list.txt not found.";
        return;
    }

    qDebug() << "  image_list.txt found.";

    QTextStream in(&file);
    while( in.atEnd() == false ){
        QString tmpStr = in.readLine();

        struct OptionalImage *oi = new OptionalImage;

        oi->isValid = false;
        oi->filename = tmpStr;

        QString testname = tmpStr;
        if( testname.startsWith("C:") == false ){    // Not full path
            if( testname.contains("/") == false ){   // only filename
                testname = pathToApp + QString("/optionalImages/") + testname;
            }
        }

        qDebug() << "  Try to load " << testname;

        QImage map;
        if( map.load(testname) == false ){
            qDebug() << "      Failed.";
            delete oi;
            continue;
        }
        qDebug() << "      Loaded.";

        oi->x = 0.0;
        oi->y = 0.0;
        oi->z = 10.0;
        oi->rotate = 0.0;
        oi->scale = 1.0;

        makeCurrent();

        glGenTextures(1, &(oi->textureID));
        glBindTexture(GL_TEXTURE_2D, oi->textureID);

        int wi = map.width();
        int hi = map.height();

        oi->halfWidth = wi * 0.5;
        oi->halfHeight = hi * 0.5;

        int wi2 = 2;
        while( wi2 <= wi )
            wi2 *= 2;

        if( wi2 > 2048 )
            wi2 = 2048;

        int hi2 = 2;
        while( hi2 <= hi )
                hi2 *= 2;

        if( hi2 > 2048 )
            hi2 = 2048;

        GLubyte *bits;
        int tsz = wi2 * hi2 * 4;
        bits = new GLubyte [tsz];

        tsz = 0;
        for(int i=0;i<wi2;++i){
            int x = (int)((i / (float)wi2) * (float)wi);
            for(int j=0;j<hi2;++j){
                int y = (int)((j / (float)hi2) * (float)hi);
                QRgb pix = map.pixel(x,y);
                bits[tsz++] = (GLubyte)qRed(pix);
                bits[tsz++] = (GLubyte)qGreen(pix);
                bits[tsz++] = (GLubyte)qBlue(pix);
                bits[tsz++] = 0xff;
            }
        }

        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D( GL_TEXTURE_2D , 0 , GL_RGBA , wi2 , hi2,  0 , GL_RGBA , GL_UNSIGNED_BYTE , bits );

        delete [] bits;

        glBindTexture(GL_TEXTURE_2D, 0);

        oi->isValid = true;
        oi->showImage = false;

        optionalImages.append( oi );

        doneCurrent();
    }

    qDebug() << "  Size of optionalImages = " << optionalImages.size();

    file.close();
}

