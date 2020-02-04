#include "objmesh.h"
#include <QDebug>


#ifndef BUFFER_OFFSET
#define BUFFER_OFFSET(bytes) ((GLubyte *)NULL + (bytes))
#endif



OBJMesh::OBJMesh( QOpenGLWidget *p )
{
    initializeOpenGLFunctions();

    widget = p;

    mNumSubset    = 0;
    mNumMaterials = 0;

    dataLoaded = false;
    shaderSet  = false;
}


OBJMesh::~OBJMesh()
{
    qDebug() << "Clean Up: OBJMesh";

    widget->makeCurrent();


    for(int i=0;i<mSubsets.size();++i){
        mSubsets[i]->mVertices.clear();
        delete mSubsets[i];
    }
    mSubsets.clear();

    for(int i=0;i<mMaterials.size();++i){
        delete mMaterials[i];
    }
    mMaterials.clear();

    for(int i=0;i<mTextures.size();++i){
        mTextures[i]->destroy();
    }
    mTextures.clear();

    for(int i=0;i<mVertexBuffer.size();++i){
        mVertexBuffer[i]->destroy();
        delete mVertexBuffer[i];
    }
    mVertexBuffer.clear();

    for(int i=0;i<mVertexArrayObjects.size();++i){
        mVertexArrayObjects[i]->destroy();
        delete mVertexArrayObjects[i];
    }
    mVertexArrayObjects.clear();

    widget->doneCurrent();

    qDebug() << "OBJMesh cleared.";
}

void OBJMesh::CreateBoudingSphere()
{
    mBoundingSphere.center = (mBoundingBox.max + mBoundingBox.min) / 2.0;
    mBoundingSphere.radius = (mBoundingBox.max - mBoundingSphere.center).length();
}

void OBJMesh::CreateBoundingBox()
{
    mBoundingBox.max = QVector3D(0.0,0.0,0.0);
    mBoundingBox.min = QVector3D(0.0,0.0,0.0);
    for(int i=0;i<mSubsets.size();++i){
        for(int j=0;j<mSubsets[i]->mVertices.size();++j){
            if( i == 0 && j == 0 ){
                mBoundingBox.max = mSubsets[i]->mVertices[j].position;
                mBoundingBox.min = mSubsets[i]->mVertices[j].position;
            }
            else{

                float x = mSubsets[i]->mVertices[j].position.x();
                float y = mSubsets[i]->mVertices[j].position.y();
                float z = mSubsets[i]->mVertices[j].position.z();

                if( mBoundingBox.max.x() < x ){
                    mBoundingBox.max.setX( x );
                }
                if( mBoundingBox.max.y() < y ){
                    mBoundingBox.max.setY( y );
                }
                if( mBoundingBox.max.z() < z ){
                    mBoundingBox.max.setZ( z );
                }

                if( mBoundingBox.min.x() > x ){
                    mBoundingBox.min.setX( x );
                }
                if( mBoundingBox.min.y() > y ){
                    mBoundingBox.min.setY( y );
                }
                if( mBoundingBox.min.z() > z ){
                    mBoundingBox.min.setZ( z );
                }
            }
        }
    }

    mBoundingBox.size = mBoundingBox.max - mBoundingBox.min;
}


bool OBJMesh::LoadOBJMeshFromFile(QString filename)
{
    QVector<QVector3D> positions;
    QVector<QVector3D> normals;
    QVector<QVector2D> texcoords;


    while( filename.contains("\\") ){
        filename = filename.replace("\\","/");
    }

    objFilename = filename;

    QString tmpFilename = filename;
    folderName = tmpFilename.mid(0, tmpFilename.lastIndexOf("/") );

    qDebug() << "[LoadOBJMeshFromFile]";
    qDebug() << "   [Filename]" << filename;
    qDebug() << "   [folderName]" << folderName;


    QFile file(filename);
    if( !file.open(QIODevice::ReadOnly | QIODevice::Text) ){
        qDebug() << "Cannnot open file: " << filename;
        return false;
    }

    QTextStream in(&file);

    while( in.atEnd() == false ){

        QString line = in.readLine();

        //qDebug() << line;

        if( line.isNull() || line.isEmpty() ){
            continue;
        }

        if( line.startsWith("#") ){
            continue;
        }

        QStringList divStr = line.split(" ");
        QString tag = QString(divStr[0]).trimmed();

        if( tag == QString("v") ){

            float x = QString(divStr[1]).trimmed().toFloat();
            float y = QString(divStr[2]).trimmed().toFloat();
            float z = QString(divStr[3]).trimmed().toFloat();
            positions.append( QVector3D(x,y,z) );

            //qDebug() << "append positions, " << x << " " << y << " " << z;

        }
        else if( tag == QString("vt") ){

            float u = QString(divStr[1]).trimmed().toFloat();
            float v = QString(divStr[2]).trimmed().toFloat();
            texcoords.append( QVector2D(u,v) );

            //qDebug() << "append texcoords, " << u << " " << v;

        }
        else if( tag == QString("vn") ){

            float x = QString(divStr[1]).trimmed().toFloat();
            float y = QString(divStr[2]).trimmed().toFloat();
            float z = QString(divStr[3]).trimmed().toFloat();
            normals.append( QVector3D(x,y,z) );

            //qDebug() << "append normals, " << x << " " << y << " " << z;
        }
        else if( tag == QString("f") ){

            int nFace = divStr.size() - 1;

            if( nFace > 3 ){
                qDebug() << " !!! Only triangle surface supported !!!";
                continue;
            }

            for(int iFace = 0;iFace < nFace;++iFace){

                int iPosition = -1;
                int iTexCoord = -1;
                int iNormal   = -1;

                QStringList surfData = QString(divStr[iFace+1]).trimmed().split("/");

                //qDebug() << " Suface[" << iFace << "]: " << QString(divStr[iFace+1]) << " size = " << surfData.size();

                iPosition = QString(surfData[0]).trimmed().toInt();

                if( surfData.size() > 1 ){
                    iTexCoord = QString(surfData[1]).trimmed().toInt();
                }

                if( surfData.size() > 2 ){
                    iNormal = QString(surfData[2]).trimmed().toInt();
                }

                struct OBJVertex *vertex = new struct OBJVertex;

                vertex->position = positions[iPosition-1];

                //qDebug() << " set vertex->position, index = " << (iPosition-1);
                if( iTexCoord >= 0 ){
                    vertex->texcoord = texcoords[iTexCoord-1];
                    //qDebug() << " set vertex->texcoord, index = " << (iTexCoord-1);
                }
                else{
                    vertex->texcoord = QVector2D(0.0,0.0);
                }

                if( iNormal >= 0 ){
                    vertex->normal = normals[iNormal-1];
                    //qDebug() << " set vertex->normal, index = " << (iNormal-1);
                }
                else{
                    vertex->normal = QVector3D(0.0,0.0,0.0);
                }

                mSubsets.last()->mVertices.append( *vertex );

                delete vertex;
            }
        }
        else if( tag == QString("mtllib") ){

            materialFilename = QString( divStr[1] ).trimmed();

            if( !LoadMaterialFile() ){
                qDebug() << "Failed to load materal file: " << materialFilename;
                file.close();
                return false;
            }

        }
        else if( tag == QString("usemtl") ){

            QString matName = QString( divStr[1] ).trimmed();

            struct OBJSubset *subset = new struct OBJSubset;

            for(int i=0;i<mNumMaterials;++i){
                if( mMaterials[i]->materialName == matName ){
                    subset->materalIndex = i;
                    break;
                }
            }

            mSubsets.append( subset );
        }
    }


    file.close();


    CreateBoundingBox();
    CreateBoudingSphere();


    mNumSubset    = mSubsets.size();
    mNumMaterials = mMaterials.size();


//    qDebug() << "mNumSubset = " << mNumSubset;
//    qDebug() << "mNumMaterials = " << mNumMaterials;


    positions.clear();
    normals.clear();
    texcoords.clear();

    SetVertexBuffer();

    dataLoaded = true;

    return true;
}


bool OBJMesh::LoadMaterialFile()
{
    QString loadFilename = folderName;

    while( loadFilename.contains("\\")){
        loadFilename = loadFilename.replace("\\","/");
    }

    if( loadFilename.endsWith("/") ){
        loadFilename += materialFilename;
    }
    else{
        loadFilename += QString("/") + materialFilename;
    }

    qDebug() << "[LoadMaterialFile]";
    qDebug() << "   [loadFilename]" << loadFilename;

    QFile file(loadFilename);
    if( !file.open(QIODevice::ReadOnly | QIODevice::Text) ){
        qDebug() << "Cannot open file.";
        return false;
    }

    QTextStream in(&file);

    int iMtlCount = -1;
    QString matName;
    QVector<struct OBJMaterial*> t_materials;

    while( in.atEnd() == false ){

        QString line = in.readLine();

        if( line.startsWith("newmtl") ){

            iMtlCount++;
            struct OBJMaterial *material = new struct OBJMaterial;
            t_materials.append( material );

            QStringList divStr = line.split(" ");
            matName = QString( divStr[1] ).trimmed();

            material->materialName = matName;
            material->ambient  = QVector3D(0.2,0.2,0.2);
            material->diffuse  = QVector3D(0.8,0.8,0.8);
            material->specular = QVector3D(1.0,1.0,1.0);
            material->shiniess = 0.0;
            material->alpha    = 1.0;
            material->ambientMapName  = QString();
            material->diffuseMapName  = QString();
            material->specularMapName = QString();

        }
        else if( line.startsWith("Ka") ){

            QStringList divStr = line.split(" ");
            float v1 = QString(divStr[1]).trimmed().toFloat();
            float v2 = QString(divStr[2]).trimmed().toFloat();
            float v3 = QString(divStr[3]).trimmed().toFloat();

            t_materials[iMtlCount]->ambient = QVector3D(v1,v2,v3);

        }
        else if( line.startsWith("Kd") ){

            QStringList divStr = line.split(" ");
            float v1 = QString(divStr[1]).trimmed().toFloat();
            float v2 = QString(divStr[2]).trimmed().toFloat();
            float v3 = QString(divStr[3]).trimmed().toFloat();

            t_materials[iMtlCount]->diffuse = QVector3D(v1,v2,v3);

        }
        else if( line.startsWith("Ks") ){

            QStringList divStr = line.split(" ");
            float v1 = QString(divStr[1]).trimmed().toFloat();
            float v2 = QString(divStr[2]).trimmed().toFloat();
            float v3 = QString(divStr[3]).trimmed().toFloat();

            t_materials[iMtlCount]->specular = QVector3D(v1,v2,v3);

        }
        else if( line.startsWith("d") ){

            QStringList divStr = line.split(" ");
            float v1 = QString(divStr[1]).trimmed().toFloat();

            t_materials[iMtlCount]->alpha = v1;

        }
        else if( line.startsWith("Ns") ){

            QStringList divStr = line.split(" ");
            float v1 = QString(divStr[1]).trimmed().toFloat();

            t_materials[iMtlCount]->shiniess = v1;

        }
        else if( line.startsWith("map_Ka") ){

            QStringList divStr = line.split(" ");

            t_materials[iMtlCount]->ambientMapName = QString( divStr[1] ).trimmed();

        }
        else if( line.startsWith("map_Kd") ){

            QStringList divStr = line.split(" ");

            t_materials[iMtlCount]->diffuseMapName = QString( divStr[1] ).trimmed();

        }
        else if( line.startsWith("map_Ks") ){

            QStringList divStr = line.split(" ");

            t_materials[iMtlCount]->specularMapName = QString( divStr[1] ).trimmed();

        }
        else if( line.startsWith("map_Bump") ){

            QStringList divStr = line.split(" ");

            t_materials[iMtlCount]->bumpMapName = QString( divStr[1] ).trimmed();

        }
    }

    file.close();


    mNumMaterials = t_materials.size();
    mMaterials.resize( mNumMaterials );
    for(int i=0;i<mNumMaterials;++i){
        mMaterials[i] = t_materials[i];
    }

    widget->makeCurrent();
    for(int i=0;i<mNumMaterials;++i){
        if( mMaterials[i]->diffuseMapName.isNull() || mMaterials[i]->diffuseMapName.isEmpty() ){
            mMaterials[i]->textureIndex = -1;
            continue;
        }

        QString textureFilename = mMaterials[i]->diffuseMapName;
        if( textureFilename.contains(":/") == false ){
            textureFilename = folderName;
            if( textureFilename.endsWith("/") == false ){
                textureFilename += QString("/");
            }
            textureFilename += mMaterials[i]->diffuseMapName;
        }

        qDebug() << "   [textureFilename]" << textureFilename;

        QImage img = QImage(textureFilename);
        if( img.isNull() ){
            qDebug() << "Failed to load image : " << textureFilename;
            continue;
        }
        QOpenGLTexture *texture = new QOpenGLTexture( img.mirrored() );
        mTextures.append( texture );
        mMaterials[i]->textureIndex = mTextures.size() - 1;
    }
    widget->doneCurrent();

    qDebug() << "[End of LoadMaterialFile]";

    return true;
}


void OBJMesh::SetVertexBuffer()
{
    qDebug() << "[SetVertexBuffer]";

    if( shaderSet == false ){
        qDebug() << "  !!! Shader Program not set. !!!";
        return;
    }

    widget->makeCurrent();

    ////////////////////////////////

    for(int i=0;i<mNumSubset;++i){

        QOpenGLVertexArrayObject *vao = new QOpenGLVertexArrayObject();
        mVertexArrayObjects.append( vao );

        bool ret = vao->create();
        if( ret == false ){
            qDebug() << "Failed to create Vertex Array Object";
            return;
        }

        vao->bind();


        QOpenGLBuffer *vbo = new QOpenGLBuffer();
        mVertexBuffer.append( vbo );

        ret = vbo->create();
        if( ret == false ){
            qDebug() << "Failed to create Vertex Buffer";
            return;
        }

        ret = vbo->bind();
        if( ret == false ){
            qDebug() << "Failed to bind Vertex Buffer";
            return;
        }

        vbo->setUsagePattern( QOpenGLBuffer::StaticDraw );

        vbo->allocate( mSubsets[i]->mVertices.constData(), mSubsets[i]->mVertices.size() * sizeof(struct OBJVertex) );


        program->enableAttributeArray( 0 );
        program->setAttributeBuffer( 0, GL_FLOAT, 0, 3, sizeof(struct OBJVertex) );

        program->enableAttributeArray( 1 );
        program->setAttributeBuffer( 1, GL_FLOAT, 2 * sizeof(QVector3D) , 2, sizeof(struct OBJVertex) );


        vbo->release();
        vao->release();
    }

    ////////////////////////////////

    widget->doneCurrent();

    qDebug() << "[End of SetVertexBuffer]";
}


void OBJMesh::SetShaderProgram(QOpenGLShaderProgram *p)
{
    if( !p ){
        return;
    }

    //widget->makeCurrent();
    if( p->hasOpenGLShaderPrograms() == false ){
        return;
    }

    program = p;

    //widget->doneCurrent();

    shaderSet = true;
}


void OBJMesh::Draw()
{
    if( dataLoaded == false ){
        return;
    }

    if( shaderSet == false ){
        return;
    }

    //qDebug() << "[Draw]";
    for(int i=0;i<mNumSubset;++i){

        //qDebug() << "Bind VAO[" << i << "]";
        mVertexArrayObjects[i]->bind();

        //qDebug() << "SetMaterial";
        int colorPos  = program->uniformLocation("vColor");
        OBJMaterial* pMat = mMaterials[mSubsets[i]->materalIndex];
        program->setUniformValue( colorPos, QVector4D(pMat->diffuse.x(), pMat->diffuse.y(), pMat->diffuse.z(), pMat->alpha) );

        int texIdx = mMaterials[mSubsets[i]->materalIndex]->textureIndex;
        int useTex  = program->uniformLocation("useTex");
        if( texIdx >= 0 ){
           // qDebug() << "Bind Texture[" << texIdx << "]";
            mTextures[texIdx]->bind();
            program->setUniformValue( useTex, 1 );
        }
        else{
            //qDebug() << "Not use texture";
            program->setUniformValue( useTex, 0 );
        }

        int offsetPos = program->uniformLocation("offsetPos");
        program->setUniformValue( offsetPos, QVector4D(0.0,0.0,0.0,0.0) );

        int modelRotateLocal = program->uniformLocation("modelRotateLocal");
        QMatrix4x4 identMat;
        identMat.setToIdentity();
        program->setUniformValue( modelRotateLocal, identMat );


        //qDebug() << "glDrawElements";
        glDrawArrays(GL_TRIANGLES, 0, mSubsets[i]->mVertices.size());

        mVertexArrayObjects[i]->release();
    }
}

