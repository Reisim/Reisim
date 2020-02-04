#ifndef OBJMESH_H
#define OBJMESH_H

#include <QString>
#include <QStringList>
#include <QVector3D>
#include <QVector2D>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLTexture>
#include <QOpenGLWidget>
#include <QOpenGLShaderProgram>
#include <QFile>
#include <QTextStream>
#include <QOpenGLFunctions>


struct OBJVertex
{
    QVector3D position;
    QVector3D normal;
    QVector3D texcoord;
};


struct OBJSubset
{
    int materalIndex;
    QVector<struct OBJVertex> mVertices;
};


struct OBJMaterial
{
    QString materialName;
    QVector3D ambient;
    QVector3D diffuse;
    QVector3D specular;
    float shiniess;
    double alpha;
    QString ambientMapName;
    QString diffuseMapName;
    QString specularMapName;
    QString bumpMapName;
    int textureIndex;
};

struct OBJBoundingSphere
{
    QVector3D center;
    float radius;
};


struct OBJBoundingBox
{
    QVector3D max;
    QVector3D min;
    QVector3D size;
};


class OBJMesh : protected QOpenGLFunctions
{
public:
    OBJMesh(QOpenGLWidget *p);
    ~OBJMesh();

    bool LoadOBJMeshFromFile(QString filename);
    void SetShaderProgram(QOpenGLShaderProgram* p);
    void Draw();

    struct OBJBoundingBox GetBoundingBox() { return mBoundingBox; }
    struct OBJBoundingSphere GetBoundingSphere() { return mBoundingSphere; }

private:

    QOpenGLWidget *widget;

    void CreateBoundingBox();
    void CreateBoudingSphere();
    bool LoadMaterialFile();
    void SetVertexBuffer();

    QString materialFilename;
    QString objFilename;
    QString folderName;


    QVector<struct OBJSubset *> mSubsets;
    QVector<struct OBJMaterial *> mMaterials;
    QVector<unsigned int> mIndices;

    QVector<QOpenGLTexture *> mTextures;
    QVector<QOpenGLBuffer*> mVertexBuffer;
    QVector<QOpenGLVertexArrayObject *> mVertexArrayObjects;
    QOpenGLShaderProgram *program;

    int mNumSubset;
    int mNumMaterials;

    bool dataLoaded;
    bool shaderSet;

    struct OBJBoundingBox mBoundingBox;
    struct OBJBoundingSphere mBoundingSphere;
};


#endif // OBJMESH_H
