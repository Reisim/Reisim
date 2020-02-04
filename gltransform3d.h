#ifndef GLTRANSFORM3D_H
#define GLTRANSFORM3D_H

#include <QVector3D>
#include <QQuaternion>
#include <QMatrix4x4>

class GLTransform3D
{
public:
    GLTransform3D();

    void translate(const QVector3D &t);
    void scale(const QVector3D &s);
    void rotate(const QQuaternion &r);

    void setTranslation(const QVector3D &t);
    void setScale(const QVector3D &s);
    void setRotation(const QQuaternion &r);

    QMatrix4x4& getWorldMatrix();
    QQuaternion& getQuaternion() { return mRotation; }
    float getScale(){ return mScale.x(); }


private:
    bool dirty;
    QVector3D mTranslation;
    QVector3D mScale;
    QQuaternion mRotation;
    QMatrix4x4 world;
};

Q_DECLARE_TYPEINFO(GLTransform3D, Q_MOVABLE_TYPE);


#endif // GLTRANSFORM3D_H
