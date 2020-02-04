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


#include "gltransform3d.h"

GLTransform3D::GLTransform3D()
{
    dirty = true;
    mTranslation = QVector3D(0.0,0.0,0.0);
    mScale = QVector3D(1.0,1.0,1.0);
    mRotation = QQuaternion(1.0,0.0,0.0,0.0);
}

void GLTransform3D::translate(const QVector3D &t)
{
    dirty = true;
    mTranslation += t;
}

void GLTransform3D::scale(const QVector3D &s)
{
    dirty = true;
    mScale *= s;
}

void GLTransform3D::rotate(const QQuaternion &r)
{
    dirty = true;
    mRotation = r * mRotation;
}

void GLTransform3D::setTranslation(const QVector3D &t)
{
    dirty = true;
    mTranslation = t;
}

void GLTransform3D::setScale(const QVector3D &s)
{
    dirty = true;
    mScale = s;
}

void GLTransform3D::setRotation(const QQuaternion &r)
{
    dirty = true;
    mRotation = r;
}

QMatrix4x4& GLTransform3D::getWorldMatrix()
{
    if( dirty ){
        dirty = false;
        world.setToIdentity();
        world.translate(mTranslation);
        world.rotate(mRotation);
        world.scale(mScale);
    }
    return world;
}

