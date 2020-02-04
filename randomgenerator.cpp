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


#include "randomgenerator.h"

RandomGenerator::RandomGenerator()
{
    ufd = std::uniform_real_distribution<> (0.0,1.0);
    nd  = std::normal_distribution<>(0.0,1.0);

}


double RandomGenerator::GenUniform()
{
    return ufd(mt);
}


double RandomGenerator::GetNormalDist(double mean,double std)
{
    double ret = 0.0;
    double z = nd(mt);
    ret = mean + z * std;
    return ret;
}


double RandomGenerator::GetExponentialDist(double mean)
{
    float ret = 0.0f;
    float temp = 1.0f - GenUniform();
    if(temp <= 1.0e-6f)
        ret = 13.8155f * mean;
    else if(temp >= 0.9f)
        ret = 0.1054f * mean;
    else if(temp > 1.0e-6f && temp < 0.9f)
        ret = -log(temp) * mean;
    return ret;
}

