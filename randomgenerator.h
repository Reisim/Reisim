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


#ifndef RANDOMGENERATOR_H
#define RANDOMGENERATOR_H


#include <random>


class RandomGenerator
{
public:
    RandomGenerator();
    double GenUniform();
    double GetNormalDist(double mean,double std);
    double GetExponentialDist(double mean);

private:

    std::mt19937 mt;
    std::uniform_real_distribution<> ufd;
    std::normal_distribution<> nd;
    std::exponential_distribution<> expd;
};

#endif // RANDOMGENERATOR_H
