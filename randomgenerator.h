/****************************************************************************
**                                 Re:sim
**
**   Copyright (C) 2020 Misaki Design.LLC
**   Copyright (C) 2020 Jun Tajima <tajima@misaki-design.co.jp>
**
**   This file is part of the Re:sim simulation software
**
**   This file may be used under the terms of the GNU Lesser General Public
**   License version 3 as published by the Free Software Foundation.
**   For more detail, visit https://www.gnu.org/licenses/gpl-3.0.html
**
*************************************************************************** */


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
