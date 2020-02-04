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
