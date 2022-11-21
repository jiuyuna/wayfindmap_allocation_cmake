#pragma once
#include <math.h>
#include <chrono> // for std::chrono functions
#include <cstddef> // for std::size_t
#include <numeric> // for std::iota
static int phase = 0;

//=========================================随机函数==============================================
inline int random(int left, int right) { //在区间[left,right]上生成一个随机整数 
    return (int)(left + ((double)rand() / RAND_MAX) * ((right)-(left)));
};
inline double random01() {  //在[0,1]间产生随机数 
    return ((double)rand()) / RAND_MAX;
}
//产生区间[a,b]内的随机浮点数
inline double randval(double a, double b) {
    return a + (b - a) * rand() / (double)RAND_MAX;
}
//均值为0，方差为1？
inline double gaussrand_NORMAL()
{
    static double V1, V2, S;
    double X;
    if (phase == 0) {
        do {
            double U1 = (double)rand() / (double)RAND_MAX;
            double U2 = (double)rand() / (double)RAND_MAX;
            V1 = 2 * U1 - 1;
            V2 = 2 * U2 - 1;
            S = V1 * V1 + V2 * V2;
        } while (S >= 1 || S == 0);
        X = V1 * sqrt(-2 * log(S) / S);
    }
    else
        X = V2 * sqrt(-2 * log(S) / S);

    phase = 1 - phase;
    return X;
}
inline double gaussrand(double mean, double stdc) {
    return mean + gaussrand_NORMAL() * stdc;
}
inline double U_Random()
{
    double f;
    f = (float)(rand() % 100);
    return f / 100;
}

//用于产生泊松分布的随机数
inline int poisson(double Lambda)
{
    int k = 0;
    long double p = 1.0;
    long double l = exp(-Lambda);
    //printf("%.15Lf\n", l);
    while (p >= l)
    {
        double u = U_Random();
        p *= u;
        k++;
    }
    return k - 1;
}


//时间类，用于为程序计时。
class Timer
{
private:
    // Type aliases to make accessing nested type easier
    using Clock = std::chrono::steady_clock;
    using Second = std::chrono::duration<double, std::ratio<1> >;

    std::chrono::time_point<Clock> m_beg{ Clock::now() };

public:

    void reset()
    {
        m_beg = Clock::now();
    }

    double elapsed() const
    {
        return std::chrono::duration_cast<Second>(Clock::now() - m_beg).count();
    }
};