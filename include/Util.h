#pragma once
#include <math.h>
#include <chrono> // for std::chrono functions
#include <cstddef> // for std::size_t
#include <numeric> // for std::iota
static int phase = 0;


//=========================================�������==============================================
inline int random(int left, int right) { //������[left,right]������һ��������� 
    return (int)(left + ((double)rand() / RAND_MAX) * ((right)-(left)));
};
inline double random01() {  //��[0,1]���������� 
    return ((double)rand()) / RAND_MAX;
}
//��������[a,b]�ڵ����������
inline double randval(double a, double b) {
    return a + (b - a) * rand() / (double)RAND_MAX;
}
//��ֵΪ0������Ϊ1��
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

//���ڲ������ɷֲ��������
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


//ʱ���࣬����Ϊ�����ʱ��
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