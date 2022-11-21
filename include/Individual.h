#pragma once

#include <iostream>
#include "Global.h"

#define kappa 0.05
#define pmut_real 0.05
#define maxpop 300 //最大种群数
#define maxvar 100  //最大变量数目
#define maxfun 10  //最大函数数目
class Individual
{
public:
    int rank; //
    double xreal[maxvar]; //个体值
    double obj[maxfun]; //目标函数值
    double fitness; //F，适应度
    double cv;
    int indsize; //ind_size = number_variable = 2*num_signs;
	Individual(int ind_size);
	~Individual();

    void simulation_evaluate();

private:

};


// TODO: 在此处引用程序需要的其他标头。
