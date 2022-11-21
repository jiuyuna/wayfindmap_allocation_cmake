#pragma once

#include <iostream>
#include "Global.h"

#define kappa 0.05
#define pmut_real 0.05
#define maxpop 300 //�����Ⱥ��
#define maxvar 100  //��������Ŀ
#define maxfun 10  //�������Ŀ
class Individual
{
public:
    int rank; //
    double xreal[maxvar]; //����ֵ
    double obj[maxfun]; //Ŀ�꺯��ֵ
    double fitness; //F����Ӧ��
    double cv;
    int indsize; //ind_size = number_variable = 2*num_signs;
	Individual(int ind_size);
	~Individual();

    void simulation_evaluate();

private:

};


// TODO: �ڴ˴����ó�����Ҫ��������ͷ��
