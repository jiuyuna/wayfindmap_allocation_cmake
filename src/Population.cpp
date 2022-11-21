#include "Population.h"
#include "Util.h"


Population::Population(int pop_size, int ind_size):popsize{ pop_size },indsize{ ind_size }
{
    for(int i=0;i<maxpop;++i)
        ind[i] = new Individual(ind_size);
}

Population::~Population()
{
    for(int i=0;i<maxpop;++i)
        delete ind[i];
}

void Population::initialize_population()
{
    //初始化种群
    // for (int i = 0; i < popsize; ++i) {
    //    Individual* ind_cur = ind[i];
    //    for (int j = 0; j < ind_cur->indsize; j += 2) {
    //        int rnd = (int)randval(0, available_points.size());
    //        while (rnd == available_points.size()) rnd = (int)randval(0, available_points.size());
    //        ind_cur->xreal[j] = available_points[rnd].X;
    //        ind_cur->xreal[j + 1] = available_points[rnd].Y;
    //    }
    // };
    
    // //测试函数的初始化种群
    // for (int i = 0; i < popsize; ++i) {
    //     Individual* ind_cur = ind[i];
    //     for (int j = 0; j < indsize; j++) {
    //         ind_cur->xreal[j] = randval(0, 1);
    //     }
    // };
}


void Population::evaluate_population()
{
//     for (int i = 0; i < popsize; ++i) {
//         Individual* ind_cur = ind[i];
//         ind_cur->simulation_evaluate();
//         //zdt1(ind_cur); //test function
//     }
}