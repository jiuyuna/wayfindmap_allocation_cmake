#pragma once
#include <vector>
#include "Population.h"
#include "Individual.h"
#include "Global.h"
#include "Simulator.h"
#include <thread>
#include <mutex>

using namespace std;

class IbeaSolver
{
public:
	int popsize; //种群数
	int indsize; //个体维度
	int max_generations;//最大迭代次数
	int eva_num; //每个解评估多少次
	static const int number_objective = 2; // 目标数量
	int n_signages;

	Population* parent_pop;
	Population* offspring_pop;
	Population* mixed_pop;

	//保存个体i和个体j间计算出的指标值
	std::vector<std::vector<double>> Indicator_value;
	std::vector<int> flag;

	//输出接口，输出每代的结果
    ofstream *outfile;

	IbeaSolver(int pop_size, int ind_size,int max_gens, int num_eva);
	~IbeaSolver();

	void initialize_population(Population* pop);
	void crossover_mutation();
	static void simulation(Individual* ind_cur,IbeaSolver* _this);
	void evaluate_population(Population* pop);
	void copy_ind(Individual* ind1, Individual* ind2);
	void merge();
    void calcFitness();
	double cal_eps_indicator(Individual* ind1, Individual* ind2);
	void environmental_selection();
	void ibea_selection();
	void track_evolution(int generations);
	void run();

private:
	mutex mymutex;

};
