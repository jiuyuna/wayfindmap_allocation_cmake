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
	int popsize; //��Ⱥ��
	int indsize; //����ά��
	int max_generations;//����������
	int eva_num; //ÿ�����������ٴ�
	static const int number_objective = 2; // Ŀ������
	int n_signages;

	Population* parent_pop;
	Population* offspring_pop;
	Population* mixed_pop;

	//�������i�͸���j��������ָ��ֵ
	std::vector<std::vector<double>> Indicator_value;
	std::vector<int> flag;
	std::vector<double> hv_values;

	//����ӿڣ����ÿ���Ľ��
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
	void caculator_hv2D(Population *pop);
	void environmental_selection();
	void ibea_selection();
	void track_evolution(int generations);
	void run();

private:
	mutex mymutex;

};
