#include <iostream>
#include "Ibea_solver.h"
#include "Individual.h"
#include "Global.h"
#include "Simulator.h"
#include <pthread.h>
#include <thread>
#include <string>
#include <random>
#include <mutex>
using namespace std;

#define D 100 //最大的个体的维度（2*标识数）

mutex mymutex;

void simulation1(Individual *ind_cur)
{
	int n_signages = ind_cur->indsize;
	Simulator sim = Simulator(ind_cur->xreal, n_signages);
	sim.run_simulation();
	mymutex.lock(); //需要互斥访问
	ind_cur->obj[0] += sim.avg_time;
	ind_cur->obj[1] += sim.avg_max_pressure;
	mymutex.unlock();
}

int main1234()
{
	Timer time = Timer();
	time.reset();
	srand(0);
	//种群数、个体维度、最大迭代数、每个解的评估次数
	// IbeaSolver solver(30,30,100,3);
	// solver.run();
	initialize();

	int n_signages = 17;
	double trial[D] = {32.4, 70, 85, 61, 14.9, 55.33, 61, 52.4723, 62.92, 63.7, 27.286, 66.6261, 74, 86, 52.4723, 73.879, 104, 64, 104, 64, 100, 61, 42.9547, 73.3187, 34.9842, 61.57, 76, 79, 89, 61, 54.61, 55, 8, 86};
	//贪心

	Individual *ind = new Individual(n_signages);
	for (int i = 0; i < 2 * n_signages; i += 2)
	{
		ind->xreal[i] = trial[i];
		ind->xreal[i + 1] = trial[i + 1];
	}

	int num_threads = 20;
	while (1)
	{
		ind->obj[0] = 0;
		ind->obj[1] = 0;
		vector<thread> ths;
		for (int i = 0; i < num_threads; ++i)
		{
			ths.push_back(thread(simulation1, ind));
		}
		for (int i = 0; i < num_threads; ++i)
		{
			ths[i].join();
		}
		ind->obj[0] = ind->obj[0] / num_threads;
		ind->obj[1] = ind->obj[1] / num_threads;
		cout << ind->obj[0] << ' ' << ind->obj[1] << endl;
	}

	cout << "Total time : " << time.elapsed() << endl;
	time.reset();
	return 0;
}

int main()
{
	Timer time = Timer();
	time.reset();
	srand(0);

	int n_signages = 10;
	initialize();
	IbeaSolver solver(30, 2 * n_signages, 1000, 30); //种群数、个体维度、最大迭代数、每个解的评估次数
	solver.run();

	cout << "Total time : " << time.elapsed() << endl;
	return 0;
}
