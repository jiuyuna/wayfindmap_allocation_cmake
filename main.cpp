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

#define D 100 // 恷寄議倖悶議略業��2*炎紛方��

mutex mymutex;

void simulation_ind(Individual *ind_cur)
{
	int n_signages = ind_cur->indsize;
	Simulator sim = Simulator(ind_cur->xreal, n_signages);
	sim.run_simulation();
	mymutex.lock(); // 俶勣札鰍恵諒
	ind_cur->obj[0] += sim.avg_time;
	ind_cur->obj[1] += sim.avg_max_pressure;
	mymutex.unlock();
}

void simulation_trial(double trial[], ofstream *outfile)
{
	Simulator sim = Simulator(trial, n_signages);
	sim.run_simulation();
	mymutex.lock(); // 俶勣札鰍恵諒
	*outfile << sim.avg_time << ',' << sim.avg_max_pressure << endl;
	mymutex.unlock();
}

void simulation_recordPre(double trial[], ofstream *outfile)
{
	Simulator sim = Simulator(trial, n_signages);
	sim.run_simulation();
	mymutex.lock(); // 俶勣札鰍恵諒
	*outfile << sim.outPressurePerSecond << endl;
	mymutex.unlock();
}




int main()
{
	srand(0);
	Timer time = Timer();
	time.reset();
	initialize();

	string allocation_method = "heatMap";

	if (allocation_method == "ibea")
	{
		IbeaSolver solver(50, 2 * n_signages, 200, 10); // 嶽蛤方、倖悶略業、恷寄亨旗方、耽倖盾議得浩肝方
		cout << n_signages << endl;
		solver.run();
		return 0;
	}
	else if (allocation_method == "heatMap")
	{
		// //scene1
		// double trial[12][D] ={
		// 	{20,2,38,20,20,38,2,20},
		// 	{30, 9, 9, 32, 29, 32, 7, 12},
		// 	{-1, -1, -1, -1, -1, -1, -1, -1},
		// 	{29.9768,13.6345,7.3,20,18.0401,31.9698,13.301,6.18201}};
		// //scene2
		// double trial[12][D] ={
		// 	{20,2,38,20,20,38,2,20},
		// 	{29, 7, 30, 27, 8, 35, 11, 11},
		// 	{18, 23, 20, 20, -1, -1, -1, -1},
		// 	{34.3053,19.5625,16.1663,23.0176,23.1852,26.9779,18.6754,15.9084}};
		// //scene3
		// double trial[12][D] ={
		// 	{57.4,37,38.5,73.5,23.5,66.5,57.4,77,98,62.5,23.5,73.5,68,17.5,12.5,57,31,70,3,87.5},
		// 	{52, 61, 14, 72, 35, 71, 91, 65, 61, 89, 51, 10, 66, 74, 53, 39, 65, 48, 64, 16},
		// 	{13, 62, 24, 72, 41, 70, 52, 7, 52, 62, 52, 67, 63, 18, 64, 47, 77, 64, 102, 62},
		// 	{60,66,104,61,72.7534,64.0564,86,61,32.9,72,25,61,54,48,44.0186,70.19,38,61,62.296,62.458 }
		// or
		//  {47.1655,64.1277,96,61,1,86.2,45.16,72.1,53,65,63.7,66.8,58,89,64.6,60.2,41,68,35,61}
		//  };
		// 10elevator_greedy{23.5,66.5,57.4,18,57.4,37,57.4,57,57.4,77,77,69.5,38.5,66.5,38.5,73.5,31,70,23.5,73.5}
		// //scene3_7outer
		// greedy{12.5,57,68,87.5,98,62.5,70,47.5,68,17.5,47,7.5,7,87.5};
		// SD{51,10,65,74,35,71,14,72,51,61,91,65,53,39};
		// WO{12,87,13,62,52,7,63,18,64,47,82,70,63,87};
		// opt{}
		
		double trial[D] = {43,61,52.5469,51.6211,61.5,86,59,78,56,89,84,61,55.1226,73.7109};

		// randomly
		// for (int i = 0; i < n_signages * 2; i += 2)
		// {
		// 	int rnd = (int)randval(0, available_points.size());
		// 	trial[i] = available_points[rnd].X;
		// 	trial[i + 1] = available_points[rnd].Y;
		// }

		Individual *ind = new Individual(n_signages);
		for (int i = 0; i < 2 * n_signages; i += 2)
		{
			ind->xreal[i] = trial[i];
			ind->xreal[i + 1] = trial[i + 1];
		}

		int num_threads = 1;
		// while (1)
		{
			ind->obj[0] = 0;
			ind->obj[1] = 0;
			vector<thread> ths;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_ind, ind));
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
	else if (allocation_method == "scene1_arbitrary")
	{
		int num_threads = 30;
		ofstream of("/home/cyx/wayfindmap_allocation_cmake/result/Comparison/difftraffic_" + scene_name + "_comparison" + to_string(n_signages), ios::out | ios::trunc);
		/*！！！！！！！！！！！！！！！！Greedy！！！！！！！！！！！！！！！！*/
		double trial_greedy[D];
		vector<ENTRANCE> temp(entrances);
		sort(temp.begin(), temp.end(), cmp2);
		for (int j = 0; j < n_signages; ++j)
		{
			trial_greedy[2 * j] = temp[j].greedy_sign_x;
			trial_greedy[2 * j + 1] = temp[j].greedy_sign_y;
		}
		of << "Greedy" << endl;
		vector<thread> ths;
		for (int i = 0; i < num_threads; ++i)
			ths.push_back(thread(simulation_trial, trial_greedy, &of));
		for (int i = 0; i < num_threads; ++i)
			ths[i].join();

		/*！！！！！！！！！！！！！！！！SD！！！！！！！！！！！！！！！！*/
		double trial_SD[D] = {30, 9, 9, 32, 29, 32, 7, 12};
		of << "SD" << endl;
		ths.clear();
		for (int i = 0; i < num_threads; ++i)
			ths.push_back(thread(simulation_trial, trial_SD, &of));
		for (int i = 0; i < num_threads; ++i)
			ths[i].join();

		/*！！！！！！！！！！！！！！！！WO！！！！！！！！！！！！！！！！*/
		double trial_WO[D] = {-1, -1, -1, -1, -1, -1, -1, -1};
		of << "WO" << endl;
		ths.clear();
		for (int i = 0; i < num_threads; ++i)
			ths.push_back(thread(simulation_trial, trial_WO, &of));
		for (int i = 0; i < num_threads; ++i)
			ths[i].join();

		of.close();

		cout << "Total time : " << time.elapsed() << endl;
		return 0;
	}
	else if (allocation_method == "scene2_arbitrary")
	{
		int num_threads = 30;
		ofstream of("/home/cyx/wayfindmap_allocation_cmake/result/Comparison/difftraffic" + scene_name + "_comparison" + to_string(n_signages), ios::out | ios::trunc);
		/*！！！！！！！！！！！！！！！！Greedy！！！！！！！！！！！！！！！！*/
		double trial_greedy[D];
		vector<ENTRANCE> temp(entrances);
		sort(temp.begin(), temp.end(), cmp2);
		for (int j = 0; j < n_signages; ++j)
		{
			trial_greedy[2 * j] = temp[j].greedy_sign_x;
			trial_greedy[2 * j + 1] = temp[j].greedy_sign_y;
		}
		of << "Greedy" << endl;
		vector<thread> ths;
		for (int i = 0; i < num_threads; ++i)
			ths.push_back(thread(simulation_trial, trial_greedy, &of));
		for (int i = 0; i < num_threads; ++i)
			ths[i].join();

		/*！！！！！！！！！！！！！！！！SD！！！！！！！！！！！！！！！！*/
		double trial_SD[D] = {29, 7, 30, 27, 8, 35, 11, 11};
		of << "SD" << endl;
		ths.clear();
		for (int i = 0; i < num_threads; ++i)
			ths.push_back(thread(simulation_trial, trial_SD, &of));
		for (int i = 0; i < num_threads; ++i)
			ths[i].join();

		/*！！！！！！！！！！！！！！！！WO！！！！！！！！！！！！！！！！*/
		// double trial_WO[D] = {65, 74, 53, 9, 51, 36, 14, 73, 34, 72, 63, 16, 64, 46, 51, 61, 63, 89, 85, 69};
		double trial_WO[D] = {18, 23, 20, 20, -1, -1, -1, -1};
		of << "WO" << endl;
		ths.clear();
		for (int i = 0; i < num_threads; ++i)
			ths.push_back(thread(simulation_trial, trial_WO, &of));
		for (int i = 0; i < num_threads; ++i)
			ths[i].join();

		of.close();

		cout << "Total time : " << time.elapsed() << endl;
		return 0;
	}
	else if (allocation_method == "scene3_arbitrary")
	{
		int num_threads = 30;
		ofstream of("/home/cyx/wayfindmap_allocation_cmake/result/Comparison/scnen3_comparison" + to_string(n_signages), ios::out | ios::trunc);
		/*！！！！！！！！！！！！！！！！Greedy！！！！！！！！！！！！！！！！*/
		double trial_greedy[D];
		vector<ENTRANCE> temp(entrances);
		sort(temp.begin(), temp.end(), cmp2);
		for (int j = 0; j < n_signages; ++j)
		{
			trial_greedy[2 * j] = temp[j].greedy_sign_x;
			trial_greedy[2 * j + 1] = temp[j].greedy_sign_y;
		}

		of << "Greedy" << endl;
		vector<thread> ths;
		for (int i = 0; i < num_threads; ++i)
			ths.push_back(thread(simulation_trial, trial_greedy, &of));
		for (int i = 0; i < num_threads; ++i)
			ths[i].join();

		/*！！！！！！！！！！！！！！！！SD！！！！！！！！！！！！！！！！*/
		double trial_SD[D]{51,10,65,74,35,71,14,72,51,61,91,65,53,39};
		// double trial_SD[D]{52, 61, 14, 72, 35, 71, 91, 65, 61, 89, 51, 10, 66, 74, 53, 39, 65, 48, 64, 16};
		// double trial_SD[D] = {94, 62, 51, 39, 61, 89, 53, 61, 81, 71, 23, 61, 64, 75, 15, 67, 59, 23, 65, 48, 13, 88, 81, 78, 34, 77, 68, 66, 65, 16, 37, 70, 52, 6};
		of << "SD" << endl;
		ths.clear();
		for (int i = 0; i < num_threads; ++i)
			ths.push_back(thread(simulation_trial, trial_SD, &of));
		for (int i = 0; i < num_threads; ++i)
			ths[i].join();

		/*！！！！！！！！！！！！！！！！WO！！！！！！！！！！！！！！！！*/
		double trial_WO[D] = {12,87,13,62,52,7,63,18,64,47,82,70,63,87};
		//10 double trial_WO[D] = {13, 62, 24, 72, 41, 70, 52, 7, 52, 62, 52, 67, 63, 18, 64, 47, 77, 64, 102, 62};
		//17 double trial_WO[D] = {37,73,12,87,13,62,16,70,24,72,41,70,47,73,52,7,52,27,54,35,54,55,57,34,57,7,64,47,77,64,87,73,102,62};
		of << "WO" << endl;
		ths.clear();
		for (int i = 0; i < num_threads; ++i)
			ths.push_back(thread(simulation_trial, trial_WO, &of));
		for (int i = 0; i < num_threads; ++i)
			ths[i].join();

		of.close();

		cout << "Total time : " << time.elapsed() << endl;
		return 0;
	}
	else if (allocation_method == "scene1_arbitrary_ranksum")
	{
		// 1000 simulations
		int num_simulations = 1000;
		int num_threads = 30;
		ofstream of("/home/cyx/wayfindmap_allocation_cmake/result/Comparison/diff_scene1_comparison_1000ranksum" + to_string(n_signages), ios::out | ios::trunc);
		/*！！！！！！！！！！！！！！！！Greedy！！！！！！！！！！！！！！！！*/
		double trial_greedy[D] = {};
		vector<ENTRANCE> temp(entrances);
		sort(temp.begin(), temp.end(), cmp2);
		for (int j = 0; j < n_signages; ++j)
		{
			trial_greedy[2 * j] = temp[j].greedy_sign_x;
			trial_greedy[2 * j + 1] = temp[j].greedy_sign_y;
		}
		of << "Greedy" << endl;
		int n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_trial, trial_greedy, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		/*！！！！！！！！！！！！！！！！SD！！！！！！！！！！！！！！！！*/
		double trial_SD[D] = {30, 9, 9, 32, 29, 32, 7, 12};
		of << "SD" << endl;
		n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_trial, trial_SD, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		/*！！！！！！！！！！！！！！！！WO！！！！！！！！！！！！！！！！*/
		double trial_WO[D] = {-1, -1, -1, -1, -1, -1, -1, -1};
		of << "WO" << endl;
		n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_trial, trial_WO, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		/*！！！！！！！！！！！！！！！！Selected solution！！！！！！！！！！！！！！！！*/
		//double trial_OPT[D] = {32.0322,19.5444,17.7766,7.04481,19.1751,33.4295,8.14856,19.0594}; //same
		double trial_OPT[D] = { 17.0791,9.42188,8.3374,24.6521,25.4761,30.0098,23.4268,9.01416 }; //diff
		of << "OPT" << endl;
		n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_trial, trial_OPT, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		of.close();

		cout << "Total time : " << time.elapsed() << endl;
		return 0;
	}
	else if (allocation_method == "scene2_arbitrary_ranksum")
	{
		// 1000 simulations
		int num_simulations = 1000;
		int num_threads = 30;
		ofstream of("/home/cyx/wayfindmap_allocation_cmake/result/Comparison/diff_scnen2_comparison_1000ranksum" + to_string(n_signages), ios::out | ios::trunc);
		/*！！！！！！！！！！！！！！！！Greedy！！！！！！！！！！！！！！！！*/
		double trial_greedy[D] = {};
		vector<ENTRANCE> temp(entrances);
		sort(temp.begin(), temp.end(), cmp2);
		for (int j = 0; j < n_signages; ++j)
		{
			trial_greedy[2 * j] = temp[j].greedy_sign_x;
			trial_greedy[2 * j + 1] = temp[j].greedy_sign_y;
		}
		of << "Greedy" << endl;
		int n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_trial, trial_greedy, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		/*！！！！！！！！！！！！！！！！SD！！！！！！！！！！！！！！！！*/
		double trial_SD[D] = {29, 7, 30, 27, 8, 35, 11, 11};
		of << "SD" << endl;
		n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_trial, trial_SD, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		/*！！！！！！！！！！！！！！！！WO！！！！！！！！！！！！！！！！*/
		double trial_WO[D] = {18, 23, 20, 20, -1, -1, -1, -1};
		of << "WO" << endl;
		n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_trial, trial_WO, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		/*！！！！！！！！！！！！！！！！Selected solution！！！！！！！！！！！！！！！！*/
		//double trial_OPT[D] = {15.3075,19.0751,21.877,29.9824,19.2014,11.3929,26.2174,20.7749}; //same
		double trial_OPT[D] =  {22.2553,15.4151,16.6,20.6482,22.6758,22.9531,19.5,12}; //diff
		of << "OPT" << endl;
		n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_trial, trial_OPT, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		of.close();

		cout << "Total time : " << time.elapsed() << endl;
		return 0;
	}
	else if (allocation_method == "scene3_arbitrary_ranksum")
	{
		// 1000 simulations
		int num_simulations = 1000;
		int num_threads = 30;
		ofstream of("/home/cyx/wayfindmap_allocation_cmake/result/Comparison/same_scnen3_comparison_1000ranksum" + to_string(n_signages), ios::out | ios::trunc);
		/*！！！！！！！！！！！！！！！！Greedy！！！！！！！！！！！！！！！！*/
		double trial_greedy[D] = {};
		vector<ENTRANCE> temp(entrances);
		sort(temp.begin(), temp.end(), cmp2);
		for (int j = 0; j < n_signages; ++j)
		{
			trial_greedy[2 * j] = temp[j].greedy_sign_x;
			trial_greedy[2 * j + 1] = temp[j].greedy_sign_y;
		}
		of << "Greedy" << endl;
		int n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_trial, trial_greedy, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		/*！！！！！！！！！！！！！！！！SD！！！！！！！！！！！！！！！！*/
		double trial_SD[D] = {51,10,65,74,35,71,14,72,51,61,91,65,53,39};
		// double trial_SD[D]{52, 61, 14, 72, 35, 71, 91, 65, 61, 89, 51, 10, 66, 74, 53, 39, 65, 48, 64, 16}; //10
		// double trial_SD[D] = {94, 62, 51, 39, 61, 89, 53, 61, 81, 71, 23, 61, 64, 75, 15, 67, 59, 23, 65, 48, 13, 88, 81, 78, 34, 77, 68, 66, 65, 16, 37, 70, 52, 6}; //17
		of << "SD" << endl;
		n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_trial, trial_SD, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		/*！！！！！！！！！！！！！！！！WO！！！！！！！！！！！！！！！！*/
		double trial_WO[D] = {12,87,13,62,52,7,63,18,64,47,82,70,63,87};
		// double trial_WO[D] = {13, 62, 24, 72, 41, 70, 52, 7, 52, 62, 52, 67, 63, 18, 64, 47, 77, 64, 102, 62}; //10
		// double trial_WO[D] = {37,73,12,87,13,62,16,70,24,72,41,70,47,73,52,7,52,27,54,35,54,55,57,34,57,7,64,47,77,64,87,73,102,62}; //17
		of << "WO" << endl;
		n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_trial, trial_WO, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		/*！！！！！！！！！！！！！！！！Selected solution！！！！！！！！！！！！！！！！*/
		double trial_OPT[D] = {49,61,22,79,43.625,69.4375,58,51.5,87,70,55,89,64,3}; //same
		//double trial_OPT[D] = {43,61,52.5469,51.6211,61.5,86,59,78,56,89,84,61,55.1226,73.7109};//diff
		of << "OPT" << endl;
		n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_trial, trial_OPT, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		of.close();

		cout << "Total time : " << time.elapsed() << endl;
		return 0;
	}
	else if (allocation_method == "recordPressure")
	{
		// 1000 simulations
		int num_simulations = 1000;
		int num_threads = 30;
		ofstream of("/home/cyx/wayfindmap_allocation_cmake/result/indicators_persecond/diff" + scene_name + "_" + to_string(n_signages) + "avg_max_pre.csv", ios::out | ios::trunc);

		/*！！！！！！！！！！！！！！！！Greedy！！！！！！！！！！！！！！！！*/
		double trial_greedy[D] = {};
		vector<ENTRANCE> temp(entrances);
		sort(temp.begin(), temp.end(), cmp2);
		for (int j = 0; j < n_signages; ++j)
		{
			trial_greedy[2 * j] = temp[j].greedy_sign_x;
			trial_greedy[2 * j + 1] = temp[j].greedy_sign_y;
		}
		of << "Greedy" << endl;
		int n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_recordPre, trial_greedy, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}
		/*！！！！！！！！！！！！！！！！SD！！！！！！！！！！！！！！！！*/
		double trial_SD[D] = {51,10,65,74,35,71,14,72,51,61,91,65,53,39};
		// double trial_SD[D]{52, 61, 14, 72, 35, 71, 91, 65, 61, 89, 51, 10, 66, 74, 53, 39, 65, 48, 64, 16};
		// double trial_SD[D] = {94, 62, 51, 39, 61, 89, 53, 61, 81, 71, 23, 61, 64, 75, 15, 67, 59, 23, 65, 48, 13, 88, 81, 78, 34, 77, 68, 66, 65, 16, 37, 70, 52, 6};
		of << "SD" << endl;
		n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_recordPre, trial_SD, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		/*！！！！！！！！！！！！！！！！WO！！！！！！！！！！！！！！！！*/
		double trial_WO[D] = {12,87,13,62,52,7,63,18,64,47,82,70,63,87};
		// double trial_WO[D] = {13, 62, 24, 72, 41, 70, 52, 7, 52, 62, 52, 67, 63, 18, 64, 47, 77, 64, 102, 62};
		// double trial_WO[D] = {37,73,12,87,13,62,16,70,24,72,41,70,47,73,52,7,52,27,54,35,54,55,57,34,57,7,64,47,77,64,87,73,102,62};
		of << "WO" << endl;
		n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_recordPre, trial_WO, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		/*！！！！！！！！！！！！！！！！Selected solution！！！！！！！！！！！！！！！！*/
		
		double trial_OPT[D] = {25,79,89.25,66,52,7,64,48.25,52.5,46.5,66.9375,88.0938,56,89 }; //same
		//double trial_OPT[D] = { 43,61,52.5469,51.6211,61.5,86,59,78,56,89,84,61,55.1226,73.7109};//diff
		of << "OPT" << endl;
		n = 0;
		while (n != num_simulations)
		{
			vector<thread> ths;
			num_threads = num_simulations - n < 30 ? num_simulations - n : 30;
			for (int i = 0; i < num_threads; ++i)
			{
				ths.push_back(thread(simulation_recordPre, trial_OPT, &of));
				n++;
			}
			for (int i = 0; i < num_threads; ++i)
			{
				ths[i].join();
			}
		}

		of.close();

		cout << "Total time : " << time.elapsed() << endl;
		return 0;
	}
	else if( allocation_method=="safety")
	{
		vector<AGENT> agents{5};
		agents[0].x = 10; agents[0].y = 10;
		agents[0].vx = 0; agents[0].vy = 1;

		if(1){
			//s1
			{
				agents[1].x = 10; agents[1].y = 10.5;
				agents[1].vx = 0; agents[1].vy = 1;
				agents[2].x = 10.5; agents[2].y = 10;
				agents[2].vx = 0; agents[2].vy = 1;
				agents[3].x = 10; agents[3].y = 9.5;
				agents[3].vx = 0; agents[3].vy = 1;
				agents[4].x = 9.5; agents[4].y = 10;
				agents[4].vx = 0; agents[4].vy = 1;
			}
		}else{
			//s2
			{
				agents[1].x = 10; agents[1].y = 10.5;
				agents[1].vx = 0; agents[1].vy = -1;
				agents[2].x = 10.5; agents[2].y = 10;
				agents[2].vx = -1; agents[2].vy = 0;
				agents[3].x = 10; agents[3].y = 9.5;
				agents[3].vx = 0; agents[3].vy = 1;
				agents[4].x = 9.5; agents[4].y = 10;
				agents[4].vx = 1; agents[4].vy = 0;
			}
		}

		// if(0){
		// 	//s1
		// 	{
		// 		agents[1].x = 10; agents[1].y = 10.5;
		// 		agents[1].vx = 0; agents[1].vy = -1;
		// 		agents[2].x = 10.5; agents[2].y = 10;
		// 		agents[2].vx = -1; agents[2].vy = 0;
		// 		agents[3].x = 10; agents[3].y = 9.5;
		// 		agents[3].vx = 0; agents[3].vy = 1;
		// 		agents[4].x = 9.5; agents[4].y = 10;
		// 		agents[4].vx = 1; agents[4].vy = 0;
		// 	}
		// }else{
		// 	//s2
		// 	{
		// 		agents[1].x = 10; agents[1].y = 10.2;
		// 		agents[1].vx = 0; agents[1].vy = -1;
		// 		agents[2].x = 10.2; agents[2].y = 10;
		// 		agents[2].vx = -1; agents[2].vy = 0;
		// 		agents[3].x = 10; agents[3].y = 9.8;
		// 		agents[3].vx = 0; agents[3].vy = 1;
		// 		agents[4].x = 9.8; agents[4].y = 10;
		// 		agents[4].vx = 1; agents[4].vy = 0;
		// 	}
		// }	

		// if(0){
		// 	//s1
		// 	{
		// 		agents[1].x = 10; agents[1].y = 10.2;
		// 		agents[1].vx = 0; agents[1].vy = 1;
		// 		agents[2].x = 10.2; agents[2].y = 10;
		// 		agents[2].vx = 0; agents[2].vy = 1;
		// 		agents[3].x = 10; agents[3].y = 9.8;
		// 		agents[3].vx = 0; agents[3].vy = 1;
		// 		agents[4].x = 9.8; agents[4].y = 10;
		// 		agents[4].vx = 0; agents[4].vy = 1;
		// 	}
		// }else{
		// 	//s2
		// 	{
		// 		agents[1].x = 10; agents[1].y = 10.2;
		// 		agents[1].vx = 0; agents[1].vy = -1;
		// 		agents[2].x = 10.2; agents[2].y = 10;
		// 		agents[2].vx = -1; agents[2].vy = 0;
		// 		agents[3].x = 10; agents[3].y = 9.8;
		// 		agents[3].vx = 0; agents[3].vy = 1;
		// 		agents[4].x = 9.8; agents[4].y = 10;
		// 		agents[4].vx = 1; agents[4].vy = 0;
		// 	}
		// }

		//interaction force
		AGENT *cur_agent = &agents[0];
		double total_f = 0;
		double total_fx = 0;
		double total_fy = 0;
		{
		
			for (size_t j = 0; j < agents.size(); ++j)
			{
				if (0 == j)
					continue;
				if (agents.size() == 1)
					break;
				AGENT *other_agent = &agents[j];
				double dis = (cur_agent->x - other_agent->x) * (cur_agent->x - other_agent->x) + (cur_agent->y - other_agent->y) * (cur_agent->y - other_agent->y); // ????agant??????
				if (dis > sense_range)
					continue;                                                     
				double d = sqrt((cur_agent->x - other_agent->x) * (cur_agent->x - other_agent->x) + (cur_agent->y - other_agent->y) * (cur_agent->y - other_agent->y)); // ????agant??????
				if (d == 0)
				{
					printf("here d == 0 error but fixed...");
					d = 1e-10;
				}
				cur_agent->m = other_agent->m = 80;
				double delta_d = cur_agent->m / c_mass + other_agent->m / c_mass - d;
				double fexp = A * exp(delta_d / B);
				double fkg = delta_d < 0 ? 0 : k1 * delta_d;
				double nijx = (cur_agent->x - other_agent->x) / d;
				double nijy = (cur_agent->y - other_agent->y) / d;
				double fnijx = (fexp + fkg) * nijx;
				double fnijy = (fexp + fkg) * nijy;


				// ????????
				double fkgx = 0;
				double fkgy = 0;
				if (delta_d > 0)
				{
					double tix = -nijy;
					double tiy = nijx;
					fkgx = k2 * delta_d;
					fkgy = k2 * delta_d;
					double delta_vij = (other_agent->vx - cur_agent->vx) * tix + (other_agent->vy - cur_agent->vy) * tiy;
					fkgx = fkgx * delta_vij * tix;
					fkgy = fkgy * delta_vij * tiy;
				}
				total_fx += fabs(fnijx + fkgx);
				total_fy += fabs(fnijy + fkgy);
			}
			total_f = sqrt(total_fx*total_fx+total_fy*total_fy);

		}

		//entropy
		double Edmax{},Evmax{};
		double Ed = 0;
		double Ev = 0;
		{
			double R = 0.7;
			int local_density = 0; 
			int n1 = 20; int n2 = 36;
			int panel_angle[n2 + 1] = {0};   // n2 = 36
			int panel_veloity[n1 + 1] = {0}; // n1 = 20
			int j=0;
			for (size_t k = 0; k < agents.size(); k++)
			{
				double dist = sqrt((agents[j].x - agents[k].x) * (agents[j].x - agents[k].x) + (agents[j].y - agents[k].y) * (agents[j].y - agents[k].y));
				if (dist <= R)
				{ // ???????????
					// ????
					local_density++;
					double Wp_x = agents[k].vx;
					double Wp_y = agents[k].vy;
					double Gp_x = 1;
					double Gp_y = 0;
					double WpGp = Wp_x * Gp_x + Wp_y * Gp_y;
					double Wp_mol = sqrt(Wp_x * Wp_x + Wp_y * Wp_y);
					double Gp_mol = sqrt(Gp_x * Gp_x + Gp_y * Gp_y);
					double theta = acos(WpGp / (Wp_mol * Gp_mol));
					if (agents[k].vy < 0)
						theta = 2 * PI - theta;
					if (agents[k].y < 0 || agents[k].y > Height || agents[k].x < 0 || agents[k].x > Width)
						continue;
					panel_angle[(int)(theta / (2 * PI / (double)n2))] += 1;
					// std::cout << agents[k].id<<' '<<(int)(theta / (2 * PI / (double)n2)) << endl;
					double velo = sqrt(agents[k].vx * agents[k].vx + agents[k].vy * agents[k].vy);
					if (velo > 2)
						velo = 2; 
					panel_veloity[(int)(velo / (2.0 / (double)n1))] += 1;
					// std::cout << agents[k].id<<' '<< (int)(velo / (5.0 / (double)n1)) << endl;
				}
			}

			for (int ii = 0; ii < n2; ++ii)
			{
				if (panel_angle[ii] == 0)
					continue;
				Ed += -((double)panel_angle[ii] / (double)local_density) * log2((double)panel_angle[ii] / (double)local_density);

			}
			for (int ii = 0; ii < n1; ++ii)
			{
				if (panel_veloity[ii] == 0)
					continue;
				Ev += -((double)panel_veloity[ii] / (double)local_density) * log2((double)panel_veloity[ii] / (double)local_density);
			}

			for (int ii = 0; ii < n2; ++ii)
			{
				Edmax = (double)log2(n2);

			}
			for (int ii = 0; ii < n1; ++ii)
			{
				Evmax = (double)log2(n1);
			}
			
		}

		cout<<total_f<<endl;
		cout<<Ed<<' '<<Ev<<' '<<Ed*Ev<<endl;
		cout<<Edmax<<' '<<Evmax<<endl;

		return 0;
	}
}

