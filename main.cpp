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

	string allocation_method = "ibea";

	if (allocation_method == "ibea")
	{
		IbeaSolver solver(50, 2 * n_signages, 300, 30); // 嶽蛤方、倖悶略業、恷寄亨旗方、耽倖盾議得浩肝方
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

		double trial[D] = {23.5,66.5,57.4,18,57.4,37,57.4,57,57.4,77,77,69.5,38.5,66.5,38.5,73.5,31,70,23.5,73.5};

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
		ofstream of("/home/cyx/wayfindmap_allocation_cmake/result/Comparison/" + scene_name + "_comparison" + to_string(n_signages), ios::out | ios::trunc);
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
		ofstream of("/home/cyx/wayfindmap_allocation_cmake/result/Comparison/" + scene_name + "_comparison" + to_string(n_signages), ios::out | ios::trunc);
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
		double trial_SD[D]{52, 61, 14, 72, 35, 71, 91, 65, 61, 89, 51, 10, 66, 74, 53, 39, 65, 48, 64, 16};
		// double trial_SD[D] = {94, 62, 51, 39, 61, 89, 53, 61, 81, 71, 23, 61, 64, 75, 15, 67, 59, 23, 65, 48, 13, 88, 81, 78, 34, 77, 68, 66, 65, 16, 37, 70, 52, 6};
		of << "SD" << endl;
		ths.clear();
		for (int i = 0; i < num_threads; ++i)
			ths.push_back(thread(simulation_trial, trial_SD, &of));
		for (int i = 0; i < num_threads; ++i)
			ths[i].join();

		/*！！！！！！！！！！！！！！！！WO！！！！！！！！！！！！！！！！*/
		double trial_WO[D] = {13, 62, 24, 72, 41, 70, 52, 7, 52, 62, 52, 67, 63, 18, 64, 47, 77, 64, 102, 62};
		// double trial_WO[D] = {37,73,12,87,13,62,16,70,24,72,41,70,47,73,52,7,52,27,54,35,54,55,57,34,57,7,64,47,77,64,87,73,102,62};
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
		ofstream of("/home/cyx/wayfindmap_allocation_cmake/result/Comparison/scnen1_comparison_1000ranksum" + to_string(n_signages), ios::out | ios::trunc);
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
		double trial_OPT[D] = {17.9341, 11.3176, 6, 20, 10.2354, 5.7783, 20, 32.525};
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
		ofstream of("/home/cyx/wayfindmap_allocation_cmake/result/Comparison/scnen2_comparison_1000ranksum" + to_string(n_signages), ios::out | ios::trunc);
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
		double trial_OPT[D] = {33.8881, 19.3932, 14.388, 25.3328, 11.7037, 7.4465, 21.7801, 33.81};
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
		ofstream of("/home/cyx/wayfindmap_allocation_cmake/result/Comparison/scnen3_comparison_1000ranksum" + to_string(n_signages), ios::out | ios::trunc);
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
		double trial_SD[D]{52, 61, 14, 72, 35, 71, 91, 65, 61, 89, 51, 10, 66, 74, 53, 39, 65, 48, 64, 16};
		// double trial_SD[D] = {94, 62, 51, 39, 61, 89, 53, 61, 81, 71, 23, 61, 64, 75, 15, 67, 59, 23, 65, 48, 13, 88, 81, 78, 34, 77, 68, 66, 65, 16, 37, 70, 52, 6};
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
		double trial_WO[D] = {13, 62, 24, 72, 41, 70, 52, 7, 52, 62, 52, 67, 63, 18, 64, 47, 77, 64, 102, 62};
		// double trial_WO[D] = {37,73,12,87,13,62,16,70,24,72,41,70,47,73,52,7,52,27,54,35,54,55,57,34,57,7,64,47,77,64,87,73,102,62};
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
		double trial_OPT[D] = {47.1655, 64.1277, 102, 64, 17, 79, 100, 61, 31.2643, 61, 63.7, 66.8, 53.2, 51.604, 64.6, 60.2, 34, 66, 35, 61};
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
		ofstream of("/home/cyx/wayfindmap_allocation_cmake/result/indicators_persecond/" + scene_name + "_" + to_string(n_signages) + "avg_max_pre.csv", ios::out | ios::trunc);
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
		double trial_SD[D]{52, 61, 14, 72, 35, 71, 91, 65, 61, 89, 51, 10, 66, 74, 53, 39, 65, 48, 64, 16};
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
		double trial_WO[D] = {13, 62, 24, 72, 41, 70, 52, 7, 52, 62, 52, 67, 63, 18, 64, 47, 77, 64, 102, 62};
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
		double trial_OPT[D] = {47.1655, 64.1277, 102, 64, 17, 79, 100, 61, 31.2643, 61, 63.7, 66.8, 53.2, 51.604, 64.6, 60.2, 34, 66, 35, 61};
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
	else
	{
		// testcode
		return 0;
	}
}

