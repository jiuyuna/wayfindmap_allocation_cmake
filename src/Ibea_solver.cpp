#include "Ibea_solver.h"
#include "Util.h"
#include <iostream>
#include <random>
#include <vector>
#include <string>
#include <cstring>

using namespace std;

IbeaSolver::IbeaSolver(int pop_size, int ind_size, int max_gens, int num_eva) : popsize{pop_size}, indsize{ind_size}, max_generations{max_gens}, eva_num{num_eva}, n_signages{ind_size / 2}
{

    outfile = new ofstream("/home/cyx/wayfindmap_allocation_cmake/result/IBEA/newrndwalk_300people_sametraffic_bound" + scene_name + "_" + to_string(n_signages) + "signs_" + to_string(popsize) + "popsize_res", ios::out);
    // outfile = new ofstream("/home/cyx/wayfindmap_allocation_cmake/result/IBEA/demo.csv", ios::out);
    parent_pop = new Population(popsize, indsize);
    offspring_pop = new Population(popsize, indsize);
    mixed_pop = new Population(2 * popsize, indsize);

    initialize_population(parent_pop);
    evaluate_population(parent_pop);
    track_evolution(0);

    Indicator_value = vector<vector<double>>(2 * popsize, vector<double>(2 * popsize, 0.0));
    flag = vector<int>(2 * popsize, 0);
}

IbeaSolver::~IbeaSolver()
{
    // ɾ���ռ�������BUG,����������ָ��ָ��ռ���ϵͳ�Զ����վ��У����˴��š�
    // delete parent_pop;
    // delete offspring_pop;
    // delete mixed_pop;
    outfile->close();
    delete outfile;
    outfile = nullptr;
}

void IbeaSolver::initialize_population(Population *pop)
{
    // initialization with Prior knowledge - three solutions generated by other methods
    if (scene_name == "scene1")
    {
        double greedy[maxvar] = {20, 2, 38, 20, 20, 38, 2, 20};
        memcpy(pop->ind[0]->xreal, greedy, 2 * n_signages * sizeof(double));
        double SD[maxvar] = {30, 9, 9, 32, 29, 32, 7, 12};
        memcpy(pop->ind[1]->xreal, SD, 2 * n_signages * sizeof(double));
        double WO[maxvar] = {-1, -1, -1, -1, -1, -1, -1, -1};
        for (int i = 0; i < 2 * n_signages; i += 2)
        {
            int rnd = (int)randval(0, (int)available_points.size());
            while (rnd == (int)available_points.size())
                rnd = (int)randval(0, available_points.size());
            WO[i] = available_points[rnd].X;
            WO[i + 1] = available_points[rnd].Y;
        }
        memcpy(pop->ind[2]->xreal, WO, 2 * n_signages * sizeof(double));
    }
    else if (scene_name == "scene2")
    {
        double greedy[maxvar] = {20, 2, 38, 20, 20, 38, 2, 20};
        memcpy(pop->ind[0]->xreal, greedy, 2 * n_signages * sizeof(double));
        double SD[maxvar] = {29, 7, 30, 27, 8, 35, 11, 11};
        memcpy(pop->ind[1]->xreal, SD, 2 * n_signages * sizeof(double));
        double WO[maxvar] = {18, 23, 20, 20, -1, -1, -1, -1};
        for (int i = 4; i < 2 * n_signages; i += 2)
        {
            int rnd = (int)randval(0, (int)available_points.size());
            while (rnd == (int)available_points.size())
                rnd = (int)randval(0, available_points.size());
            WO[i] = available_points[rnd].X;
            WO[i + 1] = available_points[rnd].Y;
        }
        memcpy(pop->ind[2]->xreal, WO, 2 * n_signages * sizeof(double));
    }
    else if (scene_name == "scene3")
    {
        //7outer
        double greedy[maxvar] = {12.5,57,68,87.5,98,62.5,70,47.5,68,17.5,47,7.5,3,87.5};
        for (int i = 2*7; i < 2 * n_signages; i += 2)
        {
            int rnd = (int)randval(0, (int)available_points.size());
            while (rnd == (int)available_points.size())
                rnd = (int)randval(0, available_points.size());
            greedy[i] = available_points[rnd].X;
            greedy[i + 1] = available_points[rnd].Y;
        }
        memcpy(pop->ind[0]->xreal, greedy, 2 * n_signages * sizeof(double));
        double SD[maxvar] = {51,10,65,74,35,71,14,72,51,61,91,65,53,39};
        memcpy(pop->ind[1]->xreal, SD, 2 * n_signages * sizeof(double));
        double WO[maxvar] = {12,87,13,62,52,7,63,18,64,47,82,70,63,87};
        memcpy(pop->ind[2]->xreal, WO, 2 * n_signages * sizeof(double));
    }
    for (int i = 3; i < popsize; ++i)
    {
        Individual *ind_cur = pop->ind[i];
        for (int j = 0; j < ind_cur->indsize; j += 2)
        {
            int rnd = (int)randval(0, (int)available_points.size());
            while (rnd == (int)available_points.size())
                rnd = (int)randval(0, available_points.size());
            ind_cur->xreal[j] = available_points[rnd].X;
            ind_cur->xreal[j + 1] = available_points[rnd].Y;
        }
    };
}

#define F 0.5  // F���Ʋ�ֱ仯�ķŴ�[0,2]
#define CR 0.7 // ���泣�� [0,1]

// test crossing variables code
// int n_c = 0;
// int n_regenerate = 0;

void regeneration(Individual *ind){
    for (int j = 0; j < ind->indsize; j += 2)
    {
        int rnd = (int)randval(0, (int)available_points.size());
        while (rnd == (int)available_points.size())
            rnd = (int)randval(0, available_points.size());
        ind->xreal[j] = available_points[rnd].X;
        ind->xreal[j + 1] = available_points[rnd].Y;
    }
}

void IbeaSolver::crossover_mutation()
{
    // //DE
    // for (int i = 0; i < popsize; ++i) {
    //     //�������r1,r2,r3����Ӧ�����Ϊr1,r2,r3���������壬r1,r2,r3,i������ͬ
    //     int r1, r2, r3;
    //     r1 = r2 = r3 = 0;
    //     do {
    //         r1 = random(0, popsize);
    //     } while (r1 == i);
    //     do {
    //         r2 = random(0, popsize);
    //     } while (r2 == i || r2 == r1);
    //     do {
    //         r3 = random(0, popsize);
    //     } while (r3 == i || r3 == r2 || r3 == r1);
    //     int j = random(0, indsize);
    //     while(j == indsize) j = random(0, indsize);
    //     for (int k = 1; k <= indsize; ++k) {
    //         if (random01() < CR || k == indsize) { //k==Dȷ�����ٵõ�һ������
    //             offspring_pop->ind[i]->xreal[j] = parent_pop->ind[r3]->xreal[j] + F * (parent_pop->ind[r1]->xreal[j] - parent_pop->ind[r2]->xreal[j]);  //�������
    //         }
    //         else {
    //             offspring_pop->ind[i]->xreal[j] = parent_pop->ind[i]->xreal[j];
    //         }
    //         j = (j + 1) % popsize;
    //     }
    //     //Խ�紦��������[0,1]
    //     for (int k = 0; k < indsize; ++k) {
    //         if (offspring_pop->ind[i]->xreal[k] < 0) offspring_pop->ind[i]->xreal[k] = 0;
    //         if (offspring_pop->ind[i]->xreal[k] > 1) offspring_pop->ind[i]->xreal[k] = 1;
    //     }
    // }

    // DE_for_ģ��
    for (int i = 0; i < popsize; ++i)
    {
        // �������r1,r2,r3����Ӧ�����Ϊr1,r2,r3���������壬r1,r2,r3,i������ͬ
        int r1, r2, r3;
        do
        {
            r1 = random(0, popsize);
        } while (r1 == i);
        do
        {
            r2 = random(0, popsize);
        } while (r2 == i || r2 == r1);
        do
        {
            r3 = random(0, popsize);
        } while (r3 == i || r3 == r2 || r3 == r1);

        int j = random(0, n_signages);
        while (j == n_signages)
            j = random(0, n_signages);

        for (int k = 1; k <= n_signages; ++k)
        {

            if (random01() < CR || k == n_signages)
            {                                                                                                                                                      // k==Dȷ�����ٵõ�һ������
                offspring_pop->ind[i]->xreal[2 * j] = parent_pop->ind[r3]->xreal[j] + F * (parent_pop->ind[r1]->xreal[2 * j] - parent_pop->ind[r2]->xreal[2 * j]); // �������
                offspring_pop->ind[i]->xreal[2 * j + 1] = parent_pop->ind[r3]->xreal[2 * j + 1] + F * (parent_pop->ind[r1]->xreal[2 * j + 1] - parent_pop->ind[r2]->xreal[2 * j + 1]);
                // cout<<"difference between x: "<<(parent_pop->ind[r1]->xreal[2 * j] - parent_pop->ind[r2]->xreal[2 * j])<<endl;
                // cout<<"difference between y: "<<(parent_pop->ind[r1]->xreal[2 * j + 1] - parent_pop->ind[r2]->xreal[2 * j + 1])<<endl;
                // n_c++;
            }
            else
            {
                offspring_pop->ind[i]->xreal[2 * j] = parent_pop->ind[i]->xreal[2 * j];
                offspring_pop->ind[i]->xreal[2 * j + 1] = parent_pop->ind[i]->xreal[2 * j + 1];
            }

            // double temp_x = offspring_pop->ind[i]->xreal[2 * j];
            // double temp_y = offspring_pop->ind[i]->xreal[2 * j+ 1 ];

            if (scene_name == "scene3")
            {
                // Խ�紦��
                {
                    // x
                    if (offspring_pop->ind[i]->xreal[2 * j] < 1)
                        offspring_pop->ind[i]->xreal[2 * j] = 1;
                    if (offspring_pop->ind[i]->xreal[2 * j] > 104) // ȷ��0<x<1050
                        offspring_pop->ind[i]->xreal[2 * j] = 104;
                    // y
                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 0 && offspring_pop->ind[i]->xreal[2 * j] < 10) && (offspring_pop->ind[i]->xreal[2 * j + 1] < 85))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 85;
                    }
                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 0 && offspring_pop->ind[i]->xreal[2 * j] < 10) && (offspring_pop->ind[i]->xreal[2 * j + 1] > 90))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 90;
                    }

                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 10 && offspring_pop->ind[i]->xreal[2 * j] < 15) && (offspring_pop->ind[i]->xreal[2 * j + 1] < 50))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 50;
                    }
                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 10 && offspring_pop->ind[i]->xreal[2 * j] < 15) && (offspring_pop->ind[i]->xreal[2 * j + 1] > 90))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 90;
                    }

                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 15 && offspring_pop->ind[i]->xreal[2 * j] < 50) && (offspring_pop->ind[i]->xreal[2 * j + 1] < 60 && offspring_pop->ind[i]->xreal[2 * j + 1] > 10))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 60;
                    }
                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 15 && offspring_pop->ind[i]->xreal[2 * j] < 40) && (offspring_pop->ind[i]->xreal[2 * j + 1] < 60))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 60;
                    }
                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 15 && offspring_pop->ind[i]->xreal[2 * j] < 50) && (offspring_pop->ind[i]->xreal[2 * j + 1] > 80))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 80;
                    }

                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 40 && offspring_pop->ind[i]->xreal[2 * j] < 50) && (offspring_pop->ind[i]->xreal[2 * j + 1] < 5))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 5;
                    }

                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 50 && offspring_pop->ind[i]->xreal[2 * j] < 65) && (offspring_pop->ind[i]->xreal[2 * j + 1] < 0))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 0;
                    }
                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 50 && offspring_pop->ind[i]->xreal[2 * j] < 65) && (offspring_pop->ind[i]->xreal[2 * j + 1] > 90))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 90;
                    }

                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 65 && offspring_pop->ind[i]->xreal[2 * j] < 75) && (offspring_pop->ind[i]->xreal[2 * j + 1] > 90))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 90;
                    }
                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 65 && offspring_pop->ind[i]->xreal[2 * j] < 75) && (offspring_pop->ind[i]->xreal[2 * j + 1] < 85 && offspring_pop->ind[i]->xreal[2 * j + 1] > 80))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 85;
                    }
                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 65 && offspring_pop->ind[i]->xreal[2 * j] < 75) && (offspring_pop->ind[i]->xreal[2 * j + 1] < 60 && offspring_pop->ind[i]->xreal[2 * j + 1] > 50))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 60;
                    }
                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 65 && offspring_pop->ind[i]->xreal[2 * j] < 75) && (offspring_pop->ind[i]->xreal[2 * j + 1] < 45 && offspring_pop->ind[i]->xreal[2 * j + 1] > 20))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 45;
                    }
                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 65 && offspring_pop->ind[i]->xreal[2 * j] < 75) && (offspring_pop->ind[i]->xreal[2 * j + 1] < 15))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 15;
                    }

                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 75 && offspring_pop->ind[i]->xreal[2 * j] < 95) && (offspring_pop->ind[i]->xreal[2 * j + 1] < 60))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 60;
                    }
                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 75 && offspring_pop->ind[i]->xreal[2 * j] < 95) && (offspring_pop->ind[i]->xreal[2 * j + 1] > 80))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 80;
                    }

                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 95 && offspring_pop->ind[i]->xreal[2 * j] <= 105) && (offspring_pop->ind[i]->xreal[2 * j + 1] < 60))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 60;
                    }
                    if ((offspring_pop->ind[i]->xreal[2 * j] >= 95 && offspring_pop->ind[i]->xreal[2 * j] <= 105) && (offspring_pop->ind[i]->xreal[2 * j + 1] > 65))
                    {
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 65;
                    }
                }
            }
            else if (scene_name == "scene1" || scene_name == "scene2")
            {
                // Խ�紦��
                {
                    // x
                    if (offspring_pop->ind[i]->xreal[2 * j] < 1)
                        offspring_pop->ind[i]->xreal[2 * j] = 1;
                    if (offspring_pop->ind[i]->xreal[2 * j] > 39)
                        offspring_pop->ind[i]->xreal[2 * j] = 39;

                    // y
                    if (offspring_pop->ind[i]->xreal[2 * j + 1] < 1)
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 1;
                    if (offspring_pop->ind[i]->xreal[2 * j + 1] > 39)
                        offspring_pop->ind[i]->xreal[2 * j + 1] = 39;

                    //regeneration
                    //x
                    // if (offspring_pop->ind[i]->xreal[2 * j] < 1 || offspring_pop->ind[i]->xreal[2 * j] > 39)
                    //     regeneration(offspring_pop->ind[i]);
                    // // y
                    // if (offspring_pop->ind[i]->xreal[2 * j + 1] < 1 || offspring_pop->ind[i]->xreal[2 * j + 1] > 39)
                    //     regeneration(offspring_pop->ind[i]);
                }
            }

            // ����y��鵱ǰ�Ƿ�����ڲ��ʺ�λ�ã��ǵĻ����������ɻ������ڸ�����ĳ�㣬�õ�Ҫ���ڿ������ڡ�
            if (myMap[(int)offspring_pop->ind[i]->xreal[2 * j]][(int)offspring_pop->ind[i]->xreal[2 * j + 1]] == 1)
            {
                int r_x = 0;
                int r_y = 0;
                Rectify(offspring_pop->ind[i]->xreal[2 * j], offspring_pop->ind[i]->xreal[2 * j + 1], r_x, r_y);
                offspring_pop->ind[i]->xreal[2 * j] = r_x;
                offspring_pop->ind[i]->xreal[2 * j + 1] = r_y;
            }

            // // determined if coordinates are regenerated
            // if(temp_x != offspring_pop->ind[i]->xreal[2 * j] || temp_y != offspring_pop->ind[i]->xreal[2 * j + 1]){
            //     // cout<<temp_x<<' '<< offspring_pop->ind[i]->xreal[2 * j]<<endl;
            //     // cout<<temp_y<<' '<< offspring_pop->ind[i]->xreal[2 * j+1]<<endl;
            //     n_regenerate++;
            // }

            j = (j + 1) % n_signages;
            if (j >= n_signages)
                break;
        }
    }

    // cout<<n_c<<' '<<n_regenerate2<<endl;
}

// ���Ժ���
void zdt1(Individual *ind)
{
    ind->obj[0] = ind->xreal[0];
    int i;
    double g = 1, sum = 0;
    for (i = 1; i < ind->indsize; i++)
    {
        sum += ind->xreal[i];
    }
    sum += 9 * (sum / (ind->indsize - 1));
    g += sum;
    ind->obj[1] = g * (1 - sqrt(ind->xreal[0] / g));
}

// ����ģ��
void IbeaSolver::simulation(Individual *ind_cur, IbeaSolver *_this)
{
    int n_signages = ind_cur->indsize;
    Simulator sim = Simulator(ind_cur->xreal, n_signages);
    sim.run_simulation();
    // ��Ҫ�������
    _this->mymutex.lock();
    ind_cur->obj[0] += sim.avg_time;
    ind_cur->obj[1] += sim.avg_max_pressure;
    _this->mymutex.unlock();
}

void IbeaSolver::evaluate_population(Population *pop)
{
    for (int i = 0; i < popsize; ++i)
    {
        Individual *ind_cur = pop->ind[i]; // zdt1(ind_cur); //test function
        // simulator
        ind_cur->obj[0] = 0;
        ind_cur->obj[1] = 0;
        // multi-threads
        int num_threads = eva_num;
        vector<thread> ths;
        for (int i = 0; i < num_threads; ++i)
        {
            ths.push_back(thread(simulation, ind_cur, this));
        }
        for (int i = 0; i < num_threads; ++i)
        {
            ths[i].join();
        }
        ind_cur->obj[0] = ind_cur->obj[0] / num_threads;
        ind_cur->obj[1] = ind_cur->obj[1] / num_threads;
    }
}

int cmp_greater(const void *v1, const void *v2)
// this sorts points worsening in the last objective
{
	Individual p = **(Individual**)v1;
	Individual q = **(Individual**)v2;
	for (int i = 2 - 1; i >= 0; i--) {
		if(q.obj[i]!=p.obj[i])
			return q.obj[i]-p.obj[i];
	}	
	return 0;
}

//Be inspired by "https://github.com/renansantosmendes/WFG_Hypervolume/blob/master/wfg.c"
void IbeaSolver::caculator_hv2D(Population *pop){
    vector<double> ref{200,500}; //reference point
    // the diffpop present the distance between individual and reference point on each dimension.
    Population diffpop(popsize,indsize); //get copy
    for (int i = 0; i < popsize; i++) {
		for(int j = 0; j < 2; j++) {
            diffpop.ind[i]->obj[j] = fabs(pop->ind[i]->obj[j] - ref[j]);
		}
	}
	qsort(diffpop.ind,popsize,sizeof(Population*) ,cmp_greater);
    double volume = diffpop.ind[0]->obj[0] * diffpop.ind[0]->obj[1];
	for (int i = 1; i < popsize; i++) {
		volume += diffpop.ind[i]->obj[1] * (diffpop.ind[i]->obj[0] - diffpop.ind[i-1]->obj[0]);
	}
	hv_values.push_back(volume);
}

void IbeaSolver::copy_ind(Individual *ind1, Individual *ind2)
{
    ind2->rank = ind1->rank;
    ind2->fitness = ind1->fitness;
    ind2->cv = ind1->cv;
    if (indsize != 0)
        for (int i = 0; i < indsize; i++)
            ind2->xreal[i] = ind1->xreal[i];
    for (int i = 0; i < indsize; i++)
        ind2->obj[i] = ind1->obj[i];
    return;
}

void IbeaSolver::merge()
{
    Population *pop1 = parent_pop;
    Population *pop2 = offspring_pop;
    Population *pop3 = mixed_pop;
    for (int i = 0; i < popsize; i++)
        copy_ind(pop1->ind[i], pop3->ind[i]);
    for (int i = 0, j = popsize; i < popsize; i++, j++)
        copy_ind(pop2->ind[i], pop3->ind[j]);
    return;
}

double IbeaSolver::cal_eps_indicator(Individual *ind1, Individual *ind2)
{
    double max_eps, temp;
    max_eps = ind1->obj[0] - ind2->obj[0];
    for (int i = 1; i < number_objective; i++)
    {
        temp = (ind1->obj[i] - ind2->obj[i]);
        if (temp > max_eps)
            max_eps = temp;
    }
    return max_eps; // Find the maximum value of eps which satisfies the indicator-formula:I_value.
}

void IbeaSolver::calcFitness()
{
    int size = 2 * popsize;
    Population *pop = mixed_pop;
    // ����ָ��ֵ
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            Indicator_value[i][j] = cal_eps_indicator(pop->ind[i], pop->ind[j]);
        }
    }

    // ��������ֱ����ָ��ֵ
    for (int i = 0; i < size; i++)
        for (int j = 0; j < size; j++)
            Indicator_value[i][j] = -exp(-Indicator_value[i][j] / kappa);

    // �õ���Ӧ��ֵ
    double sum;
    for (int i = 0; i < size; i++)
    {
        sum = 0;
        for (int j = 0; j < size; j++)
            if (i != j)
                sum += Indicator_value[j][i];
        pop->ind[i]->fitness = sum;
    }

    return;
}

void IbeaSolver::environmental_selection()
{
    int size = 2 * popsize;
    int i, j;
    int worst, new_size;

    for (int i = 0; i < size; i++)
        flag[i] = 0;

    // ������Ⱥ������size��popsize
    for (int i = size - popsize; i > 0; i--)
    {
        // �ҳ���Ӧ����С�ĸ��壬flag��Ϊ1
        for (j = 0; j < size && flag[j] == 1; j++)
            ;
        worst = j;
        for (j = j + 1; j < size; j++)
        {
            if (flag[j] != 1)
            {
                if (mixed_pop->ind[j]->fitness < mixed_pop->ind[worst]->fitness)
                    worst = j;
            }
        }
        // ʣ�µĸ��������Ӧ��ֵ
        for (j = 0; j < size; j++)
            if (flag[j] != 1)
                mixed_pop->ind[j]->fitness -= Indicator_value[worst][j];

        flag[worst] = 1;
    }

    // flag=1��ʾ���ٱ����ø��壬��flag=0���屣������
    new_size = 0;
    for (i = 0; i < size; i++)
    {
        if (flag[i] != 1)
        {
            copy_ind(mixed_pop->ind[i], parent_pop->ind[new_size]);
            new_size++;
        }
    }
}

void IbeaSolver::ibea_selection()
{
    // ������Ӧ��ֵ
    calcFitness();
    // ������Ӧ��ֵ����ѡ��
    environmental_selection();
}

void IbeaSolver::track_evolution(int generations)
{

    // �������Ļ
    for (int i = 0; i < popsize; ++i)
    {
        cout << i + 1 << "individual_value:";
        for (int j = 0; j < indsize; ++j)
        {
            cout << parent_pop->ind[i]->xreal[j] << ' ';
        }
        cout << "---";
        cout << "objective_value:";
        for (int j = 0; j < number_objective; ++j)
        {
            cout << parent_pop->ind[i]->obj[j] << ' ';
        }
        cout << "---";
        cout << "fitness_value:";
        cout << parent_pop->ind[i]->fitness << ' ';
        cout << endl;
    }

    if (generations == 0)
    {
        // ��������:
        *outfile << "paremeters:" << endl;
        *outfile << "tick: " << tick << endl;
        *outfile << "popsize: " << popsize << endl;
        *outfile << "eva_num: " << eva_num << endl;
        *outfile << "n_signages: " << n_signages << endl;
        *outfile << "CR: " << CR << endl;
        *outfile << "F: " << F << endl;

        // ��ʼ��
        *outfile << "��ʼ�����: " << endl;
        // title
        *outfile << "Ŀ��1-ʱ��"
                 << ","
                 << "Ŀ��2-ѹ��";
        for (int i = 1; i <= indsize / 2; ++i)
        {
            *outfile << ","
                     << "x" << i << ","
                     << "y" << i;
        }
        *outfile << endl;
        // Ŀ��1+Ŀ��2+����ֵ��2*n_signages��
        for (int i = 0; i < popsize; ++i)
        {
            for (int j = 0; j < number_objective; ++j)
            {
                *outfile << parent_pop->ind[i]->obj[j] << ",";
            }
            for (int j = 0; j < indsize; ++j)
            {
                *outfile << parent_pop->ind[i]->xreal[j] << ",";
            }
            *outfile << endl;
        }
    }
    else
    {
        // ��ǰ�ڼ���
        *outfile << "��ǰΪ��" << generations << "����" << endl;
        // title
        *outfile << "Ŀ��1-ʱ��"
                 << ","
                 << "Ŀ��2-ѹ��";
        for (int i = 1; i <= indsize / 2; ++i)
        {
            *outfile << ","
                     << "x" << i << ","
                     << "y" << i;
        }
        *outfile << endl;

        // Ŀ��1+Ŀ��2+����ֵ��2*n_signages��
        for (int i = 0; i < popsize; ++i)
        {
            for (int j = 0; j < number_objective; ++j)
            {
                *outfile << parent_pop->ind[i]->obj[j] << ",";
            }
            for (int j = 0; j < indsize; ++j)
            {
                *outfile << parent_pop->ind[i]->xreal[j] << ",";
            }
            *outfile << endl;
        }
    }

    if(generations%10 == 0){
        for(const auto hv:hv_values){
            *outfile << hv << ',';
        }
        cout<<endl;
    }
}

void IbeaSolver::run()
{
    Timer time = Timer();
    time.reset();
    // cout << "-----IBEA Algorithm Start-----" << endl;
    for (int generations = 1; generations <= max_generations; ++generations)
    {
        crossover_mutation(); // ���桢ͻ��
        evaluate_population(offspring_pop);
        merge();
        ibea_selection(); // ѡ��
        caculator_hv2D(parent_pop);
        track_evolution(generations);
    }
    *outfile << "Total time : " << time.elapsed() << endl;
    return;
}
