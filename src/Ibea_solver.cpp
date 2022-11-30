#include "Ibea_solver.h"
#include "Util.h"
#include <iostream>
#include <random>
#include <vector>
#include <string>

using namespace std;

IbeaSolver::IbeaSolver(int pop_size, int ind_size, int max_gens, int num_eva) : popsize{pop_size}, indsize{ind_size}, max_generations{max_gens}, eva_num{num_eva}, n_signages{ind_size / 2}
{

    outfile = new ofstream("/home/cyx/wayfindmap_allocation_cmake/result/IBEA/"+ scene_name + "_"+ to_string(n_signages) + "signs_"+ to_string(popsize) +"popsize_30eva_1000gens_0.02tick_res", ios::out);
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
    // 删除空间遇到了BUG,但是这三个指针指向空间由系统自动回收就行，无伤大雅。
    // delete parent_pop;
    // delete offspring_pop;
    // delete mixed_pop;
    outfile->close();
    delete outfile;
    outfile = nullptr;
}

void IbeaSolver::initialize_population(Population *pop)
{
    for (int i = 0; i < popsize; ++i)
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

#define F 0.9  // F控制差分变化的放大，[0,2]
#define CR 0.7 //交叉常数 [0,1]

void IbeaSolver::crossover_mutation()
{
    // //DE
    // for (int i = 0; i < popsize; ++i) {
    //     //随机生成r1,r2,r3。对应着序号为r1,r2,r3的三个个体，r1,r2,r3,i互不相同
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
    //         if (random01() < CR || k == indsize) { //k==D确保至少得到一个参数
    //             offspring_pop->ind[i]->xreal[j] = parent_pop->ind[r3]->xreal[j] + F * (parent_pop->ind[r1]->xreal[j] - parent_pop->ind[r2]->xreal[j]);  //变异操作
    //         }
    //         else {
    //             offspring_pop->ind[i]->xreal[j] = parent_pop->ind[i]->xreal[j];
    //         }
    //         j = (j + 1) % popsize;
    //     }
    //     //越界处理，假设[0,1]
    //     for (int k = 0; k < indsize; ++k) {
    //         if (offspring_pop->ind[i]->xreal[k] < 0) offspring_pop->ind[i]->xreal[k] = 0;
    //         if (offspring_pop->ind[i]->xreal[k] > 1) offspring_pop->ind[i]->xreal[k] = 1;
    //     }
    // }

    // DE_for_模拟
    for (int i = 0; i < popsize; ++i)
    {
        //随机生成r1,r2,r3。对应着序号为r1,r2,r3的三个个体，r1,r2,r3,i互不相同
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
            {                                                                                                                                                      // k==D确保至少得到一个参数
                offspring_pop->ind[i]->xreal[2 * j] = parent_pop->ind[r3]->xreal[j] + F * (parent_pop->ind[r1]->xreal[2 * j] - parent_pop->ind[r2]->xreal[2 * j]); //变异操作
                offspring_pop->ind[i]->xreal[2 * j + 1] = parent_pop->ind[r3]->xreal[2 * j + 1] + F * (parent_pop->ind[r1]->xreal[2 * j + 1] - parent_pop->ind[r2]->xreal[2 * j + 1]);
            }
            else
            {
                offspring_pop->ind[i]->xreal[2 * j] = parent_pop->ind[i]->xreal[2 * j];
                offspring_pop->ind[i]->xreal[2 * j + 1] = parent_pop->ind[i]->xreal[2 * j + 1];
            }

            if (scene_name == "scene3")
            {
                //越界处理
                {
                    // x
                    if (offspring_pop->ind[i]->xreal[2 * j] < 1)
                        offspring_pop->ind[i]->xreal[2 * j] = 1;
                    if (offspring_pop->ind[i]->xreal[2 * j] > 104) //确保0<x<1050
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
                //越界处理
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
                }
            }

            //根据y检查当前是否出现在不适合位置，是的话则重新生成或生成在附近的某点，该点要是在可行域内。
            if (myMap[(int)offspring_pop->ind[i]->xreal[2 * j]][(int)offspring_pop->ind[i]->xreal[2 * j + 1]] == 1)
            {
                int r_x = 0;
                int r_y = 0;
                Rectify(offspring_pop->ind[i]->xreal[2 * j], offspring_pop->ind[i]->xreal[2 * j + 1], r_x, r_y);
                offspring_pop->ind[i]->xreal[2 * j] = r_x;
                offspring_pop->ind[i]->xreal[2 * j + 1] = r_y;
            }
            j = (j + 1) % n_signages;
            if (j >= n_signages)
                break;
        }
    }
}

//测试函数
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

//仿真模块
void IbeaSolver::simulation(Individual *ind_cur, IbeaSolver *_this)
{
    int n_signages = ind_cur->indsize;
    Simulator sim = Simulator(ind_cur->xreal, n_signages);
    sim.run_simulation();
    //需要互斥访问
    _this->mymutex.lock();
    ind_cur->obj[0] += sim.avg_time;
    ind_cur->obj[1] += sim.avg_max_pressure;
    _this->mymutex.unlock();
}

void IbeaSolver::evaluate_population(Population *pop)
{
    for (int i = 0; i < popsize; ++i)
    {
        Individual *ind_cur = pop->ind[i];  //zdt1(ind_cur); //test function
        //simulator
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
    // 计算指标值
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            Indicator_value[i][j] = cal_eps_indicator(pop->ind[i], pop->ind[j]);
        }
    }

    // 两两个体分别计算指标值
    for (int i = 0; i < size; i++)
        for (int j = 0; j < size; j++)
            Indicator_value[i][j] = -exp(-Indicator_value[i][j] / kappa);

    // 得到适应度值
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

    //削减种群数量从size到popsize
    for (int i = size - popsize; i > 0; i--)
    {
        //找出适应度最小的个体，flag置为1
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
        //剩下的个体更新适应度值
        for (j = 0; j < size; j++)
            if (flag[j] != 1)
                mixed_pop->ind[j]->fitness -= Indicator_value[worst][j];

        flag[worst] = 1;
    }

    // flag=1表示不再保留该个体，把flag=0个体保留下来
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
    //计算适应度值
    calcFitness();
    //根据适应度值进行选择
    environmental_selection();
}

void IbeaSolver::track_evolution(int generations)
{

    //输出到屏幕
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
        //参数设置:
        *outfile << "paremeters:"<<endl;
        *outfile << "tick: "<<tick<<endl;
        *outfile << "popsize: "<<popsize<<endl;
        *outfile << "eva_num: "<<eva_num<<endl;
        *outfile << "n_signages: "<<n_signages<<endl;

        //初始化
        *outfile << "初始化结果: " << endl;
        // title
        *outfile << "目标1-时间"
                 << ","
                 << "目标2-压力";
        for (int i = 1; i <= indsize / 2; ++i)
        {
            *outfile << ","
                     << "x" << i << ","
                     << "y" << i;
        }
        *outfile << endl;
        //目标1+目标2+个体值（2*n_signages）
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
        //当前第几代
        *outfile << "当前为第" << generations << "代：" << endl;
        // title
        *outfile << "目标1-时间"
                 << ","
                 << "目标2-压力";
        for (int i = 1; i <= indsize / 2; ++i)
        {
            *outfile << ","
                     << "x" << i << ","
                     << "y" << i;
        }
        *outfile << endl;

        //目标1+目标2+个体值（2*n_signages）
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
}

void IbeaSolver::run()
{
    //cout << "-----IBEA Algorithm Start-----" << endl;
    for (int generations = 1; generations <= max_generations; ++generations)
    {
        crossover_mutation(); // 交叉、突变
        evaluate_population(offspring_pop);
        merge();
        ibea_selection(); //选择
        track_evolution(generations);
    }
    return;
}
