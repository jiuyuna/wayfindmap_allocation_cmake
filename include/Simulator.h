#pragma once
#include "Global.h"
#include "Util.h"
#include <vector>
#include <fstream>




class Simulator
{
public:
    //当前模拟id
    int Simulator_id;

    //路牌信息
    vector<SIGNAGE> signages;
    int n_signages; //标识数
    int n_entrances;
    //行人相关参数
    vector<AGENT> agents;
    int sum_people;          //进入总人数
    int getin, getout;       //用于统计t时间内进入的人数和已经出去的人数
    double total_time;       //用于统计所用行人所花费的时间
    double avg_max_pressure; //平均行人所受到的最大压力
    double sum_pre;          //某一时刻，行人受到的总压力

    // 最终需要返回的两个指标
    double avg_time;
    double pressure;

    //控制输出文件(多线程时请勿必关闭)
    static const bool saveTracks = false;
    static const bool saveIndicatorsPerSecond = false;
    static const bool saveTimeTableAndKeepSame = true;
    //文件输出接口
    ofstream *oFile1;
    ofstream *oFile2;
    //记录运行时间
    Timer *time_recoder;
    Timer *time_recoder_anyplace;

    Simulator(double trial[], int num_signs);
    ~Simulator();


    void Patn_SignToEntrance();
    void Stagnate_Avoid();
    void Tournament_Selection(AGENT *cur_agent);
    int isArrival(AGENT *cur_agent);
    void SearchSign();
    void compute_agent_force(AGENT *cur_agent, AGENT *other_agent, double *total_fx, double *total_fy);
    double Point_to_line_distance(double x0, double y0, double sx, double sy, double ex, double ey, double d, double *crx, double *cry);
    void randomWalk();
    void walkToDes();
    void SearchExits();
    void SFM();
    void step();
    void run_simulation();

private:
};
