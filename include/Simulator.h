#pragma once
#include "Global.h"
#include "Util.h"
#include <vector>
#include <fstream>




class Simulator
{
public:
    //��ǰģ��id
    int Simulator_id;

    //·����Ϣ
    vector<SIGNAGE> signages;
    int n_signages; //��ʶ��
    int n_entrances;
    //������ز���
    vector<AGENT> agents;
    int sum_people;          //����������
    int getin, getout;       //����ͳ��tʱ���ڽ�����������Ѿ���ȥ������
    double total_time;       //����ͳ���������������ѵ�ʱ��
    double avg_max_pressure; //ƽ���������ܵ������ѹ��
    double sum_pre;          //ĳһʱ�̣������ܵ�����ѹ��

    // ������Ҫ���ص�����ָ��
    double avg_time;
    double pressure;
    //��������ļ�(���߳�ʱ����عر�)
    static const bool saveTracks = false;
    static const bool saveIndicatorsPerSecond = false;
    static const bool saveTimeTableAndKeepSame = true;
    string outPressurePerSecond = {}; //

    //�ļ�����ӿ�
    ofstream *oFile1;

    //��¼����ʱ��
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
