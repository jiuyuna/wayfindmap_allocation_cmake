#pragma once

#include <iostream>
#include <string>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <random>
#include <array>

using namespace std;

#define table(i, j) *(*(table + i) + j)
#define PI acos(-1)
#define MAXSIGNAGES 50
#define MAX_ENTRANCES 30

//����ͼ����
#define MAX_Height 200
#define MAX_Width 200

// SFM
#define tao 0.5
#define v0 1.01 //��ʼ�ٶ�
#define A 1000  // A,BΪ����������
#define B 0.08
#define k1 120000      // k
#define k2 240000      //СK
#define maxv 2         //��������ٶ�
#define c_mass 320     // c_mass = m/r (r=0.25, m=80kg)
#define sense_range 10 //��������,�����������...������
#define tick 0.02      //ÿ��0.02һ�θ���

/*������ز���*/
extern string scene_name;
extern int n_signages;

extern int simulation_time;
extern int stag_time; //���˿���ָʾ��ͣ��ʱ��]

//����Agant(����)
struct AGENT
{
    // id
    int id;
    //����
    double m;

    // x,y��ʾ��ǰλ������
    double x;
    double y;
    //��ǰ���ٶ�����
    double vx;
    double vy;
    //Ŀ�ĵ�����
    double gx;
    double gy;
    //����Ŀ�ĵ�
    double lgx;
    double lgy;

    //����ʱ��
    int create_time;
    //��ʶ�����ɹ�
    int flag_create;
    //�뿪���ڵ�ʱ��
    int leave_time;

    //���id
    int entrance_id;
    //���ڵ�id����ʶ��
    int exit_id;
    //��������ѭ��ָʾ��id�����ڿ���ָʾ�ƺ����Ѱ·
    int sign_id;
    //��Astar�ҵ�һ��·��������ڵ㹹�ɣ�����n_setp�жϵ�ǰ�ߵ����ˣ��ĸ��ڵ��ˣ�
    size_t n_setp;
    //����ָʾ�Ƶ�·����Ϣ����n_setp_sign�жϵ�ǰ�ߵ����ˣ��ĸ��ڵ��ˣ�
    size_t n_setp_sign;

    // �����ж������Ƿ񿴵�ָʾ��
    bool isFind;
    // ���ڼ�¼���˶�ȡ·�Ƶ���Ϣʱ��
    int loading = 0;
    // �����ж������Ƿ����ָʾ��
    bool isKnown;
    // ��Ұ��Χ
    double FOV;
    double max_view_dist;

    //�洢agent��x��y����gx��gy��·���㡣
    vector<vector<double>> path;
    //�洢������ɵ���ʱĿ�ĵ�
    double temp_gx;
    double temp_gy;
    //ÿ��agent����һ��map��Ϣ���������Ŀ�ĵص�ѡ�񣨽������㷨��
    double map[MAX_Height][MAX_Width];

    //��¼�ϴ�����λ��
    double last_x;
    double last_y;
    //������ʱ��(record_time�ε������������ж�������Χ�߳���ֹ����)
    int record_time;
    //�������Դ���������3�ξ���Ϊ��ǰ���˵���Ϣ��Ч
    int numOfTry;

    //��¼����ʱ��
    double travel_time;

    //���ڼ�¼���˵��ܵ�ѹ����
    double max_pressure;
};

struct OBSLINE
{
    //��ʼλ��
    double sx;
    double sy;
    //�յ�λ��
    double ex;
    double ey;
    //����
    double d;
    //��ʶ
    int sign;
};

struct ENTRANCE
{
    //���id
    int door_id;
    //��ʼλ��
    double sx;
    double sy;
    //�յ�λ��
    double ex;
    double ey;
    //���г��ȵ��ų���Ϊ��
    double en_x;
    double en_y;

    //������ڵĺ��ʺ�·��λ��,��Ӧ�ú����λ���غϡ�
    double greedy_sign_x;
    double greedy_sign_y;

    //��¼��ǰ�ڵ��������ʣ���һ����̬�ֲ���
    double generate_v;
    //��¼ÿ��һ��ʱ���������������һ�����ɷֲ���
    int generate_pop;

    //����ͳ�Ƶ�ǰ�������ɶ���agent
    int num;
};

struct SIGNAGE
{
    // id
    int sign_id;
    //����λ��
    double x;
    double y;
    //��ʶ�Ŀɼ����������ȼ�ͨ���뾶����
    double vca_r;
    //��ʶ�Ŀ�������
    double vca_R;
    //��sign���������ڵ�·��
    vector<vector<double>> path[MAX_ENTRANCES];
};

// Astar���õ㣬����G��H��F
struct CPoint
{
    int X;
    int Y;
    int G;
    int H;
    int F;
    CPoint *m_pParentPoint;
    CPoint *next;

    CPoint(int x, int y) : X(x), Y(y), G(0), H(0), F(0), m_pParentPoint(nullptr), next(nullptr) {}
    bool operator<(const CPoint &p) const
    {
        return F > p.F;
    }
};

#define MAXVEX 3000
#define INFINITY_DIST 65535

typedef int Patharc[MAXVEX][MAXVEX];
typedef int ShortPathTable[MAXVEX][MAXVEX];

typedef struct
{
    int vex[MAXVEX];
    int arc[MAXVEX][MAXVEX];
    int numVertexes;
} MGraph;

extern vector<OBSLINE> obstical_lines;
extern vector<AGENT> all_agents; //��ʷ���е�������Ϣ
extern vector<CPoint> available_points;
extern vector<ENTRANCE> entrances;
extern int myMap[MAX_Height][MAX_Width];
extern int Width;
extern int Height;
extern Patharc pathTable;
extern MGraph G;

//=========================================������Ϊ������غ���==================

struct point
{
    double x;
    double y;
    point(double X, double Y) : x(X), y(Y) {}
};

//=========================================��ʼ������-�ϰ�������==============================================
void initial_obstical_lines(string str, string scene_name, vector<OBSLINE> &obstical_lines);

void init_door(string path1, string path2);

void initMap(int table[][MAX_Width]);

//���ڲ���agent
void creatAgent(int entrance_id, int create_time, int exit_id);

//��ʼ��������Ϣ()
void init_pedestrian(string traffic_path, string timetable_path);

/*�ҳ����е㣬��ɢ*/

void BFS_findponits(int x, int y);
void initAvailablePoints();
bool isNear(int i_id, int j_id);
void CreateMGraph(MGraph *G);
void ReadTable(ifstream &ifile, string shortPath);
//��ʼ��door��obstacles
void initialize();
//=========================================Astar�漰����==================

bool cmp(const CPoint *left, const CPoint *right);

//����entrance����������
bool cmp2(const ENTRANCE left, const ENTRANCE right);

void initAstar();

void removeFromOpenList(CPoint *point);

bool inCloseList(int x, int y);

bool inOpenList(int x, int y);

bool CanReach(int x, int y);
bool CanReach(CPoint *start, int x, int y, bool IsIgnoreCorner);

std::vector<CPoint *> SurrroundPoints(CPoint *point, bool IsIgnoreCorner);

int CalcG(CPoint *start, CPoint *point);
int CalcH(CPoint *end, CPoint *point);
void calcF(CPoint *point);

void FoundPoint(CPoint *tempStart, CPoint *point);

void NotFoundPoint(CPoint *tempStart, CPoint *end, CPoint *point);

CPoint *FindPath_staticFloyd(const CPoint& start, const CPoint& end);

void freeAstar();

void Patn_SignToEntrance();

//=========================================������Ϊ������غ���==================

//���ܣ����������ֱ����߻����ұ�
//���أ�0���ߡ�1��ߡ�-1�ұ�
int left_right(point a, point b, double x, double y);

//���ܣ��߶�c,d��ֱ��a,b�Ƿ��ཻ
bool intersect1(point a, point b, point c, point d);

//���ܣ��ж��߶�c,d���߶�a,b�Ƿ��ཻ
bool intersect(point a, point b, point c, point d);

//�ж���ǰ�����Ƿ񵽴�Ŀ�ĵ�
int isArrival(AGENT *cur_agent);

int isStagnate(AGENT *cur_agent);

//������ɵ�����ڵ�֮�����ϰ�Ҳcontinue(����д�����ж�����������ϰ��ĺ���)
bool isObstacle(double sx, double sy, double ex, double ey);

void surroundings_feasibility_point(double cur_x, double cur_y, int &dec_x, int &dec_y);

//���㲻ͬagent֮����໥����
void compute_agent_force(AGENT *cur_agent, AGENT *other_agent, double *total_fx, double *total_fy);

//����agent��ǽ�塢�ϰ���ľ���
double Point_to_line_distance(double x0, double y0, double sx, double sy, double ex, double ey, double d, double *crx, double *cry);

void FindPath_Agent_staticFloyd(AGENT *cur_agent, int desx, int desy);
void FindPath_Agent_staticFloyd(AGENT *cur_agent, int curx, int cury, int desx, int desy);
//���ݵ�ǰagent��λ�ã����ɵ�����Ŀ�ĵص�·��
void FindPath_Agent(AGENT *cur_agent, int desx, int desy);

//���ݵ�ǰλ�ã����ɵ�����Ŀ�ĵص�·��
void FindPath_Agent(AGENT *cur_agent, int curx, int cury, int desx, int desy);

//�в��Ե����ѡ����ѡ�����Ŀ�ĵؾ����ܷ�ɢ��
void Tournament_Selection(AGENT *cur_agent);

//�������������
void randomWalk();

bool isObstacle_between_Agent_Sign(AGENT *cur_agent, double sign_x, double sign_y);

//�Ըõ��ж��Ƿ���С�
bool isFeasible(int x, int y);

//���һ���㣬����õ�λ���ϰ��ڻ�߽��У���Ż�һ������λ�ã���λ��Ϊ���н⡣
void Rectify(double orgin_x, double orgin_y, int &r_x, int &r_y);

void findFeasiblePointSurrounding(double orgin_x, double orgin_y, double& r_x, double& r_y);

///����״̬����������Ϊ
void Stagnate_Avoid();

//����������Ѱ��ָʾ�Ƶ���Ϊ���Ѿ�����֮�����Ϊ
void SearchSign();

//��Ҫ��������
void step(int cur_i);

//ÿ������ʱ��Ҫ�������õ�һЩ����
void reset(double trial[]);

//=========================================������==============================================
void run_simulation(double trial[], double &avg_time, double &pressure, double *greedy_trial = NULL);
