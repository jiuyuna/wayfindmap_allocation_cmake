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

//最大地图长宽
#define MAX_Height 200
#define MAX_Width 200

// SFM
#define tao 0.5
#define v0 1.01 //初始速度
#define A 1000  // A,B为两个常数项
#define B 0.08
#define k1 120000      // k
#define k2 240000      //小K
#define maxv 2         //行人最大速度
#define c_mass 320     // c_mass = m/r (r=0.25, m=80kg)
#define sense_range 10 //敏感区域,在这个区域内...待补充
#define tick 0.02      //每隔0.02一次更新

/*场景相关参数*/
extern string scene_name;
extern int n_signages;

extern int simulation_time;
extern int stag_time; //行人看清指示牌停留时间]

//定义Agant(行人)
struct AGENT
{
    // id
    int id;
    //质量
    double m;

    // x,y表示当前位置坐标
    double x;
    double y;
    //当前的速度向量
    double vx;
    double vy;
    //目的地坐标
    double gx;
    double gy;
    //最终目的地
    double lgx;
    double lgy;

    //创建时间
    int create_time;
    //标识创建成功
    int flag_create;
    //离开出口的时间
    int leave_time;

    //入口id
    int entrance_id;
    //出口的id，标识符
    int exit_id;
    //行人所遵循的指示牌id，用于看到指示牌后面的寻路
    int sign_id;
    //当Astar找到一条路径后（许多节点构成），用n_setp判断当前走到哪了（哪个节点了）
    size_t n_setp;
    //根据指示牌的路径信息，用n_setp_sign判断当前走到哪了（哪个节点了）
    size_t n_setp_sign;

    // 用于判断行人是否看到指示牌
    bool isFind;
    // 用于记录行人读取路牌的信息时间
    int loading = 0;
    // 用于判断行人是否看清楚指示牌
    bool isKnown;
    // 视野范围
    double FOV;
    double max_view_dist;

    //存储agent从x，y到达gx，gy的路径点。
    vector<vector<double>> path;
    //存储随机生成的暂时目的地
    double temp_gx;
    double temp_gy;
    //每个agent保存一个map信息，用于随机目的地的选择（锦标赛算法）
    double map[MAX_Height][MAX_Width];

    //记录上次所在位置
    double last_x;
    double last_y;
    //防卡计时器(record_time次迭代后行人无行动则向周围走出防止卡死)
    int record_time;
    //卡死后尝试次数，超过3次就认为当前行人的信息无效
    int numOfTry;

    //记录旅行时间
    double travel_time;

    //用于记录行人的受到压力。
    double max_pressure;
};

struct OBSLINE
{
    //起始位置
    double sx;
    double sy;
    //终点位置
    double ex;
    double ey;
    //长度
    double d;
    //标识
    int sign;
};

struct ENTRANCE
{
    //入口id
    int door_id;
    //起始位置
    double sx;
    double sy;
    //终点位置
    double ex;
    double ey;
    //将有长度的门抽象为点
    double en_x;
    double en_y;

    //靠近入口的合适和路牌位置,不应该和入口位置重合。
    double greedy_sign_x;
    double greedy_sign_y;

    //记录当前口的生成速率，是一个正态分布。
    double generate_v;
    //记录每隔一段时间进来的人数，是一个泊松分布。
    int generate_pop;

    //用于统计当前还需生成多少agent
    int num;
};

struct SIGNAGE
{
    // id
    int sign_id;
    //放置位置
    double x;
    double y;
    //标识的可见清晰区域，先简单通过半径定义
    double vca_r;
    //标识的可以区域
    double vca_R;
    //该sign到各个出口的路径
    vector<vector<double>> path[MAX_ENTRANCES];
};

// Astar所用点，包含G、H、F
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
extern vector<AGENT> all_agents; //历史所有的行人信息
extern vector<CPoint> available_points;
extern vector<ENTRANCE> entrances;
extern int myMap[MAX_Height][MAX_Width];
extern int Width;
extern int Height;
extern Patharc pathTable;
extern MGraph G;

//=========================================行人行为交互相关函数==================

struct point
{
    double x;
    double y;
    point(double X, double Y) : x(X), y(Y) {}
};

//=========================================初始化函数-障碍物、出入口==============================================
void initial_obstical_lines(string str, string scene_name, vector<OBSLINE> &obstical_lines);

void init_door(string path1, string path2);

void initMap(int table[][MAX_Width]);

//用于产生agent
void creatAgent(int entrance_id, int create_time, int exit_id);

//初始化行人信息()
void init_pedestrian(string traffic_path, string timetable_path);

/*找出可行点，离散*/

void BFS_findponits(int x, int y);
void initAvailablePoints();
bool isNear(int i_id, int j_id);
void CreateMGraph(MGraph *G);
void ReadTable(ifstream &ifile, string shortPath);
//初始化door和obstacles
void initialize();
//=========================================Astar涉及函数==================

bool cmp(const CPoint *left, const CPoint *right);

//用于entrance的流量排序
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

//=========================================行人行为交互相关函数==================

//功能：求点在有向直线左边还是右边
//返回：0共线、1左边、-1右边
int left_right(point a, point b, double x, double y);

//功能：线段c,d和直线a,b是否相交
bool intersect1(point a, point b, point c, point d);

//功能：判断线段c,d和线段a,b是否相交
bool intersect(point a, point b, point c, point d);

//判定当前行人是否到达目的地
int isArrival(AGENT *cur_agent);

int isStagnate(AGENT *cur_agent);

//如果生成点和所在点之间有障碍也continue(后面写个个判断两点间有无障碍的函数)
bool isObstacle(double sx, double sy, double ex, double ey);

void surroundings_feasibility_point(double cur_x, double cur_y, int &dec_x, int &dec_y);

//计算不同agent之间的相互作用
void compute_agent_force(AGENT *cur_agent, AGENT *other_agent, double *total_fx, double *total_fy);

//计算agent到墙体、障碍物的距离
double Point_to_line_distance(double x0, double y0, double sx, double sy, double ex, double ey, double d, double *crx, double *cry);

void FindPath_Agent_staticFloyd(AGENT *cur_agent, int desx, int desy);
void FindPath_Agent_staticFloyd(AGENT *cur_agent, int curx, int cury, int desx, int desy);
//根据当前agent的位置，生成到给定目的地的路径
void FindPath_Agent(AGENT *cur_agent, int desx, int desy);

//根据当前位置，生成到给定目的地的路径
void FindPath_Agent(AGENT *cur_agent, int curx, int cury, int desx, int desy);

//有策略的随机选择，让选择出的目的地尽可能分散。
void Tournament_Selection(AGENT *cur_agent);

//场景内随机行走
void randomWalk();

bool isObstacle_between_Agent_Sign(AGENT *cur_agent, double sign_x, double sign_y);

//对该点判断是否可行。
bool isFeasible(int x, int y);

//输出一个点，如果该点位于障碍内或边界中，则放回一个修正位置，该位置为可行解。
void Rectify(double orgin_x, double orgin_y, int &r_x, int &r_y);

///各个状态防卡死的行为
void Stagnate_Avoid();

//定义了行人寻找指示牌的行为，已经看到之后的行为
void SearchSign();

//主要迭代过程
void step(int cur_i);

//每次评估时需要重新设置的一些参数
void reset(double trial[]);

//=========================================主函数==============================================
void run_simulation(double trial[], double &avg_time, double &pressure, double *greedy_trial = NULL);
