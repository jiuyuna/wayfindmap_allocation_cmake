#include <iostream>
#include <stdlib.h>
#include "Simulator.h"
using namespace std;

Simulator::Simulator(double trial[], int num_signs)
{
    // 模拟中需要记录的参数
    sum_people = all_agents.size(); // 进入总人数
    getin = 0, getout = 0;          // 用于统计t时间内进入的人数和已经出去的人数
    total_time = 0;                 // 用于统计所用行人所花费的时间
    avg_max_pressure = 0;           // 平均行人所受到的最大压力
    sum_pre = 0;                    // 某一时刻，行人受到的总压力
    avg_time = 0;
    pressure = 0;

    n_entrances = entrances.size();

    // 模拟中需要使用的数据结构
    n_signages = num_signs;

    // 设置signages位置
    int j = 0;
    for (int i = 0; i < n_signages; ++i)
    {
        SIGNAGE signage;
        signage.sign_id = i;
        signage.x = trial[j];
        signage.y = trial[j + 1];
        signage.vca_r = 5;
        signage.vca_R = 15;
        j += 2;
        for (int k = 0; k < n_entrances; ++k)
        {
            signage.path[k].clear();
            signage.path[k].shrink_to_fit();
        }
        signages.push_back(signage);
    }

    // 行人运行时创建

    // 控制文件输入
    if (saveTracks)
    {
        oFile1 = new ofstream("/home/cyx/wayfindmap_allocation_cmake/result/data1/demo.dat", ios::out | ios::trunc);
    }

    time_recoder = new Timer();
    time_recoder->reset();
    time_recoder_anyplace = new Timer();
    time_recoder_anyplace->reset();
}

Simulator::~Simulator()
{
    // if (saveTracks)
    // {
    //     oFile1->close();
    //     delete oFile1;
    //     oFile1 = nullptr;
    // }
    delete time_recoder;
    delete time_recoder_anyplace;
}

void Simulator::Patn_SignToEntrance()
{
    for (int j = 0; j < n_signages; ++j)
    {
        int s_x = signages[j].x;
        int s_y = signages[j].y;

        // 起始位置不应该是障碍物
        if (myMap[s_x][s_y] == 1)
        {
            surroundings_feasibility_point(s_x, s_y, s_x, s_y);
        }

        // 根据出口位置寻找路径并保存
        for (int i = 0; i < n_entrances; ++i)
        {
            int e_x = entrances[i].en_x;
            int e_y = entrances[i].en_y;
            if (myMap[e_x][e_y] == 1)
            {
                surroundings_feasibility_point(e_x, e_y, e_x, e_y);
            }
            // 优化当指示牌和出口位置重合，直接赋值
            if (s_x == e_x && s_y == e_y)
            {
                vector<double> t;
                t.clear();
                t.push_back((double)s_x + 0.5);
                t.push_back((double)s_y + 0.5);
                signages[j].path[i].push_back(t);
                continue;
            }
            CPoint start(s_x, s_y);
            CPoint end(e_x, e_y);
            CPoint *parent = FindPath_staticFloyd(start, end);

            vector<double> temp;
            temp.clear();
            while (parent != NULL)
            {
                temp.clear();
                temp.push_back((double)parent->X);
                temp.push_back((double)parent->Y);
                signages[j].path[i].push_back(temp);
                CPoint *temp_ptr = parent;
                parent = parent->next;
                delete temp_ptr;
            }
        }
    }
}
/// 各个状态防卡死的行为
void Simulator::Stagnate_Avoid()
{
    for (size_t i = 0; i < agents.size(); ++i)
    {
        AGENT *cur_agent = &agents[i];
        if(cur_agent->record_time >= 20){
            findFeasiblePointSurrounding(cur_agent->x, cur_agent->y, cur_agent->gx, cur_agent->gy);
            //if(cur_agent->id == 32) cout<<cur_agent->x<<' '<< cur_agent->y<<"->"<< cur_agent->gx<<' '<<cur_agent->gy<<endl;
            double dist = sqrt((cur_agent->x - cur_agent->gx) * (cur_agent->x - cur_agent->gx) + (cur_agent->y - cur_agent->gy) * (cur_agent->y - cur_agent->gy));
            if (dist < 0.5)
            {
                // 利用当前起点和暂时目的地生成一条可行的路径。
                cur_agent->path.clear();
                cur_agent->path.shrink_to_fit();
                cur_agent->n_setp = 0;
                FindPath_Agent_staticFloyd(cur_agent, cur_agent->temp_gx, cur_agent->temp_gy);
                // 只为排除BUG
                if (cur_agent->path.size() == 0)
                {
                    cout << "错误，当前目的地:" << cur_agent->gx << ' ' << cur_agent->gy << endl;
                    continue;
                }
                else if (cur_agent->path[0].size() == 0)
                {
                    cout << "错误，当前目的地:" << cur_agent->gx << ' ' << cur_agent->gy << endl;
                    continue;
                }
                else
                {
                    cur_agent->gx = cur_agent->path[0][0];
                    cur_agent->gy = cur_agent->path[0][1];
                    // cout << cur_agent->id<<' '<<cur_agent->x<<' '<<cur_agent->y<<','<<cur_agent->gx<<' '<<cur_agent->gy<<','<<cur_agent->temp_gx<<' '<<cur_agent->temp_gy<<endl;
                }
                cur_agent->record_time = 0;
                cur_agent->numOfTry++;

            }
        }

        if (cur_agent->numOfTry > 20)
        {
            // 不再记录该行人
            sum_people--;
            for (vector<AGENT>::iterator iter = agents.begin(); iter != agents.end(); iter++)
            { // 从vector中删除指定的某一个元素
                if (iter->id == cur_agent->id)
                {
                    // cout<< cur_agent->id<<endl;
                    agents.erase(iter);
                    break;
                }
            }
        }
    }
}

int Simulator::isArrival(AGENT *cur_agent)
{
    double curx = cur_agent->x;
    double cury = cur_agent->y;
    double desx = cur_agent->gx;
    double desy = cur_agent->gy;

    // 先看是否到达最终目的地
    if (sqrt((curx - cur_agent->lgx) * (curx - cur_agent->lgx) + (cury - cur_agent->lgy) * (cury - cur_agent->lgy)) <= 1.5)
    {
        return 2;
    }
    // 当行人靠近目的地一定范围内为到达
    if (sqrt((curx - desx) * (curx - desx) + (cury - desy) * (cury - desy)) <= 0.5)
    {
        return 1;
    }
    if (sqrt((curx - cur_agent->temp_gx) * (curx - cur_agent->temp_gx) + (cury - cur_agent->temp_gy) * (cury - cur_agent->temp_gy)) <= 0.5)
    {
        return 3;
    }
    return 0;
}

void Simulator::Tournament_Selection(AGENT *cur_agent)
{
    int candidates = 10;
    // 选择出10个点进行竞标赛选择，选择最小的
    double min = 10000;
    int select_x{}, select_y{}; // 保存选择出的坐标
    int rnd, temp_x, temp_y;
    for (int i = 0; i < candidates; i++)
    {
        rnd = (int)randval(0, available_points.size());
        while (rnd < 0 || rnd >= (int)available_points.size())
            rnd = (int)randval(0, available_points.size()); // 防止意外嗷
        temp_x = available_points[rnd].X;
        temp_y = available_points[rnd].Y;
        if (cur_agent->map[temp_y][temp_x] < min)
        {
            min = cur_agent->map[temp_y][temp_x];
            select_x = temp_x;
            select_y = temp_y;
        }
    }
    cur_agent->temp_gx = select_x;
    cur_agent->temp_gy = select_y;

    // 更新map信息
    int left_up_x, left_up_y; // right_down_x, right_down_y;
    int n_surrounding = 20;   // 对于选择出的点以及附近给予一个权重，下次考虑时权重大选择概率小。20*20，内到外权重递减。
    double weight;
    for (int i = 0; i <= n_surrounding; ++i)
    {
        weight = 1 - (double)i * (1 / (double)n_surrounding);
        left_up_x = select_x - i;
        left_up_y = select_y + i;

        if (i == 0)
            cur_agent->map[select_y][select_x] += weight;
        // 上
        int x = left_up_x;
        int y = left_up_y;
        for (int k = 0; k < i * 2; k++)
        {
            if (x + k >= Width || x + k < 0 || y < 0 || y >= Height)
                continue;
            cur_agent->map[y][x + k] += weight;
        }
        // 右
        x = left_up_x + i * 2;
        y = left_up_y;
        for (int k = 0; k < i * 2; k++)
        {
            if (y - k < 0 || y - k >= Height || x < 0 || x >= Width)
                continue;
            cur_agent->map[y - k][x] += weight;
        }
        // 下
        x = left_up_x + i * 2;
        y = left_up_y - i * 2;
        for (int k = 0; k < i * 2; k++)
        {
            if (x - k < 0 || x - k >= Width || y < 0 || y >= Height)
                continue;
            cur_agent->map[y][x - k] += weight;
        }
        // 左
        x = left_up_x;
        y = left_up_y - i * 2;
        for (int k = 0; k < i * 2; k++)
        {
            if (y + k >= Height || y + k < 0 || x < 0 || x >= Width)
                continue;
            cur_agent->map[y + k][x] += weight;
        }
    }
}

// 场景内随机行走
void Simulator::randomWalk()
{
    for (size_t i = 0; i < agents.size(); ++i)
    {
        AGENT *cur_agent = &agents[i];

        if (!cur_agent->isFind && !cur_agent->isKnown)
        {
            if (cur_agent->n_setp == cur_agent->path.size() - 1 || cur_agent->path.size() == 0)
            {

                // 使用锦标赛算法随机生成下一个暂时目的地
                Tournament_Selection(cur_agent);

                // 利用当前起点和暂时目的地生成一条可行的路径。（先使用Astar找）
                cur_agent->path.clear();
                cur_agent->path.shrink_to_fit();
                cur_agent->n_setp = 0;

                FindPath_Agent_staticFloyd(cur_agent, cur_agent->temp_gx, cur_agent->temp_gy);
                // 只为排除BUG
                if (cur_agent->path.size() == 0)
                {
                    cout << "错误，当前目的地:" << cur_agent->gx << ' ' << cur_agent->gy << endl;
                    continue;
                }
                else if (cur_agent->path[0].size() == 0)
                {
                    cout << "错误，当前目的地:" << cur_agent->gx << ' ' << cur_agent->gy << endl;
                    continue;
                }
                else
                {
                    cur_agent->gx = cur_agent->path[0][0];
                    cur_agent->gy = cur_agent->path[0][1];
                }
            }
        }
    }
}

// 定义了行人寻找指示牌的行为，已经看到之后的行为
void Simulator::SearchSign()
{
    // 判断当前行人是否看见标识
    for (size_t i = 0; i < agents.size(); ++i)
    {
        AGENT *cur_agent = &agents[i];
        if (!cur_agent->isFind && !cur_agent->isKnown)
        {
            // 阶段1，行人看到指示牌，但看不清内容，于是向指示牌走去
            // 找一个最近的指示牌
            double cur_min_dist = 100000.0;
            int pos = -1;
            for (int j = 0; j < n_signages; ++j)
            {
                // 行人是否在标识的VCA中
                double dist = sqrt((cur_agent->x - signages[j].x) * (cur_agent->x - signages[j].x) + (cur_agent->y - signages[j].y) * (cur_agent->y - signages[j].y));
                // 标识是否在行人的视野范围内
                double Wp_x = cur_agent->vx;
                double Wp_y = cur_agent->vy;
                double Gp_x = signages[j].x - cur_agent->x;
                double Gp_y = signages[j].y - cur_agent->y;
                double WpGp = Wp_x * Gp_x + Wp_y * Gp_y;
                double Wp_mol = sqrt(Wp_x * Wp_x + Wp_y * Wp_y);
                double Gp_mol = sqrt(Gp_x * Gp_x + Gp_y * Gp_y);
                double theta;
                if (Wp_mol * Gp_mol == 0) // 防止除0
                    theta = 0;
                else
                    theta = acos(WpGp / (Wp_mol * Gp_mol));

                // 用于判断行人和指示牌间是否有障碍
                if (dist < cur_min_dist && ((dist <= signages[j].vca_R && theta <= (cur_agent->FOV / 2)) || dist <= signages[j].vca_r) && !isObstacle(cur_agent->x, cur_agent->y, signages[j].x, signages[j].y))
                {
                    cur_min_dist = dist;
                    pos = j;
                }
            }
            if (pos != -1)
            {
                cur_agent->sign_id = signages[pos].sign_id; // 看到的指示牌的id
                cur_agent->temp_gx = cur_agent->gx = signages[pos].x;
                cur_agent->temp_gy = cur_agent->gy = signages[pos].y;
                cur_agent->isFind = true;
                cur_agent->path.clear();
                cur_agent->path.shrink_to_fit();
                cur_agent->n_setp = 0;
            }
        }
        else if (cur_agent->isFind && !cur_agent->isKnown)
        {
            // 向指示牌走去过程中，路标就近原则
            if (cur_agent->loading == 0)
            {
                for (int j = 0; j < n_signages; ++j)
                {
                    double cur_dist = sqrt((cur_agent->x - signages[cur_agent->sign_id].x) * (cur_agent->x - signages[cur_agent->sign_id].x) + (cur_agent->y - signages[cur_agent->sign_id].y) * (cur_agent->y - signages[cur_agent->sign_id].y));
                    double dist = sqrt((cur_agent->x - signages[j].x) * (cur_agent->x - signages[j].x) + (cur_agent->y - signages[j].y) * (cur_agent->y - signages[j].y));
                    if (dist > cur_dist || cur_agent->sign_id == signages[j].sign_id)
                        continue;
                    double Wp_x = cur_agent->vx;
                    double Wp_y = cur_agent->vy;
                    double Gp_x = signages[j].x - cur_agent->x;
                    double Gp_y = signages[j].y - cur_agent->y;
                    double WpGp = Wp_x * Gp_x + Wp_y * Gp_y;
                    double Wp_mol = sqrt(Wp_x * Wp_x + Wp_y * Wp_y);
                    double Gp_mol = sqrt(Gp_x * Gp_x + Gp_y * Gp_y);
                    double theta;
                    if (Wp_mol * Gp_mol == 0) // 防止除0
                        theta = 0;
                    else
                        theta = acos(WpGp / (Wp_mol * Gp_mol));
                    if (((dist <= cur_dist && theta <= (cur_agent->FOV / 2)) || dist <= signages[j].vca_r) && !isObstacle(cur_agent->x, cur_agent->y, signages[j].x, signages[j].y))
                    {
                        // cout << cur_agent->id << "from " << cur_agent->sign_id << "to " << signages[j].sign_id << endl;
                        //  cout<<signages[j].sign_id<<' '<<dist<<" - "<<cur_agent->sign_id<<' '<<cur_dist<<endl;
                        cur_agent->sign_id = signages[j].sign_id; // 看到的指示牌的id
                        cur_agent->temp_gx = cur_agent->gx = signages[j].x;
                        cur_agent->temp_gy = cur_agent->gy = signages[j].y;
                        cur_agent->path.clear();
                        cur_agent->path.shrink_to_fit();
                        cur_agent->n_setp = 0;
                    }
                }
            }

            // 阶段2，行人根据获取指示牌的信息。
            // 行人看见指示牌后应该指定signage_id
            for (int j = 0; j < n_signages; ++j)
            {
                if (signages[j].sign_id != cur_agent->sign_id)
                    continue;
                // 行人是否在标识的VCA中
                double dist = sqrt((cur_agent->x - signages[j].x) * (cur_agent->x - signages[j].x) + (cur_agent->y - signages[j].y) * (cur_agent->y - signages[j].y));
                // 标识是否在行人的清晰见范围内
                if (dist <= signages[j].vca_r && !isObstacle(cur_agent->x, cur_agent->y, cur_agent->gx, cur_agent->gy)) // && theta <= cur_agent->FOV/2
                {
                    // 看到之后，读取信息。
                    if (cur_agent->loading < stag_time)
                    {
                        cur_agent->loading++;
                        break;
                    }
                    // 然后朝向规划的路径的第一个节点走去
                    cur_agent->path.clear();
                    cur_agent->path.shrink_to_fit();
                    cur_agent->n_setp = 0;
                    cur_agent->n_setp_sign = 0;
                    if (signages[cur_agent->sign_id].path[cur_agent->exit_id].size() == 0)
                    { // 防止越界
                        cur_agent->gx = cur_agent->lgx;
                        cur_agent->gy = cur_agent->lgy;
                    }
                    cur_agent->gx = signages[cur_agent->sign_id].path[cur_agent->exit_id][0][0];
                    cur_agent->gy = signages[cur_agent->sign_id].path[cur_agent->exit_id][0][1];

                    cur_agent->temp_gx = entrances[cur_agent->exit_id].en_x;
                    cur_agent->temp_gy = entrances[cur_agent->exit_id].en_y;

                    cur_agent->isKnown = true; // 表示已经看见,且读取完信息。

                    ////需重新设置速度方向
                    // double d0 = sqrt((cur_agent->gx - cur_agent->x) * (cur_agent->gx - cur_agent->x) + (cur_agent->gy - cur_agent->y) * (cur_agent->gy - cur_agent->y));
                    // cur_agent->vx = v0 * (cur_agent->gx - cur_agent->x) / d0;
                    // cur_agent->vy = v0 * (cur_agent->gy - cur_agent->y) / d0;
                }
            }
        }
    }
}

// 计算不同agent之间的相互作用
void Simulator::compute_agent_force(AGENT *cur_agent, AGENT *other_agent, double *total_fx, double *total_fy)
{
    // 计算前半部分
    double d = sqrt((cur_agent->x - other_agent->x) * (cur_agent->x - other_agent->x) + (cur_agent->y - other_agent->y) * (cur_agent->y - other_agent->y)); // 两个agant间的距离
    if (d == 0)
    {
        printf("here d == 0 error but fixed...");
        d = 1e-10;
    }
    double delta_d = cur_agent->m / c_mass + other_agent->m / c_mass - d;
    double fexp = A * exp(delta_d / B);
    double fkg = delta_d < 0 ? 0 : k1 * delta_d;
    double nijx = (cur_agent->x - other_agent->x) / d;
    double nijy = (cur_agent->y - other_agent->y) / d;
    double fnijx = (fexp + fkg) * nijx;
    double fnijy = (fexp + fkg) * nijy;

    // 计算后半部分
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
    *total_fx += fnijx + fkgx;
    *total_fy += fnijy + fkgy;
}

// 计算agent到墙体、障碍物的距离
double Simulator::Point_to_line_distance(double x0, double y0, double sx, double sy, double ex, double ey, double d, double *crx, double *cry)
{
    // t0是个系数，表示agent垂直到直线的点与s点的距离/障碍物线的长度。当0<t0<1,取agent垂直到障碍的距离
    // 当t0>1,取agent到e点距离，当t0<0,取agent到s点距离 (s为障碍物起始点，e为终点)
    double t0 = ((ex - sx) * (x0 - sx) + (ey - sy) * (y0 - sy)) / (d * d);
    if (t0 < 0)
    {
        d = sqrt((x0 - sx) * (x0 - sx) + (y0 - sy) * (y0 - sy));
    }
    else if (t0 > 1)
    {
        d = sqrt((x0 - ex) * (x0 - ex) + (y0 - ey) * (y0 - ey));
    }
    else
    {
        // 计算垂直向量的模
        d = sqrt(
            (x0 - (sx + t0 * (ex - sx))) * (x0 - (sx + t0 * (ex - sx))) +
            (y0 - (sy + t0 * (ey - sy))) * (y0 - (sy + t0 * (ey - sy))));
    }
    // 垂直交叉点坐标
    *crx = sx + t0 * (ex - sx);
    *cry = sy + t0 * (ey - sy);

    return d;
}

void Simulator::walkToDes()
{
    // 如果行人看到指示牌，则朝规划的路径走去，最终到达目的地
    for (size_t i = 0; i < agents.size(); ++i)
    {
        AGENT *cur_agent = &agents[i];
        if (cur_agent->isFind && cur_agent->isKnown)
        {
            if (cur_agent->n_setp_sign < signages[cur_agent->sign_id].path[cur_agent->exit_id].size())
            {
                // 先判断是否走到了预定的节点
                double tmp_gx = signages[cur_agent->sign_id].path[cur_agent->exit_id][cur_agent->n_setp_sign][0];
                double tmp_gy = signages[cur_agent->sign_id].path[cur_agent->exit_id][cur_agent->n_setp_sign][1];
                double dist = sqrt((cur_agent->x - tmp_gx) * (cur_agent->x - tmp_gx) + (cur_agent->y - tmp_gy) * (cur_agent->y - tmp_gy));
                if (dist <= 0.5)
                {
                    if (cur_agent->n_setp_sign + 1 == signages[cur_agent->sign_id].path[cur_agent->exit_id].size())
                    {
                        cur_agent->gx = cur_agent->lgx;
                        cur_agent->gy = cur_agent->lgy;
                        continue; // 到目的地了
                    }

                    cur_agent->gx = signages[cur_agent->sign_id].path[cur_agent->exit_id][cur_agent->n_setp_sign + 1][0];
                    cur_agent->gy = signages[cur_agent->sign_id].path[cur_agent->exit_id][cur_agent->n_setp_sign + 1][1];
                    {
                        cur_agent->path.clear();
                        cur_agent->path.shrink_to_fit();
                        cur_agent->n_setp = 0;
                    }
                    cur_agent->n_setp_sign++;
                }
            }
        }
    }
}

void Simulator::SearchExits()
{
    for (size_t i = 0; i < agents.size(); ++i)
    {
        AGENT *cur_agent = &agents[i];
        if (!cur_agent->isKnown)
        {
            // discover destination
            // 出口是否在行人的视野范围内
            double Wp_x = cur_agent->vx;
            double Wp_y = cur_agent->vy;
            double Gp_x = cur_agent->lgx - cur_agent->x;
            double Gp_y = cur_agent->lgy - cur_agent->y;
            double WpGp = Wp_x * Gp_x + Wp_y * Gp_y;
            double Wp_mol = sqrt(Wp_x * Wp_x + Wp_y * Wp_y);
            double Gp_mol = sqrt(Gp_x * Gp_x + Gp_y * Gp_y);
            double theta;
            if (Wp_mol * Gp_mol == 0) // 防止除0
                theta = 0;
            else
                theta = acos(WpGp / (Wp_mol * Gp_mol));
            // 距离、视野、无障碍阻挡
            if ((sqrt((cur_agent->x - cur_agent->lgx) * (cur_agent->x - cur_agent->lgx) + (cur_agent->y - cur_agent->lgy) * (cur_agent->y - cur_agent->lgy)) <= 15) && theta <= (cur_agent->FOV / 2) && !isObstacle(cur_agent->x, cur_agent->y, cur_agent->lgx, cur_agent->lgy))
            {
                cur_agent->path.clear();
                cur_agent->path.shrink_to_fit();
                cur_agent->n_setp = 0;
                cur_agent->gx = cur_agent->temp_gx = cur_agent->lgx;
                cur_agent->gy = cur_agent->temp_gy = cur_agent->lgy;
            }
        }

        // close to destination enough
        if (isArrival(cur_agent) == 2)
        {
            getout++;
            for (vector<AGENT>::iterator iter = agents.begin(); iter != agents.end(); iter++)
            { // 从vector中删除指定的某一个元素
                if (iter->id == cur_agent->id)
                {
                    avg_max_pressure += agents[i].max_pressure;
                    total_time += agents[i].travel_time;
                    agents.erase(iter);
                    break;
                }
            }
        }

        // 到达gx gy附近，更新path
        if (isArrival(cur_agent) == 1)
        {
            // 按照路径节点行走
            if (cur_agent->n_setp < cur_agent->path.size())
            {
                // 先判断是否走到了预定的节点
                double tmp_gx = cur_agent->path[cur_agent->n_setp][0];
                double tmp_gy = cur_agent->path[cur_agent->n_setp][1];
                double dist = sqrt((cur_agent->x - tmp_gx) * (cur_agent->x - tmp_gx) + (cur_agent->y - tmp_gy) * (cur_agent->y - tmp_gy));
                if (dist <= 0.5)
                {
                    if (cur_agent->n_setp + 1 >= cur_agent->path.size())
                    {
                        cur_agent->gx = cur_agent->temp_gx;
                        cur_agent->gy = cur_agent->temp_gy;
                        continue; // 到目的地了
                    }
                    if (cur_agent->path[cur_agent->n_setp + 1].size() == 0)
                        continue; // 防止意外
                    cur_agent->gx = cur_agent->path[cur_agent->n_setp + 1][0];
                    cur_agent->gy = cur_agent->path[cur_agent->n_setp + 1][1];
                    cur_agent->n_setp++;
                }
            }
        }
    }
}

void Simulator::SFM()
{
    for (size_t i = 0; i < agents.size(); ++i)
    {
        AGENT *cur_agent = &agents[i];
        /*计算无干扰下agent的行为*/

        // 记录更新前位置
        cur_agent->last_x = cur_agent->x;
        cur_agent->last_y = cur_agent->y;

        // 计算预期的方向向量dx、dy(单位化)
        double x0 = cur_agent->x;
        double y0 = cur_agent->y;

        double d0 = sqrt((cur_agent->gx - x0) * (cur_agent->gx - x0) + (cur_agent->gy - y0) * (cur_agent->gy - y0));
        // if (d0 == 0) d0 = 0.0001;
        double dx = v0 * (cur_agent->gx - x0) / d0;
        double dy = v0 * (cur_agent->gy - y0) / d0;

        // 计算加速度向量(实际的速度向量和预期的速度向量之差 除于 预计的加速时间tao = 加速度向量)
        double dax = (dx - cur_agent->vx) / tao;
        double day = (dy - cur_agent->vy) / tao;
        // 加速度为0，行人保持匀速运动

        /*计算agents的相互作用*/
        double total_fx = 0;
        double total_fy = 0;
        for (size_t j = 0; j < agents.size(); ++j)
        {
            if (i == j)
                continue;
            if (agents.size() == 1)
                break;
            AGENT *other_agent = &agents[j];
            double dis = (cur_agent->x - other_agent->x) * (cur_agent->x - other_agent->x) + (cur_agent->y - other_agent->y) * (cur_agent->y - other_agent->y); // 两个agant间的距离
            if (dis > sense_range)
                continue;                                                      // 如果距离大于敏感距离，就表示行人间无影响(我理解是这样)
            compute_agent_force(cur_agent, other_agent, &total_fx, &total_fy); // 计算完的力存在total_fx、total_fy中
        }

        /*计算墙体、障碍物对agents的影响*/
        for (size_t j = 0; j < obstical_lines.size(); j++)
        {
            double crx, cry; // agent到障碍的垂直点坐标
            double diw = Point_to_line_distance(x0, y0, obstical_lines[j].sx, obstical_lines[j].sy, obstical_lines[j].ex, obstical_lines[j].ey, obstical_lines[j].d, &crx, &cry);
            if (diw > sense_range)
                continue;
            // 当0<t0<1时,diw才等于vir_diw
            //  niwx,niwy为标准化的垂直向量， 指向agent,斥力

            double tmpx = abs(x0 - crx) > 1e-10 ? (x0 - crx) : 1e-10;
            double tmpy = abs(y0 - cry) > 1e-10 ? (y0 - cry) : 1e-10;

            double vir_diw = sqrt(tmpx * tmpx + tmpy * tmpy);
            double niwx = tmpx / vir_diw;
            double niwy = tmpy / vir_diw;

            // 计算公式前半部分
            double drw = cur_agent->m / c_mass - diw;                            // 也就是公式中ri-diw
            double fiw_1 = abs(A * exp(drw / B)) > 1e-20 ? A * exp(drw / B) : 0; // 太小的话直接记0
            if (drw > 0)
            { // drw>0表示agent已经足够靠近墙体（障碍）了
                fiw_1 += k1 * drw;
            }
            double fniwx = fiw_1 * niwx;
            double fniwy = fiw_1 * niwy;

            // 计算公式后半部分
            double fiw_kgx = 0;
            double fiw_kgy = 0;
            if (drw > 0)
            {
                double fiw_kg = k2 * drw * (cur_agent->vx * (-niwy) + cur_agent->vy * niwx);

                fiw_kgx = fiw_kg * (-niwy);
                fiw_kgy = fiw_kg * (niwx);
            }

            // 合起来
            total_fx += fniwx - fiw_kgx;
            total_fy += fniwy - fiw_kgy;
        }

        /*根据计算好的力，更新agents*/

        // 加速度变化
        dax = dax + total_fx / cur_agent->m;
        day = day + total_fy / cur_agent->m;

        // 实际速度的变化
        cur_agent->vx = cur_agent->vx + dax * tick;
        cur_agent->vy = cur_agent->vy + day * tick;

        double dv = sqrt(cur_agent->vx * cur_agent->vx + cur_agent->vy * cur_agent->vy);
        if (dv > maxv) // 如果算出的实际速度大于最大速度，重新计算满足条件
        {
            cur_agent->vx = cur_agent->vx * maxv / dv; // 实际速度的变化
            cur_agent->vy = cur_agent->vy * maxv / dv;
        }

        // 位置的变化
        cur_agent->x = cur_agent->x + cur_agent->vx * tick;
        cur_agent->y = cur_agent->y + cur_agent->vy * tick;

        // 记录位置变化微小的agent,为了捕获真正出问题的agent
        if ((abs(cur_agent->last_x - cur_agent->x) + abs(cur_agent->last_y - cur_agent->y) < 1e-5) && ((cur_agent->isFind && cur_agent->isKnown) || (!cur_agent->isFind && !cur_agent->isKnown)))
        {
            cur_agent->record_time++;
        }
        else if (abs(cur_agent->last_x - cur_agent->x) + abs(cur_agent->last_y - cur_agent->y) < 1e-2 && isObstacle(cur_agent->x, cur_agent->y, cur_agent->gx, cur_agent->gy))
        {
            cur_agent->record_time++;
        }
        else if (abs(cur_agent->last_x - cur_agent->x) + abs(cur_agent->last_y - cur_agent->y) < 1e-2 && cur_agent->loading == 0)
        {
            cur_agent->record_time++;
        }
        
    }
}

// 主要迭代过程
void Simulator::step()
{
    time_recoder->reset();
    // 各个状态防卡死的行为
    Stagnate_Avoid();
    // cout<<"time here1: "<<time_recoder->elapsed()<<endl;
    // time_recoder->reset();

    // 1、Random Walk
    randomWalk();
    // cout<<"time here2: "<<time_recoder->elapsed()<<endl;
    // time_recoder->reset();

    // 2、Interaction with Maps
    SearchSign();
    // cout<<"time here3: "<<time_recoder->elapsed()<<endl;
    // time_recoder->reset();

    // 3、Walk torward to destination
    walkToDes();
    // cout<<"time here4: "<<time_recoder->elapsed()<<endl;
    // time_recoder->reset();

    // Interaction with exits
    SearchExits();
    // cout<<"time here5: "<<time_recoder->elapsed()<<endl;
    // time_recoder->reset();

    // social force model
    SFM();
    // cout<<"time here6: "<<time_recoder->elapsed()<<endl;
    // time_recoder->reset();
}

void Simulator::run_simulation()
{

    Patn_SignToEntrance(); // 为每个标识找到到各个出入口的路径

    vector<bool> flg_create; // 用于标记当前哪些行人已经进入场所
    for (size_t i{}; i < all_agents.size(); ++i)
        flg_create.push_back(false);

    for (int i = 0; i < simulation_time; i++)
    {
        // if (i % 1000 == 0 && i)
        // {
        //     std::cout << i << "current agents:" << agents.size() << endl;
        // }

        // 根据行人时间将行人进入场景
        for (size_t j = 0; j < all_agents.size(); ++j)
        {
            if (i >= all_agents[j].create_time && !flg_create[j])
            {
                flg_create[j] = true;
                agents.push_back(all_agents[j]);
            }
        }

        // 根据模型更新行人位置
        step();

        // 记录下时间
        for (size_t j = 0; j < agents.size(); j++)
        {
            agents[j].travel_time += tick;
        }

        // 每秒记录行人的pressure
        if (i % 50 == 0)
        {
            sum_pre = 0;
            for (size_t j = 0; j < agents.size(); j++)
            {
                // t时刻的行人j有两个指标描述压力，密度和周围混乱度(信息熵)。
                double pressure = 0;

                // 1、密度（取半径R=3)
                int R = 3;
                int local_density = 0; // 行人周围的邻居数
                // 2、熵
                // 速度、方向
                int panel_angle[12 + 1] = {0};   // n2 = 12
                int panel_veloity[20 + 1] = {0}; // n1 = 20
                for (size_t k = 0; k < agents.size(); k++)
                {
                    if (agents[j].id == agents[k].id)
                        continue;
                    double dist = sqrt((agents[j].x - agents[k].x) * (agents[j].x - agents[k].x) + (agents[j].y - agents[k].y) * (agents[j].y - agents[k].y));
                    if (dist <= R)
                    { // 统计圈内的行人
                        // 方向
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
                        panel_angle[(int)(theta / (2 * PI / 12))] += 1;
                        // std::cout << agents[k].id<<' '<<(int)(theta / (2 * PI / 12)) << endl;
                        // 速度
                        double velo = sqrt(agents[k].vx * agents[k].vx + agents[k].vy * agents[k].vy);
                        if (velo > 5)
                            velo = 5; // 最大速度5
                        panel_veloity[(int)(velo / (5.0 / 20))] += 1;
                        // std::cout << agents[k].id<<' '<< (int)(velo / (5.0 / 20)) << endl;
                    }
                }

                // 计算熵
                double Ed = 0;
                double Ev = 0;
                if (local_density == 0)
                    continue; // 行人没压力啦
                for (int ii = 0; ii < 12; ++ii)
                {
                    if (panel_angle[ii] == 0)
                        continue;
                    Ed += -((double)panel_angle[ii] / (double)local_density) * log2((double)panel_angle[ii] / (double)local_density);
                }
                for (int ii = 0; ii < 20; ++ii)
                {
                    if (panel_veloity[ii] == 0)
                        continue;
                    Ev += -((double)panel_veloity[ii] / (double)local_density) * log2((double)panel_veloity[ii] / (double)local_density);
                }
                pressure = local_density * Ed * Ev;
                sum_pre += pressure;
                // 存到agent中
                if (pressure > agents[j].max_pressure)
                {
                    agents[j].max_pressure = pressure;
                }
            }
            // 分析每秒各项参数变化
            // 人数变化
            if (saveIndicatorsPerSecond)
                outPressurePerSecond += (to_string((agents.size() == 0 ? 0 : (double)sum_pre / agents.size())) + ',');
        }

        if (saveTracks)
        {
            if (i % 10 == 0)
            {
                *oFile1 << (int)agents.size() << endl;
                for (size_t j = 0; j < agents.size(); j++)
                {
                    *oFile1 << agents[j].x << '\t' << agents[j].y << endl;
                    // cout<< agents[j].x << '\t' << agents[j].y << endl;
                }
            }
        }
    }

    // cout<<"_"<<sum_people<<endl;
    // 输出平均行走时间
    for (size_t j = 0; j < agents.size(); j++)
    {
        total_time += agents[j].travel_time;
    }
    for (size_t j = 0; j < all_agents.size(); j++)
    {
        if (flg_create[j] == false)
            total_time += all_agents[j].travel_time;
    }

    // std::cout << "avg_time is:" << total_time / sum_people << endl;
    avg_time = total_time / sum_people;

    // 输出行人平均最大压力
    for (size_t j = 0; j < agents.size(); j++)
    {
        avg_max_pressure += agents[j].max_pressure;
    }
    // std::cout << "avg_max_pressure is:" << avg_max_pressure / sum_people << endl;
    avg_max_pressure = avg_max_pressure / sum_people;
    pressure = avg_max_pressure;

    // test code:how many agents left ? print their infomation
    // cout << sum_people << endl;
    // for (size_t i{0}; i < agents.size(); ++i)
    // {
    //     AGENT *cur_agent = &agents[i];
    //     // if (cur_agent->record_time && cur_agent->loading == 500)
    //     {
    //         cout << "stag id:" << cur_agent->id << ' ' << cur_agent->isFind << ' ' << cur_agent->isKnown << ' ' << cur_agent->loading << '<' << cur_agent->x << ',' << cur_agent->y << '-' << cur_agent->gx << ',' << cur_agent->gy << '-' << cur_agent->temp_gx << ',' << cur_agent->temp_gy << '>' << cur_agent->path.size() << ' ' << cur_agent->n_setp << ' ' << cur_agent->n_setp_sign << endl; //<<cur_agent->path[0][0]<<' '<<cur_agent->path[0][1]
    //         cout << isObstacle(cur_agent->x, cur_agent->y, cur_agent->gx, cur_agent->gy) << endl;
    //         cout << cur_agent->record_time << ' ' << cur_agent->numOfTry << endl;
    //     }
    // }
}