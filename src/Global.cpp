#include "Global.h"
#include "Util.h"
#include <string>

using namespace std;

//�������� ȫ�ֱ���
string scene_name = "scene2"; // scene1, scene2, scene3
int n_signages = 4;
int simulation_time = 12000;
int stag_time = (1/tick) * 10; //���˿���ָʾ��ͣ��ʱ��
int Width;
int Height;

bool saveTimeTableAndKeepSame = true;

vector<OBSLINE> obstical_lines;
vector<AGENT> all_agents; //��ʷ���е�������Ϣ
vector<ENTRANCE> entrances;
vector<CPoint> available_points; //��¼�����пɵ���ĵ�
Patharc pathTable;

//��ʼ���õ�ȫ�ֵ�����ʱ����
unordered_map<string, int> hashmap;
int myMap[MAX_Height][MAX_Width] = {};
bool boolMap[MAX_Height][MAX_Width] = {false}; //���ڱ���ѱ����ĵ�

void initial_obstical_lines(string str)
{
    string coordstr;
    ifstream infile(str.c_str(), ios::in);
    OBSLINE obs{};
    if (!infile)
    {
        cout << "reading obstacles info failed" << endl;
        exit(1);
    }
    //��ʼ����
    while (getline(infile, coordstr))
    {
        if (coordstr == "//door_coordinate")
        {
            do
            {
                obs.sign = 1;
                getline(infile, coordstr);
                istringstream in1(coordstr);
                in1 >> obs.sx >> obs.sy;
                getline(infile, coordstr);
                istringstream in2(coordstr);
                in2 >> obs.ex >> obs.ey;
                obstical_lines.push_back(obs);

            } while (getline(infile, coordstr) && coordstr == ".");
        }
        else if (coordstr == "//circle_coordinate")
        {
            while (getline(infile, coordstr) && coordstr != "./")
            {
                double centerx, centery, r;
                istringstream in(coordstr);
                in >> centerx >> centery >> r;

                double a = 0; // atan2(0, r);
                obs.sx = centerx + r;
                obs.sy = centery;
                int Enum = 360;
                for (int j = 0; j < 5; ++j)
                {
                    a = a + PI * 2 * 60 / Enum;
                    obs.ex = centerx + r * cos(a);
                    obs.ey = centery + r * sin(a);
                    obs.sign = 3;
                    obstical_lines.push_back(obs);
                    obs.sx = obs.ex;
                    obs.sy = obs.ey;
                }
                obs.ex = obstical_lines[obstical_lines.size() - 5].sx; //Ϊ�˷�ֹ����㾫������δ�պϣ����һ���߶��ֶ��պ�
                obs.ey = obstical_lines[obstical_lines.size() - 5].sy;
                obstical_lines.push_back(obs);
                // obs.ex = obstical_lines[0].sx;
                // obs.ey = obstical_lines[0].ey;
                // cout << "size:" << obstical_lines.size() << endl;
            }
        }
        else if (coordstr == "//square_coordinate" || coordstr == "//wall_coordinate")
        {
            getline(infile, coordstr);
            istringstream in(coordstr);
            in >> obs.sx >> obs.sy;

            while (getline(infile, coordstr))
            {
                if (coordstr == "." || coordstr == "./")
                {
                    obs.ex = obstical_lines[obstical_lines.size() - 3].sx;
                    obs.ey = obstical_lines[obstical_lines.size() - 3].sy;
                    obs.sign = 3;
                    obstical_lines.push_back(obs);

                    // cout << obstical_lines.size() << endl;;
                    if (coordstr == "./")
                    {
                        break;
                    }
                    else
                    {
                        getline(infile, coordstr);
                        istringstream in(coordstr);
                        in >> obs.sx >> obs.sy;
                    }
                }
                else
                {
                    istringstream temp(coordstr);
                    temp >> obs.ex >> obs.ey;

                    obs.sign = 3;
                    obstical_lines.push_back(obs);
                    obs.sx = obs.ex;
                    obs.sy = obs.ey;
                }
            }
        }
        else if (coordstr[0] == '/')
        {
            continue;
        }
        else
        {
            obs.sign = 0;
            istringstream in(coordstr);
            in >> obs.sx >> obs.sy;

            while (getline(infile, coordstr))
            {
                if (coordstr == "." || coordstr == "./")
                {
                    break;
                }
                else if (coordstr == "--") //���ӷ�����ͬһ�ϰ��ﲻͬ�������ظ�����������
                {
                    getline(infile, coordstr);
                    istringstream in(coordstr);
                    in >> obs.sx >> obs.sy;
                }
                else if (coordstr == "__")
                {
                    getline(infile, coordstr);
                    istringstream in(coordstr);
                    in >> obs.sx >> obs.sy;
                }
                else if (coordstr == ".__")
                {
                    obs.ex = obstical_lines[obstical_lines.size() - 3].sx;
                    obs.ey = obstical_lines[obstical_lines.size() - 3].sy;
                    obstical_lines.push_back(obs);
                }
                else if (coordstr == "_/_")
                {
                    double centerx, centery, r;
                    getline(infile, coordstr);
                    istringstream in(coordstr);
                    in >> centerx >> centery >> r;

                    // double a = 0;// atan2(0, r);
                    // obs.sx = centerx + r;
                    // obs.sy = centery;
                    // int Enum = 360;
                    // for (int j = 0; j < 5; ++j)
                    //{
                    //     a = a + PI * 2 * 60 / Enum;
                    //     obs.ex = centerx + r * cos(a);
                    //     obs.ey = centery + r * sin(a);
                    //     obstical_lines.push_back(obs);
                    //     obs.sx = obs.ex;
                    //     obs.sy = obs.ey;
                    // }
                    // obs.ex = obstical_lines[obstical_lines.size() - 5].sx;//Ϊ�˷�ֹ����㾫������δ�պϣ����һ���߶��ֶ��պ�
                    // obs.ey = obstical_lines[obstical_lines.size() - 5].sy;
                    // obstical_lines.push_back(obs);
                }
                else
                {
                    istringstream temp(coordstr);
                    temp >> obs.ex >> obs.ey;
                    // cout << obs.ex << obs.ey << endl;

                    obstical_lines.push_back(obs);
                    obs.sx = obs.ex;
                    obs.sy = obs.ey;
                }
            }
        }
    }

    if (scene_name == "scene3")
    { //�Դ󳡾���һ������
        // x����550��y����50
        for (size_t i = 0; i < obstical_lines.size(); i++)
        {
            obstical_lines[i].sx += 550;
            obstical_lines[i].sx /= 10;
            obstical_lines[i].ex += 550;
            obstical_lines[i].ex /= 10;
            obstical_lines[i].sy += 50;
            obstical_lines[i].sy /= 10;
            obstical_lines[i].ey += 50;
            obstical_lines[i].ey /= 10;
        }
    }
    //���㳤��
    for (size_t i = 0; i < obstical_lines.size(); ++i)
        obstical_lines[i].d = sqrt((obstical_lines[i].sx - obstical_lines[i].ex) * (obstical_lines[i].sx - obstical_lines[i].ex) +
                                   (obstical_lines[i].sy - obstical_lines[i].ey) * (obstical_lines[i].sy - obstical_lines[i].ey));
}

void init_door(string path1, string path2)
{
    string door;
    string door_sign;
    ifstream infile(path1, ios::in);
    ifstream infile2(path2, ios::in);
    while (getline(infile, door))
    {

        if (door == "//entrance")
        {
            int id = 0;
            while (getline(infile, door))
            {
                if (door == "./")
                {
                    break;
                }
                istringstream left(door);
                ENTRANCE temp_e{};
                temp_e.door_id = id++;
                left >> temp_e.sx >> temp_e.sy;
                getline(infile, door);
                istringstream right(door);
                right >> temp_e.ex >> temp_e.ey;

                getline(infile2, door_sign);
                istringstream sign(door_sign);
                sign >> temp_e.greedy_sign_x >> temp_e.greedy_sign_y;

                if (scene_name == "scene3")
                {
                    temp_e.sx += 550;
                    temp_e.sx /= 10;
                    temp_e.sy += 50;
                    temp_e.sy /= 10;
                    temp_e.ex += 550;
                    temp_e.ex /= 10;
                    temp_e.ey += 50;
                    temp_e.ey /= 10;

                    temp_e.greedy_sign_x += 550;
                    temp_e.greedy_sign_x /= 10;
                    temp_e.greedy_sign_y += 50;
                    temp_e.greedy_sign_y /= 10;

                    //�޸�bug��
                    if (floor(temp_e.ex) == 25 && floor(temp_e.ey) == 66)
                    {
                        temp_e.ex = temp_e.ex + 1;
                    }
                    if (floor(temp_e.ex) == 40 && floor(temp_e.ey) == 66)
                    {
                        temp_e.ex = temp_e.ex + 1;
                    }
                    if (floor(temp_e.ex) == 25 && floor(temp_e.ey) == 74)
                    {
                        temp_e.ex = temp_e.ex + 1;
                    }
                    temp_e.en_x = 0.5 * (temp_e.sx + temp_e.ex);
                    temp_e.en_y = 0.5 * (temp_e.sy + temp_e.ey);

                    if (floor(temp_e.en_x) == 40 && floor(temp_e.en_y) == 73)
                    {
                        temp_e.en_x = 42;
                    }
                    if (floor(temp_e.en_x) == 74 && floor(temp_e.en_y) == 69)
                    {
                        temp_e.en_x = 75;
                    }
                }

                if (scene_name == "scene1" || scene_name == "scene2")
                {
                    temp_e.en_x = 0.5 * (temp_e.sx + temp_e.ex);
                    temp_e.en_y = 0.5 * (temp_e.sy + temp_e.ey);
                }

                entrances.push_back(temp_e);
            }
        }
    }
}

void initMap(int table[][MAX_Width])
{
    //ȷ�����񻯳���
    if (scene_name == "scene3")
    {
        Width = 120;
        Height = 120;
    }
    else if (scene_name == "scene1" || scene_name == "scene2")
    {
        Width = 50;
        Height = 50;
    }
    else
    {
        cout << "unknown scene!" << endl;
        exit(1);
    }

    //�����ϰ�
    for (size_t i = 0; i < obstical_lines.size(); ++i)
    {
        int cur_x, cur_y;
        cur_x = (int)obstical_lines[i].sx;
        cur_y = (int)obstical_lines[i].sy;

        if (obstical_lines[i].sx == obstical_lines[i].ex && obstical_lines[i].sy != obstical_lines[i].ey)
        {
            if (obstical_lines[i].ey > obstical_lines[i].sy)
            {
                while (cur_y <= obstical_lines[i].ey)
                {
                    table(cur_x, cur_y) = 1;
                    cur_y++;
                }
            }
            else
            {
                while (cur_y >= obstical_lines[i].ey)
                {
                    table(cur_x, cur_y) = 1;
                    cur_y--;
                }
            }
        }
        else if (obstical_lines[i].sx != obstical_lines[i].ex && obstical_lines[i].sy == obstical_lines[i].ey)
        {
            if (obstical_lines[i].ex > obstical_lines[i].sx)
            {
                while (cur_x <= obstical_lines[i].ex)
                {
                    table(cur_x, cur_y) = 1;
                    cur_x++;
                }
            }
            else
            {
                while (cur_x >= obstical_lines[i].ex)
                {
                    table(cur_x, cur_y) = 1;
                    cur_x--;
                }
            }
        }
        else if (obstical_lines[i].sx != obstical_lines[i].ex && obstical_lines[i].sy != obstical_lines[i].ey)
        {
            if (obstical_lines[i].sx <= 0.0001)
                continue;
            if (obstical_lines[i].sx >= 300)
                continue;
            int dx = obstical_lines[i].ex - obstical_lines[i].sx;
            int dy = obstical_lines[i].ey - obstical_lines[i].sy;

            double factor = abs(dx) > abs(dy) ? dx : dy;
            double incre_x = dx / factor;
            double incre_y = dy / factor;

            table(cur_x, cur_y) = 1;
            for (int i = 1; i <= factor; ++i)
            {
                cur_x = round(cur_x + incre_x);
                cur_y = round(cur_y + incre_y);
                table(cur_x, cur_y) = 1;
            }
        }
    }

    if (scene_name == "scene3")
    {
        //�ֶ��޸�һЩ����
        table(61, 26) = 1;
        table(61, 27) = 1;
        table(61, 28) = 1;
        table(61, 29) = 1;
        table(61, 30) = 1;
        table(62, 26) = 1;
        table(62, 27) = 1;
        table(62, 28) = 1;
        table(62, 29) = 1;
        table(62, 30) = 1;

        //�ѵ��ݾ����� ��
        table(28, 70) = 1;
        table(29, 70) = 1;
        table(28, 69) = 1;
        table(29, 69) = 1;
        //��
        table(73, 71) = 1;
        table(74, 71) = 1;
        table(73, 70) = 1;
        table(74, 70) = 1;
        table(73, 69) = 1;
        table(74, 69) = 1;
        table(73, 68) = 1;
        table(74, 68) = 1;
        //��1
        table(56, 79) = 1;
        table(57, 79) = 1;
        table(58, 79) = 1;
        table(59, 79) = 1;

        //��1
        table(56, 59) = 1;
        table(57, 59) = 1;
        table(58, 59) = 1;
        table(59, 59) = 1;
        //��2
        table(56, 39) = 1;
        table(57, 39) = 1;
        table(58, 39) = 1;
        table(59, 39) = 1;
        //��3
        table(56, 16) = 1;
        table(57, 16) = 1;
        table(58, 16) = 1;
        table(59, 16) = 1;
    }
    else if (scene_name == "scene1" || scene_name == "scene2")
    {
        table(0, 19) = 1;
        table(0, 20) = 1;
        table(0, 21) = 1;
        table(19, 0) = 1;
        table(20, 0) = 1;
        table(21, 40) = 1;
        table(19, 40) = 1;
        table(20, 40) = 1;
        table(21, 0) = 1;
        table(40, 19) = 1;
        table(40, 20) = 1;
        table(40, 21) = 1;
    }
}

//���ڲ���agent
void creatAgent(int entrance_id, int create_time, int exit_id)
{
    struct AGENT agent;
    agent.id = all_agents.size() + 1;
    agent.create_time = create_time;
    agent.entrance_id = entrance_id;
    agent.exit_id = exit_id;

    //�����������agent.x y
    agent.x = entrances[entrance_id].sx < entrances[entrance_id].ex ? entrances[entrance_id].sx + randval(0, 1) * (entrances[entrance_id].ex - entrances[entrance_id].sx) : entrances[entrance_id].ex + randval(0, 1) * (entrances[entrance_id].sx - entrances[entrance_id].ex);
    agent.y = entrances[entrance_id].sy < entrances[entrance_id].ey ? entrances[entrance_id].sy + randval(0, 1) * (entrances[entrance_id].ey - entrances[entrance_id].sy) : entrances[entrance_id].ey + randval(0, 1) * (entrances[entrance_id].sy - entrances[entrance_id].ey);

    //��������յ�
    agent.lgx = entrances[exit_id].sx < entrances[exit_id].ex ? entrances[exit_id].sx + 0.5 * (entrances[exit_id].ex - entrances[exit_id].sx) : entrances[exit_id].ex + randval(0, 1) * (entrances[exit_id].sx - entrances[exit_id].ex);
    agent.lgy = entrances[exit_id].sy < entrances[exit_id].ey ? entrances[exit_id].sy + 0.5 * (entrances[exit_id].ey - entrances[exit_id].sy) : entrances[exit_id].ey + randval(0, 1) * (entrances[exit_id].sy - entrances[exit_id].ey);

    agent.gx = agent.lgx;
    agent.gy = agent.lgy;

    //��ʼ�ٶȺ���������
    agent.vx = randval(-1, 1);
    agent.vy = randval(-1, 1);

    agent.m = gaussrand(65, 10);
    agent.FOV = 1.1 * PI;
    agent.n_setp = 0;
    agent.isFind = false;
    agent.isKnown = false;
    agent.sign_id = -1;

    agent.flag_create = 0;
    agent.travel_time = 0;
    agent.max_pressure = 0;

    agent.last_x = agent.x;
    agent.last_y = agent.y;
    agent.record_time = 0;
    agent.loading = 0;
    agent.numOfTry = 0;

    agent.temp_gx = agent.gx;
    agent.temp_gy = agent.gy;
    agent.n_setp = 0;
    agent.n_setp_sign = 0;

    //��ʼ��map
    for (int i = 0; i < Height; ++i)
        for (int j = 0; j < Width; ++j)
            agent.map[i][j] = 0;

    //�������
    all_agents.push_back(agent);
}

//��ʼ��������Ϣ()
void init_pedestrian(string traffic_path, string timetable_path)
{
    // 1�������������
    ifstream ifile(traffic_path, ios::in);
    if (ifile)
    {
        cout << "reading entrance traffic info" << endl;
        //��ʼ����
        string traffic;
        size_t i = 0;
        while (getline(ifile, traffic))
        {
            istringstream intr(traffic);
            if (i >= entrances.size())
            {
                cout << "������Ϣ��ȡ�����ļ�������볡��ʵ���������ƥ��" << endl;
            }
            else
            {
                intr >> entrances[i++].generate_v;
            }
        }
    }
    else
    {
        cout << "��������ļ������ڣ�������̬�ֲ�Ԥ������Զ������ļ�" << endl;

        ofstream ofile(traffic_path, ios::out | ios::trunc);

        //������̬�ֲ�Ϊÿ������趨��������
        default_random_engine generator;
        normal_distribution<double> distribution(1.5, 0.5);
        for (size_t i = 0; i < entrances.size(); ++i)
        {
            double number = distribution(generator);
            if (number < 0)
                continue;
            entrances[i].generate_v = number;
            ofile << number << endl;
        }
        ofile.close();
    }
    ifile.close();

    // 2���������ٵõ����˵���Ϣ������ʱ�䡢Ŀ�ĵء�
    if (saveTimeTableAndKeepSame)
    {
        ifile.open(timetable_path, ios::in);
        if (ifile)
        {
            cout << "reading pedenstrains timetable" << endl;
            string table;
            while (getline(ifile, table))
            {
                istringstream inta(table);
                int en_id, t, ex_id;
                inta >> t >> en_id >> ex_id;
                creatAgent(en_id, t, ex_id);
            }
        }
        else
        {
            int generate_people_pertime = 100; //ÿgenerate_people_pertime/50������һ������ 250
            int cur_sum_people = 0;
            int need_sum_people = 600;
            cout << "���˵���ʱ����ļ������ڣ����ݲ��ɷֲ�Ԥ������Զ������ļ�" << endl;

            ofstream ofile(timetable_path, ios::out);
            for (int i = 0; i < simulation_time; i++)
            {
                //���ɷֲ�
                if (i <= 4000 && i % generate_people_pertime == 0)
                { //ÿn�ε�����ʾ1�� tick=0.02  n = 1/tick;
                    for (size_t j = 0; j < entrances.size(); ++j)
                        entrances[j].num = 0;
                    //��ڽ���
                    for (size_t j = 0; j < entrances.size(); ++j)
                    {
                        //�����ֵΪ1��ƽ��ÿ�����һ����
                        int n_agents = poisson(entrances[j].generate_v);
                        entrances[j].num = n_agents;
                    }
                }

                if (i == 4001)
                {
                    for (size_t j = 0; j < entrances.size(); ++j)
                        entrances[j].num = 0;
                }

                for (size_t j = 0; j < entrances.size(); ++j)
                {
                    if (entrances[j].num == 0)
                        continue;
                    if ((i % generate_people_pertime) % (generate_people_pertime / entrances[j].num) == 0)
                    {
                        int exit_id{};
                        do
                        {
                            exit_id = (int)randval(0, entrances.size());
                        } while (exit_id == entrances[j].door_id || exit_id >= (int)entrances.size()); //|| entrances[exit_id].generate_v != 0
                        cur_sum_people++;
                        ofile << i << ' ' << entrances[j].door_id << ' ' << exit_id << endl;
                        if (cur_sum_people == need_sum_people)
                            break;
                    }
                }
                if (cur_sum_people == need_sum_people)
                    break;
            }
            ofile.close();
            exit(1);
        }
        ifile.close();
    }
}

bool CanReach(int x, int y)
{
    if (scene_name == "scene1" || scene_name == "scene2")
    {
        if (x <= 0 || x >= 40)
            return false;
        if (y <= 0 || y >= 40)
            return false;
    }

    return myMap[x][y] == 0;
}

/*�ҳ����е㣬��ɢ*/
void BFS_findponits(int x, int y)
{
    if (scene_name == "scene3")
    {
        if (x >= 120 || y >= 120 || x <= 0 || y <= 0)
            return;
    }
    else if (scene_name == "scene1" || scene_name == "scene2")
    {
        if (x >= 40 || y >= 40 || x <= 0 || y <= 0)
            return;
    }

    if (boolMap[x][y] == true)
    {
        return;
    }
    else
    {
        available_points.push_back(CPoint(x, y));
        boolMap[x][y] = true;
        for (int tempx = x - 1; tempx <= x + 1; tempx++)
        {
            for (int tempy = y - 1; tempy <= y + 1; tempy++)
            {
                if (CanReach(tempx, tempy))
                {
                    BFS_findponits(tempx, tempy);
                }
            }
        }
    }
}

void initAvailablePoints()
{
    available_points.clear();
    //һ�����е��ڿ�ʼ
    if (scene_name == "scene3")
        BFS_findponits(1, 89);
    else if (scene_name == "scene1" || scene_name == "scene2")
        BFS_findponits(1, 1);
}

void ReadTable(ifstream &ifile, string shortPath)
{
    if (!ifile)
    {
        cout << "ȫ��·���ļ������ڣ��쳣�˳�" << endl;
        exit(1);
    }
    cout << "reading Multi-source shortest path table" << endl;
    string table;
    for (size_t i = 0; i < available_points.size(); ++i)
    {

        getline(ifile, table);
        istringstream inta(table);
        int j = 0;
        ;
        while (inta >> pathTable[i][j])
        {
            j++;
        }
    }
    ifile.close();
}

//��ʼ��door��obstacles
void initialize()
{
    cout << "==========initial start============" << endl;
    //��ʼ��obstacles,���뻷����Ϣ
    cout << "reading obstacles info" << endl;
    // string path = "/home/cyx/wayfindmap_allocation_cmake/config/scene1_coordinate.txt";
    string path = "/home/cyx/wayfindmap_allocation_cmake/config/" + scene_name + "_coordinate.txt";
    initial_obstical_lines(path);

    //��ʼ�����λ�õ���Ϣ
    cout << "reading entrance info" << endl;
    string path1 = "/home/cyx/wayfindmap_allocation_cmake/config/" + scene_name + "_entrance.txt";
    string path2 = "/home/cyx/wayfindmap_allocation_cmake/config/" + scene_name + "_greedy_allocation.txt";
    init_door(path1, path2);

    //���񻯵�ͼ
    cout << "initialize grid map" << endl;
    initMap(myMap);

    //�������˿��е�Ŀ�ĵ�
    cout << "finding all feasibile path points: ";
    initAvailablePoints();
    cout << available_points.size() << endl;

    //��ʼ����������Լ����˵�����ʱ��
    string traffice_path = "/home/cyx/wayfindmap_allocation_cmake/config/" + scene_name + "_entrance_traffic.txt";
    string timetable_path = "/home/cyx/wayfindmap_allocation_cmake/config/" + scene_name + "_pedenstrains_timetable.txt";
    init_pedestrian(traffice_path, timetable_path);

    //����·��ͼ����ȡȫ������·����Ϣ
    for (size_t id = 0; id < available_points.size(); ++id)
    {
        string pos_id = to_string(available_points[id].X) + "," + to_string(available_points[id].Y);
        hashmap[pos_id] = id;
    }
    // CreateMGraph(&G);
    string shortPath = "/home/cyx/wayfindmap_allocation_cmake/config/" + scene_name + "_shortPathTable.txt";
    ifstream ifile(shortPath, ios::out);
    ReadTable(ifile, shortPath);
    cout << "==========initial end============" << endl;
}

CPoint *FindPath_staticFloyd(const CPoint& start, const CPoint& end)
{
    //����start��endȷ��id
    string pos_id = to_string(start.X) + "," + to_string(start.Y);
    int start_id = hashmap[pos_id];
    pos_id = to_string(end.X) + "," + to_string(end.Y);
    int end_id = hashmap[pos_id];

    //����id��ѯ·��
    int k = pathTable[start_id][end_id];
    CPoint *HeadList = new CPoint(-1, -1);
    HeadList->next = nullptr;
    CPoint *cur_ptr = HeadList;
    while (k != end_id)
    {
        cur_ptr->next = new CPoint(available_points[k].X, available_points[k].Y);
        cur_ptr = cur_ptr->next;
        k = pathTable[k][end_id];
    }
    cur_ptr->next = new CPoint(available_points[end_id].X, available_points[end_id].Y);
    cur_ptr = cur_ptr->next;
    cur_ptr->next = nullptr;

    if (HeadList->next)
    {
        CPoint *tmp = HeadList;
        HeadList = HeadList->next;
        delete tmp;
    }
    return HeadList;
}

void FindPath_Agent_staticFloyd(AGENT *cur_agent, int desx, int desy)
{
    int s_x = cur_agent->x;
    int s_y = cur_agent->y;
    int e_x = desx;
    int e_y = desy;

    //��ʼλ�ò�Ӧ�����ϰ���
    if(myMap[s_x][s_y]==1){
        surroundings_feasibility_point(cur_agent->x,cur_agent->y,s_x,s_y);
    }

    //�Ż����˺��յ��غϣ�ֱ�Ӹ�ֵ
    if (s_x == e_x && s_y == e_y)
    {
        vector<double> t;
        t.clear();
        t.push_back((double)s_x + 0.5);
        t.push_back((double)s_y + 0.5);
        cur_agent->path.push_back(t);
        return;
    }

    CPoint start(s_x, s_y);
    CPoint end(e_x, e_y);
    CPoint *parent = FindPath_staticFloyd(start, end);

    vector<double> temp;
    temp.clear();
    while (parent)
    {
        temp.clear();
        temp.push_back((double)parent->X);
        temp.push_back((double)parent->Y);
        cur_agent->path.push_back(temp);
        CPoint *temp_ptr = parent;
        parent = parent->next;
        delete temp_ptr;
    }
    // reverse(cur_agent->path.begin(), cur_agent->path.end());

}

void FindPath_Agent_staticFloyd(AGENT *cur_agent, int curx, int cury, int desx, int desy)
{
    int s_x = curx;
    int s_y = cury;
    int e_x = desx;
    int e_y = desy;

    //��ʼλ�ò�Ӧ�����ϰ���
    if(myMap[s_x][s_y]==1){
        surroundings_feasibility_point(s_x,s_y,s_x,s_y);
    }

    //�Ż����˺��յ��غϣ�ֱ�Ӹ�ֵ
    if (s_x == e_x && s_y == e_y)
    {
        vector<double> t;
        t.clear();
        t.push_back((double)s_x + 0.5);
        t.push_back((double)s_y + 0.5);
        cur_agent->path.push_back(t);
        return;
    }

    CPoint start(s_x, s_y);
    CPoint end(e_x, e_y);
    CPoint *parent = FindPath_staticFloyd(start, end);

    vector<double> temp;
    temp.clear();

    while (parent)
    {
        temp.clear();
        temp.push_back((double)parent->X);
        temp.push_back((double)parent->Y);
        cur_agent->path.push_back(temp);
        CPoint *temp_ptr = parent;
        parent = parent->next;
        delete temp_ptr;
    }
}

int left_right(point a, point b, double x, double y)
{
    double t;
    a.x -= x;
    b.x -= x;
    a.y -= y;
    b.y -= y;
    t = a.x * b.y - a.y * b.x;
    return t == 0 ? 0 : t > 0 ? 1
                              : -1;
}

//���ܣ��߶�c,d��ֱ��a,b�Ƿ��ཻ
bool intersect1(point a, point b, point c, point d)
{
    if (left_right(a, b, c.x, c.y) == 1 && left_right(a, b, d.x, d.y) == -1)
        return true;
    if (left_right(a, b, c.x, c.y) == -1 && left_right(a, b, d.x, d.y) == 1)
        return true;
    return false;
    // return  left_right(a, b, c.x, c.y) ^ left_right(a, b, d.x, d.y) == -2;
}

//���ܣ��ж��߶�c,d���߶�a,b�Ƿ��ཻ
bool intersect(point a, point b, point c, point d)
{
    return intersect1(a, b, c, d) && intersect1(c, d, a, b);
}

bool isObstacle(double sx, double sy, double ex, double ey)
{
    point A1(sx, sy);
    point B1(ex, ey);
    for (size_t i = 0; i < obstical_lines.size(); i++)
    {
        point C(obstical_lines[i].sx, obstical_lines[i].sy);
        point D(obstical_lines[i].ex, obstical_lines[i].ey);
        if (intersect(A1, B1, C, D))
            return true;
    }
    return false;
}

void surroundings_feasibility_point(double cur_x, double cur_y, int &dec_x, int &dec_y)
{
    if (myMap[(int)cur_x][(int)cur_y] == 0)
    {
        dec_x = cur_x;
        dec_y = cur_y;
        return;
    }
    else
    {
        //�ҵ�mymap��ʶΪ��������ĸ����ص�,�Ҳ����赲��
        bool flag = false;
        int itr = 1;
        while (itr != 4)
        {
            for (int x = cur_x - itr; x <= cur_x + itr; x++)
            {
                for (int y = cur_y - itr; y <= cur_y + itr; y++)
                {
                    if (x < 0 || x >= Height || y < 0 || y >= Width)
                        continue;
                    if (myMap[x][y] == 1)
                        continue;
                    if (isObstacle(cur_x, cur_y, x, y))
                        continue;
                    //������ɵ�����ڵ�֮�����ϰ�Ҳcontinue(����д�����ж�����������ϰ��ĺ���)
                    flag = true;
                    dec_x = x;
                    dec_y = y;
                }
            }
            if (flag)
                break;
            itr++;
        }
        return;
    }
}

bool cmp2(const ENTRANCE left, const ENTRANCE right)
{
    return left.generate_v > right.generate_v;
}

//�Ըõ��ж��Ƿ���С�
bool isFeasible(int x, int y) {
    if (!CanReach(x, y)) return false; //���ϰ�����
    for (size_t i = 0; i < available_points.size(); ++i) {
        CPoint P = available_points[i];
        if (x == P.X && y == P.Y) return true; //�õ����
    }
    return false; //�ڱ߽���
}

//���һ���㣬����õ�λ���ϰ��ڻ�߽��У���Ż�һ������λ�ã���λ��Ϊ���н⡣
void Rectify(double orgin_x, double orgin_y, int& r_x, int& r_y) {
    //1�����ȴ���Χѡ����е㡣
    //2����Ҫ�Ըõ��ж��Ƿ��ڱ߽��⡣
    //3�����ظõ�
    double min = 10000;
    int itr = 1;
    
    while (itr != 4) {
        for (int x = (int)orgin_x - itr; x <= (int)orgin_x + itr; x++) {
            for (int y = (int)orgin_y - itr; y <= (int)orgin_y + itr; y++) {             
                if (x<0 || x>Height || y<0 || y>Width) continue;
                if (myMap[x][y] == 1) continue;
                if (isFeasible(x, y)) {   
                    //������ɵ�����ڵ�֮�����ϰ�Ҳcontinue(����д�����ж�����������ϰ��ĺ���)
                    double dist = (x - (int)orgin_x) * (x - (int)orgin_x) + (y - (int)orgin_y) * (y - (int)orgin_y);
                    if (dist < min) {
                        min = dist;
                        r_x = x;
                        r_y = y;
                    }
                }
                
            }
        }
        if (min != 10000) break;
        itr++;
    }
    
    if(min == 10000) cout << "error!�Ҳ������н⣡ " << orgin_x << ' ' << orgin_y << endl;

}