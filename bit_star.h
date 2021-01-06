/*************************************************
 *
 *
 * This header file for the bit_star for a dubins vehicle amid dynamic obstacles
 *
 * Author: Yi Wang
 *
 * Data:5/19/2020
 *
 * **********************************************/
#ifndef BIT_star_H
#define BIT_star_H
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>
#include <stack>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <chrono>
#include <list>
#include <cstdio>
#include <fstream>
#include <cstring>
#include "obstacle.h"
using namespace std;
typedef pair<int,int> pairs;
struct hash_pair {
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& p) const
    {
        auto hash1 = hash<T1>{} (p.first);
        auto hash2 = hash<T2>{} (p.second);
        return hash1 ^ hash2;
    }
};
struct Dubins_curve{
    float rho;
    float relvar;
    float len;
    float fst_seg;
    float sec_seg;
    float fin_seg;
    float mTime;
    string path_type;
    bool operator==(const Dubins_curve& dc) {
        return (len == dc.len && fst_seg == dc.fst_seg && sec_seg == dc.sec_seg
                && fin_seg == dc.fin_seg && path_type == dc.path_type);
    }
};
class BIT_star{
public: // functions we will use
    BIT_star();               // constructs empty Map
    ~BIT_star();
    struct states{
        float x;
        float y;
        float theta;
        float speed;
        float mTime;
        vector<float> finTime;
        bool operator==(const states& s) {
            return (x == s.x && y == s.y && theta == s.theta && speed == s.speed && mTime == s.mTime && finTime == s.finTime);
        }
    };
    struct dyInformation{
        float x;
        float y;
        float heading;
        float speed;
        float mTime;
        int steps;
    };
    struct ellips{
        float s;
        float t;
        float c;
        float a;
        float slope;
        float angle;
    };
    struct edges{
        int u, v;
        float ux,uy,vx,vy;
    };
    class Dubins{
    public:
        Dubins();

        Dubins( unordered_set<pair<int,int>, hash_pair> ob ): obs(ob){}
        ~Dubins();
        struct Turing{
            float x;
            float y;
            float theta;
            bool operator==(const Turing& t) {
                return (x == t.x && y == t.y && theta == t.theta);
            }
        };

        struct segement{
            float x;
            float y;
            float theta;
        };
        struct Dubins_set{
            float pathLen;
            float lenT;
            float lenP;
            float lenQ;
            string pType;
            bool operator==(const Dubins_set& ds) {
                return ( lenT == ds.lenT && lenP == ds.lenP && lenQ == ds.lenQ && pType == ds.pType && pathLen == ds.pathLen);
            }
        };
        struct compare{
            bool operator()(Dubins_set const & ds, Dubins_set const & db){
                return ds.pathLen > db.pathLen;
            }
        };
        void get_obs(vector<DynamicObstacle > & b) {
            Dobstacle = b;
        }
        void dyobsClear(){
            Dobstacle.clear();
        }
        void startTime(float t) {
            dgetTime = t;
        }
        bool needUpdating(int t);
        bool  getUp() {
            return updateTrue;
        }
        void setUp() {
            updateTrue = false;
        }
        void set_t(int x, int y) {
            xa = x;
            ya = y;
        }
        bool get_cfree() {
            return cfree;
        }
        void set_cfree() {
            cfree = false;
        }
        float getCV(){
            return cValue;
        }
        float get_dycost() {
            return dynaCost;
        }
        void set_dycost_to_zero(){
             dynaCost = 0;
        }
        bool is_same_direct(float moving_dic);
        float dcCost(states cur, float teimIn);
        float q_path(states u, states v, string pathType);
        Dubins_curve Dubins_Optimal_path(states u, states v);
        void Stadying_location(states u, states v, string s);
        void Path_showing(Dubins_curve duc,states u, states v);
        void DyInfoUpdating(DynamicObstacle & dynamic, float t);
        float EstimatedDvalue(states u, states v,float mtime,float vtime ,int index, int des);
        float p_path(states u, states v, string pathType,float d);
        float EucDubinsDistance(BIT_star::states u, BIT_star::states v);
        bool collision_free(BIT_star *bit,states start, float x, float y);
        float t_path(states u, states v, string pathType, float d, float p);
        float DubinsPathLenght(states u, states v, string path_type, float d);
        segement seg_generation(float t, char dir, float phi,float x, float y);
        float dyCost(Dubins_curve duc, states start, states end, states org, int v);
        bool Du_collision_free(BIT_star *bit,Dubins_curve duc, states start, states end);
        void close_to_Obstacles(states start,states end, int sId, float tInvar, float t2);
        void dynamic_location(DynamicObstacle & dynamic, float x, float y, float t, int v);
        float get_probability(DynamicObstacle & dynamic, float x, float y, float t, int v);
        float calculate_angle(DynamicObstacle & dynamic, float x, float y, float t, int v);
        void Generating_dubins_path(BIT_star *bit,Dubins_curve duc, states u, states v,string check);
        float get_cur_possiblity(float x, float y, float dx, float dy, float tc,int index, int dey);
        float ta_value(float sinAplpha, float sinBeta, float cosAlpha, float cosBeta, string pathType, float d);
        void moving_path(float length, char c, float sTha,float sx, float sy, float deltalen, float t, float aspeed);
        float dysegcost(float length, char c, float sTha,float sx, float sy, float deltaLen,float t, float aspeed, int v);
        void drawing_path(BIT_star *bit,float length, char c ,float sTha, float sx, float sy, float deltaLen, string checkStyle,
                          float time, float speed, char chr);
    private:
        int dycheck, xa, ya;
        float alpha,sigma,dynaCost,tpos;
        float beta,penCollide,cValue;
        unordered_set<int > ObID;
        vector<DynamicObstacle> Dobstacle;
        float t,p,q,tanValue,rho,angle,infinity,dgetTime;
        unordered_map<int, vector<pair<float,pair<float,float>>>> dynamic_loc;
        bool ColidHap, collisionDynamic, updateTrue,angleT,esCheck,cfree;
        unordered_set<pair<int,int>, hash_pair> obs;
    };

    // prune the node which its f vlaue greatert than costGoal
    void out_putBE(); // for testing bestEdge
    void parsing_map();
    void initialized();
    states state(int v);
    void showData(int s);
    void set_src_and_goal();
    void Erase_edge(int xm);
    void Prune_edge(int dey);
    bool b_size(int Bucket[]);
    void UpDateQvalue(int xm);
    void building_edge_queue();
    void rgg(int node_numbers);
    void Prune_vertcies(int xm);
    void a_star(int src, int t);
    void expanding_state(int v);
    float Heuristic_value(int v);
    void getGridNeighbors(int s);
    void Erase_queue_Edge(int xm);
    void creating_vertice_queue();
    void lazy_Astar(BIT_star *bit);
    void erase_vertices_infinity();
    bool is_in_old_tree(int index);
    float EucDistance(int s, int g);
    float estimatedComeValue(int v);
    float loc_distance(int u, int v);
    void compute_h_from_goal( int z);
    bool outOfbound(float x, float y);
    void four_sur_goal1(int x, int y);
    float theStar_heuristic(states v);
    void sampling(int num, int count);
    void bit_star(int start, int goal);
    void add_vertices_back_to_sample();
    states Random_state(int xs, int ys);
    void updateStadyTime(int s,float t);
    void updateDynamicObstacles(float t);
    void prune(float cos,float costGoal);
    void Bucket_dijstra(float x, float y);
    float bfs_heuristic(float x, float y);
    bool Beshaman_algorithm(int u , int v);
    void theta_Heuristic(float x, float y);
    bool Theta_collision_free(int u, int v);
    float stateDistance(states v, states w);
    void updateTime(int v, float t,string s);
    void prune_rgg(float cos,float costGoal);
    void dynamic_bit_star(int start, int goal);
    void edges_prune(float cos,float costGoal);
    pair<float, int> four_sur_goal(int x, int y);
    void prune_vertices(float cos,float costGoal);
    float dist_to_four_grid(int x, int y, states v);
    void record_path(BIT_star *bit, int m, string pt);
    void Trajectory_generating(BIT_star *bit,int path);
    float dynamicHeuristicValue(int index, string str);
    bool is_in_ellispe(ellips el, float vx, float vy);
    void Creating_The_Trajectory(BIT_star *bit,int path);
    void ExpandVertex(BIT_star *bit,int v,float costGoal);
    ellips get_ellips(float sx, float sy, float tx, float ty);
    void Agent_and_dyObstacle_Trajectory_showing(int u, int v);
    void A_GetKneighbors(int v, Dubins & dubins, BIT_star *bit);
    void push(vector<pair<float,int>> & heap, float val, int len);
    void K_neighbor(int x, int y, char der, int x1, int y1, int s);
    int newAction_Generating(int size, int ns ,float t, string str);
    DynamicObstacle MotionPrediction(DynamicObstacle & dynamic, float t);
    void creating_edges_queue(int v, float costGoal, string name,vector<int > kn);
    int circle_state(BIT_star *bit, int v, int num, float svar, int size,string algo);
    unordered_set<int>::iterator  remove_state(unordered_set<int>::iterator itrr, int dey,string algo);
    dyInformation get_dyInformation(float px, float py, float heading, int steps, float speed,float t);
    void outputQ(priority_queue< pair<float,int>, vector<pair<float,int>>,greater<pair<float,int>> > QV);
    void rewiring(int v, int dey, float dCost, float dist, float fdist, float hvar,Dubins_curve path_get);
    vector<int > GetKneighbors(BIT_star *bit, int v, unordered_map<int,states> nodeContainer, string str);
    void outputE(priority_queue< pair<float,pairs>, vector<pair<float,pairs>>,greater<pair<float,pairs>> > QE);
    priority_queue< pair<float,int>, vector<pair<float,int>>,greater<pair<float,int>>> Prune_state(priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>> > q, int dey, int index);
    priority_queue< pair<float,int>, vector<pair<float,int>>,greater<pair<float,int>>> Prune_successor(unordered_map<int,float> s, priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>> > m );
    priority_queue< pair<float,int>, vector<pair<float,int>>,greater<pair<float,int>>> Prune_child(priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>> > q,int m );
private: // variables we will use
    bool path_find, found, collisionHappen, dynamic_exits, stateChange,nchange, gfind;
    int source,goal,map_x,map_y,numNodes,range_x,range_y,kneighbors,getTime,nextGoal,MAX, Tcount,tnum,GOAL,_MAX;
    float start_x,start_y,end_x,end_y,infinity,Epsilon, timeRemaining, updateRate, timeInterval,preheading,rtime,wtime;
    ellips el;
    bool *block;
    float *thstar;
    int *thparent,*first;
    int next[9], _tos,Ver[9], sEdge[9];
    float dsEdge[9];
    bool * Hvisited;
    bool * freeB;
    float *tvarl;
    float * ef;
    int W;
    unordered_set<int > freeBlock;
    unordered_map<int,pair<float,float>> startPair;
    unordered_map<int,pair<float,float>> goalPair;
    unordered_set<pair<int,int>, hash_pair> ccnode;
    unordered_set<pair<int,int>, hash_pair> connected;
    unordered_map<int,float> stayCost;
    unordered_map<int,int> parent; // checking the parent
    unordered_map<int,float> cost;// true cost to come
    unordered_map<int,float> scost; // true cost for static come
    unordered_map<int,states> Poses; /* This is for the pose of boat(x,y,theta,speed,time), we will not use the dimension for time here*/
    unordered_map<int,states> bPoses; /* this is for the poses including new sampl */
    unordered_map<int,states> rggData;//states for boat(x,y,theta,speed,time), 5D for a state
    unordered_map<int,int> lookUpPose;// look up the same pose(same (x,y,theta,speed)) which have different sates(rely on time)
    unordered_set<int> station_states;
    unordered_set<int> newGenerated;
    unordered_map<int,unordered_set<int >> Hlist;
    unordered_map<int, bool > visited; // state visited
    unordered_map<int,states> vertices; // cost for every state
    vector<DynamicObstacle > obstacleM;
    unordered_map<int,states> oldvertices;
    vector<DynamicObstacle > orgobstacleM;
    unordered_set<int > tob;
    unordered_map<int,vector<int>> kNeighbors;
    unordered_map<int,vector<int>> kNneighbors;
    unordered_map<int,unordered_set<int>> sameLocation;
    unordered_map<int,unordered_map<int,Dubins_curve>> adj;
    unordered_map<pair<int, int>,Dubins_curve, hash_pair> Edge;
    priority_queue< float, vector<float >,greater< float>> bestCost;
    priority_queue<float, vector<float>,greater< float> >Tvalue ;
    priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>> > q;
    priority_queue< pair<float,int>, vector<pair<float,int>>,greater<pair<float,int>>> BestQVvalue;
    priority_queue< pair<float,pairs>, vector<pair<float,pairs>>,greater<pair<float,pairs>> > BestEvalue; //Motion trees
};
#endif
