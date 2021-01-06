#include "bit_star.h"
void BIT_star::outputE(priority_queue< pair<float,pairs>, vector<pair<float,pairs> >,greater<pair<float,pairs>> > QE) {
    while(!QE.empty()) {
        cerr << QE.top().first << "  " << QE.top().second.first << "  " << QE.top().second.second << " (" << state(QE.top().second.second).x << "," << state(QE.top().second.second).y << "), time is: " << state(QE.top().second.second).mTime << endl;
        QE.pop();
    }
}
void BIT_star::outputQ(priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>> > QV){
    while(!QV.empty()) {
        cerr << QV.top().first << " " << QV.top().second << "   (" << state(QV.top().second).x << "," << state(QV.top().second).y << "),   " <<state(QV.top().second).mTime << "  " << parent[QV.top().second]  << "  " << cost[QV.top().second]<< endl;
        QV.pop();
    }
}
BIT_star::BIT_star() {
    //INITIALIZE private elements
    W = 2;
    source =0;
    goal = 1;
    tnum = 0;
    collisionHappen = false;
    path_find = false;
    nchange = false;
    found = false;
    start_x = 0;
    start_y = 0;
    GOAL    = 0;
    end_x   = 0;
    end_y   = 0;
    map_x   = 0;
    map_y   = 0;
    wtime   = 0;
    rtime   = 0;
    _tos    = 1;
    gfind   = false;
    getTime = 0;
    next[9] = {-1};
    Ver[9] = {-1};
    sEdge[9] = {-1};
    dsEdge[9] = {-1};
    numNodes = 0;
    nextGoal = 0;
    kneighbors = 0;
    updateRate = 1;
    preheading = 0;
    timeRemaining = 0;
    Epsilon = 0.0000001;
    _MAX = numeric_limits<int>::max();
    infinity = numeric_limits<float>:: infinity();
}

BIT_star::~BIT_star(){
    delete [] block;
    delete [] thstar;
    delete [] thparent;
    delete [] Hvisited;
    delete [] ef;
    delete [] tvarl;
    delete [] first;
};

BIT_star::dyInformation BIT_star::get_dyInformation(float x, float y, float heading, int steps, float speed, float t) {
    dyInformation dyn;
    float ddx = cos(heading)*speed*(steps) + x, ddy = sin(heading) * speed * (steps) + y;
    dyn.x = ddx;
    dyn.y = ddy;
    dyn.heading = heading;
    dyn.steps = steps;
    dyn.mTime = steps + t;
    return dyn;
}
void BIT_star::parsing_map() {
    int xs=0,ys=0, dynamicNum =0;
    char chr= '\0';
    int src = 0, t = 0;
    cin >> xs>>ys >> dynamicNum;
    map_x = xs;
    map_y = ys;
    if(dynamicNum !=0) {
        int tm =0;
        for(int i =0; i < dynamicNum; i++) {
            float dynx, dyny, instruction, steps, mtime;
            float heading, speed;
            cin >> dynx >> dyny >> instruction;
            DynamicObstacle dyobs(dynx,dyny,instruction);
            for(int j =0; j < instruction; j++) {
                cin >> heading >> speed >> steps;
                if(j == 0) {
                    dyobs.instructions.emplace_back(dynx,dyny,heading,speed,steps,0);
                }
                else {
                    float px,py,pheading,psteps,pspeed,ptime;
                    px = dyobs.instructions[j-1].x;
                    py = dyobs.instructions[j-1].y;
                    pheading = dyobs.instructions[j-1].heading;
                    pspeed = dyobs.instructions[j-1].speed;
                    ptime = dyobs.instructions[j-1].mTime;
                    psteps = dyobs.instructions[j-1].steps;
                    dyInformation dyi= get_dyInformation(px,py,pheading,psteps,pspeed,ptime);
                    dyobs.instructions.emplace_back(dyi.x,dyi.y,heading,speed,steps,dyi.mTime);
                }
            }
            dyobs.x = dyobs.instructions[0].x;
            dyobs.y = dyobs.instructions[0].y;
            obstacleM.push_back(dyobs);
        }

    }
    int p= xs<<16 | ys;
    //ccnode.resize(_MAX);
    //connected.resize(_MAX);
    thstar = new float[p];
    ef = new float[_MAX];
    tvarl = new float[p];
    block = new bool[p];
    Hvisited = new bool[p];
    thparent = new int[p];
    first = new int[p];
    corner = new bool[p];
    for(int i = ys-1; i>=0;i--) { //row y
        for(int j = 0; j<xs;j++) { // col x
            cin >> chr;
            int x = j, y = i;
            if(chr == '#') {
                  p = j<<16 | i;
                  block[p] = true;
            }
            else {
                p = j<<16 | i;
                block[p] = false;
            }
        }
    }
    orgobstacleM = obstacleM;
    cin >> start_x >> start_y;
    cin >> end_x >> end_y;
    cerr << "starting the thetaStar " << endl;
    clock_t sT = clock();
    //NANYA(end_x,end_y);
    //theta_Heuristic(end_x,end_y);
    //Bucket_dijstra(end_x,end_y);
    //bfs_heuristic(end_x,end_y);
    clock_t et = clock();
    rtime = (float) (et-sT)/CLOCKS_PER_SEC;
    cerr << "theta_star takes " << rtime << " s" <<endl;
    cout << rtime;
    /*for(int y =ys-1; y>=0;y--) {
        for(int x =0; x <xs; x++) {
                int p = x<<16 | y;
             if(!block[p]) {
                 cerr << thstar[p]  <<" :at(" << x << "," << y << ") ";
             }
             else
                 cerr << "# ";
        }
        cerr << endl;
    }*/
  // exit(-1);
}

BIT_star::states BIT_star::Random_state(int x, int y) {
    states sta;
    sta.x = ((float) x)*rand()/(RAND_MAX);
    sta.y = ((float) y)*rand()/(RAND_MAX);
    sta.theta = ((float ) 2*M_PI)*rand()/(RAND_MAX);
    sta.speed =2.5;
    return sta;
}

BIT_star::states BIT_star::state(int v) {
    if(rggData.find(v) != rggData.end())
        return rggData[v];
    return vertices[v];
}
void BIT_star::updateTime(int v, float t, string s) {
    if(rggData.find(v) != rggData.end()) {
        rggData[v].mTime = t;
        if(s == "astar")
            vertices[v].mTime = t;
    }
    else
        vertices[v].mTime = t;
}
void BIT_star::sampling(int num, int count) {
    while(count < num) {
        if(rggData.find(count) == rggData.end()) {
            states tmp = Random_state(map_x,map_y);
            float escost = sqrt(pow((state(source).x - tmp.x),2) + pow((state(source).y-tmp.y),2)) +theStar_heuristic(tmp);//sqrt(pow((state(goal).x- tmp.x),2) + pow((state(goal).y-tmp.y),2))
            int lx = tmp.x,ly = tmp.y;
            int varr = lx<<16 | ly;
            if( !block[varr] && escost < scost[1]) { // is_in_ellispe(el,tmp.x,tmp.y)Hcost[varr] tob.find(varr) == tob.end()
                //cerr <<  count << "  " << tmp.x << "  " << tmp.y << endl;
                rggData[count] = tmp;
                Poses[count] = tmp;
                count++;
            }
        }
    }
}

float BIT_star::EucDistance(int s, int t) {
    states st = state(s);
    states g  = state(t);
    return sqrt(pow((st.x-g.x),2) + pow((st.y-g.y),2));
}

float BIT_star::loc_distance(int u, int v) {
    int vx = v>>16;
    int vy = v%(1<<16);
    /*if(u == -1) {
        float dist  = sqrt((vx-end_x)*(vx-end_x)+(vy-end_y)*(vy-end_y));
        return dist;
    }*/
    int ux = u>>16;
    int uy = u%(1<<16);
    return sqrt((ux - vx)*(ux - vx) + (uy - vy)*(uy - vy));
}
void BIT_star::K_neighbor(int x, int y, char der, int x1, int y1, int s) {
    int loc = x<<16 | y;
    if(block[loc] == false && Hvisited[loc] ==false&& x >= 0 && x <map_x && y >=0 && y < map_y ) {//North
        if(der != 'd') {
            //next[_tos] = first[s];
            //first[s] = _tos;
            Ver[_tos] = loc;
            //sEdge[_tos] = 1;//loc_distance(s,loc)
            dsEdge[_tos] = 1;
            _tos++;
           // int len = tvarl[s] +1;
            //tvarl[loc] = len;
            //thstar[loc] = len/1.41;//1.39446
            //if(tvarl[s] >len) {
            //Hvisited[loc] = true;
            //bfs.push_back(loc);
        }
        else{
            int tx = (x+x1)/2, ty = (y+y1)/2;
            int tloc = tx<<16 | ty;
            if(block[tloc] == false){
                //next[_tos] = first[s];
                //first[s] = _tos;
                Ver[_tos] = loc;
                //sEdge[_tos] = 1.414;//loc_distance(s,loc)
                dsEdge[_tos] = 1.41421356237;
                _tos++;
            }
        }
    }
    else if(block[loc] && corner[loc] && Hvisited[loc] ==false) {
        if(der != 'd')
            sEdge[_tos] = 1;
        else
        {
            int tx = (x+x1)/2, ty = (y+y1)/2;
            int tloc = tx<<16 | ty;
            if(block[tloc])
                return;
            sEdge[_tos] = 1.414214;
        }
        Ver[_tos] = loc;
        _tos++;
    }
}
void BIT_star::getGridNeighbors(int s) {
    if(s == _MAX) {
        int x = end_x, y = end_y;
        four_sur_goal1(x,y);
        four_sur_goal1(x,y+1);
        four_sur_goal1(x+1,y);
        four_sur_goal1(x+1,y+1);
        return;
    }
    int x = s>>16,y = s%(1<<16);
    int xx = x, yy = y+1; // North
        K_neighbor(xx,yy,'o',x,y,s);
        xx = x+1; yy = y; // East
        K_neighbor(xx,yy,'o',x,y,s);
        xx = x; yy = y-1; // South
        K_neighbor(xx,yy,'o',x,y,s);
        xx = x-1; yy = y; // West
        K_neighbor(xx,yy,'o',x,y,s);
        //xx = x-1; yy = y+1;//NorthWest
       // K_neighbor(xx,yy,'d',x,y,s);
        //xx = x+1; yy = y+1;//NorthEast
        //K_neighbor(xx,yy,'d',x,y,s);
        //xx = x+1; yy = y-1;//SouthEast
        //K_neighbor(xx,yy,'d',x,y,s);
        //xx = x-1; yy = y-1;//South West
        //K_neighbor(xx,yy,'d',x,y,s);
}
void BIT_star::showData(int s) {
    int x = s>>16;
    int y = s%(1<<16);
    cerr << " THE CURRENT LOC(" << x << "," << y<<") is ";
}
void BIT_star::four_sur_goal1(int x, int y) {
    int p = x<<16 | y;
    Ver[_tos] = p;
    dsEdge[_tos] = loc_distance(-1,p);
    _tos++;
}
pair<float, int> BIT_star::four_sur_goal(int x, int y) {
    pair<float, int> pi;
    float dist  = sqrt((x-end_x)*(x-end_x)+(y-end_y)*(y-end_y));
    int p = x<<16 | y;
    pi.first = dist; pi.second = p;
    //thparent[p] = -1;
    Hvisited[p] = true;
    return pi;
}

float BIT_star::bfs_heuristic(float x, float y) {
          pair<float, int> gpi;
          int xx = x, yy = y;
          cerr << xx << "  " << yy << endl;
          int p = xx<<16 | yy;
          thstar[p] = 0;
          tvarl[p] = 0;
          bfs.push_back(p);
          while(!bfs.empty()) {
                int index = bfs.front();
                bfs.pop_front();
              //Hvisited[index] = true;
              _tos=1;
                getGridNeighbors(index);
              int k = 0;//cerr << "test " << endl;
              //k = first[index];
              k = _tos-1;
              //cerr << k << endl;
              for(int i= k; i>0;i--) {
                  int s = Ver[i];
                         if(!Hvisited[s]) {
                            Hvisited[s] = false;
                            tvarl[s] = infinity;
                         }
                         if( Hvisited[s] == false) {

                              int len = tvarl[index] +sEdge[i];
                              tvarl[s] = len;
                              thstar[s] = len/1.39446;//1.41
                              //if(tvarl[s] >len) {
                                  Hvisited[s] = true;
                                 bfs.push_back(s);
                              //}
                        }
                         
                }
          }
   return 0;
}
bool BIT_star::Beshaman_algorithm(int u , int v) {
    /*int x1 = u>>16, y1 = u%(1<<16), x2 = v>>16, y2 = v%(1<<16);
    int incx = 1, incy = 1;
    int dx = x2 - x1;
    int dy = y2 - y1;

    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;

    if (x2 < x1) incx = -1;
    if (y2 < y1) incy = -1;

    int x = x1;
    int y = y1;
        if (dx > dy) {
        // Slope less than 1
        int e = 2 * dy - dx;
        for (int i = 0; i < dx; i++) {
            if (e >= 0) {
                y += incy;
                e += 2 * (dy - dx);
            } else {
                e += 2 * dy;
            }
            x += incx;
            int p = x<<16 | y;
            if(block[p])
                return false;
        }

    } else {
        // Slope greater than 1
        int e = 2 * dx - dy;
        for (int i = 0; i < dy; i++) {
            if (e >= 0) {
                x += incx;
                e += 2 * (dx - dy);;
            }
            else
                e += 2 * dx;
            y += incy;
            int p = x<<16 | y;
            if(block[p])
                return false;
            //cerr << x << "  " << y <<endl;
        }
        
    }
    
    */
    return true;
}
bool BIT_star::Theta_collision_free(int u, int v) {
    int ux = u>>16, uy = u%(1<<16), vx = v>>16, vy = v%(1<<16);
    int diffx = vx - ux, diffy = vy-uy, dirx =0, diry =0,osx = 0, osy=0, step = 0;
    if(diffy >= 0) {
        diry = 1; osy = 1; // south
    }
    else {
        diffy = -diffy; diry = -1; osy = 0; //north
    }

    if(diffx >= 0) {
        dirx = 1; osx = 1; //east
    }
    else {
        diffx = -diffx; dirx = -1; osx = 0; // west
    }
    if(diffx >= diffy) {

        float delta = (float) diry*diffy/diffx;
        float my = uy;
        bool ist = false;
        if(abs(delta-0.333333) < 0.00001) {
            ist = true;
        }
        while( ux != vx) {
              my += delta;
              ux += dirx;
              if(ux == vx) {
                  int ny = my;
                  int np = ux<<16 | ny;
                  if(block[np])
                      return false;
                  break;
              }
              int cx = ux;
              int cy = my;
              int p = cx<<16 | cy;
            if(ist && abs(ux-vx) ==1) {
                my = my - 0.00001;
            }
              if(block[p])
                return false;
        }
    }
    else
    {
        float delta = (float) diffx*dirx/diffy;
        float mx = ux;
        bool ist = false;
        if(abs(delta-0.333333) < 0.00001) {
            ist = true;
        }
        while(uy != vy) {
              mx += delta;
              uy += diry;
              if(uy == vy)
                  break;
              int cx = mx;
              int cy = uy;
              int p = cx<<16 | cy;
              if(ist && abs(uy-vy) ==1) {
                  mx = mx - 0.00001;
              }
              if(block[p])
                return false;
        }
    }
    return true;
}
bool BIT_star::is_corner(int x, int y) {
    if(x!= 0 && y !=0) {
        int cp = x<<16 | y, p=0,p1=0,p2=0, cx=0, cy =0;
        cx = x-1; cy = y; p = cx<<16 | cy;
        cx = x; cy = y-1; p1 = cx<<16 | cy;
        cx = x-1; cy = y-1; p2 = cx<<16 | cy;
        if(block[cp]) {
            if (!block[p] && !block[p1] && !block[p2]) {
                return true;
            }
        }
        else {
            if((block[p] && !block[p1] && !block[p2]) || (!block[p] && block[p1] && !block[p2])
               || (!block[p] && !block[p1] && block[p2])) {
                //cerr << x << "  ******* " << y << "p: "<< block[p] << "  p1: " << block[p1] << " p2: " << block[p2]<<endl;
                return true;

            }
        }
    }
    return false;
}

void BIT_star::NANYA(float x, float y) {
    int p = 0;
    for(int i = map_y-1; i>=0;i--) { //row y
        for(int j = 0; j<map_x;j++) { // col x
            p = j<<16 | i;
            corner[p] = is_corner(j,i);
        }
    }
    int ex= x, ey = y;
    int gl = ex<<16 | ey;
    thstar[gl] = 0;
    thparent[gl] = -1;
    q.push(make_pair(0,gl));
    while(!q.empty()) {
        _tos = 1;
        int index = q.top().second;
        q.pop();
        //cord cor = showData(index);
        //cerr << "(" << cor.x << "," << cor.y <<") is expanding " << endl;
        Hvisited[index] = true;
        getGridNeighbors(index);
        int pr = thparent[index];
        int gp = thparent[pr];

        if(corner[pr] && corner[index] && pr != gl) {
            if(Theta_collision_free(gp,index))
            {
                thparent[index] = gp;
                thstar[index] = thstar[gp] + loc_distance(gp,index);
            }
        }

        for(int i= 1; i<_tos;i++) {
            int s = Ver[i];
            float hvar = 0;
            if(thstar[s] == 0)
                thstar[s] = infinity;

            if(thparent[index] != -1 && !corner[index]){
                if(Hvisited[thparent[s]] && corner[thparent[s]] && thstar[index] > thstar[parent[s]] && thparent[thparent[index]]  == thparent[thparent[s]] && block[thparent[thparent[s]]]) {
                   thparent[index] = thparent[s];
                }
                hvar = thstar[pr] + loc_distance(pr,s);
                if(thstar[s] > hvar) {
                    thstar[s] = hvar;
                    thparent[s] = pr;
                    q.push(make_pair(hvar,s));
                }
            }
            else {
                hvar = thstar[index] + dsEdge[i];
                if(thstar[s] > hvar) {
                    thstar[s] = hvar;
                    thparent[s] = index;
                    q.push(make_pair(hvar,s));
                }
            }
        }
    }
   // cerr << varl[goal] << endl;

}
void BIT_star::Bucket_dijstra(float x, float y ) {
     int inx = 0, mmm = 0;
     list<int> Bucket[16000];
     bool *is_here = new bool[_MAX];
     Bucket[inx].push_back(_MAX);
     while(true) {
         while (Bucket[inx].size() == 0 && inx < 16000) {
             inx++;
         }
         if (inx == 16000)
             break;
         auto it = Bucket[inx].begin();
        //for ( it != Bucket[inx].end(); it++) {//it++
             _tos = 1;
             int u = *it;
             if (!is_here[inx + u]) {
                 Hvisited[u] = true;
                 //it = Bucket[inx].erase(it);
                 getGridNeighbors(u);
                 float udist = tvarl[u];
                 for (int j = 1; j < _tos; j++) {
                      int suc = Ver[j];
                      float vdist = 0, ldist = 0;
                      if (tvarl[suc] == 0)
                          vdist = infinity;
                      else
                          vdist = tvarl[suc];
                 /* if (!Hvisited[suc])
                       Hvisited[suc] = false;*/
                       ldist = udist + dsEdge[j];
                      if (vdist > ldist) {//&& !Hvisited[suc]
                         int vt = vdist;
                         if (tvarl[suc] != 0) {
                         //cerr << " here  ************* " << endl;
                         //Bucket[vt].erase(suc);
                         is_here[vt + suc] = true;
                         }
                         if (u == _MAX)
                            thstar[suc] = ldist;
                         else {
                               thstar[suc] = (float) (ldist/1.39466);
                         }
                         tvarl[suc] = ldist;
                         vt = ldist;
                         Bucket[vt].push_back(suc);
                        //vn[suc] = Bucket[vt].begin();
                      }
                 }
             }
            Bucket[inx].pop_front();
         //}
         //Bucket[inx].clear();
     }
}
void BIT_star::theta_Heuristic(float x, float y) {
    priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>>> openlist;
    pair<float, int> gpi;
    gpi = four_sur_goal(x,y);//goal
    openlist.push(make_pair(gpi.first,gpi.second));
    gpi = four_sur_goal(x+1,y);//east
    openlist.push(make_pair(gpi.first,gpi.second));
    gpi = four_sur_goal(x,y+1);//north
    openlist.push(make_pair(gpi.first,gpi.second));
    gpi = four_sur_goal(x+1,y+1);//northeast
    openlist.push(make_pair(gpi.first,gpi.second));
    while(!openlist.empty()) {
        _tos = 1;
        int index = openlist.top().second;
        tvarl[index] = openlist.top().first;
        if(Hvisited[index] ==false){
            thstar[index] = (float) tvarl[index]/1.394758;//1.361.08
            }
        else
            thstar[index] = tvarl[index];
        openlist.pop();
        Hvisited[index] = true;
        getGridNeighbors(index);
        for(int i= 1; i<_tos;i++) {
            int s = Ver[i];
            if(tvarl[s] == 0) {
                tvarl[s] = infinity;
            }
            /*if(!Hvisited[s]) {
                Hvisited[s] = false;
            }*/
           // if( Hvisited[s] == false) {
                /*int pr = thparent[index];
                if(thparent[index] != -1 && Theta_collision_free(pr,s)){
                    double hvar = thstar[pr] + loc_distance(pr,s);
                    if(thstar[s] > hvar) {
                        thstar[s] = hvar;
                        thparent[s] = pr;
                        openlist.push(make_pair(hvar,s));
                    }
                }*/
                //else {
                      float hvar = tvarl[index] + dsEdge[i];//loc_distance(index,s)
                      if(tvarl[s] > hvar) {
                         tvarl[s] = hvar;
                         //thparent[s] = index;
                         openlist.push(make_pair(hvar,s));
                      }
                //}
           //}
       }
    }
}


void BIT_star::Trajectory_generating(BIT_star *bit, int t) {
    Creating_The_Trajectory(bit,t);
    exit(-1);
    cout << "************ start to print out whole tree " << endl;
    for(auto it  = Edge.begin(); it != Edge.end(); it++) {
        Dubins dubins;
        Dubins_curve path_get;
        states st = state(it->first.second);
        states sp = state(it->first.first);
        path_get = it->second;
        dubins.Generating_dubins_path(bit,path_get,sp,st,"pathG");
        cout << "*****************" << endl;
    }
}
float BIT_star::Heuristic_value(int v) {
    states st = state(v);
    return sqrt(pow( st.x- end_x,2)+ pow(st.y- end_y,2));
}


float BIT_star::estimatedComeValue(int v) {
    states st = state(v);
    return sqrt(pow((start_x- st.x),2) + pow((start_y-st.y),2));
}
void BIT_star::prune_rgg(float costGoal, float dcostGoal) {

    for(auto it = Poses.begin(); it != Poses.end();) {
        for(auto itr = sameLocation[it->first].begin(); itr != sameLocation[it->first].end();) {
            if(cost[*itr] > dcostGoal || !visited[*itr]) {
                itr = sameLocation[it->first].erase(itr);
            }
            else
                itr++;
        }
        float f = estimatedComeValue(it->first) +theStar_heuristic(state(it->first));//Heuristic_value(it->first)
        if(f >= costGoal) {
            if(rggData.find(it->first) != rggData.end())
                rggData.erase(it->first);
            it = Poses.erase(it);
        }
        else
            it++;
    }
    for(auto it = rggData.begin(); it != rggData.end();) {
        if(visited[it->first] || Poses.find(it->first) == Poses.end()) {
            it = rggData.erase(it);
        }
        else
            it++;
    }
}
void BIT_star::prune_vertices(float costGoal, float dcostGoal) {
    for(auto it = vertices.begin(); it != vertices.end();){
        float f = estimatedComeValue(it->first) +theStar_heuristic(state(it->first));// Heuristic_value(it->first)
        if(f > costGoal || (cost.find(it->first) != cost.end() && cost[it->first] > dcostGoal)) {
            it = vertices.erase(it);
        }
        else
            it++;
    }
}
void BIT_star::edges_prune(float costGoal,float dcostGoal){
    for(auto it = Edge.begin(); it != Edge.end();){
        float fv = estimatedComeValue(it->first.first) + theStar_heuristic(state(it->first.first));// Heuristic_value(it->first.first)
        float fw = estimatedComeValue(it->first.second) +theStar_heuristic(state(it->first.second)) ;//Heuristic_value(it->first.second)
        if(fv > costGoal || fw > costGoal|| cost[it->first.first] > dcostGoal || cost[it->first.second] > dcostGoal) {
            it = Edge.erase(it);
        }
        else
            it++;
    }
}
void BIT_star::add_vertices_back_to_sample(){
    for(auto it = vertices.begin(); it != vertices.end(); it++){
        if(!visited[it->first] && cost[it->first] != 0 && Poses.find(it->first) != Poses.end()) {
            cost[it->first] = infinity;
            if(rggData.find(it->first) == rggData.end()) {
                rggData[it->first] = vertices[it->first];
            }
        }
    }
}
void BIT_star::erase_vertices_infinity(){
    for(auto it = vertices.begin(); it != vertices.end();){
        if(it->first != 0 && !visited[it->first]) {
            it = vertices.erase(it);
        }
        else{
            it++;
        }
    }
}
void BIT_star::prune(float costGoal, float dcostGoal){
    prune_rgg(costGoal,dcostGoal);
    edges_prune(costGoal,dcostGoal);
    prune_vertices(costGoal,dcostGoal);
    add_vertices_back_to_sample();
    erase_vertices_infinity();
}
void BIT_star::creating_vertice_queue() {
    for(auto it = vertices.begin(); it != vertices.end();it++) {
        float var = cost[it->first] + Heuristic_value(it->first);
        BestQVvalue.push(make_pair(var,it->first));
    }
    BestEvalue.push(make_pair(cost[1],make_pair(source,goal)));
}
void BIT_star::building_edge_queue() {
    for(auto it = vertices.begin(); it != vertices.end(); it++) {
        if(it->first != 0 && Edge.find(make_pair(parent[it->first],it->first)) != Edge.end()) {
            float dist = cost[it->first] + Heuristic_value(it->first);
            BestEvalue.push(make_pair(dist,make_pair(parent[it->first],it->first)));
        }
    }
}
float BIT_star::stateDistance(states v, states w)
{
    return sqrt(pow( v.x- w.x,2)+ pow(v.y- w.y,2));
}
vector<int> BIT_star::GetKneighbors(BIT_star *bit, int v, unordered_map<int,states> nodeContainer,string str) {
    vector<int > knbors;
    Dubins dubins;
    unordered_set<int > child;
    Dubins_curve path_get;
    priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>> > tq;
    for(auto it = nodeContainer.begin(); it != nodeContainer.end(); it++) {
        float dist = stateDistance(state(v), nodeContainer[it->first]);
        if(dist > Epsilon && it->first !=v && str == "sample" && it->first != 0 ) {
            tq.push(make_pair(dist,it->first));
        }
        else {
            if(dist > Epsilon && str == "edge" && visited[it->first] && station_states.find(it->first) == station_states.end() && oldvertices.find(it->first) != oldvertices.end()) {
                tq.push(make_pair(dist,it->first));
            }
        }
    }
    int index =0;
    //cerr << tq.size() << endl;
    while(!tq.empty()) {
        if(index == kneighbors)
            break;
            knbors.push_back(tq.top().second);
            index++;
        tq.pop();
    }
    return knbors;
}

void BIT_star::Prune_vertcies( int xm) {
    priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>> > Bq;
    Bq = BestQVvalue;
    BestQVvalue = priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>> >();
    while(!Bq.empty()) {
        if(Bq.top().second != xm && parent[Bq.top().second] != xm) {
            BestQVvalue.push(Bq.top());
        }
        Bq.pop();
    }
}
float BIT_star::dist_to_four_grid(int x, int y, states v) {
       int tx = x, ty = y;
       int p = tx<<16 | ty;
       return sqrt((v.x-x)*(v.x-x) + (v.y-y)*(v.y-y)) + thstar[p];
}
float BIT_star::theStar_heuristic(states v) {
      int vx = v.x, vy = v.y, m = 0;
      float s1 = dist_to_four_grid(vx,vy,v);//southwest
      float s2 = dist_to_four_grid(vx+1,vy,v);//southeast
      float s3 = dist_to_four_grid(vx+1,vy+1,v);//northeast
      float s4 = dist_to_four_grid(vx,vy+1,v);;//northwest
      m = min(s1,min(s2,min(s3,s4)));
      return m;
}
void BIT_star::creating_edges_queue(int v, float costGoal ,string str, vector<int > kn) {
    float ghat = estimatedComeValue(v);
    states vv = state(v);
    Dubins dubins;
    dubins.get_obs(obstacleM);
    Dubins_curve path_get;
    for(int i =0; i < kn.size(); i++) {
        states ww = state(kn[i]);
        float eucd = EucDistance(v,kn[i]);
        float mtime = vv.mTime + (float)(eucd/vv.speed)/2, vtime = vv.mTime + (float)(eucd/vv.speed);
        float esdcost = dubins.EstimatedDvalue(vv,ww,mtime,vtime,v,kn[i]);
        float hhat =theStar_heuristic(ww);// Heuristic_value(kn[i])
        float dist = cost[v] + esdcost + eucd +hhat;
        float distt = cost[v] + eucd+esdcost;
        float fdist = cost[v] + esdcost + eucd + hhat;
        float dvarl = cost[v] + esdcost + eucd;
         if(cost.find(kn[i]) == cost.end()){
            cost[kn[i]] = infinity;
            scost[kn[i]] = infinity;
        }
        if(visited.find(kn[i]) == visited.end()) {
            visited[kn[i]] = false;
        }
        //cerr << costGoal << "  " << dist << "   " << kn[i]<<endl;
        if(dist < costGoal && str == "sample") {
            nchange = false;
            if (!visited[kn[i]] && cost[kn[i]] > dvarl) {
                BestEvalue.push(make_pair(fdist,make_pair(v,kn[i])));
                Tvalue.push(fdist);
            }
            else {
                if (visited[kn[i]] && station_states.find(v) != station_states.end() && oldvertices.find(v) == oldvertices.end()) {
                    int newstate = tnum;//cerr << "here " << endl;
                    states start = state(v), end = state(kn[i]);
                    float dcur = cost[v] + eucd + esdcost;
                    bool check = false;

                    for (auto itrr = sameLocation[kn[i]].begin(); itrr != sameLocation[kn[i]].end();) {
                        //if(v== 36)
                            //cerr << "*********78787 " << ef[*itrr] << endl;
                          if (sameLocation[kn[i]].find(*itrr) != sameLocation[kn[i]].end() && !visited[*itrr] &&
                            *itrr != newstate && dcur < ef[*itrr]) {
                            itrr = remove_state(itrr, kn[i], "b");
                           break;
                         }
                         else if (!visited[*itrr] && *itrr != newstate && dcur > ef[*itrr]) {
                            check = true;
                            break;
                         }
                        else {
                            itrr++;
                        }
                    }
                    if (!check && fdist < costGoal) {
                        rggData[newstate] = end;
                        lookUpPose[newstate] = kn[i];
                        parent[newstate] = v;
                        sameLocation[kn[i]].insert(newstate);
                        newGenerated.insert(newstate);
                        BestEvalue.push(make_pair(fdist, make_pair(v, newstate)));
                        cost[newstate] = infinity;
                        scost[newstate] = infinity;
                        Tvalue.push(fdist);
                        tnum++;
                    }

                }
            }

        }
        else if(str ==  "edge"){
                if((Edge.find(make_pair(v,kn[i])) == Edge.end()) && (dist < costGoal) && (dvarl < cost[kn[i]])) {
                    BestEvalue.push(make_pair(fdist,make_pair(v,kn[i])));
                    Tvalue.push(fdist);
            }
        }
    }
}

unordered_set<int>::iterator  BIT_star::remove_state(unordered_set<int>::iterator itrr, int dey,string algo){
    rggData.erase(*itrr);
    auto sit = station_states.find(*itrr);
    if(sit != station_states.end())
        station_states.erase(sit);
    Edge.erase(make_pair(parent[*itrr],*itrr));
        Prune_edge(*itrr);
        UpDateQvalue(*itrr);
    cost.erase(*itrr);
    scost.erase(*itrr);
    itrr = sameLocation[dey].erase(itrr);
    return itrr;
}

int BIT_star::circle_state(BIT_star *bit, int v, int num, float svar, int size,string algo) {
    Dubins dubins;
    float dcost = 0, dCUR = 0, eucd = 0, fdist = 0;
    dubins.get_obs(obstacleM);
    Dubins_curve path_get;
    int newstate = num;
    states start = state(v), end = state(v);
    int dtx = state(v).x;
    int dty = state(v).y;
    int p = dtx<<16 | dty;
    float hvar =theStar_heuristic(end);//Heuristic_value(v)
    end.theta = ((float) 2 * M_PI) * rand() / (RAND_MAX);
    string name = "";
        eucd = dubins.EucDubinsDistance(start, end);
     path_get = dubins.Dubins_Optimal_path(start, end);
    if(dubins.Du_collision_free(bit,path_get,start,end)) {
        float mtime = start.mTime + (float)(path_get.relvar/start.speed)/2, vtime = start.mTime + (float)(path_get.relvar/start.speed);
        dcost = dubins.EstimatedDvalue(start, end, mtime, vtime, v, newstate);
        dCUR = cost[v] + eucd + dcost + hvar;
            fdist = cost[v] + dcost + path_get.relvar + hvar;
        if((size == 0 || dCUR < svar) && fdist < cost[1]) {
            rggData[newstate] = end;
            BestEvalue.push(make_pair(fdist, make_pair(v, newstate)));
            if(lookUpPose.find(v) != lookUpPose.end()) {
                int vr = lookUpPose[v];
                sameLocation[vr].insert(newstate);
                lookUpPose[newstate] = vr;
            }
            else {
                sameLocation[v].insert(newstate);
                lookUpPose[newstate]= v;
            }
            station_states.insert(newstate);
            num++;
                adj[v][newstate] = path_get;
                stayCost[newstate] = dubins.get_dycost();
                ef[newstate] =  cost[v] + stayCost[newstate] + path_get.relvar;
                dubins.set_dycost_to_zero();
                cost[newstate] = infinity;
                scost[newstate] = infinity;
                tnum = num;
        }
    }
    return num;
}

void BIT_star::ExpandVertex(BIT_star *bit, int v, float costGoal){

    if(parent.find(1) != parent.end() && parent[1] == v) {
        gfind = true;
       return;
    }
    visited[v] = true;
    string name = "sample";
    BestQVvalue.pop();
    if(v ==0)
        BestEvalue.pop();
    vector<int > kn;
    Dubins dubins;
    int ver = v;
    if(lookUpPose.find(v) == lookUpPose.end() || oldvertices.find(v) != oldvertices.end()) {
        kNeighbors[v] = GetKneighbors(bit,v,bPoses,name);
    }
    else {
        ver= lookUpPose[v];
    }
    creating_edges_queue(v, costGoal, name, kNeighbors[ver]);
    if(oldvertices.find(v) == oldvertices.end()){
        name = "edge";
        if(lookUpPose.find(v) == lookUpPose.end() || oldvertices.find(v) != oldvertices.end()) {
            kNneighbors[v] = GetKneighbors(bit,v,Poses,name);
        }
        else {
            ver= lookUpPose[v];
        }
        creating_edges_queue(v,costGoal,name,kNneighbors[ver]);
        float var = 0;
        if(Tvalue.size() == 0) {
            var = infinity;
        }
        else
            var = Tvalue.top();
        circle_state(bit,v,tnum,var,Tvalue.size(),"b");
    }
    Tvalue = priority_queue<float, vector<float>,greater< float> >();
}
void BIT_star::Erase_edge(int xm){
    for(auto it = Edge.begin(); it != Edge.end();) {
        if(it->first.second == xm) {
            it = Edge.erase(it);
        }
        else
            it++;
    }
}
void BIT_star::Erase_queue_Edge(int xm) {
    priority_queue< pair<float,pairs>, vector<pair<float,pairs> >,greater<pair<float,pairs>> > temp;
    temp = BestEvalue;
    BestEvalue = priority_queue< pair<float,pairs>, vector<pair<float,pairs> >,greater<pair<float,pairs>> >();
    while(!temp.empty()) {
        if(temp.top().second.second != xm)  {
            BestEvalue.push(temp.top());
        }
        temp.pop();
    }
}
void BIT_star::Creating_The_Trajectory(BIT_star *bit, int xm) {
    stack<int > path;
    stack<int > dypath;
    int cp = xm;
    while(parent.find(cp) != parent.end()) {
        path.push(cp);
        dypath.push(cp);
        cp = parent[cp];
    }
    while(!path.empty()) {
        int id = path.top();
        Dubins dubins;
        Dubins_curve path_get;
        states st = state(id);
        states sp = state(parent[id]);
        path_get = Edge[make_pair(parent[id],id)];
        dubins.Generating_dubins_path(bit,path_get,sp,st,"pathG");

        path.pop();
    }
    //exit(-1);
    Dubins dubins;
    Dubins_curve path_get;
    dubins.get_obs(obstacleM);
    while(!dypath.empty()) {
        int id = dypath.top();
        states st = state(id);
        states sp = state(parent[id]);
        cerr <<st.x << " " << st.y << " " << st.mTime << "  " << cost[id] << " " << scost[id] << " " << id <<endl;
        path_get = Edge[make_pair(parent[id],id)];
        dubins.Path_showing(path_get,sp,st);

        dypath.pop();
    }
}

void BIT_star::Dubins::dynamic_location(DynamicObstacle & dynamic, float x, float y, float t, int v) {
    DyInfoUpdating(dynamic,t);
    float dheading = dynamic.heading, dspeed = dynamic.speed;
    float inTime = t-dynamic.curTime;
    float dx = cos(dheading)*dspeed*(inTime) + dynamic.x, dy = sin(dheading) * dspeed * (inTime) + dynamic.y;
    cout <<"D"<< v << " " << dx << " " << dy <<" " << dheading << " " << dspeed<< endl;
}
void BIT_star::Dubins::moving_path(float length, char c, float sTha,float sx, float sy, float deltalen, float t, float aspeed) {
    float pathC=0;
    segement seg;
    while(pathC < length) {
        seg = seg_generation(pathC,c,sTha,sx,sy);
        for(int i =0; i< Dobstacle.size();i++ ) {
            dynamic_location(Dobstacle[i], seg.x, seg.y, t+(pathC*rho)/aspeed,i);
        }
        pathC += deltalen;
    }
}
void BIT_star::Dubins::Path_showing(Dubins_curve duc,states u, states v) {
    float sx = u.x, sy = u.y, sTha = u.theta, sTime = u.mTime, dyc = 0, aspeed = u.speed;
    string pT = duc.path_type;
    vector<segement> segPoints;
    for(DynamicObstacle & dyo : Dobstacle) {
        DyInfoUpdating(dyo,u.mTime);
    }
    segement seg;
    seg = seg_generation(duc.fst_seg,pT[0],u.theta,u.x,u.y);
    segPoints.push_back(seg);
    seg = seg_generation(duc.sec_seg,pT[1],seg.theta,seg.x,seg.y);
    segPoints.push_back(seg);
    float deltaLen = (float) 0.1;//duc.len/80
    moving_path(duc.fst_seg,pT[0],sTha,sx,sy,deltaLen,sTime,aspeed);
    float fTime = sTime + (duc.fst_seg*rho)/aspeed;
    moving_path(duc.sec_seg,pT[1],segPoints[0].theta,segPoints[0].x,segPoints[0].y,deltaLen,fTime,aspeed);
    float seTime = fTime + (duc.sec_seg*rho)/aspeed;
    moving_path(duc.fin_seg,pT[2],segPoints[1].theta,segPoints[1].x,segPoints[1].y,deltaLen,seTime,aspeed);//exit(-1);
}
void BIT_star::Agent_and_dyObstacle_Trajectory_showing(int u, int v) {
    Dubins dubins;
    Dubins_curve path_get;
    dubins.get_obs(orgobstacleM);
    path_get = Edge[make_pair(u,v)];
    dubins.Path_showing(path_get,state(u),state(v));
}
void BIT_star::Prune_edge(int xm) {
    priority_queue< pair<float,pairs>, vector<pair<float,pairs> >,greater<pair<float,pairs>> > QE;
    QE = BestEvalue;
    BestEvalue = priority_queue< pair<float,pairs>, vector<pair<float,pairs> >,greater<pair<float,pairs>> >();
    while(!QE.empty()) {
        if(QE.top().second.second != xm && QE.top().second.first != xm) {
            BestEvalue.push(QE.top());
        }
        QE.pop();
    }
}

void BIT_star::UpDateQvalue(int xm) {
    priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>> > QV = BestQVvalue;
    BestQVvalue = priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>> >();
    while(!QV.empty()) {
        if(QV.top().second != xm) {
            BestQVvalue.push(QV.top());
        }
        QV.pop();
    }
}
void BIT_star::set_src_and_goal() {
    states start, Goal;
    start.x = start_x,start.y = start_y;
    start.theta = ((float ) 2*M_PI)*rand()/(RAND_MAX);
    Goal.x = end_x,Goal.y = end_y;
    Goal.theta = ((float ) 2*M_PI)*rand()/(RAND_MAX);
    Poses[source]= rggData[source] = start;
    Poses[goal]= rggData[goal] = Goal;
    scost[0] = cost[0] = 0; rggData[source].mTime = 0; rggData[source].speed =2.5;
    scost[1] = cost[1] = infinity;//theStar_heuristic(state(source))
    vertices[source] = rggData[source];
    numNodes = 2;
}
void BIT_star::bit_star(int src, int t){
    int cn =0;
    int che =0;
    BIT_star * bit = new BIT_star;
    Dubins dubins; Dubins_curve path_get;
    bit->map_x = map_x; bit->map_y = map_y;
    int mmm = src;
    bit->block = block;
    dubins.get_obs(obstacleM);
    float vy=0;
    set_src_and_goal();
    int num  = numNodes;
    clock_t sT = clock();
    char itrID = 'f';
    while(true) {
        if (BestEvalue.empty() && BestQVvalue.empty()) {
            int count = num;
            if (itrID == 'f')
                numNodes = 300;
            else
                numNodes = 100;
            num += numNodes;
            tnum = num;
            bit->Tcount = num;
           // cerr << Poses.size() << endl;
            prune(scost[1], cost[1]);
           // cerr << Poses.size() << endl;
            sampling(num, count);
           // cerr << Poses.size() << endl;
            oldvertices = vertices;
            bPoses = rggData;
            creating_vertice_queue();
            kneighbors = 15;//2*exp(1)*log(cardv)  int cardv = rggData.size() + vertices.size();
            //cerr << "test here " << Poses.size() << "  " << visited[89]<< "  " <<vertices.size() << "  " << BestQVvalue.size() << " " << BestEvalue.size() << endl;
        }
        while ( BestEvalue.size() == 0 || (!BestQVvalue.empty() && BestQVvalue.top().first <= BestEvalue.top().first)) {
            ExpandVertex(bit, BestQVvalue.top().second, cost[1]);
            if(gfind) {
                break;
            }
        }
        bool isconnected = false, incircle = false, coCheck = false;
        pairs eg; int vm = 0, xm = 0, tvm =0, txm = 0;
        float esEdgeVar = 0, esHvar = 0, dist = 0, mtime = 0, vtime = 0;
        if(!gfind) {
            eg = BestEvalue.top().second;
            BestEvalue.pop();
            vm = eg.first;
            xm = eg.second;//
            esEdgeVar = EucDistance(vm, xm);
            esHvar =theStar_heuristic(state(xm));//Heuristic_value(xm)
            visited[vm] = true;
            tvm = vm; txm = xm;
            if (station_states.find(vm) != station_states.end() || newGenerated.find(vm) != newGenerated.end()) {
                vm = lookUpPose[vm];
                //int z = vm*map_x + xm;
                if (ccnode.find(make_pair(vm, xm)) != ccnode.end())
                    coCheck = true;
            }
            vm = tvm; xm = txm;
            if (station_states.find(vm) != station_states.end() || newGenerated.find(vm) != newGenerated.end())
                vm = lookUpPose[vm];
            if (station_states.find(xm) != station_states.end() || newGenerated.find(xm) != newGenerated.end())
                xm = lookUpPose[xm];
            //int p = vm*map_x + xm;
            //if(p == 49)
            //cerr << p << "  +++++++++++++++++++++++ " << vm << "  " << xm <<endl;
            if (connected.find(make_pair(vm, xm)) != connected.end()) {
                isconnected = true;
            }
            vm = tvm; xm = txm;
            if (station_states.find(xm) != station_states.end()) {
                incircle = true;
            }
            if (!incircle) {
                path_get = dubins.Dubins_Optimal_path(state(vm), state(xm));
                // mtime = state(vm).mTime + (float)(path_get.relvar/state(vm).speed)/2; vtime = state(vm).mTime + (float)(path_get.relvar/state(vm).speed);
            }
            dist = cost[vm] + esEdgeVar  + esHvar;//+ esdcost
        }

        if(!gfind && dist < cost[1]) {
            if (!coCheck) {
               if (incircle || isconnected || dubins.Du_collision_free(bit, path_get, state(vm), state(xm))) {
                  float dcost = 0;
                  if (!incircle) {
                      tvm = vm;
                      txm = xm;
                      if (Poses.find(vm) != Poses.end() && Poses.find(xm) != Poses.end()) {
                          connected.insert(make_pair(vm,xm));
                      }
                      vm = tvm;
                      xm = txm;
                      if (isconnected)
                           dcost = dubins.dyCost(path_get, state(vm), state(xm), state(source), vm);
                      else {
                        dcost = dubins.get_dycost();
                        dubins.set_dycost_to_zero();
                      }
                  }
                  else {
                    path_get = adj[vm][xm];
                    dcost = stayCost[xm];
                  }

                    float srev = path_get.relvar;
                    float rev = srev + dcost; //True edge cost
                    float resv = rev + cost[vm];
                    if (resv + esHvar < cost[1]) {//rev+ estimatedComeValue(vm)
                        if (xm == goal) {
                            bestCost.push(cost[xm]);
                            if (resv < bestCost.top()) {
                                if (abs(resv - cost[xm]) < Epsilon) {
                                    Creating_The_Trajectory(bit, xm);
                                    break;
                                }
                            }
                            cost[xm] = resv;
                            scost[xm] = scost[vm] + srev;
                            itrID = 'n';
                            cn++;
                            clock_t tT = clock();
                            rtime = (float) (tT - sT) / CLOCKS_PER_SEC;
                            cerr << " the " << cn << " search at time: " << rtime + wtime << "  " << cost[xm] << " " << Poses.size() << " " << vertices.size() << endl;
                            cout << cn <<"  " << rtime + wtime << "  " << cost[xm] << "  " << Poses.size() << "  " << vertices.size()<< endl;
                            if (cn == 12 && xm == 1) {  //cerr << tnum << "---------------------s " << num << "  " << bit->Tcount<< endl; //cerr << parent[304] << endl;
                                parent[xm] = vm;
                                Edge[make_pair(vm, xm)] = path_get;
                                updateTime(xm, state(vm).mTime + (float) path_get.relvar / state(vm).speed, "b");
                                Trajectory_generating(bit, xm);
                                num = bit->Tcount;
                                num = tnum;
                                break;
                            }
                            num = bit->Tcount;
                            num = tnum;
                            parent[xm] = vm;
                            Edge[make_pair(vm, xm)] = path_get;
                            BestEvalue = priority_queue < pair < float, pairs >, vector < pair < float, pairs > >, greater <pair <float, pairs >> > ();
                            BestQVvalue = priority_queue < pair < float, int >, vector < pair < float, int > >, greater <pair <float,int >> >();
                            continue;
                        }
                       if (resv < cost[xm] && Edge.find(make_pair(vm, xm)) == Edge.end()) {
                           updateTime(xm, state(vm).mTime + (float) path_get.relvar / state(xm).speed, "b");
                           cost[xm] = resv;
                           scost[xm] = scost[vm] + srev;
                           if (vertices.find(xm) != vertices.end()) {
                               Erase_edge(xm);
                           } else {
                                    vertices[xm] = rggData[xm];
                                    rggData.erase(xm);
                                    BestQVvalue.push(make_pair(cost[xm] +theStar_heuristic(state(xm)),  xm));//Heuristic_value(xm)
                                    visited[xm] = false;
                           }
                           parent[xm] = vm;
                           Edge[make_pair(vm, xm)] = path_get;
                           Erase_queue_Edge(xm);
                       }
                    }
               }
               else {
                   if (dubins.get_cfree()) {
                       if (Poses.find(vm) != Poses.end() && Poses.find(xm) != Poses.end()) {
                           ccnode.insert(make_pair(vm, xm));
                       }
                       dubins.set_cfree();
                   }
               }
            }
        }
        else {
            num = bit->Tcount;
            num = tnum;
            gfind = false;
            itrID = 'n';
            //cerr << " more sampling " << endl;
            cout << cn << "  " << rtime + wtime << "  " << cost[1] << "  " << Poses.size() << "  " << vertices.size() << endl;
            BestEvalue = priority_queue< pair<float,pairs>, vector<pair<float,pairs> >,greater<pair<float,pairs>> >();
            BestQVvalue = priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>> >();
        }
        clock_t eT = clock();
        rtime = (float) (eT-sT)/CLOCKS_PER_SEC;
        //cout << rtime%1 << "  " << "  " << rtime + wtime << "  " << cost[xm] << "  " << Poses.size() << "  " << vertices.size() << endl;
        if(rtime + wtime >= 30) {
            //Trajectory_generating(bit, 1);
            cerr << cn+1 <<"  " << rtime <<" cost: " << cost[1] << " PoseNum: " << Poses.size() << " nUM OF STATE " << tnum <<endl;
            cout << cn+1 <<"  " << rtime+wtime <<"  " <<cost[1] << "  " << Poses.size() << "  " << tnum <<endl;
            break;
        }

    }
}
BIT_star::Dubins::Dubins() {
    alpha = 0;
    beta  = 0;
    t     = 0;
    p     = 0;
    q     = 0;
    angle = 0;
    tpos  = 0;
    rho   = 1.6;
    dynaCost = 0;
    tanValue = 0;
    dycheck  = 0;
    cfree = false;
    angleT = false;
    esCheck = false;
    ColidHap = false;
    sigma = 0.000063;
    updateTrue = false;
    infinity = numeric_limits<float>:: infinity();
}

BIT_star::Dubins::~Dubins() {
}

float BIT_star::Dubins::EucDubinsDistance(states u, states v){
    return sqrt(pow(u.x - v.x,2)+pow(u.y - v.y,2));
}

float mod(float x, float y) {
    return x-y*floor(x/y);
}
float BIT_star::Dubins::ta_value(float sinAlpha, float sinBeta, float cosAlpha, float cosBeta, string pathType, float d) {
    float tV=0;
    if(pathType == "lsl"){
        tV = atan2(cosBeta-cosAlpha, d + sinAlpha - sinBeta);
    }
    else if(pathType == "rsl") {
        tV = atan2( cosBeta + cosAlpha, d - sinAlpha - sinBeta);
    }
    else if(pathType == "lsr") {
        tV=atan2(-cosBeta - cosAlpha, d + sinAlpha + sinBeta);
    }
    else if(pathType == "rsr")
    {
        tV=atan2( -cosBeta + cosAlpha, d - sinAlpha + sinBeta);
    }
    else if(pathType == "lrl") {
        tV = atan2(-cosBeta+cosAlpha, d + sinAlpha - sinBeta);
    }
    else {
        tV=atan2( -cosBeta + cosAlpha, d - sinAlpha + sinBeta);
    }
    return tV;
}
float BIT_star::Dubins::t_path(states u,states v, string pathType, float d, float pt) {
    float tp =0;
    if(pathType == "lsl"){
        tanValue = ta_value(sin(alpha),sin(beta),cos(alpha),cos(beta),pathType,d);
        tp = mod(tanValue - alpha,2*M_PI);
    }
    else if(pathType == "rsr"){
        tanValue = ta_value(sin(alpha),sin(beta),cos(alpha),cos(beta),pathType,d);
        tp = mod(-tanValue + alpha,2*M_PI);
    }
    else if(pathType == "rsl") {
        tanValue = ta_value(sin(alpha),sin(beta),cos(alpha),cos(beta),pathType,d);
        tp = mod(-tanValue  + alpha + atan2(2,pt),2*M_PI);
    }
    else if(pathType == "lsr") {
        tanValue = ta_value(sin(alpha),sin(beta),cos(alpha),cos(beta),pathType,d);
        tp = mod(tanValue - alpha - atan2(-2,pt),2*M_PI);
    }
    else if(pathType == "rlr") {
        tanValue = ta_value(sin(alpha),sin(beta),cos(alpha),cos(beta),pathType,d);
        tp = mod(-tanValue + alpha + mod(pt*0.5,2*M_PI),2*M_PI);
    }
    else {
        tanValue = ta_value(sin(alpha),sin(beta),cos(alpha),cos(beta),pathType,d);
        tp = mod(-tanValue - alpha + pt*0.5,2*M_PI);
    }
    if(pt == infinity)
        return infinity;
    return tp;
}

float BIT_star::Dubins::p_path(states u,states v, string pathType, float d) {
    float pp = 0;
    float var =0;
    if(pathType == "lsl"){
        var = 2+d*d-2*cos(alpha - beta) + 2*d*(sin(alpha) -sin(beta));
    }
    else if(pathType == "rsr"){
        var =  2+d*d-2*cos(alpha - beta) + 2*d*(-sin(alpha) +sin(beta));
    }
    else if(pathType == "rsl") {
        var =  -2+d*d+2*cos(alpha - beta) - 2*d*(sin(alpha) +sin(beta));
    }
    else if(pathType == "lsr") {
        var =  -2+d*d+2*cos(alpha - beta) + 2*d*(sin(alpha) +sin(beta));
    }
    else if(pathType == "rlr") {
        var = 0.125*(6-d*d+2*cos(alpha - beta) + 2*d*(sin(alpha) -sin(beta)));
    }
    else {
        var = 0.125*(6-d*d+2*cos(alpha - beta) + 2*d*(sin(beta) -sin(alpha)));
    }
    if((var < 0 && (pathType  == "lsl" || pathType  == "rsr"|| pathType  == "lsr"
                    || pathType  == "rsl" )) || (fabs(var) >1 && ( pathType == "rlr" || pathType  == "lrl")))
        pp = infinity;
    else {
        if(pathType == "rlr" || pathType  == "lrl")
            pp = mod(2*M_PI - acos(var),2*M_PI);
        else
            pp = sqrt(var);
    }
    return pp;
}

float BIT_star::Dubins::q_path(states u,states v, string pathType) {
    float qq=0;
    if(pathType == "lsl"){
        qq = mod(beta - tanValue,2*M_PI);
    }
    else if(pathType == "rsr"){
        qq = mod(-mod(beta,2*M_PI)+tanValue, 2*M_PI);
    }
    else if(pathType == "rsl") {
        qq = mod(mod(beta,2*M_PI) - tanValue + atan2(2,p),2*M_PI);
    }
    else if(pathType == "lsr") {
        qq = mod(-mod(beta,2*M_PI)+ tanValue - atan2(-2,p),2*M_PI);
    }
    else if(pathType == "rlr") {
        qq = mod(-beta+alpha - t + mod(p,2*M_PI),2*M_PI);
    }
    else {
        qq = mod( mod(beta,2*M_PI)- alpha-t +mod(p,2*M_PI),2*M_PI);
    }
    if(p==infinity)
        return infinity;
    return qq;
}

float BIT_star::Dubins::DubinsPathLenght(states u, states v, string pathType, float d) {

    p =  p_path(u,v,pathType,d);
    t =  t_path(u,v,pathType,d,p);
    q =  q_path(u,v,pathType);
    return t+p+q;
}
Dubins_curve BIT_star::Dubins::Dubins_Optimal_path(states u, states v) {
    string pathType[6] ={"lsl","lsr","rsl","rsr","lrl","rlr"};
    priority_queue< Dubins_set , vector<Dubins_set >,compare > Bopt;
    float D = EucDubinsDistance(u,v);
    float d = (float) D/rho;
    angle = mod(atan2(v.y-u.y,v.x-u.x),2*M_PI);
    alpha = mod(u.theta - angle,2*M_PI);
    beta  = mod(v.theta - angle,2*M_PI);
    float length=0;
    for(int i =0; i < 6; i++) {
        length = DubinsPathLenght(u,v,pathType[i],d);
        Dubins_set duSet;
        duSet.pathLen = length;
        duSet.lenT = t;
        duSet.lenP = p;
        duSet.lenQ = q;
        duSet.pType = pathType[i];
        Bopt.push(duSet);
    }
    Dubins_curve dc;
    dc.rho = rho;
    dc.relvar = Bopt.top().pathLen * rho;
    dc.len = Bopt.top().pathLen;
    dc.fst_seg =  Bopt.top().lenT;
    dc.sec_seg =  Bopt.top().lenP;
    dc.fin_seg =  Bopt.top().lenQ;
    dc.path_type = Bopt.top().pType;
    return dc;
}
BIT_star::Dubins::segement BIT_star::Dubins::seg_generation(float t, char dir, float phi,float x, float y) {
    float Ttheta =0;
    if(dir == 'l'){
        x += (sin(phi+t) - sin(phi))*rho;
        y += (-cos(phi+t) + cos(phi))*rho;
        Ttheta = t+phi;
    }
    else if(dir == 'r') {
        x += rho*(-sin(phi -t) + sin(phi));
        y += rho*(cos(phi-t) - cos(phi));
        Ttheta = phi -t;
    }
    else {
        x += (t*cos(phi))*rho;
        y += (t*sin(phi))*rho;
        Ttheta = phi;
    }
    segement semt;
    semt.x = x;
    semt.y = y;
    semt.theta = mod(Ttheta,2*M_PI);
    return semt;
}
void BIT_star::Dubins::drawing_path(BIT_star *bit, float length, char c ,float sTha, float sx, float sy, float deltalen, string Cstyle,float time,
                                    float speed, char chr){
    float pathC =0, pCollide = 0;
    segement seg;
    while(pathC < length) {
        seg = seg_generation(pathC,c,sTha,sx,sy);
        if(Cstyle == "pathG")
            cout << seg.x << " " << seg.y << endl;
        else {
              float sex = seg.x, sey = seg.y;
            sex +=0.02; sey += 0.02;
            int lx = sex,ly = sey;
            int tl = lx<<16 | ly;//bit->obstacles.find(make_pair(seg.x,seg.y)) != bit->obstacles.end()
            if(bit->block[tl]){//bit->tob.find(tl) != bit->tob.end()
                ColidHap = true;
                return;
            }
            else {
                if(seg.x <=0 || seg.x > bit->map_x || seg.y <=0 || seg.y > bit->map_y) {//if(seg.x  <0 )cout << seg.x<< "  *********************** " << endl;
                    ColidHap = true;
                    return;
                }
                if(chr != '\0') {
                    float tCollide = 0, pNCollide = 1;
                    int id = 0;
                    for(DynamicObstacle &dyo : Dobstacle ) {
                        if(ObID.find(id) != ObID.end()) {
                            pNCollide *= (1 - get_probability(dyo, seg.x, seg.y, time +  pathC * rho/speed, 9999));
                        }
                        id++;
                    }
                    tCollide = 1 - pNCollide;
                    if(tCollide > pCollide)
                        pCollide = tCollide;
                }
            }
        }
        pathC += deltalen;
    }
    tpos = pCollide;
    ColidHap =false;
}

void BIT_star::Dubins::DyInfoUpdating(DynamicObstacle & dynamic, float t) {
    DynamicObstacle d;
    int m = (int) t;

    int index, cr;
    for(int i = 0; i < dynamic.instructions.size(); i++) {
        if(m >= dynamic.instructions[i].mTime && m < dynamic.instructions[i+1].mTime ) {
            index = i;
            break;
        }
    }
    dynamic.x = dynamic.instructions[index].x;
    dynamic.y = dynamic.instructions[index].y;
    dynamic.heading = dynamic.instructions[index].heading;
    dynamic.speed = dynamic.instructions[index].speed;
    dynamic.curTime = dynamic.instructions[index].mTime;
    //dynamic.steps = dynamic.instructions[index].steps;
}
float BIT_star::Dubins::EstimatedDvalue(states u, states v, float mtime, float vtime, int index, int des) {
      float pCollide = 0;
      for(DynamicObstacle & dyo : Dobstacle) {
          DyInfoUpdating(dyo,u.mTime);
      }
      states m;
      m.x = (float) (u.x+v.x)/2;
      m.y = (float) (u.y+v.y)/2;
     // cerr << u.mTime <<"  "<< mtime <<"  " << vtime << "  " << u.x << "  " << u.y << "  " << m.x << "  " << m.y << "  " << v.x << "   " << v.y <<endl;
      m.mTime = mtime;
      v.mTime = vtime;
      vector<states> tc;
      tc.push_back(u);
      tc.push_back(m);
      tc.push_back(v);
      esCheck = true;
      for(int i=0; i < tc.size(); i++) {
          float tCollide = 0, pNCollide = 1;
          for(DynamicObstacle &dyo : Dobstacle ) {
              pNCollide *= (1 - get_probability(dyo, tc[i].x, tc[i].y, tc[i].mTime,i));
          }
          tCollide = 1 - pNCollide;
          if(tCollide > pCollide)
              pCollide = tCollide;
      }
      esCheck = false;
      return pCollide*800;
}
float BIT_star::Dubins::get_probability(DynamicObstacle & dynamic, float x, float y, float t, int v) {
    DyInfoUpdating(dynamic,t);
    float dheading = dynamic.heading, dspeed = dynamic.speed;
    float dx = cos(dheading)*dspeed*(t-dynamic.curTime) + dynamic.x, dy = sin(dheading) * dspeed * (t-dynamic.curTime) + dynamic.y;
    float stderr = 1 + 0.1*(t-dynamic.curTime);//
    float var = stderr * stderr;
    float mux = x - dx, muy = y - dy;
    float first = 1 / (2.0 * M_PI * var);
    float second = -1 / 2.0;
    float third = ((mux * mux + muy * muy)) / var;
    float moving_dic = mod(- mod(dheading,2*M_PI) + atan2(muy,mux),2*M_PI);
    if(moving_dic >= 0.5*M_PI && moving_dic <= 1.5*M_PI){
        return 0;
    }
    if(angleT)
        return first * exp(second * third);
     return first * exp(second * third)*cos(moving_dic);
}


float BIT_star::Dubins::dysegcost(float length, char c, float sTha,float sx, float sy, float deltalen, float t, float aspeed, int v) {
    float pathC =0, pCollide = 0;
    segement seg;
    while(pathC < length) {
        seg = seg_generation(pathC,c,sTha,sx,sy);
        float tCollide = 0, pNCollide = 1;
        int id = 0;
        for(DynamicObstacle &dyo : Dobstacle ) {
            if(ObID.find(id) != ObID.end()) {
                pNCollide *= (1 - get_probability(dyo, seg.x, seg.y, t + (pathC * rho) / aspeed, v));
            }
            id++;
        }
        tCollide = 1 - pNCollide;
        if(tCollide > pCollide)
            pCollide = tCollide;
        pathC += deltalen;
    }
    return pCollide;
}
float BIT_star::Dubins::calculate_angle(DynamicObstacle & dynamic, float x, float y, float t, int v) {
    DyInfoUpdating(dynamic,t);
    float dheading = dynamic.heading, dspeed = dynamic.speed;
    float dx = cos(dheading)*dspeed*(t-dynamic.curTime) + dynamic.x, dy = sin(dheading) * dspeed * (t-dynamic.curTime) + dynamic.y;
    float mux = x - dx, muy = y - dy;
    float moving_dic = mod(- dheading+ atan2(muy,mux),2*M_PI);
    return moving_dic;
}
bool BIT_star::Dubins::is_same_direct(float moving_dic) {
    if(moving_dic >= 0.5*M_PI && moving_dic <= 1.5*M_PI){
        return false;
    }
    return true;
}
void BIT_star::Dubins::close_to_Obstacles(states u,states v,int index ,float t1, float t2) {
    int i = 0;
    float angle1 = 0, angle2 = 0;
    for(DynamicObstacle &dyo : Dobstacle ) {
        angle1 = calculate_angle(dyo,u.x,u.y,t1,index);
        angle2 = calculate_angle(dyo,v.x,v.y,t2,index);
        if(is_same_direct(angle1) && !is_same_direct(angle2)) {
            ObID.insert(i);
        }
        else if(is_same_direct(angle1) && is_same_direct(angle2)) {
            angleT = true;
            float p1 = get_probability(dyo, u.x, u.y, t1,index);
            float p2 = get_probability(dyo, v.x, v.y, t2,index);
            if((p1 > sigma || p2>sigma)) {
                ObID.insert(i);
            }
            angleT = false;
        }
        i++;
    }
}
float BIT_star::Dubins::dyCost(Dubins_curve duc,states u, states v, states org, int inv) {
    float sx = u.x, sy = u.y, sTha = u.theta, sTime = u.mTime, dyc = 0, aspeed = u.speed;
    string pT = duc.path_type;
    float d1, d2, d3;
    vector<segement> segPoints;
    unordered_set<int > O1;
    unordered_set<int > O2;
    segement seg;
    for(DynamicObstacle & dyo : Dobstacle) {
        DyInfoUpdating(dyo,u.mTime);
    }
    float eTime = u.mTime + (float) duc.relvar/v.speed;
    close_to_Obstacles(u,v,inv,u.mTime,eTime);
    if(ObID.size() == 0){
        return 0;
    }
    seg = seg_generation(duc.fst_seg,pT[0],u.theta,u.x,u.y);
    segPoints.push_back(seg);
    seg = seg_generation(duc.sec_seg,pT[1],seg.theta,seg.x,seg.y);
    segPoints.push_back(seg);
    float deltaLen = (float) 0.1;//duc.len/80
    d1 = dysegcost(duc.fst_seg,pT[0],sTha,sx,sy,deltaLen,sTime,aspeed,inv);
    float fTime = sTime + (duc.fst_seg*rho)/aspeed;
    d2 = dysegcost(duc.sec_seg,pT[1],segPoints[0].theta,segPoints[0].x,segPoints[0].y,deltaLen,fTime,aspeed,inv);
    float seTime = fTime + (duc.sec_seg*rho)/aspeed;
    d3 = dysegcost(duc.fin_seg,pT[2],segPoints[1].theta,segPoints[1].x,segPoints[1].y,deltaLen,seTime,aspeed,inv);
    dyc = max(d1,max(d2,d3));
    if(dyc > 1)
        dyc = 1;
    ObID.clear();
    return dyc*5500;
}
void BIT_star::Dubins::Generating_dubins_path(BIT_star *bit, Dubins_curve duc,states u, states v, string checking) {
    float sx = u.x, sy = u.y, sTha = u.theta, sTime = u.mTime, dyc =0, aspeed = u.speed, d1 = 0, d2 = 0, d3 = 0;
    string pT = duc.path_type;
    int inv = 55555555;
    char chr = '\0';
    vector<segement > segPoints;
    segement seg;
    for(DynamicObstacle & dyo : Dobstacle) {
        DyInfoUpdating(dyo,u.mTime);
    }
    float eTime = u.mTime + (float) duc.relvar/v.speed;
    close_to_Obstacles(u,v,inv,u.mTime,eTime);
    if(ObID.size() != 0){
        chr = 'p';
    }
    seg = seg_generation(duc.fst_seg,pT[0],u.theta,u.x,u.y);
    segPoints.push_back(seg);
    seg = seg_generation(duc.sec_seg,pT[1],seg.theta,seg.x,seg.y);
    segPoints.push_back(seg);
    float deltaLen = (float) 0.1;//duc.len/80
    drawing_path(bit,duc.fst_seg,pT[0],sTha,sx,sy,deltaLen,checking,sTime,aspeed,chr);    /*the first segment start: */
    if(ColidHap) {
        dynaCost = 0;
        return;
    }
    d1 = tpos;
    tpos = 0;
    float fTime = sTime + (duc.fst_seg*rho)/aspeed;
    drawing_path(bit,duc.sec_seg,pT[1],segPoints[0].theta,segPoints[0].x,segPoints[0].y,deltaLen,checking,fTime,aspeed,chr);
    if(ColidHap) {
        dynaCost = 0;
        return;
    }
    d2 = tpos;
    tpos = 0;
    float seTime = fTime + (duc.sec_seg*rho)/aspeed;
    drawing_path(bit,duc.fin_seg,pT[2],segPoints[1].theta,segPoints[1].x,segPoints[1].y,deltaLen,checking,seTime,aspeed,chr);
    if(ColidHap) {
        dynaCost = 0;
        return;
    }
    d3 = tpos;
    tpos = 0;
    dyc = max(d1,max(d2,d3));
    if(dyc > 1)
        dyc = 1;
    ObID.clear();
    dynaCost = dyc*5500;
    //if(checking == "pathG")
    //cout << v.x << " " << v.y << endl;
}
bool BIT_star::Dubins::Du_collision_free(BIT_star *bit, Dubins_curve path_get, states start, states end) {
    Generating_dubins_path(bit,path_get,start,end,"collison");
    if(!ColidHap)
        return true;
    cfree = true;
    return false;
}
