#include<iostream>
#include<algorithm>
#include<map>
#include<queue>
#include<stack>
#include<set>
#include<bitset>
#include<vector>
#include<string>
#include <iomanip>
#include <deque>
#include<list>
#include<cmath>
#include<fstream>
#define rep(i,n) for(int i=0;i<(n);i++)
using namespace std;
using ll=long long;
using P=pair<int,int>;
using P_S=pair<int,string>;
using P_D=pair<double,int>;
using T=tuple<int,int,char,ll,string>;
#define max_x_y 50
#define N 50//num of customer
#define Q 12//num of StopPoint
#define K 3//num of drone
#define POP 30000//population size
#define G 80//number of generations
#define S 5//tournament size

double pc=0.6;//probability of crossover
double pm=0.01;//probability of mutation
int c_pc=0;
int c_pm=0;
int best_score=1e9;
long time_start,time_finish;

struct Point{
    int x,y;
};

int m_x,m_y,m_z,move_place;

Point C[N];
Point Stop_Point[Q];
int w[N][Q];
vector<vector<int>>first_solution_where_todeliver(Q);
vector<vector<vector<int>>>X(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q
vector<vector<vector<int>>>Answer(POP,vector<vector<int>>(Q));//P[POP][Q]:Package delivered at point Q answer POP
vector<vector<int>>best_Answer(Q);

int score[POP]={0};//P[POP] score

void input(){
    ifstream input_file("instance.txt");
    rep(i,N){
        char a;
        input_file>>C[i].x>>a>>C[i].y;
    }
    rep(i,Q){
        char a;
        input_file>>Stop_Point[i].x>>a>>Stop_Point[i].y;
    }
    input_file.close();
}

void cal_dist_customer_StopPoint()
{
    for(int i=0;i<N;i++){
        for(int j=0;j<Q;j++){
            w[i][j]=ceil(sqrt((C[i].x-Stop_Point[j].x)*(C[i].x-Stop_Point[j].x)+(C[i].y-Stop_Point[j].y)*(C[i].y-Stop_Point[j].y)));
        }
    }
}

int swap_Stop_Point_dist(int a,int b){
    if(abs(a-b)>1){
        int d1=sqrt((Stop_Point[a-1].x-Stop_Point[a].x)*(Stop_Point[a-1].x-Stop_Point[a].x)+(Stop_Point[a-1].y-Stop_Point[a].y)*(Stop_Point[a-1].y-Stop_Point[a].y));
        int d2=sqrt((Stop_Point[a+1].x-Stop_Point[a].x)*(Stop_Point[a+1].x-Stop_Point[a].x)+(Stop_Point[a+1].y-Stop_Point[a].y)*(Stop_Point[a+1].y-Stop_Point[a].y));
        int d3=sqrt((Stop_Point[b-1].x-Stop_Point[b].x)*(Stop_Point[b-1].x-Stop_Point[b].x)+(Stop_Point[b-1].y-Stop_Point[b].y)*(Stop_Point[b-1].y-Stop_Point[b].y));
        int d4=sqrt((Stop_Point[b+1].x-Stop_Point[b].x)*(Stop_Point[b+1].x-Stop_Point[b].x)+(Stop_Point[b+1].y-Stop_Point[b].y)*(Stop_Point[b+1].y-Stop_Point[b].y));

        int d5=sqrt((Stop_Point[b-1].x-Stop_Point[a].x)*(Stop_Point[b-1].x-Stop_Point[a].x)+(Stop_Point[b-1].y-Stop_Point[a].y)*(Stop_Point[b-1].y-Stop_Point[a].y));
        int d6=sqrt((Stop_Point[b+1].x-Stop_Point[a].x)*(Stop_Point[b+1].x-Stop_Point[a].x)+(Stop_Point[b+1].y-Stop_Point[a].y)*(Stop_Point[b+1].y-Stop_Point[a].y));
        int d7=sqrt((Stop_Point[a-1].x-Stop_Point[b].x)*(Stop_Point[a-1].x-Stop_Point[b].x)+(Stop_Point[a-1].y-Stop_Point[b].y)*(Stop_Point[a-1].y-Stop_Point[b].y));
        int d8=sqrt((Stop_Point[a+1].x-Stop_Point[b].x)*(Stop_Point[a+1].x-Stop_Point[b].x)+(Stop_Point[a+1].y-Stop_Point[b].y)*(Stop_Point[a+1].y-Stop_Point[b].y));

        int c=d1+d2+d3+d4,d=d5+d6+d7+d8;
        if(c>d) return 1;
        return 0;
    }
    else{
        int d1=sqrt((Stop_Point[a-1].x-Stop_Point[a].x)*(Stop_Point[a-1].x-Stop_Point[a].x)+(Stop_Point[a-1].y-Stop_Point[a].y)*(Stop_Point[a-1].y-Stop_Point[a].y));
        int d2=sqrt((Stop_Point[b+1].x-Stop_Point[b].x)*(Stop_Point[b+1].x-Stop_Point[b].x)+(Stop_Point[b+1].y-Stop_Point[b].y)*(Stop_Point[b+1].y-Stop_Point[b].y));

        int d3=sqrt((Stop_Point[b+1].x-Stop_Point[a].x)*(Stop_Point[b+1].x-Stop_Point[a].x)+(Stop_Point[b+1].y-Stop_Point[a].y)*(Stop_Point[b+1].y-Stop_Point[a].y));
        int d4=sqrt((Stop_Point[a-1].x-Stop_Point[b].x)*(Stop_Point[a-1].x-Stop_Point[b].x)+(Stop_Point[a-1].y-Stop_Point[b].y)*(Stop_Point[a-1].y-Stop_Point[b].y));

        int c=d1+d2,d=d3+d4;
        if(c>d) return 1;
        return 0; 
    }
}

void truck_root_decide()//truck
{
    vector<vector<int>>dist(Q,vector<int>(Q));
    rep(i,Q){
        rep(j,Q){
            if(i==j) dist[i][j]=max_x_y*max_x_y;
            else dist[i][j]=ceil(sqrt((Stop_Point[i].x-Stop_Point[j].x)*(Stop_Point[i].x-Stop_Point[j].x)+(Stop_Point[i].y-Stop_Point[j].y)*(Stop_Point[i].y-Stop_Point[j].y)));
        }
    }
    //----------------Stop_Point-------------------
    vector<bool>check(Q,false);
    check[0]=true;
    check[Q-1]=true;
    int now=0,count=2;
    vector<int>root;
    root.push_back(now);
    while(count<Q){
        int next,min_dist=max_x_y*max_x_y;
        rep(i,Q){
            if(min_dist>dist[now][i]&&check[i]==false){
                min_dist=dist[now][i];
                next=i;
            }
        }
        check[next]=true;
        now=next;
        root.push_back(next);
        count++;
    }
    root.push_back(Q-1);
    vector<Point> new_Stop_Point(Q);
    rep(i,Q){
        new_Stop_Point[i]=Stop_Point[root[i]];
    }
    rep(i,Q) Stop_Point[i]=new_Stop_Point[i];
    /*rep(i,Q){
        cout<<Stop_Point[i].x<<" "<<Stop_Point[i].y<<endl;
    }*/
//---------------------------swap--------------------------------------
    count=0;
    while(count<5){
        for(int i=1;i<Q-1;i++){
            for(int j=i;j<Q-1;j++){
                if(i!=j){
                    if(swap_Stop_Point_dist(i,j)==1) {
                        swap(Stop_Point[i],Stop_Point[j]);
                    }
                }
            }
        }
        count++;
    }
//--------------
    ofstream outputfile("truck_root_1.txt");
    rep(i,Q){
        outputfile<<Stop_Point[i].x<<" "<<Stop_Point[i].y<<endl;
    }
    outputfile.close();
}

void decide_where_todeliver()//decide which stopPoint to deliver by random
{
    int count=0;
    while(count<POP){
        rep(i,N){
            int a=rand()%(Q-2);
            Answer[count][a+1].push_back(i);
            first_solution_where_todeliver[a+1].push_back(i);
        }
        bool check[N]={false};
        /*for(int i=1;i<Q-1;i++){
            rep(j,first_solution_where_todeliver[i].size()) check[first_solution_where_todeliver[i][j]]=true;
        }
        for(int i=1;i<Q-1;i++){
            rep(j,first_solution_where_todeliver[i].size()) cout<<first_solution_where_todeliver[i][j]<<" ";
            cout<<endl;
        }*/
        count++;
    }
}

void decide_drone_todeliver_by_LPT(vector<vector<int>>A)
{
    vector<vector<vector<int>>>x(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q
    for(int i=1;i<Q-1;i++){
        int time[K]={0};
        rep(j,A[i].size()){
            int min_processing_drone=-1,min_processing_time=1e9;
            rep(k,K){
                if(time[k]<min_processing_time){
                    min_processing_drone=k;
                    min_processing_time=time[k];
                }
            }
            x[min_processing_drone][i].push_back(A[i][j]);
            time[min_processing_drone]+=w[A[i][j]][i];
        }

        cout<<"Stop Point "<<i<<endl;
        rep(j,K){
            cout<<"drone "<<j<<":";
            rep(k,x[j][i].size()){
                cout<<x[j][i][k]<<" ";
                X[j][i].push_back(x[j][i][k]);//----------------------------
            }
            X[j][i].push_back(-1);//-------------------finish = -1
            cout<<"cost:"<<time[j]<<endl;
        }
        cout<<endl;
    }
}

void decide_drone_todeliver_by_LPT_and_score(vector<vector<int>>A,int number)
{
    vector<vector<vector<int>>>x(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q
    int sum=0;
    for(int i=1;i<Q-1;i++){
        int time[K]={0};
        vector<P>B;
        rep(j,A[i].size()){
            B.push_back({w[A[i][j]][i],A[i][j]});
        }
        sort(B.rbegin(),B.rend());
        rep(j,A[i].size()){
            int min_processing_drone=-1,min_processing_time=1e9;
            rep(k,K){
                if(time[k]<min_processing_time){
                    min_processing_drone=k;
                    min_processing_time=time[k];
                }
            }
            x[min_processing_drone][i].push_back(B[j].second);
            time[min_processing_drone]+=w[B[j].second][i];
        }
        int max_score=0;
        rep(j,K){
            if(max_score<time[j]) max_score=time[j];
        }
        //cout<<max_score<<endl;
        sum+=max_score;
    }
    score[number]=sum;
}

void solve_GA()
{
    int count=0;
    while(count<G){
        vector<vector<vector<int>>>next_Answer(POP,vector<vector<int>>(Q));//P[POP][Q]:Package delivered at point Q answer POP
        int num_answer=0;
        while(num_answer<POP){
            vector<vector<int>>P1(Q),P2(Q),O1(Q),O2(Q);
            int min_score_P1=1e9,min_index_P1=-1;
            rep(i,S) {
                int random=rand()%POP;
                if(score[random]<min_score_P1){
                    min_index_P1=random;
                    min_score_P1=score[random];
                }
            }
            rep(i,Q){
                rep(j,Answer[min_index_P1][i].size()){
                    P1[i].push_back(Answer[min_index_P1][i][j]);
                }
            }

            int min_score_P2=1e9,min_index_P2=-1;
            rep(i,S) {
                int random=rand()%POP;
                if(score[random]<min_score_P2){
                    min_index_P2=random;
                    min_score_P2=score[random];
                }
            }
            rep(i,Q){
                rep(j,Answer[min_index_P2][i].size()){
                    P2[i].push_back(Answer[min_index_P2][i][j]);
                }
            }
            double r=(double)rand()/RAND_MAX;
            //cout<<r<<endl;
            if(r<=pc) {
                //cout<<"pc"<<endl;
                c_pc++;
                //---------------------------debug--------------
                /*for(int i=1;i<Q-1;i++){
                    cout<<"[";
                    rep(j,P1[i].size()){
                        cout<<P1[i][j]<<" ";
                    }
                    cout<<"] ";
                }
                cout<<"score:"<<score[min_index_P1]<<endl;

                for(int i=1;i<Q-1;i++){
                    cout<<"[";
                    rep(j,P2[i].size()){
                        cout<<P2[i][j]<<" ";
                    }
                    cout<<"] ";
                }
                cout<<"score:"<<score[min_index_P2]<<endl;*/
                //----------------------------------------------------------------
                int k=rand()%(Q-2)+1;
                //cout<<"k:"<<k<<endl;
                vector<int>tmp1,tmp2;
                rep(i,P1[k].size()) tmp1.push_back(P1[k][i]);
                rep(i,P2[k].size()) tmp2.push_back(P2[k][i]);
                P1[k].resize(0);
                rep(i,tmp2.size()) P1[k].push_back(tmp2[i]);
                P2[k].resize(0);
                rep(i,tmp1.size()) P2[k].push_back(tmp1[i]);
                //--------------------------------------------------------------------
                /*for(int i=1;i<Q-1;i++){
                    cout<<"[";
                    rep(j,P1[i].size()){
                        cout<<P1[i][j]<<" ";
                    }
                    cout<<"] ";
                }
                cout<<endl;
                for(int i=1;i<Q-1;i++){
                    cout<<"[";
                    rep(j,P2[i].size()){
                        cout<<P2[i][j]<<" ";
                    }
                    cout<<"] ";
                }
                cout<<endl;*/
                //------------------------------------
                int seen[N]={0};
                rep(i,Q){
                    rep(j,P1[i].size()){
                        seen[P1[i][j]]++;
                    }
                }
                //cout<<"0:";
                rep(i,N){
                    if(seen[i]==0){
                        //cout<<i<<" ";
                        int  rr=rand()%(Q-2)+1;
                        P1[rr].push_back(i);
                    }
                }
                //cout<<endl<<"2:";
                rep(i,N){
                    if(seen[i]==2){
                        //cout<<i<<" ";
                        rep(j,Q){
                            if(j!=k){
                                rep(h,P1[j].size()){
                                    if(P1[j][h]==i) P1[j][h]=-1;
                                }

                            }
                        }
                    }
                }
                //cout<<endl;

                /*for(int i=1;i<Q-1;i++){
                    cout<<"[";
                    rep(j,P1[i].size()){
                        cout<<P1[i][j]<<" ";
                    }
                    cout<<"]";
                }
                cout<<endl;*/
                for(int i=1;i<Q-1;i++){
                    rep(j,P1[i].size()){
                        if(P1[i][j]!=-1) O1[i].push_back(P1[i][j]);
                    }
                }
                /*for(int i=1;i<Q-1;i++){
                    cout<<"[";
                    rep(j,O1[i].size()){
                        cout<<O1[i][j]<<" ";
                    }
                    cout<<"]";
                }
                cout<<endl;*/
                //--------------------------------------------------------
                rep(i,N) seen[i]=0;
                rep(i,Q){
                    rep(j,P2[i].size()){
                        seen[P2[i][j]]++;
                    }
                }
                //cout<<"0:";
                rep(i,N){
                    if(seen[i]==0){
                        //cout<<i<<" ";
                        int  rr=rand()%(Q-2)+1;
                        P2[rr].push_back(i);
                    }
                }
                //cout<<endl<<"2:";
                rep(i,N){
                    if(seen[i]==2){
                        //cout<<i<<" ";
                        rep(j,Q){
                            if(j!=k){
                                rep(h,P2[j].size()){
                                    if(P2[j][h]==i) P2[j][h]=-1;
                                }

                            }
                        }
                    }
                }
                //cout<<endl;

                /*for(int i=1;i<Q-1;i++){
                    cout<<"[";
                    rep(j,P2[i].size()){
                        cout<<P2[i][j]<<" ";
                    }
                    cout<<"]";
                }
                cout<<endl;*/
                for(int i=1;i<Q-1;i++){
                    rep(j,P2[i].size()){
                        if(P2[i][j]!=-1) O2[i].push_back(P2[i][j]);
                    }
                }
                /*for(int i=1;i<Q-1;i++){
                    cout<<"[";
                    rep(j,O2[i].size()){
                        cout<<O2[i][j]<<" ";
                    }
                    cout<<"]";
                }
                cout<<endl;*/
            }

            //--------------------------------------------------------------------
            else if(pc<r&&r<=pc+pm){
                //cout<<"pm"<<endl;
                c_pm++;
                m_x=rand()%(Q-2)+1;
                //cout<<"m_x"<<m_x<<endl;
                if(P1[m_x].size()!=0){
                    m_y=rand()%P1[m_x].size();
                    //cout<<"m_y"<<m_y<<endl;
                    move_place=P1[m_x][m_y];
                    P1[m_x][m_y]=-1;
                   // cout<<"move"<<move_place<<endl;
                    m_z=rand()%(Q-2)+1;
                    //cout<<"m_z"<<m_z<<endl;
                    P1[m_z].push_back(move_place);
                }

                m_x=rand()%(Q-2)+1;
                //cout<<"m_x"<<m_x<<endl;
                if(P2[m_x].size()!=0){
                    m_y=rand()%P2[m_x].size();
                    //cout<<"m_y"<<m_y<<endl;
                    move_place=P2[m_x][m_y];
                    P2[m_x][m_y]=-1;
                   // cout<<"move"<<move_place<<endl;
                    m_z=rand()%(Q-2)+1;
                    //cout<<"m_z"<<m_z<<endl;
                    P2[m_z].push_back(move_place);
                }

                for(int i=1;i<Q-1;i++){
                    rep(j,P1[i].size()){
                        if(P1[i][j]!=-1) O1[i].push_back(P1[i][j]);
                    }
                }

                for(int i=1;i<Q-1;i++){
                    rep(j,P2[i].size()){
                        if(P2[i][j]!=-1) O2[i].push_back(P2[i][j]);
                    }
                }
            }
            //-------------------------------------------------
            else {
                //cout<<"copy"<<endl;
                for(int i=1;i<Q-1;i++){
                    rep(j,P1[i].size()){
                        if(P1[i][j]!=-1) O1[i].push_back(P1[i][j]);
                    }
                }

                for(int i=1;i<Q-1;i++){
                    rep(j,P2[i].size()){
                        if(P2[i][j]!=-1) O2[i].push_back(P2[i][j]);
                    }
                }
  
            }
            //---------------------------------------------
            for(int i=1;i<Q-1;i++){
                rep(j,O1[i].size()){
                    next_Answer[num_answer][i].push_back(O1[i][j]);
                }
            }
            for(int i=1;i<Q-1;i++){
                rep(j,O2[i].size()){
                    next_Answer[num_answer+1][i].push_back(O2[i][j]);
                }
            }

            num_answer=num_answer+2;
        }

        rep(i,POP){
            //cout<<i<<endl;
            for(int j=1;j<Q-1;j++){
                Answer[i][j].clear();
                rep(h,next_Answer[i][j].size()){
                    Answer[i][j].push_back(next_Answer[i][j][h]);
                }
            }
        }
        int min_index=-1,min_score=1e9;
        rep(i,POP){
            decide_drone_todeliver_by_LPT_and_score(Answer[i],i);
            if(min_score>score[i]){
                min_score=score[i];
                min_index=i;
            }
            //cout<<score[i]<<endl;
        }
        cout<<score[min_index]<<endl;

        if(best_score>score[min_index]){
            best_score=score[min_index];
            for(int i=1;i<Q-1;i++){
                best_Answer[i].clear();
                rep(j,Answer[min_index][i].size()){
                    best_Answer[i].push_back(Answer[min_index][i][j]);
                }
            }
        }
        count++;
    }
}

void last_decide_drone_todeliver_by_LPT_and_score(vector<vector<int>>A,int number)
{
    vector<vector<vector<int>>>x(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q
    for(int i=1;i<Q-1;i++){
        int time[K]={0};
        vector<P>B;
        rep(j,A[i].size()){
            B.push_back({w[A[i][j]][i],A[i][j]});
        }
        sort(B.rbegin(),B.rend());
        rep(j,A[i].size()){
            int min_processing_drone=-1,min_processing_time=1e9;
            rep(k,K){
                if(time[k]<min_processing_time){
                    min_processing_drone=k;
                    min_processing_time=time[k];
                }
            }
            x[min_processing_drone][i].push_back(B[j].second);
            time[min_processing_drone]+=w[B[j].second][i];
        }
        cout<<"Stop Point "<<i<<endl;
        rep(j,K){
            cout<<"drone "<<j<<":";
            rep(k,x[j][i].size()){
                cout<<x[j][i][k]<<" ";
                X[j][i].push_back(x[j][i][k]);//----------------------------
            }
            X[j][i].push_back(-1);//-------------------finish = -1
            cout<<"cost:"<<time[j]<<endl;
        }
        cout<<endl;
    }
}

void real_last_decide_drone_todeliver_by_LPT_and_score(vector<vector<int>>A)
{
    vector<vector<vector<int>>>x(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q
    for(int i=1;i<Q-1;i++){
        int time[K]={0};
        vector<P>B;
        rep(j,A[i].size()){
            B.push_back({w[A[i][j]][i],A[i][j]});
        }
        sort(B.rbegin(),B.rend());
        rep(j,A[i].size()){
            int min_processing_drone=-1,min_processing_time=1e9;
            rep(k,K){
                if(time[k]<min_processing_time){
                    min_processing_drone=k;
                    min_processing_time=time[k];
                }
            }
            x[min_processing_drone][i].push_back(B[j].second);
            time[min_processing_drone]+=w[B[j].second][i];
        }
        cout<<"Stop Point "<<i<<endl;
        rep(j,K){
            cout<<"drone "<<j<<":";
            rep(k,x[j][i].size()){
                cout<<x[j][i][k]<<" ";
                X[j][i].push_back(x[j][i][k]);//----------------------------
            }
            X[j][i].push_back(-1);//-------------------finish = -1
            cout<<"cost:"<<time[j]<<endl;
        }
        cout<<endl;
    }
}


void output_customer_place()
{
    ofstream outputfile("customer_place_1.txt");
    rep(i,N){
        outputfile<<C[i].x<<" "<<C[i].y<<endl;
    }
    outputfile.close();
}

void output_answer(vector<vector<vector<int>>>x)
{
    ofstream outputfile("answer_1.txt");
    for(int i=1;i<Q-1;i++){
        rep(j,K){
            rep(k,x[j][i].size()){
                if(k==x[j][i].size()-1) outputfile<<x[j][i][k];
                else outputfile<<x[j][i][k]<<" ";
            }
            outputfile<<endl;
        }
    }
}

void output_txt()
{
    ofstream outputfile("result_1.txt");
    for(int i=1;i<Q-1;i++){
        outputfile<<"Stop Point "<<i<<endl;
        rep(j,K){
            int cost=0,em=15;
            vector<int>a;
            outputfile<<"drone "<<j+1<<" : ";
            rep(k,X[j][i].size()-1){
                em-=3;
                if(X[j][i][k]<10) em++;
                cost+=w[X[j][i][k]][i];
                a.push_back(w[X[j][i][k]][i]);
                outputfile<<" "<<X[j][i][k];
            }
            rep(k,em) outputfile<<" ";
            outputfile<<"cost : ";
            rep(k,a.size()) {
                if(k==a.size()-1) outputfile<<a[k]<<"=";
                else outputfile<<a[k]<<"+";
            }
            outputfile<<cost<<endl;
        }
        outputfile<<endl;
    }
    outputfile<<"best score : "<<best_score<<endl;
    outputfile<<"time : "<<(double)(time_finish-time_start)/CLOCKS_PER_SEC;
}

int main()
{
    srand((unsigned int)time(NULL));
    input();
    time_start=clock();
    truck_root_decide();
    cal_dist_customer_StopPoint();
    decide_where_todeliver();
    int min_answer=1e9,min_index=-1;
    rep(i,POP){
        decide_drone_todeliver_by_LPT_and_score(Answer[i],i);
        if(min_answer>score[i]){
            min_answer=score[i];
            min_index=i;
        }
    }
    cout<<score[min_index]<<endl;
    solve_GA();
    time_finish=clock();
    cout<<"num of pc:"<<c_pc<<endl;
    cout<<"num of pm:"<<c_pm<<endl;  
    cout<<"finish"<<endl;
    real_last_decide_drone_todeliver_by_LPT_and_score(best_Answer);
    cout<<"score : "<<best_score<<endl;
    cout<<"time : "<<(double)(time_finish-time_start)/CLOCKS_PER_SEC<<endl;
    output_customer_place();
    output_txt();
    output_answer(X);
}