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
#define K 7//num of drone

int best_score=1e9;

struct Point{
    int x,y;
};

Point C[N];
Point Stop_Point[Q];
int w[N][Q];
vector<vector<int>>first_solution_where_todeliver(Q);
vector<vector<vector<int>>>X(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q
long start,finish;
double limit_time=3.0;

void input(){
    ifstream input_file("instance_1.txt");
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

void decide_todeliver_near()
{
    rep(i,N){
        int min_distance=1e9,index=-1;
        for(int j=1;j<Q-1;j++){
            if(min_distance>w[i][j]){
                min_distance=w[i][j];
                index=j;
            }
        }
        first_solution_where_todeliver[index].push_back(i);
    }
    /*for(int i=1;i<Q-1;i++){
        cout<<"Stop_point"<<i<<":[";
        rep(j,first_solution_where_todeliver[i].size()){
            cout<<first_solution_where_todeliver[i][j]<<" ";
        }
        cout<<"]"<<endl;
    }*/
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

        rep(j,K){
            rep(k,x[j][i].size()){
                X[j][i].push_back(x[j][i][k]);//----------------------------
            }
            X[j][i].push_back(-1);//-------------------finish = -1
        }
    }
}

void cal_score(vector<vector<int>>A){
    vector<vector<vector<int>>>x(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q
    int sum=0;
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
        int max_score=0;
        rep(j,K){
            if(max_score<time[j]) max_score=time[j];
        }
        //cout<<max_score<<endl;
        sum+=max_score;
    }
    if(best_score>sum) {
        best_score=sum;
        cout<<sum<<endl;
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
    ofstream outputfile("answer_2.txt");
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

int main()
{
    srand((unsigned int)time(NULL));
    input();
    truck_root_decide();
    cal_dist_customer_StopPoint();
    decide_todeliver_near();
    cal_score(first_solution_where_todeliver);

    start=clock();
    while((double)(finish-start)/CLOCKS_PER_SEC<limit_time){
        vector<vector<int>>copy_answer(Q);
        for(int i=1;i<Q-1;i++){
            rep(j,first_solution_where_todeliver[i].size()){
                copy_answer[i].push_back(first_solution_where_todeliver[i][j]);
            }
        }
        int x=rand()%(Q-2)+1;
        if(copy_answer[x].size()!=0){
            int y=rand()%copy_answer[x].size();
            int z;
            while(1){
                z=rand()%(Q-2)+1;
                if(z!=x) break;
            }
            copy_answer[z].push_back(copy_answer[x][y]);
        }
        int pre_score=best_score;
        cal_score(copy_answer); 
        if(pre_score>best_score){
            cout<<"update"<<endl;
            rep(i,Q){
                first_solution_where_todeliver[i].clear();
                rep(j,copy_answer[i].size()){
                    first_solution_where_todeliver[i].push_back(copy_answer[i][j]);
                }
            }
        }
        finish=clock();
    }
    decide_drone_todeliver_by_LPT(first_solution_where_todeliver);
    output_customer_place();
    output_answer(X);
}