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

int best_score=1e9;
double start_temp;
double end_temp=0;


struct Point{
    int x,y;
};

Point C[N];
Point Stop_Point[Q];
int w[N][Q];
vector<vector<int>>first_solution_where_todeliver(Q);
vector<vector<int>>best_solution_where_todeliver(Q);
int best_solution_score;
vector<vector<vector<int>>>X(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q
long time_start,time_finish,start,finish;
double limit_time=5,annealing_limit_time=60;
double annealing_counter=0;
double annealing_show=(double)annealing_limit_time/400;
vector<pair<double,int>>processing_result;

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

int cal_score(vector<vector<int>>A){
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
    //cout<<sum<<endl;
    return sum;
}

void insert_search(vector<vector<int>>A)
{
        int x;
        while(1){
            x=rand()%(Q-2)+1;
            if(A[x].size()!=0) break;
        }
        int y=rand()%A[x].size();
        int z;
        while(1){
            z=rand()%(Q-2)+1;
            if(z!=x) break;
        }
        vector<vector<int>>copy_answer(Q);
        for(int i=1;i<Q-1;i++){
            rep(j,A[i].size()){
                if(i==x&&j==y) copy_answer[z].push_back(A[i][j]);
                else copy_answer[i].push_back(A[i][j]);
            }
        }
        int copy_score=cal_score(copy_answer); 
        if(copy_score<best_score){
            cout<<"insert:"<<copy_score<<endl;
            rep(i,Q){
                first_solution_where_todeliver[i].clear();
                rep(j,copy_answer[i].size()){
                    first_solution_where_todeliver[i].push_back(copy_answer[i][j]);
                }
            }
        }
}

void swap_search(vector<vector<int>>A)
{
        vector<vector<int>>copy_answer(Q);
        for(int i=1;i<Q-1;i++){
            rep(j,A[i].size()){
                copy_answer[i].push_back(A[i][j]);
            }
        }
        int x,y;
        while(1){
            x=rand()%(Q-2)+1;
            if(copy_answer[x].size()!=0){
                y=rand()%copy_answer[x].size();
                break;
            }
        }
        int z,w;
        while(1){
            z=rand()%(Q-2)+1;
            if(copy_answer[z].size()!=0){
                w=rand()%copy_answer[z].size();
                break;
            }
        }       
        swap(copy_answer[x][y],copy_answer[z][w]);
        int copy_score=cal_score(copy_answer); 
        if(copy_score<best_score){
            best_score=copy_score;
            cout<<"insert :"<<copy_score<<endl;
            rep(i,Q){
                first_solution_where_todeliver[i].clear();
                rep(j,copy_answer[i].size()){
                    first_solution_where_todeliver[i].push_back(copy_answer[i][j]);
                }
            }
        }
}

void annealing_swap(vector<vector<int>>A){
        vector<vector<int>>copy_answer(Q);
        for(int i=1;i<Q-1;i++){
            rep(j,A[i].size()){
                copy_answer[i].push_back(A[i][j]);
            }
        }
        int x,y;
        while(1){
            x=rand()%(Q-2)+1;
            if(copy_answer[x].size()!=0){
                y=rand()%copy_answer[x].size();
                break;
            }
        }
        int z,w;
        while(1){
            z=rand()%(Q-2)+1;
            if(copy_answer[z].size()!=0){
                w=rand()%copy_answer[z].size();
                break;
            }
        }       
        swap(copy_answer[x][y],copy_answer[z][w]);
        int copy_score=cal_score(copy_answer); 
        double tmp=start_temp+(end_temp-start_temp)*((double)(time_finish-time_start)/CLOCKS_PER_SEC)/annealing_limit_time;
        //cout<<limit_time-(double)(finish-start)/CLOCKS_PER_SEC<<endl;
        double prob=exp((best_score-copy_score)/tmp);
        double p=(double)(rand()%RAND_MAX)/RAND_MAX;
        if(prob>p){
            if((double)(time_finish-time_start)/CLOCKS_PER_SEC-annealing_counter>annealing_show){
                processing_result.push_back({(double)(time_finish-time_start)/CLOCKS_PER_SEC,copy_score});
                annealing_counter+=annealing_show;
                    cout<<copy_score<<endl;
            }
            best_score=copy_score;
            rep(i,Q){
                first_solution_where_todeliver[i].clear();
                rep(j,copy_answer[i].size()){
                    first_solution_where_todeliver[i].push_back(copy_answer[i][j]);
                }
            }
            if(best_score<best_solution_score){
                best_solution_score=best_score;
                rep(i,Q){
                    best_solution_where_todeliver[i].clear();
                    rep(j,first_solution_where_todeliver[i].size()){
                        best_solution_where_todeliver[i].push_back(first_solution_where_todeliver[i][j]);
                    }   
                }

            }
        }
  
}

void annealing_insert(vector<vector<int>>A)
{
        int x;
        while(1){
            x=rand()%(Q-2)+1;
            if(A[x].size()!=0) break;
        }
        int y=rand()%A[x].size();
        int z;
        while(1){
            z=rand()%(Q-2)+1;
            if(z!=x) break;
        }
        vector<vector<int>>copy_answer(Q);
        for(int i=1;i<Q-1;i++){
            rep(j,A[i].size()){
                if(i==x&&j==y) copy_answer[z].push_back(A[i][j]);
                else copy_answer[i].push_back(A[i][j]);
            }
        }
        int copy_score=cal_score(copy_answer); 
        double tmp=start_temp+(end_temp-start_temp)*((double)(time_finish-time_start)/CLOCKS_PER_SEC)/annealing_limit_time;
        //cout<<limit_time-(double)(finish-start)/CLOCKS_PER_SEC<<endl;
        double prob=exp((best_score-copy_score)/tmp);
        double p=(double)(rand()%RAND_MAX)/RAND_MAX;
        if(prob>p){
            if((double)(time_finish-time_start)/CLOCKS_PER_SEC-annealing_counter>annealing_show){
                processing_result.push_back({(double)(time_finish-time_start)/CLOCKS_PER_SEC,copy_score});
                annealing_counter+=annealing_show;
                cout<<copy_score<<endl;
            }
            best_score=copy_score;
            rep(i,Q){
                first_solution_where_todeliver[i].clear();
                rep(j,copy_answer[i].size()){
                    first_solution_where_todeliver[i].push_back(copy_answer[i][j]);
                }
            }
            if(best_score<best_solution_score){
                best_solution_score=best_score;
                rep(i,Q){
                    best_solution_where_todeliver[i].clear();
                    rep(j,first_solution_where_todeliver[i].size()){
                        best_solution_where_todeliver[i].push_back(first_solution_where_todeliver[i][j]);
                    }   
                }

            }
        }
  
}
void output_txt()
{
    ofstream outputfile("result_3.txt");
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
    outputfile<<"best score : "<<best_solution_score<<endl;
    outputfile<<"time : "<<(double)(time_finish-time_start)/CLOCKS_PER_SEC;
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
    ofstream outputfile("answer_3.txt");
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
void output_annealing_result()
{
    ofstream outputfile("annealing_result_6.txt");
    rep(i,processing_result.size()){
        outputfile<<processing_result[i].first<<" "<<processing_result[i].second<<endl;
    }
}

int main()
{
    srand((unsigned int)time(NULL));
    input();
    //truck_root_decide();
    cal_dist_customer_StopPoint();
    decide_todeliver_near();
    best_score=cal_score(first_solution_where_todeliver);
    cout<<best_score<<endl;
    start=clock();
    while((double)(finish-start)/CLOCKS_PER_SEC<limit_time){
        int ss=rand()%100;
        if(ss<60) swap_search(first_solution_where_todeliver);
        else insert_search(first_solution_where_todeliver);
        finish=clock();
    }

    rep(i,Q){
        rep(j,first_solution_where_todeliver[i].size()){
            best_solution_where_todeliver[i].push_back(first_solution_where_todeliver[i][j]);
        }
    }
    best_solution_score=best_score;
    time_start=clock();
    time_finish=clock();
    start_temp=max((double)best_score/100,1.0);
    processing_result.push_back({0,best_score});
    while((double)(time_finish-time_start)/CLOCKS_PER_SEC<annealing_limit_time){
        int ss=rand()%100;
        if(ss<40)annealing_swap(first_solution_where_todeliver);
        else annealing_insert(first_solution_where_todeliver);
        time_finish=clock();
    }
    cout<<"best_score : "<<best_solution_score<<endl;
    decide_drone_todeliver_by_LPT(best_solution_where_todeliver);
    output_txt();
    output_customer_place();
    output_answer(X);
    output_annealing_result();
}