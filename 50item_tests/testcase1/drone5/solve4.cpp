/*ドローンがトラックに戻らず次の停止ポイントに行くのもOK問題
やること
・2地点間の距離をユークリッド距離にする、往復はかける２する
・ドローンのスケジューリングの際一番飛行時間かかるものを各ドローンごとに最後に配達するようにする
・次の停止ポイントに移動した際にドローンが使えるか判定する変数追加
・pythonの表示方法考える
*/
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
using P=pair<double,int>;
using P_S=pair<int,string>;
using P_D=pair<double,int>;
#define max_x_y 50
#define N 50//num of customer
#define Q 12//num of StopPoint
#define K 5//num of drone

double best_score=1e9;

struct Point{
    int x,y;
};

Point C[N];
Point Stop_Point[Q];
double w[N][Q];//顧客ｎから停止ポイントｑの往復距離
double dist_by_truck[Q][Q];
vector<vector<int>>solution_where_todeliver(Q);
vector<vector<vector<int>>>X(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q
long start,finish,time_start,time_finish;
double limit_time=3.0;
double T=2;//トラックの移動距離にかける係数(ドローンより移動速度遅いこと表現)
vector<vector<double>>best_late_drone(K,vector<double>(Q,0));//停止ポイントQでドローンKがどのくらい遅れてくるか
vector<vector<bool>>best_fly_next_Point(K,vector<bool>(Q,false));//停止ポイントQでドローンKを最後Q+１に飛ばすか

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
            w[i][j]=2*sqrt((C[i].x-Stop_Point[j].x)*(C[i].x-Stop_Point[j].x)+(C[i].y-Stop_Point[j].y)*(C[i].y-Stop_Point[j].y));
        }
    }
}

void cal_dist_truck_StopPoint()
{
    for(int i=0;i<Q;i++){
        for(int j=0;j<Q;j++){
            dist_by_truck[i][j]=T*sqrt((Stop_Point[i].x-Stop_Point[j].x)*(Stop_Point[i].x-Stop_Point[j].x)+(Stop_Point[i].y-Stop_Point[j].y)*(Stop_Point[i].y-Stop_Point[j].y));
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
        double min_distance=1e9;
        int index=-1;
        for(int j=1;j<Q-1;j++){
            if(min_distance>w[i][j]){
                min_distance=w[i][j];
                index=j;
            }
        }
        solution_where_todeliver[index].push_back(i);
    }
    /*for(int i=1;i<Q-1;i++){
        cout<<"Stop_point"<<i<<":[";
        rep(j,solution_where_todeliver[i].size()){
            cout<<solution_where_todeliver[i][j]<<" ";
        }
        cout<<"]"<<endl;
    }*/
}

void decide_drone_todeliver_by_LPT(vector<vector<int>>A)
{
    vector<vector<vector<int>>>x(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q
    double sum=0;
    vector<vector<double>>late_drone(K,vector<double>(Q,0));//停止ポイントQでドローンKがどのくらい遅れてくるか
    vector<vector<bool>>fly_next_Point(K,vector<bool>(Q,false));//停止ポイントQでドローンKを最後Q+１に飛ばすか
    for(int i=1;i<Q-1;i++){
        double time[K]={0};
        rep(j,K){
            time[j]=late_drone[j][i];
        }
        vector<P>B;
        rep(j,A[i].size()){
            B.push_back({w[A[i][j]][i],A[i][j]});
        }
        sort(B.rbegin(),B.rend());
//K台のドローン次の停止ポイントに飛ばすとする
        if(B.size()<K){
            rep(j,B.size()){
                x[j][i].push_back(B[j].second);
                late_drone[j][i+1]=max(0.0,B[j].first/2+w[B[j].second][i+1]/2-dist_by_truck[i][i+1]);
                fly_next_Point[j][i]=true;
            }
        }
        else{
            for(int j=K;j<B.size();j++){
                int min_processing_drone=-1;
                double min_processing_time=1e9;
                rep(k,K){
                    if(time[k]<min_processing_time){
                        min_processing_drone=k;
                        min_processing_time=time[k];
                    }
                }
                x[min_processing_drone][i].push_back(B[j].second);
                time[min_processing_drone]+=w[B[j].second][i];
            }
            int dd[K];
            rep(j,K) dd[j]=1;
            rep(j,K){
                int min_processing_drone=0;
                double min_processing_time=1e9;
                double depart_truck_time=0;
                rep(k,K){
                    if(dd[k]==1&&time[k]<min_processing_time){
                        min_processing_drone=k;
                        min_processing_time=time[k];
                    }
                    if(time[k]>depart_truck_time) depart_truck_time=time[k];
                }
                fly_next_Point[min_processing_drone][i]=true;
                dd[min_processing_drone]=0;
                x[min_processing_drone][i].push_back(B[j].second);
                late_drone[j][i+1]=max(0.0,B[j].first/2+w[B[j].second][i+1]/2-dist_by_truck[i][i+1]-(depart_truck_time-time[j]));          
            }

        }

        double max_score=0;
        rep(j,K){
            if(max_score<time[j]) max_score=time[j];
        }
        sum+=max_score;
        rep(i,K) sum+=late_drone[i][Q-1];

        cout<<"Stop Point "<<i<<endl;
        //printf(" %.2f\n",max_score);
        rep(j,K){
            cout<<"drone "<<j<<":";
            rep(k,x[j][i].size()){
                cout<<x[j][i][k]<<" ";
                X[j][i].push_back(x[j][i][k]);//----------------------------
            }
            if(fly_next_Point[j][i]) X[j][i+1].push_back(x[j][i][x[j][i].size()-1]);
            X[j][i].push_back(-1);//-------------------finish = -1
            cout<<"cost:";
            if(late_drone[j][i]>0) printf("%.2f ",late_drone[j][i]);
            rep(k,x[j][i].size()){
                if(k==x[j][i].size()-1){
                    if(!fly_next_Point[j][i]) printf("%.2f ",w[x[j][i][k]][i]);
                }
                else printf("%.2f ",w[x[j][i][k]][i]);
            }
            printf(" = %.2f\n",time[j]);
        }
        cout<<endl;
    }
    rep(i,K) X[i][Q-1].push_back(-1);
    rep(i,K){
        rep(j,Q){
            best_fly_next_Point[i][j]=fly_next_Point[i][j];
            best_late_drone[i][j]=late_drone[i][j];
        }
    }
}

double cal_score(vector<vector<int>>A){
    vector<vector<vector<int>>>x(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q
    double sum=0;
    for(int i=1;i<Q-1;i++){
        double time[K]={0};
        vector<P>B;
        rep(j,A[i].size()){
            B.push_back({w[A[i][j]][i],A[i][j]});
        }
        sort(B.rbegin(),B.rend());
        rep(j,B.size()){
            int min_processing_drone=-1;
            double min_processing_time=1e9;
            rep(k,K){
                if(time[k]<min_processing_time){
                    min_processing_drone=k;
                    min_processing_time=time[k];
                }
            }
            x[min_processing_drone][i].push_back(B[j].second);
            time[min_processing_drone]+=w[B[j].second][i];
        }
        double max_score=0;
        rep(j,K){
            if(max_score<time[j]) max_score=time[j];
        }
        cout<<i<<":"<<max_score<<endl;
        sum+=max_score;
    }
    //cout<<sum<<endl;
    return sum;
}

double cal_score_dronenextPoint(vector<vector<int>>A){
    vector<vector<vector<int>>>x(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q
    double sum=0;
    vector<vector<double>>late_drone(K,vector<double>(Q,0));//停止ポイントQでドローンKがどのくらい遅れてくるか
    vector<vector<bool>>fly_next_Point(K,vector<bool>(Q,false));//停止ポイントQでドローンKを最後Q+１に飛ばすか
    for(int i=1;i<Q-1;i++){
        double time[K]={0};
        rep(j,K){
            time[j]=late_drone[j][i];
        }
        vector<P>B;
        rep(j,A[i].size()){
            B.push_back({w[A[i][j]][i],A[i][j]});
        }
        sort(B.rbegin(),B.rend());
//K台のドローン次の停止ポイントに飛ばすとする
        if(B.size()<K){
            rep(j,B.size()){
                x[j][i].push_back(B[j].second);
                late_drone[j][i+1]=max(0.0,B[j].first/2+w[B[j].second][i+1]/2-dist_by_truck[i][i+1]);
                fly_next_Point[j][i]=true;
            }
        }
        else{
            for(int j=K;j<B.size();j++){
                int min_processing_drone=-1;
                double min_processing_time=1e9;
                rep(k,K){
                    if(time[k]<min_processing_time){
                        min_processing_drone=k;
                        min_processing_time=time[k];
                    }
                }
                x[min_processing_drone][i].push_back(B[j].second);
                time[min_processing_drone]+=w[B[j].second][i];
            }
            int dd[K];
            rep(j,K) dd[j]=1;
            rep(j,K){
                int min_processing_drone=0;
                double min_processing_time=1e9;
                double depart_truck_time=0;
                rep(k,K){
                    if(dd[k]==1&&time[k]<min_processing_time){
                        min_processing_drone=k;
                        min_processing_time=time[k];
                    }
                    if(time[k]>depart_truck_time) depart_truck_time=time[k];
                }
                fly_next_Point[min_processing_drone][i]=true;
                dd[min_processing_drone]=0;
                x[min_processing_drone][i].push_back(B[j].second);
                late_drone[j][i+1]=max(0.0,B[j].first/2+w[B[j].second][i+1]/2-dist_by_truck[i][i+1]-(depart_truck_time-time[j]));          
            }

        }

        double max_score=0;
        rep(j,K){
            if(max_score<time[j]) max_score=time[j];
        }
        sum+=max_score;

        /*cout<<"Stop Point "<<i<<endl;
        rep(j,K){
            cout<<"drone "<<j<<":";
            rep(k,x[j][i].size()){
                cout<<x[j][i][k]<<" ";
            }
            cout<<" cost:";
            rep(k,x[j][i].size()){
                printf("%.2f ",w[x[j][i][k]][i]);
            }
            cout<<"     "<<"late time next:"<<late_drone[j]<<endl;
        }
        cout<<endl;*/
    }
    //cout<<sum<<endl;
    rep(i,K) sum+=late_drone[i][Q-1];
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
        double copy_score=cal_score_dronenextPoint(copy_answer); 
        if(copy_score<best_score){
            cout<<"insert:"<<copy_score<<endl;
            rep(i,Q){
                solution_where_todeliver[i].clear();
                rep(j,copy_answer[i].size()){
                    solution_where_todeliver[i].push_back(copy_answer[i][j]);
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
        double copy_score=cal_score_dronenextPoint(copy_answer); 
        if(copy_score<best_score){
            best_score=copy_score;
            cout<<"swap:"<<copy_score<<endl;
            rep(i,Q){
                solution_where_todeliver[i].clear();
                rep(j,copy_answer[i].size()){
                    solution_where_todeliver[i].push_back(copy_answer[i][j]);
                }
            }
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
    ofstream outputfile("answer_4.txt");
    for(int i=1;i<Q;i++){
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
    ofstream outputfile("result_4.txt");
    for(int i=1;i<Q-1;i++){
        outputfile<<"Stop Point "<<i<<endl;
        rep(j,K){
            int em=20;
            double cost=0;
            vector<double>a;
            outputfile<<"drone "<<j+1<<" : ";
            rep(k,X[j][i].size()-1){
                em-=3;
                if(X[j][i][k]<10) em++;
                if(X[j][i].size()==1&&best_fly_next_Point[j][i]){
                        outputfile<<" "<<X[j][i][k];                   
                }
                else if(k==0){
                    if(best_fly_next_Point[j][i-1]==true){
                        cost+=best_late_drone[j][i];
                        a.push_back(best_late_drone[j][i]);
                    }
                    else{
                        cost+=w[X[j][i][k]][i];
                        a.push_back(w[X[j][i][k]][i]);
                        outputfile<<" "<<X[j][i][k];
                    }
                }
                else if(k==X[j][i].size()-2){
                    if(best_fly_next_Point[j][i]==true)
                    {
                        outputfile<<" "<<X[j][i][k];
                    }
                    else{
                        cost+=w[X[j][i][k]][i];
                        a.push_back(w[X[j][i][k]][i]);
                        outputfile<<" "<<X[j][i][k];           
                    }
                }
                else{
                    cost+=w[X[j][i][k]][i];
                    a.push_back(w[X[j][i][k]][i]);
                    outputfile<<" "<<X[j][i][k];           
                }
            }
            rep(k,em) outputfile<<" ";
            outputfile<<"cost : ";
            rep(k,a.size()) {
                if(k==a.size()-1) outputfile<<a[k]<<"=";
                else outputfile<<a[k]<<"+";
            }
            outputfile<<cost<<"             drone late cost"<<best_late_drone[j][i]<<endl;
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
    cal_dist_truck_StopPoint();
    decide_todeliver_near();
    best_score=cal_score_dronenextPoint(solution_where_todeliver);
    cout<<best_score<<endl;
    start=clock();
    while((double)(finish-start)/CLOCKS_PER_SEC<limit_time){
        int ss=rand()%100;
        if(ss<60) swap_search(solution_where_todeliver);
        else insert_search(solution_where_todeliver);
        finish=clock();
    }
    time_finish=clock();
    decide_drone_todeliver_by_LPT(solution_where_todeliver);
    cout<<"score : "<<best_score<<endl;
    //cout<<"time : "<<(double)(time_finish-time_start)/CLOCKS_PER_SEC<<endl;
    output_txt();
    output_customer_place();
    output_answer(X);
}