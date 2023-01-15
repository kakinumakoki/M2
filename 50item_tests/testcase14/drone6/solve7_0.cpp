/*ドローンがトラックに戻らず次の停止ポイントに行くのもOK問題
やること
・2地点間の距離をユークリッド距離にする、往復はかける２する
・ドローンのスケジューリングの際一番飛行時間かかるものを各ドローンごとに最後に配達するようにする
・次の停止ポイントに移動した際にドローンが使えるか判定する変数追加
・pythonの表示方法考える
出発地点からもドローン飛ばせるようにしたもの
焼きなまし法
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
#define K 6//num of drone

double greedy_score;
double now_score;//今のスコア
double best_score;
double real_best_score;//全探索の中で最良スコア

struct Point{
    int x,y;
};

Point C[N];//顧客の座標
Point Stop_Point[Q];//停止ポイントの座標
double w[N][Q];//顧客ｎから停止ポイントｑの往復距離
double dist_by_truck[Q][Q];//出発デポ最終デポを含む停止ポイント間の距離
vector<vector<int>>greedy_solution_where_todeliver(Q);//現在の解のどの停止ポイントでどの顧客を届けるかの集合
vector<vector<int>>solution_where_todeliver(Q);//現在の解のどの停止ポイントでどの顧客を届けるかの集合
vector<vector<int>>best_solution_where_todeliver(Q);//全探索の中で最良解のどの停止ポイントでどの顧客を届けるかの集合
vector<vector<int>>real_best_solution_where_todeliver(Q);//全探索の中で最良解のどの停止ポイントでどの顧客を届けるかの集合
vector<vector<vector<int>>>X(K,vector<vector<int>>(Q));//出力用のどの停止ポイントでどのドローンがどの顧客を届けるかを決定した集合
long start,finish;//探索用のタイマー
long time_start,time_finish;//プログラム全体のタイマー
double T=1.5;//トラックの移動距離にかける係数(ドローンより移動速度遅いこと表現)
vector<vector<double>>best_late_drone(K,vector<double>(Q,0));//停止ポイントQでドローンKがどのくらい遅れてくるか
vector<vector<bool>>best_fly_next_Point(K,vector<bool>(Q,false));//停止ポイントQでドローンKを最後Q+１に飛ばすか
double startTemp,nowTemp;
long annealing_endTime=60.0,annealing_nowTime;
vector<pair<double,double>>processing_result;
double annealing_counter=0;
double annealing_show=(double)annealing_endTime/400;
int annealing_finish_counter;

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
        for(int j=0;j<Q-1;j++){
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

void decide_drone_todeliver_by_LPT_test(vector<vector<int>>A)
{
    cout<<"-------------------------result------------------------"<<endl;
    vector<vector<vector<int>>>x(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q
    double sum=0;
    vector<vector<double>>late_drone(K,vector<double>(Q,0));//停止ポイントQでドローンKがどのくらい遅れてくるか
    vector<vector<bool>>fly_next_Point(K,vector<bool>(Q,false));//停止ポイントQでドローンKを最後Q+１に飛ばすか
    for(int i=0;i<Q-1;i++){
//-------------------ドローンjが前の停止ポイントiから遅れているか確認
        double depart_truck_time=0;
        double time[K];
        double best_cost=10000;
        int which;
        //--------------------停止ポイントiで訪れるべき顧客番号と距離を格納
        for(int pattern=0;pattern<4;pattern++){
            vector<P>B;
            rep(j,A[i].size()){
                if(pattern==0) B.push_back({w[A[i][j]][i]+w[A[i][j]][i+1],A[i][j]});
                else if(pattern==1) B.push_back({w[A[i][j]][i],A[i][j]});    
                else if(pattern==2) B.push_back({w[A[i][j]][i+1],A[i][j]});
                else if(pattern==3) B.push_back({w[A[i][j]][i]+2*w[A[i][j]][i+1],A[i][j]});
            }
            if(pattern==0||pattern==1||pattern==3)sort(B.rbegin(),B.rend());//大きい順に
            else if(pattern==2) sort(B.begin(),B.end());//小さい順に
    //-------------------訪れるべき顧客がK人より少ない時トラックはすぐドローンを飛ばして出発する
    //-------------------もし前の停止ポイントから遅れているドローン入れば来るまで待つ
    //------------------訪れるべき顧客がK人以上なら
            int size=B.size();
            int sss=min(K,size);
            for(int num=0;num<=0;num++)
            {
                vector<bool>f_n_P(K,false);
                vector<double>l_d(K,0);
                vector<vector<vector<int>>>y(K,vector<vector<int>>(Q));//drone[K][Q]:Package delivered by drone K at point Q 
                double t[K],dp=0;
                rep(j,K) t[j]=late_drone[j][i]; 
                for(int j=num;j<B.size();j++){
    //------------------トップnum人を除いたものでLPT
    //------------------ここでスケジュールを組まれたものは荷物を届けた後トラックに戻ってくる
                    int min_processing_drone=-1;
                    double min_processing_time=1e9;
                    rep(k,K){
                        if(t[k]<min_processing_time){
                            min_processing_drone=k;
                            min_processing_time=t[k];
                        }
                    }
                    t[min_processing_drone]+=w[B[j].second][i];
                    y[min_processing_drone][i].push_back(B[j].second);
                }
                rep(j,K) dp=max(dp,t[j]);
    //----------------num人の顧客を届けたドローンはトラックに戻らず次の停止ポイントに移動
                for(int j=0;j<num;j++){
                    int min_processing_drone=0;
                    double min_processing_time=1e9;
                    rep(k,K){
                        if(f_n_P[k]==false&&t[k]<min_processing_time){
                            min_processing_drone=k;
                            min_processing_time=t[k];
                        }
                    }
                    y[min_processing_drone][i].push_back(B[j].second);
                    f_n_P[min_processing_drone]=true;
                    double dist=w[B[j].second][i]/2+w[B[j].second][i+1]/2;
                    l_d[min_processing_drone]=max(0.0,dist-dist_by_truck[i][i+1]-(dp-t[min_processing_drone]));
                }
    //-------------------K台のドローンの中でトラックに戻ってくるまでの時間が一番長いものが停止ポイントiでのコスト
                double max_score=0;
                rep(j,K) max_score=max(max_score,t[j]);
                //cout<<num<<":"<<max_score<<endl;
                if(best_cost>max_score){
                    best_cost=max_score;
                    which=pattern;
                    rep(j,K){
                        late_drone[j][i+1]=l_d[j];
                        time[j]=t[j];
                        fly_next_Point[j][i]=f_n_P[j];
                        x[j][i].clear();
                        rep(k,y[j][i].size()){
                            x[j][i].push_back(y[j][i][k]);
                        }
                    }
                }
            }
        }
        sum+=best_cost;
//--------------------------------出力------------------------------------------
        printf("Stop Point %d Score: %.2f\n",i,best_cost);
        cout<<"pattern "<<which<<endl;
        cout<<"-------------------------------------"<<endl;
        cout<<"item number&cost"<<endl;
        rep(j,K){
            cout<<"drone "<<j<<": ";
            rep(k,x[j][i].size()){
                cout<<x[j][i][k]<<" ";
                X[j][i].push_back(x[j][i][k]);
            }
            if(fly_next_Point[j][i]) {
               // cout<<"go next stop point   ";
                X[j][i+1].push_back(x[j][i][x[j][i].size()-1]);//次の停止ポイントに行くなら描画のため追加
            }
            X[j][i].push_back(-1);//-------------------finish = -1
            cout<<"   cost:";
            if(late_drone[j][i]>0) printf("late cost(%.2f) ",late_drone[j][i]);
            rep(k,x[j][i].size()){
                if(k==x[j][i].size()-1){
                    if(!fly_next_Point[j][i]) printf("%.2f ",w[x[j][i][k]][i]);
                }
                else printf("%.2f ",w[x[j][i][k]][i]);
            }
            printf(" = %.2f\n",time[j]);
        }
        cout<<"-------------------------------------"<<endl;
        cout<<"next drone late?"<<endl;
        cout<<"truck : "<<dist_by_truck[i][i+1]<<endl;
        rep(j,K){
            cout<<"drone "<<j<<": ";//<<dist_by_truck[i][i+1]-(best_cost-time[j]);
            rep(k,x[j][i].size()){
                if(k!=x[j][i].size()-1){
                    //cout<<w[x[j][i][k]][i]<<" ";
                    continue;
                }
                else{
                    if(fly_next_Point[j][i]) cout<<w[x[j][i][k]][i]/2+w[x[j][i][k]][i+1]/2<<"-"<<dist_by_truck[i][i+1]<<"-"<<(best_cost-time[j])<<"="<<w[x[j][i][k]][i]/2+w[x[j][i][k]][i+1]/2-dist_by_truck[i][i+1]-(best_cost-time[j])<<" ";
                    else cout<<w[x[j][i][k]][i];
                }
            }
            //cout<<"||"<<late_drone[j][i]<<endl;
            cout<<endl;
        }

        cout<<endl;
        if(i==Q-2){
            cout<<"last destination"<<endl;
            rep(j,K){
                cout<<"drone "<<j<<" ";
                X[j][Q-1].push_back(-1);
                if(late_drone[j][Q-1]>0) printf("late cost(%.2f) ",late_drone[j][Q-1]);
                else printf("Not late");
                cout<<endl;
            }
        }
        cout<<endl;
    }
    rep(i,K){
        rep(j,Q){
            best_fly_next_Point[i][j]=fly_next_Point[i][j];
            best_late_drone[i][j]=late_drone[i][j];
        }
    }
}

double cal_score_dronenextPoint_test(vector<vector<int>>A){
    double sum=0;
    vector<vector<double>>late_drone(K,vector<double>(Q,0));//停止ポイントQでドローンKがどのくらい遅れてくるか
    for(int i=0;i<Q-1;i++){
//-------------------ドローンjが前の停止ポイントiから遅れているか確認
        double depart_truck_time=0;
        double best_cost=100000;
        //--------------------停止ポイントiで訪れるべき顧客番号と距離を格納
        for(int pattern=0;pattern<4;pattern++){
            vector<P>B;
            rep(j,A[i].size()){
                if(pattern==0) B.push_back({w[A[i][j]][i]+w[A[i][j]][i+1],A[i][j]});
                else if(pattern==1) B.push_back({w[A[i][j]][i],A[i][j]});    
                else if(pattern==2) B.push_back({w[A[i][j]][i+1],A[i][j]});
                else if(pattern==3) B.push_back({w[A[i][j]][i]+2*w[A[i][j]][i+1],A[i][j]});
            }
            if(pattern==0||pattern==1||pattern==3)sort(B.rbegin(),B.rend());
            else if(pattern==2) sort(B.begin(),B.end());
    //-------------------訪れるべき顧客がK人より少ない時トラックはすぐドローンを飛ばして出発する
    //-------------------もし前の停止ポイントから遅れているドローン入れば来るまで待つ
    //------------------訪れるべき顧客がK人以上なら
            int size=B.size();
            int sss=min(K,size);
            for(int num=0;num<=0;num++)
            {
                vector<bool>f_n_P(K,false);
                vector<double>l_d(K,0);
                double t[K],dp=0;
                rep(j,K) t[j]=late_drone[j][i]; 
                for(int j=num;j<B.size();j++){
    //------------------現在の停止ポイントから遠いもののトップnum人を除いたものでLPT
    //------------------ここでスケジュールを組まれたものは荷物を届けた後トラックに戻ってくる
                    int min_processing_drone=-1;
                    double min_processing_time=1e9;
                    rep(k,K){
                        if(t[k]<min_processing_time){
                            min_processing_drone=k;
                            min_processing_time=t[k];
                        }
                    }
                    t[min_processing_drone]+=w[B[j].second][i];
                }
                rep(j,K) dp=max(dp,t[j]);
    //----------------遠いものトップnum人の顧客を届けたドローンはトラックに戻らず次の停止ポイントに移動
                for(int j=0;j<num;j++){
                    int min_processing_drone=0;
                    double min_processing_time=1e9;
                    rep(k,K){
                        if(f_n_P[k]==false&&t[k]<min_processing_time){
                            min_processing_drone=k;
                            min_processing_time=t[k];
                        }
                    }
                    f_n_P[min_processing_drone]=true;
                    double dist=w[B[j].second][i]/2+w[B[j].second][i+1]/2;
                    l_d[min_processing_drone]=max(0.0,dist-dist_by_truck[i][i+1]-(dp-t[min_processing_drone]));
                }
    //-------------------K台のドローンの中でトラックに戻ってくるまでの時間が一番長いものが停止ポイントiでのコスト
                double max_score=0;
                rep(j,K) max_score=max(max_score,t[j]);
                if(best_cost>max_score){
                    best_cost=max_score;
                    rep(j,K){
                        late_drone[j][i+1]=l_d[j];
                    }
                }
            }
        }
        sum+=best_cost;
    }
//---------------------最後の停止ポイントから最終目的地に移動するまでにトラックより遅いドローンいたらコストに加算
    double max_score=0;
    for(int i=0;i<K;i++) max_score=max(late_drone[i][Q-1],max_score);
    //cout<<"last:"<<max_score<<endl;
    sum+=max_score;
    return sum;
}

void annealing_insert_search(vector<vector<int>>A)
{
        int x;
        while(1){
            x=rand()%(Q-1);
            if(A[x].size()!=0) break;
        }
        int y=rand()%A[x].size();
        int z;
        while(1){
            z=rand()%(Q-1);
            if(z!=x) break;
        }
        vector<vector<int>>copy_answer(Q);
        for(int i=0;i<Q-1;i++){
            rep(j,A[i].size()){
                if(i==x&&j==y) copy_answer[z].push_back(A[i][j]);
                else copy_answer[i].push_back(A[i][j]);
            }
        }
        double copy_score=cal_score_dronenextPoint_test(copy_answer); 
        if(copy_score<now_score){
            annealing_finish_counter=0;
        }
        else annealing_finish_counter++;
        annealing_nowTime=clock();
        double probability=exp((now_score-copy_score)/nowTemp);
        if(copy_score<now_score||probability>(double)(rand()%RAND_MAX)/RAND_MAX){
            if((double)(annealing_nowTime-start)/CLOCKS_PER_SEC-annealing_counter>annealing_show){
                processing_result.push_back({(double)(annealing_nowTime-start)/CLOCKS_PER_SEC,copy_score});
                annealing_counter+=annealing_show;
                //cout<<copy_score<<endl;
            }
            rep(i,Q){
                solution_where_todeliver[i].clear();
                rep(j,copy_answer[i].size()){
                    solution_where_todeliver[i].push_back(copy_answer[i][j]);
                }
            }
        }
}

void annealing_swap_search(vector<vector<int>>A)
{
        vector<vector<int>>copy_answer(Q);
        for(int i=0;i<Q-1;i++){
            rep(j,A[i].size()){
                copy_answer[i].push_back(A[i][j]);
            }
        }
        int x,y;
        while(1){
            x=rand()%(Q-1);
            if(copy_answer[x].size()!=0){
                y=rand()%copy_answer[x].size();
                break;
            }
        }
        int z,w;
        while(1){
            z=rand()%(Q-1);
            if(copy_answer[z].size()!=0){
                w=rand()%copy_answer[z].size();
                break;
            }
        }       
        swap(copy_answer[x][y],copy_answer[z][w]);
        double copy_score=cal_score_dronenextPoint_test(copy_answer); 
        if(copy_score<now_score){
            annealing_finish_counter=0;
        }
        else annealing_finish_counter++;
        annealing_nowTime=clock();
        //cout<<copy_score<<endl;
        double probability=exp((now_score-copy_score)/nowTemp);
        if(copy_score<now_score||probability>(double)(rand()%RAND_MAX)/RAND_MAX){
            if((double)(annealing_nowTime-start)/CLOCKS_PER_SEC-annealing_counter>annealing_show){
                processing_result.push_back({(double)(annealing_nowTime-start)/CLOCKS_PER_SEC,copy_score});
                annealing_counter+=annealing_show;
                //cout<<copy_score<<endl;
            }
            now_score=copy_score;
            rep(i,Q){
                solution_where_todeliver[i].clear();
                rep(j,copy_answer[i].size()){
                    solution_where_todeliver[i].push_back(copy_answer[i][j]);
                }
            }
        }
}

double decide_tmp(vector<vector<int>>A,double Tmp){
    double p=0.07;
    int solve_counter=0,loop=100,update_num=0;
    double right=Tmp,left=0;
    Tmp=(right+left)/2;
    while(1){
        if(solve_counter==loop){
            if(abs(right-left)<0.01) {
               // cout<<"decide Tmp = "<<Tmp<<endl;
                return Tmp;
            }
            else if((double)update_num/loop-p<0){
                left=Tmp;
            }
            else {
                right=Tmp;
            }
            solve_counter=0;
            update_num=0;
            Tmp=(right+left)/2;
        }
        double select=rand()%100;
        if(select<40)//swap
        {
            vector<vector<int>>copy_answer(Q);
            for(int i=0;i<Q-1;i++){
                rep(j,A[i].size()){
                    copy_answer[i].push_back(A[i][j]);
                }
            }
            int x,y;
            while(1){
                x=rand()%(Q-1);
                if(copy_answer[x].size()!=0){
                    y=rand()%copy_answer[x].size();
                    break;
                }
            }
            int z,w;
            while(1){
                z=rand()%(Q-1);
                if(copy_answer[z].size()!=0){
                    w=rand()%copy_answer[z].size();
                    break;
                }
            }       
            swap(copy_answer[x][y],copy_answer[z][w]);
            double copy_score=cal_score_dronenextPoint_test(copy_answer); 
            double probability=exp((greedy_score-copy_score)/Tmp);
            if(copy_score<greedy_score||probability>(double)(rand()%RAND_MAX)/RAND_MAX) update_num++;
        }
        else//shift
        {
            int x;
            while(1){
                x=rand()%(Q-1);
                if(A[x].size()!=0) break;
            }
            int y=rand()%A[x].size();
            int z;
            while(1){
                z=rand()%(Q-1);
                if(z!=x) break;
            }
            vector<vector<int>>copy_answer(Q);
            for(int i=0;i<Q-1;i++){
                rep(j,A[i].size()){
                    if(i==x&&j==y) copy_answer[z].push_back(A[i][j]);
                    else copy_answer[i].push_back(A[i][j]);
                }
            }
            double copy_score=cal_score_dronenextPoint_test(copy_answer); 
            double probability=exp((greedy_score-copy_score)/Tmp);
            if(copy_score<greedy_score||probability>(double)(rand()%RAND_MAX)/RAND_MAX) update_num++;
        }
        solve_counter++;
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
void output_annealing_result()
{
    ofstream outputfile("annealing_result_6.txt");
    rep(i,processing_result.size()){
        outputfile<<processing_result[i].first<<" "<<processing_result[i].second<<endl;
    }
}
void output_answer(vector<vector<vector<int>>>x)
{
    ofstream outputfile("answer_7.txt");
    for(int i=0;i<Q;i++){
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
    ofstream outputfile("result_7.txt");
    for(int i=0;i<Q-1;i++){
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
                if(k==X[j][i].size()-2&&best_fly_next_Point[j][i]){
                    outputfile<<" "<<X[j][i][k];                   
                }
                /*else if(best_fly_next_Point[j][i-1]&&k==0){
                    continue;
                }*/
                else{
                    outputfile<<" "<<X[j][i][k];                   
                    a.push_back(w[X[j][i][k]][i]);
                }
            }
            rep(k,em) outputfile<<" ";
            em=30;
            outputfile<<"cost : ";
            rep(k,a.size()) {
                em-=8;
                cost+=a[k];
                if(k==a.size()-1) outputfile<<a[k]<<"=";
                else outputfile<<a[k]<<"+";
            }
            cost+=best_late_drone[j][i];
            outputfile<<cost;
            rep(k,em) outputfile<<" ";     
            outputfile<<"drone late cost : "<<best_late_drone[j][i];
            if(best_fly_next_Point[j][i]) outputfile<<"  次の停止ポイントに移動";
            outputfile<<endl;
        }
        outputfile<<endl;
    }
    outputfile<<"best score : "<<real_best_score<<endl;
    outputfile<<"time : "<<(double)(time_finish-time_start)/CLOCKS_PER_SEC;
}

void reset()
{
    now_score=greedy_score;
    rep(i,Q-1){
        solution_where_todeliver[i].clear();
        rep(j,greedy_solution_where_todeliver[i].size()){
            solution_where_todeliver[i].push_back(greedy_solution_where_todeliver[i][j]);
        }
    }
    if(best_score<real_best_score){
        real_best_score=best_score;
        rep(i,Q-1){
            real_best_solution_where_todeliver[i].clear();
            rep(j,best_solution_where_todeliver[i].size()){
                real_best_solution_where_todeliver[i].push_back(best_solution_where_todeliver[i][j]);
            }
        }
    }
    best_score=greedy_score;
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
    greedy_score=cal_score_dronenextPoint_test(solution_where_todeliver);
    rep(i,Q-1){
        rep(j,solution_where_todeliver[i].size()){
            greedy_solution_where_todeliver[i].push_back(solution_where_todeliver[i][j]);
        }
    }
    //cout<<greedy_score<<endl;
    now_score=greedy_score;
    best_score=greedy_score;
    real_best_score=greedy_score;
    for(int loop=0;loop<10;loop++){
        nowTemp=decide_tmp(greedy_solution_where_todeliver,greedy_score);
        start=clock();
        long timer=clock();
        annealing_finish_counter=0;
        while((double)(finish-start)/CLOCKS_PER_SEC<annealing_endTime){
            annealing_finish_counter++;
            int ss=rand()%100;
            if(ss<40) annealing_swap_search(solution_where_todeliver);
            else annealing_insert_search(solution_where_todeliver);
            if(now_score<best_score) {
                //cout<<now_score<<endl;
                best_score=now_score;
                rep(i,Q){
                    best_solution_where_todeliver[i].clear();
                    rep(j,solution_where_todeliver[i].size()){
                        best_solution_where_todeliver[i].push_back(solution_where_todeliver[i][j]);
                    }
                }
            }
            finish=clock();
            if((double)(finish-timer)/CLOCKS_PER_SEC>1){
                nowTemp=nowTemp*0.95;
                //cout<<"!!!!!!!!!!!!!!!!!"<<nowTemp<<"!!!!!!!!!!!!!!!!!!!"<<endl;
                timer=clock();
            }
            if(annealing_finish_counter>10000){
                //cout<<"reset ";
                annealing_finish_counter=0;
                nowTemp=decide_tmp(greedy_solution_where_todeliver,greedy_score);
                rep(i,Q){
                    solution_where_todeliver[i].clear();
                    rep(j,greedy_solution_where_todeliver[i].size()){
                        solution_where_todeliver[i].push_back(greedy_solution_where_todeliver[i][j]);
                    }
                }
                now_score=greedy_score;
            }
        }
        cout<<"loop "<<loop<<": best score "<<best_score<<endl;
        cout<<"----------------------------------------------"<<endl;
        reset();
    }
    time_finish=clock();
    decide_drone_todeliver_by_LPT_test(real_best_solution_where_todeliver);
    cout<<"score : "<<real_best_score<<endl;
    cout<<"time : "<<(double)(time_finish-time_start)/CLOCKS_PER_SEC<<endl;
    output_txt();
    output_customer_place();
    //output_annealing_result();
    output_answer(X);
}