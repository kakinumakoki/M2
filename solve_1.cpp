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


struct Point{
    int x,y;
};


int N;//顧客数
int Q;//停止ポイント数 0は始点デポ Qは終点デポ
vector<Point>C;
vector<Point>Stop_Point;

int swap_Stop_Point_dist(int a,int b){
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

void input(){
    ifstream input_file("instance_1.txt");
    input_file>>N;
    C.resize(N);
    rep(i,N){
        char a;
        input_file>>C[i].x>>a>>C[i].y;
    }
    input_file>>Q;
    Stop_Point.resize(Q);
    rep(i,Q){
        char a;
        input_file>>Stop_Point[i].x>>a>>Stop_Point[i].y;
    }
    input_file.close();
}

void truck_root_decide()//truckのルート決定
{
    vector<vector<int>>dist(Q,vector<int>(Q));
    rep(i,Q){
        rep(j,Q){
            if(i==j) dist[i][j]=max_x_y*max_x_y;
            else dist[i][j]=ceil(sqrt((Stop_Point[i].x-Stop_Point[j].x)*(Stop_Point[i].x-Stop_Point[j].x)+(Stop_Point[i].y-Stop_Point[j].y)*(Stop_Point[i].y-Stop_Point[j].y)));
        }
    }
    //----------------Stop_Pointを(0,0)からNN法-------------------
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
//---------------------------swap近傍--------------------------------------
    count=0;
    while(count<3){
        for(int i=1;i<Q-1;i++){
            for(int j=1;j<Q-1;j++){
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

void output_customer_place()
{
    ofstream outputfile("customer_place_1.txt");
    rep(i,N){
        outputfile<<C[i].x<<" "<<C[i].y<<endl;
    }
    outputfile.close();
}

int main()
{
    //----------入力-----------------------------
    input();
    output_customer_place();
    truck_root_decide();
    //----------顧客と停止ポイント間の往復時間の計算-------------------------
    vector<vector<int>> w(N,vector<int>(Q));//顧客と停止ポイント間の往復時間
    for(int i=0;i<N;i++){
        for(int j=0;j<Q;j++){
            w[i][j]=ceil(sqrt((C[i].x-Stop_Point[j].x)*(C[i].x-Stop_Point[j].x)+(C[i].y-Stop_Point[j].y)*(C[i].y-Stop_Point[j].y)));
        }
    }

}