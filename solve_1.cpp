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


int N;//�ڋq��
int Q;//��~�|�C���g�� 0�͎n�_�f�| Q�͏I�_�f�|
vector<Point>C;
vector<Point>Stop_Point;


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

void truck_root_decide()
{
    vector<vector<int>>dist(Q,vector<int>(Q));
    rep(i,Q){
        rep(j,Q){
            if(i==j) dist[i][j]=max_x_y*max_x_y;
            else dist[i][j]=ceil(sqrt((Stop_Point[i].x-Stop_Point[j].x)*(Stop_Point[i].x-Stop_Point[j].x)+(Stop_Point[i].y-Stop_Point[j].y)*(Stop_Point[i].y-Stop_Point[j].y)));
        }
    }
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
    rep(i,Q){
        cout<<Stop_Point[root[i]].x<<" "<<Stop_Point[root[i]].y<<endl;
    }

    ofstream outputfile("truck_root_1.txt");
    rep(i,Q){
        outputfile<<Stop_Point[root[i]].x<<" "<<Stop_Point[root[i]].y<<endl;
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
    //----------����-----------------------------
    input();
    output_customer_place();
    truck_root_decide();
    //----------�ڋq�ƒ�~�|�C���g�Ԃ̉������Ԃ̌v�Z-------------------------
    vector<vector<int>> w(N,vector<int>(Q));//�ڋq�ƒ�~�|�C���g�Ԃ̉�������
    for(int i=0;i<N;i++){
        for(int j=0;j<Q;j++){
            w[i][j]=ceil(sqrt((C[i].x-Stop_Point[j].x)*(C[i].x-Stop_Point[j].x)+(C[i].y-Stop_Point[j].y)*(C[i].y-Stop_Point[j].y)));
        }
    }

}