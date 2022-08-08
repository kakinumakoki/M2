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
#define N 50
#define Q 10
#define max_x_y 50

struct Point
{
    int x,y;
};

int main()
{
    srand((unsigned int)time(NULL));
    Point C[N],K[Q+2];//C:�ڋq���W  K:�g���b�N��~���W(K[0]�͎n�_�f�|,K[Q+1]�͏I�_�f�|)
    bool check[50][50]={0};
    int count=0;
    while(count<N){
        int x,y;
        x=rand()%max_x_y;
        y=rand()%max_x_y;
        C[count].x=x;
        C[count].y=y;
        if(check[x][y]==false){
            check[x][y]=true;
            count++;
        }
    }
    count=1;
    K[0]={0,0};
    check[0][0]=true;
    K[Q+1]={max_x_y,max_x_y};
    while(count<Q+1){
        int x,y;
        x=rand()%max_x_y;
        y=rand()%max_x_y;
        K[count].x=x;
        K[count].y=y;
        if(check[x][y]==false){
            check[x][y]=true;
            count++;
        }
    }
    cout<<"�ڋq���W"<<endl;
    rep(i,N){
       printf("%3d : %3d\n",C[i].x,C[i].y);
    }
    cout<<"��~�|�C���g���W"<<endl;
    rep(i,Q+2){
       printf("%3d : %3d\n",K[i].x,K[i].y);       
    }
    //--�o��-----------------------
    ofstream outputfile("instance_2.txt");
    rep(i,N){
        outputfile<<C[i].x<<","<<C[i].y<<endl;
    }
    rep(i,Q+2){
        outputfile<<K[i].x<<","<<K[i].y<<endl;
    }
    outputfile.close();
}