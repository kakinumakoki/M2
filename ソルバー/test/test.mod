/*********************************************
 * OPL 22.1.0.0 Model
 * Author: rard6
 * Creation Date: 2022/08/19 at 12:08:53
 *********************************************/
dvar int+ x;
dvar int+ y;

maximize 2*x+3*y;

subject to{
 0<=x;
 x<=4;
 0<=y;
 3<=y;
};