import sys
from matplotlib import pyplot

truck_x=[]
truck_y=[]
customer_x=[]
customer_y=[]

mat_1=[]
with open('truck_root_1.txt', 'r') as fin:  
    for line in fin.readlines():  
        row = []  
        toks = line.split(' ')  
        for tok in toks:
            num = int(tok)
            row.append(num)
        mat_1.append(row)

for i,a in enumerate(mat_1):
    for k,v in enumerate(a):
        if k==0:
            truck_x.append(v)
        else:
            truck_y.append(v)

mat_2=[]
with open('customer_place_1.txt', 'r') as fin:  
    for line in fin.readlines():  
        row = []  
        toks = line.split(' ')  
        for tok in toks:
            num = int(tok)
            row.append(num)
        mat_2.append(row)

for i,a in enumerate(mat_2):
    for k,v in enumerate(a):
        if k==0:
            customer_x.append(v)
        else:
            customer_y.append(v)

mat3=[]
with open('answer_3.txt') as fin:
    for line in fin.readlines():
        row = []
        toks = line.split(' ')
        for tok in toks:
            num = int(tok)
            row.append(num)
        mat3.append(row)    

pyplot.scatter(customer_x,customer_y)
pyplot.plot(truck_x,truck_y,color="black",linewidth=2)
for i,a in enumerate(truck_x):
    if i==0 or i==len(truck_x)-1:
        pyplot.scatter(truck_x[i],truck_y[i],marker='s',color="black")
    else:
        pyplot.scatter(truck_x[i],truck_y[i],color="red",linewidth=3)
ii=0
jj=1
for i,a in enumerate(mat3):
    for k,v in enumerate(a):
        if v==-1:
            ii=ii+1
        else:
            tx=truck_x[jj]
            ty=truck_y[jj]
            cx=customer_x[v]
            cy=customer_y[v]
            if ii==0:
                pyplot.plot([tx,cx],[ty,cy],color="green")
            if ii==1:
                pyplot.plot([tx,cx],[ty,cy],color="yellow")
            if ii==2:
                pyplot.plot([tx,cx],[ty,cy],color="cyan")
            if ii==3:
                pyplot.plot([tx,cx],[ty,cy],color="red")

        if ii==4:
            ii=0
            jj=jj+1

pyplot.plot([],[],label="drone1",color="green")
pyplot.plot([],[],label="drone2",color="yellow")
pyplot.plot([],[],label="drone3",color="cyan")
pyplot.plot([],[],label="drone4",color="red")

pyplot.plot([],[],label="truck root",color="black")
pyplot.scatter([],[],marker="o",color="red",label="Stop Point")
pyplot.scatter([],[],marker="o",color="blue",label="Customer")
pyplot.scatter([],[],marker="s",color="black",label="Depo")

pyplot.legend(loc='upper left',bbox_to_anchor=(1,1))

pyplot.show()