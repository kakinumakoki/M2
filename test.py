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


pyplot.scatter(customer_x,customer_y)
pyplot.plot(truck_x,truck_y,color="red")
pyplot.scatter(truck_x,truck_y,color="red")
pyplot.show()