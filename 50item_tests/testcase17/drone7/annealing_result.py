import sys
from matplotlib import pyplot
x=[]
y=[]

mat=[]
with open('annealing_result_6.txt', 'r') as fin:  
    for line in fin.readlines():  
        row = []  
        toks = line.split(' ')  
        for tok in toks:
            num = float(tok)
            row.append(num)
        mat.append(row)

for i,a in enumerate(mat):
    for k,v in enumerate(a):
        if k==0:
            x.append(v)
        else:
            y.append(v)


pyplot.plot(x,y)

pyplot.xlabel("x")
pyplot.ylabel("y")
pyplot.grid(True)

pyplot.show()
