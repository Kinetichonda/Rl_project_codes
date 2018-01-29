import numpy as np
import random
import math
#dataset creation
[a1,b1]=[np.random.uniform(-1,1),np.random.uniform(-1,1)]
[a2,b2]=[np.random.uniform(-1,1),np.random.uniform(-1,1)]
from numpy import ones,vstack
from numpy.linalg import lstsq
points = [(a1,b1),(a2,b2)]
x_coords, y_coords = zip(*points)
A = vstack([x_coords,ones(len(x_coords))]).T
m, c = lstsq(A, y_coords)[0]
print("Line Solution is y = {m}x + {c}".format(m=m,c=c))
[x,y]=[np.random.uniform(-1,1,100),np.random.uniform(-1,1,100)]
Output=[]
for [a,b] in zip(x,y):
	if (m*a-b<0):
		Output.append(-1)
	else:
		Output.append(1)
Input=zip(np.ones(100),x,y)
def cross_entropy(feature, res,weights):
	return math.log(1 + math.exp(-res * np.dot(feature,weights)))
weights=[0,0,0]
def gradient(coordinate,res,weights):
        U = -np.multiply(res, coordinate)
        D= 1 + (math.exp(res*np.dot(weights, coordinate)))
        return np.divide(U,D)
eta=0.01
distance=1
count=0
weight_old=[4,2,4]
distance=100
while distance>0.01:
	order=range(100)
	random.shuffle(order)
	for i in (order):
		feature=Input[i]
		res=Output[i]
		grad=gradient(feature,res,weights)
		weights=weights-eta*grad
	distance= np.linalg.norm(weight_old-weights)
	weight_old=weights
	count=count+1

print(count)

#Testing to estimate E_out
E_out=0
[x_test,y_test]=[np.random.uniform(-1,1,100),np.random.uniform(-1,1,100)]
for [a,b] in zip(x_test,y_test):
	if (m*a-b<0):
		Res=-1
	else:
		Res=1
	E_out=E_out+cross_entropy([1,a,b],Res,weights)
print(E_out)






