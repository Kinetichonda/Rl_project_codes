import numpy as np
import scipy
import math
import sys
sys.path.append('..')
eta=0.1
thresh=10**-14
#function to compute gradient of required function
def gradient(u,v):
	grad_u=2*(math.exp(v) + 2*v*math.exp(-u))*(u*math.exp(v)-2*v*math.exp(-u))
	grad_v=2*(u*math.exp(v) - 2*math.exp(-u))*(u*math.exp(v)-2*v*math.exp(-u))
	return [grad_u, grad_v]
def compute_fn(u,v):
	val=(u*math.exp(v)-2*v*math.exp(-u))**2
	return val
def gradient_desc(u,v):
	[grad_u,grad_v]=gradient(u,v)
	u=u-eta*grad_u
	v=v-eta*grad_v
	return [u,v]
u=1
v=1
count=0
val=compute_fn(u,v)
while(val>10**-14):
	[u_it,v_it]=gradient_desc(u,v)
	val=compute_fn(u_it,v_it)
	count=count+1
	[u,v]=[u_it,v_it]
print(count)

