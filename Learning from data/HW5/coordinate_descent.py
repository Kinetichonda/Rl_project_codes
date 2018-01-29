import math

def gradient(u,v):
	grad_u=2*(math.exp(v) + 2*v*math.exp(-u))*(u*math.exp(v)-2*v*math.exp(-u))
	grad_v=2*(u*math.exp(v) - 2*math.exp(-u))*(u*math.exp(v)-2*v*math.exp(-u))
	return [grad_u, grad_v]
def u_desc(u,v):
	[grad_u,grad_v]=gradient(u,v)
	u=u-eta*grad_u
	return u

def v_desc(u,v):
	[grad_u,grad_v]=gradient(u,v)
	v=v-eta*grad_v
	return v
u=1
v=1
eta=0.1
for i in range(15):
	u_it=u_desc(u,v)
	u=u_it
	v_it=v_desc(u,v)
	v=v_it

def compute_fn(u,v):
	val=(u*math.exp(v)-2*v*math.exp(-u))**2
	return val
print(compute_fn(u,v))


