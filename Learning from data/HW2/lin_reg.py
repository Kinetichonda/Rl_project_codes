#linear regression
#Data generation
import numpy as np
E_in=0
for i in range(1000):
	N=1000
    [x,y]=[np.random.uniform(-1,1,N),np.random.uniform(-1,1,N)]
	Input=zip(np.ones(N),x,y)
	Output=np.sign(x**2+y**2-0.6)
	a=np.arange(1000)
	np.random.shuffle(a)
	b=a[:N/10]
	for t in b:
		Output[t]=(-Output[t])
    Input=np.array(Input)
    Output=np.array(Output)  
	(xTx)=np.matmul((np.transpose(Input)),Input)
	Z=np.matmul(np.linalg.pinv(xTx),np.transpose(Input))
	W=np.matmul(Z,Y)

	


