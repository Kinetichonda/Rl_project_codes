import numpy as np
import pandas as pd
#Training
train=pd.read_table('in.dta',delim_whitespace=True,header=None)
train.columns=['x1','x2','res']
train_out=train['res']
train=train.drop('res',axis=1)
train['x1^2']=train['x1']**2
train['x2^2']=train['x2']**2
train['x1x2']=train['x1']*train['x2']
train['|x1-x2|']=train['x1']-train['x2']
train['|x1-x2|']=train['|x1-x2|'].abs()
train['|x1+x2|']=train['x1']+train['x2']
train['|x1+x2|']=train['|x1+x2|'].abs()
train['const']=pd.Series(np.ones(35))
train=np.array(train)
for i in range(7,0,-1):
	train[:,i]=train[:,i-1]
train[:,0]=np.ones(35)
print(train)
Y=np.array(train_out)
training=train[:25][:]
y_train=Y[:25]
y_val=Y[25:]
min=100
val=train[25:][:]
print(val.shape)
for k in range(4,9):
	xTx=np.matmul((np.transpose(training[:,:k])),training[:,:k])
	Z=np.matmul(np.linalg.pinv(xTx),np.transpose(training[:,:k]))
	W=np.matmul(Z,y_train)
	S=np.sign(np.matmul(val[:,:k],W))
	E=float(float(len(y_val[S!=y_val]))/float(len(y_val)))
	print(float(len(y_val[S!=y_val])))
	if(E<=min):
		min=E
		index=k-1
print(index)
print(min)

