#Linreg with regulariser
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
Y=np.array(train_out)
(xTx)=np.matmul((np.transpose(train)),train)
k=-3
#change k values for getting answer
lamda= 10 ** k
A=xTx+ 2*lamda*np.eye(xTx.shape[0])
B=np.matmul(np.transpose(train),Y)
W=np.matmul(np.linalg.pinv(A),B)
S=np.sign(np.matmul(train,W))
E_in=float(float(len(Y[S !=Y]))/float(len(Y)))
print(E_in)
#Testing
test=pd.read_table('out.dta',delim_whitespace=True,header=None)
test.columns=['x1','x2','res']
test_out=test['res']
test=test.drop('res',axis=1)
test['x1^2']=test['x1']**2
test['x2^2']=test['x2']**2
test['x1x2']=test['x1']*test['x2']
test['|x1-x2|']=test['x1']-test['x2']
test['|x1-x2|']=test['|x1-x2|'].abs()
test['|x1+x2|']=test['x1']+test['x2']
test['|x1+x2|']=test['|x1+x2|'].abs()
test['const']=pd.Series(np.ones(35))
test=np.array(test)
Y_test=np.array(test_out)
S_test=np.sign(np.matmul(test,W))
E_out=(float(len(Y_test[S_test!=Y_test]))/float(len(Y_test)))
print(E_out)