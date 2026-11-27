"""Small pure arithmetic checks; no simulator, prediction reconstruction or outcomes."""
import numpy as np
import response_metrics as m

def check():
    # Same worldangular velocity in different endpoint frames must subtract tozero.
    q0=np.r_[np.zeros(3),[1.,0.,0.,0.],np.zeros(3)];q1=q0.copy();q1[3:7]=[np.sqrt(.5),0,0,np.sqrt(.5)]
    a=np.zeros(9);b=a.copy();a[3:6]=[1,0,0];b[3:6]=[0,-1,0]
    np.testing.assert_allclose(m.physical_difference(q0,a,q1,b,np.eye(3)),0,atol=1e-15)
    # Initial axes differ from world; transverse response and worldyaw are distinct.
    R=np.array([[0.,0,1],[0,1,0],[-1,0,0]]);G=np.zeros((9,5));G[3,0]=2
    p=m.prediction(G,np.array([1,0,0,0,0]),R);assert p[3]==2 and p[9]==-2
    assert m.score([0],[0],.01)['status']=='unresolved'
    assert m.score([0],[.02],.01,is_transverse=True)['transverse_nearzero_absolute_pass'] is False
    assert m.score([0],[.005],.01,is_transverse=True)['transverse_nearzero_absolute_pass'] is True
    assert m.score([.1],[-.1],.01)['passes_screen'] is False
    assert m.score([.1],[.1],.01)['passes_screen'] is True
    a=np.arange(10)*.1;lin=m.linearity(a,a/2);assert all(x['passes_screen'] in (True,None) for x in lin.values())
    print('Eight pure arithmetic checks passed; zero physics; not repository test-suite count.')

if __name__=='__main__':check()
