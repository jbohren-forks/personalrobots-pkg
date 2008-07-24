from numpy import *
from scipy import *
from numpy.linalg.linalg import *
from numpy.linalg import *

a0=array([ 1., 0.,  1.])
a1=array([ 0., 1., -1.])
# a2=array([-1.,-1.,  0.])
a2=array([-10.,-1.,  -1.])
a3=array([0.0, 0.0, 0.0])

At = matrix([a0, a1, a2, a3])

a0=array([ 1., 0.,  1.])
a1=array([ 0., 1.,  1.])
a2=array([ 0., 0.,  2.])
a3=array([0.0, 0.0, 1.])

At = matrix([a0, a1, a2])
#At = matrix([a0, a1, a2, a3])

print 'At='
print At
A=At.getT()
# ac = (A[:,0]+A[:,1]+A[:,2])/3.
ac = A.mean(1)
print 'centroid of A: ', ac.getT()
Ac = A-ac
print 'Ac = A centered ='
print Ac

#sigma A
Avarv = var(A, 1)
Avar = Avarv.sum()
print 'variance of A: ', Avar
print Avarv
print std(A, 1)

# rotation matrix
theta=pi/4
Rz=matrix([[ cos(theta), -sin(theta), 0],
           [ sin(theta),  cos(theta), 0],
           [          0,           0, 1]])
Ry=matrix([[ cos(theta), 0, -sin(theta)],
           [          0, 1,           0],
           [ sin(theta), 0,  cos(theta)]])
Rx=matrix([[ 1, 0, 0],
           [0, cos(theta), -sin(theta)],
           [0, sin(theta),  cos(theta)]])

R=Rx*Ry*Rz
print 'original rotation matrix='
print R
C=R * A
T=mat([[100], [0], [0]])
B=C+T
print 'B='
print B
# center of points in B
# note that the points in B is in columns
#bc = (B[:,0]+B[:,1]+B[:,2])/3.
bc = B.mean(1)
print 'centroid of B: ', bc.getT()
Bc = B-bc
print 'Bc = B centered: '
print Bc

#sigma B square
Bvarv = var(B, 1)
Bvar  = Bvarv.sum()
print 'variance of B=: ', Bvar
#print Bvarv
#print std(B, 1)

cc = C.mean(1)
print 'centroid of C: ', cc.getT()
Cc = C-cc
print 'Cc = C centered: '
print Cc

# sigma C square
Cvarv = var(C, 1)
Cvar  = Cvarv.sum()
print 'variance of C=: ', Cvar

Act=Ac.getT()
print 'Ac^t='
print Act

Q=Bc*Act
print 'Q='
print Q
print det(Q)
print 'rank(Q): ', rank(Q)

U, S, Vt = svd(Q)
print 'U='
print U
#print U*U.getT()
print det(U)
print 'S=',S
print 'Vt='
print Vt
#print Vt*Vt.getT()
print det(Vt)
print 'reconstructed Q='
Q1 = U*diag(S)*Vt
print Q1
print det(Q1)

if det(U)*det(Vt)<0:
    S1=matrix(diag([1,1,-1]))
else:
    S1=matrix(diag([1,1,1]))
R1=U*S1*Vt
#R2=Vt.getT()*U.getT()
print 'reconstructed R='
print R1
print 'norm(R-R1)=', norm(R-R1)
print 'det(R1)=', det(R1), det(U*Vt)

Ac2=matrix([[Ac[0,0], Ac[0,1]],
            [Ac[1,0], Ac[1,1]],
            [Ac[2,0], Ac[2,1]]]);
Bc2=matrix([[Bc[0,0], Bc[0,1]],
            [Bc[1,0], Bc[1,1]],
            [Bc[2,0], Bc[2,1]]]);

Q=Bc2*Ac2.getT();
print 'Q='
print Q
print det(Q)
print 'rank(Q): ', rank(Q)

U, S, Vt = svd(Q)
print 'U='
print U
#print U*U.getT()
print det(U)
print 'S=',S
print 'Vt='
print Vt
#print Vt*Vt.getT()
print det(Vt)
print 'reconstructed Q='
Q1 = U*diag(S)*Vt
print Q1
print det(Q1)

if det(U)*det(Vt)<0:
    S1=matrix(diag([1,1,-1]))
else:
    S1=matrix(diag([1,1,1]))
R1=U*S1*Vt
#R2=Vt.getT()*U.getT()
print 'reconstructed R='
print R1
print 'norm(R-R1)=', norm(R-R1)
print 'det(R1)=', det(R1), det(U*Vt)

Ac1=matrix([[Ac[0,0]],
            [Ac[1,0]],
            [Ac[2,0]]]);
Bc1=matrix([[Bc[0,0]],
            [Bc[1,0]],
            [Bc[2,0]]]);

Q=Bc1*Ac1.getT();
print 'Q='
print Q
print det(Q)
print 'rank(Q): ', rank(Q)

U, S, Vt = svd(Q)
print 'U='
print U
#print U*U.getT()
print det(U)
print 'S=',S
print 'Vt='
print Vt
#print Vt*Vt.getT()
print det(Vt)

print 'reconstructed Q='
Q1 = U*diag(S)*Vt
print Q1
print det(Q1)

if det(U)*det(Vt)<0:
    S1=matrix(diag([1,1,-1]))
else:
    S1=matrix(diag([1,1,1]))
R1=U*S1*Vt
#R2=Vt.getT()*U.getT()
print 'reconstructed R='
print R1
print 'norm(R-R1)=', norm(R-R1)
print 'det(R1)=', det(R1), det(U*Vt)
trS = trace(diag(S)*S1)
c=trS/Avar
print 'c=', c
c=1.0
print 'c=', c

Bc1=R*Ac
print 'reconstructed Bc='
print Bc1
print 'norm(Bc-Bc1) =', norm(Bc-Bc1)

T1 = bc - c*R1*ac
print 'reconstructed translation: '
print T1
print 'norm(T-T1)=', norm(T-T1)

print Bc1
B1=R1*A+T1
print 'reconstructed B='
print B1
print 'norm(B-B1)=', norm(B-B1)

minError = Bvar - trS*trS/Avar
print 'min error: ',
print minError

QC = Cc*Act
print 'QC='
print QC
print det(QC)

UC, SC, VCt = svd(QC)
print 'UC='
print UC
print det(UC)
print 'SC=',SC
print 'VCt='
print VCt
print det(VCt)

print 'reconstructed QC='
QC1 = UC*diag(SC)*VCt
print QC1
print det(QC1)

if det(VCt)*det(UC)<0:
    S1=matrix(diag([1,1,-1]))
else:
    S1=matrix(diag([1,1,1]))
RC1=UC*S1*VCt
print 'reconstructed RC='
print RC1
print 'norm(R-RC1)=', norm(R-RC1)
trSC = trace(diag(SC)*S1)
cC=trSC/Avar
print 'cC=', cC
cC=1.0
print 'cC=', cC

Cc1=RC1*Ac
print 'reconstructed Cc='
print Cc1
print 'norm(Cc-Cc1) =', norm(Cc-Cc1)

TC1 = cc - cC*RC1*ac
print 'reconstructed translation: '
print TC1
print 'norm(T-T1)=', norm(T-TC1)

print Cc1
C1=RC1*A+TC1
print 'reconstructed C='
print C1
print 'norm(C-C1)=', norm(C-C1)


