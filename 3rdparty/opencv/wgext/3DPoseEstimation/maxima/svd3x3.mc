Q:matrix([q00, q01, q02],
	 [q10, q11, q12],
         [q20, q21, q22]);
Qt:transpose(Q);
QQt:Q.Qt;
QtQ:Qt.Q;
I:diagmatrix(3,1);
C1:QQt-x*I;
charEq1:determinant(C1);
charEq1:expand(charEq1);
programmode: false;
breakup: true;
x1:solve(charEq1,x);
C2:QtQ-x*I;
charEq2:determinant(C2);
charEq2:expand(charEq2);
x2:solve(charEq2,x);
A:matrix([a00, a01, a02],
	 [a01, a11, a12],
	 [a02, a12, a22]);


               [                  2              2                      2     ]
               [ a00 a11 a22 - a01  a22 - a00 a12  + 2 a01 a02 a12 - a02  a11 ]