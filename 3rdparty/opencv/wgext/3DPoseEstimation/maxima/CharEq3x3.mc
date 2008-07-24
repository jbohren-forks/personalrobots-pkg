A:matrix([a00, a01, a02],
	 [a01, a11, a12],
	 [a02, a12, a22]);
I:diagmatrix(3,1);
C:A-x*I;
charEq:determinant(C);
charEq:expand(charEq);
programmode: false;
breakup: true;
solve(charEq,x);


Q:matrix([q00, q01, q02],
	 [q10, q11, q12],
         [q20, q21, q22]);
Qt:transpose(Q);
QQt:Q.Qt;
QtQ:Qt.Q;