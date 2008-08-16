MA:matrix([a00, a01, a02, a03],
	  [a10, a11, a12, a13],
	  [a20, a21, a22, a23],
	  [a30, a31, a32, a33]);
MB:matrix([b00, b01, b02, b03],
	  [b10, b11, b12, b13],
	  [b20, b21, b22, b23],
	  [b30, b31, b32, b33]);
MT:matrix([t00, t01, t02, t03],
	  [t10, t11, t12, t13],
	  [t20, t21, t22, t23],
	  [t30, t31, t32, t33]);
MTdx:matrix([t00, t01, t02, t03+d],
	    [t10, t11, t12, t13],
	    [t20, t21, t22, t23],
	    [t30, t31, t32, t33]);
MV:matrix([x],[y],[z],[1]);
T:MA.MT.MB;
Tdx:MA.MTdx.MB;
XYZW0:T.MV;
XYZW1:Tdx.MV;
expand(XYZW0[4])-expand(XYZW1[4]);


