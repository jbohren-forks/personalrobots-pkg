#euler angles
cz:cos(z);
sz:sin(z);
cy:cos(y);
sy:sin(y);
cx:cos(x);
sx:sin(x);

Qx:matrix([cz, -sz, 0],
	  [sz,  cz, 0],
	  [ 0,   0, 1]);
Qy:matrix([1,   0,  0],
	  [0,  cy,-sy],
	  [0,  sy, cy]);
Qz:matrix([cx, -sx, 0],
	  [sx,  cx, 0],
	  [ 0,   0, 1]);

# euler angle given as z-y-x, for right multiplication
Qzyx:Qx.Qy.Qz;

# eulter angle given as x-y-z, for left multiplication
Qxyz:Qz.Qy.Qx

diff(MR,a);
diff(MR,b);
diff(MR,c);
