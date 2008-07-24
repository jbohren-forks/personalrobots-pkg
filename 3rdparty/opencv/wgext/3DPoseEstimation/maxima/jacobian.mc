load("eigen");
ca:cos(a);
sa:sin(a);
cb:cos(b);
sb:sin(b);
cg:cos(g);
sg:sin(g);

MA:matrix([ca, -sa, 0],
	  [sa,  ca, 0],
	  [ 0,   0, 1]);
MB:matrix([1,   0,  0],
	  [0,  cb,-sb],
	  [0,  sb, cb]);
MG:matrix([cg, -sg, 0],
	  [sg,  cg, 0],
	  [ 0,   0, 1]);
MR: MG.MB.MA;
MT: columnvector([x,y,z]);
MRT:addcol(MR, MT);
MRT:addrow(MRT, [0,0,0,1]);
Gamma:matrix([1,       0,         0,          -Clx],
	     [0,   Fx/Fy,         0,     -Cy*Fx/Fy],
	     [0,       0,         0,            Fx],
	     [0,       0,      1/Tx, -(Clx-Crx)/Tx]);
InvGamma:invert(Gamma);
G:matrix([1,   0, 0, g03],
	[0,  g11, 0, g13],
	[0,    0, 0, g23],
	[0,    0,g32,g33]);
IG:matrix([ig00, 0, ig02,0],
	[0,ig11,ig12,0],
	[0,   0,ig22,ig23],
	[0,   0,ig32,0]);
H:G.MRT.IG;
W0:columnvector([u0,v0,d0,1]);
W1:columnvector([u1,v1,d1,1]);
R:W1-H.W0;
r:R.R;
diff(r,a);
diff(r,b);
diff(r,c);
diff(r,x);
diff(r,y);
diff(r,z);
