/* a form derived from Kurt's document: Small Visual System Calibration, p21 */
DispToCart:matrix([1, 0, 0, -Clx],
		  [0, Fx/Fy, 0, -Cy*Fx/Fy],
		  [0, 0, 0, Fx],
		  [0, 0, 1/Tx, -(Clx-Crx)/Tx]);
CartToDispR:matrix([Fx, 0, Crx, -Fx*Tx],
		   [0, Fy, Cy, 0],
		   [0, 0,  1,  0]);
CartToDisp:matrix([Fx, 0, Clx, 0],
		  [0, Fy, Cy, 0],
      	          [0, 0, Clx-Crx, Fx*Tx],
		  [0, 0, 1, 0]);
expand(DispToCart.CartToDisp);
/* a form that is more symmetrical and goes along with CartToDisp - multiple to I */
DispToCart2:DispToCart/Fx;
/* the following line shall produce an identity matrix of 4x4 */
expand(DispToCart2.CartToDisp);

