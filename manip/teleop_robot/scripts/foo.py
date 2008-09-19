import rostools
rostools.update_path('scipy')
from scipy import *
rostools.update_path('matplotlib')
from pylab import *

def fourier_coeffs(n, sigvect, T):#function statements must end in a colon
	dt=T/(len(sigvect)-1)
	t=arange(0,T+dt,dt)
	wo=2*pi/T
	cosfunc=cos(n*wo*t)*sigvect
	sinfunc=sin(n*wo*t)*sigvect
	A=integrate.trapz(cosfunc)*2/T*dt
	B=integrate.trapz(sinfunc)*2/T*dt
	return A, B, cosfunc, sinfunc



t2=arange(1,100,0.01)
t2=t2[:4096]
T=t2[-1]
print T
N = int(len(t2)/2)
y2=cos(2*3.14*t2)
fy2=fft(y2)
fy2=fy2[:N]

#A1,B1,cos1,sin1=fourier_coeffs(1,y2,T)
#A2,B2,cos2,sin2=fourier_coeffs(2,y2,T)

