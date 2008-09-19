import *
from scipy import *
t2=arange(1,100,0.1)
N = int(t2/2)
y2=cos(t)
fy2=fft(y2)
fy2=fy2[:N]
