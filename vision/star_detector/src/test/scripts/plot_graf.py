#!/usr/bin/python
from pylab import *

def loadRepeatabilityCurve(infix, overlap):
    files = ["data/img1-%d.%s.rep" % (i, infix) for i in range(2, 7)]
    repeatability = zeros(5)
    index = overlap/5 - 1
    for i in range(5):
        X = load(files[i])
        repeatability[i] = X[index, 2]
    return repeatability

angle = range(20, 70, 10)

wg20 = loadRepeatabilityCurve("willow-800p", 20)
#wg9_20 = loadRepeatabilityCurve("willow9-800p", 20)
wgmod20 = loadRepeatabilityCurve("willow_mod-800p", 20)
wgmod2_20 = loadRepeatabilityCurve("willow_mod2-800p", 20)
sift20 = loadRepeatabilityCurve("sift-800p", 20)
surf20 = loadRepeatabilityCurve("surf-800p", 20)
#surf_d20 = loadRepeatabilityCurve("surf_d-800p", 20)

wg40 = loadRepeatabilityCurve("willow-800p", 40)
#wg9_40 = loadRepeatabilityCurve("willow9-800p", 40)
wgmod40 = loadRepeatabilityCurve("willow_mod-800p", 40)
wgmod2_40 = loadRepeatabilityCurve("willow_mod2-800p", 40)
sift40 = loadRepeatabilityCurve("sift-800p", 40)
surf40 = loadRepeatabilityCurve("surf-800p", 40)
#surf_d40 = loadRepeatabilityCurve("surf_d-800p", 40)

subplot(211)
plot(angle, wg40, 'bo-')
plot(angle, wgmod40, 'co-')
plot(angle, wgmod2_40, 'mo-')
#plot(angle, wg9_40, 'co-')
plot(angle, sift40, 'gs-')
plot(angle, surf40, 'rx-')
#plot(angle, surf_d40, 'r<-')
ylabel('repeatability %')
title('Overlap threshold 40%')
grid(True)
legend(('Star', 'Star2', 'Star3', 'SIFT', 'SURF'))
#legend(('WG', 'WG-9', 'SIFT', 'SURF', 'SURF+'))

subplot(212)
plot(angle, wg20, 'bo-')
plot(angle, wgmod20, 'co-')
plot(angle, wgmod2_20, 'mo-')
#plot(angle, wg9_20, 'co-')
plot(angle, sift20, 'gs-')
plot(angle, surf20, 'rx-')
#plot(angle, surf_d20, 'r<-')
xlabel('viewpoint angle')
ylabel('repeatability %')
title('Overlap threshold 20%')
grid(True)

show()
