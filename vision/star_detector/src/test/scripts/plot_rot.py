#!/usr/bin/python
from pylab import *
from matplotlib.ticker import MultipleLocator, FormatStrFormatter

def loadRepeatabilityCurve(infix, angles, overlap=40):
    files = ["rotated/rot%d.%s.rep" % (a, infix) for a in angles]
    repeatability = zeros(len(angles))
    index = overlap/5 - 1
    for i in range(len(angles)):
        X = load(files[i])
        repeatability[i] = X[index, 2]
    return repeatability

angles = range(15, 180+15, 15)

lenient_ot = 15
wg_lenient = loadRepeatabilityCurve("willow", angles, lenient_ot)
wgmod_lenient = loadRepeatabilityCurve("willow_mod", angles, lenient_ot)
wgmod2_lenient = loadRepeatabilityCurve("willow_mod2", angles, lenient_ot)
#wg9_lenient = loadRepeatabilityCurve("willow9", angles, lenient_ot)
sift_lenient = loadRepeatabilityCurve("sift", angles, lenient_ot)
surf_lenient = loadRepeatabilityCurve("surf", angles, lenient_ot)

strict_ot = 5
wg_strict = loadRepeatabilityCurve("willow", angles, strict_ot)
wgmod_strict = loadRepeatabilityCurve("willow_mod", angles, strict_ot)
wgmod2_strict = loadRepeatabilityCurve("willow_mod2", angles, strict_ot)
#wg9_strict = loadRepeatabilityCurve("willow9", angles, strict_ot)
sift_strict = loadRepeatabilityCurve("sift", angles, strict_ot)
surf_strict = loadRepeatabilityCurve("surf", angles, strict_ot)

majorLocator = MultipleLocator(15)
majorFormatter = FormatStrFormatter('%d')

ax = subplot(211)
#ax = subplot(111)
plot(angles, wg_strict, 'bo-')
plot(angles, wgmod_strict, 'co-')
plot(angles, wgmod2_strict, 'mo-')
#plot(angles, wg9_strict, 'co-')
plot(angles, sift_strict, 'gs-')
plot(angles, surf_strict, 'rx-')
ylabel('repeatability %')
title("Overlap threshold %d%%" % strict_ot)
grid(True)
ax.xaxis.set_major_locator(majorLocator)
ax.xaxis.set_major_formatter(majorFormatter)
legend(('Star', 'StarTest', 'StarTest2', 'SIFT', 'SURF'))

ax = subplot(212)
plot(angles, wg_lenient, 'bo-')
plot(angles, wgmod_lenient, 'co-')
plot(angles, wgmod2_lenient, 'mo-')
#plot(angles, wg9_lenient, 'co-')
plot(angles, sift_lenient, 'gs-')
plot(angles, surf_lenient, 'rx-')
xlabel('rotation angle (degrees)')
ylabel('repeatability %')
title("Overlap threshold %d%%" % lenient_ot)
grid(True)
ax.xaxis.set_major_locator(majorLocator)
ax.xaxis.set_major_formatter(majorFormatter)

show()
