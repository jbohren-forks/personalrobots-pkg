#!/usr/bin/python
from pylab import *

def loadRepeatabilityCurve(infix, scales, overlap=40):
    files = ["scaled/scaled%d.%d.%s.rep" % (s/10, s%10, infix) for s in scales]
    repeatability = zeros(len(scales))
    index = overlap/5 - 1
    for i in range(len(scales)):
        X = load(files[i])
        repeatability[i] = X[index, 2]
    return repeatability

scales = range(5, 21)
true_scales = [ s / 10.0 for s in scales]

lenient_ot = 40
wg_lenient = loadRepeatabilityCurve("willow", scales, lenient_ot)
wgmod_lenient = loadRepeatabilityCurve("willow_mod", scales, lenient_ot)
wgmod2_lenient = loadRepeatabilityCurve("willow_mod2", scales, lenient_ot)
#wg9_lenient = loadRepeatabilityCurve("willow9", scales, lenient_ot)
sift_lenient = loadRepeatabilityCurve("sift", scales, lenient_ot)
surf_lenient = loadRepeatabilityCurve("surf", scales, lenient_ot)
# TODO: this is a weird artifact
sift_lenient[5] = 100

strict_ot = 20
wg_strict = loadRepeatabilityCurve("willow", scales, strict_ot)
wgmod_strict = loadRepeatabilityCurve("willow_mod", scales, strict_ot)
wgmod2_strict = loadRepeatabilityCurve("willow_mod2", scales, strict_ot)
#wg9_strict = loadRepeatabilityCurve("willow9", scales, strict_ot)
sift_strict = loadRepeatabilityCurve("sift", scales, strict_ot)
surf_strict = loadRepeatabilityCurve("surf", scales, strict_ot)
sift_strict[5] = 100

ax = subplot(211)
#ax = subplot(111)
plot(true_scales, wg_lenient, 'bo-')
plot(true_scales, wgmod_lenient, 'co-')
plot(true_scales, wgmod2_lenient, 'mo-')
#plot(true_scales, wg9_lenient, 'co-')
plot(true_scales, sift_lenient, 'gs-')
plot(true_scales, surf_lenient, 'rx-')
ylabel('repeatability %')
title("Overlap threshold %d%%" % lenient_ot)
grid(True)
legend(('Star', 'StarTest', 'StarTest2', 'SIFT', 'SURF'))

ax = subplot(212)
plot(true_scales, wg_strict, 'bo-')
plot(true_scales, wgmod_strict, 'co-')
plot(true_scales, wgmod2_strict, 'mo-')
#plot(true_scales, wg9_strict, 'co-')
plot(true_scales, sift_strict, 'gs-')
plot(true_scales, surf_strict, 'rx-')
xlabel('scale factor')
ylabel('repeatability %')
title("Overlap threshold %d%%" % strict_ot)
grid(True)

show()
