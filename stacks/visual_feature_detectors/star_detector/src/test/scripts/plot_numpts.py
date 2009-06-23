#!/usr/bin/python
from pylab import *

def loadRepeatabilityCurve(infix, keypt_counts, overlap=40):
    files = ["data/img1-2.%s-%dp.rep" % (infix, p) for p in keypt_counts]
    repeatability = zeros(len(keypt_counts))
    index = overlap/5 - 1
    for i in range(len(keypt_counts)):
        X = load(files[i])
        repeatability[i] = X[index, 2]
    return repeatability

keypt_counts = range(10, 50, 10)
keypt_counts.extend(range(50, 550, 50))
keypt_counts.extend(range(600, 900, 100))

wg40 = loadRepeatabilityCurve("willow", keypt_counts, 40)
#wg9_40 = loadRepeatabilityCurve("willow9", keypt_counts, 40)
wgmod40 = loadRepeatabilityCurve("willow_mod", keypt_counts, 40)
wgmod2_40 = loadRepeatabilityCurve("willow_mod2", keypt_counts, 40)
sift40 = loadRepeatabilityCurve("sift", keypt_counts, 40)
surf40 = loadRepeatabilityCurve("surf", keypt_counts, 40)

wg20 = loadRepeatabilityCurve("willow", keypt_counts, 20)
#wg9_20 = loadRepeatabilityCurve("willow9", keypt_counts, 20)
wgmod20 = loadRepeatabilityCurve("willow_mod", keypt_counts, 20)
wgmod2_20 = loadRepeatabilityCurve("willow_mod2", keypt_counts, 20)
sift20 = loadRepeatabilityCurve("sift", keypt_counts, 20)
surf20 = loadRepeatabilityCurve("surf", keypt_counts, 20)

subplot(211)
plot(keypt_counts, wg40, 'bo-')
plot(keypt_counts, wgmod40, 'co-')
plot(keypt_counts, wgmod2_40, 'mo-')
#plot(keypt_counts, wg9_40, 'co-')
plot(keypt_counts, sift40, 'gs-')
plot(keypt_counts, surf40, 'rx-')
ylabel('repeatability %')
title('Overlap threshold 40%')
grid(True)

subplot(212)
plot(keypt_counts, wg20, 'bo-')
plot(keypt_counts, wgmod20, 'co-')
plot(keypt_counts, wgmod2_20, 'mo-')
#plot(keypt_counts, wg9_20, 'co-')
plot(keypt_counts, sift20, 'gs-')
plot(keypt_counts, surf20, 'rx-')
xlabel('# keypoints')
ylabel('repeatability %')
title('Overlap threshold 20%')
grid(True)
legend(('Star', 'StarTest', 'StarTest2', 'SIFT', 'SURF'))

show()
