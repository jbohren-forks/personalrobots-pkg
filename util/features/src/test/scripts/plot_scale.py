#!/usr/bin/python
from pylab import *
from matplotlib.ticker import MultipleLocator, FormatStrFormatter

def loadRepeatabilityCurve(infix, scales, overlap=40):
    files = ["scale_data/img1-3.%s-%ds.rep" % (infix, s) for s in scales]
    repeatability = zeros(len(scales))
    index = overlap/5 - 1
    for i in range(len(scales)):
        X = load(files[i])
        repeatability[i] = X[index, 2]
    return repeatability

#majorLocator = LinearLocator(1, 23)
#majorFormatter = FormatStrFormatter('%d')

scales = range(3, 23, 2)

wg40 = loadRepeatabilityCurve("willow", scales, 40)
strict_ot = 20
wg_strict = loadRepeatabilityCurve("willow", scales, strict_ot)

ax = subplot(111)
plot(scales, wg40, 'bo-')
plot(scales, wg_strict, 'ro-')
xlabel('# scales')
ylabel('repeatability %')
title('Repeatability vs. scales')
grid(True)
#ax.xaxis.set_major_locator(majorLocator)
#ax.xaxis.set_major_formatter(majorFormatter)
legend(('40% ot', '20% ot'))

show()
