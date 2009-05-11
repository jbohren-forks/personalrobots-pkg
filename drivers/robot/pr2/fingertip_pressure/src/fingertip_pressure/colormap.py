def colormap1(data):
    data = data * 6000
    if data < 1000:
        return (0,0,0)
    if data < 3000:
        x = (data-1000)/2000.
        return 0,0,x
    if data < 6000:
        x = (data-3000)/3000.
        return x,0,(1-x)
    if data < 10000:
        x = (data-6000)/4000.
        return 1.0,x,x
    return 1.0,1.0,1.0

def colormap2(data):
    if data < -1:
        return 0,0,255
    if data < 0:
        x = data+1
        return 0,0,255*(1-x)
    if data < 1:
        x = data
        return 255*x,0,0
    if data < 3:
        x = data-1
        return 255,255*x,255*x
    return 255,255,255

color = colormap2
