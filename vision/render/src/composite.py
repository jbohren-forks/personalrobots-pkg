from vop import *

class color:
    def __init__(self, r, g, b, a=1.0):
        self.r = r
        self.g = g
        self.b = b
        self.a = a
    def channels(self, mode = "RGB"):
        if mode == "RGB":
            return [self.r, self.g, self.b]
        else:
            return [self.r, self.g, self.b, self.a]
    def __mul__(self, other):
        if isinstance(other, color):
            return color(self.r * other.r, self.g * other.g, self.b * other.b, self.a * other.a)
        else:
            return color(self.r * other, self.g * other, self.b * other, self.a * other)
    def __add__(self, other):
        return color(self.r + other.r, self.g + other.g, self.b + other.b, self.a + other.a)
    def __sub__(self, other):
        return color(self.r - other.r, self.g - other.g, self.b - other.b, self.a - other.a)
    def modulate(self, v):
        return color(self.r * v, self.g * v, self.b * v, self.a * v)
#    def take(self, i):
#        return color(take(self.r, i), take(self.g, i), take(self.b, i), take(self.a, i))
    def expand(self, mask):
        if 0:
            ex = add.accumulate(mask) - 1
            r = where(mask, take(self.r, ex), 0)
            g = where(mask, take(self.g, ex), 0)
            b = where(mask, take(self.b, ex), 0)
            a = where(mask, take(self.a, ex), 0)
        else:
            r = expand(mask, self.r, 0);
            g = expand(mask, self.g, 0);
            b = expand(mask, self.b, 0);
            a = expand(mask, self.a, 0);
        return color(r, g, b, a)
    def __str__(self):
        return str((self.r, self.g, self.b))

def invert(c):
    return color(1,1,1) - c

def overA(a, b):
    r = a + b * (1.0 - a.a)
    r.a = 1.0
    return r

def over(a, b):
    return a * a.a + b * (1.0 - a.a)

def screen(a, b):
    return invert(invert(a) * invert(b))

def fog(base, fog, factor):
    a = base.a
    bf = base * (1 - factor)
    ff = fog * factor * a
    c = bf + ff
    return color(c.r, c.g, c.b, a)

def colorx(fro, to):
    return color(float(to.r) / fro.r, float(to.g) / fro.g, float(to.b) / fro.b)

def monochrome(c):
    l = c.r * .299 + c.g * .587 + c.b * .114
    return color(l,l,l,c.a)

def blend(a, b, t):
    return a * (1 - t) + b * t

filter_85b = color(194. / 103., 202. / 156., 190. / 217.)
filter_h85b = (filter_85b + color(1,1,1)) * 0.5
sky = color(194./255.0, 202./255.0, 190./255.0)

def hsv(cc):
    (r,g,b) = (cc.r,cc.g,cc.b)
    mn = minimum(minimum(r, g), b)
    mx = maximum(maximum(r, g), b)
    v = mx
    delta = mx - mn
    if alltrue(delta == 0.0):
        return [0,0,0]
    rd = 1.0 / delta
    h = 60 * where(r == mx, (g - b)*rd, where(g == mx, 2 + (b - r) * rd, 4 + (r - g) * rd))
    h = where(delta > 0, where(h < 0, 360 + h, h), -1)
    s = where(delta > 0, delta / mx, 0)
    return [h,s,v]

def nth(index, L):
    r = L[0]
    for i in range(1, len(L)):
        r = where(i == index, L[i], r)
    return r

def fromhsv(hsv):
    (h,s,v) = hsv
    h /= 60
    i = floor(h)
    f = h - i

    p = v * ( 1 - s );
    q = v * ( 1 - s * f );
    t = v * ( 1 - s * ( 1 - f ) );

    return color(nth(i, [v,q,p,p,t,v]),
                 nth(i, [t,v,v,q,p,p]),
                 nth(i, [p,p,t,v,v,q]))

def desaturate(c, factor):
    a = c.a
    [h,s,v] = hsv(c)
    s *= factor
    c = fromhsv([h,s,v])
    c.a = a
    return c

def mpow(x, y):
    r = 1.
    for i in range(8):
        x = sqrt(x)
        y *= 2
        if y >= 1:
            r *= x
            y -= 1
    return r
