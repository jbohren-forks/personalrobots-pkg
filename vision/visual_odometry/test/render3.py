import rostools
rostools.update_path('visual_odometry')
import rostest
import rospy

from vop import *
from composite import *

import Image

do_compression=0

class vec:
    def distance(self, other):
        return abs(self - other)
    def __neg__(self):
        return self * -1.
    def __abs__(self):
        d = self.dot(self)
        if isinstance(d, float):
            return sqrt(d)
        else:
            return sqrt(d)
    def normalize(self):
        a = abs(self)
        return self * (1. / a)

class vec3(vec):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def __mul__(self, other):
        return vec3(self.x * other, self.y * other, self.z * other)
    def __add__(self, other):
        return vec3(self.x + other.x, self.y + other.y, self.z + other.z)
    def __sub__(self, other):
        return vec3(self.x - other.x, self.y - other.y, self.z - other.z)
    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z
    def __repr__(self):
        return "vec3(%s,%s,%s)" % (str(self.x), str(self.y), str(self.z))
    def compress(self, mask):
        #print len(compress(mask, self.x)), "/", len(self.x)
        return vec3(compress(mask, self.x), compress(mask, self.y), compress(mask, self.z))
    def expand(self, mask):
        ex = add.accumulate(mask) - 1
        x = where(mask, take(self.x, ex), 0.)
        y = where(mask, take(self.y, ex), 0.)
        z = where(mask, take(self.z, ex), 0.)
        return vec3(x, y, z)

def mag2(v):
    return v.dot(v)

def distance(a, b):
    return abs(a - b)

def cross(a, b):
  return vec3(a.y * b.z - b.y * a.z, a.z * b.x - b.z * a.x, a.x * b.y - b.x * a.y)

class quat:
    def __init__(self, x, y, z, w):
        self.x = (x)
        self.y = (y)
        self.z = (z)
        self.w = (w)
    def __abs__(self):
        return sqrt(self.x*self.x + self.y*self.y + self.z*self.z + self.w*self.w)
    def normalize(self):
        fac = 1.0 / abs(self)
        return quat(fac * self.x, fac * self.y, fac * self.z, fac * self.w)
    def __str__(self):
        return "(%f, %f, %f, %f)" % (self.x, self.y, self.z, self.w)

#                 2     2                                      
#          1 - (2Y  + 2Z )   2XY + 2ZW         2XZ - 2YW       
#                                                              
#                                   2     2                    
#     M =  2XY - 2ZW         1 - (2X  + 2Z )   2YZ + 2XW       
#                                                              
#                                                     2     2  
#          2XZ + 2YW         2YZ - 2XW         1 - (2X  + 2Y ) 
#                                                              

    def tomatrix(self):
        x = self.x
        y = self.y
        z = self.z
        w = self.w
        return Numeric.transpose(Numeric.array([[1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w), 0],
                      [2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w), 0],
                      [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y), 0],
                      [ 0, 0, 0, 1]], Numeric.Float))
    def tolist(self):
        return [ self.x, self.y, self.z, self.w ]

    def __mul__(self, other):
        v1 = vec3(self.x, self.y, self.z)
        v2 = vec3(other.x, other.y, other.z)
        w1 = self.w
        w2 = other.w
        rv = v2 * w1 + v1 * w2 + cross(v1, v2)
        rw = w1 * w2 - v1.dot(v2)
        return quat(rv.x, rv.y, rv.z, rw)

    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w

    def scale(self, fac):
        return quat(fac * self.x, fac * self.y, fac * self.z, fac * self.w)

    def inverse(self):
        length = 1.0 / self.dot(self)
        return quat(self.x * -length, self.y * -length, self.z * -length, self.w * length)
        
    def __add__(self, other):
        return quat(self.x + other.x, self.y + other.y, self.z + other.z, self.w + other.w)

    def __sub__(self, other):
        return quat(self.x - other.x, self.y - other.y, self.z - other.z, self.w - other.w)

    def set_value(self, other):
        self.x = other.x
        self.y = other.y
        self.z = other.z
        self.w = other.w

    def lerp(self, t, a, b):
        d = a.dot(b)

        # XXX fixme - if d is a vop then this is always true
        if d > 0.995:
            self.set_value(a.scale(1.0 - t) + b.scale(t))
        else:
            if 0:   # math version
                theta_0 = math.acos(d)
                theta = theta_0*t
                v2 = (b - a.scale(d)).normalize()
                self.set_value(a.scale(math.cos(theta)) + v2.scale(math.sin(theta)))
            else:
                theta_0 = acos(d)
                theta = theta_0*t
                v2 = (b - a.scale(d)).normalize()
                self.set_value(a.scale(cos(theta)) + v2.scale(sin(theta)))


def mkquat(axis, th):
    axis = axis.normalize()
    sin_a = sin(th / 2)
    cos_a = cos(th / 2)
    X    = axis.x * sin_a;
    Y    = axis.y * sin_a;
    Z    = axis.z * sin_a;
    W    = cos_a;
    return quat(X, Y, Z, W)

def lerp(a, b, t):
    return (a * (1. - t)) + (b * t)

def clamp(x, lo, hi):
    return where(x < lo, lo, where(hi < x, hi, x))

def frac(x):
    return x - floor(x)

class ray:
    def __init__(self, origin, direction, rand):
        self.o = origin
        self.d = direction
        self.rand = rand
    def project(self, d):
        return self.o + self.d * d
    def random(self):
        return self.rand
    def compress(self, mask):
        return ray(self.o.compress(mask), self.d.compress(mask))

def vecbyQ(vv, q):
    r = q.inverse() * quat(vv.x,vv.y,vv.z,0) * q
    return vec3(r.x, r.y, r.z)

class ray_camera:

    def __init__(self, pose, stereocam):
        (Fx, Fy, Tx, Clx, Crx, Cy) = stereocam.params

        def local(x, y, z):
           x,y,z = pose.xform(x, y, z)
           return vec3(float(x), float(y), float(z))
        S = local(0, 0, 0)
        C = local(0, 0, Fx)
        R = local(320., 0., 0.)
        U = local(0, 240., 0)
        tx = local(Tx, 0, 0)

        self.center = [local(0, 0, 0), local(Tx, 0, 0)]
        self.pcenter = [local(0, 0, Fx), local(Tx, 0, Fx)]
        self.up = [U, U]
        self.right = [R, R]

        self.xcenter = (320. - Clx) / 320.
        self.ycenter = (240. - Cy) / 240.

        self.rpool = array([random.random() for i in range(1024)])

    def compress(self, mask):
        c = ray_camera(vec3(0,0,0), quat(0, 0, 0, 1), 1, 1)
        c.center = self.center.compress(mask)
        c.pcenter = self.pcenter.compress(mask)
        c.up = self.up.compress(mask)
        c.right = self.right.compress(mask)
        return c

    def genray(self, sx, sy, time, side):
        scr = self.pcenter[side] + self.right[side] * (sx + self.xcenter) + self.up[side] * (sy + self.ycenter)
        rand = take(self.rpool, frac(time + 8.1 * sx + 1.07 * (sx * sx) + 3.41 * sy + 7.7 * ((99-sy) * sy)) * 1024)
        return ray(self.center[side], (scr - self.center[side]).normalize(), rand)

    # intersection of ray (self.center to p) and plane (self.pcenter, self.right, self.up)
    def trace(self, p):
        z = pt2plane(p - self.center, (self.pcenter - self.center)) / abs(self.pcenter - self.center)
        x = pt2plane(p - self.center, self.right) / abs(self.right)
        y = pt2plane(p - self.center, self.up) / abs(self.up)
        return vec3(x / z, y / z, z)

    # Would trace above return a value in range -1,1 in x and y
    def wouldhit(self, p):
        z = pt2plane(p - self.center, (self.pcenter - self.center)) / abs(self.pcenter - self.center)
        x = pt2plane(p - self.center, self.right) / abs(self.right)
        y = pt2plane(p - self.center, self.up) / abs(self.up)

        return (0 < z) & (abs(x) < z) & (abs(y) < z)

MAXFLOAT = 9999999.
def safesqrt(f):
    return sqrt(f)
def arrayrange(f):
    return arange(float(f))
def bytes(a):
    return a.tostring()
def hse2(L, PL):
    if len(L) == 1:
        return (L[0], PL)
    else:
        mid = len(L) / 2
        (amin, AL) = hse2(L[:mid], PL[:mid])
        (bmin, BL) = hse2(L[mid:], PL[mid:])
        m = minimum(amin, bmin)
        awins = (m == amin)
        bwins = ~awins
        return (m, [awins & p for p in AL] + [bwins & p for p in BL])
def hse(HL, PL):
    (m, P) = hse2(HL, PL)
    return P

def color2string(x, size):
    if isinstance(x, float) or isinstance(x, int):
        return chr(int(min(x,1) * 255)) * size
    else:
        return bytes((minimum(x, 1.) * 255.))

def shader(mask, f, p, n, o, r, args):
    if sometrue(mask):
        if not do_compression or alltrue(mask):
            final = apply(f, (p,n,o,r, args))
        else:
            puniq = p.compress(mask)
            runiq = r.compress(mask)
            nuniq = n.compress(mask)
            if 1:
                argsuniq = args
            else:
                if 'scene' in args:
                    rosc = []
                    for x in args['scene']:
                        rosc += [ x.compress(mask).newscene(rosc) ]
                    argsuniq = rosc[args['scene'].index(o)].args
                    o = rosc[args['scene'].index(o)]
                else:
                    argsuniq = compressArgs(args, mask)

            result = apply(f, (puniq,nuniq,o,runiq,argsuniq))
            final = result.expand(mask)
    else:
        final = color(0.,0.,0.,0.)
    return final

def fog(col, distance, p):
    if 0:
        maxdist = 64
        fogfact = distance * (1.0/maxdist)
        fogcolor = color(0.6,0.6,0.6)
        col = lerp(col, fogcolor, fogfact)
        ok = distance < maxdist
        col = color(where(ok, col.r, fogcolor.r), where(ok, col.g, fogcolor.g), where(ok, col.b, fogcolor.b))
        return col * where(p, 1, 0)
    else:
        return color(where(p,col.r,0),where(p,col.g,0),where(p,col.b,0),where(p,col.a,0))

def raytrace(ray, scene):
    # print "raytracing with scene size", len(scene)
    Result = [ o.hit(ray) for o in scene]
    Ps = [ r[0] for r in Result ]
    Hs = [ r[1] for r in Result ]
    Ns = [ r[2] for r in Result ]

    slicer = [ sometrue(p) for p in Ps ]
    objects_in_tile = sum([s and 1 for s in slicer])
    if objects_in_tile == 0:
        final = color(0., 0., 1., 1.)
    else:
        tPs = [ P for (s, P) in zip(slicer, Ps) if s ]
        tHs = [ H for (s, H) in zip(slicer, Hs) if s ]
        tNs = [ N for (s, N) in zip(slicer, Ns) if s ]
        tOs = [ H for (s, H) in zip(slicer, scene) if s ]
        tHPs = hse(tHs, tPs)
        Cs = [fog(o.shade(ray, h, n, p), h, p) for (o, h, n, p) in zip(tOs, tHs, tNs, tHPs)]
        distances = [ where(p, h, 0) for (p, h) in zip(tHPs, tHs) ]
        final = Cs[0]
        for i in Cs[1:]:
            final += i
        rnd = 0.1 * ray.random()
        final.r += rnd
        final.g += rnd
        final.b += rnd
        final_d = distances[0]
        for i in distances[1:]:
            final_d += i
        final_d = ray.d * final_d

    return (objects_in_tile, final, final_d)

def render_stereo_scene(imL, imR, imD, cam, scene, time):
    im = imL
    x1d = (-1. + arrayrange(float(imL.size[0])) / float(im.size[0] / 2))
    y1d = (-1. + arrayrange(float(imL.size[1])) / float(im.size[1] / 2))

    tilexd = im.size[0] / 8
    tileyd = im.size[1] / 8
    tilesz = tilexd * tileyd

    xs_dict = {}
    for x in range(0, im.size[0], tilexd):
        xs_dict[x] = duplicate(array(x1d.tolist()[x:x+tilexd]), tileyd)

    for side in range(2):
      for y in range(0, im.size[1], tileyd):
          ys = replicate(array(y1d.tolist()[y:y+tileyd]), tilexd)
          for x in range(0, im.size[0], tilexd):
              xs = xs_dict[x]

              ray = cam.genray(xs, ys, time, side)
              (objects_in_tile, final, final_d) = raytrace(ray, scene)

              tup = tuple([ Image.fromstring("L", (tilexd, tileyd), color2string(chan, tilesz)) for chan in final.channels()])
              tile = Image.merge("RGB", tup)
              [imL,imR][side].paste(tile, (x,y))
              
              if side == 0:
                imD[0].paste(Image.fromstring("F", (tilexd, tileyd), final_d.x.toraw()), (x,y))
                imD[1].paste(Image.fromstring("F", (tilexd, tileyd), final_d.y.toraw()), (x,y))
                imD[2].paste(Image.fromstring("F", (tilexd, tileyd), final_d.z.toraw()), (x,y))

dummyNormal = vec3(0.,1.,0.)
class sphere:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
    def compress(self, mask):
        return sphere(self.center.compress(mask), self.radius)
    def hit(self, r):
        # a = mag2(r.d)
        a = 1.
        v = r.o - self.center
        b = 2 * (r.d.x * v.x + r.d.y * v.y + r.d.z * v.z)
        c = mag2(self.center) + mag2(r.o) + 2 * (-self.center.x * r.o.x - self.center.y * r.o.y - self.center.z * r.o.z) - self.radius * self.radius
        det = (b * b) - (4 * c)
        pred = 0 < det
 
        if not sometrue(pred):
            return (pred, MAXFLOAT, dummyNormal)
        sq = safesqrt(det)
        h0 = (-b - sq) / (2)
        h1 = (-b + sq) / (2)

        h = minimum(h0, h1)

        pred = pred & (h > 0)
        normal = (r.project(h) - self.center) * (1.0 / self.radius)
        return (pred, where(pred, h, MAXFLOAT), normal)

class isphere:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
    def compress(self, mask):
        return isphere(self.center.compress(mask), self.radius)
    def hit(self, r):
        # a = mag2(r.d)
        a = 1.
        v = r.o - self.center
        b = 2 * (r.d.x * v.x + r.d.y * v.y + r.d.z * v.z)
        c = mag2(self.center) + mag2(r.o) + 2 * (-self.center.x * r.o.x - self.center.y * r.o.y - self.center.z * r.o.z) - self.radius * self.radius
        det = (b * b) - (4 * c)
        pred = 0 < det
        if not sometrue(pred):
            return (pred, MAXFLOAT, dummyNormal)
        sq = safesqrt(det)
        h0 = (-b - sq) / (2)
        h1 = (-b + sq) / (2)

        h = maximum(h0, h1)

        pred = pred & (h > 0)
        normal = (r.project(h) - self.center) * (1.0 / self.radius)
        return (pred, where(pred, h, MAXFLOAT), normal)

def shadeRed(p, n, o, r, args):
    return color(1,0,0)

########################################################################

# Based on "JAVA REFERENCE IMPLEMENTATION OF IMPROVED NOISE - COPYRIGHT 2002 KEN PERLIN."
# http://mrl.nyu.edu/~perlin/noise/

p0 = [ 151,160,137,91,90,15,
   131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
   190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
   88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
   77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
   102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
   135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
   5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
   223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
   129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
   251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
   49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
   138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180] * 2

perm = array(p0)
perm_30 = array([(i / 4) % 3 for i in p0]) > 1.
perm_31 = array([(i / 4) % 3 for i in p0]) < 2.
perm_20 = array([i % 2 for i in p0]) > 0.
perm_21 = array([i % 4 for i in p0]) > 2.

def fmod(x, y):
    q = x / y
    f = q - floor(q)
    return f * y

def mod256(x):
    q = x * (1.0 / 256.0)
    f = q - floor(q)
    return f * 256

def fade(t):
    return t * t * t * (t * (t * 6 - 15) + 10);

def grad(prehash, x, y, z):
    u = where(take(perm_30, prehash), x, y)
    v = where(take(perm_31, prehash), z, y)
    return where(take(perm_20, prehash), u, -u) + where(take(perm_21, prehash), v, -v)

def noise(p):
    X = mod256(p.x)
    Y = mod256(p.y)
    Z = mod256(p.z)

    x = p.x - floor(p.x)
    y = p.y - floor(p.y)
    z = p.z - floor(p.z)

    u = fade(x)
    v = fade(y)
    w = fade(z)

    A  = take(perm, X) + Y
    AA = take(perm, A) + Z
    AB = take(perm, A + 1) + Z
    B  = take(perm, X+1) + Y
    BA = take(perm, B) + Z
    BB = take(perm, B + 1) + Z

    return lerp(lerp(lerp(grad(AA  , x    , y    , z    ), grad(BA  , x - 1, y    , z    ), u),
                     lerp(grad(AB  , x    , y - 1, z    ), grad(BB  , x - 1, y - 1, z    ), u), v),
                lerp(lerp(grad(AA+1, x    , y    , z - 1), grad(BA+1, x - 1, y    , z - 1), u),
                     lerp(grad(AB+1, x    , y - 1, z - 1), grad(BB+1, x - 1, y - 1, z - 1), u), v), w)

def shadeCloud(p, n, o, r, args):
    p *= args['scale']
    #p = vec3(floor(p.x), floor(p.y), floor(p.z))
    t = clamp(0.5 + noise(p) + noise(p*4) * 0.5, 0, 1)
    return color(t,t,t)
    t = 0.0
    for i in range(10):
        pow = 1 << i
        t += (noise(p * pow)) / pow
    t = where(t < 0.0, 0.0, 1.0)
    t = clamp(1.0 * t, 0, 1)
    return lerp(color(0.1, 0.1, 0.1), color(.9, .9, .9), t)

def shadeLitCloud(p, n, o, r, args):
    diffcol = shadeCloud(p, n, o, r, args)
    i = maximum(0.3, n.dot(vec3(0,-1,-1).normalize()))
    return diffcol * i

class object:
    def __init__(self, shape, material, args = {}):
        self.shape = shape
        self.material = material
        self.args = args
    def compress(self, mask):
        return object(self.shape.compress(mask), self.material, compressArgs(self.args, mask))
    def newscene(self, ros):
        return object(self.shape, self.material, dreplace(self.args, 'scene', ros))
    def hit(self, ray):
        return self.shape.hit(ray)
    def shade(self, ray, h, n, mask):
        return shader(mask, self.material, ray.project(h), n, self, ray.d, self.args)
    def tolocal(self, p):
        return p

if 0:
  pylab.hist(noise(vec3(arrayrange(100000) * 1e-2, 7.7, 9.0)), bins=100)
  pylab.show()
  sys.exit(1)

import numpy
import pylab

import camera
import random
import math
import sys
import visualize

def rotation(angle, x, y, z):
  return numpy.array([
    [ 1 + (1-cos(angle))*(x*x-1)         ,    -z*sin(angle)+(1-cos(angle))*x*y   ,    y*sin(angle)+(1-cos(angle))*x*z  ],
    [ z*sin(angle)+(1-cos(angle))*x*y    ,    1 + (1-cos(angle))*(y*y-1)         ,    -x*sin(angle)+(1-cos(angle))*y*z ],
    [ -y*sin(angle)+(1-cos(angle))*x*z   ,    x*sin(angle)+(1-cos(angle))*y*z    ,    1 + (1-cos(angle))*(z*z-1)       ]])

stereo_cam = camera.Camera((389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95))

random.seed(0)
scene = []
if 0:
  for obj in range(100):
    theta = random.random() * math.pi * 2
    center = vec3(math.cos(theta), math.sin(theta), obj / 10.)
    scene += [object(sphere(center, 0.2), shadeLitCloud, {'scale':10})]
else:
  for obj in range(180):
    theta = (obj / 180.0) * math.pi * 2
    phi = random.random() * math.pi * 2
    rmaj = 4.0
    costh = math.cos(theta)
    sinth = math.sin(theta)
    r = rmaj + math.cos(phi)
    center = vec3(rmaj - r * costh, math.sin(phi), r * sinth)
    scene += [object(sphere(center, 0.2), shadeLitCloud, {'scale':10})]

scene += [object(isphere(vec3(0,0,0), 1000), shadeLitCloud, {'scale':0.001})]

from stereo import ComputedDenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris

class imgStereo:
  def __init__(self, im):
    self.size = im.size
    self.data = im.tostring()
  def tostring(self):
    return self.data

class PhonyVisualOdometer(VisualOdometer):
  def find_keypoints(self, frame):
    pass
  def find_disparities(self, frame):
    pass
  def collect_descriptors(self, frame):
    pass
  def temporal_match(self, af0, af1, want_distances = False):
    return [(i,i) for i in range(400)]

points = [ (-1 + 2 * random.random(), -1 + 2 * random.random(), 1 + 5 * random.random()) for i in range(400)]

class PhonyFrame:
  def __init__(self, pose, cam):
    p3d = [pose.xform(*p) for p in points]
    self.kp = [cam.cam2pix(*p) for p in p3d]
    def r(): return -0.5 + random.random() * 1
    self.kp = [(x+r(),y+r(),d) for (x,y,d) in self.kp]

vos = [
  #VisualOdometer(stereo_cam, sba=(1,1,1)),
  #VisualOdometer(stereo_cam, sba=None, inlier_error_threshold = 1.0),
  VisualOdometer(stereo_cam, sba=(3,10,10), inlier_error_threshold = 1.0),
  #VisualOdometer(stereo_cam, sba=(5,5,10), inlier_error_threshold = 1.5),
  #VisualOdometer(stereo_cam, sba=(5,5,10), inlier_error_threshold = 2.0),
  #VisualOdometer(stereo_cam, sba=(5,5,10), inlier_error_threshold = 2.5),
  #VisualOdometer(stereo_cam, sba=(5,5,10), inlier_error_threshold = 3.0),
  #VisualOdometer(stereo_cam, sba=(5,5,10), inlier_error_threshold = 3.5),
  #VisualOdometer(stereo_cam, feature_detector = FeatureDetector4x4(FeatureDetectorHarris), scavenge = True, sba = None),

  #PhonyVisualOdometer(stereo_cam),
  #PhonyVisualOdometer(stereo_cam, sba = (5,5,5))
  #VisualOdometer(stereo_cam, feature_detector = FeatureDetector4x4(FeatureDetectorFast)),
  #VisualOdometer(stereo_cam, scavenge = True, sba = (1,1,5)),
]

class MagicStereoFrame(SparseStereoFrame):
  def __init__(self, L, R, D, cam):
    SparseStereoFrame.__init__(self, L, R)
    self.D = D
    self.cam = cam
  def lookup_disparity(self, px, py):
    x = self.D[0].getpixel((px,py))
    y = self.D[1].getpixel((px,py))
    z = self.D[2].getpixel((px,py))
    (x,y,d) = self.cam.cam2pix(x, y, z)
    assert abs(x - px) < 2
    return d

use_magic_disparity = False

trajectory = [ [] for i in vos]
for i in range(401):
  print i
  if 0:
    desired_pose = Pose(rotation(0, 0,1,0), (0,0,0.01*i))
    cam = ray_camera(desired_pose, stereo_cam)
  else:
    theta = (i / 400.0) * math.pi * 2
    r = 4.0
    cam = ray_camera(Pose(rotation(theta, 0,1,0), (r - r * math.cos(theta),0,r * math.sin(theta))), stereo_cam)

  if 0:
    imL = Image.new("RGB", (640,480))
    imR = Image.new("RGB", (640,480))
    imD = [ Image.new("F", (640,480)), Image.new("F", (640,480)), Image.new("F", (640,480)) ]
    render_stereo_scene(imL, imR, imD, cam, scene, i * 0.033)
    imL = imL.convert("L")
    imR = imR.convert("L")
    im = Image.merge("RGB", (imL, imR, imR))
    im.save("out%06d.png" % i)
    imD[0].save("out%06d-x.tiff" % i)
    imD[1].save("out%06d-y.tiff" % i)
    imD[2].save("out%06d-z.tiff" % i)
  else:
    fn = i
    im = Image.open("out%06d.png" % fn)
    imL,imR,_ = im.split()
    if use_magic_disparity:
      imD = [ Image.open("out%06d-x.tiff" % fn), Image.open("out%06d-y.tiff" % fn), Image.open("out%06d-z.tiff" % fn) ]
    imL,imR = imR,imL
    imL = imL.transpose(Image.FLIP_LEFT_RIGHT)
    imR = imR.transpose(Image.FLIP_LEFT_RIGHT)
  if 1:
    for j,vo in enumerate(vos):
      f = SparseStereoFrame(imgStereo(imL), imgStereo(imR))
      #f = MagicStereoFrame(imgStereo(imL), imgStereo(imR), imD, stereo_cam)

      #f = PhonyFrame(~desired_pose, stereo_cam)
      #vo.lock_handle_frame(f)
      vo.handle_frame(f)
      #visualize.viz(vo, f)
      (x,y,z) = vo.pose.xform(0,0,0)
      if 0:
        print vo.name(), (x,y,z)
        (ex,ey,ez) = desired_pose.xform(0,0,0)
        Ys[j].append(z - ez)
      else:
        trajectory[j].append((x,y,z))

def planar(x, y, z):
  from scipy import optimize

  def silly(sol, args):
      a,b,c = sol
      x,y,z = args
      return sum((y - (a*x + b*z + c)) ** 2)

  sol = [1.0, 1.0, 1.0]
  sol = optimize.fmin(silly, sol, args=((x,y,z),))

  a,b,c = sol
  return sqrt(sum((y - (a*x + b*z + c)) ** 2) / len(x))

colors = [ 'blue', 'red', 'green', 'black', 'magenta', 'cyan' ]
for i in range(len(vos)):
  pylab.plot([x for (x,y,z) in trajectory[i]], [z for (x,y,z) in trajectory[i]], label = vos[i].name(), color = colors[i])

xlim = pylab.xlim()
ylim = pylab.ylim()
xrange = xlim[1] - xlim[0]
yrange = ylim[1] - ylim[0]
r = max(xrange, yrange) * 1.5
mid = sum(xlim) / 2
pylab.xlim(mid - 0.5 * r, mid + 0.5 * r)
mid = sum(ylim) / 2
pylab.ylim(mid - 0.5 * r, mid + 0.5 * r)

pylab.legend()
#pylab.savefig("foo.png")
pylab.show()

quality_pose = Pose()

for i,vo in enumerate(vos):
  vos[i].planarity = planar(numpy.array([x for (x,y,z) in trajectory[i]]), numpy.array([y for (x,y,z) in trajectory[i]]), numpy.array([z for (x,y,z) in trajectory[i]]))
  print vo.name()
  print "distance from start:", vo.pose.distance()
  print "planarity", vo.planarity
  print "pose", vo.pose.comparison(quality_pose)
  vo.summarize_timers()
  print vo.log_keyframes
  print
