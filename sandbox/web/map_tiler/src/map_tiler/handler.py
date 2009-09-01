#!/usr/bin/env python

import rospy
import rosservice
import Image
import cStringIO
import unavail

import tf
import tf.transformations

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point

_map_cache = {}
_scale_cache = {}
_tile_cache = {}

def config_plugin():
  global goal_publisher
  goal_publisher = rospy.Publisher('/move_base/activate', PoseStamped)
  return [{'url': '/map', 'handler': map_tiler_handler}]

def get_map(service_name):
  map_key = service_name
  if not _map_cache.has_key(map_key):
    service_class = rosservice.get_service_class_by_name('/' + service_name)
    service_proxy = rospy.ServiceProxy(service_name, service_class)
    resp = service_proxy()
    size = (resp.map.info.width, resp.map.info.height)
    im = Image.frombuffer('L', size, resp.map.data, "raw", 'L', 0, -1)
    remap = {0:255, 100: 0, 255:128}
    im = im.point(lambda x: remap.get(x, 0))
    resolution = resp.map.info.resolution
    _map_cache[map_key] = (im, resolution)
  else:
     (im, resolution) = _map_cache[map_key]
  return (im, resolution)

def get_scaled_map(service_name, scale):
  scale_key = '%s:%s' % (service_name, scale)
  if not _scale_cache.has_key(scale_key):
    (im, resolution) = get_map(service_name)
    if scale != 1:
      size = im.size
      im = im.resize((int(size[0] / scale), int(size[1] / scale)), Image.ANTIALIAS)
    _scale_cache[scale_key] = im
  else:
    im = _scale_cache[scale_key]
  return im

def get_tile(service_name, scale, x, y, width, height):
  tile_key = '%s:%s:%s:%s:%s:%s' % (service_name, scale, x, y, width, height)
  if not _tile_cache.has_key(tile_key):
    # Get the map (possibly cached) from the map server
    im = get_scaled_map(service_name, scale)

    # Crop the requested tile and convert to JPEG
    im = im.crop((x, y, x + width, y + height))
    buf = cStringIO.StringIO()
    im.save(buf, format='JPEG')
    jpeg = buf.getvalue()
    _tile_cache[tile_key] = jpeg
  else:
    jpeg = _tile_cache[tile_key]
  return jpeg

def send_image(self, jpeg):
  self.send_response(200)
  self.send_header('Cache-Control', 'max-age=3600')
  self.send_header('Content-Type', 'image/jpeg')
  self.send_header('Content-Length', str(len(jpeg)))
  self.end_headers()
  self.wfile.write(jpeg) 
  
def get_tile_handler(self, path, qdict):
  service_name = qdict.get('service', ['static_map'])[0]
  scale = float(qdict.get('scale', [1])[0])
  x = int(qdict.get('x', [0])[0])
  y = int(qdict.get('y', [0])[0])
  width = int(qdict.get('width', [256])[0])
  height = int(qdict.get('height', [256])[0])

  try:
    jpeg = get_tile(service_name, scale, x, y, width, height)
  except Exception, reason:
    import base64
    jpeg = base64.decodestring(unavail.unavail)
  send_image(self, jpeg)

def set_goal_handler(self, path, qdict):
  x = float(qdict.get('x', [0])[0])
  y = float(qdict.get('y', [0])[0])
  angle = float(qdict.get('angle', [0])[0])
  h = rospy.Header()
  h.stamp= rospy.get_rostime()
  h.frame_id = '/map'
  p = Point(x, y, 0)
  q = tf.transformations.quaternion_from_euler(0, 0, angle)
  q = Quaternion(q[0], q[1], q[2], q[3])
  goal = PoseStamped(h, Pose(p, q))
  goal_publisher.publish(goal)
  self.send_success()

def get_extents_handler(self, path, qdict):
  service_name = qdict.get('service', ['static_map'])[0]
  callback = qdict.get("callback", [''])[0]
  (im, resolution) = get_map(service_name)
  buf = cStringIO.StringIO()
  buf.write('{')
  buf.write('"width": %d,' % im.size[0])
  buf.write('"height": %d,' % im.size[1])
  buf.write('"resolution": %s,' % resolution)
  buf.write('}')
  buf = buf.getvalue()
  self.send_success(buf, callback)

def map_tiler_handler(self, path, qdict):
  if path == '/map/get_tile':
    get_tile_handler(self, path, qdict)
  elif path == '/map/get_extents':
    get_extents_handler(self, path, qdict)
  elif path == '/map/set_goal':
    set_goal_handler(self, path, qdict)
  else:
    return False
  return True


