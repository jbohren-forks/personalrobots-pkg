#!/usr/bin/env python

import rospy
import rosservice
import Image
import cStringIO
import time

_map_cache = {}
_scale_cache = {}

def config_plugin():
  return [{'url': '/map', 'handler': map_tiler_handler}]

def map_tiler_handler(self, path, qdict):
  service_name = qdict.get('service', ['static_map'])[0]
  scale = float(qdict.get('scale', [1])[0])
  x = int(qdict.get('x', [0])[0])
  y = int(qdict.get('y', [0])[0])
  width = int(qdict.get('width', [256])[0])
  height = int(qdict.get('height', [256])[0])
  map_key = service_name
  scale_key = '%s:%s' % (service_name, scale)
  start_time = time.time()
  try:
    im = _scale_cache[scale_key]
  except KeyError:
    try:
      im = _map_cache[map_key]
    except KeyError:
      rospy.wait_for_service(service_name)
      service_class = rosservice.get_service_class_by_name('/' + service_name)
      service_proxy = rospy.ServiceProxy(service_name, service_class)
      resp = service_proxy()
      size = (resp.map.info.width, resp.map.info.height)
      im = Image.frombuffer('L', size, resp.map.data, "raw", 'L', 0, -1)
      remap = {0:255, 100: 0, 255:128}
      im = im.point(lambda x: remap.get(x, 0))
      _map_cache[map_key] = im
    if scale != 1:
      size = im.size
      im = im.resize((int(size[0] / scale), int(size[1] / scale)), Image.BICUBIC)
    _scale_cache[scale_key] = im

  im = im.crop((x, y, x + width, y + height))
  buf = cStringIO.StringIO()
  im.save(buf, format='JPEG')
  jpeg = buf.getvalue()
  self.send_response(200)
  self.send_header('Content-Type', 'image/jpeg')
  self.send_header('Content-Length', str(len(jpeg)))
  self.end_headers()
  self.wfile.write(jpeg)
  print " Overall time: %f\n" % (time.time() - start_time)
