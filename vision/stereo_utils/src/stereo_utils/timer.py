"""
:mod:`stereo_utils.timer` --- Stopwatch
=======================================
"""

import time

class Timer:
  def __init__(self):
    self.sum = 0
    self.log = []
    self.running = False

  def reset(self):
    self.sum = 0
    self.log = []
    self.running = False
  def start(self):
    assert not self.running
    self.running = True
    self.started = time.time()
  def stop(self):
    assert self.running
    self.running = False
    took = time.time() - self.started
    self.sum += took
    self.log.append(took)
  def summ(self):
    if self.log == []:
      return "0 calls"
    else:
      return "%d calls, avg %fms" % (len(self.log), 1e3 * self.sum / len(self.log))

class TimedClass:
  """
  *timing* is a sequence of timing categories.  The inheriting class will call::

    self.timer['x'].start()
    ... do something
    self.timer['x'].stop()

  for each timer.

  """
  def __init__(self, timing):
    self.calls = 0
    self.timer = {}
    for t in timing:
      self.timer[t] = Timer()

  def summarize_timers(self):
    """
    Print a summary of time spent in each registered timing category.
    """
    print
    print self.name()
    for n,t in self.timer.items():
      print "  %-20s %s" % (n, t.summ())
