
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
