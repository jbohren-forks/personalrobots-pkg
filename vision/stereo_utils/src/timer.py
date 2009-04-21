
import time

class Timer:
  def __init__(self):
    self.sum = 0
    self.log = []

  def reset(self):
    self.sum = 0
  def start(self):
    self.started = time.time()
  def stop(self):
    took = time.time() - self.started
    self.sum += took
    self.log.append(took)
  def summ(self):
    if self.log == []:
      return "0 calls"
    else:
      return "%d calls, avg %fms" % (len(self.log), 1e3 * self.sum / len(self.log))
