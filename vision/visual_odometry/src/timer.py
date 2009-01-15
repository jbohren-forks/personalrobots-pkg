
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
