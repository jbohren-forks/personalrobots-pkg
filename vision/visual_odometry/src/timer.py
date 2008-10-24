
import time

class Timer:
  def __init__(self):
    self.sum = 0
  def reset(self):
    self.sum = 0
  def start(self):
    self.sum -= time.time()
  def stop(self):
    self.sum += time.time()
