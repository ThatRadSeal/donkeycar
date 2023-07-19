import queue

class VelocityQueue:

  def __init__(self):
    # queue to keep track of car velocities read from the encoder
    self.queue = queue.Queue()

  # return the next item in queue without removing
  def top(self):
    if not self.queue.empty():
      velocity = self.queue.get()
      self.queue.put(velocity)
      return velocity
    else:
      return 0

velocity_queue = VelocityQueue()
