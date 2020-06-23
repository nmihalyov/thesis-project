import math

from assets.Vector import *

class Quaternion:
  def __init__(self, w=1, x=0, y=0, z=0):
    self.w = w
    self.x = x
    self.y = y
    self.z = z


  def set_from_vector(self, angle, dir):
    half_angle = angle / 2
    sin_half_angle = math.sin(half_angle)
    self.w = math.cos(half_angle)
    self.x = sin_half_angle * dir.x
    self.y = sin_half_angle * dir.y
    self.z = sin_half_angle * dir.z


  def multiply(self, q):
    res = Quaternion()
    res.w = self.w * q.w - self.x * q.x - self.y * q.y - self.z * q.z
    res.x = self.w * q.x + self.x * q.w + self.y * q.z - self.z * q.y
    res.y = self.w * q.y - self.x * q.z + self.y * q.w + self.z * q.x
    res.z = self.w * q.z + self.x * q.y - self.y * q.x + self.z * q.w
    return res


  def norm(self):
    return self.x ** 2 + self.y ** 2 + self.z ** 2 + self.w ** 2


  def inverse(self):
    res = Quaternion()
    n = self.norm()
    res.w = self.w / n
    res.x = -self.x / n
    res.y = -self.y / n
    res.z = -self.z / n
    return res


  def rotate(self, v):
    vect_quad = Quaternion(w=0, x=v.x, y=v.y, z=v.z)
    q1 = self.multiply(vect_quad)
    res = q1.multiply(self.inverse())
    return Vector3(x=res.x, y=res.y, z=res.z)