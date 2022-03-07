#!/usr/bin/env python

class Robot:
	def __init__(self, x, y, heading, spot):
		self.x = float(0)
		self.y = float(0)
		self.heading = float(0)
		self.spot = float(0)
		self.distance = float(0)

class Spot:
	def __init__(self, x, y, number):
		self.x = float(0)
		self.y = float(0)
		self.occupied = bool(False)
		self.number = float(0)

