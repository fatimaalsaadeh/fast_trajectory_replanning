import math
from algorithms import *


class Node:
    def __init__(self, is_blocked=False):
        self.is_blocked = is_blocked
        self.parent = None
        self.next = None
        self.x = 0
        self.y = 0
        self.h = 0
        self.g = 0
        self.f = 0
        self.search=0

def manhattan_distance(parent, current):
    (x1, y1) = parent
    (x2, y2) = current
    return (abs(x1 - x2) + abs(y1 - y2))
