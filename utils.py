import math


class Node:
    # 0 = blocked, 1 = regular unblocked, 2 = hard to traverse
    # a = regular unblocked cell with highway
    # b = hard to traverse with highway
    # default settings
    def __init__(self, is_blocked=False):
        self.is_blocked = is_blocked
        self.top = False
        self.bottom = False
        self.left = False
        self.right = False
        self.parent = None
        self.child = None
        self.h = 0
        self.g = 0
        self.f = 0


class Heap:
    def __init__(self):
        self


def manhattan_distance():
    return None


def compute_path():
    return None


def heuristic(self, goal, current, hType):
    (x1, y1) = goal
    (x2, y2) = current

    dx = abs(x1 - x2)
    dy = abs(y1 - y2)

    # diagnol distance, assuming highways
    if hType == 0:
        return .25 * (dx + dy) + (math.sqrt(2) - 2 * .25) * min(dx, dy)

    # straight line distance, assuming highways
    if hType == 1:
        return math.sqrt(dx ** 2 + dy ** 2) / 4

    # Manhattan distance, assuming highways
    if hType == 2:
        return .25 * (dx + dy)

    # straight line, assuming regular
    if hType == 3:
        return math.sqrt(dx ** 2 + dy ** 2)

    # Manhattan, assuming regular
    if hType == 4:
        return (dx + dy)

    # diagnol, assuming regular
    if hType == 5:
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
