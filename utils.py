
class Node:
    def __init__(self, is_blocked=False, x=0, y=0):
        self.is_blocked = is_blocked
        self.parent = None
        self.next = None
        self.x = x
        self.y = y
        self.h = 0
        self.g = 0
        self.f = 0
        self.search = 0

    def __lt__(self, other):
        return self.f < other.f

    def add_child(self, node):
        temp = self.next
        self.next = node
        node.next = temp
