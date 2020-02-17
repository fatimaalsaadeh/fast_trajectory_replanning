import math

import numpy as np
import matplotlib
import sys
import tkinter
import heapq
import math

from numpy.distutils.fcompiler import str2bool

from algorithms import *

class Node:
    # 0 = blocked, 1 = regular unblocked, 2 = hard to traverse
    # a = regular unblocked cell with highway
    # b = hard to traverse with highway
    # default settings
    def __init__(self, is_blocked=False):
        self.blocked = False
        '''
        self.top = False
        self.bottom = False
        self.left = False
        self.right = False
        '''
        self.parent = None
        self.child = None
        self.x = 0
        self.y = 0
        self.h = 0
        self.g = 0
        self.f = 0
