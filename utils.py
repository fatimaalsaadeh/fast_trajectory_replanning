import math

import numpy as np
import matplotlib
import sys
import tkinter
import heapq
import math

from numpy.distutils.fcompiler import str2bool

from algorithms import *



BLACK = '#000000'
DARK_GRAY = '#494949'
LIGHT_GRAY = '#979899'
WHITE = '#fff'

class grid:
	### Metadata
	gridInfo = [] # numpy array
	# numpy array(s) holding the integer coordinates
	start = []
	goal = []
	centers = [] # 8 centers for hard to traverse

	### GUI variables
	numRows = 0
	numCols = 0
	sideLength = 15
	canvas = 0

	def _fillMetaData(self):
		self.numRows = len(self.gridInfo)
		self.numCols = len(self.gridInfo[0])
		for i in range(len(self.gridInfo)):
			for j in range(len(self.gridInfo[0])):
				node = self.gridInfo[i][j]

		for i in range(len(self.gridInfo)):
			for j in range(len(self.gridInfo[0])):
				node = self.gridInfo[i][j]
		self.gridInfo = np.asarray(self.gridInfo)

	def _drawCell(self, node, tag, r, c, border):
		color = DARK_GRAY
		# Background Color
		if((r == self.start[0] and c == self.start[1]) or (r == self.goal[0] and c == self.goal[1])):
			node.is_blocked=False
		if not node.is_blocked:
			color = WHITE
		if node.is_blocked:
			color = LIGHT_GRAY

		self.canvas.create_rectangle(
			r*self.sideLength, c*self.sideLength,
			(r+1)*self.sideLength, (c+1)*self.sideLength,
			outline=border, fill=color, tag=tag)

		cpt_x = (r*self.sideLength + (r+1)*self.sideLength)/2
		cpt_y = (c*self.sideLength + (c+1)*self.sideLength)/2
		# Mark Start and Goal Nodes
		if (r == self.goal[0] and c == self.goal[1]):
			self.canvas.create_oval(
				r*self.sideLength+2,c*self.sideLength+2,
				(r+1)*self.sideLength-2, (c+1)*self.sideLength-2,
				fill='red')
			self.gridInfo[r][c].is_blocked=False
		if (r == self.start[0] and c == self.start[1]):
			self.canvas.create_oval(
				r*self.sideLength+2,c*self.sideLength+2,
				(r+1)*self.sideLength-2, (c+1)*self.sideLength-2,
				fill='green')
			self.gridInfo[r][c].is_blocked=False


	def _drawButton(self, root, point):
		x = point[0]
		y = point[1]
		f = tkinter.Frame(root, height=self.sideLength+1, width=self.sideLength+1)
		f.pack_propagate(0)
		b = tkinter.Button(f, bg = "red", bd = 1, command = lambda: self._print(point))
		f.pack()
		b.pack()
		f.place(x = x*self.sideLength, y = y*self.sideLength)

	def _print(self, info):
		x, y = info[0], info[1]
		node = self.gridInfo[y][x]
		temp = ""
		if (x==self.start[0] and y ==self.start[1]):
			temp = temp + "Start:	"
		if (x==self.goal[0] and y ==self.goal[1]):
			temp = temp + "Goal:	"
		temp = temp + "x="+str(x)+" y="+str(y)
		print (temp)

	def createGridFromFile(self, file):
		# print "Extracting information from file..."
		fp = open(file, 'r')
		lines = fp.readlines()
		lines = [line.rstrip('\n') for line in lines]

		# print "Processing start and goal nodes..."
		start = np.asarray(lines[0].split())
		self.start = [int(a) for a in start]
		goal = np.asarray(lines[1].split())
		self.goal = [int(a) for a in goal]

		# print "Extracting grid data ..."
		temp = lines[2:]
		info = []
		[info.append(line.split()) for line in temp]
		for i in range(len(info)):
			gridRow = []
			for j in range (len(info[0])):
				x = Node( str2bool(info[i][j]))
				gridRow.append(x)
			self.gridInfo.append(gridRow)
		# print [[a.blockType for a in row] for row in self.gridInfo]
		self._fillMetaData()

	def createGUI(self, root):
		if self.numCols > 50 or self.numRows > 50:
			self.sideLength = 7
		self.width = self.numCols*self.sideLength
		self.height = self.numRows*self.sideLength
		self.canvas = tkinter.Canvas(root, width=self.width, height=self.height)
		root.wm_title("Map\n")
		for c in range(self.numRows):
			for r in range(self.numCols):
				node  = self.gridInfo[c][r]
				tag = str(r)+" "+str(c)
				self._drawCell(node, tag, r, c, BLACK)
		self.canvas.pack()

	def createButtonGUI(self, root, pathInfo):
		self.createGUI(root)
		for i in range(len(pathInfo)):
			self._drawButton(root, pathInfo[i])
		self.canvas.pack()


grid_o = grid()
grid_o.createGridFromFile("map0.txt")
root = tkinter.Tk()
alg = Algorithm()
startn= Node()
startn.x=grid_o.start[0]
startn.y=grid_o.start[1]
goaln= Node()
goaln.x=grid_o.goal[0]
goaln.y=grid_o.goal[1]
alg.start =  startn
alg.goal = goaln
alg.grid_info=grid_o.gridInfo

r, p, c, numExpanded = alg.repeated_astar(True, False, 1.25)
if r is not None:
	grid_o.createButtonGUI(root, p)
else:
	grid_o.createGUI(root)
root.mainloop()
grid_o.createGUI(root)
root.mainloop()
