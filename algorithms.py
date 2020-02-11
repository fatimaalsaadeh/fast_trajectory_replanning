import heapq
import sys

from utils import *

global adapt_counter
global ggoal
global reached

def neighbors(grid, gridInfo, start):
    (x, y) = (start.x, start.y)
    points = [(x - 1, y - 1), (x, y - 1), (x + 1, y - 1), (x - 1, y), (x + 1, y), (x - 1, y + 1), (x, y + 1),
              (x + 1, y + 1)]

    i = 0
    k = 8
    while i < k:
        # check point is in bounds and not blocked
        if points[i][0] < 0 and points[i][0] >= 101 and points[i][1] < 0 and points[i][1] >= 101:
           points.remove(points[i])
           i -= 1
           k -= 1
        else :
            if(not isinstance(gridInfo[points[i][1]][points[i][0]], Node)):
              gridInfo[points[i][1]][points[i][0]] = Node()

        i += 1

    # print points
    return points



def compute_path(goal, openSet, visited, grid, gridInfo, cost,path):
    while openSet:
        current = heapq.heappop(openSet)
        if current.x == goal.x and current.y == goal.y:
            reached = True
            break
        for next in neighbors(current):
            costCur = cost[current[1]] + manhattan_distance(grid, gridInfo, current[1], next)
            node = gridInfo[next[1]][next[0]]
            if next not in cost or costCur < cost[next]:
                cost[next] = costCur
                node.h = heuristic(goal, next, 4)
                node.g = costCur
                node.f = node.g + node.h
                heapq.heappush(openSet, (node.f, next))
                path[next] = current[1]
                visited.append(next)




def adaptastar(grid, gridInfo, start_x, start_y, goal_x, goal_y):
    start = gridInfo[start_x][start_y]
    goal =  gridInfo[goal_x][goal_y]
    neighbors(start, grid, gridInfo)

    path = {}
    cost = {}

    cost[start] = manhattan_distance(start, goal)
    path[start] = None
    reached = False

    # while not openSet[0] is None:
    while start.x != goal.x and start.y != goal.y:
        openSet = []
        visited = []
        adapt_counter+1
        start.g = 0
        start.h = cost[start]
        heapq.heappush(openSet, (0, start))
        visited.append(start)
        compute_path(goal, openSet, visited, grid, gridInfo,cost)
        current = heapq.heappop(openSet)
    curP = goal
    pathOrder = []
    while curP is not None and reached:
        x, y = curP
        pathOrder.insert(0, (x, y, gridInfo[y][x].h, gridInfo[y][x].g, gridInfo[y][x].f))
        curP = path[curP]
    if reached:
        return reached, pathOrder[::-1], cost[goal], len(cost)
    return reached, pathOrder[::-1], sys.maxsize, len(cost)



def repeated_forward_astar():
    return None


def repeated_backward_astar():
    return None

