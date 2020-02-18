import heapq
import sys

from grid import *


def manhattan_distance(start, goal):
    return abs(start.x - goal.x) + abs(start.y - goal.y)


class Algorithm:
    def __init__(self, grid):
        self.grid = None
        self.grid_info = grid.gridArr
        self.counter = 0
        self.delta = []
        self.goal = grid.goal
        self.start = grid.start
        self.reached = None

    def neighbors(self, start, seen=False):
        neighborslist = []
        (x, y) = start
        points = [(x, y - 1), (x - 1, y), (x + 1, y), (x, y + 1)]
        i = 0
        k = 4
        while i < k:
            if points[i][0] < 0 or points[i][0] >= 101 or points[i][1] < 0 or points[i][1] >= 101:
                points.remove(points[i])
                i -= 1
                k -= 1
            else:
                position = self.grid_info[points[i][1]][points[i][0]]
                neighborslist.append(position)
                if(seen and position.is_blocked):
                    position.is_seen = True
            i += 1
        return points

    def initialize_node(self, cur_node, delta, counter):
        cur_node_o = self.grid_info[cur_node[1]][cur_node[0]]
        if (cur_node_o.search != self.counter and cur_node_o.search != 0):
            if (cur_node_o.g + cur_node_o.h < cur_node_o.g):
                cur_node_o.h = self.goal.g - cur_node_o.g
            cur_node_o.h = max(manhattan_distance(cur_node_o, self.goal), cur_node_o.h)
            cur_node_o.g = sys.maxsize
        elif (cur_node_o.search == 0):
            cur_node_o.h = manhattan_distance(cur_node_o, self.goal)
            cur_node_o.g = sys.maxsize
        cur_node_o.search = counter
        self.grid_info[cur_node[1]][cur_node[0]] = cur_node_o

    def move(self, path_end, start_node, goal_node, isBackward=False):
        if not isBackward:
            while path_end.parent != start_node and path_end != self.goal:
                path_end = path_end.parent
            start_node = path_end
        else:
            path_prev = goal_node.parent
            while path_prev.parent != None and path_end != self.start:
                path_prev = path_prev.parent

        while self.start != None:
            prev = start_node
            cur = self.grid[self.start.x][self.start.y]
            if cur.is_blocked:
                cur = prev
                self.start = self.grid[cur.x][cur.y]
                break
            if cur == goal_node:
                self.start = self.grid[cur.x][cur.y]
                cur.parent = prev
                break
            if cur != start_node and prev != cur and cur.parent == None:
                cur.parent = prev
            self.start = self.start.next
        return cur

    def adaptive_astar(self, tie_break):
        start = (self.start.x, self.start.y)
        goal = (self.goal.x, self.goal.y)
        open_set = []
        closed_set = []
        path = {}
        cost = {}

        counter = 0
        delta = []
        delta.append(counter)

        heapq.heappush(open_set, (0, start))
        path[start] = None
        cost[start] = 0
        reached = False

        self.grid_info[start[1]][start[0]].g = 0
        self.grid_info[start[1]][start[0]].h = manhattan_distance(self.grid_info[start[1]][start[0]], self.grid_info[goal[1]][goal[0]])
        self.grid_info[start[1]][start[0]].f = self.grid_info[start[1]][start[0]].g + self.grid_info[start[1]][start[0]].h

        current = heapq.heappop(open_set)[1]
        while current != goal:
            self.initialize_node(current, delta, counter)
            self.initialize_node(goal, delta, counter)
            self.grid_info[current[1]][current[0]].g = 0
            for next in self.neighbors(current):
                cur_cost = cost[current] + (sys.maxsize if self.grid_info[next[1]][next[0]].is_blocked else 1)
                node = self.grid_info[next[1]][next[0]]
                if next not in cost or cur_cost < cost[next]:
                    cost[next] = cur_cost
                    if tie_break != 0:
                        node.h = manhattan_distance(self.grid_info[goal[1]][goal[0]], self.grid_info[next[1]][next[0]])
                    node.g = cur_cost
                    node.f = node.g + tie_break * node.h
                    heapq.heappush(open_set, (node.f, next))
                    path[next] = current
            prev = current
            current = heapq.heappop(open_set)[1]
            if current == goal:
                reached = True
                break
            else:
                self.initialize_node(current, delta, counter)
                if self.grid_info[current[1]][current[0]].g + self.grid_info[current[1]][current[0]].h < self.grid_info[prev[1]][prev[0]].g:
                    self.grid_info[current[1]][current[0]].h = self.grid_info[goal[1]][goal[0]].g - self.grid_info[current[1]][current[0]].g
        cur_p = goal
        pathOrder = []
        while cur_p is not None and reached:
            x, y = cur_p
            pathOrder.insert(0, (x, y, self.grid_info[y][x].h, self.grid_info[y][x].g, self.grid_info[y][x].f))
            cur_p = path[cur_p]
        if reached:
            return reached, pathOrder[::-1], cost[goal], len(cost)
        return reached, pathOrder[::-1], sys.maxsize, len(cost)

    def repeated_astar(self, is_forward, is_backward, tie_break):
        start = (self.start.x, self.start.y)
        goal = (self.goal.x, self.goal.y)
        open_set = []
        closed_set = []
        path = {}
        cost = {}
        counter = 0
        delta = []
        delta.append(counter)
        heapq.heappush(open_set, (0, start))
        path[start] = None
        cost[start] = 0
        reached = False
        if (is_forward):
            self.grid_info[start[1]][start[0]].g = 0
            self.grid_info[start[1]][start[0]].h = manhattan_distance(self.grid_info[start[1]][start[0]], self.grid_info[goal[1]][goal[0]])
            self.grid_info[start[1]][start[0]].f = self.grid_info[start[1]][start[0]].g + self.grid_info[start[1]][start[0]].h
            current = heapq.heappop(open_set)[1]
            self.neighbors(current, True)
            while current != goal:
                counter+=1
                self.grid_info[current[1]][current[0]].search = counter
                self.grid_info[current[1]][current[0]].g = 0
                self.grid_info[current[1]][current[0]].h = manhattan_distance(self.grid_info[start[1]][start[0]], self.grid_info[goal[1]][goal[0]])
                self.grid_info[current[1]][current[0]].f = self.grid_info[current[1]][current[0]].h + self.grid_info[current[1]][current[0]].g
                self.grid_info[goal[1]][goal[0]].g = sys.maxsize
                self.grid_info[goal[1]][goal[0]].search = counter
                open_set = []
                closed_set = []
                heapq.heappush(open_set, (self.grid_info[current[1]][current[0]].f, current))
                while self.grid_info[goal[1]][goal[0]].g >open_set[0][0]:
                    current = heapq.heappop(open_set)[1]
                    closed_set.append(current)
                    for next in self.neighbors(current):
                        node = self.grid_info[next[1]][next[0]]
                        if(node.search < counter):
                            node.g = sys.maxsize
                            node.search = counter
                        costCur = cost[current] + (
                            sys.maxsize if self.grid_info[next[1]][next[0]].is_seen else 1)
                        if next not in cost or costCur < cost[next]:
                            cost[next] = costCur
                            if tie_break != 0:
                                node.h = manhattan_distance(self.grid_info[goal[1]][goal[0]],
                                                                 self.grid_info[next[1]][next[0]])
                            node.g = costCur
                            node.f = node.g + tie_break * node.h
                            heapq.heappush(open_set, (node.f, next))
                            path[next] = current[1]
                if not open_set:
                    print("blocked target")
                    reached = False
                    break
            cur_p = goal
            pathOrder = []
            while cur_p is not None and reached:
                x, y = cur_p
                pathOrder.insert(0, (x, y, self.grid_info[y][x].h, self.grid_info[y][x].g, self.grid_info[y][x].f))
                cur_p = path[cur_p]
            if reached:
                return reached, pathOrder[::-1], cost[goal], len(cost)
            return reached, pathOrder[::-1], sys.maxsize, len(cost)
        elif (is_backward):
            start_node.g = 0
            start_node.h = manhattan_distance(start_node, goal_node)
            start_node.f = start_node.g + start_node.h
            current_node = self.start
            while current_node != goal_node:
                self.open_set = []
                self.counter = self.counter + 1
                current_node.g = 0
                goal_node.search = self.counter
                goal_node.g = sys.maxsize
                heapq.heappush(self.open_set, current_node)
                self.compute_path(1.5)
                if not self.open_set:
                    print("blocked target")
                    reached = False
                    break
                path_end = heapq.heappop(self.open_set)
                cur = self.move(path_end, current_node, goal_node)
                if cur is None:
                    break
                current_node = cur
            if reached:
                return current_node



def main():
    grid_o = Grid()
    root = tk.Tk()
    alg = Algorithm(grid_o)

    r, p, c, num_expanded = alg.adaptive_astar(1.25)
    if r is not None:
        grid_o.display_path(root, p)
    else:
        grid_o.create_maze(root)
    root.mainloop()

if __name__ == "__main__":
    main()
