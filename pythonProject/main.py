import heapq

# 地图的宽度和高度
WIDTH = 10
HEIGHT = 10

# 此地图中0表示可通行，1表示障碍物，2表示起点，3表示终点
grid = [
    [0, 0, 0, 0, 1, 0, 3, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [2, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
]

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = 0  # 从起点到当前节点的实际代价
        self.h = 0  # 从当前节点到目标节点的预计代价
        self.f = 0  # f = g + h
        self.parent = None

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __lt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash((self.x, self.y))

# 启发式函数
def heuristic(current, goal):
    return abs(current.x - goal.x) + abs(current.y - goal.y)

# A*算法具体实现
def astar_search(start, goal):
    open_list = []
    closed_list = set()
    heapq.heappush(open_list, start)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node == goal:
            path = []
            while current_node is not None:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        closed_list.add(current_node)

        for x, y in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_x, new_y = current_node.x + x, current_node.y + y

            if 0 <= new_x < WIDTH and 0 <= new_y < HEIGHT and grid[new_x][new_y] != 1:
                neighbor = Node(new_x, new_y)
                neighbor.g = current_node.g + 1
                neighbor.h = heuristic(neighbor, goal)
                neighbor.f = neighbor.g + neighbor.h
                neighbor.parent = current_node

                if neighbor in closed_list:
                    continue

                if neighbor not in open_list or neighbor.f < neighbor.f:
                    heapq.heappush(open_list, neighbor)

    return None

# 找到起点和目标点的位置
start_pos = None
goal_pos = None
for i in range(HEIGHT):
    for j in range(WIDTH):
        if grid[i][j] == 2:
            start_pos = (i, j)
        elif grid[i][j] == 3:
            goal_pos = (i, j)

# 输出路径
start_node = Node(start_pos[0], start_pos[1])
goal_node = Node(goal_pos[0], goal_pos[1])
path = astar_search(start_node, goal_node)
print("路径是", path)