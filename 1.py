#!/usr/bin/env python3
"""
A* vs HCS/CPL – Split‑Cell Comparison
Animation advances both algorithms by the same number of expansions per frame.
"""

import pygame
import heapq
import random
import math
import sys
from collections import deque, defaultdict

sys.setrecursionlimit(1000000)

# ---------- SPEED: expansions per frame (fair) ----------
STEPS_PER_FRAME = 1   # number of node expansions per frame for each side
FRAME_DELAY_MS = 600      # milliseconds between frames
MAP_SIZE = (150, 150)     # (cols, rows)

# Higher = faster animation, lower = slower.

# ---------- COLORS ----------
BG = (5, 5, 15)
WALL = (40, 45, 75)
EMPTY = (70, 80, 120)

A_STAR_EXPLORED = (200, 30, 30)    # deep red
A_STAR_FRONTIER = (255, 165, 0)    # orange
RIGHT_EXPLORED = (0, 255, 120)
RIGHT_FRONTIER = (180, 255, 180)
PATH_ASTAR = (100, 200, 255)
PATH_RIGHT = (100, 255, 100)
START_COL = (255, 255, 100)
GOAL_COL = (255, 80, 80)
CURRENT_NODE = (255, 255, 255)

GRID_LINE = (70, 80, 110)

BUTTON_BG = (30, 60, 120)
BUTTON_HOVER = (50, 100, 200)
BUTTON_RUN = (200, 60, 60)
BUTTON_RUN_HOVER = (240, 80, 80)
BUTTON_TOGGLE = (60, 120, 60)
BUTTON_TOGGLE_HOVER = (80, 160, 80)
BUTTON_MAP = (100, 60, 120)
BUTTON_MAP_HOVER = (140, 80, 160)
BUTTON_AUTO = (120, 100, 60)
BUTTON_AUTO_HOVER = (160, 140, 80)

TEXT_COL = (255, 255, 255)
STATS_COL = (150, 200, 255)

# ---------- CONFIG ----------
COLS = 150
ROWS = 150
BASE_CELL = 4
BUTTON_HEIGHT = 40
BUTTON_MARGIN = 8
STATUS_HEIGHT = 60

# ---------- MAP GENERATORS ----------
def generate_rooms(cols=COLS, rows=ROWS):
    grid = [[1 for _ in range(cols)] for _ in range(rows)]
    room_size = 10
    gap = 4
    room_centers = []

    for ry in range(gap, rows - room_size, room_size + gap):
        for rx in range(gap, cols - room_size, room_size + gap):
            for y in range(ry, ry + room_size):
                for x in range(rx, rx + room_size):
                    if 0 <= x < cols and 0 <= y < rows:
                        grid[y][x] = 0
            room_centers.append((rx + room_size//2, ry + room_size//2))

    rooms_per_row = (cols - gap) // (room_size + gap)
    for i, (cx, cy) in enumerate(room_centers):
        if (i + 1) % rooms_per_row != 0 and i + 1 < len(room_centers):
            nx, ny = room_centers[i + 1]
            for x in range(min(cx, nx), max(cx, nx) + 1):
                if 0 <= x < cols:
                    if 0 <= cy < rows: grid[cy][x] = 0
                    if 0 <= cy + 1 < rows: grid[cy + 1][x] = 0
        if i + rooms_per_row < len(room_centers):
            nx, ny = room_centers[i + rooms_per_row]
            for y in range(min(cy, ny), max(cy, ny) + 1):
                if 0 <= y < rows:
                    if 0 <= cx < cols: grid[y][cx] = 0
                    if 0 <= cx + 1 < cols: grid[y][cx + 1] = 0

    # Lakes inside rooms
    for _ in range(80):
        if room_centers:
            cx, cy = random.choice(room_centers)
            dx = random.randint(-room_size//2 + 1, room_size//2 - 1)
            dy = random.randint(-room_size//2 + 1, room_size//2 - 1)
            lx = cx + dx
            ly = cy + dy
            if 0 < lx < cols-1 and 0 < ly < rows-1 and grid[ly][lx] == 0:
                radius = random.randint(2, 4)
                for y in range(ly - radius, ly + radius + 1):
                    for x in range(lx - radius, lx + radius + 1):
                        if 0 <= x < cols and 0 <= y < rows:
                            if math.sqrt((x - lx)**2 + (y - ly)**2) < radius:
                                if grid[y][x] == 0:
                                    is_corridor = False
                                    for dx2, dy2 in [(-1,0),(1,0),(0,-1),(0,1)]:
                                        nx2, ny2 = x+dx2, y+dy2
                                        if 0 <= nx2 < cols and 0 <= ny2 < rows and grid[ny2][nx2] == 1:
                                            is_corridor = True
                                            break
                                    if not is_corridor:
                                        grid[y][x] = 1
    return grid

def generate_maze(cols=COLS, rows=ROWS):
    grid = [[1 for _ in range(cols)] for _ in range(rows)]
    stack = [(1, 1)]
    grid[1][1] = 0
    while stack:
        x, y = stack[-1]
        neighbors = []
        for dx, dy in [(2,0), (-2,0), (0,2), (0,-2)]:
            nx, ny = x + dx, y + dy
            if 0 < nx < cols-1 and 0 < ny < rows-1 and grid[ny][nx] == 1:
                neighbors.append((nx, ny, dx//2, dy//2))
        if neighbors:
            nx, ny, wx, wy = random.choice(neighbors)
            grid[ny][nx] = 0
            grid[y + wy][x + wx] = 0
            stack.append((nx, ny))
        else:
            stack.pop()
    return grid

def generate_random(cols=COLS, rows=ROWS):
    grid = [[0 for _ in range(cols)] for _ in range(rows)]
    for y in range(rows):
        for x in range(cols):
            if random.random() < 0.3:
                grid[y][x] = 1
    for y in range(rows):
        grid[y][0] = 1
        grid[y][cols-1] = 1
    for x in range(cols):
        grid[0][x] = 1
        grid[rows-1][x] = 1
    return grid

MAP_GENERATORS = {
    "Rooms": generate_rooms,
    "Maze": generate_maze,
    "Random": generate_random,
}
MAP_NAMES = list(MAP_GENERATORS.keys())

# ---------- AUTO PLACE START/GOAL ----------
def auto_place_start_goal(grid):
    cols = len(grid[0])
    rows = len(grid)
    free = [(x, y) for y in range(rows) for x in range(cols) if grid[y][x] == 0]
    if len(free) < 2:
        return None, None
    threshold = int(0.5 * min(cols, rows))
    for _ in range(100):
        s = random.choice(free)
        g = random.choice(free)
        if abs(s[0]-g[0]) + abs(s[1]-g[1]) >= threshold:
            return s, g
    sample = random.sample(free, min(50, len(free)))
    best_dist = 0
    best_pair = (free[0], free[-1])
    for i in range(len(sample)):
        for j in range(i+1, len(sample)):
            d = abs(sample[i][0]-sample[j][0]) + abs(sample[i][1]-sample[j][1])
            if d > best_dist:
                best_dist = d
                best_pair = (sample[i], sample[j])
    return best_pair

# ---------- A* ----------
def run_astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    def heuristic(x, y):
        return math.sqrt((x - goal[0])**2 + (y - goal[1])**2)

    open_heap = [(heuristic(start[0], start[1]), 0, start)]
    came_from = {}
    g_score = {start: 0.0}
    closed = set()
    frontier = {start}
    steps = []

    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        frontier.discard(current)
        steps.append((frontier.copy(), closed.copy(), current))
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, steps

        x, y = current
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < cols and 0 <= ny < rows and grid[ny][nx] == 0:
                cost = 1.0 if dx == 0 or dy == 0 else math.sqrt(2)
                neighbor = (nx, ny)
                tentative = g_score[current] + cost
                if neighbor not in g_score or tentative < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    h = heuristic(nx, ny)
                    heapq.heappush(open_heap, (tentative + h, tentative, neighbor))
                    frontier.add(neighbor)
    return [], steps

# ---------- HCS ----------
class SimpleHCS:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.bottlenecks = set()
        self.room_id = {}
        self.rooms = {}
        self.room_centroid = {}
        self.convex_rooms = set()
        self._decompose()

    def _decompose(self):
        H, W = self.rows, self.cols
        N = H * W
        disc = [-1] * N
        low = [0] * N
        parent = [-1] * N
        is_art = [False] * N
        time = 0

        def get_neighbors(x, y):
            nbrs = []
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x+dx, y+dy
                if 0 <= nx < W and 0 <= ny < H and self.grid[ny][nx] == 0:
                    nbrs.append((nx, ny))
            return nbrs

        def dfs(u):
            nonlocal time
            y, x = u // W, u % W
            disc[u] = low[u] = time
            time += 1
            children = 0
            for nx, ny in get_neighbors(x, y):
                v = ny * W + nx
                if disc[v] == -1:
                    parent[v] = u
                    children += 1
                    dfs(v)
                    low[u] = min(low[u], low[v])
                    if parent[u] != -1 and low[v] >= disc[u]:
                        is_art[u] = True
                elif v != parent[u]:
                    low[u] = min(low[u], disc[v])
            if parent[u] == -1 and children > 1:
                is_art[u] = True

        for y in range(H):
            for x in range(W):
                if self.grid[y][x] == 0:
                    u = y * W + x
                    if disc[u] == -1:
                        dfs(u)

        self.bottlenecks = {(x, y) for y in range(H) for x in range(W)
                            if is_art[y*W+x] and self.grid[y][x] == 0}

        visited = set()
        room_counter = 0
        for (x, y) in self.bottlenecks:
            self.room_id[(x, y)] = -1

        free_cells = {(x, y) for y in range(H) for x in range(W) if self.grid[y][x] == 0}
        for (x, y) in free_cells:
            if (x, y) not in self.bottlenecks and (x, y) not in visited:
                q = deque([(x, y)])
                visited.add((x, y))
                cells = {(x, y)}
                while q:
                    cx, cy = q.popleft()
                    for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                        nx, ny = cx+dx, cy+dy
                        if 0 <= nx < W and 0 <= ny < H and self.grid[ny][nx] == 0:
                            if (nx, ny) not in self.bottlenecks and (nx, ny) not in visited:
                                visited.add((nx, ny))
                                cells.add((nx, ny))
                                q.append((nx, ny))
                for cell in cells:
                    self.room_id[cell] = room_counter
                self.rooms[room_counter] = cells
                # Pre‑compute centroid
                sum_x = sum(x for x, y in cells)
                sum_y = sum(y for x, y in cells)
                self.room_centroid[room_counter] = (sum_x / len(cells), sum_y / len(cells))
                room_counter += 1

        for rid, cells in self.rooms.items():
            if not cells:
                continue
            min_x = min(x for x, y in cells)
            max_x = max(x for x, y in cells)
            min_y = min(y for x, y in cells)
            max_y = max(y for x, y in cells)
            if (max_x - min_x + 1) * (max_y - min_y + 1) != len(cells):
                continue
            convex = True
            for y in range(min_y, max_y+1):
                for x in range(min_x, max_x+1):
                    if self.grid[y][x] != 0 or self.room_id.get((x, y)) != rid:
                        convex = False
                        break
                if not convex:
                    break
            if convex:
                self.convex_rooms.add(rid)

    def run_hcs_search(self, start, goal):
        rows, cols = self.rows, self.cols

        room_to_bottlenecks = defaultdict(set)
        bottle_to_rooms = defaultdict(set)
        for (x, y) in self.bottlenecks:
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x+dx, y+dy
                if 0 <= nx < cols and 0 <= ny < rows and self.grid[ny][nx] == 0:
                    rid = self.room_id.get((nx, ny))
                    if rid is not None and rid >= 0:
                        room_to_bottlenecks[rid].add((x, y))
                        bottle_to_rooms[(x, y)].add(rid)

        start_room = self.room_id.get(start, -2)
        goal_room = self.room_id.get(goal, -2)
        if start_room < 0 or goal_room < 0:
            return self._fallback_astar(start, goal)

        if start_room == goal_room:
            if start_room in self.convex_rooms:
                return [start, goal], []
            else:
                return self._local_astar(start, goal, self.rooms[start_room])

        room_adj = defaultdict(dict)
        for rid, b_set in room_to_bottlenecks.items():
            for b in b_set:
                for other_rid in bottle_to_rooms.get(b, []):
                    if other_rid != rid:
                        room_adj[rid][other_rid] = 1
                        room_adj[other_rid][rid] = 1

        # Use pre‑computed centroids
        goal_centroid = self.room_centroid[goal_room]
        def room_heuristic(rid):
            cx1, cy1 = self.room_centroid[rid]
            return math.hypot(cx1 - goal_centroid[0], cy1 - goal_centroid[1])

        open_set = [(0, start_room)]
        came_from_room = {}
        g_room = {start_room: 0}
        closed_rooms = set()

        while open_set:
            g, current_rid = heapq.heappop(open_set)
            if current_rid in closed_rooms:
                continue
            closed_rooms.add(current_rid)
            if current_rid == goal_room:
                room_seq = []
                rid = goal_room
                while rid in came_from_room:
                    room_seq.append(rid)
                    rid = came_from_room[rid]
                room_seq.append(start_room)
                room_seq.reverse()
                break
            for nb, weight in room_adj.get(current_rid, {}).items():
                if nb not in g_room or g + weight < g_room[nb]:
                    g_room[nb] = g + weight
                    came_from_room[nb] = current_rid
                    heapq.heappush(open_set, (g + weight + room_heuristic(nb), nb))
        else:
            return self._fallback_astar(start, goal)

        allowed_cells = set()
        for rid in room_seq:
            allowed_cells.update(self.rooms[rid])
        for i in range(len(room_seq)-1):
            r1, r2 = room_seq[i], room_seq[i+1]
            b1 = room_to_bottlenecks.get(r1, set())
            b2 = room_to_bottlenecks.get(r2, set())
            common = b1 & b2
            if common:
                allowed_cells.update(common)
        return self._local_astar(start, goal, allowed_cells)

    def _local_astar(self, start, goal, allowed_cells):
        rows, cols = self.rows, self.cols
        def heuristic(x, y):
            return math.sqrt((x - goal[0])**2 + (y - goal[1])**2)

        open_heap = [(heuristic(start[0], start[1]), 0, start)]
        came_from = {}
        g_score = {start: 0.0}
        closed = set()
        frontier = {start}
        steps = []

        while open_heap:
            _, g, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)
            frontier.discard(current)
            steps.append((frontier.copy(), closed.copy(), current))
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path, steps

            x, y = current
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < cols and 0 <= ny < rows and (nx, ny) in allowed_cells and self.grid[ny][nx] == 0:
                    cost = 1.0 if dx == 0 or dy == 0 else math.sqrt(2)
                    neighbor = (nx, ny)
                    tentative = g_score[current] + cost
                    if neighbor not in g_score or tentative < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative
                        h = heuristic(nx, ny)
                        heapq.heappush(open_heap, (tentative + h, tentative, neighbor))
                        frontier.add(neighbor)
        return [], steps

    def _fallback_astar(self, start, goal):
        return run_astar(self.grid, start, goal)

# ---------- CPL ----------
class GridWorldSimple:
    def __init__(self, grid):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0])
        self.free_cells = {(x, y) for y in range(self.height) for x in range(self.width) if grid[y][x] == 0}

    def get_neighbors(self, x, y):
        neighbors = []
        W, H, g = self.width, self.height, self.grid
        if x > 0 and g[y][x-1] == 0: neighbors.append((x-1, y, 1.0))
        if x < W-1 and g[y][x+1] == 0: neighbors.append((x+1, y, 1.0))
        if y > 0 and g[y-1][x] == 0: neighbors.append((x, y-1, 1.0))
        if y < H-1 and g[y+1][x] == 0: neighbors.append((x, y+1, 1.0))
        if x > 0 and y > 0 and g[y-1][x-1] == 0 and g[y][x-1] == 0 and g[y-1][x] == 0:
            neighbors.append((x-1, y-1, math.sqrt(2)))
        if x < W-1 and y > 0 and g[y-1][x+1] == 0 and g[y][x+1] == 0 and g[y-1][x] == 0:
            neighbors.append((x+1, y-1, math.sqrt(2)))
        if x > 0 and y < H-1 and g[y+1][x-1] == 0 and g[y][x-1] == 0 and g[y+1][x] == 0:
            neighbors.append((x-1, y+1, math.sqrt(2)))
        if x < W-1 and y < H-1 and g[y+1][x+1] == 0 and g[y][x+1] == 0 and g[y+1][x] == 0:
            neighbors.append((x+1, y+1, math.sqrt(2)))
        return neighbors

class CPL:
    def __init__(self, grid):
        self.grid = grid
        self.world = GridWorldSimple(grid)
        self.rects = []
        self.cell_to_rect = {}
        self._decompose()
        self.n = len(self.rects)
        self.graph = defaultdict(list)
        self.portal_cells = set()
        self.rect_portals = defaultdict(list)
        self._build_graph()

    def _decompose(self):
        free = self.world.free_cells.copy()
        assigned = set()
        while free:
            seed = next(iter(free))
            x, y = seed
            x_left = x
            x_right = x
            while x_left - 1 >= 0 and (x_left-1, y) in free and (x_left-1, y) not in assigned:
                x_left -= 1
            while x_right + 1 < self.world.width and (x_right+1, y) in free and (x_right+1, y) not in assigned:
                x_right += 1
            y_top = y
            y_bottom = y
            while y_top - 1 >= 0:
                row_clear = True
                for x in range(x_left, x_right+1):
                    if (x, y_top-1) not in free or (x, y_top-1) in assigned:
                        row_clear = False
                        break
                if not row_clear:
                    break
                y_top -= 1
            while y_bottom + 1 < self.world.height:
                row_clear = True
                for x in range(x_left, x_right+1):
                    if (x, y_bottom+1) not in free or (x, y_bottom+1) in assigned:
                        row_clear = False
                        break
                if not row_clear:
                    break
                y_bottom += 1
            rect = (x_left, y_top, x_right, y_bottom)
            self.rects.append(rect)
            for yy in range(y_top, y_bottom+1):
                for xx in range(x_left, x_right+1):
                    self.cell_to_rect[(xx, yy)] = len(self.rects)-1
                    free.discard((xx, yy))
                    assigned.add((xx, yy))

    def _build_graph(self):
        # Identify portal cells
        for idx, (x1, y1, x2, y2) in enumerate(self.rects):
            for x in range(x1, x2+1):
                for y in (y1, y2):
                    cell = (x, y)
                    for nx, ny, _ in self.world.get_neighbors(x, y):
                        nidx = self.cell_to_rect.get((nx, ny))
                        if nidx is not None and nidx != idx:
                            self.portal_cells.add(cell)
                            break
            for y in range(y1, y2+1):
                for x in (x1, x2):
                    cell = (x, y)
                    for nx, ny, _ in self.world.get_neighbors(x, y):
                        nidx = self.cell_to_rect.get((nx, ny))
                        if nidx is not None and nidx != idx:
                            self.portal_cells.add(cell)
                            break

        for cell in self.portal_cells:
            self.rect_portals[self.cell_to_rect[cell]].append(cell)

        # Connect portals within a rectangle only if they share a row or column
        # (reduces edges while preserving connectivity)
        for idx, portals in self.rect_portals.items():
            for i in range(len(portals)):
                for j in range(i+1, len(portals)):
                    p1, p2 = portals[i], portals[j]
                    if p1[0] == p2[0] or p1[1] == p2[1]:
                        dist = math.hypot(p1[0]-p2[0], p1[1]-p2[1])
                        self.graph[p1].append((p2, dist))
                        self.graph[p2].append((p1, dist))

        # Adjacent cells across rectangles (zero cost)
        for y in range(self.world.height):
            for x in range(self.world.width):
                cell = (x, y)
                if cell not in self.cell_to_rect:
                    continue
                idx = self.cell_to_rect[cell]
                for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                    nx, ny = x+dx, y+dy
                    ncell = (nx, ny)
                    nidx = self.cell_to_rect.get(ncell)
                    if nidx is not None and nidx != idx:
                        self.graph[cell].append((ncell, 0.0))
                        self.graph[ncell].append((cell, 0.0))

        # Remove duplicate edges
        for u in self.graph:
            best = {}
            for v, w in self.graph[u]:
                if v not in best or w < best[v]:
                    best[v] = w
            self.graph[u] = [(v, w) for v, w in best.items()]

    def find_path(self, start, goal):
        if start not in self.cell_to_rect or goal not in self.cell_to_rect:
            return float('inf'), 0, [], []

        start_rect = self.cell_to_rect[start]
        goal_rect = self.cell_to_rect[goal]
        if start_rect == goal_rect:
            cost = math.hypot(start[0]-goal[0], start[1]-goal[1])
            return cost, 0, [start, goal], []

        g = defaultdict(list)
        for u, vlist in self.graph.items():
            g[u] = vlist.copy()
        start_node = ('start', start)
        goal_node = ('goal', goal)
        for p in self.rect_portals.get(start_rect, []):
            d = math.hypot(start[0]-p[0], start[1]-p[1])
            g[start_node].append((p, d))
            g[p].append((start_node, d))
        for p in self.rect_portals.get(goal_rect, []):
            d = math.hypot(goal[0]-p[0], goal[1]-p[1])
            g[goal_node].append((p, d))
            g[p].append((goal_node, d))

        def heuristic(node):
            if node == goal_node:
                return 0.0
            if isinstance(node, tuple) and node[0] == 'start':
                return math.hypot(start[0]-goal[0], start[1]-goal[1])
            return math.hypot(node[0]-goal[0], node[1]-goal[1])

        counter = 0
        open_heap = [(heuristic(start_node), 0.0, counter, start_node)]
        counter += 1
        g_score = {start_node: 0.0}
        came_from = {}
        expanded = set()
        expansions = 0
        steps = []
        expanded_rects = set()

        while open_heap:
            _, g_cost, _, u = heapq.heappop(open_heap)
            if u in expanded:
                continue
            expanded.add(u)
            expansions += 1

            if isinstance(u, tuple) and len(u)==2 and u in self.cell_to_rect:
                expanded_rects.add(self.cell_to_rect[u])

            # Frontier = actual cells in open heap
            frontier_cells = set()
            for _, _, _, node in open_heap:
                if isinstance(node, tuple) and len(node)==2 and node in self.cell_to_rect:
                    frontier_cells.add(node)

            # Explored = individual expanded cells + all cells from expanded rectangles
            expanded_cells = {node for node in expanded if isinstance(node, tuple) and len(node)==2 and node in self.cell_to_rect}
            rect_cells = set()
            for rid in expanded_rects:
                if rid < len(self.rects):
                    x1, y1, x2, y2 = self.rects[rid]
                    for y in range(y1, y2+1):
                        for x in range(x1, x2+1):
                            if self.grid[y][x] == 0:
                                rect_cells.add((x, y))
            expanded_cells = expanded_cells.union(rect_cells)

            current_cell = u if isinstance(u, tuple) and len(u)==2 and u in self.cell_to_rect else None
            steps.append((frontier_cells, expanded_cells, current_cell))

            if u == goal_node:
                path = []
                while u is not None:
                    if isinstance(u, tuple) and len(u)==2 and u[0] in ('start','goal'):
                        path.append(u[1])
                    else:
                        path.append(u)
                    u = came_from.get(u)
                path.reverse()
                return g_cost, expansions, path, steps

            for v, w in g.get(u, []):
                if v in expanded:
                    continue
                new_g = g_cost + w
                if v not in g_score or new_g < g_score[v]:
                    g_score[v] = new_g
                    came_from[v] = u
                    f = new_g + heuristic(v)
                    heapq.heappush(open_heap, (f, new_g, counter, v))
                    counter += 1

        return float('inf'), expansions, [], steps

# ---------- MAIN ----------
def main():
    pygame.init()

    cols, rows = COLS, ROWS
    info = pygame.display.Info()
    max_w = min(info.current_w, 1200)
    max_h = min(info.current_h, 900) - 120

    cell_w = max_w // (2 * cols)
    cell_h = max_h // rows
    cell = max(2, min(cell_w, cell_h, BASE_CELL))
    if cell < 2:
        cell = 2

    grid_w = 2 * cols * cell
    grid_h = rows * cell
    screen_h = grid_h + BUTTON_HEIGHT + 2 * BUTTON_MARGIN + STATUS_HEIGHT
    screen_w = grid_w

    screen = pygame.display.set_mode((screen_w, screen_h))
    pygame.display.set_caption("A* (left) vs Toggleable Right (HCS/CPL)")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 18)
    big_font = pygame.font.Font(None, 24)

    # ---------- State ----------
    map_idx = 0
    current_map_name = MAP_NAMES[map_idx]
    grid = MAP_GENERATORS[current_map_name](cols, rows)
    start, goal = auto_place_start_goal(grid)
    right_algo = "HCS"

    astar_path = []
    astar_steps = []
    right_path = []
    right_steps = []
    astar_idx = 0
    right_idx = 0
    astar_frontier = set()
    astar_explored = set()
    right_frontier = set()
    right_explored = set()
    animating = False
    running_search = False
    status = f"Map: {current_map_name} | Tap to place manually, or Auto Place"
    current_a = None
    current_r = None

    hcs_engine = SimpleHCS(grid)
    cpl_engine = CPL(grid)

    # Buttons
    run_btn = pygame.Rect(10, grid_h + BUTTON_MARGIN, 80, BUTTON_HEIGHT)
    reset_btn = pygame.Rect(100, grid_h + BUTTON_MARGIN, 80, BUTTON_HEIGHT)
    toggle_btn = pygame.Rect(190, grid_h + BUTTON_MARGIN, 130, BUTTON_HEIGHT)
    map_btn = pygame.Rect(330, grid_h + BUTTON_MARGIN, 120, BUTTON_HEIGHT)
    auto_btn = pygame.Rect(460, grid_h + BUTTON_MARGIN, 110, BUTTON_HEIGHT)
    quit_btn = pygame.Rect(screen_w - 100, grid_h + BUTTON_MARGIN, 90, BUTTON_HEIGHT)

    def reset_all():
        nonlocal grid, start, goal, astar_path, astar_steps, right_path, right_steps
        nonlocal astar_idx, right_idx, astar_frontier, astar_explored
        nonlocal right_frontier, right_explored, animating, running_search, status
        nonlocal current_a, current_r, hcs_engine, cpl_engine
        grid = MAP_GENERATORS[current_map_name](cols, rows)
        start, goal = auto_place_start_goal(grid)
        hcs_engine = SimpleHCS(grid)
        cpl_engine = CPL(grid)
        astar_path = []
        astar_steps = []
        right_path = []
        right_steps = []
        astar_idx = 0
        right_idx = 0
        astar_frontier = set()
        astar_explored = set()
        right_frontier = set()
        right_explored = set()
        animating = False
        running_search = False
        status = f"Map: {current_map_name} | Start/Goal auto-placed"
        current_a = None
        current_r = None

    def run_both():
        nonlocal astar_path, astar_steps, right_path, right_steps
        nonlocal astar_idx, right_idx, astar_frontier, astar_explored
        nonlocal right_frontier, right_explored, animating, running_search, status
        nonlocal current_a, current_r
        if start is None or goal is None:
            status = "Place both Start and Goal!"
            return
        if running_search or animating:
            return
        running_search = True
        status = f"Running A* vs {right_algo} on {current_map_name}..."

        astar_path, astar_steps = run_astar(grid, start, goal)

        if right_algo == "HCS":
            right_path, right_steps = hcs_engine.run_hcs_search(start, goal)
        else:
            cost, exp, right_path, right_steps = cpl_engine.find_path(start, goal)

        astar_idx = 0
        right_idx = 0
        astar_frontier = set()
        astar_explored = set()
        right_frontier = set()
        right_explored = set()
        current_a = None
        current_r = None
        animating = True
        running_search = False
        status = f"A*: {len(astar_steps)} steps | {right_algo}: {len(right_steps)} steps"

    reset_all()

    running = True
    while running:
        mouse_pos = pygame.mouse.get_pos()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                x, y = event.pos
                if run_btn.collidepoint(x, y):
                    run_both()
                elif reset_btn.collidepoint(x, y):
                    reset_all()
                elif toggle_btn.collidepoint(x, y):
                    right_algo = "CPL" if right_algo == "HCS" else "HCS"
                    status = f"Switched right to {right_algo}"
                    reset_all()
                elif map_btn.collidepoint(x, y):
                    map_idx = (map_idx + 1) % len(MAP_NAMES)
                    current_map_name = MAP_NAMES[map_idx]
                    reset_all()
                elif auto_btn.collidepoint(x, y):
                    s, g = auto_place_start_goal(grid)
                    if s and g:
                        start, goal = s, g
                        status = f"Auto-placed Start/Goal (dist ≥ 50%)"
                    else:
                        status = "Not enough free space!"
                elif quit_btn.collidepoint(x, y):
                    running = False
                else:
                    # Manual placement
                    if not animating and not running_search and y < grid_h:
                        col = x // (2 * cell)
                        row = y // cell
                        if 0 <= col < cols and 0 <= row < rows and grid[row][col] == 0:
                            pos = (col, row)
                            if start is None:
                                start = pos
                                status = "Start set! Tap for Goal (red)"
                            elif goal is None and pos != start:
                                goal = pos
                                status = "Ready! Press 'Run' to compare"
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    reset_all()
                elif event.key == pygame.K_SPACE:
                    run_both()
                elif event.key == pygame.K_ESCAPE:
                    running = False

        # ---------- Animation – fair: same number of expansions per frame ----------
        if animating:
            # Advance A* by STEPS_PER_FRAME expansions
            for _ in range(STEPS_PER_FRAME):
                if astar_idx < len(astar_steps):
                    astar_frontier, astar_explored, current_a = astar_steps[astar_idx]
                    astar_idx += 1
                else:
                    break

            # Advance right algorithm by STEPS_PER_FRAME expansions
            for _ in range(STEPS_PER_FRAME):
                if right_idx < len(right_steps):
                    right_frontier, right_explored, current_r = right_steps[right_idx]
                    right_idx += 1
                else:
                    break

            pygame.time.delay(50)   # ~20 fps

            if astar_idx >= len(astar_steps) and right_idx >= len(right_steps):
                animating = False
                status = f"Done! A* steps: {len(astar_steps)}, {right_algo} steps: {len(right_steps)}"

        # ---------- Drawing ----------
        screen.fill(BG)

        for y in range(rows):
            for x in range(cols):
                rect_l = pygame.Rect((x*2)*cell, y*cell, cell, cell)
                rect_r = pygame.Rect((x*2+1)*cell, y*cell, cell, cell)

                if grid[y][x] == 1:
                    col_l = WALL
                    col_r = WALL
                else:
                    col_l = EMPTY
                    col_r = EMPTY

                    if (x, y) in astar_explored:
                        col_l = A_STAR_EXPLORED
                    elif (x, y) in astar_frontier:
                        col_l = A_STAR_FRONTIER

                    if (x, y) in right_explored:
                        col_r = RIGHT_EXPLORED
                    elif (x, y) in right_frontier:
                        col_r = RIGHT_FRONTIER

                pygame.draw.rect(screen, col_l, rect_l)
                pygame.draw.rect(screen, col_r, rect_r)
                if cell > 3:
                    pygame.draw.rect(screen, GRID_LINE, rect_l, 1)
                    pygame.draw.rect(screen, GRID_LINE, rect_r, 1)

        # Paths (only after animation)
        if not animating:
            if astar_path:
                for (x, y) in astar_path:
                    if grid[y][x] == 0:
                        rect = pygame.Rect((x*2)*cell, y*cell, cell, cell)
                        pygame.draw.rect(screen, PATH_ASTAR, rect)
            if right_path:
                for (x, y) in right_path:
                    if grid[y][x] == 0:
                        rect = pygame.Rect((x*2+1)*cell, y*cell, cell, cell)
                        pygame.draw.rect(screen, PATH_RIGHT, rect)

        # Current node highlight
        if animating:
            if current_a:
                cx, cy = current_a
                rect = pygame.Rect((cx*2)*cell, cy*cell, cell, cell)
                pygame.draw.rect(screen, CURRENT_NODE, rect, 2)
            if current_r:
                cx, cy = current_r
                rect = pygame.Rect((cx*2+1)*cell, cy*cell, cell, cell)
                pygame.draw.rect(screen, CURRENT_NODE, rect, 2)

        # Start & Goal
        if start:
            sx, sy = start
            rect_l = pygame.Rect((sx*2)*cell, sy*cell, cell, cell)
            rect_r = pygame.Rect((sx*2+1)*cell, sy*cell, cell, cell)
            pygame.draw.rect(screen, START_COL, rect_l)
            pygame.draw.rect(screen, START_COL, rect_r)
        if goal:
            gx, gy = goal
            rect_l = pygame.Rect((gx*2)*cell, gy*cell, cell, cell)
            rect_r = pygame.Rect((gx*2+1)*cell, gy*cell, cell, cell)
            pygame.draw.rect(screen, GOAL_COL, rect_l)
            pygame.draw.rect(screen, GOAL_COL, rect_r)

        # Buttons
        def draw_btn(rect, text, color, hover):
            col = hover if rect.collidepoint(mouse_pos) else color
            pygame.draw.rect(screen, col, rect, border_radius=6)
            surf = font.render(text, True, TEXT_COL)
            screen.blit(surf, (rect.x + (rect.w - surf.get_width())//2, rect.y + 8))

        draw_btn(run_btn, "Run", BUTTON_RUN, BUTTON_RUN_HOVER)
        draw_btn(reset_btn, "Reset", BUTTON_BG, BUTTON_HOVER)
        draw_btn(toggle_btn, f"Right: {right_algo}", BUTTON_TOGGLE, BUTTON_TOGGLE_HOVER)
        draw_btn(map_btn, f"Map: {current_map_name}", BUTTON_MAP, BUTTON_MAP_HOVER)
        draw_btn(auto_btn, "Auto Place", BUTTON_AUTO, BUTTON_AUTO_HOVER)
        draw_btn(quit_btn, "Quit", (80, 40, 40), (120, 60, 60))

        # Status & Stats
        status_surf = big_font.render(status, True, STATS_COL)
        screen.blit(status_surf, (10, screen_h - 50))

        a_stats = f"A*: steps {len(astar_steps)}, explored {len(astar_explored)}, frontier {len(astar_frontier)}"
        r_stats = f"{right_algo}: steps {len(right_steps)}, explored {len(right_explored)}, frontier {len(right_frontier)}"
        surf_a = font.render(a_stats, True, (150, 200, 255))
        surf_r = font.render(r_stats, True, (150, 255, 150))
        screen.blit(surf_a, (10, screen_h - 30))
        screen.blit(surf_r, (10, screen_h - 15))

        # Legend
        legend = [
            ("A*E", A_STAR_EXPLORED),
            ("A*F", A_STAR_FRONTIER),
            ("R E", RIGHT_EXPLORED),
            ("R F", RIGHT_FRONTIER),
            ("A*P", PATH_ASTAR),
            ("R P", PATH_RIGHT),
        ]
        x_off = screen_w - 280
        for i, (label, col) in enumerate(legend):
            if x_off + i*70 < screen_w - 10:
                pygame.draw.rect(screen, col, (x_off + i*70, grid_h + 6, 14, 14))
                lbl = font.render(label, True, TEXT_COL)
                screen.blit(lbl, (x_off + i*70 + 18, grid_h + 4))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()