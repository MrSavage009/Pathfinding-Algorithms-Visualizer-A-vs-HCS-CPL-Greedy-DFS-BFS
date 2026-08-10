#!/usr/bin/env python3
"""
Blind Exploration: Greedy DFS (left) vs Greedy BFS (right)
Tie‑breaker by discovery order: LIFO for DFS, FIFO for BFS.
Discrete speed options: 100,75,50,30,20,10 ms.
"""

import pygame
import heapq
import random
import math
import sys
from collections import deque, defaultdict

sys.setrecursionlimit(1000000)

# ---------- CONFIG ----------
STEPS_PER_FRAME = 1
SPEED_OPTIONS = [100, 75, 50, 30, 20, 10]
DEFAULT_SPEED_INDEX = 0
FRAME_DELAY_MS = SPEED_OPTIONS[DEFAULT_SPEED_INDEX]

COLS = 50
ROWS = 50
BASE_CELL = 6
BUTTON_HEIGHT = 40
BUTTON_MARGIN = 8
STATUS_HEIGHT = 60

# ---------- COLORS ----------
BG = (5, 5, 15)
WALL = (40, 45, 75)
CELL_BORDER = (80, 90, 110)

LEFT_EMPTY = (80, 100, 160)
LEFT_EXPLORED = (0, 150, 255)
LEFT_FRONTIER = (150, 220, 255)
LEFT_AGENT = (255, 255, 100)

RIGHT_EMPTY = (60, 40, 40)
RIGHT_EXPLORED = (200, 30, 30)
RIGHT_FRONTIER = (255, 165, 0)
RIGHT_AGENT = (255, 100, 255)

START_COL = (255, 255, 100)
GOAL_COL = (255, 80, 80)

OUTLINE_COL = (120, 140, 180)

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
BUTTON_SPEED = (60, 60, 120)
BUTTON_SPEED_HOVER = (80, 80, 160)

TEXT_COL = (255, 255, 255)
STATS_COL = (150, 200, 255)

# ---------- MAP GENERATORS (unchanged) ----------
def generate_rooms(cols=COLS, rows=ROWS):
    grid = [[1 for _ in range(cols)] for _ in range(rows)]
    room_size = 6
    gap = 3
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

# ---------- Blind Explorer with greedy + tie‑breaker ----------
class BlindExplorer:
    def __init__(self, grid, start, goal, strategy):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.start = start
        self.goal = goal
        self.pos = start
        self.visited = set([start])
        self.total_cost = 0
        self.steps = 0
        self.finished = False
        self.frontier = set()
        self._update_frontier()
        self.strategy = strategy          # 'dfs' or 'bfs'
        self.parent = {}
        self.parent[start] = None
        self.discovery_time = {}          # time when each cell was first discovered
        self.discovery_time[start] = 0
        self._time_counter = 1

        # For discovery order (only used to add neighbours, not for target selection)
        self.dfs_stack = [start]
        self.bfs_queue = deque([start])
        self.discovered = set([start])

    def _discover_neighbours(self, pos):
        x, y = pos
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < self.cols and 0 <= ny < self.rows and self.grid[ny][nx] == 0:
                nxt = (nx, ny)
                if nxt not in self.discovered:
                    self.discovered.add(nxt)
                    self.parent[nxt] = pos
                    self.discovery_time[nxt] = self._time_counter
                    self._time_counter += 1
                    if self.strategy == 'dfs':
                        self.dfs_stack.append(nxt)
                    elif self.strategy == 'bfs':
                        self.bfs_queue.append(nxt)

    def _update_frontier(self):
        self.frontier = set()
        for (x,y) in self.visited:
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x+dx, y+dy
                if 0 <= nx < self.cols and 0 <= ny < self.rows and self.grid[ny][nx] == 0:
                    if (nx, ny) not in self.visited:
                        self.frontier.add((nx, ny))

    def _dist_to_all_visited(self):
        dist = {self.pos: 0}
        q = deque([self.pos])
        while q:
            cur = q.popleft()
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = cur[0]+dx, cur[1]+dy
                nxt = (nx, ny)
                if nxt in self.visited and nxt not in dist:
                    dist[nxt] = dist[cur] + 1
                    q.append(nxt)
        return dist

    def _pick_target(self):
        if not self.frontier:
            return None
        dist_visited = self._dist_to_all_visited()

        # For each frontier, compute its distance from current position
        candidates = []   # list of (distance, discovery_time, cell)
        for cell in self.frontier:
            x, y = cell
            min_d = float('inf')
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x+dx, y+dy
                nb = (nx, ny)
                if nb in self.visited and nb in dist_visited:
                    d = dist_visited[nb] + 1
                    if d < min_d:
                        min_d = d
            if min_d < float('inf'):
                candidates.append((min_d, self.discovery_time.get(cell, 0), cell))

        if not candidates:
            return None

        # Sort by distance, then by discovery_time
        # For DFS: use reverse order (most recent first)
        # For BFS: use normal order (oldest first)
        if self.strategy == 'dfs':
            candidates.sort(key=lambda t: (t[0], -t[1]))   # smallest distance, then largest time
        else:  # bfs
            candidates.sort(key=lambda t: (t[0], t[1]))    # smallest distance, then smallest time

        return candidates[0][2]

    def step(self):
        if self.finished:
            return
        if self.pos == self.goal:
            self.finished = True
            return

        self._discover_neighbours(self.pos)
        target = self._pick_target()
        if target is None:
            self.finished = True
            return

        if abs(target[0] - self.pos[0]) + abs(target[1] - self.pos[1]) == 1:
            next_cell = target
        else:
            parent = self.parent.get(target)
            if parent is None:
                self.finished = True
                return
            path = self._shortest_path(self.pos, parent)
            if path and len(path) > 1:
                next_cell = path[1]
            else:
                next_cell = parent

        self.pos = next_cell
        if next_cell not in self.visited:
            self.visited.add(next_cell)
        self.total_cost += 1.0
        self.steps += 1
        self._update_frontier()

        if self.strategy == 'dfs':
            # update stack to reflect current path (not strictly needed for target selection)
            if next_cell not in self.dfs_stack:
                self.dfs_stack.append(next_cell)
            else:
                while self.dfs_stack and self.dfs_stack[-1] != next_cell:
                    self.dfs_stack.pop()

        if self.pos == self.goal:
            self.finished = True

    def _shortest_path(self, start, target):
        if start == target:
            return [start]
        visited_set = set()
        q = deque()
        q.append((start, [start]))
        while q:
            node, path = q.popleft()
            if node == target:
                return path
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = node[0]+dx, node[1]+dy
                if 0 <= nx < self.cols and 0 <= ny < self.rows and self.grid[ny][nx] == 0:
                    nxt = (nx, ny)
                    if nxt in self.visited and nxt not in visited_set:
                        visited_set.add(nxt)
                        q.append((nxt, path + [nxt]))
        return None

    def get_state(self):
        return self.frontier.copy(), self.visited.copy(), self.pos

# ---------- MAIN ----------
def main():
    global FRAME_DELAY_MS

    pygame.init()
    info = pygame.display.Info()
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    pygame.display.set_caption("Greedy DFS vs Greedy BFS (tie‑breaker)")
    screen_width, screen_height = screen.get_size()

    bottom_height = BUTTON_HEIGHT + 2 * BUTTON_MARGIN + STATUS_HEIGHT
    available_h = screen_height - bottom_height
    available_w = screen_width

    cell = min(available_w // COLS, available_h // ROWS)
    cell = max(4, cell)

    grid_w = COLS * cell
    grid_h = ROWS * cell
    offset_x = (screen_width - grid_w) // 2
    offset_y = 0

    button_y = grid_h + BUTTON_MARGIN

    font = pygame.font.Font(None, max(18, int(18 * screen_width / 1280)))
    big_font = pygame.font.Font(None, max(24, int(24 * screen_width / 1280)))

    clock = pygame.time.Clock()

    # ---------- State ----------
    map_idx = 0
    current_map_name = MAP_NAMES[map_idx]
    grid = MAP_GENERATORS[current_map_name](COLS, ROWS)
    start, goal = auto_place_start_goal(grid)

    left_agent = BlindExplorer(grid, start, goal, 'dfs')
    right_agent = BlindExplorer(grid, start, goal, 'bfs')
    right_strategy = "BFS"

    animating = False
    running_search = False
    status = f"Map: {current_map_name} | Delay: {FRAME_DELAY_MS} ms"
    left_frontier, left_visited, left_pos = left_agent.get_state()
    right_frontier, right_visited, right_pos = right_agent.get_state()

    speed_index = SPEED_OPTIONS.index(FRAME_DELAY_MS) if FRAME_DELAY_MS in SPEED_OPTIONS else 0

    # ---------- Buttons ----------
    def btn_width(text):
        return max(60, font.size(text)[0] + 20)

    btn_run_w = btn_width("Run")
    btn_reset_w = btn_width("Reset")
    btn_toggle_w = btn_width("Right: BFS")
    btn_map_w = btn_width("Map: Rooms")
    btn_auto_w = btn_width("Auto Place")
    btn_plus_w = 35
    btn_minus_w = 35
    btn_quit_w = btn_width("Quit")

    gap = 8
    x = offset_x
    run_btn = pygame.Rect(x, button_y, btn_run_w, BUTTON_HEIGHT)
    x += btn_run_w + gap
    reset_btn = pygame.Rect(x, button_y, btn_reset_w, BUTTON_HEIGHT)
    x += btn_reset_w + gap
    toggle_btn = pygame.Rect(x, button_y, btn_toggle_w, BUTTON_HEIGHT)
    x += btn_toggle_w + gap
    map_btn = pygame.Rect(x, button_y, btn_map_w, BUTTON_HEIGHT)
    x += btn_map_w + gap
    auto_btn = pygame.Rect(x, button_y, btn_auto_w, BUTTON_HEIGHT)
    x += btn_auto_w + gap
    speed_minus_btn = pygame.Rect(x, button_y, btn_minus_w, BUTTON_HEIGHT)
    x += btn_minus_w + gap
    speed_plus_btn = pygame.Rect(x, button_y, btn_plus_w, BUTTON_HEIGHT)
    x += btn_plus_w + gap
    quit_btn = pygame.Rect(screen_width - btn_quit_w - offset_x, button_y, btn_quit_w, BUTTON_HEIGHT)

    def reset_all():
        nonlocal grid, start, goal, left_agent, right_agent
        nonlocal left_frontier, left_visited, left_pos
        nonlocal right_frontier, right_visited, right_pos
        nonlocal animating, running_search, status
        grid = MAP_GENERATORS[current_map_name](COLS, ROWS)
        start, goal = auto_place_start_goal(grid)
        left_agent = BlindExplorer(grid, start, goal, 'dfs')
        right_agent = BlindExplorer(grid, start, goal, 'bfs' if right_strategy == "BFS" else 'random')
        left_frontier, left_visited, left_pos = left_agent.get_state()
        right_frontier, right_visited, right_pos = right_agent.get_state()
        animating = False
        running_search = False
        status = f"Map: {current_map_name} | Delay: {FRAME_DELAY_MS} ms"

    def run_both():
        nonlocal animating, running_search, status
        if start is None or goal is None:
            status = "Place both Start and Goal!"
            return
        if running_search or animating:
            return
        left_agent.__init__(grid, start, goal, 'dfs')
        right_agent.__init__(grid, start, goal, 'bfs' if right_strategy == "BFS" else 'random')
        left_frontier, left_visited, left_pos = left_agent.get_state()
        right_frontier, right_visited, right_pos = right_agent.get_state()
        animating = True
        running_search = False
        status = f"Exploring... (delay: {FRAME_DELAY_MS} ms)"

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
                    right_strategy = "Random" if right_strategy == "BFS" else "BFS"
                    status = f"Right strategy: {right_strategy} | Delay: {FRAME_DELAY_MS} ms"
                    reset_all()
                elif map_btn.collidepoint(x, y):
                    map_idx = (map_idx + 1) % len(MAP_NAMES)
                    current_map_name = MAP_NAMES[map_idx]
                    reset_all()
                elif auto_btn.collidepoint(x, y):
                    s, g = auto_place_start_goal(grid)
                    if s and g:
                        start, goal = s, g
                        status = f"Auto-placed Start/Goal | Delay: {FRAME_DELAY_MS} ms"
                        reset_all()
                    else:
                        status = "Not enough free space!"
                elif speed_minus_btn.collidepoint(x, y):
                    speed_index = (speed_index + 1) % len(SPEED_OPTIONS)
                    FRAME_DELAY_MS = SPEED_OPTIONS[speed_index]
                    status = f"Delay: {FRAME_DELAY_MS} ms"
                elif speed_plus_btn.collidepoint(x, y):
                    speed_index = (speed_index - 1) % len(SPEED_OPTIONS)
                    FRAME_DELAY_MS = SPEED_OPTIONS[speed_index]
                    status = f"Delay: {FRAME_DELAY_MS} ms"
                elif quit_btn.collidepoint(x, y):
                    running = False
                else:
                    if not animating and not running_search:
                        gx = (x - offset_x) // cell
                        gy = y // cell
                        if 0 <= gx < COLS and 0 <= gy < ROWS:
                            if grid[gy][gx] == 0:
                                pos = (gx, gy)
                                if start is None:
                                    start = pos
                                    status = f"Start set! Tap for Goal | Delay: {FRAME_DELAY_MS} ms"
                                elif goal is None and pos != start:
                                    goal = pos
                                    status = f"Ready! Press 'Run' | Delay: {FRAME_DELAY_MS} ms"
                                    reset_all()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    reset_all()
                elif event.key == pygame.K_SPACE:
                    run_both()
                elif event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    speed_index = (speed_index - 1) % len(SPEED_OPTIONS)
                    FRAME_DELAY_MS = SPEED_OPTIONS[speed_index]
                    status = f"Delay: {FRAME_DELAY_MS} ms"
                elif event.key == pygame.K_MINUS:
                    speed_index = (speed_index + 1) % len(SPEED_OPTIONS)
                    FRAME_DELAY_MS = SPEED_OPTIONS[speed_index]
                    status = f"Delay: {FRAME_DELAY_MS} ms"

        # ---------- Animation ----------
        if animating:
            if not left_agent.finished:
                left_agent.step()
            if not right_agent.finished:
                right_agent.step()

            left_frontier, left_visited, left_pos = left_agent.get_state()
            right_frontier, right_visited, right_pos = right_agent.get_state()

            pygame.time.delay(FRAME_DELAY_MS)

            if left_agent.finished and right_agent.finished:
                animating = False
                status = (f"Done! Left (DFS): cost {left_agent.total_cost:.1f}, steps {left_agent.steps} | "
                          f"Right ({right_strategy}): cost {right_agent.total_cost:.1f}, steps {right_agent.steps} | "
                          f"Delay: {FRAME_DELAY_MS} ms")

        # ---------- Drawing ----------
        screen.fill(BG)

        half = cell // 2

        for y in range(ROWS):
            for x in range(COLS):
                cx = offset_x + x * cell
                cy = y * cell

                rect_l = pygame.Rect(cx, cy, half, cell)
                rect_r = pygame.Rect(cx + half, cy, cell - half, cell)

                if grid[y][x] == 1:
                    col_l = WALL
                    col_r = WALL
                else:
                    col_l = LEFT_EMPTY
                    if (x, y) in left_visited:
                        col_l = LEFT_EXPLORED
                    elif (x, y) in left_frontier:
                        col_l = LEFT_FRONTIER

                    col_r = RIGHT_EMPTY
                    if (x, y) in right_visited:
                        col_r = RIGHT_EXPLORED
                    elif (x, y) in right_frontier:
                        col_r = RIGHT_FRONTIER

                pygame.draw.rect(screen, col_l, rect_l)
                pygame.draw.rect(screen, col_r, rect_r)

                cell_rect = pygame.Rect(cx, cy, cell, cell)
                pygame.draw.rect(screen, CELL_BORDER, cell_rect, 1)

        pygame.draw.rect(screen, OUTLINE_COL, (offset_x, 0, grid_w, grid_h), 2)

        # Agents
        if left_pos:
            lx, ly = left_pos
            rect = pygame.Rect(offset_x + lx * cell, ly * cell, half, cell)
            pygame.draw.rect(screen, LEFT_AGENT, rect)
            pygame.draw.rect(screen, (255, 255, 255), rect, 1)
        if right_pos:
            rx, ry = right_pos
            rect = pygame.Rect(offset_x + rx * cell + half, ry * cell, cell - half, cell)
            pygame.draw.rect(screen, RIGHT_AGENT, rect)
            pygame.draw.rect(screen, (255, 255, 255), rect, 1)

        # Start & Goal
        if start:
            sx, sy = start
            rect_l = pygame.Rect(offset_x + sx * cell, sy * cell, half, cell)
            rect_r = pygame.Rect(offset_x + sx * cell + half, sy * cell, cell - half, cell)
            pygame.draw.rect(screen, START_COL, rect_l)
            pygame.draw.rect(screen, START_COL, rect_r)
            pygame.draw.rect(screen, (255, 255, 255), rect_l, 1)
            pygame.draw.rect(screen, (255, 255, 255), rect_r, 1)
        if goal:
            gx, gy = goal
            rect_l = pygame.Rect(offset_x + gx * cell, gy * cell, half, cell)
            rect_r = pygame.Rect(offset_x + gx * cell + half, gy * cell, cell - half, cell)
            pygame.draw.rect(screen, GOAL_COL, rect_l)
            pygame.draw.rect(screen, GOAL_COL, rect_r)
            pygame.draw.rect(screen, (255, 255, 255), rect_l, 1)
            pygame.draw.rect(screen, (255, 255, 255), rect_r, 1)

        # ---------- Buttons ----------
        def draw_btn(rect, text, color, hover):
            col = hover if rect.collidepoint(mouse_pos) else color
            pygame.draw.rect(screen, col, rect, border_radius=6)
            surf = font.render(text, True, TEXT_COL)
            screen.blit(surf, (rect.x + (rect.w - surf.get_width())//2, rect.y + 8))

        draw_btn(run_btn, "Run", BUTTON_RUN, BUTTON_RUN_HOVER)
        draw_btn(reset_btn, "Reset", BUTTON_BG, BUTTON_HOVER)
        draw_btn(toggle_btn, f"Right: {right_strategy}", BUTTON_TOGGLE, BUTTON_TOGGLE_HOVER)
        draw_btn(map_btn, f"Map: {current_map_name}", BUTTON_MAP, BUTTON_MAP_HOVER)
        draw_btn(auto_btn, "Auto Place", BUTTON_AUTO, BUTTON_AUTO_HOVER)
        draw_btn(speed_minus_btn, "-", BUTTON_SPEED, BUTTON_SPEED_HOVER)
        draw_btn(speed_plus_btn, "+", BUTTON_SPEED, BUTTON_SPEED_HOVER)
        draw_btn(quit_btn, "Quit", (80, 40, 40), (120, 60, 60))

        speed_text = font.render(f"{FRAME_DELAY_MS} ms", True, TEXT_COL)
        screen.blit(speed_text, (speed_plus_btn.x + speed_plus_btn.w + 8, button_y + 10))

        # Status & Stats
        status_y = screen_height - STATUS_HEIGHT - 10
        status_surf = big_font.render(status, True, STATS_COL)
        screen.blit(status_surf, (10, status_y))

        left_stats = f"Greedy DFS: cost {left_agent.total_cost:.1f}, steps {left_agent.steps}"
        right_stats = f"{right_strategy}: cost {right_agent.total_cost:.1f}, steps {right_agent.steps}"
        surf_l = font.render(left_stats, True, (150, 200, 255))
        surf_r = font.render(right_stats, True, (255, 150, 150))
        screen.blit(surf_l, (10, status_y + 25))
        screen.blit(surf_r, (10, status_y + 45))

        # Legend
        legend = [
            ("L Vis", LEFT_EXPLORED),
            ("L Front", LEFT_FRONTIER),
            ("R Vis", RIGHT_EXPLORED),
            ("R Front", RIGHT_FRONTIER),
        ]
        x_off = screen_width - 280
        for i, (label, col) in enumerate(legend):
            if x_off + i*70 < screen_width - 10:
                pygame.draw.rect(screen, col, (x_off + i*70, status_y + 5, 14, 14))
                lbl = font.render(label, True, TEXT_COL)
                screen.blit(lbl, (x_off + i*70 + 18, status_y + 3))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()