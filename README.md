# 🧠 Pathfinding Algorithms Visualizer  
### *From Offline Planning to Real‑Time Exploration*

[![Python 3.7+](https://img.shields.io/badge/Python-3.7+-blue.svg)](https://www.python.org/)
[![Pygame](https://img.shields.io/badge/Pygame-2.0+-green.svg)](https://www.pygame.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

> **Three interactive Pygame demos** that bring search algorithms to life – split‑screen, step‑by‑step, with full visual feedback.

---

## 📖 Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [The Three Demos](#-the-three-demos)
  - [1. A* vs Hierarchical Planning (HCS/CPL)](#1-a-vs-hierarchical-planning-hcscpl)
  - [2. Greedy DFS vs Greedy BFS (Smart Blind Exploration)](#2-greedy-dfs-vs-greedy-bfs-smart-blind-exploration)
  - [3. Simple DFS vs BFS / Random (Naïve Blind Exploration)](#3-simple-dfs-vs-bfs--random-naïve-blind-exploration)
- [Why This Matters](#-why-this-matters)
- [How to Run](#-how-to-run)
- [Controls](#-controls)
- [Visualisation & Split‑Cell Design](#-visualisation--splitcell-design)
- [Dependencies](#-dependencies)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🧭 Overview

This repository contains **three self‑contained Pygame scripts** that visualise different families of search algorithms in a **side‑by‑side split‑cell grid**.  
Each script compares two strategies **live**, showing their exploration front, visited cells, and final paths – all at a controllable speed.

The progression from **offline planning** (with a complete map) to **online blind exploration** (without a map) makes this a unique educational tool.

---

## ✨ Features

- **Split‑Cell Visualisation** – each grid cell is vertically halved: left shows algorithm A, right shows algorithm B.
- **Fair Animation** – both algorithms advance by the **same number of steps** per frame (or the same physical moves).
- **Fullscreen & Scalable** – adapts to any screen size.
- **Real‑Time Interaction** – place start/goal manually, auto‑place, change maps, adjust speed on the fly.
- **Multiple Map Types** – Rooms, Mazes, and Random obstacles.
- **Physical Movement Modelling** – in the blind demos, agents actually walk and **pay backtracking costs**.

---

## 🎮 The Three Demos

### 1. A* vs Hierarchical Planning (HCS/CPL)
**File:** `1.py`  
**Scenario:** The agent knows the **complete map** and the goal location.  
**Algorithms:**
- **Left:** Classic A* with 8‑connectivity and Euclidean heuristic.
- **Right:** Toggle between  
  - **HCS** (Hierarchical Convex Segmentation) – decomposes free space into convex rooms and bottlenecks, then plans at the room level.  
  - **CPL** (Corridor‑Portal‑Local) – decomposes into maximal rectangles, builds a portal graph, and runs A* on that graph.  

**Insight:** Hierarchical methods can **explore far fewer cells** than vanilla A*, showing the power of abstraction.

---

### 2. Greedy DFS vs Greedy BFS (Smart Blind Exploration)
**File:** `2.py`  
**Scenario:** The agent **does not know** the map or the goal location – it must physically walk and discover as it goes.  
**Algorithms:**
- **Left:** Greedy DFS – always moves towards the **closest unvisited frontier** by actual path cost through visited cells; ties broken by **most recent discovery** (LIFO).
- **Right:** Greedy BFS – same distance‑based greedy, but ties broken by **earliest discovery** (FIFO).

**Insight:** Both minimise backtracking, but DFS tends to **deep‑dive** into newly discovered areas, while BFS **explores wider** – revealing the fundamental difference between stack and queue even when both are greedy.

**Extra:** Discrete speed options (100, 75, 50, 30, 20, 10 ms per frame) let you watch the subtle behaviour at different paces.

---

### 3. Simple DFS vs BFS / Random (Naïve Blind Exploration)
**File:** `3.py`  
**Scenario:** Same blind setup, but **without any greedy heuristic** – just pure exploration.  
**Algorithms:**
- **Left:** Classic DFS – go as deep as possible, backtrack when stuck.
- **Right:** Toggle between  
  - **BFS** – explore in FIFO order (with backtracking to reach the next frontier).  
  - **Random** – wander aimlessly.

**Insight:** Without a heuristic, DFS often gets trapped in long corridors, BFS wastes effort walking back, and Random is hopeless. This highlights why **greedy distance** (as in demo 2) is a huge improvement.

---

## 🧠 Why This Matters

| Aspect | Explanation |
|--------|-------------|
| **Educational Value** | See the trade‑offs between **offline vs online**, **global vs local**, **depth vs breadth** in a single, visually intuitive environment. |
| **Real‑World Connection** | The blind demos simulate physical robots that must **pay for movement** – making the cost of backtracking visible. |
| **Algorithmic Insight** | The split‑cell design lets you directly compare **exploration patterns** – you can literally watch how HCS “jumps” across rooms while A* spreads out cell by cell. |
| **Interactive Learning** | Change maps, adjust speed, place start/goal – you can **experiment** and see results instantly. |

---

## 🚀 How to Run

1. **Install Pygame** (if not already installed):
   ```bash
   pip install pygame
   ```

2. **Clone or download** this repository.

3. **Run any of the three scripts**:
   ```bash
   python 1.py   # A* vs HCS/CPL
   python 2.py   # Greedy DFS vs Greedy BFS
   python 3.py   # Simple DFS vs BFS / Random
   ```

All scripts launch in **fullscreen** – press `ESC` to quit.

---

## ⌨️ Controls

| Key / Button | Action |
|--------------|--------|
| `Run` | Start the search comparison |
| `Reset` | Reset the map and agents |
| `Right: ...` | Toggle the right algorithm (varies per demo) |
| `Map: ...` | Cycle through map types (Rooms, Maze, Random) |
| `Auto Place` | Automatically place start and goal (far apart) |
| `+` / `-` | Increase / decrease speed (demo 2 only – discrete speeds) |
| `R` | Reset (keyboard) |
| `Space` | Run (keyboard) |
| `ESC` | Quit |

---

## 🎨 Visualisation & Split‑Cell Design

- **Each cell** is split vertically:  
  - **Left half** – shows the state of the **left algorithm** (blue/red tones).  
  - **Right half** – shows the state of the **right algorithm** (green/orange/red tones).  
- **Colour coding:**  
  - **Visited/Explored** – solid colour.  
  - **Frontier** – lighter / partially filled.  
  - **Agent** – bright white/yellow/magenta dot.  
  - **Path** – a distinct colour shown after completion.  
  - **Walls** – dark grey.  
- **Grid outline** – subtle borders separate cells, and a thin separator runs between the two halves.

This design **eliminates clutter** – you can instantly see what each algorithm is doing, even when they are in the same cell.

---

## 📦 Dependencies

- **Python 3.7+**
- **Pygame 2.0+**

No other libraries are required.

---

## 🤝 Contributing

Contributions are welcome! Feel free to:
- Add new algorithms (LRTA*, Dijkstra, RRT, etc.).
- Improve performance or visual design.
- Add more map generators or terrain costs.
- Write additional documentation or tutorials.

Please open an issue or pull request – I’ll review it as soon as possible.

---

## 📄 License

This project is licensed under the **MIT License** – you can use, modify, and distribute it freely, as long as you keep the original copyright notice.

---

*Made with ❤️ for students, researchers, and curious minds – let’s make search algorithms visual and fun!*
