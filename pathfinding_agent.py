"""
Dynamic Pathfinding Agent - Tkinter Version (Optimized)
A* and Greedy Best-First Search with GUI
Handles Best Case (Fast) and Worst Case (Properly)
"""

import tkinter as tk
from tkinter import ttk, messagebox
import heapq
import math
import random
import time
import threading
from collections import deque

# ==================== CONSTANTS ====================
CELL_SIZE = 30
GRID_ROWS = 15
GRID_COLS = 20

# Colors (RGB tuples for tkinter)
COLORS = {
    'EMPTY': '#F0F0F0',      # Light gray
    'WALL': '#404040',        # Dark gray
    'START': '#4CAF50',       # Green
    'GOAL': '#F44336',        # Red
    'FRONTIER': '#FFD700',    # Yellow
    'VISITED': '#90CAF9',     # Light blue
    'PATH': '#00E676',        # Bright green
    'AGENT': '#FF4081',       # Pink
    'GRID_LINE': '#BDBDBD',   # Gray for grid lines
    'BUTTON': '#E0E0E0',      # Light gray for buttons
    'BUTTON_ACTIVE': '#BDBDBD' # Darker gray for active buttons
}

# ==================== NODE CLASS ====================
class Node:
    __slots__ = ('row', 'col', 'is_wall', 'is_start', 'is_goal', 
                 'is_frontier', 'is_visited', 'is_path', 'is_agent',
                 'g_cost', 'h_cost', 'f_cost', 'parent')
    
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.is_wall = False
        self.is_start = False
        self.is_goal = False
        self.is_frontier = False
        self.is_visited = False
        self.is_path = False
        self.is_agent = False
        self.g_cost = float('inf')
        self.h_cost = 0
        self.f_cost = float('inf')
        self.parent = None
        
    def get_color(self):
        """Return color based on node state"""
        if self.is_agent:
            return COLORS['AGENT']
        if self.is_start:
            return COLORS['START']
        if self.is_goal:
            return COLORS['GOAL']
        if self.is_path:
            return COLORS['PATH']
        if self.is_visited:
            return COLORS['VISITED']
        if self.is_frontier:
            return COLORS['FRONTIER']
        if self.is_wall:
            return COLORS['WALL']
        return COLORS['EMPTY']
    
    def reset_search_state(self):
        """Reset all search-related flags"""
        self.is_frontier = False
        self.is_visited = False
        self.is_path = False
        self.is_agent = False
        self.g_cost = float('inf')
        self.h_cost = 0
        self.f_cost = float('inf')
        self.parent = None

# ==================== GRID CLASS ====================
class Grid:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.grid = [[Node(i, j) for j in range(cols)] for i in range(rows)]
        self.start = None
        self.goal = None
        self.metrics = {
            'nodes_visited': 0,
            'path_cost': 0,
            'execution_time': 0
        }
        self.random_seed = int(time.time())
        
    def reset_search(self):
        """Reset search state for all nodes"""
        for row in self.grid:
            for node in row:
                node.reset_search_state()
        self.metrics['nodes_visited'] = 0
        self.metrics['path_cost'] = 0
        
    def generate_random_maze(self, density=0.3, use_seed=True):
        """Generate random walls with given density"""
        if use_seed:
            random.seed(self.random_seed)
            
        self.reset_search()
        for row in self.grid:
            for node in row:
                if node != self.start and node != self.goal:
                    node.is_wall = random.random() < density
                    
    def new_random_seed(self):
        """Generate new random seed"""
        self.random_seed = int(time.time())
        return self.random_seed
                    
    def clear_walls(self):
        """Remove all walls"""
        for row in self.grid:
            for node in row:
                if node != self.start and node != self.goal:
                    node.is_wall = False
                    
    def get_neighbors(self, node):
        """Get valid neighbors (4-directional) - optimized"""
        neighbors = []
        r, c = node.row, node.col
        
        # Up
        if r > 0 and not self.grid[r-1][c].is_wall:
            neighbors.append(self.grid[r-1][c])
        # Down
        if r < self.rows-1 and not self.grid[r+1][c].is_wall:
            neighbors.append(self.grid[r+1][c])
        # Left
        if c > 0 and not self.grid[r][c-1].is_wall:
            neighbors.append(self.grid[r][c-1])
        # Right
        if c < self.cols-1 and not self.grid[r][c+1].is_wall:
            neighbors.append(self.grid[r][c+1])
            
        return neighbors
    
    def get_node_from_pos(self, x, y):
        """Get node from pixel coordinates"""
        col = x // CELL_SIZE
        row = y // CELL_SIZE
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.grid[row][col]
        return None

# ==================== HEURISTIC FUNCTIONS ====================
def manhattan_distance(node, goal):
    return abs(node.row - goal.row) + abs(node.col - goal.col)

def euclidean_distance(node, goal):
    return math.sqrt((node.row - goal.row)**2 + (node.col - goal.col)**2)

# ==================== SEARCH ALGORITHMS (OPTIMIZED) ====================
def greedy_bfs(grid, start, goal, heuristic_func, update_callback=None):
    """Greedy Best-First Search - Optimized version"""
    grid.reset_search()
    frontier = []
    counter = 0
    
    # Initialize start
    start.h_cost = heuristic_func(start, goal)
    heapq.heappush(frontier, (start.h_cost, counter, start))
    start.is_frontier = True
    
    # Fast lookup sets
    frontier_set = {start}
    visited_set = set()
    
    nodes_visited = 0
    start_time = time.time()
    
    while frontier:
        # Visual update (less frequent for speed)
        nodes_visited += 1
        if update_callback and nodes_visited % 5 == 0:
            update_callback()
            time.sleep(0.001)
        
        # Pop node with lowest h_cost
        _, _, current = heapq.heappop(frontier)
        frontier_set.remove(current)
        current.is_frontier = False
        current.is_visited = True
        visited_set.add(current)
        
        # Check goal
        if current == goal:
            path = []
            temp = current
            while temp:
                path.append(temp)
                temp = temp.parent
            path.reverse()
            
            grid.metrics['nodes_visited'] = nodes_visited
            grid.metrics['execution_time'] = (time.time() - start_time) * 1000
            grid.metrics['path_cost'] = len(path) - 1
            return path
        
        # Expand neighbors
        for neighbor in grid.get_neighbors(current):
            if neighbor not in visited_set and not neighbor.is_wall:
                new_h = heuristic_func(neighbor, goal)
                
                if neighbor not in frontier_set:
                    neighbor.parent = current
                    neighbor.h_cost = new_h
                    counter += 1
                    heapq.heappush(frontier, (new_h, counter, neighbor))
                    frontier_set.add(neighbor)
                    neighbor.is_frontier = True
    
    grid.metrics['nodes_visited'] = nodes_visited
    grid.metrics['execution_time'] = (time.time() - start_time) * 1000
    return None

def a_star(grid, start, goal, heuristic_func, update_callback=None):
    """A* Search - Optimized version"""
    grid.reset_search()
    frontier = []
    counter = 0
    
    # Initialize start
    start.g_cost = 0
    start.h_cost = heuristic_func(start, goal)
    start.f_cost = start.g_cost + start.h_cost
    heapq.heappush(frontier, (start.f_cost, counter, start))
    start.is_frontier = True
    
    # Fast lookup structures
    frontier_set = {start}
    visited_set = set()
    g_costs = {start: 0}
    
    nodes_visited = 0
    start_time = time.time()
    
    while frontier:
        # Visual update (less frequent for speed)
        nodes_visited += 1
        if update_callback and nodes_visited % 5 == 0:
            update_callback()
            time.sleep(0.001)
        
        # Pop node with lowest f_cost
        _, _, current = heapq.heappop(frontier)
        frontier_set.remove(current)
        current.is_frontier = False
        current.is_visited = True
        visited_set.add(current)
        
        # Check goal
        if current == goal:
            path = []
            temp = current
            while temp:
                path.append(temp)
                temp = temp.parent
            path.reverse()
            
            grid.metrics['nodes_visited'] = nodes_visited
            grid.metrics['execution_time'] = (time.time() - start_time) * 1000
            grid.metrics['path_cost'] = current.g_cost
            return path
        
        # Expand neighbors
        for neighbor in grid.get_neighbors(current):
            if neighbor.is_wall or neighbor in visited_set:
                continue
            
            tentative_g = current.g_cost + 1
            
            if tentative_g < g_costs.get(neighbor, float('inf')):
                neighbor.parent = current
                neighbor.g_cost = tentative_g
                neighbor.h_cost = heuristic_func(neighbor, goal)
                neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                
                g_costs[neighbor] = tentative_g
                
                if neighbor not in frontier_set:
                    counter += 1
                    heapq.heappush(frontier, (neighbor.f_cost, counter, neighbor))
                    frontier_set.add(neighbor)
                    neighbor.is_frontier = True
    
    grid.metrics['nodes_visited'] = nodes_visited
    grid.metrics['execution_time'] = (time.time() - start_time) * 1000
    return None

# ==================== MAIN APPLICATION CLASS ====================
class PathfindingApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Dynamic Pathfinding Agent - Optimized")
        self.root.geometry("1400x800")
        self.root.configure(bg='#2C3E50')
        
        # Grid settings
        self.rows = GRID_ROWS
        self.cols = GRID_COLS
        self.grid = Grid(self.rows, self.cols)
        
        # Set default start and goal
        self.grid.start = self.grid.grid[0][0]
        self.grid.start.is_start = True
        self.grid.goal = self.grid.grid[self.rows-1][self.cols-1]
        self.grid.goal.is_goal = True
        
        # Algorithm settings
        self.current_algorithm = "A*"
        self.current_heuristic = "Manhattan"
        self.dynamic_mode = False
        self.searching = False
        self.path = None
        self.agent_path = []
        self.agent_idx = 0
        self.agent_path_set = set()  # Fast lookup for path
        self.edit_mode = "Wall"
        
        # Canvas for grid
        canvas_width = self.cols * CELL_SIZE + 2
        canvas_height = self.rows * CELL_SIZE + 2
        self.canvas = tk.Canvas(
            root, 
            width=canvas_width, 
            height=canvas_height,
            bg=COLORS['EMPTY'],
            highlightbackground=COLORS['GRID_LINE'],
            highlightthickness=1
        )
        self.canvas.place(x=20, y=20)
        
        # Control panel
        self.create_control_panel()
        
        # Bind events
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas.bind("<Button-3>", self.on_canvas_right_click)
        self.root.bind("<space>", lambda e: self.find_path())
        self.root.bind("<r>", lambda e: self.reset_grid())
        self.root.bind("<m>", lambda e: self.generate_maze())
        self.root.bind("<c>", lambda e: self.clear_walls())
        
        # Draw initial grid
        self.draw_grid()
        
        # Animation variables
        self.animation_id = None
        
    def create_control_panel(self):
        """Create all control buttons and displays"""
        panel_x = self.cols * CELL_SIZE + 40
        panel_y = 20
        
        # Title
        title = tk.Label(
            self.root,
            text="Dynamic Pathfinding Agent",
            font=("Arial", 16, "bold"),
            fg="#ECF0F1",
            bg="#2C3E50"
        )
        title.place(x=panel_x, y=panel_y)
        
        # Algorithm selection
        y_offset = 60
        algo_label = tk.Label(
            self.root,
            text="Algorithm:",
            font=("Arial", 11, "bold"),
            fg="#ECF0F1",
            bg="#2C3E50"
        )
        algo_label.place(x=panel_x, y=panel_y + y_offset)
        
        self.algo_var = tk.StringVar(value="A*")
        algo_frame = tk.Frame(self.root, bg="#2C3E50")
        algo_frame.place(x=panel_x, y=panel_y + y_offset + 25)
        
        tk.Radiobutton(
            algo_frame, text="A* (Optimal)", variable=self.algo_var, value="A*",
            command=self.update_algorithm, bg="#2C3E50", fg="white",
            selectcolor="#34495E", activebackground="#2C3E50"
        ).pack(side=tk.LEFT, padx=5)
        
        tk.Radiobutton(
            algo_frame, text="Greedy BFS (Fast)", variable=self.algo_var, value="Greedy",
            command=self.update_algorithm, bg="#2C3E50", fg="white",
            selectcolor="#34495E", activebackground="#2C3E50"
        ).pack(side=tk.LEFT, padx=5)
        
        # Heuristic selection
        y_offset += 80
        heur_label = tk.Label(
            self.root,
            text="Heuristic:",
            font=("Arial", 11, "bold"),
            fg="#ECF0F1",
            bg="#2C3E50"
        )
        heur_label.place(x=panel_x, y=panel_y + y_offset)
        
        self.heur_var = tk.StringVar(value="Manhattan")
        heur_frame = tk.Frame(self.root, bg="#2C3E50")
        heur_frame.place(x=panel_x, y=panel_y + y_offset + 25)
        
        tk.Radiobutton(
            heur_frame, text="Manhattan", variable=self.heur_var, value="Manhattan",
            command=self.update_heuristic, bg="#2C3E50", fg="white",
            selectcolor="#34495E", activebackground="#2C3E50"
        ).pack(side=tk.LEFT, padx=5)
        
        tk.Radiobutton(
            heur_frame, text="Euclidean", variable=self.heur_var, value="Euclidean",
            command=self.update_heuristic, bg="#2C3E50", fg="white",
            selectcolor="#34495E", activebackground="#2C3E50"
        ).pack(side=tk.LEFT, padx=5)
        
        # Edit mode selection
        y_offset += 80
        edit_label = tk.Label(
            self.root,
            text="Edit Mode:",
            font=("Arial", 11, "bold"),
            fg="#ECF0F1",
            bg="#2C3E50"
        )
        edit_label.place(x=panel_x, y=panel_y + y_offset)
        
        self.edit_var = tk.StringVar(value="Wall")
        edit_frame = tk.Frame(self.root, bg="#2C3E50")
        edit_frame.place(x=panel_x, y=panel_y + y_offset + 25)
        
        for mode in ["Wall", "Start", "Goal"]:
            tk.Radiobutton(
                edit_frame, text=mode, variable=self.edit_var, value=mode,
                command=self.update_edit_mode, bg="#2C3E50", fg="white",
                selectcolor="#34495E", activebackground="#2C3E50"
            ).pack(side=tk.LEFT, padx=5)
        
        # Dynamic mode checkbox
        y_offset += 80
        self.dynamic_var = tk.BooleanVar(value=False)
        tk.Checkbutton(
            self.root,
            text="Dynamic Obstacles",
            variable=self.dynamic_var,
            command=self.update_dynamic_mode,
            bg="#2C3E50", fg="white",
            selectcolor="#34495E", activebackground="#2C3E50"
        ).place(x=panel_x, y=panel_y + y_offset)
        
        # Control buttons
        y_offset += 50
        button_width = 15
        
        tk.Button(
            self.root, text="🔍 Find Path", width=button_width,
            command=self.find_path, bg="#27AE60", fg="white",
            font=("Arial", 10, "bold"), relief=tk.RAISED
        ).place(x=panel_x, y=panel_y + y_offset)
        
        tk.Button(
            self.root, text="🎲 Random Maze", width=button_width,
            command=self.generate_maze, bg="#F39C12", fg="white",
            font=("Arial", 10, "bold"), relief=tk.RAISED
        ).place(x=panel_x + 120, y=panel_y + y_offset)
        
        y_offset += 40
        tk.Button(
            self.root, text="🧹 Clear Walls", width=button_width,
            command=self.clear_walls, bg="#7F8C8D", fg="white",
            font=("Arial", 10, "bold"), relief=tk.RAISED
        ).place(x=panel_x, y=panel_y + y_offset)
        
        tk.Button(
            self.root, text="🔄 Reset", width=button_width,
            command=self.reset_grid, bg="#E74C3C", fg="white",
            font=("Arial", 10, "bold"), relief=tk.RAISED
        ).place(x=panel_x + 120, y=panel_y + y_offset)
        
        y_offset += 40
        tk.Button(
            self.root, text="⏹️ Stop", width=button_width,
            command=self.stop_search, bg="#E74C3C", fg="white",
            font=("Arial", 10, "bold"), relief=tk.RAISED
        ).place(x=panel_x, y=panel_y + y_offset)
        
        tk.Button(
            self.root, text="🔀 New Seed", width=button_width,
            command=self.new_random_seed, bg="#3498DB", fg="white",
            font=("Arial", 10, "bold"), relief=tk.RAISED
        ).place(x=panel_x + 120, y=panel_y + y_offset)
        
        # Instructions
        y_offset += 70
        inst_frame = tk.Frame(self.root, bg="#34495E", relief=tk.RIDGE, bd=2)
        inst_frame.place(x=panel_x-10, y=panel_y + y_offset, width=250, height=180)
        
        instructions = [
            "📋 INSTRUCTIONS",
            "• Left Click: Toggle Wall",
            "• Right Click: Erase Wall",
            "• Edit Mode: Start/Goal",
            "• Space: Find Path",
            "• R: Reset | M: Maze",
            "• Dynamic: Auto obstacles"
        ]
        
        for i, inst in enumerate(instructions):
            color = "#F1C40F" if i == 0 else "#ECF0F1"
            font = ("Arial", 9, "bold") if i == 0 else ("Arial", 8, "normal")
            tk.Label(
                inst_frame, text=inst,
                fg=color, bg="#34495E",
                font=font,
                anchor="w"
            ).pack(padx=10, pady=2, fill="x")
        
        # Metrics display
        y_offset += 200
        metrics_frame = tk.Frame(self.root, bg="#34495E", relief=tk.RIDGE, bd=2)
        metrics_frame.place(x=panel_x-10, y=panel_y + y_offset, width=250, height=150)
        
        tk.Label(
            metrics_frame, text="📊 METRICS",
            fg="#F1C40F", bg="#34495E",
            font=("Arial", 11, "bold")
        ).pack(pady=5)
        
        self.metrics_labels = {}
        metrics_list = [
            ('visited', "Nodes Visited: 0"),
            ('cost', "Path Cost: 0"),
            ('time', "Time: 0 ms"),
            ('seed', f"Seed: {self.grid.random_seed}"),
            ('status', "Status: Ready")
        ]
        
        for key, text in metrics_list:
            label = tk.Label(
                metrics_frame, text=text,
                fg="#ECF0F1", bg="#34495E",
                font=("Courier", 9)
            )
            label.pack(anchor="w", padx=10, pady=1)
            self.metrics_labels[key] = label
        
        # Legend
        y_offset += 170
        legend_frame = tk.Frame(self.root, bg="#34495E", relief=tk.RIDGE, bd=2)
        legend_frame.place(x=panel_x-10, y=panel_y + y_offset, width=250, height=180)
        
        tk.Label(
            legend_frame, text="🎨 LEGEND",
            fg="#F1C40F", bg="#34495E",
            font=("Arial", 11, "bold")
        ).pack(pady=5)
        
        legend_items = [
            ('START', '#4CAF50', 'Start (S)'),
            ('GOAL', '#F44336', 'Goal (G)'),
            ('WALL', '#404040', 'Wall'),
            ('FRONTIER', '#FFD700', 'Frontier'),
            ('VISITED', '#90CAF9', 'Visited'),
            ('PATH', '#00E676', 'Path'),
            ('AGENT', '#FF4081', 'Agent (A)')
        ]
        
        for color_key, color_val, text in legend_items:
            frame = tk.Frame(legend_frame, bg="#34495E")
            frame.pack(anchor="w", padx=10, pady=1)
            
            color_box = tk.Canvas(frame, width=15, height=15, bg=color_val, 
                                  highlightthickness=1, highlightbackground="black")
            color_box.pack(side=tk.LEFT, padx=2)
            
            tk.Label(frame, text=text, fg="white", bg="#34495E", 
                    font=("Arial", 8)).pack(side=tk.LEFT, padx=5)
        
    def update_algorithm(self):
        self.current_algorithm = self.algo_var.get()
        
    def update_heuristic(self):
        self.current_heuristic = self.heur_var.get()
        
    def update_edit_mode(self):
        self.edit_mode = self.edit_var.get()
        
    def update_dynamic_mode(self):
        self.dynamic_mode = self.dynamic_var.get()
        
    def new_random_seed(self):
        """Generate new random seed"""
        seed = self.grid.new_random_seed()
        self.metrics_labels['seed'].config(text=f"Seed: {seed}")
        
    def draw_grid(self):
        """Draw the grid on canvas - optimized"""
        self.canvas.delete("all")
        
        # Draw cells
        for row in range(self.rows):
            for col in range(self.cols):
                node = self.grid.grid[row][col]
                x1 = col * CELL_SIZE
                y1 = row * CELL_SIZE
                x2 = x1 + CELL_SIZE
                y2 = y1 + CELL_SIZE
                
                # Fill cell
                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=node.get_color(),
                    outline=COLORS['GRID_LINE'],
                    width=1
                )
                
                # Add labels
                if node.is_start:
                    self.canvas.create_text(
                        x1 + CELL_SIZE//2, y1 + CELL_SIZE//2,
                        text="S", fill="white", font=("Arial", 10, "bold")
                    )
                elif node.is_goal:
                    self.canvas.create_text(
                        x1 + CELL_SIZE//2, y1 + CELL_SIZE//2,
                        text="G", fill="white", font=("Arial", 10, "bold")
                    )
                elif node.is_agent:
                    self.canvas.create_text(
                        x1 + CELL_SIZE//2, y1 + CELL_SIZE//2,
                        text="A", fill="white", font=("Arial", 10, "bold")
                    )
        
        self.canvas.update_idletasks()
    
    def on_canvas_click(self, event):
        """Handle canvas click events"""
        node = self.grid.get_node_from_pos(event.x, event.y)
        if not node:
            return
        
        if self.edit_mode == "Wall":
            if node != self.grid.start and node != self.grid.goal:
                node.is_wall = not node.is_wall
                
        elif self.edit_mode == "Start":
            if node != self.grid.goal and not node.is_wall:
                if self.grid.start:
                    self.grid.start.is_start = False
                self.grid.start = node
                node.is_start = True
                
        elif self.edit_mode == "Goal":
            if node != self.grid.start and not node.is_wall:
                if self.grid.goal:
                    self.grid.goal.is_goal = False
                self.grid.goal = node
                node.is_goal = True
        
        self.draw_grid()
    
    def on_canvas_drag(self, event):
        """Handle canvas drag for drawing walls"""
        node = self.grid.get_node_from_pos(event.x, event.y)
        if node and self.edit_mode == "Wall":
            if node != self.grid.start and node != self.grid.goal:
                node.is_wall = True
                self.draw_grid()
    
    def on_canvas_right_click(self, event):
        """Right click to erase walls"""
        node = self.grid.get_node_from_pos(event.x, event.y)
        if node and self.edit_mode == "Wall":
            if node != self.grid.start and node != self.grid.goal:
                node.is_wall = False
                self.draw_grid()
    
    def get_heuristic_func(self):
        """Return appropriate heuristic function"""
        return manhattan_distance if self.current_heuristic == "Manhattan" else euclidean_distance
    
    def find_path_thread(self):
        """Run search in separate thread"""
        heuristic = self.get_heuristic_func()
        
        if self.current_algorithm == "A*":
            self.path = a_star(
                self.grid, self.grid.start, self.grid.goal,
                heuristic, self.safe_draw
            )
        else:
            self.path = greedy_bfs(
                self.grid, self.grid.start, self.grid.goal,
                heuristic, self.safe_draw
            )
        
        # Update UI in main thread
        self.root.after(0, self.search_complete)
    
    def safe_draw(self):
        """Thread-safe draw call"""
        self.root.after(0, self.draw_grid)
    
    def find_path(self):
        """Run the selected search algorithm"""
        if not self.grid.start or not self.grid.goal:
            messagebox.showerror("Error", "Start and Goal must be set!")
            return
        
        if self.searching:
            return
        
        self.searching = True
        self.update_metrics(status="Searching...")
        
        # Run search in thread to keep UI responsive
        threading.Thread(target=self.find_path_thread, daemon=True).start()
    
    def search_complete(self):
        """Handle search completion"""
        self.searching = False
        
        if self.path:
            # Mark path
            for node in self.path:
                node.is_path = True
            
            # Create fast lookup set for path
            self.agent_path_set = set(self.path)
            
            self.draw_grid()
            
            # Update metrics
            self.update_metrics(
                visited=self.grid.metrics['nodes_visited'],
                cost=self.grid.metrics['path_cost'],
                time_ms=self.grid.metrics['execution_time'],
                status="Path Found! ✓"
            )
            
            # Start dynamic mode if enabled
            if self.dynamic_mode:
                self.agent_path = self.path
                self.agent_idx = 0
                self.start_dynamic_mode()
        else:
            self.update_metrics(status="No Path Found!")
    
    def start_dynamic_mode(self):
        """Start dynamic obstacle simulation"""
        if not self.dynamic_mode or not self.agent_path:
            return
        
        self.agent_idx = 0
        self.simulate_agent()
    
    def simulate_agent(self):
        """Simulate agent movement with dynamic obstacles"""
        if not self.dynamic_mode or self.agent_idx >= len(self.agent_path):
            self.update_metrics(status="Goal Reached! ✓")
            return
        
        # Clear previous agent
        for row in self.grid.grid:
            for node in row:
                node.is_agent = False
        
        # Move agent
        current = self.agent_path[self.agent_idx]
        current.is_agent = True
        self.draw_grid()
        
        # Spawn random obstacle (15% chance)
        if random.random() < 0.15:
            self.spawn_random_obstacle(current)
        
        self.agent_idx += 1
        
        # Schedule next move
        self.animation_id = self.root.after(200, self.simulate_agent)
    
    def spawn_random_obstacle(self, agent_pos):
        """Spawn random obstacle and replan if needed"""
        r = random.randint(1, self.rows-2)
        c = random.randint(1, self.cols-2)
        node = self.grid.grid[r][c]
        
        if (node != self.grid.start and node != self.grid.goal and 
            node != agent_pos and not node.is_wall):
            
            node.is_wall = True
            self.draw_grid()
            
            # Fast check if obstacle blocks path using set
            if node in self.agent_path_set and node in self.agent_path[self.agent_idx:]:
                # Replan path
                heuristic = self.get_heuristic_func()
                new_path = a_star(
                    self.grid, agent_pos, self.grid.goal,
                    heuristic, self.safe_draw
                )
                
                if new_path:
                    self.agent_path = new_path
                    self.agent_path_set = set(new_path)
                    self.agent_idx = 0
                    self.update_metrics(status="Replanned!")
                else:
                    self.update_metrics(status="Blocked!")
                    self.dynamic_mode = False
    
    def generate_maze(self):
        """Generate random maze"""
        self.stop_search()
        self.grid.generate_random_maze(0.3)
        self.draw_grid()
        self.metrics_labels['seed'].config(text=f"Seed: {self.grid.random_seed}")
        self.update_metrics(status="Maze Generated")
    
    def clear_walls(self):
        """Clear all walls"""
        self.stop_search()
        self.grid.clear_walls()
        self.draw_grid()
        self.update_metrics(status="Walls Cleared")
    
    def reset_grid(self):
        """Reset the entire grid"""
        self.stop_search()
        self.grid = Grid(self.rows, self.cols)
        self.grid.start = self.grid.grid[0][0]
        self.grid.start.is_start = True
        self.grid.goal = self.grid.grid[self.rows-1][self.cols-1]
        self.grid.goal.is_goal = True
        self.path = None
        self.agent_path = []
        self.agent_path_set = set()
        self.draw_grid()
        self.metrics_labels['seed'].config(text=f"Seed: {self.grid.random_seed}")
        self.update_metrics(status="Ready")
    
    def stop_search(self):
        """Stop any ongoing animation"""
        if self.animation_id:
            self.root.after_cancel(self.animation_id)
            self.animation_id = None
        self.searching = False
        self.update_metrics(status="Stopped")
    
    def update_metrics(self, visited=0, cost=0, time_ms=0, status="Ready"):
        """Update metrics display"""
        self.metrics_labels['visited'].config(text=f"Nodes Visited: {visited}")
        self.metrics_labels['cost'].config(text=f"Path Cost: {cost}")
        self.metrics_labels['time'].config(text=f"Time: {time_ms:.1f} ms")
        self.metrics_labels['status'].config(text=f"Status: {status}")

# ==================== RUN APPLICATION ====================
if __name__ == "__main__":
    root = tk.Tk()
    app = PathfindingApp(root)
    root.mainloop()