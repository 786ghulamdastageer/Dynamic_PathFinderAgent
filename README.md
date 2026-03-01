AI Pathfinder – Informed Search Visualization
An interactive Python application that visualizes two informed search algorithms on a grid with dynamic obstacles and real-time re-planning.

Application: Dynamic Pathfinding Agent
GUI Library: Tkinter (Built-in, no external dependencies)

Features
Algorithm	Description	Priority
A* Search	Combines actual cost g(n) + heuristic h(n)	f(n) = g(n) + h(n)
Greedy Best-First Search	Uses only heuristic to estimate distance	f(n) = h(n)
Heuristic Functions
Heuristic	Formula	Best For
Manhattan Distance	|dx| + |dy|	4-directional grids (optimal)
Euclidean Distance	√(dx² + dy²)	Continuous movement, diagonals
Dynamic Obstacles
Random obstacles spawn during agent movement with 15% probability

If an obstacle blocks the current path, the agent automatically replans from its current position

Real-time path adaptation simulates real-world navigation

Visualization Colors
Element	Color	Hex Code	Meaning
Start Node	Emerald Green	#4CAF50	Source position 'S'
Goal Node	Red	#F44336	Destination position 'G'
Wall	Dark Gray	#404040	Impassable obstacle
Frontier	Yellow	#FFD700	Nodes in priority queue
Visited	Light Blue	#90CAF9	Already expanded nodes
Final Path	Bright Green	#00E676	Solution path
Agent	Pink	#FF4081	Current agent position 'A'
Empty Cell	Light Gray	#F0F0F0	Walkable area
Installation
bash
# No external libraries needed! Uses built-in Tkinter
# Just run the Python file
python pathfinding_agent.py
Requirements: Python 3.6+ (Tkinter comes pre-installed)

How to Run
bash
# Save the code as pathfinding_agent.py
python pathfinding_agent.py
The application window will open with a 15×20 grid and full control panel.

Controls
Action	How To
Place walls	Left-click & drag on grid
Erase walls	Right-click & drag
Set Start (S)	Select "Start" mode → Click on cell
Set Goal (G)	Select "Goal" mode → Click on cell
Select Algorithm	Click radio button (A* / Greedy BFS)
Select Heuristic	Click radio button (Manhattan / Euclidean)
Find Path	Click 🔍 Find Path button or press Space
Random Maze	Click 🎲 Random Maze button or press M
Clear Walls	Click 🧹 Clear Walls button or press C
Reset Grid	Click 🔄 Reset button or press R
Stop Animation	Click ⏹️ Stop button or press Esc
New Random Seed	Click 🔀 New Seed button
Toggle Fullscreen	Press F11
Dynamic Mode
Control	Description
Dynamic Obstacles	Checkbox to enable/disable dynamic mode
When ON	Obstacles spawn randomly during agent movement
Replanning	Agent automatically finds new path if blocked
Algorithm Comparison
Criteria	A* Search	Greedy Best-First Search
Priority	f(n) = g(n) + h(n)	f(n) = h(n)
Complete	✅ Yes	❌ No
Optimal	✅ Yes (with admissible heuristic)	❌ No
Speed	🐢 Moderate	⚡ Fast
Memory	📚 Higher (stores g_costs)	📦 Lower
Best For	Optimal path required	Quick approximate paths
Project Structure
text
informed-search/
├── pathfinding_agent.py   # Complete application (single file)
├── README.md              # This file
└── screenshots/           # (Optional) Screenshots folder
    ├── best_case_a_star.png
    ├── worst_case_greedy.png
    └── dynamic_mode.png
Movement & Costs
4-directional movement (Up, Down, Left, Right)

Each move costs exactly 1.0 (unit cost)

No diagonal movement (maintains Manhattan heuristic admissibility)

Performance Notes
Best Case: Open grid with clear path – finds solution in < 100ms

Worst Case: Complex maze with many obstacles – explores extensively but guarantees optimal path (A*)

Dynamic Mode: Replanning happens in milliseconds

Threaded Search: UI remains responsive during computation

Keyboard Shortcuts Summary
Key	Function
Space	Find Path
R	Reset Grid
M	Random Maze
C	Clear Walls
F11	Toggle Fullscreen
Esc	Stop Operation
Troubleshooting
Issue	Solution
"Tkinter not found"	Install Python with Tkinter: sudo apt-get install python3-tk (Linux)
Slow animation	Reduce grid size or disable dynamic mode
No path found	Ensure start and goal are reachable (not blocked by walls)
Replanning not working	Check that Dynamic Obstacles checkbox is ON
Future Improvements
Bidirectional A* for faster optimal search

Weighted A* for speed/optimality tradeoff

Save/load custom maps

Variable terrain costs (mud, sand, road)

Multi-agent pathfinding

License
This project is created for educational purposes as part of AI2002 Artificial Intelligence course assignment.
