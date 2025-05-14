# RRT Path Planning

## Overview

This script implements a 2D Rapidly-exploring Random Tree (RRT) path planner. Given a start and goal point and a list of rectangular obstacles, it finds a collision-free path and outputs the waypoints the robot should follow.

RRT was chosen over PRM because this is a single-query problem (one start and one goal).

## Input Format

Prepare your input in a text file (e.g., input.txt) with the following format:

``` 
<start_x>,<start_y>;<goal_x>,<goal_y>
<x1>,<y1>;<x2>,<y2>
<x1>,<y1>;<x2>,<y2>
...
-1
```

Where:

* The first line contains the start and goal coordinates separated by ;.
* Each subsequent line (before -1) defines a rectangular obstacle using two diagonal corners.
* -1 indicates the end of input.

### Example: `input.txt`

``` bash
10,10;80,30
20,10;20,50
20,50;90,50
30,30;40,40
-1
```

## Output Format

If a valid path is found, the output will be a list of (x, y) waypoints (one per line) from the start to the goal:

``` bash
10,10
...
80,30
```

If no path is found, the output will be:

``` bash
No path found.
```

## How to Run

Use input redirection in your terminal to provide the input from a file:

``` bash
python main.py < input.txt
```

This will print the path to standard output and also show a plot visualising:

* Obstacles (black rectangles)
* Start (green point)
* Goal (red point)
* RRT tree (gray edges)
* Final path (blue line)

## Requirements

* Python 3.x
* `matplotlib` library

Install required package using pip:

``` bash
pip install matplotlib
```

## Notes

* The environment is assumed to be a 100x100 grid.
* RRT uses a maximum of 10,000 iterations with a step size of 5 units.
* The algorithm includes 10% goal bias to improve convergence speed.
