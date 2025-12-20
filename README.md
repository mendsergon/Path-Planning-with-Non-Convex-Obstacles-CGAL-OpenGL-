# **Path Planning with Non-Convex Obstacles (CGAL + OpenGL)**

### **Project Summary**

This C++ program implements a complete **path planning system** for navigating around non-convex polygonal obstacles within a rectangular workspace. Using the **Computational Geometry Algorithms Library (CGAL)** for robust geometric operations and **OpenGL/GLFW** for real-time visualization, the system determines whether a collision-free path exists between two user-selected points. When a path exists, it computes and displays the **shortest viable route** that avoids all obstacle interiors.

---

### **Core Features**

*   Handles an arbitrary number of **non-convex polygonal obstacles**.
*   **Interactive point selection** via mouse clicks to define path start and end points.
*   Constructs a **visibility graph** of the free space to model navigable connections.
*   Computes the **shortest collision-free path** using Dijkstra's algorithm on the visibility graph.
*   Provides **real-time visual feedback** showing obstacles, selected points, and the computed path.
*   **Regenerates random obstacle configurations** with guaranteed non-convex shapes (e.g., stars, L-shapes).
*   Implements exact **collision detection** to ensure paths never intersect obstacle interiors.

---

### **Key Methods and Algorithms**

#### **1. Geometric Predicates with CGAL**

*   **Point-in-Polygon Test**: Uses `CGAL::Polygon_2::bounded_side()` for an exact, robust test to determine if a query point lies inside, outside, or on the boundary of a polygon. This is critical for validating start/end points and checking segment midpoints.
*   **Segment-Polygon Intersection**: Implements `segmentIntersectsPolygon()` which checks if a line segment improperly intersects any edge of a polygon or if its midpoint lies within the polygon, ensuring paths stay in free space.

#### **2. Visibility Graph Construction**

*   **Node Sampling**: Populates the graph with nodes from:
    1.  All vertices of every obstacle.
    2.  Midpoints along obstacle edges for finer navigation.
    3.  A sparse grid of points sampled from the free space.
*   **Edge Creation (Visibility Checking)**: An edge is created between two nodes `p_i` and `p_j` if and only if the line segment `p_i p_j` does **not** intersect the interior of any obstacle, as determined by `segmentIntersectsPolygon()`. This establishes which nodes can "see" each other.
    The graph is stored as an adjacency list with edge weights equal to the Euclidean distance between connected nodes.

#### **3. Shortest Path Computation (Dijkstra's Algorithm)**

*   **Graph Search**: Given start `S` and end `E` points, they are temporarily added as nodes to the visibility graph and connected to all other visible nodes.
*   **Path Finding**: Dijkstra's algorithm is then run on this augmented graph to find the shortest path from `S` to `E`.
*   **Path Extraction**: The sequence of graph nodes is extracted and returned as the collision-free path.

#### **4. Free Space Management**

*   The `FreeSpace` class encapsulates the rectangular boundary and the list of obstacles.
*   It provides methods like `isPointFree()` and `isPathFree()` to query the validity of points and segments within the defined workspace.

---

### **Skills Demonstrated**

*   **Computational Geometry**: Implementation of fundamental algorithms for polygon intersection and visibility.
*   **CGAL Proficiency**: Effective use of a sophisticated geometry library for exact numerical predicates.
*   **Graph Theory Application**: Design and construction of a visibility graph to solve a continuous geometric problem.
*   **Path Planning Algorithms**: Implementation of a classic robotics/navigation algorithm (visibility graph + Dijkstra).
*   **Interactive Graphics Programming**: Creation of a real-time GUI with OpenGL/GLFW for visualization and user input.
*   **C++ Software Design**: Organized, modular code structure separating geometry, logic, and visualization concerns.

---

### **File Overview**

| File Name | Description |
| :--- | :--- |
| **main.cpp** | Main program entry point. Handles GLFW window setup, OpenGL rendering, user input callbacks, and the main application loop. |
| **geometry.h** | Header file declaring the `GeometryUtils` static class and the `FreeSpace` class, along with CGAL kernel and polygon type definitions. |
| **geometry.cpp** | Implementation of all geometric functions: `isPointInsidePolygon`, `segmentIntersectsPolygon`, and the `FreeSpace` class methods. |
| **path_finder.h** | Header file declaring the `PathFinder` class, the `PathFinderResult` struct, and the `Point2Hash` functor for hashing CGAL points. |
| **path_finder.cpp** | Implementation of the pathfinding logic: visibility graph construction (`buildGraph`) and shortest path search (`findPath`). |
| **Makefile** | Build configuration file to compile and link the project with all necessary libraries. |

---

### **How to Compile and Run**

#### **1. Install Dependencies**

First, install the required development libraries for your Linux distribution.

*   **Ubuntu / Debian**:
    ```bash
    sudo apt update
    sudo apt install libcgal-dev libglfw3-dev libgmp-dev libmpfr-dev g++ make
    ```

*   **Fedora / RHEL**:
    ```bash
    sudo dnf install cgal-devel glfw-devel gmp-devel mpfr-devel gcc-c++ make
    ```

*   **Arch Linux**:
    ```bash
    sudo pacman -S cgal glfw gmp mpfr gcc make
    ```

*   **openSUSE**:
    ```bash
    sudo zypper install cgal-devel glfw-devel gmp-devel mpfr-devel gcc-c++ make
    ```

#### **2. Compile the Program**

Navigate to the project directory containing the source files and `Makefile`.

```bash
# Clean any previous builds and compile the project
make clean
make
```

#### **3. Run the Application**

```bash
./path_planning
```

#### **4. Program Controls**

*   **Left Mouse Click**: Place points. The first click sets the start point, the second sets the end point and triggers path calculation.
*   **R Key**: Regenerate a new random set of non-convex obstacles.
*   **C Key**: Clear the currently placed start and end points.
*   **ESC Key**: Exit the program.

#### **5. Understanding the Display**

*   **Dark Blue Background**: The bounded workspace.
*   **Red Polygons**: Non-convex obstacles.
*   **Blue Circles**: User-selected start and end points.
*   **Green Line & Dots**: The computed shortest collision-free path (if one exists).
*   **Console Output**: Provides text feedback on the program's status and pathfinding results.
