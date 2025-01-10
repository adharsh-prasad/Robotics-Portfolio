# **Path Planning Algorithms for Lunar Base Operations**

This project is part of a larger multi-robot coordination system for lunar base operations and warehouse management. The current focus is on developing and benchmarking efficient path planning algorithms for a segmented lunar base map. The main goals are:

1. Implement and compare three different path planning algorithms
2. Optimize performance for a segmented map structure
3. Lay groundwork for future collision avoidance implementation


---

## **Motivation**

Establishing and maintaining a lunar base involves numerous challenges, including navigation in constrained environments. Robots operating in such environments require efficient path planning algorithms to minimize energy consumption and computation time while ensuring precise navigation. This project addresses these needs by developing and benchmarking path planning approaches tailored to the lunar base's unique constraints.

---

## **Features**

- **Path Planning**:
   - Custom A* Algorithm: An optimized implementation of A* with heuristics adapted for the lunar environment and segmented map structure
   - Built-in MATLAB A* Algorithm: Utilizing MATLAB's native A* implementation as a baseline
   - Modified A* with Geometric Constraints: Enhanced version incorporating line-of-sight constraints for improved navigation in the lunar environment

- **3D Lunar Base Map Creation**:
   - Generated using MATLAB to represent rail-based paths and modular super-adobe structures
   - Includes binary occupancy maps derived from the 3D map
   - Segmented map structure for improved computational efficiency

- **Benchmarking and Visualization**:
   - Comparative analysis of execution times and path lengths
   - Visualization of paths on binary occupancy map and 3D environment
   - Rover simulation along generated paths

- **Scalability**:
  - Modular design allows integration with future collision avoidance and task scheduling algorithms

---

## **Technical Approach**

### **Map Creation and Evolution**
- **Initial Approach**:
   - First iteration: Static 3D map created in SolidWorks and imported into MATLAB
   - Used for preliminary testing and proof of concept
   - Limitations included:
      - Difficult to modify or expand
      - Static nature restricted dynamic operations
      - Limited flexibility for testing different configurations


- **Current Implementation**:
   - Fully programmatic 3D map generation in MATLAB
   - Features modular and interconnected paths
   - Key components:
      - Super-adobe structures (SA) with rail-based paths
      - Main pathways connecting different sections
      - Junction points for path intersections
      - User-configurable parameters for easy modification
   Innovative Path Connection Strategy
      - Implemented directional connections to optimize path planning:
         - Super-adobe structures (SA) → Main paths (one-way connections)
         - Main paths ↛ Super-adobe structures (no direct connection)
         - Removal of redundant SA connections from MA and J nodes
         - One-way connections from SA to MA/J nodes
      - Benefits:
         - Preservation of path completeness while reducing search space
         - Prevents circular path dependencies
         - Reduces computational complexity during path planning
         - Optimizes search space for A* algorithm
   Map Parameters
      - Binary Occupancy Map:
         - Paths represented as traversable areas (0s)
         - Obstacles as non-traversable areas (1s)
      - Dimensions: 30x70 units
      - Robot step size: 0.1x0.1 unit
      - Visualization using MATLAB's patch objects

#### Map Data Structure:
```matlab
paths = Dictionary() containing:
- connections: Adjacent path segments
- coordinates: [n×3] array of path coordinates
- tangents: [n×3] array of path tangent vectors
- headings: Path direction indicators
- start_point: [x, y, z] coordinates
- end_point: [x, y, z] coordinates
- distance: Path segment length
```

### **Path Planning Algorithm**
- **Custom AStar Implementation**:
   - Our custom A* algorithm is optimized for the lunar base's segmented path structure, utilizing several key optimizations:
   - #### Key Optimizations:
      - Uses mid-points instead of endpoints for heuristic calculations to reduce computational overhead
      - Implements Manhattan distance heuristic to match the grid-based movement constraints
      - Leverages segment-based navigation (66 segments) vs. grid-based approach (2100 grids)
   - #### Pseudocode:
```
function CustomAstar(paths, start_node, end_node):
    // Pre-compute end coordinates
    end_coords = paths[end_node].start_point
    
    // Initialize path array
    final_path = zeros(ceil(length(paths)/4))
    path_idx = 1
    final_path[path_idx] = start_node
    current_node = start_node
    
    while true:
        // Get neighboring segments
        neighbours = paths[current_node].connections
        scores = zeros(length(neighbours))
        
        // Calculate Manhattan distance scores using mid-points
        for each neighbor in neighbours:
            scores[i] = manhattan_distance(
                paths[neighbor].mid_point, 
                end_coords
            )
        
        // Select best neighbor
        current_node = neighbours[argmin(scores)]
        final_path[++path_idx] = current_node
        
        // Check termination condition
        if distance_to_goal < threshold:
            break
            
    return final_path[1:path_idx]
```

- **Priority Queue Astar Implementation**:
   - This version enhances the basic A* algorithm with a priority queue data structure for improved performance.
   - #### Key Optimizations:
      - Uses mid-points instead of endpoints for heuristic calculations to reduce computational overhead
      - Implements Manhattan distance heuristic to match the grid-based movement constraints
      - Leverages segment-based navigation (66 segments) vs. grid-based approach (2100 grids)
   - #### Pseudocode:
```
function PriorityQueueAstar(paths, start_node, end_node):
    // Initialize data structures
    node_list = keys(paths)
    g_score = inf(n_nodes)
    f_score = inf(n_nodes)
    came_from = zeros(n_nodes)
    
    // Setup start node
    start_idx = node_to_idx[start_node]
    g_score[start_idx] = 0
    f_score[start_idx] = manhattan_distance(
        paths[start_node].end_point,
        paths[end_node].start_point
    )
    
    // Initialize priority queue
    open_set = PriorityQueue()
    open_set.insert(start_idx, f_score[start_idx])
    
    while !open_set.empty():
        current = open_set.pop()
        
        // Check goal condition
        if distance_to_goal < threshold:
            return reconstruct_path(came_from, current)
            
        // Process neighbors
        for neighbor in paths[current].connections:
            tentative_g = g_score[current] + paths[current].distance
            
            if tentative_g < g_score[neighbor]:
                // Update path
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + manhattan_distance(
                    paths[neighbor].mid_point,
                    paths[end_node].start_point
                )
                open_set.insert(neighbor, f_score[neighbor])
                
    return null  // No path found
```
- **Built-in MATLAB Astar**:
   - Utilizes MATLAB's Navigation Toolbox for baseline comparison, operating on a full grid representation rather than segments.
   
- **Key Implementation Notes:**:
   - Manhattan Distance Choice
      - Selected due to lunar base's orthogonal path structure
      - Perfectly matches vertical and horizontal movement constraints
      - Reduces computational complexity compared to Euclidean distance
   - Mid-point Optimization
      - Using mid-points instead of endpoints for heuristic calculations
      - Reduces computational overhead without significant impact on path quality
      - Particularly effective in segment-based approach
   - Segment-based Navigation
      - Reduces decision space from 2100 grids to 66 segments
      - Each segment contains pre-computed connectivity information
      - Significantly improves computational efficiency
---

### **Rover Simulation**

**Implementation Details**
- Rover modeled as a cuboid with dimensions:
  - Length: 0.5m
  - Width: 0.4m
  - Height: 0.2m
- Visualization using MATLAB's patch objects for 3D rendering

**Path Following Mechanics**
- Utilizes pre-computed tangent vectors stored in path data structure
- Smooth transitions achieved through:
  - Direct use of pre-computed path tangents for orientation
  - Automatic rotation calculation based on path direction
  - Position updates synchronized with orientation changes

**Key Features**
- Real-time visualization of rover movement along planned path
- Smooth transitions between path segments due to pre-computed tangent data
- Accurate representation of rover orientation using rotation matrices

**Implementation Benefits**
- Pre-computed tangent vectors eliminate need for runtime angle calculations
- Results in exceptionally smooth movement and natural-looking transitions
- Reduces computational overhead during simulation
- Enables realistic visualization of rover behavior on lunar base paths

---


### **Benchmarking and Results**

- Metrics: Execution time, path length, computational complexity
- Comparison of segment-based (66 segments, max 6 branches) vs. grid-based (2100 grids, 4 branches) approaches
- Performance analysis at different resolutions (1x1 and 0.1x0.1 grid sizes)

**Methodology**
- Number of test cases: 50 random start-end pairs
- Number of runs per pair: 10 runs
- Two resolution settings tested: 0.1 and 1.0
- Both real-time and CPU time measurements

**Performance Metrics**
- Execution time (real and CPU)
- Path length
- Success rate
- Statistical variance

**Visualization**

The comprehensive benchmarking approach ensures reliable performance comparison between the different path planning implementations.
| Algorithm | Avg. Execution Time (ms) | Avg. Path Length (units) | Computational Complexity |
|-----------|--------------------------|--------------------------|--------------------------|
| Custom A* |                          |                          | O(n log n)               |
| Built-in A* |                        |                          | O(n log n)               |
| Modified A* |                        |                          | O(n log n)               |

**Key Findings**
- Custom A* shows improved performance in segmented map structure
- Built-in A* serves as reliable baseline but lacks lunar-specific optimizations
- Modified A* demonstrates superior performance in constrained environments
- Segment-based approach significantly reduces decision space compared to grid-based method
- Performance improvements more pronounced at higher resolutions (0.1x0.1 grid)

---

## **Impact**

This project had a profound impact on my academic and professional growth:
- **Academic Recognition**: 
  - Earned top project recognition in a peer-reviewed class evaluation.
- **Professional Development**: 
  - Helped secure a position at the Space Robotics Lab at the University of Arizona.
- **Personal Growth**: 
  - Reinforced my passion for robotics and control systems through hands-on application and dynamic simulations.

---

## **Future Work**
   - Collision Avoidance: Enhance algorithms for dynamic obstacles and multi-robot scenarios
   - Task Planning: Implement genetic algorithm for optimal task allocation and scheduling
   - Hardware Integration: Test algorithms on physical robots in simulated lunar environments
   - Performance Optimization: Further investigate and optimize Custom A* implementation to fully leverage reduced decision space in segmented approach


---

## **Try It Yourself**

### **Prerequisites**
- MATLAB with Simulink (R2022a or later).
- Robotics Toolbox for MATLAB.

### **Setup**
1. Clone the repository:
   ```bash
   git clone https://github.com/adharsh-prasad/Robotics-Portfolio/tree/main/Lunar_Operations_Task_Planning
   ```
2. Open MATLAB and navigate to the project directory
3. Run the main script to generate maps, execute algorithms, and visualize results

### **Customization**
Modify parameters in the script files to explore different scenarios:
- Map dimensions and resolution
- Rover dimensions and movement characteristics
- Algorithm-specific parameters

### **Acknowledgments**
This project is part of ongoing research to optimize robotic operations for lunar base construction and maintenance. 
