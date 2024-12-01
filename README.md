# README.md  

# Drone Path Planning and Graph Generation Project  

## Description  
This project demonstrates an implementation of a drone-based path-planning system using Dijkstra's algorithm. It generates a graph of nodes based on feasible travel ranges between locations and utilizes priority queues for optimal route calculations. The application integrates tools like Python's pyproj for geodesic calculations, Pydeck for visualization, and the OSMPythonTools library for querying OpenStreetMap data.  

The system calculates travel paths for a drone, taking into account its battery life, speed, and charging requirements. It provides visualizations and detailed reports, showcasing optimal paths between lighthouses within specified geographic boundaries.

---

## Features  
- **Graph Generation**: Automatically constructs a graph based on a drone's travel feasibility between nodes (lighthouses).  
- **Path Planning**: Implements Dijkstra's algorithm for finding the shortest paths between specified locations.  
- **Drone Simulation**: Simulates drone behavior, including battery life, speed, and recharge time.  
- **Data Visualization**: Creates HTML-based visualizations using Pydeck, rendering nodes, paths, and routes.  
- **Comprehensive Reports**: Provides detailed logs, including distances, flight times, rest times, and total travel duration for routes.  

---

## Requirements  
To run the project, install the following dependencies:  

```bash  
pip install pyproj pandas OSMPythonTools pydeck  
```  

---

## How to Run  
1. **Clone the Repository**:  
   Download or clone the project repository to your local machine.  

2. **Install Dependencies**:  
   Ensure all required libraries are installed using the provided command.  

3. **Run the Program**:  
   Execute the script to generate graphs, compute paths, and visualize the results.  

   ```bash  
   python <script_name>.py  
   ```  

4. **View Visualization**:  
   Open the generated `path_layer.html` file in a browser to view the visualized map and routes.  

---

## Key Classes and Functionalities  

### 1. **GenerateGraph**  
Creates a graph based on geolocation data and calculates feasible connections between nodes for the drone.  
- Queries data using the OpenStreetMap API.  
- Computes distances between nodes using geodesic calculations.  

### 2. **Graph, Edge, and Drone**  
- **Graph**: Stores nodes and edges representing locations and their connections.  
- **Edge**: Represents a connection between two locations, including distance and vertex data.  
- **Drone**: Models drone attributes such as speed, battery life, and charging requirements, determining travel feasibility.  

### 3. **PriorityQueue**  
Implements a priority queue to support Dijkstra's algorithm for path optimization.  

### 4. **Algorithms**  
Includes Dijkstra's algorithm for shortest path computation.  

### 5. **DrawGraph**  
Visualizes the generated graph and paths using Pydeck, rendering points and routes on an interactive map.  

---

## Outputs  

### **Visualization**:  
- **Orange Dots**: Named lighthouses.  
- **Blue Dots**: Unnamed lighthouses.  
- **Randomized Color Lines**: Different paths between nodes.  

### **Report**:  
- Total distance and time for each path.  
- Number of stops, flight times, and rest times.  

---

## Additional Notes  

- The project is designed for educational and research purposes, showcasing the application of graph algorithms and visualization tools in real-world scenarios.  
- Modify the bounding box and query parameters in the `Submission` class to adapt the system to other regions or node types.  

---

## Future Enhancements  
- Add support for variable drone specifications (e.g., different speeds or battery capacities).  
- Include real-time data updates for dynamic graph generation.  
- Extend visualizations with additional layers for advanced insights.  

---  

## Author  
Ronald Baker  
[Contact Information/Links]  

Feel free to reach out with questions or suggestions for improvement.