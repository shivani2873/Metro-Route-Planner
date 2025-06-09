
# Metro-Route-Planner

## Project Overview

The **Metro Route Planner** is a C++ based application built to help users identify optimal routes between metro stations. It incorporates various graph traversal algorithms to deliver the shortest, most economical, and overall best routes based on criteria such as distance and fare. This application can be highly beneficial for daily commuters or tourists navigating through a metro network.

## Key Features

* **Shortest Route**: Calculates the minimum distance route using Dijkstra’s algorithm.
* **Lowest Cost Route**: Applies the Breadth-First Search (BFS) algorithm to find the route with the lowest fare.
* **Optimal Route**: Uses Depth-First Search (DFS) to evaluate and choose the best route considering multiple factors.

## Algorithms Implemented

1. **Dijkstra’s Algorithm** – Determines the shortest distance between two stations.
2. **Breadth-First Search (BFS)** – Finds the most cost-effective route.
3. **Depth-First Search (DFS)** – Selects the most optimal route based on combined metrics.

## Usage Instructions

### Sample Metro Map

The planner works on a sample network with metro stations including:

* Station A
* Station B
* Station C
* Station D
* Station E
* Station F
* Station G

### How to Run

Compile and execute the C++ source file using the terminal:

```bash
g++ -o metro_route_planner metro_route_planner.cpp
./metro_route_planner
```

Once executed, the application will display routes based on user input for source and destination stations.

### Sample Output

```
Shortest Path (Distance: 20, Cost: 80): Station E -> Station C -> Station B  
Cheapest Path (Distance: 20, Cost: 60): Station E -> Station C -> Station B  
Best Path (Distance: 20, Cost: 60): Station E -> Station C -> Station B
```

## Code Organization

* **Station Class**: Stores station details such as ID, name, and metro line.
* **Edge Class**: Represents connections between stations including metrics like distance and fare.
* **dijkstra()**: Implements Dijkstra’s logic to find shortest distance.
* **bfs()**: Uses BFS for lowest cost traversal.
* **dfs()**: Employs DFS to determine an optimal route.

## Potential Improvements

* Integrate real-time metro scheduling or delay data.
* Include additional edge/station attributes like time of travel and transfer stations.
* Build a graphical interface to enhance usability and user experience.

## Contribution Guidelines
Contributions are welcome! Please fork the repository and submit a pull request with your improvements.
