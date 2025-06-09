#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <limits>
#include <algorithm>

using namespace std;

// Structure to represent a station
struct Station {
    int id;
    string name;
    int line; // Line number to which the station belongs
    // You can include additional attributes like latitude, longitude, etc. if needed
};

// Structure to represent an edge between two stations
struct Edge {
    int source;
    int destination;
    int distance;
    int cost;
    // You can include additional attributes like travel time, transportation mode, etc. if needed
};

// Function to find the shortest path using Dijkstra's algorithm
vector<int> dijkstra(const vector<vector<Edge>>& graph, int source, int destination, vector<Station>& stations, int& shortestDistance, int& shortestCost) {
    int numStations = graph.size();
    vector<int> distance(numStations, numeric_limits<int>::max());
    vector<int> cost(numStations, numeric_limits<int>::max());
    vector<int> parent(numStations, -1);
    vector<bool> visited(numStations, false);

    distance[source] = 0;
    cost[source] = 0;

    for (int i = 0; i < numStations - 1; ++i) {
        int minDistance = numeric_limits<int>::max();
        int currentStation = -1;

        // Find the station with the minimum distance from the set of unvisited stations
        for (int j = 0; j < numStations; ++j) {
            if (!visited[j] && distance[j] < minDistance) {
                minDistance = distance[j];
                currentStation = j;
            }
        }

        if (currentStation == -1)
            break;

        visited[currentStation] = true;

        // Update the distance and cost of neighboring stations
        for (const Edge& edge : graph[currentStation]) {
            int neighbor = edge.destination;
            int newDistance = distance[currentStation] + edge.distance;
            int newCost = cost[currentStation] + edge.cost;

            if (!visited[neighbor] && newDistance < distance[neighbor]) {
                distance[neighbor] = newDistance;
                cost[neighbor] = newCost;
                parent[neighbor] = currentStation;
            }
        }
    }

    // Reconstruct the shortest path
    vector<int> path;
    int current = destination;
    shortestDistance = distance[destination];
    shortestCost = cost[destination];
    while (current != -1) {
        path.push_back(current);
        current = parent[current];
    }
    reverse(path.begin(), path.end());

    return path;
}

// Function to find the cheapest path using BFS algorithm
vector<int> bfs(const vector<vector<Edge>>& graph, int source, int destination, vector<Station>& stations, int& cheapestDistance, int& cheapestCost) {
    int numStations = graph.size();
    vector<int> distance(numStations, numeric_limits<int>::max());
    vector<int> cost(numStations, numeric_limits<int>::max());
    vector<int> parent(numStations, -1);

    distance[source] = 0;
    cost[source] = 0;

    queue<int> q;
    q.push(source);

    while (!q.empty()) {
        int currentStation = q.front();
        q.pop();

        for (const Edge& edge : graph[currentStation]) {
            int neighbor = edge.destination;
            int newDistance = distance[currentStation] + edge.distance;
            int newCost = cost[currentStation] + edge.cost;

            if (newDistance < distance[neighbor] || (newDistance == distance[neighbor] && newCost < cost[neighbor])) {
                distance[neighbor] = newDistance;
                cost[neighbor] = newCost;
                parent[neighbor] = currentStation;
                q.push(neighbor);
            }
        }
    }

    // Reconstruct the cheapest path
    vector<int> path;
    int current = destination;
    cheapestDistance = distance[destination];
    cheapestCost = cost[destination];
    while (current != -1) {
        path.push_back(current);
        current = parent[current];
    }
    reverse(path.begin(), path.end());

    return path;
}

// Function to find the best path using DFS algorithm
void dfsUtil(const vector<vector<Edge>>& graph, int current, int destination, vector<bool>& visited, vector<int>& path, vector<int>& bestPath, int& bestDistance, int& bestCost) {
    visited[current] = true;
    path.push_back(current);

    if (current == destination && (bestPath.empty() || path.size() < bestPath.size())) {
        bestPath = path;
        bestDistance = 0;
        bestCost = 0;
        for (int i = 1; i < path.size(); ++i) {
            int source = path[i - 1];
            int dest = path[i];
            for (const Edge& edge : graph[source]) {
                if (edge.destination == dest) {
                    bestDistance += edge.distance;
                    bestCost += edge.cost;
                    break;
                }
            }
        }
    }

    for (const Edge& edge : graph[current]) {
        int neighbor = edge.destination;
        if (!visited[neighbor]) {
            dfsUtil(graph, neighbor, destination, visited, path, bestPath, bestDistance, bestCost);
        }
    }

    visited[current] = false;
    path.pop_back();
}

vector<int> dfs(const vector<vector<Edge>>& graph, int source, int destination, vector<Station>& stations, int& bestDistance, int& bestCost) {
    int numStations = graph.size();
    vector<bool> visited(numStations, false);
    vector<int> path;
    vector<int> bestPath;

    dfsUtil(graph, source, destination, visited, path, bestPath, bestDistance, bestCost);

    return bestPath;
}

int main() {
    // Example graph representing the hypothetical Delhi Metro station network
    int numStations = 7;
    vector<vector<Edge>> graph(numStations);
    vector<Station> stations = {
        {0, "Station A", 1},
        {1, "Station B", 1},
        {2, "Station C", 2},
        {3, "Station D", 2},
        {4, "Station E", 3},
        {5, "Station F", 3},
        {6, "Station G", 1}
    };

    // Add edges between stations with corresponding distances and costs
    graph[0].push_back({0, 1, 10, 50});
    graph[0].push_back({0, 2, 20, 30});
    graph[1].push_back({1, 2, 5, 10});
    graph[1].push_back({1, 3, 15, 40});
    graph[2].push_back({2, 1, 5, 20});
    graph[2].push_back({2, 3, 10, 25});
    graph[2].push_back({2, 4, 20, 50});
    graph[3].push_back({3, 2, 10, 10});
    graph[3].push_back({3, 5, 20, 30});
    graph[4].push_back({4, 2, 20, 30});
    graph[4].push_back({4, 5, 10, 20});
    graph[4].push_back({4, 6, 5, 10});
    graph[5].push_back({5, 3, 20, 10});
    graph[5].push_back({5, 4, 10, 10});
    graph[5].push_back({5, 6, 15, 20});

    int source = 4; // Source station ID
    int destination = 1; // Destination station ID

    // Find the shortest path
    int shortestDistance;
    int shortestCost;
    vector<int> shortestPath = dijkstra(graph, source, destination, stations, shortestDistance, shortestCost);
    cout << "Shortest Path (Distance: " << shortestDistance << ", Cost: " << shortestCost << "): ";
    for (int i = 0; i < shortestPath.size(); ++i) {
        cout << stations[shortestPath[i]].name;
        if (i != shortestPath.size() - 1)
            cout << " -> ";
    }
    cout << endl;

    // Find the cheapest path
    int cheapestDistance;
    int cheapestCost;
    vector<int> cheapestPath = bfs(graph, source, destination, stations, cheapestDistance, cheapestCost);
    cout << "Cheapest Path (Distance: " << cheapestDistance << ", Cost: " << cheapestCost << "): ";
    for (int i = 0; i < cheapestPath.size(); ++i) {
        cout << stations[cheapestPath[i]].name;
        if (i != cheapestPath.size() - 1)
            cout << " -> ";
    }
    cout << endl;

    // Find the best path
    int bestDistance;
    int bestCost;
    vector<int> bestPath = dfs(graph, source, destination, stations, bestDistance, bestCost);
    cout << "Best Path (Distance: " << bestDistance << ", Cost: " << bestCost << "): ";
    for (int i = 0; i < bestPath.size(); ++i) {
        cout << stations[bestPath[i]].name;
        if (i != bestPath.size() - 1)
            cout << " -> ";
    }
    cout << endl;

    return 0;
}
