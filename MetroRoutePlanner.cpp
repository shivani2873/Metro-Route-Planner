#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
using namespace std;

// Station & Edge Structs
struct Station {
    int id;
    string name;
    int line;
};

struct Edge {
    int destination;
    int distance; // in km
    int cost;     // in Rs
};

// ------------------ Dijkstra for Shortest Distance ------------------
vector<int> dijkstra(const vector<vector<Edge>>& graph, int source, int destination,
                     const vector<Station>& stations, int& finalDistance, int& finalCost) {
    int n = graph.size();
    vector<int> dist(n, numeric_limits<int>::max());
    vector<int> cost(n, 0);
    vector<int> parent(n, -1);

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;
    dist[source] = 0;
    pq.push({0, source});

    while (!pq.empty()) {
        auto top = pq.top();   
        int currWeight = top.first;
        int u = top.second;
        pq.pop();

        if (u == destination) break;

        for (auto& edge : graph[u]) {
            int v = edge.destination;
            int newDist = dist[u] + edge.distance;

            if (newDist < dist[v]) {
                dist[v] = newDist;
                cost[v] = cost[u] + edge.cost;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    // Reconstruct path
    vector<int> path;
    for (int cur = destination; cur != -1; cur = parent[cur]) path.push_back(cur);
    reverse(path.begin(), path.end());

    finalDistance = dist[destination];
    finalCost = cost[destination];
    return path;
}

// ------------------ BFS for Cheapest Cost ------------------
vector<int> bfsCheapest(const vector<vector<Edge>>& graph, int source, int destination,
                        const vector<Station>& stations, int& finalDistance, int& finalCost) {
    int n = graph.size();
    vector<int> cost(n, numeric_limits<int>::max());
    vector<int> dist(n, 0);
    vector<int> parent(n, -1);

    queue<int> q;
    cost[source] = 0;
    q.push(source);

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (auto& edge : graph[u]) {
            int v = edge.destination;
            int newCost = cost[u] + edge.cost;

            if (newCost < cost[v]) {
                cost[v] = newCost;
                dist[v] = dist[u] + edge.distance;
                parent[v] = u;
                q.push(v);
            }
        }
    }

    // Reconstruct path
    vector<int> path;
    for (int cur = destination; cur != -1; cur = parent[cur]) path.push_back(cur);
    reverse(path.begin(), path.end());

    finalDistance = dist[destination];
    finalCost = cost[destination];
    return path;
}

// ------------------ DFS for Least Hops ------------------
void dfsUtil(const vector<vector<Edge>>& graph, int current, int destination,
             vector<bool>& visited, vector<int>& path, vector<int>& bestPath,
             int& bestDistance, int& bestCost) {

    visited[current] = true;
    path.push_back(current);

    if (current == destination) {
        int totalDist = 0, totalCost = 0;
        for (int i = 1; i < path.size(); i++) {
            int u = path[i - 1], v = path[i];
            for (auto& edge : graph[u]) {
                if (edge.destination == v) {
                    totalDist += edge.distance;
                    totalCost += edge.cost;
                }
            }
        }
        if (bestPath.empty() || path.size() < bestPath.size()) {
            bestPath = path;
            bestDistance = totalDist;
            bestCost = totalCost;
        }
    }

    for (auto& edge : graph[current]) {
        if (!visited[edge.destination]) {
            dfsUtil(graph, edge.destination, destination, visited, path, bestPath, bestDistance, bestCost);
        }
    }

    visited[current] = false;
    path.pop_back();
}

vector<int> dfs(const vector<vector<Edge>>& graph, int source, int destination,
                const vector<Station>& stations, int& bestDistance, int& bestCost) {
    vector<bool> visited(graph.size(), false);
    vector<int> path, bestPath;
    dfsUtil(graph, source, destination, visited, path, bestPath, bestDistance, bestCost);
    return bestPath;
}

// ------------------ MAIN ------------------
int main() {
    int numStations = 7;
    vector<Station> stations = {
        {0, "Station A", 1}, {1, "Station B", 1}, {2, "Station C", 2},
        {3, "Station D", 2}, {4, "Station E", 3}, {5, "Station F", 3},
        {6, "Station G", 1}
    };

    vector<vector<Edge>> graph(numStations);

    // Graph edges (bidirectional)
    graph[0].push_back({1, 10, 50}); graph[1].push_back({0, 10, 50});
    graph[0].push_back({2, 20, 30}); graph[2].push_back({0, 20, 30});
    graph[1].push_back({2, 5, 10});  graph[2].push_back({1, 5, 10});
    graph[1].push_back({3, 15, 40}); graph[3].push_back({1, 15, 40});
    graph[2].push_back({3, 10, 25}); graph[3].push_back({2, 10, 25});
    graph[2].push_back({4, 20, 50}); graph[4].push_back({2, 20, 50});
    graph[3].push_back({5, 20, 30}); graph[5].push_back({3, 20, 30});
    graph[4].push_back({5, 10, 20}); graph[5].push_back({4, 10, 20});
    graph[4].push_back({6, 5, 10});  graph[6].push_back({4, 5, 10});
    graph[5].push_back({6, 15, 20}); graph[6].push_back({5, 15, 20});

    int source = 4;       // Station E
    int destination = 1;  // Station B

    // Shortest Path (minimize distance)
    int shortestDist, shortestCost;
    vector<int> shortestPath = dijkstra(graph, source, destination, stations, shortestDist, shortestCost);
    cout << "Shortest Path (Distance: " << shortestDist << ", Cost: " << shortestCost << "): ";
    for (int i = 0; i < shortestPath.size(); i++) {
        cout << stations[shortestPath[i]].name << (i == shortestPath.size()-1 ? "" : " -> ");
    }
    cout << "\n";

    // Cheapest Path (minimize cost using BFS)
    int cheapestDist, cheapestCost;
    vector<int> cheapestPath = bfsCheapest(graph, source, destination, stations, cheapestDist, cheapestCost);
    cout << "Cheapest Path (Distance: " << cheapestDist << ", Cost: " << cheapestCost << "): ";
    for (int i = 0; i < cheapestPath.size(); i++) {
        cout << stations[cheapestPath[i]].name << (i == cheapestPath.size()-1 ? "" : " -> ");
    }
    cout << "\n";

    // Best Path (using DFS, by hops)
    int bestDist, bestCost;
    vector<int> bestPath = dfs(graph, source, destination, stations, bestDist, bestCost);
    cout << "Best Path (Distance: " << bestDist << ", Cost: " << bestCost << "): ";
    for (int i = 0; i < bestPath.size(); i++) {
        cout << stations[bestPath[i]].name << (i == bestPath.size()-1 ? "" : " -> ");
    }
    cout << "\n";

    return 0;
}
