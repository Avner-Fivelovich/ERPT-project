#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <vector>
#include <queue>
#include <limits>
#include <functional>
#include <algorithm>
#include "graph.hpp"

/**
 * @brief Type alias for priority queue elements in Dijkstra's algorithm
 * 
 * Pairs (distance, node_id) used in the priority queue, sorted by distance
 */
using QN = std::pair<double, int>;

/**
 * @brief Type alias for the priority queue used in Dijkstra's algorithm
 * 
 * Min-heap priority queue that returns the node with the smallest distance first
 */
using PQ = std::priority_queue<QN, std::vector<QN>, std::greater<QN>>;

/**
 * @brief Structure to hold the results of Dijkstra's algorithm
 * 
 * Contains the distance array, the shortest path, and performance statistics
 */
struct DijkstraResult {
    std::vector<double> dist;   ///< Distance from source to each node
    std::vector<Node> path;     ///< Shortest path from source to target
    size_t nodes_popped = 0;    ///< Number of nodes popped from priority queue (performance metric)
};

/**
 * @brief Implementation of Dijkstra's shortest path algorithm
 * 
 * This class provides a static implementation of Dijkstra's algorithm
 * for finding shortest paths in a weighted graph.
 */
class Dijkstra {
public:
    /**
     * @brief Reconstruct a path from parent pointers
     * 
     * Given a graph and an array of parent pointers, reconstructs the
     * shortest path from source to target.
     * 
     * @param g The graph
     * @param parent Array of parent pointers
     * @param source Source node identifier
     * @param target Target node identifier
     * @return Vector of nodes representing the path
     */
    static std::vector<Node> get_path_from_parent(const Graph& g, const std::vector<int>& parent, int source, int target) {
        std::vector<Node> path;
        if (target == -1) return path;

        for (int v = target; v != -1; v = parent[v]) {
            path.push_back(g.get_node(v));
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    /**
     * @brief Run Dijkstra's algorithm to find shortest paths
     * 
     * Computes shortest paths from a source node to either a single target
     * or to all nodes in the graph.
     * 
     * @param g The graph
     * @param source Source node identifier
     * @param target Target node identifier (-1 to compute paths to all nodes)
     * @return DijkstraResult containing distances, path, and performance metrics
     */
    static DijkstraResult run(const Graph& g, int source, int target = -1) {
        const int n = g.node_count();
        std::vector<double> dist(n, INF);  // Initialize distances to infinity
        std::vector<int> parent(n, -1);    // Initialize parent pointers
        size_t nodes_popped = 0;           // Counter for performance measurement
        
        PQ pq;                             // Priority queue for Dijkstra's algorithm
        dist[source] = 0;                  // Distance to source is 0
        pq.push({0, source});              // Start from the source
        
        // Main Dijkstra loop
        while (!pq.empty()) {
            auto [d, u] = pq.top(); pq.pop(), nodes_popped++;
            if (d != dist[u]) continue;    // Skip outdated entries
            if (target >= 0 && u == target) break;  // Early termination if target is reached
            
            // Relax edges from current node
            for (auto &e : g.neighbors(u)) {
                int v = e.to;
                double nd = d + e.w;  // New distance through current node
                if (nd < dist[v]) {   // If we found a shorter path
                    dist[v] = nd;     // Update distance
                    parent[v] = u;    // Update parent
                    pq.push({nd, v}); // Add to priority queue
                }
            }
        }
        
        return {dist, get_path_from_parent(g, parent, source, target), nodes_popped};
    }

    /// Constant representing infinity for distance calculations
    static constexpr double INF = std::numeric_limits<double>::infinity();
};

#endif // DIJKSTRA_HPP