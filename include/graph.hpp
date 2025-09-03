#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <cstdint>
#include <limits>

/**
 * @brief Represents a node in the graph with geographic coordinates
 * 
 * Each node has an identifier and geographic coordinates (latitude and longitude)
 * that allow for visualization and geographic-based computations.
 */
struct Node {
    int id;            ///< Unique identifier for the node
    double longitude;  ///< Geographic longitude of the node
    double latitude;   ///< Geographic latitude of the node
};

/**
 * @brief Represents an edge in the graph connecting two nodes
 * 
 * Each edge has a target node, a weight representing the cost of traversal,
 * and an optional contracted node used by contraction hierarchies.
 */
struct Edge {
    int to;            ///< Target node identifier
    double w;          ///< Edge weight (cost of traversal)
    int contracted = -1; ///< Node that was contracted to create this edge (-1 for original edges)
};

/**
 * @brief Graph data structure for road networks
 * 
 * This undirected graph implementation stores nodes with geographic coordinates
 * and weighted edges between nodes. It provides methods for adding and accessing
 * nodes and edges, as well as specialized methods for contraction hierarchies.
 */
class Graph {
public:
    /**
     * @brief Default constructor creating an empty graph
     */
    Graph() = default;
    
    /**
     * @brief Constructor creating a graph with a specified number of nodes
     * 
     * @param n Number of nodes in the graph
     */
    explicit Graph(int n) { init(n); }

    /**
     * @brief Initialize the graph with a specified number of nodes
     * 
     * Creates an empty graph with n nodes and no edges.
     * 
     * @param n Number of nodes
     */
    void init(int n) {
        adj.assign(n, {});
        nodes.assign(n, {-1, -1.0, -1.0});
    }

    /**
     * @brief Initialize a node with its identifier and coordinates
     * 
     * @param id Node identifier
     * @param latitude Geographic latitude
     * @param longitude Geographic longitude
     */
    void init_node(int id, double latitude, double longitude) {
        nodes[id].id = id;
        nodes[id].latitude = latitude;
        nodes[id].longitude = longitude;
    }

    /**
     * @brief Get a node by its identifier
     * 
     * @param id Node identifier
     * @return Node object (returns a dummy node with id=-1 if not found)
     */
    Node get_node(int id) const {
        if (id < 0 || id >= node_count()) return {-1, -1, -1};
        return nodes[id];
    }

    /**
     * @brief Get all nodes in the graph
     * 
     * @return Constant reference to the vector of all nodes
     */
    const std::vector<Node>& get_all_nodes() const {
        return nodes;
    }

    /**
     * @brief Get the number of nodes in the graph
     * 
     * @return Number of nodes
     */
    int node_count() const { return (int)nodes.size(); }

    /**
     * @brief Add an undirected edge between two nodes
     * 
     * @param u Source node identifier
     * @param v Target node identifier
     * @param w Edge weight
     * @param c Contracted node (-1 for original edges)
     */
    void add_edge(int u, int v, double w, int c = -1) {
        if (u == v) return;
        if (u < 0 || v < 0 || u >= node_count() || v >= node_count()) return;
        adj[u].push_back({v, w, c});
        adj[v].push_back({u, w, c});
    }

    /**
     * @brief Add a contraction hierarchy edge
     * 
     * Adds an edge for contraction hierarchies, handling existing edges
     * by updating them if the new edge provides a shorter path.
     * 
     * @param u Source node identifier
     * @param v Target node identifier
     * @param w Edge weight
     * @param c Contracted node that created this shortcut
     */
    void add_CH_edge(int u, int v, double w, int c) {
        if (u == v) return;
        if (u < 0 || v < 0 || u >= node_count() || v >= node_count()) return;
        int from = (adj[u].size() < adj[v].size()) ? u : v;
        int to = (from == u) ? v : u;
        for (auto &e : adj[from]) {
            if (e.to == to) {
                if (w < e.w) {
                    e.w = w;
                    for(auto &e2 : adj[to]) {
                        if (e2.to == from) {
                            e2.w = w;
                            e2.contracted = c;
                            break;
                        }
                    }
                }
                e.contracted = c;
                return;
            }
        }
        adj[u].push_back({v, w, c});
    }
    
    /**
     * @brief Check if an edge exists between two nodes
     * 
     * @param u First node identifier
     * @param v Second node identifier
     * @return true if the edge exists, false otherwise
     */
    bool has_edge(int u, int v) const {
        int shorter = (adj[u].size() < adj[v].size()) ? u : v;
        int longer = (shorter == u) ? v : u;
        for (const auto& e : adj[shorter]) {
            if (e.to == longer) return true;
        }
        return false;
    }

    /**
     * @brief Get the weight of an edge between two nodes
     * 
     * @param u First node identifier
     * @param v Second node identifier
     * @return Edge weight, or infinity if the edge doesn't exist
     */
    double get_weight(int u, int v) const {
        int shorter = (adj[u].size() < adj[v].size()) ? u : v;
        int longer = (shorter == u) ? v : u;
        for (const auto& e : adj[shorter]) {
            if (e.to == longer) return e.w;
        }
        return std::numeric_limits<double>::infinity();
    }

    /**
     * @brief Set the weight of an edge between two nodes
     * 
     * @param u First node identifier
     * @param v Second node identifier
     * @param val New weight value
     */
    void set_weight(int u, int v, double val) {
        for (auto& e : adj[u]) {
            if (e.to == v){
                e.w = val;
                break;
            }
        }
        for (auto& e : adj[v]) {
            if (e.to == u) {
                e.w = val;
                break;
            }
        }
    }

    /**
     * @brief Get all neighbors of a node
     * 
     * @param u Node identifier
     * @return Constant reference to the vector of edges connecting to neighbors
     */
    const std::vector<Edge>& neighbors(int u) const {
        return adj[u];
    }

    /**
     * @brief Get the total number of edges in the graph
     * 
     * @return Number of edges (accounting for the graph being undirected)
     */
    size_t get_num_of_edges() const {
        int count = 0;
        for (const auto& edges : adj) {
            count += edges.size();
        }
        return count / 2; // Each edge is counted twice
    }

private:
    std::vector<Node> nodes;  ///< Array of nodes in the graph
    std::vector<std::vector<Edge>> adj;  ///< Adjacency list representation of edges
};

#endif // GRAPH_HPP