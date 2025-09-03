#ifndef CUSTOMIZABLE_CONTRACTION_HIERARCHY_HPP
#define CUSTOMIZABLE_CONTRACTION_HIERARCHY_HPP

#include "graph.hpp"
#include "dijkstra.hpp"
#include <vector>
#include <unordered_map>
#include <string>
#include <limits>
#include <chrono>
#include <random>
#include <algorithm>
#include <memory>

#define CCH CustomizableContractionHierarchy

#define NOW() std::chrono::high_resolution_clock::now()
#define TO_MILI(x) std::chrono::duration_cast<std::chrono::milliseconds>(x)
#define TO_MICRO(x) std::chrono::duration_cast<std::chrono::microseconds>(x)

/**
 * @brief Hash functor for pairs to use in unordered containers
 */
struct PairHash {
    size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ std::hash<int>()(p.second);
    }
};



/**
 * @brief Customizable Contraction Hierarchy (CCH) implementation
 * 
 * CCH is a route planning technique that separates preprocessing into metric-independent 
 * and metric-dependent phases, allowing for fast weight customization.
 * 
 * This implementation follows a 3-phase approach:
 * - PHASE I:   Metric-independent preprocessing (all-in shortcuts, triangle enumeration)
 * - PHASE II:  Metric-dependent customization (weight computation via triangle relaxation)
 * - PHASE III: Query answering (bidirectional CH-Dijkstra)
 * 
 * The implementation uses KaHIP for generating high-quality nested dissection orders.
 */
class CustomizableContractionHierarchy {
public:
    /**
     * @brief Operating system enum for path handling in external tool calls
     */
    enum class OS {
        WINDOWS,  ///< Windows OS (uses WSL for KaHIP)
        LINUX     ///< Linux OS (direct execution)
    };

    /**
     * @brief Structure to represent a triangle in the graph
     * 
     * Triangles are fundamental to CCH customization and consist of three nodes:
     * two base nodes (u,v) and an apex node (w) with higher rank.
     */
    struct Triangle {
        int u, v, w;                         ///< Vertices forming the triangle
        double weight_uv, weight_uw, weight_vw; ///< Edge weights within the triangle
    };

    /**
     * @brief Structure to represent an edge with associated triangles
     * 
     * Each CCH edge maintains a list of triangles that it forms the base of,
     * which allows for efficient customization.
     */
    struct CCHEdge {
        int to;                      ///< Target vertex
        double weight;               ///< Edge weight
        std::vector<int> triangles;  ///< Indices to lower triangles this edge is part of
    };

    /**
     * @brief Constructor for CCH with external ordering tool configuration
     * 
     * @param g Original graph to build CCH on
     * @param kahipPath Path to the KaHIP node ordering executable
     * @param graphFilePath Path where graph will be written for KaHIP
     * @param orderingFilePath Path where node ordering will be stored
     * @param operatingSystem OS type for proper command execution
     */
    CustomizableContractionHierarchy(
        const Graph& g,
        const std::string& kahipPath,
        const std::string& graphFilePath,
        const std::string& orderingFilePath,
        OS operatingSystem = OS::WINDOWS
    );
    
    /**
     * @brief PHASE I: Performs the metric-independent preprocessing
     * 
     * This phase consists of several steps:
     * 1. Compute a nested dissection order using KaHIP
     * 2. Build the CCH structure with all-in shortcuts
     * 3. Identify and store all lower triangles with each shortcut
     * 4. Sort shortcuts by the highest rank of any of their lower triangles
     * 
     * This preprocessing is done once and reused across all customizations.
     */
    void preprocess();
    
    /**
     * @brief PHASE II: Customizes the CCH with current edge weights
     * 
     * This phase assigns weights to the shortcuts based on the original graph weights:
     * 1. Assign original weights to all edges
     * 2. Process shortcuts in order of highest rank of their lower triangles
     * 3. For each shortcut, relax via all its lower triangles
     * 
     * This customization is very fast and can be repeated for different metrics.
     */
    void customize();
    
    /**
     * @brief PHASE II: Customizes the CCH with new random weights for specified edges
     * 
     * This version assigns random weights to specified edges before customization.
     * 
     * @param edge_weights Pairs of node IDs representing edges to assign random weights to
     */
    void customize(const std::vector<std::pair<int, int>>& edge_weights);
    
    /**
     * @brief PHASE III: Performs a shortest path query between two nodes
     * 
     * Executes a bidirectional CH-Dijkstra search:
     * 1. Upward search from source node
     * 2. Downward search from target node visiting only nodes seen in upward phase
     * 3. Path reconstruction
     * 
     * @param s Source node ID
     * @param t Target node ID
     * @return DijkstraResult containing distance, path, and statistics
     */
    DijkstraResult query(int s, int t);
    
    /**
     * @brief Gets the total number of edges in the CCH structure
     * 
     * @return Number of edges in the hierarchy
     */
    size_t get_num_of_edges() const;
    
    /**
     * @brief Gets the number of shortcuts added during preprocessing
     * 
     * @return Number of shortcut edges
     */
    size_t get_num_of_shortcuts() const;
    
    /**
     * @brief Gets the average number of triangles per edge
     * 
     * This is a metric for the complexity of the customization phase.
     * 
     * @return Average number of triangles per edge
     */
    double get_avg_triangles_per_edge() const;
    
    /**
     * @brief Gets the maximum number of triangles for any edge
     * 
     * @return Maximum triangle count for any edge
     */
    size_t get_max_triangles_per_edge() const;
    
    /**
     * @brief Gets the time taken for preprocessing
     * 
     * @return Preprocessing time in milliseconds
     */
    std::chrono::milliseconds get_preprocessing_time() const;
    
    /**
     * @brief Gets the time taken for customization
     * 
     * @return Customization time in microseconds
     */
    std::chrono::microseconds get_customization_time() const;
    
    /**
     * @brief Exports all edges in the CCH for visualization or analysis
     * 
     * @return Vector of tuples (source, target, weight)
     */
    std::vector<std::tuple<int,int,double>> export_all_edges() const;
    
private:
    Graph original_graph;      ///< Original graph with current weights
    int n;                     ///< Number of nodes
    
    // Path configuration for external tools
    std::string kahip_path;         ///< Path to KaHIP executable
    std::string graph_file_path;    ///< Path to write graph for KaHIP
    std::string ordering_file_path; ///< Path to read/write node ordering
    OS os_type;                     ///< Operating system type
    
    // CCH data structures
    std::vector<int> rank;                   ///< Node ordering (rank[v] = position of v)
    std::vector<std::vector<CCHEdge>> up;    ///< Upward edges in the hierarchy
    std::vector<std::vector<CCHEdge>> down;  ///< Downward edges in the hierarchy
    std::vector<Triangle> triangles;         ///< All triangles in the CCH
    std::vector<std::pair<int, int>> sorted_shortcuts; ///< Shortcuts sorted by highest rank of their triangles
    
    // Statistics
    size_t num_shortcuts;                       ///< Number of shortcuts added
    std::chrono::milliseconds preprocessing_time; ///< Time taken for preprocessing
    std::chrono::microseconds customization_time; ///< Time taken for customization
    
    // PHASE I methods: metric-independent preprocessing
    /**
     * @brief Computes a nested dissection ordering using KaHIP
     * 
     * Nested dissection produces an ordering that creates a good separation of the graph,
     * which is essential for minimizing the number of shortcuts.
     * 
     * @return Vector of node IDs in the computed order
     */
    std::vector<int> compute_nested_dissection_order();
    
    /**
     * @brief Writes the graph to a file in KaHIP format
     * 
     * @param filename Path to write the graph file
     * @return True if successful, false otherwise
     */
    bool writeGraphForKaHIP(const std::string& filename);
    
    /**
     * @brief Runs the KaHIP node ordering algorithm
     * 
     * @param inputGraphFile Path to the input graph file
     * @param outputOrderingFile Path to write the ordering
     * @return True if successful, false otherwise
     */
    bool runKaHIPNodeOrdering(const std::string& inputGraphFile, const std::string& outputOrderingFile);
    
    /**
     * @brief Reads a node ordering from a file
     * 
     * @param filename Path to read the ordering from
     * @return Vector of node IDs in the read order
     */
    std::vector<int> readNodeOrdering(const std::string& filename);
    
    /**
     * @brief Builds the CCH structure using the given node ordering
     * 
     * Creates the upward and downward edge arrays based on the node ranking.
     * 
     * @param order Vector of node IDs in the desired order
     */
    void build_cch(const std::vector<int>& order);
    
    /**
     * @brief Identifies all lower triangles in the hierarchy
     * 
     * A lower triangle consists of two nodes u, v with an edge between them
     * and a higher-ranked node w that is adjacent to both u and v.
     * This method identifies all such triangles in the hierarchy.
     */
    void identify_lower_triangles();
    
    /**
     * @brief Sorts shortcuts by the highest rank of their lower triangles
     * 
     * This ordering is essential for efficient customization, as it ensures
     * that weights are computed in the correct order.
     */
    void sort_shortcuts_by_highest_rank();
    
    /**
     * @brief Converts file paths between Windows and WSL format
     * 
     * @param path Windows path to convert
     * @return Converted path for WSL access
     */
    std::string convertPath(const std::string& path);
    
    // PHASE II methods: metric-dependent customization
    /**
     * @brief Performs basic customization (copying original weights)
     * 
     * Assigns weights from the original graph to all edges in the hierarchy.
     */
    void basic_customization();
    
    /**
     * @brief Processes shortcuts in pre-sorted order
     * 
     * For each shortcut, relaxes it via all its lower triangles to find
     * the shortest path that it represents.
     */
    void process_shortcuts_in_order();
    
    // PHASE III: Query helper methods
    /**
     * @brief Query data structure for storing search state
     */
    struct QueryData {
        std::vector<double> dist;     ///< Distance from source/target
        std::vector<int> parent;      ///< Parent pointers for path reconstruction
        std::vector<bool> visited;    ///< Visited flags for search
    };
    
    /**
     * @brief Performs upward search from source node
     * 
     * @param s Source node ID
     * @param forward Reference to forward search state
     */
    void upward_search(int s, QueryData& forward);
    
    /**
     * @brief Performs downward search from target node
     * 
     * @param t Target node ID
     * @param forward Reference to completed forward search
     * @param backward Reference to backward search state
     */
    void downward_search(int t, const QueryData& forward, QueryData& backward);
    
    /**
     * @brief Reconstructs the shortest path from search results
     * 
     * @param s Source node ID
     * @param t Target node ID
     * @param backward Reference to backward search state
     * @return Vector of nodes representing the path
     */
    std::vector<Node> reconstruct_path(int s, int t, const QueryData& backward);
};

#endif // CUSTOMIZABLE_CONTRACTION_HIERARCHY_HPP