#ifndef UNIFIED_CONTRACTION_HIERARCHY_HPP
#define UNIFIED_CONTRACTION_HIERARCHY_HPP

#include <vector>
#include <queue>
#include <cstdint>
#include <limits>
#include <optional>
#include <iostream>
#include <unordered_set>
#include <random>
#include "dijkstra.hpp"

/**
 * @brief Implementation of Contraction Hierarchies with regular and POI-based modes
 * 
 * Unified Contraction Hierarchy (UCH) provides two operation modes:
 * 1. REGULAR_CH: Standard contraction hierarchy for fastest point-to-point routing
 * 2. POI_CH: Specialized contraction that prioritizes Points of Interest (POIs)
 *    for efficiently finding paths that visit at least one POI
 * 
 * The class supports preprocessing, querying, and POI-specific operations.
 */
class UnifiedContractionHierarchy {
public:
    /**
     * @brief Operation modes for the contraction hierarchy
     */
    enum Mode {
        REGULAR_CH,  ///< Standard Contraction Hierarchy
        POI_CH       ///< Contraction Hierarchy with POI support
    };

    /**
     * @brief Constructor with mode selection
     * 
     * @param g Original graph to build hierarchy on
     * @param mode Operation mode (REGULAR_CH or POI_CH)
     */
    explicit UnifiedContractionHierarchy(const Graph& g, Mode mode = REGULAR_CH);

    /**
     * @brief Preprocesses the graph according to the selected mode
     * 
     * Builds the contraction hierarchy by:
     * 1. Computing node contraction order based on selected mode
     * 2. Contracting nodes in that order, adding shortcuts as needed
     * 3. Building the final hierarchy structure
     */
    void preprocess();
    
    /**
     * @brief Unified query function that uses the appropriate algorithm based on mode
     * 
     * @param s Source node ID
     * @param t Target node ID
     * @return DijkstraResult containing distance, path, and statistics
     */
    DijkstraResult query(int s, int t);
    
    /**
     * @brief Regular CH bidirectional query implementation
     * 
     * Performs standard bidirectional Dijkstra search in the contraction hierarchy.
     * 
     * @param s Source node ID
     * @param t Target node ID
     * @return DijkstraResult containing distance, path, and statistics
     */
    DijkstraResult CH_query(int s, int t);
    
    /**
     * @brief Randomly assigns nodes as POIs (Points of Interest)
     * 
     * @param percentage Percentage of nodes to designate as POIs (default: 5%)
     */
    void assignRandomPOIs(double percentage = 5.0);
    
    /**
     * @brief Finds shortest path that visits at least one POI
     * 
     * @param s Source node ID
     * @param t Target node ID
     * @return DijkstraResult containing distance, path, and statistics
     */
    DijkstraResult findShortestPathViaPOI(int s, int t);
    
    /**
     * @brief Gets the set of POI node IDs
     * 
     * @return Constant reference to the POI node set
     */
    const std::unordered_set<int>& getPOIs() const { return poi_nodes; }
    
    /**
     * @brief Gets a sorted list of POI node IDs
     * 
     * @return Vector of POI node IDs
     */
    std::vector<int> getPOIList() const;
    
    /**
     * @brief Infinity value for distance calculations
     */
    static constexpr double INF = std::numeric_limits<double>::infinity();
    
    /**
     * @brief Gets the total number of edges in the hierarchy
     * 
     * @return Number of edges (including shortcuts)
     */
    size_t get_num_of_edges() const;
    
    /**
     * @brief Gets the number of nodes in the graph
     * 
     * @return Node count
     */
    int node_count() const { return n; }
    
    /**
     * @brief Gets the underlying graph with shortcuts
     * 
     * @return Constant reference to the graph
     */
    const Graph& underlying_graph() const { return ch_graph; }
    
    /**
     * @brief Expands a shortcut path to its full underlying path
     * 
     * @param path Path with possible shortcuts
     * @param weight Expected total path weight (used for optimization)
     * @return Expanded path with all intermediate nodes
     */
    std::vector<Node> expand_shortcut_path(const std::vector<Node>& path, double weight) const;
    
    /**
     * @brief Exports all edges for visualization or analysis
     * 
     * @return Vector of tuples (source, target, weight, contracted_node)
     */
    std::vector<std::tuple<int,int,double,int>> export_all_edges() const;
    
    /**
     * @brief Prints debug information about the hierarchy
     * 
     * @param os Output stream to print to
     */
    void print_debug(std::ostream& os) const;
    
    /**
     * @brief Prints debug information to standard output
     */
    void print_debug() const;

private:
    /**
     * @brief Internal structure for representing a shortcut
     */
    struct Shortcut {
        int u, v;            ///< Endpoint nodes
        double w;            ///< Shortcut weight
        int contracted;      ///< ID of the contracted node
    };

    /**
     * @brief Priority queue item for regular CH node ordering
     */
    struct PQItem {
        int priority;                    ///< Node priority (lower = contract earlier)
        int node;                        ///< Node ID
        std::vector<Shortcut> shortcuts; ///< Shortcuts to add when this node is contracted
        
        /**
         * @brief Comparison operator for priority queue
         * 
         * @param other Other PQItem to compare with
         * @return True if this item has lower priority
         */
        bool operator<(const PQItem& other) const {
            return priority > other.priority;
        }
    };
    
    /**
     * @brief Priority queue item for POI CH node ordering
     */
    struct POIPQItem {
        int node;                        ///< Node ID
        int edgeDiff;                    ///< Edge difference metric
        std::vector<Shortcut> shortcuts; ///< Shortcuts to add when this node is contracted
        bool isPOI;                      ///< Whether this node is a POI
        
        /**
         * @brief Comparison operator for priority queue
         * 
         * POIs are prioritized to be contracted last, then by edge difference
         * 
         * @param other Other POIPQItem to compare with
         * @return True if this item has lower priority
         */
        bool operator<(const POIPQItem& other) const {
            if (isPOI != other.isPOI) return isPOI > other.isPOI;
            return edgeDiff > other.edgeDiff;
        }
    };

    // Common data members
    Mode mode;                    ///< Operation mode
    Graph ch_graph;               ///< Working graph with shortcuts
    int n;                        ///< Number of nodes
    std::vector<int> rank;        ///< Node ordering (rank[node] = position)
    std::vector<bool> contracted; ///< Whether each node is contracted

    // Hierarchy edges
    std::vector<std::vector<Edge>> up; ///< Upward edges to higher-ranked nodes
    
    // POI-specific data
    std::unordered_set<int> poi_nodes; ///< Set of POI node IDs
    bool preprocessed = false;         ///< Whether preprocessing is complete
    
    // Timestamp-based optimization data for witness searches
    std::vector<double> distF;     ///< Forward distances
    std::vector<double> distB;     ///< Backward distances
    std::vector<int> last_visitF;  ///< Last forward visit timestamp
    std::vector<int> last_visitB;  ///< Last backward visit timestamp
    std::vector<char> visF, visB;  ///< Visit flags
    std::vector<int> parentF, parentB; ///< Parent pointers
    int curr_visitF;               ///< Current forward timestamp
    int curr_visitB;               ///< Current backward timestamp

    /**
     * @brief Regular CH preprocessing implementation
     */
    void preprocess_regular();
    
    /**
     * @brief POI CH preprocessing implementation
     */
    void preprocess_poi();
    
    /**
     * @brief Computes priority for a node in regular CH mode
     * 
     * @param v Node ID
     * @return PQItem with priority and necessary shortcuts
     */
    PQItem compute_priority_regular(int v);
    
    /**
     * @brief Computes priority for a node in POI CH mode
     * 
     * @param v Node ID
     * @return POIPQItem with priority and necessary shortcuts
     */
    POIPQItem compute_priority_poi(int v);
    
    /**
     * @brief POI-specific query implementation
     * 
     * @param s Source node ID
     * @param t Target node ID
     * @return DijkstraResult containing distance, path, and statistics
     */
    DijkstraResult queryViaPOI(int s, int t);
    
    /**
     * @brief Contracts a node, adding necessary shortcuts
     * 
     * @param v Node ID to contract
     * @param shortcuts_to_add Shortcuts that must be added
     */
    void contract_node(int v, std::vector<Shortcut>& shortcuts_to_add);
    
    /**
     * @brief Builds the final hierarchy structure after contraction
     */
    void build_hierarchy();
    
    /**
     * @brief Expands a single edge, recursively handling shortcuts
     * 
     * @param u Source node ID
     * @param v Target node ID
     * @param out Vector to append expanded nodes to
     */
    void expand_edge(int u, int v, std::vector<int>& out) const;
    
    /**
     * @brief Checks if a witness path exists that avoids a specific node
     * 
     * @param source Start node ID
     * @param target End node ID
     * @param limit Maximum distance to consider
     * @param via_v Node ID to avoid
     * @param directWeight Weight of direct path through via_v
     * @return True if a witness path exists
     */
    bool witness_path_exists(int source, int target, double limit, int via_v, double directWeight);
    
    /**
     * @brief Resets visit data for a node using timestamp-based method
     * 
     * @param nodeId Node ID to check/reset
     * @param forward Whether this is for forward search
     */
    void check_last_visit(int nodeId, bool forward = true);
};

#endif // UNIFIED_CONTRACTION_HIERARCHY_HPP