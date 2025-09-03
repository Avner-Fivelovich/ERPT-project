#include "customizable_contraction_hierarchy.hpp"
#include <queue>
#include <cassert>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <unordered_set>
#include <utility>

/**
 * @brief Constructor for the Customizable Contraction Hierarchy
 * 
 * Initializes the CCH structure with the original graph and configuration paths.
 * 
 * @param g Original graph to build CCH on
 * @param kahipPath Path to the KaHIP node ordering executable
 * @param graphFilePath Path where graph will be written for KaHIP
 * @param orderingFilePath Path where node ordering will be stored
 * @param operatingSystem OS type for proper command execution
 */
CustomizableContractionHierarchy::CustomizableContractionHierarchy(
    const Graph& g,
    const std::string& kahipPath,
    const std::string& graphFilePath,
    const std::string& orderingFilePath,
    OS operatingSystem
) : original_graph(g), 
    n(g.node_count()), 
    kahip_path(kahipPath),
    graph_file_path(graphFilePath),
    ordering_file_path(orderingFilePath),
    os_type(operatingSystem),
    num_shortcuts(0) {
}

/**
 * @brief PHASE I: Performs the metric-independent preprocessing
 * 
 * This method executes the complete metric-independent phase of CCH:
 * 1. Computing a nested dissection order using KaHIP
 * 2. Building the CCH structure with all necessary shortcuts
 * 3. Identifying all lower triangles
 * 4. Sorting shortcuts by triangle ranks
 * 5. Running initial customization with original weights
 * 
 * The preprocessing is done once and reused for all customizations.
 */
void CustomizableContractionHierarchy::preprocess() {
    auto start = NOW();

    std::cout << "PHASE I: Computing metric-independent information..." << std::endl;
    
    // 1. Compute nested dissection order using KaHIP
    std::vector<int> order = compute_nested_dissection_order();
    
    // 2. Build the CCH structure with all-in shortcuts
    build_cch(order);
    
    // 3. Identify and store all lower triangles with each shortcut
    identify_lower_triangles();
    
    // 4. Sort shortcuts by highest node rank of any lower triangle
    sort_shortcuts_by_highest_rank();

    auto end = NOW();
    preprocessing_time = TO_MILI(end - start);
    
    std::cout << "Phase I completed in " << preprocessing_time.count() << " ms." << std::endl;

    // Run initial customization with original weights (PHASE II)
    customize();
}

/**
 * @brief Converts file paths between Windows and WSL format
 * 
 * When using WSL on Windows, paths need to be converted to the appropriate format.
 * For example, "C:\temp\file.txt" becomes "/mnt/c/temp/file.txt"
 * 
 * @param path Windows path to convert
 * @return Converted path for WSL access
 */
std::string CustomizableContractionHierarchy::convertPath(const std::string& path) {
    if (os_type == OS::LINUX) {
        return path; // No conversion needed for Linux paths
    }
    
    // Convert Windows path to WSL path
    std::string wslPath = "/mnt/";
    if (path.length() >= 2 && path[1] == ':') {
        char driveLetter = std::tolower(path[0]);
        wslPath += driveLetter;
        
        // Add the rest of the path, replacing backslashes with forward slashes
        for (size_t i = 2; i < path.length(); i++) {
            if (path[i] == '\\') {
                wslPath += '/';
            } else {
                wslPath += path[i];
            }
        }
        return wslPath;
    }
    
    // If not a standard Windows path, just replace backslashes
    std::string result = path;
    std::replace(result.begin(), result.end(), '\\', '/');
    return result;
}

/**
 * @brief Computes a nested dissection ordering using KaHIP
 * 
 * This function:
 * 1. Writes the graph to a file in KaHIP format
 * 2. Runs the KaHIP node ordering algorithm
 * 3. Reads the resulting ordering from the output file
 * 
 * Nested dissection recursively partitions the graph, placing separator nodes
 * last in the ordering, which is ideal for minimizing shortcuts in CCH.
 * 
 * @return Vector of node IDs in the computed order
 */
std::vector<int> CustomizableContractionHierarchy::compute_nested_dissection_order() {
    std::cout << "Computing nested dissection order..." << std::endl;
    
    // Write graph to file in KaHIP format
    if (!writeGraphForKaHIP(graph_file_path)) {
        std::cerr << "Failed to write graph file for KaHIP!" << std::endl;
        // Return a default ordering as fallback
        std::vector<int> default_order(n);
        for (int i = 0; i < n; ++i) {
            default_order[i] = i;
        }
        return default_order;
    }
    
    // Run KaHIP node ordering algorithm
    if (!runKaHIPNodeOrdering(graph_file_path, ordering_file_path)) {
        std::cerr << "Failed to run KaHIP node ordering!" << std::endl;
        // Return a default ordering as fallback
        std::vector<int> default_order(n);
        for (int i = 0; i < n; ++i) {
            default_order[i] = i;
        }
        return default_order;
    }
    
    // Read the node ordering
    std::vector<int> ordering = readNodeOrdering(ordering_file_path);
    
    if (ordering.empty() || ordering.size() != n) {
        std::cerr << "Invalid node ordering received from KaHIP!" << std::endl;
        // Return a default ordering as fallback
        std::vector<int> default_order(n);
        for (int i = 0; i < n; ++i) {
            default_order[i] = i;
        }
        return default_order;
    }
    
    return ordering;
}

/**
 * @brief Writes the graph to a file in KaHIP format
 * 
 * Converts the graph to the format expected by KaHIP:
 * - First line: numNodes numEdges 0 (0 indicates unweighted)
 * - Following lines: list of neighbors for each node
 * 
 * @param filename Path to write the graph file
 * @return True if successful, false otherwise
 */
bool CustomizableContractionHierarchy::writeGraphForKaHIP(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    // Count nodes
    int numNodes = n;
    
    // First count edges correctly (each undirected edge counted only once)
    int numEdges = 0;
    std::unordered_set<std::pair<int, int>, PairHash> countedEdges;
    
    for (int u = 0; u < numNodes; ++u) {
        for (const auto& edge : original_graph.neighbors(u)) {
            int v = edge.to;
            auto edgePair = std::make_pair(std::min(u, v), std::max(u, v));
            if (countedEdges.find(edgePair) == countedEdges.end()) {
                numEdges++;
                countedEdges.insert(edgePair);
            }
        }
    }

    // Write header: numNodes numEdges 0 (0 indicates unweighted)
    file << numNodes << " " << numEdges << " 0" << std::endl;

    // Write adjacency lists - each line should contain ALL neighbors for vertex i
    for (int u = 0; u < numNodes; ++u) {
        std::stringstream line;
        
        // Each vertex's line contains all its neighbors
        for (const auto& edge : original_graph.neighbors(u)) {
            int v = edge.to;
            // Convert to 1-based indexing for Metis format
            line << (v + 1) << " ";
        }
        
        file << line.str() << std::endl;
    }

    file.close();
    return true;
}

/**
 * @brief Runs the KaHIP node ordering algorithm
 * 
 * Executes KaHIP as an external process to compute a node ordering.
 * Handles platform differences between Windows (using WSL) and Linux.
 * 
 * @param inputGraphFile Path to the input graph file
 * @param outputOrderingFile Path to write the ordering
 * @return True if successful, false otherwise
 */
bool CustomizableContractionHierarchy::runKaHIPNodeOrdering(
    const std::string& inputGraphFile, 
    const std::string& outputOrderingFile) {
    
    // Convert paths based on operating system
    std::string execCommand;
    std::string convertedInputPath = inputGraphFile;
    std::string convertedOutputPath = outputOrderingFile;
    
    if (os_type == OS::WINDOWS) {
        // For Windows, use WSL
        convertedInputPath = convertPath(inputGraphFile);
        convertedOutputPath = convertPath(outputOrderingFile);
        execCommand = "wsl " + kahip_path + " " + 
                     convertedInputPath + " --output=" + convertedOutputPath;
    } else {
        // For Linux, direct execution
        execCommand = kahip_path + " " + 
                     convertedInputPath + " --output=" + convertedOutputPath;
    }
    
    std::cout << "Executing: " << execCommand << std::endl;
    
    // Execute command
    int result = system(execCommand.c_str());
    
    return (result == 0);
}

/**
 * @brief Reads a node ordering from a file
 * 
 * Parses the ordering file produced by KaHIP, converting from 1-based
 * to 0-based indexing and validating the result.
 * 
 * @param filename Path to read the ordering from
 * @return Vector of node IDs in the read order
 */
std::vector<int> CustomizableContractionHierarchy::readNodeOrdering(const std::string& filename) {
    std::vector<int> ordering;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open node ordering file: " << filename << std::endl;
        return ordering; // Return empty vector on error
    }
    
    int node;
    while (file >> node) {
        // KaHIP outputs 1-based indices, convert back to 0-based
        ordering.push_back(node - 1);
    }
    
    // Validate the ordering
    if (ordering.size() != n) {
        std::cerr << "Warning: Node ordering size (" << ordering.size() 
                  << ") does not match graph size (" << n << ")" << std::endl;
    }
    
    return ordering;
}

/**
 * @brief Builds the CCH structure using the given node ordering
 * 
 * Creates the upward and downward edges in the hierarchy based on
 * the provided node ordering, adding all necessary edges.
 * 
 * @param order Vector of node IDs in the desired order
 */
void CustomizableContractionHierarchy::build_cch(const std::vector<int>& order) {
    std::cout << "Building CCH structure..." << std::endl;
    
    // Initialize CCH data structures
    rank.resize(n);
    for (int i = 0; i < n; ++i) {
        rank[order[i]] = i;  // Invert the order to get ranks
    }
    
    up.resize(n);
    down.resize(n);
    
    // Count original edges to track shortcuts later
    size_t original_edge_count = original_graph.get_num_of_edges();
    
    // Add edges respecting the hierarchy
    for (int u = 0; u < n; ++u) {
        for (const auto& e : original_graph.neighbors(u)) {
            int v = e.to;
            
            // Skip duplicate edges
            if (u > v) continue;
            
            // Connect based on rank
            if (rank[u] < rank[v]) {
                up[u].push_back({v, e.w, {}});
                down[v].push_back({u, e.w, {}});
            } else {
                up[v].push_back({u, e.w, {}});
                down[u].push_back({v, e.w, {}});
            }
        }
    }
    
    // Count shortcuts added
    size_t total_edges = 0;
    for (int i = 0; i < n; ++i) {
        total_edges += up[i].size();
    }
    num_shortcuts = total_edges - original_edge_count;
    
    std::cout << "CCH built with " << num_shortcuts << " shortcuts." << std::endl;
}

/**
 * @brief Identifies all lower triangles in the hierarchy
 * 
 * A lower triangle consists of two nodes u, v with an edge between them
 * and a higher-ranked node w that is adjacent to both u and v.
 * 
 * For each edge (u,v), this method finds all higher-ranked nodes w that
 * form a triangle with u and v, and associates this triangle with the edge.
 */
void CustomizableContractionHierarchy::identify_lower_triangles() {
    std::cout << "Identifying lower triangles..." << std::endl;
    
    // Create a helper map for fast edge lookup
    std::vector<std::unordered_map<int, int>> edge_idx(n);
    
    // Index all edges for fast lookup
    for (int u = 0; u < n; ++u) {
        for (size_t i = 0; i < up[u].size(); ++i) {
            edge_idx[u][up[u][i].to] = i;
        }
    }
    
    // Process nodes in order of decreasing rank to find lower triangles
    for (int rank_idx = n-1; rank_idx >= 0; --rank_idx) {
        // Find the node with this rank
        int w = -1;
        for (int i = 0; i < n; ++i) {
            if (rank[i] == rank_idx) {
                w = i;
                break;
            }
        }
        
        if (w == -1) continue;
        
        // Look at all pairs of neighbors of w with lower rank
        std::vector<int> lower_neighbors;
        for (const auto& e : down[w]) {
            lower_neighbors.push_back(e.to);
        }
        
        // Find triangles
        for (size_t i = 0; i < lower_neighbors.size(); ++i) {
            int u = lower_neighbors[i];
            for (size_t j = i+1; j < lower_neighbors.size(); ++j) {
                int v = lower_neighbors[j];
                
                // Check if edge (u,v) exists
                if (edge_idx[u].find(v) != edge_idx[u].end() || 
                    (rank[v] < rank[u] && edge_idx[v].find(u) != edge_idx[v].end())) {
                    
                    // Found triangle (u,v,w)
                    int triangle_idx = triangles.size();
                    
                    // Get weight_uw from down[w]
                    double weight_uw = std::numeric_limits<double>::infinity();
                    for (const auto& e : down[w]) {
                        if (e.to == u) {
                            weight_uw = e.weight;
                            break;
                        }
                    }
                    
                    // Get weight_vw from down[w]
                    double weight_vw = std::numeric_limits<double>::infinity();
                    for (const auto& e : down[w]) {
                        if (e.to == v) {
                            weight_vw = e.weight;
                            break;
                        }
                    }
                    
                    // Get weight_uv
                    double weight_uv = std::numeric_limits<double>::infinity();
                    if (rank[u] < rank[v]) {
                        auto it = edge_idx[u].find(v);
                        if (it != edge_idx[u].end()) {
                            weight_uv = up[u][it->second].weight;
                        }
                    } else {
                        auto it = edge_idx[v].find(u);
                        if (it != edge_idx[v].end()) {
                            weight_uv = up[v][it->second].weight;
                        }
                    }
                    
                    // Store the triangle
                    triangles.push_back({u, v, w, weight_uv, weight_uw, weight_vw});
                    
                    // Add triangle reference to the edge
                    if (rank[u] < rank[v]) {
                        auto it = edge_idx[u].find(v);
                        if (it != edge_idx[u].end()) {
                            up[u][it->second].triangles.push_back(triangle_idx);
                        }
                    } else {
                        auto it = edge_idx[v].find(u);
                        if (it != edge_idx[v].end()) {
                            up[v][it->second].triangles.push_back(triangle_idx);
                        }
                    }
                }
            }
        }
    }
    
    std::cout << "Found " << triangles.size() << " lower triangles." << std::endl;
}

/**
 * @brief Sorts shortcuts by the highest rank of their lower triangles
 * 
 * This ordering is essential for efficient customization, as it ensures
 * that edge weights are computed in the correct order (higher-ranked triangles first).
 * 
 * For each shortcut, the method finds the highest-ranked node in any of its
 * lower triangles and sorts shortcuts based on this rank.
 */
void CustomizableContractionHierarchy::sort_shortcuts_by_highest_rank() {
    std::cout << "Sorting shortcuts by highest rank of lower triangles..." << std::endl;
    
    // Create a vector of all shortcuts with their highest triangle rank
    std::vector<std::pair<int, std::pair<int, int>>> shortcuts_with_rank;
    
    for (int u = 0; u < n; ++u) {
        for (size_t i = 0; i < up[u].size(); ++i) {
            const auto& edge = up[u][i];
            int v = edge.to;
            
            // Find highest rank node in any lower triangle
            int highest_rank = -1;
            for (int triangle_idx : edge.triangles) {
                const auto& t = triangles[triangle_idx];
                highest_rank = std::max(highest_rank, rank[t.w]);
            }
            
            // Store shortcut with its rank
            shortcuts_with_rank.push_back({highest_rank, {u, v}});
        }
    }
    
    // Sort shortcuts by the highest rank (descending order)
    std::sort(shortcuts_with_rank.begin(), shortcuts_with_rank.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });
    
    // Store sorted shortcuts for later use in customization
    sorted_shortcuts.clear();
    for (const auto& [_, edge] : shortcuts_with_rank) {
        sorted_shortcuts.push_back(edge);
    }
    
    std::cout << "Sorted " << sorted_shortcuts.size() << " shortcuts by rank." << std::endl;
}

/**
 * @brief PHASE II: Customizes the CCH with current edge weights
 * 
 * This method executes the metric-dependent customization phase of CCH:
 * 1. Basic customization: assigning original weights to all edges
 * 2. Processing shortcuts in order, updating weights via triangle relaxation
 * 
 * Customization is very fast and can be repeated for different metrics.
 */
void CustomizableContractionHierarchy::customize() {
    auto start = NOW();

    std::cout << "PHASE II: Computing metric-dependent information..." << std::endl;
    
    // 1. Assign new edge costs to the original edges
    basic_customization();
    
    // 2. Sweep through shortcuts in presorted order
    process_shortcuts_in_order();

    auto end = NOW();
    customization_time = TO_MICRO(end - start);

    std::cout << "Phase II completed in " << customization_time.count() / 1000.0 << " ms." << std::endl;
}

/**
 * @brief Customizes the CCH with new random weights for specified edges
 * 
 * This version first updates the weights in the original graph before
 * running the standard customization process.
 * 
 * @param edge_changes Pairs of node IDs representing edges to assign random weights to
 */
void CustomizableContractionHierarchy::customize(const std::vector<std::pair<int, int>>& edge_changes) {
    auto start = NOW();
    
    std::cout << "PHASE II: Customizing with new random weights..." << std::endl;
    
    // Create a copy of the original graph
    Graph temp_graph = original_graph;
    
    // Apply new edge weights
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    for (const auto& [u, v] : edge_changes) {
        double weight = dis(gen);
        temp_graph.set_weight(u, v, weight);
    }
    
    // Update original_graph
    original_graph = temp_graph;
    
    // PHASE II: Metric-dependent customization
    // 1. Basic customization - assign new costs to original edges
    basic_customization();
    
    // 2. Process shortcuts in presorted order
    process_shortcuts_in_order();
    
    auto end = NOW();
    customization_time = TO_MICRO(end - start);
    
    std::cout << "Phase II completed in " << customization_time.count() / 1000.0 << " ms." << std::endl;
}

/**
 * @brief Performs basic customization (copying original weights)
 * 
 * Assigns weights from the original graph to all edges in the hierarchy.
 * This is the first step of customization, before triangle relaxation.
 */
void CustomizableContractionHierarchy::basic_customization() {
    // Reset all edge weights in up and down graphs
    for (int u = 0; u < n; ++u) {
        for (auto& e : up[u]) {
            int v = e.to;
            // Find the weight in the original graph
            e.weight = original_graph.get_weight(u, v);
        }
        
        for (auto& e : down[u]) {
            int v = e.to;
            // Find the weight in the original graph
            e.weight = original_graph.get_weight(u, v);
        }
    }
}

/**
 * @brief Processes shortcuts in pre-sorted order
 * 
 * For each shortcut, relaxes it via all its lower triangles to find
 * the shortest path that it represents.
 * 
 * The shortcuts are processed in order of the highest rank of their
 * lower triangles, ensuring correct weight propagation.
 */
void CustomizableContractionHierarchy::process_shortcuts_in_order() {
    // Process shortcuts in the presorted order (by highest rank of lower triangles)
    for (const auto& [u, v] : sorted_shortcuts) {
        // Find edge in up graph
        for (auto& e : up[u]) {
            if (e.to == v) {
                // For each lower triangle of this shortcut
                for (int triangle_idx : e.triangles) {
                    const auto& t = triangles[triangle_idx];
                    
                    // Get the middle node of the triangle
                    int w = t.w;
                    
                    // Get weights of edges (u,w) and (w,v)
                    double weight_uw = std::numeric_limits<double>::infinity();
                    double weight_wv = std::numeric_limits<double>::infinity();
                    
                    for (const auto& e_uw : up[u]) {
                        if (e_uw.to == w) {
                            weight_uw = e_uw.weight;
                            break;
                        }
                    }
                    
                    for (const auto& e_wv : down[w]) {
                        if (e_wv.to == v) {
                            weight_wv = e_wv.weight;
                            break;
                        }
                    }
                    
                    // Calculate path cost through the triangle
                    double path_cost = weight_uw + weight_wv;
                    
                    // Update shortcut weight if path is shorter
                    if (path_cost < e.weight) {
                        e.weight = path_cost;
                        
                        // Also update corresponding edge in down graph
                        for (auto& e_down : down[v]) {
                            if (e_down.to == u) {
                                e_down.weight = path_cost;
                                break;
                            }
                        }
                    }
                }
                break;
            }
        }
    }
}

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
DijkstraResult CustomizableContractionHierarchy::query(int s, int t) {
    std::cout << "PHASE III: Executing query from " << s << " to " << t << "..." << std::endl;
    
    // Handle same source and target case
    if (s == t) {
        DijkstraResult result;
        result.dist.resize(n, std::numeric_limits<double>::infinity());
        result.dist[s] = 0;
        result.path.push_back(original_graph.get_node(s));
        return result;
    }
    
    // Initialize query data structures
    QueryData forward;
    forward.dist.resize(n, std::numeric_limits<double>::infinity());
    forward.parent.resize(n, -1);
    forward.visited.resize(n, false);
    
    QueryData backward;
    backward.dist.resize(n, std::numeric_limits<double>::infinity());
    backward.parent.resize(n, -1);
    backward.visited.resize(n, false);
    
    // 1. Forward search (up phase)
    upward_search(s, forward);
    
    // 2. Backward search (down phase)
    downward_search(t, forward, backward);
    
    // 3. Reconstruct path
    std::vector<Node> path = reconstruct_path(s, t, backward);
    
    // Prepare result
    DijkstraResult result;
    result.dist = backward.dist;
    result.path = path;
    result.nodes_popped = 0; // We don't track this for CCH
    
    return result;
}

/**
 * @brief Performs upward search from source node
 * 
 * First phase of the CCH query algorithm, exploring upward in the hierarchy
 * from the source node.
 * 
 * @param s Source node ID
 * @param forward Reference to forward search state
 */
void CustomizableContractionHierarchy::upward_search(int s, QueryData& forward) {
    // Initialize source
    forward.dist[s] = 0;
    
    // Use priority queue for upward search
    using PQItem = std::pair<double, int>;
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;
    pq.push({0, s});
    
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (d > forward.dist[u]) continue;  // Skip if we already found a better path
        if (forward.visited[u]) continue;   // Skip if already visited
        forward.visited[u] = true;
        
        // Relax upward edges
        for (const auto& e : up[u]) {
            int v = e.to;
            double nd = d + e.weight;
            
            if (nd < forward.dist[v]) {
                forward.dist[v] = nd;
                forward.parent[v] = u;
                pq.push({nd, v});
            }
        }
    }
}

/**
 * @brief Performs downward search from target node
 * 
 * Second phase of the CCH query algorithm, exploring downward in the hierarchy
 * from the target node, but only visiting nodes that were seen in the forward phase.
 * 
 * @param t Target node ID
 * @param forward Reference to completed forward search
 * @param backward Reference to backward search state
 */
void CustomizableContractionHierarchy::downward_search(int t, const QueryData& forward, QueryData& backward) {
    // Initialize target
    backward.dist[t] = 0;
    backward.parent[t] = -1;
    
    // Use priority queue for downward search
    using PQItem = std::pair<double, int>;
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;
    pq.push({0, t});
    
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (d > backward.dist[u]) continue;  // Skip if we already found a better path
        if (backward.visited[u]) continue;   // Skip if already visited
        backward.visited[u] = true;
        
        // Relax downward edges
        for (const auto& e : down[u]) {
            int v = e.to;
            
            // Skip if not visited in forward search
            if (!forward.visited[v]) continue;
            
            double nd = d + e.weight;
            if (nd < backward.dist[v]) {
                backward.dist[v] = nd;
                backward.parent[v] = u;
                pq.push({nd, v});
            }
        }
    }
}

/**
 * @brief Reconstructs the shortest path from search results
 * 
 * Traces parent pointers from the source to the target to build
 * the complete shortest path.
 * 
 * @param s Source node ID
 * @param t Target node ID
 * @param backward Reference to backward search state
 * @return Vector of nodes representing the path
 */
std::vector<Node> CustomizableContractionHierarchy::reconstruct_path(int s, int t, const QueryData& backward) {
    std::vector<Node> path;
    if (backward.dist[s] == std::numeric_limits<double>::infinity()) {
        return path; // No path found
    }
    
    // Start from s and follow parent pointers
    for (int v = s; v != -1; v = backward.parent[v]) {
        path.push_back(original_graph.get_node(v));
        if (v == t) break;  // Stop when we reach t
    }
    
    // If path doesn't end with t, the path construction failed
    if (path.empty() || path.back().id != t) {
        return {}; // Invalid path
    }
    
    return path;
}

/**
 * @brief Gets the total number of edges in the CCH structure
 * 
 * @return Number of edges in the hierarchy
 */
size_t CustomizableContractionHierarchy::get_num_of_edges() const {
    size_t count = 0;
    for (const auto& edges : up) {
        count += edges.size();
    }
    return count;
}

/**
 * @brief Gets the number of shortcuts added during preprocessing
 * 
 * @return Number of shortcut edges
 */
size_t CustomizableContractionHierarchy::get_num_of_shortcuts() const {
    return num_shortcuts;
}

/**
 * @brief Gets the average number of triangles per edge
 * 
 * This is a metric for the complexity of the customization phase.
 * 
 * @return Average number of triangles per edge
 */
double CustomizableContractionHierarchy::get_avg_triangles_per_edge() const {
    size_t total_triangles = 0;
    size_t total_edges = 0;
    
    for (const auto& edges : up) {
        for (const auto& edge : edges) {
            total_triangles += edge.triangles.size();
            total_edges++;
        }
    }
    
    return total_edges > 0 ? static_cast<double>(total_triangles) / total_edges : 0.0;
}

/**
 * @brief Gets the maximum number of triangles for any edge
 * 
 * @return Maximum triangle count for any edge
 */
size_t CustomizableContractionHierarchy::get_max_triangles_per_edge() const {
    size_t max_triangles = 0;
    
    for (const auto& edges : up) {
        for (const auto& edge : edges) {
            max_triangles = std::max(max_triangles, edge.triangles.size());
        }
    }
    
    return max_triangles;
}

/**
 * @brief Gets the time taken for preprocessing
 * 
 * @return Preprocessing time in milliseconds
 */
std::chrono::milliseconds CustomizableContractionHierarchy::get_preprocessing_time() const {
    return preprocessing_time;
}

/**
 * @brief Gets the time taken for customization
 * 
 * @return Customization time in microseconds
 */
std::chrono::microseconds CustomizableContractionHierarchy::get_customization_time() const {
    return customization_time;
}

/**
 * @brief Exports all edges in the CCH for visualization or analysis
 * 
 * @return Vector of tuples (source, target, weight)
 */
std::vector<std::tuple<int,int,double>> CustomizableContractionHierarchy::export_all_edges() const {
    std::vector<std::tuple<int,int,double>> edges;
    
    for (int u = 0; u < n; ++u) {
        for (const auto& e : up[u]) {
            edges.emplace_back(u, e.to, e.weight);
        }
    }
    
    return edges;
}