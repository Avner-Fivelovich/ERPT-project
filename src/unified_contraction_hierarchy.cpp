#include "unified_contraction_hierarchy.hpp"
#include <algorithm>
#include <functional>

/**
 * @brief Constructor for the Unified Contraction Hierarchy
 * 
 * Initializes all data structures needed for the CH algorithm.
 * 
 * @param g Original graph to build hierarchy on
 * @param mode Operation mode (REGULAR_CH or POI_CH)
 */
UnifiedContractionHierarchy::UnifiedContractionHierarchy(const Graph& g, Mode mode)
    : mode(mode), ch_graph(g), n(g.node_count()) {
    rank.assign(n, -1);
    contracted.assign(n, false);
    distF.assign(n, INF);
    distB.assign(n, INF);
    last_visitF.assign(n, 0);
    last_visitB.assign(n, 0);
    visF.assign(n, false);
    visB.assign(n, false);
    parentF.assign(n, -1);
    parentB.assign(n, -1);
    curr_visitF = -1;
    curr_visitB = -1;
}

/**
 * @brief Main preprocessing function that delegates to the appropriate mode-specific implementation
 * 
 * Selects the appropriate preprocessing algorithm based on the selected mode.
 */
void UnifiedContractionHierarchy::preprocess() {
    if (mode == REGULAR_CH) {
        preprocess_regular();
    } else {
        preprocess_poi();
    }
}

/**
 * @brief Regular CH preprocessing implementation
 * 
 * Builds the contraction hierarchy using a priority queue approach:
 * 1. Compute initial node priorities
 * 2. Contract nodes in order of increasing priority
 * 3. Update priorities of affected nodes
 * 4. Build final hierarchy structure
 */
void UnifiedContractionHierarchy::preprocess_regular() {
    // Priority queue for choosing next contraction
    std::priority_queue<PQItem> pq;
    for (int v = 0; v < n; ++v) {
        pq.push(compute_priority_regular(v));
    }
    
    int contractedCount = 0;
    while (!pq.empty()) {
        auto top = pq.top(); pq.pop();
        int v = top.node;
        if (contracted[v]) continue;
        
        auto currentPrio = compute_priority_regular(v);
        if (currentPrio.priority != top.priority) {
            // Reinsert with updated priority
            pq.push(currentPrio);
            continue;
        }
        
        // Contract v
        contract_node(v, currentPrio.shortcuts);
        rank[v] = contractedCount++;
    }
    
    build_hierarchy();
    preprocessed = true;
}

/**
 * @brief POI CH preprocessing implementation
 * 
 * Similar to regular CH but uses POI-aware node priorities:
 * 1. Ensure POIs are assigned (randomly if needed)
 * 2. Compute node priorities with POI awareness
 * 3. Contract nodes with POIs contracted last
 * 4. Build final hierarchy structure
 */
void UnifiedContractionHierarchy::preprocess_poi() {
    if (poi_nodes.empty()) {
        assignRandomPOIs(); // Ensure we have POIs
    }
    
    // Priority queue for choosing next contraction
    std::priority_queue<POIPQItem> pq;
    for (int v = 0; v < n; ++v) {
        pq.push(compute_priority_poi(v));
    }
    
    int contractedCount = 0;
    while (!pq.empty()) {
        auto top = pq.top(); pq.pop();
        int v = top.node;
        if (contracted[v]) continue;
        
        auto currentPrio = compute_priority_poi(v);
        if (currentPrio.edgeDiff != top.edgeDiff) {
            // Reinsert with updated priority
            pq.push(currentPrio);
            continue;
        }
        
        // Contract v
        contract_node(v, currentPrio.shortcuts);
        rank[v] = contractedCount++;
    }
    
    build_hierarchy();
    preprocessed = true;
}

/**
 * @brief Computes node priority for regular CH contraction
 * 
 * Calculates the "edge difference" heuristic for a node:
 * - Edge difference = #shortcuts_added - #edges_removed
 * - Lower values mean better contraction candidates
 * 
 * @param v Node ID to compute priority for
 * @return PQItem containing priority and required shortcuts
 */
UnifiedContractionHierarchy::PQItem UnifiedContractionHierarchy::compute_priority_regular(int v) {
    if (contracted[v]) return {std::numeric_limits<int>::max(), v, {}};
    
    // Estimate number of shortcuts needed
    int deg = (int)ch_graph.neighbors(v).size();
    std::vector<Shortcut> shortcutsNeeded;
    const auto& nbrs = ch_graph.neighbors(v);
    
    for (size_t i = 0; i < nbrs.size(); ++i) {
        int u = nbrs[i].to;
        if (contracted[u]) continue;
        
        for (size_t j = i + 1; j < nbrs.size(); ++j) {
            int w = nbrs[j].to;
            if (u == w || contracted[w]) continue;
            
            double uwWeight = nbrs[i].w + nbrs[j].w;
            // Witness search: is there an alternative path <= uwWeight avoiding v?
            double limit = uwWeight;
            bool witness = witness_path_exists(u, w, limit, v, uwWeight);
            
            if (!witness) {
                shortcutsNeeded.push_back({u, w, uwWeight, v});
            }
        }
    }
    
    // Edge difference heuristic
    int edgeDiff = shortcutsNeeded.size() - deg;
    return {edgeDiff, v, shortcutsNeeded};
}

/**
 * @brief Computes node priority for POI CH contraction
 * 
 * Similar to regular priority but gives POI nodes higher priority
 * to ensure they remain in the hierarchy longer.
 * 
 * @param v Node ID to compute priority for
 * @return POIPQItem containing priority and required shortcuts
 */
UnifiedContractionHierarchy::POIPQItem UnifiedContractionHierarchy::compute_priority_poi(int v) {
    if (contracted[v]) return {v, std::numeric_limits<int>::max(), {}, true};
    
    // Estimate number of shortcuts needed
    int deg = (int)ch_graph.neighbors(v).size();
    std::vector<Shortcut> shortcutsNeeded;
    const auto& nbrs = ch_graph.neighbors(v);
    
    for (size_t i = 0; i < nbrs.size(); ++i) {
        int u = nbrs[i].to;
        if (contracted[u]) continue;
        
        for (size_t j = i + 1; j < nbrs.size(); ++j) {
            int w = nbrs[j].to;
            if (u == w || contracted[w]) continue;
            
            double uwWeight = nbrs[i].w + nbrs[j].w;
            // Witness search: is there an alternative path <= uwWeight avoiding v?
            double limit = uwWeight;
            bool witness = witness_path_exists(u, w, limit, v, uwWeight);
            
            if (!witness) {
                shortcutsNeeded.push_back({u, w, uwWeight, v});
            }
        }
    }
    
    // Edge difference heuristic with POI priority
    int edgeDiff = shortcutsNeeded.size() - deg;
    bool isPOI = poi_nodes.count(v) > 0;
    
    return {v, edgeDiff, shortcutsNeeded, isPOI};
}

/**
 * @brief Contracts a node, adding necessary shortcuts
 * 
 * When a node is contracted, shortcuts are added to preserve
 * shortest paths that would have gone through this node.
 * 
 * @param v Node ID to contract
 * @param shortcuts_to_add Vector of shortcuts to add
 */
void UnifiedContractionHierarchy::contract_node(int v, std::vector<Shortcut> &shortcuts_to_add) {
    for (auto &sc : shortcuts_to_add) {
        int u = sc.u, w = sc.v;
        double weight = sc.w;
        // Add shortcut in both directions
        ch_graph.add_edge(u, w, weight, v);
    }
    contracted[v] = true;
}

/**
 * @brief Builds the final hierarchy structure after contraction
 * 
 * After all nodes are contracted, this creates the upward edges
 * used during query phase based on the contraction ordering.
 */
void UnifiedContractionHierarchy::build_hierarchy() {
    // Clear existing up edges
    up.assign(n, {});
    // Build adjacency including original edges and shortcuts
    for (int v = 0; v < n; ++v) {
        for (auto &e : ch_graph.neighbors(v)) {
            int u = e.to;
            if (rank[u] > rank[v]) {
                up[v].push_back(e);
            }
        }
    }
}

/**
 * @brief Checks if a witness path exists that avoids a specific node
 * 
 * Used during contraction to determine if shortcuts are necessary.
 * Performs a bounded Dijkstra search to find alternative paths.
 * 
 * @param source Start node ID
 * @param target End node ID
 * @param limit Maximum distance to consider
 * @param via_v Node ID to avoid
 * @param directWeight Weight of direct path through via_v
 * @return true if a witness path exists, false otherwise
 */
bool UnifiedContractionHierarchy::witness_path_exists(int source, int target, double limit, int via_v, double directWeight) {
    // Bounded Dijkstra ignoring node via_v, only through uncontracted nodes
    using QN = std::pair<double,int>;
    std::priority_queue<QN, std::vector<QN>, std::greater<QN>> pq;
    // Increment timestamp for this search
    curr_visitF++;
    check_last_visit(source);
    distF[source] = 0;
    pq.push({0, source});
    
    while (!pq.empty()) {
        auto [d,u] = pq.top(); pq.pop();
        check_last_visit(u);
        if (d != distF[u]) continue;
        if (u == target && d <= directWeight) return true;
        
        for (auto &e : ch_graph.neighbors(u)) {
            int v = e.to;
            if (v == via_v) continue;
            if (contracted[v]) continue;
            
            double nd = d + e.w;
            check_last_visit(v);
            if (nd < distF[v] && nd <= limit) {
                distF[v] = nd;
                pq.push({nd, v});
            }
        }
    }
    return false;
}

/**
 * @brief Regular CH bidirectional query implementation
 * 
 * Performs a bidirectional Dijkstra search in the contraction hierarchy:
 * 1. Forward search from source using upward edges
 * 2. Backward search from target using upward edges
 * 3. Find meeting point with lowest total distance
 * 4. Reconstruct path through the meeting point
 * 
 * @param s Source node ID
 * @param t Target node ID
 * @return DijkstraResult containing distance, path, and statistics
 */
DijkstraResult UnifiedContractionHierarchy::CH_query(int s, int t) {
    if (s == t) {
        distF[s] = 0;
        return {distF, {ch_graph.get_node(s)}, 0};
    }
    curr_visitF++;
    curr_visitB++;
    
    using PQ = std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<std::pair<double,int>>>;
    PQ pqF, pqB;
    
    check_last_visit(s, true);
    distF[s] = 0; pqF.push({0,s});
    check_last_visit(t, false);
    distB[t] = 0; pqB.push({0,t});
    double best = INF;
    size_t nodes_popped = 0;
    int meeting_point = -1;

    auto relax_dir = [&](int u, double du, const std::vector<std::vector<Edge>>& adj,
                         std::vector<double>& curr_dist, std::vector<char>& vis,
                         const std::vector<double>& otherDist, PQ& pq, std::vector<int>& parent,
                        int& meeting_point, bool forward) {
        check_last_visit(u, forward);
        if (du > curr_dist[u]) return;
        vis[u] = 1;
        check_last_visit(u, !forward);
        if (otherDist[u] < INF) {
            if (best > du + otherDist[u])
            {
                best = du + otherDist[u];
                meeting_point = u;
            }
        }
        for (auto &e : adj[u]) {
            int v = e.to;
            double nd = du + e.w;
            check_last_visit(v, forward);
            if (nd < curr_dist[v]) {
                curr_dist[v] = nd;
                parent[v] = u;
                pq.push({nd, v});
            }
        }
    };
    
    while (!pqF.empty() || !pqB.empty()) {
        if (!pqF.empty()) {
            auto [d,u] = pqF.top(); pqF.pop(), nodes_popped++;
            check_last_visit(u, true);
            if (d != distF[u]) continue;
            if (d <= best) relax_dir(u, d, up, distF, visF, distB, pqF, parentF, meeting_point, true);
        }
        
        if (!pqB.empty()) {
            auto [d,u] = pqB.top(); pqB.pop(), nodes_popped++;
            if (d != distB[u]) continue;
            if (d <= best) relax_dir(u, d, up, distB, visB, distF, pqB, parentB, meeting_point, false);
        }
    }

    if (best >= INF || meeting_point < 0) return {{}, {}, 0}; // no path
    
    //update the dist from the meeting point to the target based on the edge weights of the found path in the backward search
    for (int prev = meeting_point, cur = parentB[meeting_point]; cur != -1; prev = cur, cur = parentB[cur]){
        check_last_visit(cur, true);
        check_last_visit(cur, false);
        check_last_visit(prev, true);
        check_last_visit(prev, false);
        distF[cur] = distF[prev] + (distB[prev] - distB[cur]);
    }

    // Get forward path (s to meeting_point)
    std::vector<Node> f_path = Dijkstra::get_path_from_parent(ch_graph, parentF, s, meeting_point);
    // Get backward path (meeting_point to t)
    std::vector<Node> b_path = Dijkstra::get_path_from_parent(ch_graph, parentB, t, meeting_point);

    // Combine paths when the b_path is backward, skip the first node of b_path to avoid duplicating meeting_point
    if (b_path.size() > 1) {
        f_path.insert(f_path.end(), b_path.rbegin() + 1, b_path.rend());
    }

    return {distF, f_path, nodes_popped};
}

/**
 * @brief POI-specific query implementation
 * 
 * Specialized bidirectional search that prioritizes visiting POI nodes
 * and only considers paths that include at least one POI.
 * 
 * @param s Source node ID
 * @param t Target node ID
 * @return DijkstraResult containing distance, path through a POI, and statistics
 */
DijkstraResult UnifiedContractionHierarchy::queryViaPOI(int s, int t) {
    if (!preprocessed){
        std::cout<< "Warning: POI CH query called before preprocessing." << std::endl;
        return {{}, {}, 0};
    }
    // QN is: <isn'tPOI, distance, node>
    // Prioritize popping POI nodes early
    using QN = std::tuple<bool, double, int>;
    using PQ = std::priority_queue<QN, std::vector<QN>, std::greater<QN>>;
    
    if (s == t) {
        distF[s] = 0;
        return {distF, {ch_graph.get_node(s)}, 0};
    }

    curr_visitF++;
    curr_visitB++;

    
    PQ pqF, pqB;

    check_last_visit(s, true);
    distF[s] = 0; pqF.push({!(poi_nodes.count(s) > 0), 0, s});

    check_last_visit(t, false);
    distB[t] = 0; pqB.push({!(poi_nodes.count(t) > 0), 0, t});

    double best = INF;
    size_t nodes_popped = 0;
    int meeting_point = -1;

    auto relax_dir = [&](int u, double du, const std::vector<std::vector<Edge>>& adj,
                         std::vector<double>& curr_dist, std::vector<char>& vis,
                         const std::vector<double>& otherDist, PQ& pq, std::vector<int>& parent,
                        int& meeting_point, bool forward) {
        check_last_visit(u, forward);
        if (du > curr_dist[u]) return;
        vis[u] = 1;
        check_last_visit(u, !forward);
        if (otherDist[u] < INF) {
            if ((best > du + otherDist[u]) && poi_nodes.count(u) > 0)
            {
                best = du + otherDist[u];
                meeting_point = u;
            }
        }
        for (auto &e : adj[u]) {
            int v = e.to;
            double nd = du + e.w;
            check_last_visit(v, forward);
            if (nd < curr_dist[v]) {
                curr_dist[v] = nd;
                parent[v] = u;
                pq.push({!(poi_nodes.count(v) > 0), nd, v});
            }
        }
    };
    
    while (!pqF.empty() || !pqB.empty()) {
        if (!pqF.empty()) {
            auto [temp, d, u] = pqF.top(); pqF.pop(), nodes_popped++;
            check_last_visit(u, true);
            if (d != distF[u]) continue;
            if (d <= best) relax_dir(u, d, up, distF, visF, distB, pqF, parentF, meeting_point, true);
        }
        
        if (!pqB.empty()) {
            auto [temp, d, u] = pqB.top(); pqB.pop(), nodes_popped++;
            check_last_visit(u, false);
            if (d != distB[u]) continue;
            if (d <= best) relax_dir(u, d, up, distB, visB, distF, pqB, parentB, meeting_point, false);
        }
    }

    if (best >= INF || meeting_point < 0) {
        return {{}, {}, 0}; // no path
    }

    //update the dist from the meeting point to the target based on the edge weights of the found path in the backward search
    for (int prev = meeting_point, cur = parentB[meeting_point]; cur != -1; prev = cur, cur = parentB[cur]){
        check_last_visit(cur, true);
        check_last_visit(cur, false);
        check_last_visit(prev, true);
        check_last_visit(prev, false);
        distF[cur] = distF[prev] + (distB[prev] - distB[cur]);
    }

    // Get forward path (s to meeting_point)
    std::vector<Node> f_path = Dijkstra::get_path_from_parent(ch_graph, parentF, s, meeting_point);
    // Get backward path (meeting_point to t)
    std::vector<Node> b_path = Dijkstra::get_path_from_parent(ch_graph, parentB, t, meeting_point);

    // Combine paths when the b_path is backward, skip the first node of b_path to avoid duplicating meeting_point
    if (b_path.size() > 1) {
        f_path.insert(f_path.end(), b_path.rbegin() + 1, b_path.rend());
    }

    return {distF, f_path, nodes_popped};
}

/**
 * @brief Randomly assigns nodes as POIs (Points of Interest)
 * 
 * Designates a specified percentage of nodes as POIs to be used
 * in POI routing queries.
 * 
 * @param percentage Percentage of nodes to designate as POIs (default: 5%)
 */
void UnifiedContractionHierarchy::assignRandomPOIs(double percentage) {
    if (mode != POI_CH) {
        std::cerr << "Warning: assignRandomPOIs called in REGULAR_CH mode\n";
        return;
    }

    if (preprocessed) {
        std::cerr << "Warning: The graph is already preprocessed\n";
        return;
    }

    // Clear any existing POIs
    poi_nodes.clear();

    // Calculate number of POIs based on percentage
    int poi_count = static_cast<int>(n * percentage / 100.0);
    poi_count = std::max(1, poi_count); // Ensure at least one POI
    
    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, n - 1);
    
    // Select POIs randomly
    while (poi_nodes.size() < poi_count) {
        int node_id = dis(gen);
        poi_nodes.insert(node_id);
    }
    
    // Reset preprocessed flag
    preprocessed = false;
}

/**
 * @brief Gets a sorted list of POI node IDs
 * 
 * @return Vector of POI node IDs in ascending order
 */
std::vector<int> UnifiedContractionHierarchy::getPOIList() const {
    if (mode != POI_CH) {
        std::cerr << "Warning: getPOIList called in REGULAR_CH mode\n";
        return {};
    }
    
    std::vector<int> result(poi_nodes.begin(), poi_nodes.end());
    std::sort(result.begin(), result.end());
    return result;
}

/**
 * @brief Finds shortest path that visits at least one POI
 * 
 * Wrapper for POI-specific query that ensures preprocessing is done.
 * 
 * @param s Source node ID
 * @param t Target node ID
 * @return DijkstraResult containing distance, path through a POI, and statistics
 */
DijkstraResult UnifiedContractionHierarchy::findShortestPathViaPOI(int s, int t) {
    if (mode != POI_CH) {
        std::cerr << "Warning: findShortestPathViaPOI called in REGULAR_CH mode, falling back to CH query\n";
        return CH_query(s, t); // Fall back to CH query
    }
    
    // Ensure CH is preprocessed
    if (!preprocessed) {
        preprocess();
    }
    
    return queryViaPOI(s, t);
}

/**
 * @brief Unified query function that uses the appropriate algorithm based on mode
 * 
 * Dispatches to either regular CH query or POI CH query based on the mode.
 * 
 * @param s Source node ID
 * @param t Target node ID
 * @return DijkstraResult containing distance, path, and statistics
 */
DijkstraResult UnifiedContractionHierarchy::query(int s, int t) {
    return (mode == REGULAR_CH) ? CH_query(s, t) : queryViaPOI(s, t);
}

/**
 * @brief Expands a shortcut path to its full underlying path
 * 
 * Converts a path that may contain shortcuts into the full path
 * with all intermediate nodes.
 * 
 * @param path Path with possible shortcuts
 * @param weight Expected total path weight (used for optimization)
 * @return Expanded path with all intermediate nodes
 */
std::vector<Node> UnifiedContractionHierarchy::expand_shortcut_path(const std::vector<Node>& path, double weight) const {
    if (path.empty()) return {};
    if (path.size() == 1) return path;

    std::vector<int> ids;
    ids.reserve(static_cast<size_t>(weight));
    ids.push_back(path[0].id);
    
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        int u = path[i].id;
        int v = path[i + 1].id;
        expand_edge(u, v, ids);
    }
    
    // Convert ids back to Node objects
    std::vector<Node> expanded;
    expanded.reserve(ids.size());
    for (int id : ids) {
        expanded.push_back(ch_graph.get_node(id));
    }
    
    return expanded;
}

/**
 * @brief Expands a single edge, recursively handling shortcuts
 * 
 * Recursively expands shortcuts to include all intermediate nodes
 * in the path.
 * 
 * @param u Source node ID
 * @param v Target node ID
 * @param out Vector to append expanded nodes to
 */
void UnifiedContractionHierarchy::expand_edge(int u, int v, std::vector<int>& out) const {
    // Locate the edge u -> v
    int contracted_mid = -2; // -2 = edge not found sentinel
    for (const auto& e : ch_graph.neighbors(u)) {
        if (e.to == v) {
            contracted_mid = e.contracted; // -1 => original edge, >=0 => shortcut via that node id
            break;
        }
    }
    
    // If edge missing or original -> just append v
    if (contracted_mid == -2 || contracted_mid == -1) {
        if (out.empty() || out.back() != v) out.push_back(v);
        return;
    }
    
    // Shortcut: recursively expand (u, mid) then (mid, v)
    int mid = contracted_mid;
    expand_edge(u, mid, out);
    expand_edge(mid, v, out);
}

/**
 * @brief Exports all edges for visualization or analysis
 * 
 * @return Vector of tuples (source, target, weight, contracted_node)
 */
std::vector<std::tuple<int,int,double,int>> UnifiedContractionHierarchy::export_all_edges() const {
    std::vector<std::tuple<int,int,double,int>> out;
    for (int u = 0; u < n; ++u) {
        for (auto &e : ch_graph.neighbors(u)) {
            if (u < e.to) {
                out.emplace_back(u, e.to, e.w, e.contracted);
            }
        }
    }
    return out;
}

/**
 * @brief Gets the total number of edges in the hierarchy
 * 
 * @return Number of edges (including shortcuts)
 */
size_t UnifiedContractionHierarchy::get_num_of_edges() const {
    return ch_graph.get_num_of_edges();
}

/**
 * @brief Prints debug information about the hierarchy
 * 
 * Outputs detailed information about the contraction hierarchy
 * including node ranks, contracted status, edges, and POIs.
 * 
 * @param os Output stream to print to
 */
void UnifiedContractionHierarchy::print_debug(std::ostream& os) const {
    os << "=== " << (mode == REGULAR_CH ? "Regular" : "POI") << " Contraction Hierarchy Debug ===\n";
    os << "Nodes: " << n << "\n\n";

    os << "Rank / Contracted:\n";
    for (int i = 0; i < n; ++i) {
        os << "  " << i << ": rank=" << rank[i]
           << " contracted=" << (contracted[i] ? 1 : 0);
        
        if (mode == POI_CH && poi_nodes.count(i) > 0) {
            os << " (POI)";
        }
        
        os << "\n";
    }
    os << "\n";

    auto dump_adj = [&](const char* title, const std::vector<std::vector<Edge>>& adj) {
        os << title << " edges:\n";
        for (int i = 0; i < n; ++i) {
            if (adj[i].empty()) continue;
            os << "  " << i << ":";
            for (auto const& e : adj[i]) {
                os << " (" << e.to << "," << e.w << ")";
            }
            os << "\n";
        }
        os << "\n";
    };

    dump_adj("Up", up);
    
    if (mode == POI_CH) {
        os << "POIs (" << poi_nodes.size() << "):\n  ";
        for (int poi : poi_nodes) {
            os << poi << " ";
        }
        os << "\n";
    }
    
    os << "\nUnderlying graph edge count: " << get_num_of_edges() << "\n";
    os << "==================================\n";
}

/**
 * @brief Prints debug information to standard output
 */
void UnifiedContractionHierarchy::print_debug() const {
    print_debug(std::cout);
}

/**
 * @brief Resets visit data for a node using timestamp-based method
 * 
 * Uses a timestamp-based approach to efficiently reset distance,
 * visited, and parent data for a node.
 * 
 * @param nodeId Node ID to check/reset
 * @param forward Whether this is for forward search
 */
void UnifiedContractionHierarchy::check_last_visit(int nodeId, bool forward) {
    if (forward) {
        if (last_visitF[nodeId] < curr_visitF) {
            last_visitF[nodeId] = curr_visitF;
            distF[nodeId] = INF;
            visF[nodeId] = false;
            parentF[nodeId] = -1;
        }
    } else {
        if (last_visitB[nodeId] < curr_visitB) {
            last_visitB[nodeId] = curr_visitB;
            distB[nodeId] = INF;
            visB[nodeId] = false;
            parentB[nodeId] = -1;
        }
    }
}
