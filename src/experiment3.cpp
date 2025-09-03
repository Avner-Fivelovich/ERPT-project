#include "experiment3.hpp"
#include <set>
#include <sstream>
#include <filesystem>
#include <iomanip>
#include <json.hpp>
#include <thread>

using json = nlohmann::json;
#define NOW() std::chrono::high_resolution_clock::now()
#define TO_MILI(x) std::chrono::duration_cast<std::chrono::milliseconds>(x)
#define TO_MICRO(x) std::chrono::duration_cast<std::chrono::microseconds>(x)

/**
 * @brief Runs experiments for POI routing using Contraction Hierarchies
 * 
 * This implementation:
 * 1. Loads graph data from an OSM dataset
 * 2. Randomly designates 5% of nodes as Points of Interest (POIs)
 * 3. Performs CH preprocessing optimized for POI routing
 * 4. Runs test queries to find paths between random points that must visit a POI
 * 5. Compares these paths with regular shortest paths
 * 6. Generates visualization data and starts an interactive web application
 * 
 * Results are written to the specified output file and an interactive visualization
 * is created to allow users to explore the POI routing capabilities.
 * 
 * @param output_file Path for the output file where results will be written
 * @param osm_number OSM dataset number to run experiments on (default: 5)
 */
void part_3_experiments(std::string output_file, int osm_number) {
    std::cout << "Starting Part 3 experiments with OSM" << osm_number << std::endl;
    srand(static_cast<unsigned int>(time(nullptr)));
    auto start = NOW(), end = NOW();
    
    // Phase 1: Load graph from OSM dataset
    std::cout << "Loading graph from OSM" << osm_number << "..." << std::endl;
    Graph graph = get_graphs_from_osm_numbers({osm_number})[0];
    
    // Open output file for writing results
    std::ofstream output(output_file);
    if (!output.is_open()) {
        std::cerr << "Failed to open output file: " << output_file << std::endl;
        exit(1);
    }

    output << "Results for POI routing experiments:" << std::endl;
    output << "====================================" << std::endl << std::endl;

    output << "Graph osm" << osm_number << ":" << std::endl;
    output << "Number of nodes: " << graph.node_count() << std::endl;
    output << "Number of edges: " << graph.get_num_of_edges() << std::endl;
    
    // Phase 2: Create POI routing structure and designate POIs
    std::cout << "Creating POI routing structure..." << std::endl;
    UCH poi_query(graph, UCH::Mode::POI_CH);
    
    // Assign 5% of nodes as POIs
    poi_query.assignRandomPOIs(5.0);
    auto poi_list = poi_query.getPOIList();
    output << "Number of POIs: " << poi_list.size() << std::endl;
    
    // Phase 3: Preprocess the CH structure for POI routing
    std::cout << "Running CH preprocessing for POI routing..." << std::endl;
    start = NOW();
    poi_query.preprocess();
    end = NOW();
    
    // Calculate and report preprocessing statistics
    auto duration = TO_MILI(end - start);
    auto total_ms = duration.count();
    auto seconds = (total_ms % 60000) / 1000;
    auto minutes = total_ms / 60000;
    auto milliseconds = total_ms % 1000;

    output << "Preprocessing time: " << minutes << " min " << seconds << " sec " << milliseconds << " ms" << std::endl;

    // Phase 4: Run test queries to evaluate POI routing performance
    std::cout << "Running test queries..." << std::endl;
    int num_queries = 20;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, graph.node_count() - 1);
    
    int successful_queries = 0;
    double total_time_ms = 0;
    double total_nodes_popped = 0;
    
    output << "\nRunning " << num_queries << " random queries:" << std::endl;
    
    for (int j = 0; j < num_queries; ++j) {
        // Generate random source and target nodes
        int s = dis(gen);
        int t = dis(gen);
        
        // Perform POI routing query - find path from s to t that visits at least one POI
        start = NOW();
        auto result = poi_query.query(s, t);
        end = NOW();
        auto query_time = TO_MICRO(end - start);
        
        if (!result.path.empty()) {
            successful_queries++;
            total_time_ms += query_time.count() / 1000.0;
            total_nodes_popped += result.nodes_popped;
            
            output << "Query " << j+1 << ": " << s << " -> " << t << " via POI" << std::endl;
            output << "  Path length: " << result.path.size() << " nodes" << std::endl;
            output << "  Distance: " << result.dist[t] << std::endl;
            output << "  Nodes popped: " << result.nodes_popped << std::endl;
            output << "  Query time: " << query_time.count() / 1000.0 << " ms" << std::endl;
            
            // Output path (limiting to 10 nodes for readability)
            output << "  Path: ";
            for (size_t k = 0; k < std::min<size_t>(result.path.size(), 10); ++k) {
                output << result.path[k].id;
                if (k < result.path.size() - 1) output << " -> ";
            }
            if (result.path.size() > 10) output << " -> ... -> " << result.path.back().id;
            output << std::endl;
            
            // Check if path contains a POI (validation check)
            bool contains_poi = false;
            for (const auto& node : result.path) {
                if (poi_query.getPOIs().count(node.id) > 0) {
                    contains_poi = true;
                    output << "  Contains POI node: " << node.id << std::endl;
                    break;
                }
            }
            
            if (!contains_poi) {
                output << "  ERROR: Path does not contain a POI!" << std::endl;
            }
        } else {
            output << "Query " << j+1 << ": " << s << " -> " << t << " (No path found)" << std::endl;
        }
        
        output << "  ---" << std::endl;
    }
    
    // Output summary statistics
    if (successful_queries > 0) {
        output << "\nSummary for osm" << osm_number << ":" << std::endl;
        output << "Successful queries: " << successful_queries << "/" << num_queries << std::endl;
        output << "Average query time: " << (total_time_ms / successful_queries) << " ms" << std::endl;
        output << "Average nodes popped: " << (total_nodes_popped / successful_queries) << std::endl;
    } else {
        output << "\nNo successful queries for osm" << osm_number << std::endl;
    }
    
    output << "====================================" << std::endl << std::endl;
    
    output.close();
    std::cout << "POI routing experiments completed. Results written to " << output_file << std::endl;

    // Phase 5: Generate visualization data for the interactive web application
    std::cout << "Generating visualization data..." << std::endl;
    experimentInteractiveMap(poi_query);
    
    // Phase 6: Start the web server to host the interactive visualization
    std::cout << "Starting interactive web application for visualization..." << std::endl;
    std::cout << "You can select start and end nodes on the map to compare:" << std::endl;
    std::cout << "  - Regular shortest path" << std::endl;
    std::cout << "  - Shortest path visiting at least one POI" << std::endl;
    startVisualizationServer(poi_query);
}

/**
 * @brief Generates the JSON data required for the interactive map visualization
 * 
 * This creates a data file containing:
 * - Graph nodes with their geographic coordinates
 * - Regular edges representing roads
 * - Shortcut edges created by the contraction hierarchy
 * - POI node identifiers
 * 
 * @param poi_query The POI routing object with the processed graph
 * @return bool Success status of the operation
 */
bool experimentInteractiveMap(UCH& poi_query) {
    // Create the output directory
    std::filesystem::create_directories("results/part_3");
    
    // Generate the JSON data file
    json data;
    
    // Add graph data
    data["graph"]["nodes"] = json::array();
    data["graph"]["regular_edges"] = json::array();
    data["graph"]["shortcut_edges"] = json::array();
    
    // Add nodes with their geographic coordinates
    const Graph& graph = poi_query.underlying_graph();
    for (int i = 0; i < graph.node_count(); i++) {
        Node node = graph.get_node(i);
        data["graph"]["nodes"].push_back({
            {"id", node.id},
            {"lat", node.latitude},
            {"lon", node.longitude}
        });
    }
    
    // Add edges (distinguishing between regular edges and shortcuts)
    auto ch_edges = poi_query.export_all_edges();
    for (auto &ce : ch_edges) {
        int u, v, w, c; std::tie(u, v, w, c) = ce;
        if (c == -1) {
            // Regular edge (actual road)
            data["graph"]["regular_edges"].push_back({
                {"source", u}, {"target", v}, {"weight", w}
            });
        } else {
            // Shortcut edge (created by CH preprocessing)
            data["graph"]["shortcut_edges"].push_back({
                {"source", u}, {"target", v}, {"weight", w}, {"contracted_mid", c}
            });
        }
    }
    
    // Add POIs for visualization
    data["pois"] = json::array();
    for (int poi_id : poi_query.getPOIs()) {
        data["pois"].push_back({
            {"id", poi_id}
        });
    }
    
    // Write the JSON data to file
    std::string json_path = "results/part_3/visualization_data.json";
    std::ofstream json_file(json_path);
    if (!json_file.is_open()) {
        std::cerr << "Failed to open JSON file: " << json_path << std::endl;
        return false;
    }
    json_file << std::setw(2) << data << std::endl;
    json_file.close();
    
    std::cout << "Visualization data written to " << json_path << std::endl;
    std::cout << "Open the HTML file in your browser to view the map" << std::endl;
    
    return true;
}

/**
 * @brief Starts a local web server to host the interactive visualization application
 * 
 * The server provides:
 * - Static files for the visualization interface
 * - API endpoints for path computation
 * - Automatic browser opening for the visualization
 * 
 * @param poi_query The POI routing object to handle path queries
 * @param port Port number for the server
 * @return bool Success status of the server startup
 */
bool startVisualizationServer(UCH& poi_query, int port) {
    httplib::Server server;
    
    // Serve static files (HTML, CSS, JavaScript) from the results directory
    server.set_mount_point("/", "results/part_3");
    
    // API endpoint for path finding and comparison
    server.Get("/api/compare_paths", [&poi_query](const httplib::Request& req, httplib::Response& res) {
            handleComparePaths(req, res, poi_query);
        });
    
    // Open the visualization in the default browser
    std::string url = "http://localhost:" + std::to_string(port);
    
    #ifdef _WIN32
        std::string cmd = "start \"\" \"" + url + "\"";
        system(cmd.c_str());
    #elif __APPLE__
        std::string cmd = "open \"" + url + "\"";
        system(cmd.c_str());
    #else
        std::string cmd = "xdg-open \"" + url + "\"";
        int result = system(cmd.c_str());
        if (result != 0) {
            std::cout << "Please open " << url << " in your browser\n";
        }
    #endif
    
    // Give browser time to start
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::cout << "Web server started on port " << port << std::endl;
    std::cout << "Press Ctrl+C to stop the server" << std::endl;
    
    // Start listening for requests
    return server.listen("localhost", port);
}

/**
 * @brief Handles API requests to compare regular shortest paths with POI-visiting paths
 * 
 * This function:
 * 1. Parses the start and end node parameters from the request
 * 2. Computes both regular and POI-visiting paths
 * 3. Formats the results as JSON for the frontend visualization
 * 
 * @param req HTTP request object containing query parameters
 * @param res HTTP response object for returning the results
 * @param poi_query The POI routing object to process the queries
 */
void handleComparePaths(const httplib::Request& req, httplib::Response& res, UCH& poi_query) {
    int start_node = -1, end_node = -1;
    
    // Parse start and end node parameters from the request
    if (req.has_param("start")) {
        start_node = std::stoi(req.get_param_value("start"));
    }
    
    if (req.has_param("end")) {
        end_node = std::stoi(req.get_param_value("end"));
    }
    
    // Validate inputs
    if (start_node < 0 || end_node < 0) {
        json error = {
            {"status", "error"},
            {"message", "Invalid start or end node"}
        };
        res.set_content(error.dump(), "application/json");
        return;
    }
    
    // Compute both paths:
    // 1. POI path - shortest path that visits at least one POI
    // 2. Regular path - standard shortest path (may or may not visit a POI)
    DijkstraResult poi_result = poi_query.findShortestPathViaPOI(start_node, end_node);
    DijkstraResult regular_result = poi_query.CH_query(start_node, end_node);
    
    json response;
    response["status"] = "success";
    
    // Process regular path (with shortcuts)
    if (!regular_result.path.empty()) {
        std::vector<int> regular_path_ids;
        for (const auto& node : regular_result.path) {
            regular_path_ids.push_back(node.id);
        }
        response["regular_path"] = regular_path_ids;
        response["regular_distance"] = regular_result.dist[end_node];
        
        // Also add the unpacked (expanded) path for visualization
        auto expanded_path = poi_query.expand_shortcut_path(regular_result.path, regular_result.dist[end_node]);
        std::vector<int> expanded_path_ids;
        for (const auto& node : expanded_path) {
            expanded_path_ids.push_back(node.id);
        }
        response["regular_path_unpacked"] = expanded_path_ids;
    } else {
        response["regular_path"] = json::array();
        response["regular_path_unpacked"] = json::array();
        response["regular_distance"] = 0;
    }
    
    // Process POI path (with shortcuts)
    if (!poi_result.path.empty()) {
        std::vector<int> poi_path_ids;
        for (const auto& node : poi_result.path) {
            poi_path_ids.push_back(node.id);
        }
        response["poi_path"] = poi_path_ids;
        response["poi_distance"] = poi_result.dist[end_node];
        
        // Also add the unpacked (expanded) POI path for visualization
        auto expanded_poi_path = poi_query.expand_shortcut_path(poi_result.path, poi_result.dist[end_node]);
        std::vector<int> expanded_poi_path_ids;
        for (const auto& node : expanded_poi_path) {
            expanded_poi_path_ids.push_back(node.id);
        }
        response["poi_path_unpacked"] = expanded_poi_path_ids;
        
        // Count POIs on the path (for statistics)
        int poi_count = 0;
        for (const auto& node : poi_result.path) {
            if (poi_query.getPOIs().count(node.id) > 0) {
                poi_count++;
            }
        }
        response["poi_count"] = poi_count;
    } else {
        response["poi_path"] = json::array();
        response["poi_path_unpacked"] = json::array();
        response["poi_distance"] = 0;
        response["poi_count"] = 0;
    }
    
    // Return the response as JSON
    res.set_content(response.dump(), "application/json");
}
