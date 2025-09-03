#include "experiment1.hpp"
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
 * @brief Runs experiments comparing Dijkstra's algorithm with Contraction Hierarchies
 * 
 * This implementation:
 * 1. Loads graph data from OSM datasets
 * 2. Performs CH preprocessing and measures:
 *    - Preprocessing time
 *    - Number of edges added
 *    - Memory overhead
 * 3. Runs comparative analysis between original Dijkstra and CH-Dijkstra:
 *    - Execution time
 *    - Number of nodes explored
 *    - Path correctness verification
 * 4. Generates visualization for OSM5 graph to illustrate:
 *    - Original graph structure
 *    - CH graph with shortcuts
 *    - Example shortest paths computed by both algorithms
 * 
 * Results are written to the specified output file and visualizations are generated
 * for the OSM5 dataset when included in the experiment.
 * 
 * @param output_file Path for the output file where results will be written
 * @param osm_numbers List of OSM dataset numbers to run experiments on
 * @param mode Mode for the Contraction Hierarchy (REGULAR_CH or POI_CH)
 */
void part_1_experiments(std::string output_file, std::vector<int> osm_numbers, UCH::Mode mode) {
    std::cout << "Starting Part 1 experiments with mode: " << (mode == UCH::Mode::REGULAR_CH ? "REGULAR_CH" : "POI_CH") << std::endl;
    srand(static_cast<unsigned int>(time(nullptr)));
    using pairs_of_int = std::vector<std::pair<int, int>>;
    auto start = NOW(), end = NOW();
    
    // Load graphs from OSM datasets
    std::vector<Graph> graphs = get_graphs_from_osm_numbers(osm_numbers); 
    std::vector<UCH> CH_graphs;
    CH_graphs.reserve(graphs.size());

    // Open output file for writing results
    std::ofstream file(output_file);
    if (!file.is_open()) {
        std::cout << "Failed to open file: " << output_file << std::endl;
        exit(1);
    }

    // Phase 1: Run CH preprocessing on all road networks
    file << "Running the CH preprocessing algorithm on all road networks in the provided data set:" << std::endl;
    for (size_t i = 0; i < graphs.size(); i++) {
        Graph &curr_graph = graphs[i];
        size_t num_of_original_edges = curr_graph.get_num_of_edges();
        file << "Running CH on osm" << osm_numbers[i] << ":" << std::endl;
        
        // Measure preprocessing time
        start = NOW();
        CH_graphs.push_back(UCH(curr_graph, mode));
        UCH &CH_graph = CH_graphs.back();
        CH_graph.preprocess(); // Run the CH preprocessing
        end = NOW();
        
        // Calculate and report preprocessing statistics
        auto duration = TO_MILI(end - start);
        size_t num_of_new_edges = CH_graph.get_num_of_edges();
        file << "Number of edges before CH: " << num_of_original_edges << ", after CH: " << num_of_new_edges << "." << std::endl;
        size_t num_of_added_edges = num_of_new_edges - num_of_original_edges;
        file << "Number of edges added by CH: " << num_of_added_edges << "." << std::endl;
        if (num_of_original_edges == 0) {
            file << "No edges in the graph, cannot calculate percentage of edges added." << std::endl;
        } else {
            file << "Percentage of edges added: " << (100 * static_cast<double>(num_of_added_edges) / num_of_original_edges) << "%." << std::endl;
        }
        auto total_ms = duration.count();
        auto seconds = (total_ms % 60000) / 1000;
        auto minutes = total_ms / 60000;
        auto milliseconds = total_ms % 1000;
        file << "Time taken for CH preprocessing on osm" << osm_numbers[i] << ": "
                << minutes << " minutes " << seconds << " seconds and " << milliseconds << " ms." << std::endl;
        file << "--------------------------------------------------" << std::endl;
    }

    // Phase 2: Compare Dijkstra and CH-Dijkstra performance
    file << "Running Dijkstra on the original graphs and CH-Dijkstra on the " << (mode == UCH::Mode::REGULAR_CH ? "regular CH graphs" : "POI CH graphs") << std::endl;

    // Store results for plotting and analysis
    std::vector<std::tuple<int, std::pair<double, double>, std::pair<double, double>>> results_for_plot(osm_numbers.size()); 

    // Store example paths for OSM5 visualization
    pairs_of_int example_pairs;
    
    // Iterate through each OSM dataset
    for (size_t i = 0; i < graphs.size(); i++) {
        Graph &curr_graph = graphs[i];
        UCH &CH_graph = CH_graphs[i];
        
        // Generate random source-target pairs for testing
        int num_of_pairs = 100 + (rand() % 100); // Between 100-200 random pairs
        pairs_of_int pairs(num_of_pairs);
        for (int j = 0; j < num_of_pairs; j++) {
            int first = rand() % curr_graph.node_count();
            int second = rand() % curr_graph.node_count();
            pairs[j] = {first, second};
        }
        
        // Save example pairs for OSM5 visualization
        if (osm_numbers[i] == 5) {
            for (size_t i = 0; i < 4; i++) {
                example_pairs.push_back(pairs[rand() % pairs.size()]);
            }
        }

        // Initialize performance metrics
        size_t dijkstra_runtime = 0, CH_dijkstra_runtime = 0;
        int dijkstra_nodes_popped = 0, CH_dijkstra_nodes_popped = 0;
        int64_t dijkstra_dist = -1, CH_dijkstra_dist = -1;

        file << "Running Dijkstra and CH-Dijkstra on osm" << osm_numbers[i] << " on " << pairs.size() << " pairs." << std::endl;
        
        // Run Dijkstra and CH-Dijkstra on each source-target pair
        for (int j = 0; j < pairs.size(); j++) {        
            bool print = false; // Set to true to print individual path details
            auto [start_node, end_node] = pairs[j];

            // Run standard Dijkstra
            start = NOW();
            auto DijkstraResult = Dijkstra::run(curr_graph, start_node, end_node);
            end = NOW();
            dijkstra_runtime += TO_MICRO(end - start).count();
            dijkstra_nodes_popped += DijkstraResult.nodes_popped;
            
            // Check if Dijkstra found a path
            if (DijkstraResult.dist.size() == 0) {
                file << "Error: Dijkstra returned no path from " << start_node << " to " << end_node << "." << std::endl;
                dijkstra_dist = -1;
            } else {
                dijkstra_dist = DijkstraResult.dist[end_node];
            }
            
            // Run CH-Dijkstra
            start = NOW();
            auto CH_DijkstraResult = CH_graph.query(start_node, end_node);
            auto expanded_CH_path = CH_graph.expand_shortcut_path(CH_DijkstraResult.path, CH_DijkstraResult.dist[end_node]);
            end = NOW();
            CH_dijkstra_runtime += TO_MICRO(end - start).count();
            CH_dijkstra_nodes_popped += CH_DijkstraResult.nodes_popped;
            
            // Check if CH-Dijkstra found a path
            if (CH_DijkstraResult.dist.size() == 0) {
                file << "Error: CH-Dijkstra returned no path from " << start_node << " to " << end_node << "." << std::endl;
                CH_dijkstra_dist = -1;
                if (dijkstra_dist != -1) continue;
            } else {
                CH_dijkstra_dist = CH_DijkstraResult.dist[end_node];
            }
            
            // Verify that both algorithms returned the same path length
            if (dijkstra_dist != CH_dijkstra_dist || DijkstraResult.path.size() != expanded_CH_path.size()) {
                file << "Error: Dijkstra and CH-Dijkstra returned different weights for the path from " << start_node << " to " << end_node << "!" << std::endl;
                file << "Weight of the path is: " << dijkstra_dist << "." << std::endl;
                file << "Weight of the CH path is: " << CH_DijkstraResult.dist[end_node] << "." << std::endl;
                file << "Shortest path from " << start_node << " to " << end_node << " is: " << get_vector_as_string(DijkstraResult.path) << std::endl;
                file << "Shortest path from " << start_node << " to " << end_node << " is: " << get_vector_as_string(expanded_CH_path) << std::endl;
                file << "--------------------------------------------------" << std::endl;
            }
        }
        
        // Store results for later analysis and visualization
        results_for_plot[i] = std::make_tuple(osm_numbers[i], 
            std::make_pair(static_cast<double>(dijkstra_runtime) / num_of_pairs, static_cast<double>(dijkstra_nodes_popped) / num_of_pairs), 
            std::make_pair(static_cast<double>(CH_dijkstra_runtime) / num_of_pairs, static_cast<double>(CH_dijkstra_nodes_popped) / num_of_pairs));
        
        // Report average performance metrics
        file << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
        
        // Format and output Dijkstra average runtime
        double total_us = (static_cast<double>(dijkstra_runtime) / num_of_pairs);
        auto seconds = static_cast<int>(total_us / 1e6);
        auto milliseconds = static_cast<int>(total_us / 1e3) % 1000;
        auto microseconds = static_cast<int>(total_us) % 1000;
        file << "Average running time of Dijkstra on osm" << osm_numbers[i] << ": " << seconds << " seconds " << milliseconds << " ms and " << microseconds << " µs." << std::endl;

        // Format and output CH-Dijkstra average runtime
        total_us = (static_cast<double>(CH_dijkstra_runtime) / num_of_pairs);
        seconds = static_cast<int>(total_us / 1e6);
        milliseconds = static_cast<int>(total_us / 1e3) % 1000;
        microseconds = static_cast<int>(total_us) % 1000;
        file << "Average running time of CH-Dijkstra on osm" << osm_numbers[i] << ": " << seconds << " seconds " << milliseconds << " ms and " << microseconds << " µs." << std::endl;

        // Calculate and report runtime difference
        total_us = (abs(static_cast<double>(CH_dijkstra_runtime) - static_cast<double>(dijkstra_runtime)) / num_of_pairs);
        seconds = static_cast<int>(total_us / 1e6);
        milliseconds = static_cast<int>(total_us / 1e3) % 1000;
        microseconds = static_cast<int>(total_us) % 1000;
        file << "difference in average running time between Dijkstra and CH-Dijkstra on osm" << osm_numbers[i] << ": " << seconds << " seconds " << milliseconds << " ms and " << microseconds << " µs." << std::endl;
        file << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;

        // Report nodes popped from priority queue statistics
        file << "Average number of nodes popped from the priority queue in Dijkstra on osm" << osm_numbers[i] << ": " << (static_cast<double>(dijkstra_nodes_popped) / num_of_pairs) << "." << std::endl;
        file << "Average number of nodes popped from the priority queue in CH-Dijkstra on osm" << osm_numbers[i] << ": " << (static_cast<double>(CH_dijkstra_nodes_popped) / num_of_pairs) << "." << std::endl;
        file << "Difference in average number of nodes popped from the priority queue between Dijkstra and CH-Dijkstra on osm" << osm_numbers[i] << ": " << (std::abs(static_cast<double>(CH_dijkstra_nodes_popped - dijkstra_nodes_popped)) / num_of_pairs) << "." << std::endl;
        file << "**************************************************" << std::endl;
    }
    
    // Phase 3: Generate summary table
    file << "\nSummary table (times in us, popped = avg nodes popped):\n";
    file << "osm\tDijkstraTime\tDijkstraPopped\tCHTime\tCHPopped\n";
    for (const auto &entry : results_for_plot) {
        auto [osm, dijkstra, ch] = entry;
        file << osm << '\t'
                << dijkstra.first << '\t' << dijkstra.second << '\t'
                << ch.first << '\t' << ch.second << '\n';
    }

    // Phase 4: Generate visualization for OSM5 dataset
    for (size_t i=0; i<osm_numbers.size(); ++i) {
        if (osm_numbers[i] == 5) {
            Graph &base = graphs[i];
            UCH &CHg = CH_graphs[i];

            // Ensure we have results for 4 examples
            if (example_pairs.size() > 4) {
                example_pairs.resize(4);
            }

            // Extract base graph edges for visualization
            auto edges_base = [&](){
                std::vector<std::tuple<int,int,double>> out;
                for (int u=0; u<base.node_count(); ++u) {
                    for (auto &e : base.neighbors(u)) {
                        if (u < e.to) out.push_back({u, e.to, e.w});
                    }
                }
                return out;
            }();

            // Extract CH edges (both regular and shortcuts)
            auto ch_edges = CHg.export_all_edges();

            // Create output directory if it doesn't exist
            std::filesystem::create_directories("results/part_1");
            std::string json_path = "results/part_1/osm5_visualization.json";

            // Build JSON for visualization
            nlohmann::json j;
            j["meta"] = { {"description","OSM5 visualization dataset"}, {"examples_count", example_pairs.size()}};

            // Add nodes with coordinates
            j["nodes"] = json::array();
            for (auto &nd : base.get_all_nodes()) {
                if (nd.id < 0) continue;
                j["nodes"].push_back({
                    {"id", nd.id},
                    {"lat", nd.latitude},
                    {"lon", nd.longitude}
                });
            }

            // Add original graph edges
            j["graph_edges"] = json::array();
            for (auto &e : edges_base) {
                j["graph_edges"].push_back({ {"u", std::get<0>(e)}, {"v", std::get<1>(e)}, {"w", std::get<2>(e)} });
            }

            // Add CH edges, separating regular edges and shortcuts
            j["regular_edges"] = json::array();
            j["shortcut_edges"] = json::array();
            for (auto &ce : ch_edges) {
                int u, v, w, c; std::tie(u, v, w, c) = ce;
                if (c == -1) {
                    // Regular edge
                    j["regular_edges"].push_back({
                        {"u", u}, {"v", v}, {"w", w}
                    });
                } else {
                    // Shortcut edge
                    j["shortcut_edges"].push_back({
                        {"u", u}, {"v", v}, {"w", w}, {"contracted_mid", c}
                    });
                }
            }

            // Add example paths for visualization
            j["examples"] = json::array();
            for (auto &ex : example_pairs) {
                auto dijkstra_path = Dijkstra::run(base, ex.first, ex.second).path;
                auto ch_path_shortcut = CHg.query(ex.first, ex.second);
                auto ch_path_unpacked = CHg.expand_shortcut_path(ch_path_shortcut.path, ch_path_shortcut.dist[ex.second]);
                j["examples"].push_back({
                    {"source", ex.first},
                    {"target", ex.second},
                    {"dijkstra_path", get_vector_as_string(dijkstra_path)},
                    {"ch_path_shortcut", get_vector_as_string(ch_path_shortcut.path)},
                    {"ch_path_unpacked", get_vector_as_string(ch_path_unpacked)}
                });
            }

            // Write visualization data to JSON file
            std::ofstream jf(json_path);
            jf << std::setw(2) << j << "\n";
            jf.close();
            std::cout << "Wrote visualization JSON to " << json_path << "\n";

            // Get full path to the project folder for server
            std::string vis_path = std::filesystem::current_path().string();
            
            // Normalize path for current OS
            #ifdef _WIN32
                // Replace forward slashes with backslashes for Windows
                std::replace(vis_path.begin(), vis_path.end(), '/', '\\');
            #endif
            
            std::cout << "Starting visualization server for " << vis_path << " on port 8000...\n";

            // Start local HTTP server to serve visualization files
            std::string server_cmd;
            #ifdef _WIN32
                server_cmd = "start /b cmd /c \"cd \"" + vis_path + "\" && python -m http.server 8000\"";
            #else
                server_cmd = "cd \"" + vis_path + "\" && python -m http.server 8000 &";
            #endif
            system(server_cmd.c_str());

            // Give server time to start
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // Open example files in browser
            for (int i = 1; i <= 4; ++i) {
                std::string url = "http://localhost:8000/visualization/osm5_example" + std::to_string(i) + ".html";
                std::cout << "Opening " << url << "\n";
                
                #ifdef _WIN32
                    std::string cmd = "start \"\" \"" + url + "\"";
                #elif __APPLE__
                    std::string cmd = "open \"" + url + "\"";
                #else
                    std::string cmd = "xdg-open \"" + url + "\"";
                    int result = system(cmd.c_str());
                    if (result != 0) {
                        std::cout << "Please open " << url << " in your browser\n";
                    }
                #endif
                
                system(cmd.c_str());
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            // Open CH graph visualization
            std::string url = "http://localhost:8000/visualization/osm5_ch_graph.html";
            std::cout << "Opening " << url << "\n";
            
            #ifdef _WIN32
                std::string cmd = "start \"\" \"" + url + "\"";
            #elif __APPLE__
                std::string cmd = "open \"" + url + "\"";
            #else
                std::string cmd = "xdg-open \"" + url + "\"";
                int result = system(cmd.c_str());
                if (result != 0) {
                    std::cout << "Please open " << url << " in your browser\n";
                }
            #endif
            
            system(cmd.c_str());
            std::this_thread::sleep_for(std::chrono::seconds(1));    

            // Wait for files to load
            std::cout << "Visualization files are loading, please wait...\n";
            std::this_thread::sleep_for(std::chrono::seconds(5));

            // Stop server
            #ifdef _WIN32
                system("taskkill /f /im python.exe > nul 2>&1");
            #else
                system("pkill -f \"python -m http.server 8000\" > /dev/null 2>&1");
            #endif

            std::cout << "Visualization complete.\n";

            break;
        }
    }

    file.close();
    std::cout << "Experiments completed. Results written to " << output_file << std::endl;
}