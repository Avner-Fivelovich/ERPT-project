#include "experiment2.hpp"
#include <set>
#include <sstream>
#include <filesystem>
#include <iomanip>
#include <json.hpp>
#include <iostream>
#include <thread>

// OS detection using preprocessor macros
#if defined(_WIN32) || defined(_WIN64)
    #define IS_WINDOWS true
#else
    #define IS_WINDOWS false
#endif

using json = nlohmann::json;
#define NOW() std::chrono::high_resolution_clock::now()
#define TO_MILI(x) std::chrono::duration_cast<std::chrono::milliseconds>(x)
#define TO_MICRO(x) std::chrono::duration_cast<std::chrono::microseconds>(x)

/**
 * @brief Helper function that implements the actual CCH experiments
 * 
 * This function performs the main experiment logic:
 * 1. Load graphs from OSM datasets
 * 2. Run CCH preprocessing on each graph
 * 3. Perform customization tests
 * 4. Run query experiments comparing Dijkstra with CCH-Dijkstra
 * 5. Generate statistics and write results to file
 * 
 * @param output_file Path for the output file
 * @param osm_numbers List of OSM dataset numbers to run experiments on
 * @param paths Vector containing paths for KaHIP, graph file, and ordering file
 * @param os_type Operating system type for proper command execution
 */
void run_experiments(
    std::string output_file,
    std::vector<int> osm_numbers,
    const std::vector<std::string>& paths,
    CustomizableContractionHierarchy::OS os_type
) {
    // Extract paths from the vector
    std::string kahip_path = paths[0];
    std::string graph_file_path = paths[1];
    std::string ordering_file_path = paths[2];

    std::cout << "Starting Part 2 experiments (Customizable Contraction Hierarchies)" << std::endl;
    std::cout << "Using KaHIP at: " << kahip_path << std::endl;
    std::cout << "Graph file: " << graph_file_path << std::endl;
    std::cout << "Ordering file: " << ordering_file_path << std::endl;
    
    srand(static_cast<unsigned int>(time(nullptr)));
    auto start = NOW(), end = NOW();
    
    // Phase 1: Load graphs from OSM datasets
    std::cout << "Loading graphs from OSM datasets..." << std::endl;
    std::vector<Graph> graphs = get_graphs_from_osm_numbers(osm_numbers);
    
    // Create output directory if it doesn't exist
    std::filesystem::create_directories("results/part_2");
    
    // Open output file for writing results
    std::ofstream file(output_file);
    if (!file.is_open()) {
        std::cout << "Failed to open file: " << output_file << std::endl;
        exit(1);
    }
    
    file << "Running the CCH preprocessing algorithm on road networks:" << std::endl;
    file << "=======================================================" << std::endl << std::endl;
    
    // Collect results for comparison table
    std::vector<std::tuple<int, double, size_t, double, size_t, double, double>> results;
    
    // Phase 2: Run CCH preprocessing on all road networks
    for (size_t i = 0; i < graphs.size(); i++) {
        Graph &curr_graph = graphs[i];
        size_t num_of_original_edges = curr_graph.get_num_of_edges();
        
        file << "Running CCH on osm" << osm_numbers[i] << ":" << std::endl;
        file << "Node count: " << curr_graph.node_count() << ", Edge count: " << num_of_original_edges << std::endl;
        
        // Run CCH preprocessing with the provided paths
        std::cout << "Running CCH preprocessing on osm" << osm_numbers[i] << "..." << std::endl;
        start = NOW();
        CCH cch(
            curr_graph,
            kahip_path,
            graph_file_path,
            ordering_file_path,
            os_type
        );
        cch.preprocess(); // This includes initial customization
        end = NOW();
        
        // Calculate and report preprocessing statistics
        auto duration = TO_MILI(end - start);
        auto total_ms = duration.count();
        auto preprocessing_ms = cch.get_preprocessing_time().count();
        auto customization_ms = cch.get_customization_time().count() / 1000.0; // Convert μs to ms
        
        size_t num_of_new_edges = cch.get_num_of_edges();
        size_t num_of_added_edges = cch.get_num_of_shortcuts();
        double avg_triangles = cch.get_avg_triangles_per_edge();
        size_t max_triangles = cch.get_max_triangles_per_edge();
        
        file << "Number of edges before CCH: " << num_of_original_edges 
             << ", after CCH: " << num_of_new_edges << "." << std::endl;
        file << "Number of shortcuts added by CCH: " << num_of_added_edges << "." << std::endl;
        
        if (num_of_original_edges == 0) {
            file << "No edges in the graph, cannot calculate percentage of edges added." << std::endl;
        } else {
            double percentage = 100.0 * static_cast<double>(num_of_added_edges) / num_of_original_edges;
            file << "Percentage of edges added: " << percentage << "%." << std::endl;
        }
        
        file << "Average number of triangles per edge: " << avg_triangles << std::endl;
        file << "Maximum number of triangles for any edge: " << max_triangles << std::endl;
        
        auto seconds = (preprocessing_ms % 60000) / 1000;
        auto minutes = preprocessing_ms / 60000;
        auto milliseconds = preprocessing_ms % 1000;
        
        file << "Time taken for CCH preprocessing on osm" << osm_numbers[i] << ": "
             << minutes << " minutes " << seconds << " seconds and " << milliseconds << " ms." << std::endl;
        file << "Initial customization time: " << customization_ms << " ms." << std::endl;
        
        // Store results for comparison
        results.emplace_back(osm_numbers[i], preprocessing_ms, num_of_added_edges, 
                            avg_triangles, max_triangles, customization_ms, 0.0);
        
        file << "--------------------------------------------------" << std::endl;
        
        // Phase 3: Run multiple customizations with random weights
        std::cout << "Running customization tests on osm" << osm_numbers[i] << "..." << std::endl;
        file << "Running 5 customizations with random weights:" << std::endl;
        
        double total_customization_time = 0.0;
        
        // Generate list of all edges for random weight assignment
        std::vector<std::pair<int, int>> edges;
        for (int u = 0; u < curr_graph.node_count(); ++u) {
            for (const auto& e : curr_graph.neighbors(u)) {
                if (u < e.to) { // Avoid duplicates
                    edges.emplace_back(u, e.to);
                }
            }
        }
        
        for (int run = 0; run < 5; ++run) {
            // Run customization with random weights
            start = NOW();
            cch.customize(edges);
            end = NOW();
            
            auto customization_duration = TO_MICRO(end - start);
            double custom_ms = customization_duration.count() / 1000.0;
            total_customization_time += custom_ms;
            
            file << "  Run " << (run + 1) << ": " << custom_ms << " ms" << std::endl;
        }
        
        double avg_customization_time = total_customization_time / 5.0;
        file << "Average customization time with random weights: " << avg_customization_time << " ms." << std::endl;
        
        // Update the average customization time in results
        std::get<6>(results.back()) = avg_customization_time;
        
        file << "--------------------------------------------------" << std::endl;
        
        // Phase 4: Run query experiments
        std::cout << "Running query experiments on osm" << osm_numbers[i] << "..." << std::endl;
        file << "Running query experiments with " << osm_numbers[i] << ":" << std::endl;
        
        // Generate random source-target pairs for testing
        int num_of_pairs = 100;
        std::vector<std::pair<int, int>> pairs(num_of_pairs);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, curr_graph.node_count() - 1);
        
        for (int j = 0; j < num_of_pairs; j++) {
            int first = dis(gen);
            int second = dis(gen);
            pairs[j] = {first, second};
        }
        
        // Run Dijkstra and CCH-Dijkstra on each source-target pair
        size_t dijkstra_runtime = 0, cch_dijkstra_runtime = 0;
        int dijkstra_nodes_popped = 0, cch_dijkstra_nodes_popped = 0;
        
        file << "Running Dijkstra and CCH-Dijkstra on osm" << osm_numbers[i] 
             << " on " << pairs.size() << " pairs." << std::endl;
        
        int mismatches = 0;
        
        for (int j = 0; j < pairs.size(); j++) {
            auto [start_node, end_node] = pairs[j];
            
            // Run standard Dijkstra
            start = NOW();
            auto DijkstraResult = Dijkstra::run(curr_graph, start_node, end_node);
            end = NOW();
            dijkstra_runtime += TO_MICRO(end - start).count();
            dijkstra_nodes_popped += DijkstraResult.nodes_popped;
            
            // Run CCH-Dijkstra
            start = NOW();
            auto CCH_DijkstraResult = cch.query(start_node, end_node);
            end = NOW();
            cch_dijkstra_runtime += TO_MICRO(end - start).count();
            
            // Check if the weights are equal (with tolerance for floating-point errors)
            double dijkstra_dist = (DijkstraResult.dist.empty() || end_node >= DijkstraResult.dist.size()) 
                                 ? std::numeric_limits<double>::infinity() 
                                 : DijkstraResult.dist[end_node];
            double cch_dist = (CCH_DijkstraResult.dist.empty() || end_node >= CCH_DijkstraResult.dist.size()) 
                            ? std::numeric_limits<double>::infinity() 
                            : CCH_DijkstraResult.dist[end_node];
            
            // Allow for small floating-point differences
            constexpr double EPSILON = 1e-6;
            if (std::abs(dijkstra_dist - cch_dist) > EPSILON) {
                mismatches++;
                file << "  Warning: Distance mismatch for pair (" << start_node << ", " << end_node 
                     << "): Dijkstra=" << dijkstra_dist << ", CCH=" << cch_dist << std::endl;
            }
        }
        
        // Report path verification results
        if (mismatches == 0) {
            file << "All path distances match between Dijkstra and CCH-Dijkstra." << std::endl;
        } else {
            file << "Found " << mismatches << " path distance mismatches between Dijkstra and CCH-Dijkstra." << std::endl;
        }
        
        // Calculate and report average performance metrics
        double avg_dijkstra_time = static_cast<double>(dijkstra_runtime) / num_of_pairs;
        double avg_cch_time = static_cast<double>(cch_dijkstra_runtime) / num_of_pairs;
        
        file << "Average running time of Dijkstra on osm" << osm_numbers[i] << ": " 
             << avg_dijkstra_time / 1000.0 << " ms." << std::endl;
        file << "Average running time of CCH-Dijkstra on osm" << osm_numbers[i] << ": " 
             << avg_cch_time / 1000.0 << " ms." << std::endl;
        file << "Speedup of CCH-Dijkstra over Dijkstra: " 
             << (avg_dijkstra_time > 0 ? avg_dijkstra_time / avg_cch_time : 0) << "x" << std::endl;
        
        file << "**************************************************" << std::endl;
    }
    
    // Phase 5: Generate summary table
    std::cout << "Generating summary table..." << std::endl;
    file << "\nSummary table for CCH:\n";
    file << "osm\tPreprocessTime(ms)\tShortcuts\tAvgTriangles\tMaxTriangles\tInitCustomTime(ms)\tAvgCustomTime(ms)\n";
    for (const auto& entry : results) {
        auto [osm, preproc_time, shortcuts, avg_triangles, max_triangles, init_custom_time, avg_custom_time] = entry;
        file << osm << '\t'
             << preproc_time << '\t'
             << shortcuts << '\t'
             << avg_triangles << '\t'
             << max_triangles << '\t'
             << init_custom_time << '\t'
             << avg_custom_time << '\n';
    }
    
    file.close();
    std::cout << "Part 2 experiments completed. Results written to " << output_file << std::endl;
}

/**
 * @brief Get the path to the executable directory
 * 
 * This utility function attempts to find the directory where the program
 * is running, which helps locate files relative to the executable.
 * 
 * @return std::string Path to the executable directory
 */
std::string getExecutablePath() {
    std::filesystem::path executablePath;
    
    // Try to get the executable path
    try {
        executablePath = std::filesystem::current_path();
    } catch (const std::filesystem::filesystem_error&) {
        // Fallback to current directory if we can't get executable path
        executablePath = ".";
    }
    
    return executablePath.string();
}

/**
 * @brief Runs CCH experiments with default or WSL-based configuration
 * 
 * This implementation:
 * 1. Sets up the appropriate paths based on OS detection or user preference
 * 2. Calls the helper function to run the actual experiments
 * 
 * @param output_file Path for the output file where results will be written
 * @param osm_numbers List of OSM dataset numbers to run experiments on
 * @param use_default_paths Whether to use default paths with local executable
 */
void part_2_experiments(std::string output_file, std::vector<int> osm_numbers, bool use_default_paths) {
    if (use_default_paths) {
        // Get the executable directory
        std::string basePath = getExecutablePath();
        
        // Create temporary directories if they don't exist
        std::filesystem::create_directories(basePath + "/temp");
        
        // Setup paths based on detected OS
        std::vector<std::string> default_paths;
        CustomizableContractionHierarchy::OS os_type;
        
        if (IS_WINDOWS) {
            // Windows paths
            default_paths = {
                basePath + "/node_ordering/node_ordering.exe", // Local Windows executable
                basePath + "/temp/graph_for_kahip.txt",        // Graph file path
                basePath + "/temp/node_ordering.txt"           // Ordering file path
            };
            os_type = CustomizableContractionHierarchy::OS::WINDOWS;
            std::cout << "Detected Windows OS, using Windows paths" << std::endl;
        } else {
            // Linux paths
            default_paths = {
                basePath + "/node_ordering/node_ordering",     // Local Linux executable
                basePath + "/temp/graph_for_kahip.txt",        // Graph file path
                basePath + "/temp/node_ordering.txt"           // Ordering file path
            };
            os_type = CustomizableContractionHierarchy::OS::LINUX;
            std::cout << "Detected Linux OS, using Linux paths" << std::endl;
        }
        
        // Use the helper function with default paths and detected OS
        run_experiments(output_file, osm_numbers, default_paths, os_type);
    } else {
        // Fall back to the old WSL-based approach
        std::vector<std::string> paths = {
            "~/KaHIP/deploy/node_ordering_cmd",    // KaHIP path in WSL
            "C:\\temp\\graph_for_kahip.txt",       // Windows path for graph file
            "C:\\temp\\node_ordering.txt"          // Windows path for ordering file
        };
        
        // Use the helper function with WSL paths and Windows OS
        run_experiments(output_file, osm_numbers, paths, CustomizableContractionHierarchy::OS::WINDOWS);
    }
}

/**
 * @brief Runs CCH experiments with custom paths and OS configuration
 * 
 * This implementation:
 * 1. Validates the provided paths
 * 2. Calls the helper function to run the actual experiments
 * 
 * @param output_file Path for the output file where results will be written
 * @param osm_numbers List of OSM dataset numbers to run experiments on
 * @param paths Vector containing paths for KaHIP, graph file, and ordering file
 * @param os_type Operating system type for proper command execution
 */
void part_2_experiments(
    std::string output_file,
    std::vector<int> osm_numbers,
    const std::vector<std::string>& paths,
    CustomizableContractionHierarchy::OS os_type
) {
    // Check if paths vector has the required elements
    if (paths.size() < 3) {
        std::cerr << "Error: paths vector must contain at least 3 elements (KaHIP path, graph file path, ordering file path)" << std::endl;
        return;
    }
    
    // Call the helper function with the provided parameters
    run_experiments(output_file, osm_numbers, paths, os_type);
}

/**
 * @brief Parse command line arguments for CCH experiments
 * 
 * This helper function parses command line arguments to configure the experiment.
 * 
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @param output_file Reference to store the output file path
 * @param osm_numbers Reference to store the OSM numbers
 * @param paths Reference to store the paths vector
 * @param os_type Reference to store the OS type
 * @return bool True if custom paths should be used, false for default paths
 */
bool parseArgs(
    int argc, 
    char* argv[], 
    std::string& output_file,
    std::vector<int>& osm_numbers,
    std::vector<std::string>& paths,
    CustomizableContractionHierarchy::OS& os_type
) {
    // Check for minimum required arguments
    if (argc < 2) {
        return false;
    }
    
    // Check for --default flag
    if (argc == 2 && std::string(argv[1]) == "--default") {
        return false; // Will use default paths
    }
    
    // Check for minimum required arguments for custom paths
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " [--default] or " << argv[0] << " <kahip_path> <graph_file> <ordering_file> [linux|windows] [output_file] [osm_numbers...]" << std::endl;
        exit(1);
    }
    
    // Extract paths
    paths.push_back(argv[1]); // KaHIP path
    paths.push_back(argv[2]); // Graph file path
    paths.push_back(argv[3]); // Ordering file path
    
    // OS type (optional)
    if (argc > 4) {
        std::string os_arg = argv[4];
        if (os_arg == "linux" || os_arg == "Linux" || os_arg == "LINUX") {
            os_type = CustomizableContractionHierarchy::OS::LINUX;
        }
    }
    
    // Output file (optional)
    if (argc > 5) {
        output_file = argv[5];
    }
    
    // OSM numbers (optional)
    if (argc > 6) {
        osm_numbers.clear();
        for (int i = 6; i < argc; i++) {
            try {
                int osm = std::stoi(argv[i]);
                osm_numbers.push_back(osm);
            } catch (const std::exception& e) {
                std::cerr << "Warning: Invalid OSM number '" << argv[i] << "', skipping." << std::endl;
            }
        }
    }
    
    return true;
}

/**
 * @brief Get experiment configuration from user input
 * 
 * This helper function prompts the user for input to configure the experiment.
 * 
 * @param output_file Reference to store the output file path
 * @param osm_numbers Reference to store the OSM numbers
 * @param paths Reference to store the paths vector
 * @param os_type Reference to store the OS type
 */
void getUserInput(
    std::string& output_file,
    std::vector<int>& osm_numbers,
    std::vector<std::string>& paths,
    CustomizableContractionHierarchy::OS& os_type
) {
    // First, ask if user wants to use default paths
    std::string use_default;
    std::cout << "Use default paths with local node_ordering executable? (y/n) [default: y]: ";
    std::getline(std::cin, use_default);
    
    if (use_default.empty() || use_default == "y" || use_default == "Y") {
        // User wants default paths, return empty paths to signal this
        return;
    }
    
    std::string kahip_path, graph_file_path, ordering_file_path, os_input;
    
    // Get KaHIP path
    std::cout << "Enter KaHIP node_ordering_cmd path (e.g., ~/KaHIP/deploy/node_ordering_cmd): ";
    std::getline(std::cin, kahip_path);
    
    // Get graph file path
    std::cout << "Enter graph file path (e.g., C:\\temp\\graph_for_kahip.txt): ";
    std::getline(std::cin, graph_file_path);
    
    // Get ordering file path
    std::cout << "Enter ordering file path (e.g., C:\\temp\\node_ordering.txt): ";
    std::getline(std::cin, ordering_file_path);
    
    // Get OS type
    std::cout << "Enter OS type (windows/linux) [default: windows]: ";
    std::getline(std::cin, os_input);
    if (os_input == "linux" || os_input == "Linux" || os_input == "LINUX") {
        os_type = CustomizableContractionHierarchy::OS::LINUX;
    }
    
    // Store paths
    paths.push_back(kahip_path);
    paths.push_back(graph_file_path);
    paths.push_back(ordering_file_path);
    
    // Optional: Ask if user wants to specify custom OSM numbers
    std::string custom_osm;
    std::cout << "Use default OSM numbers (1-5)? (y/n) [default: y]: ";
    std::getline(std::cin, custom_osm);
    
    if (custom_osm == "n" || custom_osm == "N") {
        std::cout << "Enter OSM numbers separated by spaces (e.g., 1 3 5): ";
        std::string osm_input;
        std::getline(std::cin, osm_input);
        
        // Parse OSM numbers
        std::stringstream ss(osm_input);
        int osm;
        osm_numbers.clear();
        while (ss >> osm) {
            osm_numbers.push_back(osm);
        }
        
        if (osm_numbers.empty()) {
            std::cout << "No valid OSM numbers entered. Using default (1-5)." << std::endl;
            osm_numbers = {1, 2, 3, 4, 5};
        }
    }
    
    // Optional: Ask if user wants to specify custom output file
    std::string custom_output;
    std::cout << "Use default output file? (y/n) [default: y]: ";
    std::getline(std::cin, custom_output);
    
    if (custom_output == "n" || custom_output == "N") {
        std::cout << "Enter output file path: ";
        std::getline(std::cin, output_file);
    }
}

/**
 * @brief Runs CCH experiments with command line arguments
 * 
 * This implementation:
 * 1. Parses command line arguments or prompts for user input
 * 2. Sets up the experiment configuration
 * 3. Calls the appropriate function to run the experiments
 * 
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 */
void part_2_experiments(int argc, char* argv[]) {
    // Default values
    std::string output_file = "results/part_2/experiment_2.txt";
    std::vector<int> osm_numbers = {1, 2, 3, 4, 5}; // Reduced default set
    std::vector<std::string> paths;
    CustomizableContractionHierarchy::OS os_type = CustomizableContractionHierarchy::OS::WINDOWS;
    
    // Try to parse command-line arguments
    bool use_custom_paths = parseArgs(argc, argv, output_file, osm_numbers, paths, os_type);
    
    if (!use_custom_paths) {
        // Either no arguments or --default flag was used
        if (argc >= 2 && std::string(argv[1]) == "--default") {
            std::cout << "Using default paths with local node_ordering executable" << std::endl;
            part_2_experiments(output_file, osm_numbers, true);
            return;
        }
        
        // No or insufficient command-line arguments, prompt user for input
        getUserInput(output_file, osm_numbers, paths, os_type);
        
        // If paths is still empty after getUserInput, use default paths
        if (paths.empty()) {
            std::cout << "Using default paths with local node_ordering executable" << std::endl;
            part_2_experiments(output_file, osm_numbers, true);
            return;
        }
    }
    
    // Validate paths
    if (paths.size() < 3 || paths[0].empty() || paths[1].empty() || paths[2].empty()) {
        std::cerr << "Error: Invalid paths provided. Exiting." << std::endl;
        return;
    }
    
    // Display configuration
    std::cout << "\nConfiguration:" << std::endl;
    std::cout << "KaHIP path: " << paths[0] << std::endl;
    std::cout << "Graph file path: " << paths[1] << std::endl;
    std::cout << "Ordering file path: " << paths[2] << std::endl;
    std::cout << "OS type: " << (os_type == CustomizableContractionHierarchy::OS::WINDOWS ? "Windows" : "Linux") << std::endl;
    std::cout << "Output file: " << output_file << std::endl;
    std::cout << "OSM numbers: ";
    for (int osm : osm_numbers) {
        std::cout << osm << " ";
    }
    std::cout << std::endl << std::endl;
    
    // Run the experiments
    run_experiments(output_file, osm_numbers, paths, os_type);
}