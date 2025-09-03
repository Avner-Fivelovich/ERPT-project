#include "data_parser.hpp"

/**
 * @brief Loads multiple graphs from OSM dataset numbers
 * 
 * For each OSM number i, loads the file "RoadNetworks/osmi.txt"
 * 
 * @param osm_numbers Vector of OSM dataset numbers to load
 * @return Vector of Graph objects, one for each OSM number
 */
std::vector<Graph> get_graphs_from_osm_numbers(const std::vector<int>& osm_numbers){
    std::vector<Graph> graphs(osm_numbers.size()); // Create a vector of Graphs based on the osm numbers
    for (size_t i = 0; i < osm_numbers.size(); i++)
    {
        std::string filename = "RoadNetworks/osm" + std::to_string(osm_numbers[i]) + ".txt"; // Construct the filename based on user input
        graphs[i] = get_data_from_file(filename); // Get the data from the file
    }
    return graphs; // Return the vector of graphs
}

/**
 * @brief Parse command line arguments to extract OSM dataset numbers
 * 
 * This function handles three cases:
 * 1. No arguments: Prompts user for input interactively
 * 2. One argument: Uses it as a single OSM number
 * 3. Multiple arguments: Processes each as a separate OSM number
 * 
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return Vector of OSM dataset numbers to process
 */
std::vector<int> get_osm_numbers(int argc, char *argv[]){
    std::vector<int> osm_numbers(argc-1); // Create a vector to hold osm numbers based on the number of arguments
    int osm_number = 0; // Initialize osm_number to 0

    if (argc == 1) { // If no arguments are provided, initate interactive mode
        std::cout << "No arguments provided. choose the osm number(1 - 11):" << std::endl;
        std::cin >> osm_number;
        osm_numbers.assign(1, osm_number); // Assign the input to osm_numbers
        if (std::cin.fail() || osm_numbers[0] < 1 || osm_numbers[0] > 11) {
            std::cout << "Invalid input. Exiting program." << std::endl;
            exit(1); // Exit if the input is invalid
        }
    }
    else if (argc == 2) { // If one argument is provided, use it as the osm number
        try {
            // Attempt to convert command line argument to integer
            osm_number = std::stoi(argv[1]);
            // Validate the range
            if (osm_number < 1 || osm_number > 11) {
                std::cout << "Invalid osm number. Must be between 1 and 11. Exiting program." << std::endl;
                exit(1); // Exit if the input is invalid
            }
            // Assign the valid input to osm_numbers
            osm_numbers.assign(1, osm_number);
        } catch (const std::exception& e) {
            // Catch any standard exceptions
            std::cout << "Unexpected error occurred: " << e.what() << ". Exiting program." << std::endl;
            exit(1);
        }
    } 
    else { // If more than one argument is provided, then run on all of them
        for (size_t i = 1; i < argc; i++)
        {
            try {
                // Attempt to convert command line argument to integer
                osm_number = std::stoi(argv[i]);
                // Validate the range
                if (osm_number < 1 || osm_number > 11) {
                    std::cout << "Invalid osm number. Must be between 1 and 11. Exiting program." << std::endl;
                    exit(1); // Exit if the input is invalid
                }
                // Assign the valid input to osm_numbers
                osm_numbers[i - 1] = osm_number; // Store the osm number in the vector
            } catch (const std::exception& e) {
                // Catch any standard exceptions
                std::cout << "Unexpected error occurred on arg " << i << ": " << e.what() << ". Exiting program." << std::endl;
                exit(1);
            }
        }
    }
    return osm_numbers; // Return the vector of osm numbers
}

/**
 * @brief Load a graph from a file
 * 
 * Reads a graph from a text file in the specified format:
 * - First line: number of nodes, number of edges
 * - Next n lines: node_id, latitude, longitude for each node
 * - Next m lines: source_id, target_id for each edge
 * 
 * @param filename Path to the file containing graph data
 * @return Graph object constructed from the file data
 */
Graph get_data_from_file(const std::string& filename){
    int num_of_nodes, num_of_edges, node_id, neighbor_id;
    double node_longitude, node_latitude; 
    double default_weight = 1.0; // Default weight for each edge

    std::ifstream file(filename); // Open the file
    if (!file.is_open()) { // Check if the file is opened successfully
        std::cout << "Failed to open file: " << filename << std::endl;
        exit(1);
    }
    file >> num_of_nodes >> num_of_edges; // Read the number of nodes and edges from the file
    Graph graph(num_of_nodes); // Create a Graph object with the number of nodes

    // Phase 1: Read node data and initialize each node
    for (size_t i = 0; i < num_of_nodes; i++)
    {
        file >> node_id >> node_latitude >> node_longitude; // Read the node details
        if (file.fail()) { // Check if reading from the file failed
            std::cout << "Error reading node data from file: " << filename << std::endl;
            exit(1);
        }
        graph.init_node(node_id, node_latitude, node_longitude);
    }
    // Phase 2: Read edge data and establish bidirectional connections
    for (size_t i = 0; i < num_of_edges; i++)
    {
        file >> node_id >> neighbor_id; // Read the edge details
        if (file.fail()) { // Check if reading from the file failed
            std::cout << "Error reading node data from file: " << filename << std::endl;
            exit(1);
        }
        if (node_id > num_of_nodes ){
            std::cout << "Node ID " << node_id << " exceeds the number of nodes in the file: " << filename << std::endl;
            exit(1); // Exit if the node ID exceeds the number of nodes
        }
        if (neighbor_id > num_of_nodes){
            std::cout << "Node ID " << neighbor_id << " exceeds the number of nodes in the file: " << filename << std::endl;
            exit(1); // Exit if the node ID exceeds the number of nodes
        }
        graph.add_edge(node_id, neighbor_id, default_weight); // Add the edge to the graph
    }
    file.close(); // Close the file after reading
    return graph; // Return the populated graph
}

/**
 * @brief Convert a vector of nodes to a string representation
 * 
 * Creates a string representation of a node vector in the format "[id1, id2, ...]"
 * 
 * @param vec Vector of nodes to convert
 * @return String representation of the node vector
 */
std::string get_vector_as_string(const std::vector<Node>& vec) {
    std::string result = "[";
    for (size_t i = 0; i < vec.size(); i++) {
        result += std::to_string(vec[i].id);
        if (i < vec.size() - 1) {
            result += ", ";
        }
    }
    result += "]";
    return result; // Return the string representation of the vector
}