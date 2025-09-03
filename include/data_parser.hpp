#ifndef DATA_PARSER_HPP
#define DATA_PARSER_HPP

#include "graph.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

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
Graph get_data_from_file(const std::string& filename);

/**
 * @brief Parse command line arguments to extract OSM dataset numbers
 * 
 * Processes command line arguments to determine which OSM datasets to use.
 * If no arguments are provided, prompts the user for input.
 * 
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return Vector of OSM dataset numbers to process
 */
std::vector<int> get_osm_numbers(int argc, char* argv[]);

/**
 * @brief Convert a vector of nodes to a string representation
 * 
 * Creates a string representation of a node vector in the format "[id1, id2, ...]"
 * 
 * @param vec Vector of nodes to convert
 * @return String representation of the node vector
 */
std::string get_vector_as_string(const std::vector<Node>& vec);

/**
 * @brief Load multiple graphs from OSM dataset numbers
 * 
 * Loads multiple graphs based on their OSM dataset numbers.
 * For each number i, loads the file "RoadNetworks/osmi.txt"
 * 
 * @param osm_numbers Vector of OSM dataset numbers to load
 * @return Vector of Graph objects, one for each OSM number
 */
std::vector<Graph> get_graphs_from_osm_numbers(const std::vector<int>& osm_numbers);

#endif // DATA_PARSER_HPP