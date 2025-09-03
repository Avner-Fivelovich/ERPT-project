#ifndef EXPERIMENT2_HPP
#define EXPERIMENT2_HPP
#include "customizable_contraction_hierarchy.hpp"
#include "data_parser.hpp"
#include "dijkstra.hpp"
#include <chrono>
#include <random>
#include <cstdlib>

/**
 * @brief Runs experiments for Customizable Contraction Hierarchies
 * 
 * This function performs a series of experiments to evaluate the performance of 
 * Customizable Contraction Hierarchies (CCH) compared to standard Dijkstra's algorithm:
 * 1. Runs CCH preprocessing on multiple road networks
 * 2. Performs initial customization and measures its time
 * 3. Runs multiple customizations with random weights to evaluate customization performance
 * 4. Compares query performance between Dijkstra and CCH-Dijkstra
 * 5. Generates comprehensive statistics and performance metrics
 * 
 * @param output_file Path for the output file where results will be written
 * @param osm_numbers List of OSM dataset numbers to run experiments on
 * @param use_default_paths Whether to use default paths for the node ordering executable
 */
void part_2_experiments(
    std::string output_file = "results/part_2/experiment_2.txt", 
    std::vector<int> osm_numbers = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
    bool use_default_paths = true
);

/**
 * @brief Runs CCH experiments with custom paths and OS configuration
 * 
 * This overloaded version allows specifying custom paths for:
 * - The KaHIP node ordering executable
 * - The graph file for KaHIP
 * - The node ordering output file
 * 
 * It also allows specifying the operating system for proper command execution.
 * 
 * @param output_file Path for the output file where results will be written
 * @param osm_numbers List of OSM dataset numbers to run experiments on
 * @param paths Vector of paths: [kahip_path, graph_file_path, ordering_file_path]
 * @param os_type Operating system type (WINDOWS or LINUX)
 */
void part_2_experiments(
    std::string output_file,
    std::vector<int> osm_numbers,
    const std::vector<std::string>& paths,
    CustomizableContractionHierarchy::OS os_type
);

/**
 * @brief Runs CCH experiments with command line arguments
 * 
 * This version parses command line arguments to configure the experiment,
 * prompting the user for input if necessary arguments are missing.
 * 
 * Usage: program [--default] or program <kahip_path> <graph_file> <ordering_file> [linux|windows] [output_file] [osm_numbers...]
 * 
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 */
void part_2_experiments(int argc, char* argv[]);

#endif // EXPERIMENT2_HPP